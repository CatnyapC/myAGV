"""Reference photos and explicit map observations. No model or camera claims."""
from contextlib import suppress
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import shutil
import struct
import time
import uuid
import zlib

from navigation import load_stations
from .map_data import png_rgb
from .simulation import validate_pose
from .storage import atomic_bytes, atomic_json

MAX_IMAGE_BYTES = 4 * 1024 * 1024


def validate_png(data):
    if len(data) > MAX_IMAGE_BYTES or data[:8] != b'\x89PNG\r\n\x1a\n':
        raise ValueError('A PNG image up to 4 MiB is required')
    offset, compressed, size, ended = 8, bytearray(), None, False
    while offset + 12 <= len(data):
        length = struct.unpack('!I', data[offset:offset+4])[0]
        kind, payload = data[offset+4:offset+8], data[offset+8:offset+8+length]
        end = offset + 12 + length
        if end > len(data) or zlib.crc32(kind + payload) != struct.unpack('!I', data[end-4:end])[0]:
            raise ValueError('Invalid PNG chunk')
        if kind == b'IHDR':
            if size or offset != 8 or length != 13:
                raise ValueError('Invalid PNG header')
            w, h, depth, color, compression, filtering, interlace = struct.unpack('!2I5B', payload)
            if not (0 < w*h <= 4_000_000 and w > 0 and h > 0) or depth != 8 or color not in (2, 6) or compression or filtering or interlace:
                raise ValueError('Use a non-interlaced 8-bit RGB/RGBA PNG up to 4 megapixels')
            size = (w, h, 3 if color == 2 else 4)
        elif kind == b'IDAT':
            compressed.extend(payload)
        elif kind == b'IEND':
            if length or end != len(data):
                raise ValueError('Invalid PNG ending')
            ended = True
            break
        elif kind[:1].isupper() and kind != b'PLTE':
            raise ValueError('Unsupported PNG chunk')
        offset = end
    if not size or not ended:
        raise ValueError('Incomplete PNG image')
    width, height, channels = size
    expected = (width*channels + 1)*height
    try:
        decoder = zlib.decompressobj()
        raw = decoder.decompress(compressed, expected + 1)
        if len(raw) != expected or not decoder.eof or decoder.unused_data or any(raw[i] > 4 for i in range(0, len(raw), width*channels+1)):
            raise ValueError('Invalid PNG pixels')
    except zlib.error as exc:
        raise ValueError('Invalid PNG pixels') from exc


def demo_frame(camera_id):
    """Synthetic red-cup fixture, never displayed as a live camera stream."""
    width, height, rows = 320, 240, bytearray()
    for y in range(height):
        rows.append(0)
        for x in range(width):
            cup = ((x-160)**2 + (y-120)**2 < 42**2) if camera_id == 'arm' else (125 < x < 190 and 85 < y < 165)
            handle = 188 < x < 211 and 107 < y < 141
            rim = (125 < x < 190 and 82 < y < 91) if camera_id == 'front' else (35**2 < (x-160)**2 + (y-120)**2 < 42**2)
            rows.extend(b'\xe5\x48\x4d' if cup or handle else b'\x34\x34\x38' if y < 175 else b'\x46\x46\x4a')
            if rim:
                rows[-3:] = b'\xff\x9c\x99'
    return png_rgb(width, height, rows)


def digest_station(station):
    return hashlib.sha256(json.dumps(station, sort_keys=True, allow_nan=False).encode()).hexdigest()


class PhotoIndex:
    def __init__(self, directory, stations_path):
        self.directory, self.stations_path = Path(directory), Path(stations_path)
        path = self.directory / 'photo_index.json'
        self.value = json.loads(path.read_text()) if path.exists() else dict(version=1, revision=0, photos=[], items=[])
        self.validate(self.value)

    def validate(self, value):
        if value.get('version') != 1 or type(value.get('revision')) is not int or value['revision'] < 0 or not isinstance(value.get('photos'), list) or not isinstance(value.get('items'), list):
            raise ValueError('Invalid photo index')
        if len(value['photos']) > 1000 or len(value['items']) > 500:
            # ponytail: bounded JSON index; use SQLite when surveys exceed 1000 frames.
            raise ValueError('Photo index limit reached')
        ids = set()
        for entry in [*value['photos'], *value['items']]:
            uuid.UUID(entry['id'])
            if entry['id'] in ids:
                raise ValueError('Duplicate photo/item ID')
            ids.add(entry['id'])
        item_ids = {i['id'] for i in value['items']}
        for item in value['items']:
            if not isinstance(item.get('name'), str) or not 1 <= len(item['name'].strip()) <= 100 or not isinstance(item.get('appearance'), str) or len(item['appearance']) > 1000:
                raise ValueError('Invalid item features')
            if 'station_link' in item:
                link = item['station_link']
                if not all(isinstance(link.get(k), str) and link[k] for k in ('name', 'map_id', 'digest')):
                    raise ValueError('Invalid station link')
        for photo in value['photos']:
            if 'current' in photo and type(photo['current']) is not bool:
                raise ValueError('Invalid observation freshness')
            if 'update_key' in photo and (photo.get('source') != 'ros' or not isinstance(photo['update_key'], str) or not 1 <= len(photo['update_key']) <= 100):
                raise ValueError('Invalid map update observation key')
            if type(photo.get('saved_at_s')) not in (int, float) or not math.isfinite(photo['saved_at_s']) or photo['saved_at_s'] <= 0:
                raise ValueError('Invalid photo timestamp')
            if photo['kind'] == 'reference':
                if photo.get('source') != 'phone' or any(k in photo for k in ('base_pose', 'map_id', 'captured_at_s', 'frame')):
                    raise ValueError('Reference photos cannot claim map observations')
            elif photo['kind'] == 'observation':
                if photo.get('source') not in ('simulation', 'ros') or photo.get('camera_id') not in ('front', 'arm') or not all(isinstance(photo.get(k), str) and photo[k] for k in ('map_id', 'frame')):
                    raise ValueError('Invalid observation source')
                validate_pose(photo['base_pose'])
                if type(photo.get('captured_at_s')) not in (int, float) or not math.isfinite(photo['captured_at_s']) or photo['captured_at_s'] <= 0 or type(photo.get('map_revision')) is not int or photo['map_revision'] < 0:
                    raise ValueError('Invalid acquisition metadata')
            else:
                raise ValueError('Unknown photo kind')
            if photo.get('item_id') and photo['item_id'] not in item_ids:
                raise ValueError('Photo refers to a missing item')
        by_id = {p['id']: p for p in value['photos']}
        for item in value['items']:
            goal = item.get('grasp_goal')
            if goal is not None:
                if not isinstance(goal, dict) or set(goal) != {'front', 'arm'}:
                    raise ValueError('Invalid grasp goal')
                for camera, photo_id in goal.items():
                    photo = by_id.get(photo_id, {})
                    if photo.get('kind') != 'observation' or photo.get('source') != 'ros' or photo.get('camera_id') != camera or photo.get('item_id') != item['id']:
                        raise ValueError('Invalid grasp goal camera association')

    def stations(self):
        return load_stations(self.stations_path)

    def snapshot(self, map_id):
        value = deepcopy(self.value)
        try:
            stations = self.stations()
        except (ValueError, OSError, TypeError):
            stations = None
        for photo in value['photos']:
            photo['image_url'] = f"/api/photos/{photo['id']}/image"
            photo['available'] = self.image_path(photo['id']).is_file()
            photo['map_matches'] = photo['kind'] == 'reference' or photo['map_id'] == map_id
        for item in value['items']:
            link = item.get('station_link')
            observations = [p for p in value['photos'] if p.get('item_id') == item['id'] and p['kind'] == 'observation' and p['map_id'] == map_id and p['available']]
            item['observation_current'] = any(p.get('current', True) for p in observations)
            item['last_seen_s'] = max((p['captured_at_s'] for p in observations), default=0)
            item['station_status'] = 'none' if not link else 'unavailable' if stations is None else 'map_mismatch' if link['map_id'] != map_id else 'ready' if link['name'] in stations and digest_station(stations[link['name']]) == link['digest'] else 'stale'
            item['fetch_available'] = False  # Task/grasp gates are not part of the photo milestone.
            goal = item.get('grasp_goal')
            item['grasp_goal_ready'] = bool(goal) and all(self.image_path(photo_id).is_file() and self.image_path(photo_id).with_suffix('.jpg').is_file() for photo_id in goal.values())
        return value

    def image_path(self, photo_id):
        return self.directory / 'images' / f'{str(uuid.UUID(photo_id))}.png'

    def check(self, revision):
        if type(revision) is not int or revision != self.value['revision']:
            raise ValueError('Photo index changed; reload before editing')

    def commit(self, value):
        by_id = {p['id']: p for p in value['photos']}
        for item in value['items']:
            if item.get('grasp_goal') and any(by_id.get(photo_id, {}).get('item_id') != item['id']
                                             for photo_id in item['grasp_goal'].values()):
                item.pop('grasp_goal')
        value['revision'] = self.value['revision'] + 1
        self.validate(value)
        atomic_json(self.directory / 'photo_index.json', value)
        self.value = value

    def add(self, image, metadata, revision):
        self.check(revision)
        validate_png(image)
        self.directory.mkdir(parents=True, exist_ok=True)
        if shutil.disk_usage(self.directory).free < len(image) + 32*1024*1024:
            raise ValueError('Insufficient image storage')
        photo = dict(id=str(uuid.uuid4()), saved_at_s=time.time(), **metadata)
        value = deepcopy(self.value)
        replaced = []
        if photo.get('update_key'):
            for old in list(value['photos']):
                if old.get('update_key') == photo['update_key'] and old.get('map_id') == photo['map_id'] and old.get('camera_id') == photo['camera_id']:
                    if old.get('item_id'):
                        old['current'] = False  # A new image cannot inherit identity of a moving object.
                    else:
                        value['photos'].remove(old)
                        replaced.append(old)
        value['photos'].append(photo)
        self.validate(value)
        path = self.image_path(photo['id'])
        atomic_bytes(path, image)
        try:
            self.commit(value)
        except BaseException:
            with suppress(OSError):
                path.unlink()
            raise
        for old in replaced:
            with suppress(OSError):
                self.image_path(old['id']).unlink()
        return photo

    def edit(self, photo_id, data, map_id):
        if not isinstance(data, dict) or any(not isinstance(data.get(k, ''), str) for k in ('item_id', 'name', 'appearance', 'station_name')):
            raise ValueError('Invalid photo label fields')
        self.check(data['expected_revision'])
        value = deepcopy(self.value)
        photo = next((p for p in value['photos'] if p['id'] == photo_id), None)
        if photo is None:
            raise ValueError('Photo not found')
        if photo['kind'] == 'observation' and photo['map_id'] != map_id:
            raise ValueError('Observation belongs to another map')
        item_id = data.get('item_id', '')
        if item_id == 'new':
            item = dict(id=str(uuid.uuid4()), name=data['name'].strip(), appearance=data.get('appearance', '').strip())
            value['items'].append(item)
        elif item_id:
            item = next((i for i in value['items'] if i['id'] == item_id), None)
            if item is None:
                raise ValueError('Item not found')
        else:
            item = None
        if item:
            if data.get('confirmed') is not True:
                raise ValueError('Explicit item association confirmation required')
            photo['item_id'] = item['id']
            photo['association_source'] = 'manual'
            if 'name' in data:
                item.update(name=data['name'].strip(), appearance=data.get('appearance', '').strip())
            station_name = data.get('station_name', '')
            if station_name:
                station = self.stations().get(station_name)
                if station is None:
                    raise ValueError('Station not found')
                link = dict(name=station_name, map_id=map_id, digest=digest_station(station))
                if item.get('station_link') != link and data.get('station_confirmed') is not True:
                    raise ValueError('Explicit station confirmation required')
                item['station_link'] = link
            else:
                item.pop('station_link', None)
        else:
            photo.pop('item_id', None)
            photo.pop('association_source', None)
        used = {p.get('item_id') for p in value['photos']}
        value['items'] = [i for i in value['items'] if i['id'] in used]
        self.commit(value)

    def save_grasp_goal(self, item_id, cameras, revision, map_id):
        self.check(revision)
        if not isinstance(cameras, dict) or set(cameras) != {'front', 'arm'}:
            raise ValueError('Both grasp goal cameras required')
        value = deepcopy(self.value)
        item = next((i for i in value['items'] if i['id'] == item_id), None)
        if item is None:
            raise ValueError('Save the item before recording its grasp goal')
        for camera, photo_id in cameras.items():
            photo = next((p for p in value['photos'] if p['id'] == photo_id), {})
            if photo.get('kind') != 'observation' or photo.get('source') != 'ros' or photo.get('camera_id') != camera or photo.get('map_id') != map_id:
                raise ValueError('Measured current-map grasp goal required')
            if not self.image_path(photo_id).is_file() or not self.image_path(photo_id).with_suffix('.jpg').is_file():
                raise ValueError('Grasp goal images unavailable')
            photo.update(item_id=item_id, association_source='manual')
        item['grasp_goal'] = dict(cameras)
        self.commit(value)

    def delete(self, photo_id, revision):
        self.check(revision)
        value = deepcopy(self.value)
        value['photos'] = [p for p in value['photos'] if p['id'] != photo_id]
        if len(value['photos']) == len(self.value['photos']):
            raise ValueError('Photo not found')
        used = {p.get('item_id') for p in value['photos']}
        value['items'] = [i for i in value['items'] if i['id'] in used]
        self.commit(value)
        # ponytail: best-effort orphan cleanup; index gates access, add a sweeper for long surveys.
        with suppress(OSError):
            self.image_path(photo_id).unlink()
        with suppress(OSError):
            self.image_path(photo_id).with_suffix('.jpg').unlink()
