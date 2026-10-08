"""Loopback WebUI API with demo simulation or read-only ROS snapshots."""
import argparse
import asyncio
from contextlib import suppress
import json
import math
import os
from pathlib import Path
import uuid
from urllib.parse import urlsplit

from aiohttp import web
from .map_data import demo_map, dominant_angle, render_map, validate_rectangle
from .simulation import Simulation, plan_path, validate_pose
from .photos import MAX_IMAGE_BYTES, PhotoIndex, demo_frame
from .storage import atomic_json
from .hardware import Hardware
from .resolve import MODEL, MODEL_OPTIONS, describe_photo, load_key, resolve_items, target_preview, validate_llm

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SETTINGS = dict(auto_align=True, manual_angle_deg=0, llm_model=MODEL, reasoning_effort='off')
DISPLAY_FIELDS = ('auto_align', 'manual_angle_deg')


def load_json(path, fallback):
    return json.loads(path.read_text()) if path.exists() else fallback


def validate_settings(value):
    if not isinstance(value, dict) or set(value) != set(DEFAULT_SETTINGS):
        raise ValueError('Invalid settings fields')
    angle = value['manual_angle_deg']
    if type(value['auto_align']) is not bool or isinstance(angle, bool) or not isinstance(angle, (int, float)) or not math.isfinite(angle) or not -180 <= angle <= 180:
        raise ValueError('Invalid display settings')
    validate_llm(value['llm_model'], value['reasoning_effort'])
    return value


class Editor:
    def __init__(self, directory, hardware_dir=None):
        self.directory = Path(directory)
        self.hardware = Hardware(hardware_dir) if hardware_dir else None
        if self.hardware:
            self.hardware.refresh()
            if not self.hardware.grid:
                raise ValueError('Waiting for ROS bridge map snapshot')
        self.grid = self.hardware.grid if self.hardware else demo_map()
        self.settings = load_json(self.directory / 'settings.json', dict(revision=0, values=DEFAULT_SETTINGS.copy()))
        if isinstance(self.settings['values'], dict) and set(self.settings['values']) == set(DISPLAY_FIELDS):
            self.settings['values'] = {**DEFAULT_SETTINGS, **self.settings['values']}
        validate_settings(self.settings['values'])
        self.zones = dict(map_id=self.grid['map_id'], revision=0, zones=[])
        if self.hardware and self.hardware.zones:
            self.zones = self.hardware.zones
        if not self.hardware:
            self.zones = load_json(self.directory / 'no_go_zones.json', self.zones)
        if self.zones['map_id'] != self.grid['map_id']:
            raise ValueError('Saved zones belong to another map; explicit review required')
        for zone in self.zones['zones']:
            validate_rectangle(zone['corners'], self.grid)
        self.detected = dominant_angle(self.grid)
        self.metadata, self.png = render_map(self.grid, self.settings['values'], self.settings['revision'], self.detected)
        self.sockets = set()
        self.lock = asyncio.Lock()
        self.replies = {}  # Bounded retry cache; duplicate command IDs never add another zone.
        self.sim = None if self.hardware else Simulation(self.grid, self.zones)
        self.cost_png = None
        self.llm = dict(status='not_configured')

    def state(self):
        if self.hardware:
            navigation = self.hardware.telemetry(self.grid)
            return dict(robot_id='myagv-ros', demo=False, read_only=False, phase=navigation['phase'],
                        hardware=self.hardware.control, motion_available=self.hardware.control.get('motion_available', False),
                        simulation_available=False, cameras_available=bool(self.hardware.cameras),
                        cameras=self.hardware.cameras, localization_available=navigation['pose'] is not None,
                        zone_enforcement='ros-global-local' if navigation['costmap']['ready'] else 'pending', navigation=navigation,
                        llm={**self.llm, 'model': self.settings['values']['llm_model'],
                             'reasoning_effort': self.settings['values']['reasoning_effort']})
        return dict(robot_id='local-demo', demo=True, phase=self.sim.phase, motion_available=False,
                    simulation_available=True, cameras_available=False, localization_available=False,
                    zone_enforcement='simulated-global',
                    navigation=self.sim.telemetry(), llm={**self.llm, 'model': self.settings['values']['llm_model'],
                                                        'reasoning_effort': self.settings['values']['reasoning_effort']})

    @property
    def generation(self):
        return self.hardware.generation if self.hardware else self.sim.generation

    async def emit(self, kind):
        for socket in tuple(self.sockets):
            if not socket.closed:
                try:
                    message = dict(type=kind)
                    if kind == 'telemetry':
                        message['state'] = self.state()
                    await asyncio.wait_for(socket.send_json(message), .2)
                except (ConnectionError, asyncio.TimeoutError):
                    self.sockets.discard(socket)
                    if self.hardware:
                        with suppress(RuntimeError, ValueError):
                            await self.hardware.stop('Telemetry unavailable')
                    elif not self.sockets:
                        self.sim.stop('Telemetry unavailable')
                    with suppress(ConnectionError, asyncio.TimeoutError):
                        await asyncio.wait_for(socket.close(code=1001, drain=False), .2)


EDITOR = web.AppKey('editor', Editor)


@web.middleware
async def boundary(request, handler):
    lan_origin = os.environ.get('MYAGV_WEB_ORIGIN', '')
    if request.url.host not in ('localhost', '127.0.0.1', '::1') and f'http://{request.host}' != lan_origin:
        raise web.HTTPForbidden(text='Local demo only')
    origin = request.headers.get('Origin')
    if origin:
        parsed = urlsplit(origin)
        allowed = {request.host, '127.0.0.1:5173', 'localhost:5173'}
        if parsed.scheme != 'http' or parsed.netloc not in allowed:
            raise web.HTTPForbidden(text='Origin rejected')
    try:
        response = await handler(request)
    except ValueError as exc:
        response = web.json_response(dict(error=str(exc)), status=400)
    except (TypeError, KeyError):
        response = web.json_response(dict(error='Invalid request fields'), status=400)
    except RuntimeError as exc:
        response = web.json_response(dict(error=str(exc)), status=503)
    except OSError:
        response = web.json_response(dict(error='Storage operation failed; changes were not saved'), status=500)
    except web.HTTPException as exc:
        response = web.json_response(dict(error=exc.text), status=exc.status)
    response.headers['Cache-Control'] = 'no-store' if request.path.startswith('/api/') else 'no-cache'
    response.headers['X-Content-Type-Options'] = 'nosniff'
    return response


def create_app(directory=ROOT / 'web_runtime', stations_path=ROOT / 'stations.json', hardware_dir=None):
    app = web.Application(middlewares=[boundary], client_max_size=MAX_IMAGE_BYTES + 1024)
    editor = Editor(directory, hardware_dir)
    app[EDITOR] = editor
    photos = PhotoIndex(directory, stations_path)
    photo_lock = asyncio.Lock()
    editor.llm['status'], llm_key = load_key()
    resolve_lock = asyncio.Lock()  # ponytail: one request; per-user locks if multi-user control is added.

    def photo_stopped():
        if (editor.sim and editor.sim.phase != 'idle') or (editor.hardware and editor.hardware.control.get('phase') != 'idle'):
            raise web.HTTPConflict(text='Photo editing requires a stopped robot')

    async def photo_index(request):
        return web.json_response(await asyncio.to_thread(photos.snapshot, editor.grid['map_id']))

    async def resolve(request):
        data = await request.json()
        if not isinstance(data, dict) or not isinstance(data.get('text'), str) or not 1 <= len(data['text'].strip()) <= 500 or type(data.get('expected_revision')) is not int:
            raise ValueError('Use a request of 1–500 characters and an index revision')
        photo_stopped()
        if data.get('map_id') != editor.grid['map_id'] or data['expected_revision'] != photos.value['revision']:
            raise web.HTTPConflict(text='Map or photo index changed; retry explicitly')
        if not llm_key:
            raise web.HTTPServiceUnavailable(text='OpenRouter key not configured or unavailable')
        if resolve_lock.locked():
            raise web.HTTPTooManyRequests(text='An item resolution is already running')
        async with resolve_lock:
            generation, zone_revision, settings_revision = editor.generation, editor.zones['revision'], editor.settings['revision']
            config = dict(editor.settings['values'])
            def check_current():
                if editor.generation != generation or (editor.sim and editor.sim.phase != 'idle') or editor.zones['revision'] != zone_revision or editor.settings['revision'] != settings_revision or data['map_id'] != editor.grid['map_id'] or data['expected_revision'] != photos.value['revision']:
                    raise web.HTTPConflict(text='Resolution cancelled or records changed; retry explicitly')
            index = await asyncio.to_thread(photos.snapshot, editor.grid['map_id'])
            check_current()
            result = await resolve_items(data['text'].strip(), index['items'], llm_key, config['llm_model'], config['reasoning_effort'])
            check_current()
            if result['status'] == 'matched':
                index = await asyncio.to_thread(photos.snapshot, editor.grid['map_id'])
                try:
                    records = await asyncio.to_thread(photos.stations)
                except (ValueError, OSError, TypeError):
                    records = {}
            # Storage/STOP can change while collecting the final station snapshot.
            check_current()
            if result['status'] == 'matched':
                result.update(target_preview(index, records, result['item_ids'][0]))
            return web.json_response({**result, 'index_revision': data['expected_revision'], 'map_id': data['map_id'], 'settings_revision': settings_revision})

    async def stations(request):
        try:
            return web.json_response(await asyncio.to_thread(photos.stations))
        except (ValueError, OSError, TypeError):
            return web.json_response({})

    async def reference_upload(request):
        photo_stopped()
        if request.content_type != 'image/png':
            raise ValueError('Upload a normalized PNG reference image')
        image = await request.read()
        async with photo_lock:
            photo_stopped()
            added = await asyncio.to_thread(photos.add, image, dict(kind='reference', source='phone'), int(request.query['expected_revision']))
        await editor.emit('photos')
        return web.json_response({**await asyncio.to_thread(photos.snapshot, editor.grid['map_id']), 'added_id': added['id']})

    async def ingest_capture(capture_id):
        capture_id = str(uuid.UUID(capture_id))
        existing = next((p for p in photos.value['photos'] if p.get('capture_id') == capture_id), None)
        path = editor.hardware.directory / 'captures' / capture_id
        if existing:
            added = existing
        else:
            metadata = json.loads(path.with_suffix('.json').read_text())
            if metadata.get('capture_id') != capture_id or metadata.get('source') != 'ros':
                raise ValueError('Invalid hardware acquisition')
            added = await asyncio.to_thread(photos.add, path.with_suffix('.png').read_bytes(), metadata, photos.value['revision'])
        path.with_suffix('.json').unlink(missing_ok=True)
        path.with_suffix('.png').unlink(missing_ok=True)
        return added

    async def capture_photo(request):
        data = await request.json()
        async with photo_lock:
            photo_stopped()
            if data['map_id'] != editor.grid['map_id'] or data['camera_id'] not in ('front', 'arm'):
                raise ValueError('Invalid map or camera for Demo capture')
            photos.check(data['expected_revision'])
            if editor.hardware:
                command = {**data, 'id': data.get('id', str(uuid.uuid4())), 'type': 'capture',
                           'expected_revision': data['zone_revision']}
                result = await editor.hardware.command(command, wait=True)
                added = await ingest_capture(result['capture_id'])
                await editor.emit('photos')
                return web.json_response({**photos.snapshot(editor.grid['map_id']), 'added_id': added['id']})
            nav = editor.sim.telemetry()  # Freeze acquisition metadata before any await.
            metadata = dict(kind='observation', source='simulation', captured_at_s=nav['stamp_s'],
                            base_pose=nav['pose'], camera_id=data['camera_id'], map_id=nav['map_id'],
                            map_revision=editor.grid['revision'], frame=nav['frame'])
            image = await asyncio.to_thread(demo_frame, data['camera_id'])
            added = await asyncio.to_thread(photos.add, image, metadata, data['expected_revision'])
        await editor.emit('photos')
        return web.json_response({**await asyncio.to_thread(photos.snapshot, editor.grid['map_id']), 'added_id': added['id']})

    async def photo_edit(request):
        data = await request.json()
        async with photo_lock:
            photo_stopped()
            if request.method == 'DELETE':
                await asyncio.to_thread(photos.delete, request.match_info['id'], data['expected_revision'])
            else:
                await asyncio.to_thread(photos.edit, request.match_info['id'], data, editor.grid['map_id'])
        await editor.emit('photos')
        return web.json_response(await asyncio.to_thread(photos.snapshot, editor.grid['map_id']))

    async def photo_describe(request):
        data = await request.json()
        photo_stopped()
        photos.check(data['expected_revision'])
        photo_id = request.match_info['id']
        if not any(p['id'] == photo_id for p in photos.value['photos']):
            raise web.HTTPNotFound(text='Photo not found')
        path = photos.image_path(photo_id)
        if not path.is_file():
            raise web.HTTPNotFound(text='Stored image unavailable')
        if not llm_key:
            raise web.HTTPServiceUnavailable(text='OpenRouter key not configured or unavailable')
        if resolve_lock.locked():
            raise web.HTTPTooManyRequests(text='An LLM request is already running')
        async with resolve_lock:
            generation, settings_revision = editor.generation, editor.settings['revision']
            image = await asyncio.to_thread(path.read_bytes)
            result = await describe_photo(image, llm_key, editor.settings['values']['reasoning_effort'])
            photo_stopped()
            photos.check(data['expected_revision'])
            if generation != editor.generation or settings_revision != editor.settings['revision']:
                raise web.HTTPConflict(text='Description cancelled or settings changed; retry explicitly')
            return web.json_response({**result, 'model': MODEL})

    async def photo_image(request):
        photo_id = request.match_info['id']
        if not any(p['id'] == photo_id for p in photos.value['photos']):
            raise web.HTTPNotFound(text='Photo not found')
        path = photos.image_path(photo_id)
        if not path.is_file():
            raise web.HTTPNotFound(text='Stored image unavailable')
        return web.FileResponse(path, headers={'Content-Type': 'image/png'})

    async def state(request):
        return web.json_response(editor.state())

    async def map_info(request):
        return web.json_response(editor.metadata)

    async def map_image(request):
        if request.query.get('view_revision') != str(editor.metadata['display']['view_revision']):
            raise web.HTTPConflict(text='Display revision changed; reload map')
        return web.Response(body=editor.png, content_type='image/png')

    async def zones(request):
        return web.json_response(editor.zones)

    async def costmap(request):
        if editor.hardware:
            return web.json_response({**editor.hardware.costmap(request.path == '/api/navigation-map'), 'source': 'ros'})
        grid = editor.sim.navigation if request.path == '/api/navigation-map' else editor.sim.costmap
        return web.json_response({**grid, 'zone_revision': editor.sim.revision, 'source': 'simulation'})

    async def costmap_image(request):
        if editor.hardware:
            grid = editor.hardware.costmap()
            view, zone = editor.metadata['display']['view_revision'], grid['zone_revision']
            if request.query.get('view_revision') != str(view) or request.query.get('zone_revision') != str(zone):
                raise web.HTTPConflict(text='Costmap revision changed; reload')
            # MapView expects the same world extents as the map raster.
            if any(grid[k] != editor.grid[k] for k in ('width', 'height', 'resolution_m', 'origin')):
                raise RuntimeError('Global costmap geometry differs from map')
            grid = {**grid, 'cells': [-1 if c < 0 else 0 if c == 0 else 100 for c in grid['cells']]}
            _, png = await asyncio.to_thread(render_map, grid, editor.settings['values'], view, editor.detected,
                {-1: b'\x00\x00\x00\x00', 0: b'\x00\x00\x00\x00', 100: b'\xe5\x48\x4d\x60'})
            return web.Response(body=png, content_type='image/png', headers={'Cache-Control': 'no-store'})
        view, zone = editor.metadata['display']['view_revision'], editor.sim.revision
        if request.query.get('view_revision') != str(view) or request.query.get('zone_revision') != str(zone):
            raise web.HTTPConflict(text='Costmap revision changed; reload')
        if not editor.cost_png or editor.cost_png[0] != (view, zone):
            _, png = await asyncio.to_thread(render_map, editor.sim.costmap, editor.settings['values'], view, editor.detected,
                                             {-1: b'\x00\x00\x00\x00', 0: b'\x00\x00\x00\x00', 100: b'\xe5\x48\x4d\x60'})
            if (view, zone) != (editor.metadata['display']['view_revision'], editor.sim.revision):
                raise web.HTTPConflict(text='Costmap revision changed; reload')
            editor.cost_png = ((view, zone), png)
        return web.Response(body=editor.cost_png[1], content_type='image/png')

    async def settings(request):
        if request.method == 'GET':
            return web.json_response({**editor.settings, 'llm_models': MODEL_OPTIONS})
        data = await request.json()
        async with editor.lock:
            if data['expected_revision'] != editor.settings['revision']:
                raise web.HTTPConflict(text='Settings changed; reload before editing')
            values = validate_settings(data['values'])
            saved = dict(revision=editor.settings['revision'] + 1, values=values)
            metadata, png = editor.metadata, editor.png
            if any(values[k] != editor.settings['values'][k] for k in DISPLAY_FIELDS):
                metadata, png = await asyncio.to_thread(render_map, editor.grid, values, metadata['display']['view_revision'] + 1, editor.detected)
            atomic_json(editor.directory / 'settings.json', saved)
            editor.settings, editor.metadata, editor.png = saved, metadata, png
        await editor.emit('settings')
        return web.json_response({**saved, 'llm_models': MODEL_OPTIONS})

    async def commands(request):
        data = await request.json()
        if editor.hardware:
            if data['type'] == 'cancel':
                return web.json_response(await editor.hardware.stop('Cancelled'))
            async with editor.lock:
                if data['map_id'] != editor.grid['map_id'] or data['expected_revision'] != editor.zones['revision']:
                    raise ValueError('Map or no-go revision changed; reload')
                kind = data['type']
                if kind in ('zone_add', 'zone_delete'):
                    current = list(editor.zones['zones'])
                    if kind == 'zone_add':
                        current.append(dict(id=str(uuid.uuid5(uuid.UUID(data['id']), 'zone')), corners=validate_rectangle(data['corners'], editor.grid)))
                    else:
                        current = [z for z in current if z['id'] != data['zone_id']]
                        if len(current) == len(editor.zones['zones']):
                            raise ValueError('Zone not found')
                    data = {**data, 'type': 'zones', 'zones': current}
                if kind == 'fetch':
                    if data.get('index_revision') != photos.value['revision']:
                        raise ValueError('Photo index changed; reconfirm selection')
                    item = next((i for i in photos.snapshot(editor.grid['map_id'])['items'] if i['id'] == data.get('item_id')), None)
                    if not item or item['station_status'] != 'ready':
                        raise ValueError('Confirmed current-map station association required')
                    link = item['station_link']
                    data = {**data, 'station': link['name'], 'station_digest': link['digest']}
                result = await editor.hardware.command(data, wait=kind in ('zone_add', 'zone_delete', 'teach', 'transport_record'))
                if 'zones' in result:
                    editor.zones = result['zones']
            await editor.emit('zones' if 'zones' in result else 'state')
            return web.json_response(result)
        async with editor.lock:
            command_id = data['id']
            if not isinstance(command_id, str) or not 1 <= len(command_id) <= 100:
                raise ValueError('Invalid command ID')
            fingerprint = json.dumps(data, sort_keys=True, allow_nan=False)
            if command_id in editor.replies:
                old, reply = editor.replies[command_id]
                if old != fingerprint:
                    raise web.HTTPConflict(text='Command ID reused with different arguments')
                return web.json_response(reply)
            if data['map_id'] != editor.grid['map_id'] or data['expected_revision'] != editor.zones['revision']:
                raise web.HTTPConflict(text='Map or zones changed; reload before editing')
            kind = data['type']
            if kind == 'cancel':
                editor.sim.stop('Cancelled')
            elif kind in ('navigate', 'demo_reset'):
                if editor.sim.phase != 'idle':
                    raise web.HTTPConflict(text='Stop the simulated task first')
                if kind == 'demo_reset':
                    editor.sim.reset()
                else:
                    if data.get('frame') != editor.grid['frame']:
                        raise ValueError('Goal frame must match the map frame')
                    goal = validate_pose(data['goal'])
                    generation = editor.sim.generation
                    editor.sim.phase, editor.sim.status = 'planning', 'Planning'
                    try:
                        path = await asyncio.to_thread(plan_path, editor.sim.costmap, dict(editor.sim.pose), goal)
                        if generation != editor.sim.generation:
                            raise web.HTTPConflict(text='Navigation cancelled while planning')
                        editor.sim.start(goal, path)
                    except BaseException:
                        if generation == editor.sim.generation:
                            editor.sim.stop('Planning failed')
                        raise
            elif kind not in ('zone_add', 'zone_delete'):
                raise web.HTTPBadRequest(text='Unsupported Demo command')
            if kind in ('navigate', 'cancel', 'demo_reset'):
                reply = dict(id=command_id, status='running' if kind == 'navigate' else 'completed', navigation=editor.sim.telemetry())
                editor.replies[command_id] = fingerprint, reply
                if len(editor.replies) > 200:
                    del editor.replies[next(iter(editor.replies))]
                await editor.emit('telemetry')
                return web.json_response(reply)
            if editor.sim.phase != 'idle':
                raise web.HTTPConflict(text='No-go edits require a stopped simulator')
            current = list(editor.zones['zones'])
            if data['type'] == 'zone_add':
                if len(current) >= 100:
                    raise web.HTTPBadRequest(text='Zone limit reached')
                current.append(dict(id=str(uuid.uuid4()), corners=validate_rectangle(data['corners'], editor.grid)))
            elif data['type'] == 'zone_delete':
                current = [z for z in current if z['id'] != data['zone_id']]
                if len(current) == len(editor.zones['zones']):
                    raise web.HTTPNotFound(text='Zone not found')
            saved = dict(map_id=editor.grid['map_id'], revision=editor.zones['revision'] + 1, zones=current)
            navigation, global_grid = await asyncio.to_thread(editor.sim.rebuild, saved, False)
            atomic_json(editor.directory / 'no_go_zones.json', saved)
            editor.zones = saved
            editor.sim.navigation, editor.sim.costmap, editor.sim.revision = navigation, global_grid, saved['revision']
            editor.sim.goal = None
            reply = dict(id=command_id, status='completed', zones=saved, enforcement='simulated-global')
            editor.replies[command_id] = fingerprint, reply
            if len(editor.replies) > 200:
                del editor.replies[next(iter(editor.replies))]
        await editor.emit('zones')
        return web.json_response(reply)

    async def stop(request):
        if editor.hardware:
            return web.json_response(await editor.hardware.stop())
        editor.sim.stop('Stopped')
        await editor.emit('telemetry')
        return web.json_response(dict(status='completed', reason='Simulation stopped; hardware is not connected'))

    async def events(request):
        socket = web.WebSocketResponse(heartbeat=20, max_msg_size=8192)
        await socket.prepare(request)
        editor.sockets.add(socket)
        session = str(uuid.uuid4())
        await socket.send_json(dict(type='state'))

        async def challenge():
            while True:
                await socket.send_json(editor.hardware.challenge(session))
                await asyncio.sleep(.15)

        task = asyncio.create_task(challenge()) if editor.hardware else None
        try:
            async for message in socket:
                if editor.hardware and message.type == web.WSMsgType.TEXT:
                    await editor.hardware.heartbeat(session, json.loads(message.data))
        except (RuntimeError, ValueError, ConnectionError):
            pass
        finally:
            if task:
                task.cancel()
                with suppress(asyncio.CancelledError, ConnectionError):
                    await task
            editor.sockets.discard(socket)
            if editor.hardware:
                with suppress(RuntimeError, ValueError):
                    await editor.hardware.disconnect(session)
            elif not editor.sockets:
                editor.sim.stop('UI disconnected')
            await editor.emit('telemetry')
        return socket

    async def shutdown(application):
        if editor.hardware:
            with suppress(RuntimeError, ValueError):
                await editor.hardware.stop('API shutdown')
        for socket in tuple(editor.sockets):
            await socket.close(code=1001, message=b'Service stopping')

    async def simulation_tick(application):
        async def advance():
            while True:
                await asyncio.sleep(.1)
                if editor.hardware:
                    await asyncio.to_thread(editor.hardware.refresh)
                    async with editor.lock:
                        grid = editor.hardware.grid
                        changed = grid is not None and grid['map_id'] != editor.grid['map_id']
                        if changed:
                            detected = await asyncio.to_thread(dominant_angle, grid)
                            view = editor.metadata['display']['view_revision'] + 1
                            metadata, png = await asyncio.to_thread(render_map, grid, editor.settings['values'], view, detected)
                            editor.grid, editor.metadata, editor.png, editor.detected = grid, metadata, png, detected
                            editor.zones = dict(map_id=grid['map_id'], revision=0, zones=[])
                            editor.hardware.generation += 1
                    if editor.hardware.zones and editor.hardware.zones['map_id'] == editor.grid['map_id']:
                        editor.zones = editor.hardware.zones
                    captures = editor.hardware.directory / 'captures'
                    if not photo_lock.locked():
                        async with photo_lock:
                            for capture_path in sorted(captures.glob('*.json'))[:20]:
                                try:
                                    await ingest_capture(capture_path.stem)
                                    await editor.emit('photos')
                                except (ValueError, OSError, KeyError):
                                    pass  # Retain original capture for recovery when storage/index is repaired.
                    await editor.emit('map' if changed else 'telemetry')
                    continue
                async with editor.lock:
                    running = editor.sim.phase == 'running'
                    if running:
                        editor.sim.advance(.1)
                if running:
                    await editor.emit('telemetry')
        task = asyncio.create_task(advance())
        yield
        task.cancel()
        with suppress(asyncio.CancelledError):
            await task

    async def static(request):
        dist = ROOT / 'web' / 'dist'
        path = (dist / request.match_info['path']).resolve()
        if not path.is_relative_to(dist.resolve()):
            raise web.HTTPForbidden(text='Invalid asset path')
        if path.is_file():
            return web.FileResponse(path)
        if request.match_info['path'] == '' and (dist / 'index.html').is_file():
            return web.FileResponse(dist / 'index.html')
        raise web.HTTPNotFound(text='Build frontend first, or use the PM2 development frontend on port 5173')

    async def camera_image(request):
        camera_id = request.match_info['id']
        if not editor.hardware or camera_id not in editor.hardware.cameras:
            raise web.HTTPServiceUnavailable(text='Camera unavailable or stale')
        camera = editor.hardware.cameras[camera_id]
        if not editor.hardware.fresh(camera['stamp_s']):
            raise web.HTTPServiceUnavailable(text='Camera unavailable or stale')
        return web.FileResponse(editor.hardware.directory / f'{camera_id}.jpg', headers={'Cache-Control': 'no-store'})

    app.router.add_get('/api/state', state)
    app.router.add_get('/api/items', photo_index)
    app.router.add_post('/api/resolve', resolve)
    app.router.add_get('/api/stations', stations)
    app.router.add_post('/api/photos/reference', reference_upload)
    app.router.add_post('/api/photos/capture', capture_photo)
    app.router.add_patch('/api/photos/{id}', photo_edit)
    app.router.add_delete('/api/photos/{id}', photo_edit)
    app.router.add_get('/api/photos/{id}/image', photo_image)
    app.router.add_post('/api/photos/{id}/describe', photo_describe)
    app.router.add_get('/api/map', map_info)
    app.router.add_get('/api/map.png', map_image)
    app.router.add_get('/api/no-go-zones', zones)
    app.router.add_get('/api/navigation-map', costmap)
    app.router.add_get('/api/global-costmap', costmap)
    app.router.add_get('/api/global-costmap.png', costmap_image)
    app.router.add_get('/api/settings', settings)
    app.router.add_put('/api/settings', settings)
    app.router.add_post('/api/commands', commands)
    app.router.add_post('/api/stop', stop)
    app.router.add_get('/api/events', events)
    app.router.add_get('/api/cameras/{id}.jpg', camera_image)
    app.router.add_get('/{path:.*}', static)
    app.on_shutdown.append(shutdown)
    app.cleanup_ctx.append(simulation_tick)
    return app


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8791)
    parser.add_argument('--data-dir', type=Path, default=ROOT / 'web_runtime')
    parser.add_argument('--hardware-dir', type=Path, default=os.environ.get('MYAGV_HARDWARE_DIR'))
    args = parser.parse_args()
    web.run_app(create_app(args.data_dir, hardware_dir=args.hardware_dir), host='127.0.0.1', port=args.port)
