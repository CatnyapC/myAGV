"""One durable map checkpoint; ROS launch exports are rebuilt from it on startup."""
import hashlib
import json
import math
import os
from pathlib import Path

from web_backend.storage import atomic_bytes, atomic_json


def map_digest(grid):
    raw = {key: grid[key] for key in ('frame', 'width', 'height', 'resolution_m', 'origin')}
    raw['cells'] = grid.get('raw_cells', grid['cells'])
    return hashlib.sha256(json.dumps(raw, sort_keys=True, allow_nan=False).encode()).hexdigest()


def validate_saved_map(grid):
    if not isinstance(grid, dict):
        raise ValueError('Invalid saved map')
    try:
        width, height = grid['width'], grid['height']
        if (grid['frame'] != 'map' or type(width) is not int or type(height) is not int
                or width <= 0 or height <= 0 or width * height > 2_000_000
                or not isinstance(grid['map_id'], str) or not grid['map_id']
                or type(grid['revision']) is not int or grid['revision'] <= 0):
            raise ValueError('Invalid saved map identity or dimensions')
        values = [grid['resolution_m']] + [grid['origin'][key] for key in ('x_m', 'y_m', 'yaw_rad')]
        if not all(type(v) in (int, float) and math.isfinite(v) for v in values) or values[0] <= 0:
            raise ValueError('Invalid saved map geometry')
        for cells in (grid['cells'], grid.get('raw_cells', grid['cells'])):
            if (not isinstance(cells, list) or len(cells) != width * height
                    or any(type(c) is not int or c not in (-1, 0, 100) for c in cells)):
                raise ValueError('Invalid saved map cells')
        digest = map_digest(grid)
        if grid.get('content_hash', digest) != digest:
            raise ValueError('Saved map checksum mismatch')
    except (KeyError, TypeError, OverflowError) as exc:
        raise ValueError('Invalid saved map') from exc
    return grid


def load_saved_map(directory):
    path = Path(directory).parent / 'saved_map.json'
    if not path.exists():
        path = Path(directory) / 'map.json'  # Adopt existing bridge snapshots once.
        if not path.exists():
            return None
        grid = json.loads(path.read_text())
        if isinstance(grid, dict) and type(grid.get('revision')) is int and grid['revision'] == 0:
            return None
    else:
        grid = json.loads(path.read_text())
    return validate_saved_map(grid)


def save_map(directory, grid):
    validate_saved_map(grid)
    atomic_json(Path(directory).parent / 'saved_map.json', {**grid, 'content_hash': map_digest(grid)})


def map_matches(grid, saved):
    return (all(grid[key] == saved[key] for key in ('frame', 'width', 'height'))
            and grid['cells'] == saved.get('raw_cells', saved['cells'])
            and math.isclose(grid['resolution_m'], saved['resolution_m'], rel_tol=0, abs_tol=1e-8)
            and all(math.isclose(grid['origin'][key], saved['origin'][key], rel_tol=0, abs_tol=1e-6)
                    for key in ('x_m', 'y_m', 'yaw_rad')))


def prepare_localization(directory, fallback_map):
    directory = Path(directory)
    marker = directory.parent / 'mapping.json'
    saved = load_saved_map(directory)
    if saved:
        if marker.exists():
            session = json.loads(marker.read_text())['session_id']
            if not saved['map_id'].startswith('slam-' + session + '-'):
                raise ValueError('Saved map belongs to another mapping session')
        save_map(directory, saved)
        # Export raw occupancy, never the no-go mask or dynamic costmap.
        cells = saved.get('raw_cells', saved['cells'])
        width, height = saved['width'], saved['height']
        pixels = bytes({-1: 205, 0: 254, 100: 0}[cells[y * width + x]]
                       for y in range(height - 1, -1, -1) for x in range(width))
        image = directory.parent / 'saved_map.pgm'
        yaml = image.with_suffix('.yaml')
        atomic_bytes(image, ('P5\n%d %d\n255\n' % (width, height)).encode() + pixels)
        origin = saved['origin']
        text = ('image: saved_map.pgm\nresolution: %s\norigin: [%s, %s, %s]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n') % (
                    saved['resolution_m'], origin['x_m'], origin['y_m'], origin['yaw_rad'])
        atomic_bytes(yaml, text.encode())
        marker.unlink(missing_ok=True)
        return str(yaml.resolve()), False
    return str(fallback_map), marker.exists()


def main():
    root = Path(__file__).resolve().parent
    directory = Path(os.environ.get('MYAGV_HARDWARE_DIR', str(root / 'web_runtime' / 'ros')))
    fallback = os.environ.get('MYAGV_MAP_FILE', str(Path.home() / 'maps' / 'room.yaml'))
    map_file, mapping = prepare_localization(directory, fallback)
    os.execvp('roslaunch', ['roslaunch', '--wait', str(root / 'web_hardware.launch'),
                           'map_file:=' + map_file, 'mapping:=' + str(mapping).lower()])


if __name__ == '__main__':
    main()
