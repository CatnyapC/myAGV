"""Read-only, expiring snapshots from the separate ROS/system-Python process."""
import json
import math
from pathlib import Path
import time


class Hardware:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.grid = None
        self.map_mtime = None
        self.snapshot = {}
        self.cameras = {}
        self.generation = 0

    def refresh(self):
        try:
            path = self.directory / 'map.json'
            mtime = path.stat().st_mtime_ns
            if mtime != self.map_mtime:
                grid = json.loads(path.read_text())
                size = grid['width'] * grid['height']
                if not 0 < size <= 2_000_000 or len(grid['cells']) != size or any(c not in (-1, 0, 100) for c in grid['cells']):
                    raise ValueError('Invalid hardware occupancy grid')
                if not math.isfinite(grid['resolution_m']) or grid['resolution_m'] <= 0 or not all(math.isfinite(v) for v in grid['origin'].values()):
                    raise ValueError('Invalid hardware map geometry')
                self.grid, self.map_mtime = grid, mtime
            self.snapshot = json.loads((self.directory / 'state.json').read_text())
        except (OSError, ValueError, KeyError, TypeError):
            self.snapshot = {}
        self.cameras = {}
        for camera_id in ('front', 'arm'):
            try:
                camera = json.loads((self.directory / f'{camera_id}.json').read_text())
                if self.fresh(camera.get('stamp_s')) and (self.directory / f'{camera_id}.jpg').is_file():
                    self.cameras[camera_id] = dict(url=f'/api/cameras/{camera_id}.jpg?stamp={camera["stamp_s"]}',
                                                   stamp_s=camera['stamp_s'])
            except (OSError, ValueError, TypeError):
                pass

    @staticmethod
    def fresh(stamp):
        return isinstance(stamp, (int, float)) and 0 <= time.time() - stamp <= 2

    def telemetry(self, grid):
        fresh = self.fresh(self.snapshot.get('stamp_s'))
        pose = self.snapshot.get('pose') if fresh and self.fresh(self.snapshot.get('pose_stamp_s')) and self.snapshot.get('frame') == grid['frame'] and self.snapshot.get('map_id') == grid['map_id'] else None
        if pose is not None and (not isinstance(pose, dict) or set(pose) != {'x_m', 'y_m', 'yaw_rad'} or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in pose.values())):
            pose = None
        return dict(map_id=grid['map_id'], frame=grid['frame'], source='ros', stamp_s=time.time(),
                    pose=pose, goal=None, path=[], moving=False, phase='idle',
                    status='Read-only hardware' if pose else 'Localization unavailable or stale',
                    footprint=dict(length_m=.32, width_m=.28),
                    costmap=dict(ready=False, applied_zone_revision=0, clearance_m=0))
