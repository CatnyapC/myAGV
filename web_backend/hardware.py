"""Snapshots plus a short-lived, single-tab control lease to the ROS process."""
import asyncio
import json
import math
from pathlib import Path
import time
import uuid


class Hardware:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.grid = None
        self.map_mtime = None
        self.snapshot = {}
        self.control = {}
        self.cameras = {}
        self.zones = None
        self.generation = 0
        self.sessions = {}
        self.owner = None

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
        try:
            control = json.loads((self.directory / 'control.json').read_text())
            self.control = control if self.fresh(control.get('stamp_s'), 1) else {}
            self.zones = json.loads((self.directory / 'zones.json').read_text())
        except (OSError, ValueError, KeyError, TypeError):
            self.control = {}
        self.cameras = {}
        for camera_id in ('front', 'arm'):
            try:
                camera = json.loads((self.directory / f'{camera_id}.json').read_text())
                if self.fresh(camera.get('stamp_s')) and (self.directory / f'{camera_id}.jpg').is_file():
                    self.cameras[camera_id] = dict(url=f'/api/cameras/{camera_id}.jpg?stamp={camera["stamp_s"]}', stamp_s=camera['stamp_s'])
            except (OSError, ValueError, TypeError):
                pass

    @staticmethod
    def fresh(stamp, limit=2):
        return type(stamp) in (int, float) and 0 <= time.time() - stamp <= limit

    async def rpc(self, packet):
        async def exchange():
            reader, writer = await asyncio.open_unix_connection(str(self.directory / 'control.sock'))
            try:
                writer.write((json.dumps(packet, allow_nan=False) + '\n').encode())
                await writer.drain()
                result = json.loads(await reader.readline())
                if result.get('error') and 'status' not in result:
                    raise ValueError(result['error'])
                return result
            finally:
                writer.close()
                await writer.wait_closed()
        try:
            return await asyncio.wait_for(exchange(), .5)
        except (OSError, asyncio.TimeoutError, json.JSONDecodeError) as exc:
            raise RuntimeError('Hardware controller unavailable; stop unconfirmed') from exc

    def challenge(self, session):
        nonce = str(uuid.uuid4())
        previous = self.sessions.get(session, {})
        self.sessions[session] = dict(nonce=nonce, issued=time.monotonic(), ack=previous.get('ack', 0))
        return dict(type='lease', session_id=session, nonce=nonce)

    async def heartbeat(self, session, message):
        state = self.sessions.get(session)
        now = time.monotonic()
        if not state or message.get('nonce') != state.get('nonce') or not 0 <= now - state['issued'] <= .3:
            return False
        state['nonce'], state['ack'] = None, now
        if session == self.owner and self.control:
            await self.rpc(dict(op='heartbeat', boot_id=self.control['boot_id'], session_id=session,
                                deadline=now + .45, input=message.get('input')))
        return True

    async def command(self, data, wait=False):
        state = self.sessions.get(data.get('session_id'), {})
        if not self.control or not self.fresh(self.control.get('stamp_s'), 1):
            raise RuntimeError('Hardware controller unavailable')
        if not 0 <= time.monotonic() - state.get('ack', 0) <= .3:
            raise ValueError('Control heartbeat missing or stale; keep this page visible and retry')
        if self.control['phase'] not in ('idle', 'fault') and self.owner != data.get('session_id'):
            raise ValueError('Another tab owns the active task')
        # Claim before sending so the first heartbeat cannot miss an accepted command.
        old_owner = self.owner
        self.owner = data['session_id']
        packet = {**data, 'op': 'command', 'boot_id': self.control['boot_id'], 'deadline': time.monotonic() + .45}
        try:
            result = await self.rpc(packet)
        except Exception:
            self.owner = old_owner
            raise
        if wait and result.get('status') == 'accepted':
            end = time.monotonic() + 15
            while time.monotonic() < end:
                await asyncio.sleep(.1)
                result = await self.rpc(dict(op='result', id=data['id']))
                if result.get('status') != 'pending':
                    break
            else:
                await self.stop('Command result timeout')
                raise RuntimeError('Command result timeout; inspect robot status before retry')
        if result.get('status') in ('failed', 'cancelled'):
            raise ValueError(result.get('error', result['status']))
        return result

    async def stop(self, reason='Stopped', passive=False):
        self.generation += 1
        self.owner = None
        result = await self.rpc(dict(op='stop', reason=reason, passive=passive))
        return result if result.get('status') == 'ignored' else dict(status='stopping', reason='Waiting for measured base/arm stop')

    async def disconnect(self, session):
        self.sessions.pop(session, None)
        if self.owner == session:
            await self.stop('Controlling UI disconnected', passive=True)

    def costmap(self, navigation=False):
        name = 'navigation_map.json' if navigation else 'global_costmap.json'
        try:
            grid = json.loads((self.directory / name).read_text())
        except (OSError, ValueError) as exc:
            raise RuntimeError('ROS costmap unavailable') from exc
        if not self.grid or grid['map_id'] != self.grid['map_id'] or not self.control.get('zones_ready') or grid['zone_revision'] != self.control['zone_revision'] or (not navigation and not self.fresh(grid.get('stamp_s'))):
            raise RuntimeError('ROS costmap stale or revision pending')
        return grid

    def telemetry(self, grid):
        current = self.fresh(self.snapshot.get('stamp_s'))
        pose = self.snapshot.get('pose') if current and self.fresh(self.snapshot.get('pose_stamp_s')) and self.snapshot.get('frame') == grid['frame'] and self.snapshot.get('map_id') == grid['map_id'] else None
        if pose is not None and (not isinstance(pose, dict) or set(pose) != {'x_m', 'y_m', 'yaw_rad'} or not all(type(v) in (int, float) and math.isfinite(v) for v in pose.values())):
            pose = None
        control = self.control if self.fresh(self.control.get('stamp_s'), 1) else {}
        goal = control.get('goal')
        if goal:
            goal = dict(x_m=goal['x_m'], y_m=goal['y_m'], yaw_rad=math.radians(goal['yaw_deg']))
        radius = control.get('clearance_m', .25)
        return dict(map_id=grid['map_id'], frame=grid['frame'], source='ros', stamp_s=time.time(),
                    pose=pose, goal=goal, path=self.snapshot.get('path', []) if current else [],
                    moving=control.get('moving', False), phase=control.get('phase', 'unavailable'),
                    status=control.get('status', 'Hardware controller unavailable; stop unconfirmed'),
                    footprint=dict(length_m=2*radius, width_m=2*radius),
                    costmap=dict(ready=control.get('zones_ready', False), applied_zone_revision=control.get('zone_revision', -1), clearance_m=radius))
