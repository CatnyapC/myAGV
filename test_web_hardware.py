"""Hardware availability guards and expiration; no ROS or physical movement."""
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import AsyncMock, Mock

from aiohttp.test_utils import TestClient, TestServer
from web_backend.hardware import Hardware
from web_backend.map_data import demo_map
from web_backend.server import EDITOR, create_app
from web_backend.storage import atomic_bytes, atomic_json


class HardwareTest(unittest.IsolatedAsyncioTestCase):
    async def test_battery_endpoint_rejects_stale_or_invalid_feedback(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bridge = root / 'ros'
            atomic_json(bridge / 'map.json', {**demo_map(), 'map_id': 'ros-test'})
            async with TestClient(TestServer(create_app(root / 'ui', hardware_dir=bridge))) as client:
                hardware = client.server.app[EDITOR].hardware
                now = time.time()
                battery = dict(voltage_v=12.3, stamp_s=now)
                hardware.snapshot = dict(stamp_s=now, battery=battery)
                response = await client.get('/api/battery')
                self.assertEqual(await response.json(), battery)
                self.assertEqual(response.headers['Cache-Control'], 'no-store')
                for invalid in (None, [], {}, dict(voltage_v=12.3, stamp_s=now - 6),
                                dict(voltage_v=12.3, stamp_s=now + 10),
                                *(dict(voltage_v=v, stamp_s=now) for v in (0, -1, 26, True, '12', float('nan'), float('inf')))):
                    hardware.snapshot = dict(stamp_s=now, battery=invalid)
                    self.assertIsNone(await (await client.get('/api/battery')).json())
                hardware.snapshot = dict(stamp_s=now - 3, battery=battery)
                self.assertIsNone(await (await client.get('/api/battery')).json())
            async with TestClient(TestServer(create_app(root / 'demo'))) as client:
                self.assertIsNone(await (await client.get('/api/battery')).json())

    async def test_stop_recovery_waits_for_feedback_and_reports_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bridge = root / 'ros'
            grid = demo_map()
            atomic_json(bridge / 'map.json', grid)
            app = create_app(root / 'ui', hardware_dir=bridge)
            hardware = app[EDITOR].hardware
            hardware.refresh = Mock()
            hardware.control = dict(stamp_s=time.time(), boot_id='boot', phase='fault')
            hardware.sessions['tab'] = dict(ack=time.monotonic())
            data = dict(id='recovery', type='recover_stop', confirmed=True, session_id='tab',
                        map_id=grid['map_id'], expected_revision=app[EDITOR].zones['revision'])
            async with TestClient(TestServer(app)) as client:
                for result, status in ((dict(status='completed'), 200),
                                       (dict(status='failed', error='P340 USB input/output error'), 400),
                                       (dict(status='cancelled', error='Recovery stopped'), 400)):
                    with self.subTest(result=result):
                        hardware.sessions['tab']['ack'] = time.monotonic()
                        hardware.control['stamp_s'] = time.time()
                        hardware.rpc = AsyncMock(side_effect=[dict(status='accepted'), dict(status='pending'), result])
                        response = await client.post('/api/commands', json=data)
                        self.assertEqual(response.status, status)
                        self.assertEqual(await response.json(), result if status == 200 else dict(error=result['error']))
                        self.assertEqual(hardware.rpc.await_count, 3)
                        self.assertEqual(hardware.rpc.call_args.args[0], dict(op='result', id=data['id']))
                hardware.rpc = AsyncMock(return_value=dict(status='stopping'))

    async def test_real_source_stale_feedback_and_motion_guards(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bridge = root / 'ros'
            grid = {**demo_map(), 'map_id': 'ros-test'}
            pose = dict(x_m=1, y_m=2, yaw_rad=.3)
            snapshot = dict(stamp_s=time.time(), pose_stamp_s=time.time(), pose=pose,
                            frame=grid['frame'], map_id=grid['map_id'])
            atomic_json(bridge / 'map.json', grid)
            atomic_json(bridge / 'state.json', snapshot)
            atomic_bytes(bridge / 'front.jpg', b'jpeg-fixture')
            atomic_json(bridge / 'front.json', dict(stamp_s=time.time()))
            reader = Hardware(bridge)
            reader.refresh()
            self.assertEqual(reader.telemetry(grid)['pose'], pose)
            self.assertIn('front', reader.cameras)
            for invalid in ({'stamp_s': time.time() - 10}, {'pose_stamp_s': time.time() - 10},
                            {'map_id': 'other-map'}, {'pose': {'x_m': float('nan'), 'y_m': 0, 'yaw_rad': 0}}):
                reader.snapshot = {**snapshot, **invalid}
                self.assertIsNone(reader.telemetry(grid)['pose'])
            atomic_json(bridge / 'front.json', dict(stamp_s=time.time() - 10))
            reader.refresh()
            self.assertNotIn('front', reader.cameras)

            async with TestClient(TestServer(create_app(root / 'ui', hardware_dir=bridge))) as client:
                state = await (await client.get('/api/state')).json()
                self.assertFalse(state['demo'])
                self.assertFalse(state['read_only'])
                self.assertFalse(state['motion_available'])
                self.assertFalse(state['simulation_available'])
                self.assertEqual(state['navigation']['source'], 'ros')
                self.assertEqual((await client.get('/api/map')).status, 200)
                for command in ('navigate', 'demo_reset', 'zone_add', 'zone_delete', 'fetch', 'home'):
                    self.assertEqual((await client.post('/api/commands', json={'type': command})).status, 400)
                self.assertEqual((await client.post('/api/photos/capture', json={})).status, 409)
                self.assertEqual((await client.get('/api/cameras/front.jpg')).status, 503)
                self.assertEqual((await client.get('/api/global-costmap')).status, 503)
                for invalid in ([], dict(passive='true')):
                    self.assertEqual((await client.post('/api/stop', json=invalid)).status, 400)
                response = await client.post('/api/stop', json={})
                self.assertEqual(response.status, 503)
                self.assertEqual((await response.json())['error'], 'Hardware controller unavailable; stop unconfirmed')


if __name__ == '__main__':
    unittest.main()
