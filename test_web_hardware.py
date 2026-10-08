"""Read-only hardware guards and expiration; no ROS or physical movement."""
import tempfile
import time
import unittest
from pathlib import Path

from aiohttp.test_utils import TestClient, TestServer
from web_backend.hardware import Hardware
from web_backend.map_data import demo_map
from web_backend.server import create_app
from web_backend.storage import atomic_bytes, atomic_json


class HardwareTest(unittest.IsolatedAsyncioTestCase):
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
                self.assertTrue(state['read_only'])
                self.assertFalse(state['motion_available'])
                self.assertFalse(state['simulation_available'])
                self.assertEqual(state['navigation']['source'], 'ros')
                self.assertEqual((await client.get('/api/map')).status, 200)
                for command in ('navigate', 'demo_reset', 'zone_add', 'zone_delete', 'fetch', 'home'):
                    self.assertEqual((await client.post('/api/commands', json={'type': command})).status, 403)
                self.assertEqual((await client.post('/api/photos/capture', json={})).status, 501)
                self.assertEqual((await client.get('/api/cameras/front.jpg')).status, 503)
                self.assertEqual((await client.get('/api/global-costmap')).status, 503)
                response = await client.post('/api/stop', json={})
                self.assertEqual(response.status, 503)
                self.assertEqual((await response.json())['status'], 'unconfirmed')


if __name__ == '__main__':
    unittest.main()
