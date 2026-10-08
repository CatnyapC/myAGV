"""Reachability, bounded update windows, pause ownership and refreshed observations."""
from copy import deepcopy
import asyncio
import math
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import AsyncMock, Mock, patch
import uuid

from aiohttp.test_utils import TestClient, TestServer

from robot_control import Stopped, UpdatePaused
from test_robot_control import fake_control, packet
from web_backend.map_data import demo_map
from web_backend.photos import PhotoIndex, demo_frame
from web_backend.resolve import target_preview
from web_backend.server import EDITOR, create_app
from web_backend.storage import atomic_json


class MapUpdateTest(unittest.TestCase):
    def test_updates_require_manual_points_and_validate_each_pose(self):
        c = fake_control()
        c.travel_guard = Mock()
        poses = [dict(x_m=1, y_m=2, yaw_rad=.5), dict(x_m=2, y_m=3, yaw_rad=1)]
        self.assertEqual(c.plan_update({'goals': poses}), dict(goals=poses, capped=False))
        self.assertEqual([call.args[0] for call in c.travel_guard.call_args_list[1:]], poses)
        for goals in (None, [], {}, poses * 11, [dict(x_m=float('nan'), y_m=0, yaw_rad=0)]):
            with self.assertRaises(ValueError):
                c.plan_update({'goals': goals})
        with self.assertRaises(ValueError):
            c.plan_update({})
        c.travel_guard.side_effect = [None, ValueError('Blocked goal')]
        with self.assertRaisesRegex(ValueError, 'Blocked goal'):
            c.plan_update({'goals': poses})

    def test_window_expiry_pauses_and_owner_explicitly_starts_new_window(self):
        c = fake_control()
        c.robot.ros = Mock(is_shutdown=Mock(return_value=False))
        c.owner, c.task_id, c.phase = 'tab', 'updating', 'updating_map'
        c.deadline = time.monotonic() + .45
        c.map_update = dict(active=True, map_id='map', zone_revision=2, deadline=time.monotonic() - 1,
                            limit_s=60, pause_requested=False, pause_reason='', storage_error='', goals=[], state='updating')
        c.watchdog()
        self.assertTrue(c.map_update['pause_requested'])
        self.assertFalse(c.base_enabled)
        c.robot.nav.client.cancel_all_goals.assert_called_once()
        def start_again(_seconds):
            self.assertEqual(c.phase, 'update_paused')
            self.assertEqual(c.snapshot()['map_update']['remaining_s'], 0)
            args = {**packet(), 'type': 'update_start', 'task_id': c.task_id}
            with self.assertRaises(ValueError):
                c.receive({**args, 'session_id': 'other'})
            with self.assertRaises(ValueError):
                c.receive({**args, 'task_id': 'old'})
            c.receive(args)
        with patch('robot_control.wait_arm', return_value=[0, 10, 20]), patch('robot_control.time.sleep', side_effect=start_again):
            c.wait_update()
        self.assertEqual(c.phase, 'updating_map')
        self.assertGreater(c.map_update['deadline'], time.monotonic() + 59)
        self.assertFalse(c.map_update['pause_requested'])
        c.robot.nav.wait_stopped.assert_called_once()

    def test_pause_retries_current_view_and_updates_repeat_until_cancelled(self):
        c = fake_control()
        c.robot.ros = Mock(is_shutdown=Mock(return_value=False))
        c.owner, c.task_id, c.deadline = 'tab', 'updating', time.monotonic() + .45
        poses = [dict(x_m=1, y_m=1, yaw_rad=0), dict(x_m=2, y_m=1, yaw_rad=math.pi / 2)]
        c.plan_update = Mock(return_value=dict(goals=poses, capped=False))
        c.fold = Mock()
        dispatched = []
        def go(pose):
            dispatched.append(pose)
            if len(dispatched) == 1:
                c.receive({**packet(), 'type': 'update_pause', 'task_id': c.task_id})
                c.travel_guard(pose)
            if len(dispatched) == 5:
                raise Stopped('STOP')
        c.go = go
        def resume(_seconds):
            self.assertEqual(c.map_update['captures'], 0)
            self.assertEqual(c.map_update['completed'], 0)
            c.receive({**packet(), 'type': 'update_start', 'task_id': c.task_id})
        with tempfile.TemporaryDirectory() as directory:
            c.directory = Path(directory)
            with patch('robot_control.wait_arm', return_value=[0, 10, 20]), patch('robot_control.time.sleep', side_effect=resume):
                with self.assertRaises(Stopped):
                    c.run_update(dict(limit_s=60))
        self.assertEqual(dispatched[0], dispatched[1])
        self.assertEqual(dispatched[1], dispatched[3])
        captures = [call for call in c.capture.call_args_list if 'update_key' in call.kwargs]
        self.assertEqual(len(captures), 3)
        self.assertEqual(captures[0].kwargs['update_key'], captures[2].kwargs['update_key'])
        self.assertFalse(c.map_update['active'])
        self.assertEqual(c.map_update['state'], 'interrupted')

    def test_navigation_guard_enforces_window_even_without_ros_timer(self):
        c = fake_control()
        c.deadline = time.monotonic() + .45
        c.base_enabled = True
        c.map_update = dict(active=True, map_id='map', zone_revision=2,
                            deadline=time.monotonic() - 1, pause_requested=False)
        with self.assertRaises(UpdatePaused):
            c.travel_guard(dict(x_m=1, y_m=1, yaw_deg=0))
        self.assertFalse(c.base_enabled)
        c.robot.zero.assert_called_once()
        c.robot.nav.client.cancel_all_goals.assert_called_once()
        c.robot.validate_goal.assert_not_called()

    def test_refresh_preserves_history_without_inheriting_moving_item_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            stations = root / 'stations.json'
            stations.write_text('{}')
            index = PhotoIndex(root, stations)
            meta = dict(kind='observation', source='ros', map_id='map', frame='map', map_revision=1,
                        camera_id='front', base_pose=dict(x_m=1, y_m=2, yaw_rad=0), captured_at_s=1, update_key='view', current=True)
            first = index.add(demo_frame('front'), meta, 0)
            index.edit(first['id'], dict(expected_revision=1, item_id='new', name='Cup', appearance='Red', confirmed=True), 'map')
            second = index.add(demo_frame('front'), {**meta, 'captured_at_s': 2}, 2)
            third = index.add(demo_frame('front'), {**meta, 'captured_at_s': 3}, 3)
            snapshot = index.snapshot('map')
            self.assertEqual(len(snapshot['photos']), 2)
            self.assertFalse(snapshot['photos'][0]['current'])
            self.assertNotIn('item_id', snapshot['photos'][1])
            self.assertFalse(snapshot['items'][0]['observation_current'])
            self.assertIn('blocked_reason', target_preview(snapshot, {}, snapshot['items'][0]['id']))
            self.assertFalse(index.image_path(second['id']).exists())
            self.assertTrue(index.image_path(third['id']).exists())
            reopened = PhotoIndex(root, stations).snapshot('map')
            self.assertEqual(reopened, snapshot)
            before = deepcopy(index.value)
            with patch('web_backend.photos.atomic_json', side_effect=OSError('Disk full')):
                with self.assertRaises(OSError):
                    index.add(demo_frame('front'), {**meta, 'captured_at_s': 4}, 4)
            self.assertEqual(index.value, before)
            self.assertTrue(index.image_path(third['id']).exists())


class MapUpdateAPITest(unittest.IsolatedAsyncioTestCase):
    async def test_update_guards_and_failed_background_import_cancel_motion(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bridge = root / 'ros'
            grid = demo_map()
            atomic_json(bridge / 'map.json', grid)
            app = create_app(root / 'ui', hardware_dir=bridge)
            hardware = app[EDITOR].hardware
            hardware.refresh = Mock()
            hardware.control = dict(stamp_s=time.time(), boot_id='boot', phase='idle')
            hardware.sessions['tab'] = dict(ack=time.monotonic())
            hardware.rpc = AsyncMock(return_value=dict(status='accepted'))
            data = dict(id=str(uuid.uuid4()), type='map_update', session_id='tab',
                        map_id=grid['map_id'], expected_revision=app[EDITOR].zones['revision'], limit_s=60, goals=[dict(x_m=1, y_m=1, yaw_rad=0)])
            async with TestClient(TestServer(app)) as client:
                self.assertEqual((await client.post('/api/commands', json=data)).status, 400)
                hardware.cameras['front'] = dict(stamp_s=time.time(), url='camera')
                self.assertEqual((await client.post('/api/commands', json=data)).status, 200)
                self.assertEqual(hardware.rpc.call_args.args[0]['limit_s'], 60)
                self.assertEqual(hardware.rpc.call_args.args[0]['goals'], data['goals'])
                hardware.control.update(phase='update_paused', map_update=dict(active=True))
                capture_id = str(uuid.uuid4())
                capture_path = bridge / 'captures' / capture_id
                atomic_json(capture_path.with_suffix('.json'), dict(capture_id=capture_id, source='ros'))
                capture_path.with_suffix('.png').write_bytes(b'invalid-image')
                for _ in range(20):
                    if any(call.args[0].get('op') == 'stop' for call in hardware.rpc.call_args_list):
                        break
                    await asyncio.sleep(.05)
                stops = [call.args[0] for call in hardware.rpc.call_args_list if call.args[0].get('op') == 'stop']
                self.assertTrue(stops)
                self.assertIn('photo storage failed', stops[0]['reason'])
                self.assertTrue(capture_path.with_suffix('.json').exists())
                self.assertTrue(capture_path.with_suffix('.png').exists())
                hardware.control['map_update']['active'] = False
                hardware.rpc = AsyncMock(return_value=dict(status='stopping'))


if __name__ == '__main__':
    unittest.main()
