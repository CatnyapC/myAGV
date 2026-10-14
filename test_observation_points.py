"""Saved camera poses survive restart, stay map-scoped and update individually."""
import math
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import AsyncMock, Mock, patch
import uuid

from aiohttp.test_utils import TestClient, TestServer

from test_robot_control import fake_control
from web_backend.map_data import demo_map
from web_backend.observation_points import ObservationPoints
from web_backend.server import EDITOR, create_app
from web_backend.storage import atomic_json


def point(x=1, y=2, yaw=.5):
    return dict(id=str(uuid.uuid4()), x_m=x, y_m=y, yaw_rad=yaw)


class ObservationPointsTest(unittest.TestCase):
    def test_restart_map_scope_migration_and_failed_write(self):
        with tempfile.TemporaryDirectory() as directory:
            store = ObservationPoints(directory)
            points = [point(), point(2, 3)]
            saved = store.save('map-a', points, 1)
            self.assertEqual(ObservationPoints(directory).load('map-a'), saved)
            self.assertEqual(store.load('map-b')['points'], [])
            with patch('web_backend.storage.os.replace', side_effect=OSError('disk full')):
                with self.assertRaises(OSError):
                    store.save('map-a', [], 2)
            self.assertEqual(store.load('map-a'), saved)
            store.migrate('map-b', dict(map_id='map-a', goals=[dict(x_m=1, y_m=2, yaw_rad=0)]))
            self.assertEqual(store.load('map-b')['points'], [])
            store.migrate('map-b', dict(map_id='map-b', goals=[dict(x_m=1, y_m=2, yaw_rad=0)]))
            migrated = store.load('map-b')
            self.assertEqual(len(migrated['points']), 1)
            store.migrate('map-b', dict(map_id='map-b', goals=[dict(x_m=8, y_m=9, yaw_rad=0)]))
            self.assertEqual(store.load('map-b'), migrated)
            store.save('map-b', [], 2)
            store.migrate('map-b', dict(map_id='map-b', goals=[dict(x_m=8, y_m=9, yaw_rad=0)]))
            self.assertEqual(store.load('map-b')['points'], [])

    def test_invalid_points_do_not_replace_saved_data(self):
        with tempfile.TemporaryDirectory() as directory:
            store = ObservationPoints(directory)
            p = point()
            saved = store.save('map', [p], 1)
            for invalid in (None, {}, [p] * 21, [p, p], [{**p, 'id': 3}],
                            [{**p, 'x_m': float('nan')}], [{**p, 'y_m': True}],
                            [{**p, 'yaw_rad': math.pi + .1}], [{**p, 'extra': 1}]):
                with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                    store.save('map', invalid, 2)
                self.assertEqual(store.load('map'), saved)

    def test_selected_survey_sends_full_targets_once_and_reuses_photo_key(self):
        c = fake_control()
        c.travel_guard = Mock()
        c.move_arm = Mock()
        c.config['transport_angles'] = [90, 10, 20, 89]
        pose = dict(x_m=1, y_m=2, yaw_rad=.5)
        c.execute(dict(type='survey', goals=[pose]))
        c.move_arm.assert_called_once_with([90, 10, 20, 89])
        c.robot.nav.go_to.assert_called_once()
        self.assertEqual(c.robot.nav.go_to.call_args.args[0], dict(x_m=1, y_m=2, yaw_deg=math.degrees(.5)))
        self.assertEqual(c.capture.call_args_list[-1].kwargs, dict(update_key='1.00:2.00:0.50'))
        self.assertEqual(c.capture.call_count, 2)  # One non-persisted preflight, then one saved photo.


class ObservationPointsAPITest(unittest.IsolatedAsyncioTestCase):
    async def test_save_restart_conflict_and_map_switch(self):
        with tempfile.TemporaryDirectory() as directory:
            points = [point(), point(3, 4)]
            async with TestClient(TestServer(create_app(directory))) as client:
                current = await (await client.get('/api/observation-points')).json()
                data = dict(map_id=current['map_id'], expected_revision=0, points=points)
                response = await client.put('/api/observation-points', json=data)
                self.assertEqual(response.status, 200, await response.text())
                saved = await response.json()
                self.assertEqual((await client.put('/api/observation-points', json=data)).status, 409)
                self.assertEqual((await client.put('/api/observation-points', json={**data, 'expected_revision': True})).status, 409)
                self.assertEqual((await client.put('/api/observation-points', json={**data, 'map_id': 'old', 'expected_revision': 1})).status, 409)
            app = create_app(directory)
            async with TestClient(TestServer(app)) as client:
                self.assertEqual(await (await client.get('/api/observation-points')).json(), saved)
                app[EDITOR].grid = {**app[EDITOR].grid, 'map_id': 'new-map'}
                self.assertEqual((await (await client.get('/api/observation-points')).json())['points'], [])
                self.assertEqual((await client.put('/api/observation-points', json={**data, 'expected_revision': 1})).status, 409)

    async def test_single_update_resolves_only_saved_point_and_preserves_guards(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bridge = root / 'ros'
            grid = demo_map()
            atomic_json(bridge / 'map.json', grid)
            points = [point(), point(3, 4)]
            app = create_app(root / 'ui', hardware_dir=bridge)
            hardware = app[EDITOR].hardware
            hardware.refresh = Mock()
            hardware.control = dict(stamp_s=time.time(), boot_id='boot', phase='idle')
            hardware.sessions['tab'] = dict(ack=time.monotonic())
            hardware.command = AsyncMock(return_value=dict(status='accepted'))
            saved = app[EDITOR].observation_points.save(grid['map_id'], points, 1)
            data = dict(id=str(uuid.uuid4()), type='survey', session_id='tab', map_id=grid['map_id'],
                        expected_revision=app[EDITOR].zones['revision'], observation_point_id=points[1]['id'], observation_revision=1,
                        goals=[dict(x_m=999, y_m=999, yaw_rad=0)])
            async with TestClient(TestServer(app)) as client:
                self.assertEqual((await client.post('/api/commands', json=data)).status, 400)
                hardware.command.assert_not_called()
                hardware.cameras['front'] = dict(stamp_s=time.time(), url='camera')
                response = await client.post('/api/commands', json=data)
                self.assertEqual(response.status, 200, await response.text())
                self.assertEqual(hardware.command.call_args.args[0]['goals'], [{k: v for k, v in points[1].items() if k != 'id'}])
                hardware.command.reset_mock()
                self.assertEqual((await client.post('/api/commands', json={**data, 'observation_revision': 0})).status, 409)
                self.assertEqual((await client.post('/api/commands', json={**data, 'observation_point_id': str(uuid.uuid4())})).status, 400)
                hardware.command.assert_not_called()
                hardware.control['phase'] = 'navigating'
                response = await client.put('/api/observation-points', json=dict(map_id=grid['map_id'], expected_revision=1, points=[]))
                self.assertEqual(response.status, 409)
                self.assertEqual(app[EDITOR].observation_points.load(grid['map_id']), saved)


if __name__ == '__main__':
    unittest.main()
