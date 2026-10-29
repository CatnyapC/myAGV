"""Saved camera poses survive restart, stay map-scoped and update individually."""
import asyncio
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
from web_backend.photos import PhotoIndex, demo_frame
from web_backend.server import EDITOR, create_app
from web_backend.storage import atomic_bytes, atomic_json


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
    async def test_manual_update_captures_once_saves_current_point_and_estimates_items(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bridge, ui = root / 'ros', root / 'ui'
            grid = demo_map()
            atomic_json(bridge / 'map.json', grid)
            store = PhotoIndex(ui, root / 'stations.json')
            image = demo_frame('front')
            reference = store.add(image, dict(kind='reference', source='phone'), 0)
            store.edit(reference['id'], dict(expected_revision=1, item_id='new', name='Cup', confirmed=True), grid['map_id'])
            item_id = store.value['items'][0]['id']
            measured = dict(x_m=3.1, y_m=4.2, yaw_rad=-1)
            points = [point(1, 2), point(3, 4, .7)]
            capture_id = str(uuid.uuid4())

            async def capture(command, wait):
                self.assertTrue(wait)
                self.assertEqual(command['type'], 'capture')
                self.assertEqual(command['camera_id'], 'front')
                self.assertEqual(command['expected_revision'], 0)
                self.assertEqual(command['session_id'], 'tab')
                self.assertEqual(command['stop_epoch'], 4)
                path = bridge / 'captures' / capture_id
                atomic_bytes(path.with_suffix('.png'), image)
                atomic_json(path.with_suffix('.json'), dict(capture_id=capture_id, source='ros', kind='observation',
                    camera_id='front', captured_at_s=time.time(), map_id=grid['map_id'], map_revision=0,
                    frame=grid['frame'], base_pose=measured))
                return dict(capture_id=capture_id)

            infer = AsyncMock(return_value=[dict(item_id=item_id, x_m=3.5, y_m=4, confidence=.9, uncertainty_m=.4)])
            with patch('web_backend.server.load_key', return_value=('configured', 'fake')), \
                 patch('web_backend.server.estimate_items', infer):
                app = create_app(ui, root / 'stations.json', hardware_dir=bridge)
                hardware = app[EDITOR].hardware
                hardware.refresh = Mock()
                hardware.control = dict(stamp_s=time.time(), phase='idle')
                hardware.command = AsyncMock(side_effect=capture)
                data = dict(id=str(uuid.uuid4()), session_id='tab', stop_epoch=4, map_id=grid['map_id'],
                    expected_revision=2, zone_revision=0, camera_id='front', save_observation_point=True)
                async with TestClient(TestServer(app)) as client:
                    app[EDITOR].observation_points.save(grid['map_id'], points, 1)
                    hardware.control['phase'] = 'navigating'
                    self.assertEqual((await client.post('/api/photos/capture', json=data)).status, 409)
                    hardware.command.assert_not_called()
                    hardware.control['phase'] = 'idle'
                    self.assertEqual((await client.post('/api/photos/capture', json={**data, 'map_id': 'old'})).status, 400)
                    self.assertEqual((await client.post('/api/photos/capture', json={**data, 'expected_revision': 0})).status, 400)
                    hardware.command.assert_not_called()
                    response = await client.post('/api/photos/capture', json=data)
                    self.assertEqual(response.status, 200, await response.text())
                    snapshot = await response.json()
                    observation = next(p for p in snapshot['photos'] if p['id'] == snapshot['added_id'])
                    self.assertEqual(observation['base_pose'], measured)
                    saved_points = await (await client.get('/api/observation-points')).json()
                    self.assertEqual(saved_points['points'][:2], points)
                    self.assertEqual(saved_points['points'][2], dict(id=observation['observation_point_id'], **measured))
                    hardware.command.assert_awaited_once()
                    for _ in range(50):
                        snapshot = await (await client.get('/api/items')).json()
                        if snapshot['locations']:
                            break
                        await asyncio.sleep(.05)
                    self.assertEqual(snapshot['locations'][0]['item_id'], item_id)
                    infer.assert_awaited_once()
                    self.assertEqual(infer.call_args.args[3]['base_pose'], observation['base_pose'])
            saved = PhotoIndex(ui, root / 'stations.json').snapshot(grid['map_id'])
            self.assertEqual(saved['locations'][0]['item_id'], item_id)

    async def test_manual_capture_needs_no_existing_points_items_or_model(self):
        with tempfile.TemporaryDirectory() as directory, \
             patch('web_backend.server.load_key', return_value=('not_configured', None)):
            app = create_app(directory)
            async with TestClient(TestServer(app)) as client:
                pose = app[EDITOR].sim.telemetry()['pose']
                response = await client.post('/api/photos/capture', json=dict(map_id=app[EDITOR].grid['map_id'],
                    camera_id='front', expected_revision=0, save_observation_point=True))
                self.assertEqual(response.status, 200, await response.text())
                photo = (await response.json())['photos'][0]
                self.assertEqual(photo['base_pose'], pose)
                points = await (await client.get('/api/observation-points')).json()
                self.assertEqual(points['points'], [dict(id=photo['observation_point_id'], **pose)])
            self.assertEqual(ObservationPoints(directory).load(app[EDITOR].grid['map_id']), points)

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
