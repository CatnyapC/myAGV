import array
from contextlib import nullcontext
from pathlib import Path
from types import SimpleNamespace
import tempfile
import asyncio
import json
import time
import unittest
import uuid
from unittest.mock import AsyncMock, Mock, call, patch
from aiohttp.test_utils import TestClient, TestServer

from fetch_calibration import alignment_command, calibration_config, pickup_delta, position_alignment, vision_image
from test_robot_control import fake_control
from web_backend.resolve import locate_pickup
from web_backend.map_data import demo_map
from web_backend.photos import PhotoIndex, demo_frame
from web_backend.server import EDITOR, create_app
from web_backend.storage import atomic_bytes, atomic_json


class FetchCalibrationTest(unittest.IsolatedAsyncioTestCase):
    def test_preview_allows_off_axis_pose_but_alignment_requires_front_pose(self):
        c = fake_control()
        c.calibrate_fetch = Mock()
        packet = dict(type='fetch_test', preview=True, fetch_settings={})
        with patch('robot_control.wait_arm', return_value=[92.64, 3.65, 49.35, 92.64]):
            c.execute(packet)
            c.calibrate_fetch.assert_called_once_with(packet, {})
            c.calibrate_fetch.reset_mock()
            c.execute(dict(packet, preview=False))
            c.calibrate_fetch.assert_called_once_with(dict(packet, preview=False), {})
            c.calibrate_fetch.reset_mock()
        with patch('robot_control.wait_arm', return_value=[110, 3.65, 49.35, 92.64]):
            c.execute(packet)
            c.calibrate_fetch.reset_mock()
            with self.assertRaisesRegex(ValueError, 'J1'):
                c.execute(dict(packet, preview=False))
            c.calibrate_fetch.assert_not_called()
        c.arm.set_coords.assert_not_called()
        c.arm.set_angles.assert_not_called()

    def test_image_positions_convert_to_bounded_moves(self):
        p = dict(arm_goal=.56, arm_current=.30, front_goal=.50, front_current=.50)
        self.assertEqual(position_alignment(p, None, []), dict(x_mm=-2, y_mm=0))
        self.assertEqual(position_alignment(dict(p, arm_current=.8, front_current=.3), None, []), dict(x_mm=2, y_mm=-2))
        self.assertEqual(position_alignment(dict(p, arm_current=.57), None, []), dict(x_mm=0, y_mm=0))
        previous = dict(positions=dict(p, arm_current=.60), commanded_mm=dict(X=2, Y=0))
        self.assertEqual(position_alignment(dict(p, arm_current=.58), None, [previous]), dict(x_mm=2, y_mm=0))
        for value in (None, True, float('nan'), -0.1, 1.1):
            with self.assertRaises(ValueError):
                position_alignment(dict(p, arm_current=value), None, [])

    def test_grasp_height_records_only_measured_z_without_motion_or_capture(self):
        c = fake_control()
        c.arm.get_coords_info.return_value = [111, 222, 12.5]
        with patch('robot_control.wait_arm'), patch('robot_control.arm_deadline', return_value=nullcontext()):
            self.assertEqual(c.execute(dict(type='capture_grasp_height')), dict(grasp_z_mm=12.5))
        c.capture.assert_not_called()
        c.arm.set_coords.assert_not_called()
        c.arm.set_angles.assert_not_called()

    def test_mapping_limits_validation_and_compression(self):
        self.assertEqual(alignment_command({'x_mm': 2, 'y_mm': 1}, None), {'X': 2, 'Y': 1})
        self.assertEqual(alignment_command({'x_mm': -2, 'y_mm': -1}, None), {'X': -2, 'Y': -1})
        self.assertEqual(alignment_command({'x_mm': 0, 'y_mm': 0}, None), {'X': 0, 'Y': 0})
        self.assertEqual(alignment_command({'x_mm': 2, 'y_mm': -2}, {'max_step_mm': 1}), {'X': 1, 'Y': -1})
        for bad in (None, {}, {'x_mm': None, 'y_mm': 0}, {'x_mm': True, 'y_mm': 0},
                    {'x_mm': float('nan'), 'y_mm': 0}, {'x_mm': 2.1, 'y_mm': 0},
                    {'x_mm': .05, 'y_mm': 0}, {'x_mm': 0, 'y_mm': 0, 'grip': True}):
            with self.assertRaises(ValueError):
                alignment_command(bad, None)
        with self.assertRaises(ValueError):
            calibration_config({'max_step_mm': 3})
        self.assertEqual(pickup_delta('X', 1), (0, 1, 0))
        self.assertEqual(pickup_delta('Y', 1), (-1, 0, 0))
        cv2 = SimpleNamespace(INTER_AREA=3, IMWRITE_JPEG_QUALITY=1,
            resize=Mock(return_value='small'), imencode=Mock(return_value=(True, array.array('B', [1, 2]))))
        self.assertTrue(vision_image(SimpleNamespace(shape=(480, 640, 3)), cv2).startswith('data:image/jpeg;base64,'))
        self.assertEqual(cv2.resize.call_args.args[1], (480, 360))
        cv2.imencode.assert_called_once_with('.jpg', 'small', [1, 60])
        c = fake_control()
        c.arm.get_coords_info.side_effect = [[0, 200, 0], [0, 201, 0]]
        c.arm.get_angles_info.return_value = [90, 0, 0]
        with patch('robot_control.arm_deadline', return_value=nullcontext()):
            c.arm_step('X', 1, pickup=True)
        c.arm.set_coords.assert_called_once_with([0, 201, 0], 30)

    async def test_minimal_provider_request(self):
        images = dict(front='data:image/jpeg;base64,AQ==', arm='data:image/jpeg;base64,Ag==')
        provider = AsyncMock(return_value=dict(arm_goal=.5, arm_current=.5, front_goal=.5, front_current=.5))
        history = [dict(images=images, commanded_mm=dict(X=1, Y=-1))] * 2
        trace = {}
        with patch('web_backend.resolve.request_json', provider):
            await locate_pickup(images, dict(name='cup', appearance='red'), 'fake-key', images, history, prompt='Edited prompt', trace=trace)
        body = provider.call_args.args[0]
        self.assertEqual(provider.call_args.args[2], 30)
        self.assertEqual(body['reasoning'], {'enabled': False})
        self.assertEqual(body['max_tokens'], 64)
        self.assertTrue(body['response_format']['json_schema']['strict'])
        self.assertEqual([part['image_url']['detail'] for part in body['messages'][1]['content']
                          if part['type'] == 'image_url'], ['low'] * 8)
        labels = [part['text'] for part in body['messages'][1]['content'] if part['type'] == 'text']
        self.assertTrue(any('GOAL' in label for label in labels))
        self.assertTrue(any('"X": 1' in label and '"Y": -1' in label for label in labels))
        self.assertEqual(trace['messages'][0]['content'], 'Edited prompt')
        self.assertEqual(json.loads(trace['result_json']), provider.return_value)

    def test_closed_loop_recaptures_and_missing_target_stops(self):
        c = fake_control()
        c.task_id, c.arm_step = 'task', Mock()
        c.capture.return_value = {'image': 'compressed'}
        with tempfile.TemporaryDirectory() as folder:
            c.directory = Path(folder)
            results = iter([{'x_mm': 2, 'y_mm': 2}, {'x_mm': 0, 'y_mm': 0}])
            requests = []

            def respond(path, value):
                self.assertGreater(value['expires_at_s'] - time.time(), 34)
                requests.append(value)
                atomic_json(path, value)
                atomic_json(c.directory / 'fetch_vision_response.json', {'id': value['id'], 'result': next(results)})

            with patch('robot_control.atomic_json', side_effect=respond):
                c.calibrate_fetch({'vision_item': {'name': 'cup'}}, None)
            self.assertEqual(c.capture.call_args_list, [call('front', vision=True), call('arm', vision=True)] * 2)
            self.assertEqual([entry.args[0] for entry in c.arm_step.call_args_list], ['X', 'X', 'Y', 'Y'])
            self.assertEqual(requests[1]['history'][0]['commanded_mm'], {'X': 2, 'Y': 2})
            self.assertFalse(list(c.directory.iterdir()))
            c.arm_step.reset_mock()
            results = iter([{'x_mm': None, 'y_mm': 0}])
            with patch('robot_control.atomic_json', side_effect=respond), self.assertRaises(ValueError):
                c.calibrate_fetch({'vision_item': {'name': 'cup'}}, None)
            c.arm_step.assert_not_called()
            self.assertFalse(list(c.directory.iterdir()))
            requests.clear()
            results = iter([{'x_mm': .5, 'y_mm': 0}] * 8 + [{'x_mm': 0, 'y_mm': 0}])
            with patch('robot_control.atomic_json', side_effect=respond):
                c.calibrate_fetch({'vision_item': {'name': 'cup'}}, None)
            self.assertEqual(len(requests), 9)
            self.assertEqual(c.arm_step.call_count, 8)
            self.assertEqual(len(requests[-1]['history']), 2)
            c.arm_step.reset_mock()
            results = iter([{'x_mm': 2, 'y_mm': 1}])
            with patch('robot_control.atomic_json', side_effect=respond):
                c.calibrate_fetch({'vision_item': {'name': 'cup'}, 'preview': True}, None)
            c.arm_step.assert_not_called()
            self.assertTrue(requests[-1]['preview'])

    async def test_settings_persist_and_live_round_log_records_request_and_response(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bridge, ui = root / 'ros', root / 'ui'
            grid = {**demo_map(), 'map_id': 'test-map'}
            atomic_json(bridge / 'map.json', grid)
            index = PhotoIndex(ui, root / 'stations.json')
            image = demo_frame('front')
            reference = index.add(image, dict(kind='reference', source='phone'), 0)
            index.edit(reference['id'], dict(expected_revision=1, item_id='new', name='Cup', confirmed=True), grid['map_id'])
            item_id = index.value['items'][0]['id']
            goal = {}
            for camera in ('front', 'arm'):
                photo = index.add(image, dict(kind='observation', source='ros', camera_id=camera,
                    map_id=grid['map_id'], map_revision=0, frame='map', captured_at_s=time.time(),
                    base_pose=dict(x_m=0, y_m=0, yaw_rad=0)), index.value['revision'])
                photo_id = goal[camera] = photo['id']
                atomic_bytes(ui / 'images' / (photo_id + '.jpg'), camera.encode())
            index.save_grasp_goal(item_id, goal, index.value['revision'], grid['map_id'])
            entered, release = asyncio.Event(), asyncio.Event()

            async def provider(body, key, timeout):
                entered.set()
                await release.wait()
                return dict(arm_goal=.5, arm_current=.7, front_goal=.5, front_current=.3)

            with patch('web_backend.server.load_key', return_value=('configured', 'private-test-key')), patch('web_backend.resolve.request_json', side_effect=provider):
                app = create_app(ui, root / 'stations.json', hardware_dir=bridge)
                hardware = app[EDITOR].hardware
                hardware.refresh = Mock()
                hardware.control = dict(phase='idle', stamp_s=time.time())
                async with TestClient(TestServer(app)) as client:
                    settings = await (await client.get('/api/fetch-settings')).json()
                    values = {**settings['values'], 'max_step_mm': .5, 'prompt': 'Edited prompt'}
                    self.assertEqual((await client.put('/api/fetch-settings', json=dict(expected_revision=0, values={**values, 'max_step_mm': 3}))).status, 400)
                    saved = await (await client.put('/api/fetch-settings', json=dict(expected_revision=0, values=values))).json()
                    self.assertEqual(saved['revision'], 1)
                    self.assertEqual(json.loads((ui / 'fetch_settings.json').read_text()), saved)
                    self.assertEqual((await client.put('/api/fetch-settings', json=dict(expected_revision=0, values=values))).status, 409)
                    hardware.command = AsyncMock(return_value=dict(status='accepted'))
                    command = dict(id=str(uuid.uuid4()), type='fetch_test', preview=True, map_id=grid['map_id'],
                        expected_revision=app[EDITOR].zones['revision'], item_id=item_id, index_revision=index.value['revision'], fetch_revision=1,
                        fetch_settings=dict(max_step_mm=100))
                    self.assertEqual((await client.post('/api/commands', json=command)).status, 200)
                    forwarded = hardware.command.call_args.args[0]
                    self.assertEqual(forwarded['fetch_settings'], values)
                    self.assertEqual(forwarded['vision_item']['goal'], goal)
                    self.assertNotIn('station', forwarded)
                    command['fetch_revision'] = 0
                    self.assertEqual((await client.post('/api/commands', json=command)).status, 400)
                    hardware.control.update(phase='calibrating', boot_id='boot', task_id='task', stop_epoch=0)
                    self.assertEqual((await client.put('/api/fetch-settings', json=dict(expected_revision=1, values=values))).status, 409)
                    images = dict(front='data:image/jpeg;base64,AQ==', arm='data:image/jpeg;base64,Ag==')
                    value = dict(id='request', task_id='task', boot_id='boot', stop_epoch=0, map_id=grid['map_id'],
                        expires_at_s=time.time()+20, round=1, preview=True, prompt=values['prompt'], limits=values,
                        images=images, history=[], item=dict(name='Cup', appearance='Red', goal=goal))
                    atomic_json(bridge / 'fetch_vision_request.json', value)
                    try:
                        await asyncio.wait_for(entered.wait(), 2)
                        log = await (await client.get('/api/fetch-log')).json()
                        self.assertEqual(log['rounds'][0]['status'], 'requesting')
                        self.assertEqual(len([p for p in log['rounds'][0]['messages'][1]['content'] if p['type'] == 'image_url']), 4)
                        self.assertNotIn('private-test-key', json.dumps(log))
                        self.assertEqual(await (await client.get(f"/api/fetch-log?after={log['revision']}")).json(), dict(revision=log['revision']))
                    finally:
                        release.set()
                    for _ in range(40):
                        log = await (await client.get('/api/fetch-log')).json()
                        if log['rounds'][0]['status'] != 'requesting':
                            break
                        await asyncio.sleep(.025)
                    entry = log['rounds'][0]
                    self.assertEqual(entry['status'], 'returned')
                    self.assertEqual(json.loads(entry['result_json']), dict(arm_goal=.5, arm_current=.7, front_goal=.5, front_current=.3))
                    self.assertEqual(entry['limited_mm'], dict(X=.5, Y=-.5))
                    self.assertEqual(entry['messages'][0]['content'], values['prompt'])
                restarted = create_app(ui, root / 'stations.json', hardware_dir=bridge)
                async with TestClient(TestServer(restarted)) as client:
                    self.assertEqual(await (await client.get('/api/fetch-settings')).json(), saved)
                    self.assertEqual((await (await client.get('/api/fetch-log')).json())['rounds'], [])


if __name__ == '__main__':
    unittest.main()
