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
from navigation import Navigation
from aiohttp.test_utils import TestClient, TestServer

from fetch_calibration import alignment_command, calibration_config, pickup_delta, qualitative_alignment, vision_image
from test_robot_control import fake_control
from web_backend.resolve import locate_pickup
from web_backend.map_data import demo_map
from web_backend.photos import PhotoIndex, demo_frame
from web_backend.server import EDITOR, create_app
from web_backend.storage import atomic_bytes, atomic_json


class FetchCalibrationTest(unittest.IsolatedAsyncioTestCase):
    def test_rotation_buttons_use_saved_sizes_without_arm_or_camera(self):
        c = fake_control()
        c.arm, c.arm_homed, c.turn_fetch = None, False, Mock()
        config = calibration_config(dict(turn_step_deg=1.6, turn_speed_rad_s=.02, turn_duration_scale=.6))
        for size, angle in (('s', .4), ('m', .8), ('l', 1.6)):
            c.execute(dict(type='fetch_turn_test', size=size, fetch_settings=config))
            c.turn_fetch.assert_called_with(angle, config)
        c.capture.assert_not_called()
        with self.assertRaises(ValueError):
            c.execute(dict(type='fetch_turn_test', size='invalid', fetch_settings=config))

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

    def test_visual_labels_convert_to_bounded_moves(self):
        for size, step, arm_step in (('large', 2, 10), ('medium', 1, 5), ('small', .5, 1)):
            self.assertEqual(qualitative_alignment(dict(arm='left_' + size, front='right_' + size), None), dict(x_mm=-arm_step, turn_deg=step))
            self.assertEqual(qualitative_alignment(dict(arm='right_' + size, front='left_' + size), None), dict(x_mm=arm_step, turn_deg=-step))
        self.assertEqual(qualitative_alignment(dict(arm='aligned_perfectly', front='aligned_perfectly'), None), dict(x_mm=0, turn_deg=0))
        self.assertEqual(qualitative_alignment(dict(arm='right_large', front='right_large'), dict(max_step_mm=.5)), dict(x_mm=.5, turn_deg=2))
        for label in ('unknown', 'aligned', None, 0, 'right', {}, []):
            with self.assertRaises(ValueError):
                qualitative_alignment(dict(arm=label, front='aligned_perfectly'), None)

    def test_base_turn_is_clockwise_without_translation_and_stops(self):
        c = fake_control()
        c.travel_guard = Mock()
        c.robot.nav.get_odom_pose.side_effect = [dict(x_m=0, y_m=0, yaw_deg=0),
            dict(x_m=0, y_m=0, yaw_deg=0), dict(x_m=0, y_m=0, yaw_deg=-2)]
        with patch('robot_control.time.sleep'):
            c.turn_fetch(2)
        self.assertEqual(c.robot.velocity[:2], (0, 0))
        self.assertLess(c.robot.velocity[2], 0)
        self.assertFalse(c.base_enabled)
        c.robot.zero.assert_called()
        c.robot.nav.get_odom_pose.side_effect = [dict(x_m=0, y_m=0, yaw_deg=0), dict(x_m=.02, y_m=0, yaw_deg=-2)]
        with patch('robot_control.time.sleep'):
            c.turn_fetch(2)
        self.assertFalse(c.base_enabled)

    def test_rotation_odometry_is_fresh_and_independent_of_map_pose(self):
        nav = Navigation.__new__(Navigation)
        pose = SimpleNamespace(position=SimpleNamespace(x=.2, y=.3),
            orientation=SimpleNamespace(x=0, y=0, z=0, w=1))
        nav.odom = (time.monotonic(), SimpleNamespace(pose=SimpleNamespace(pose=pose)))
        self.assertEqual(nav.get_odom_pose(), dict(x_m=.2, y_m=.3, yaw_deg=0))
        nav.odom = (time.monotonic() - 1, nav.odom[1])
        with self.assertRaisesRegex(RuntimeError, 'Fresh odometry'):
            nav.get_odom_pose()

    def test_rotation_pulse_expires_without_aborting_visual_loop(self):
        c = fake_control()
        c.travel_guard = Mock()
        c.robot.nav.get_odom_pose.return_value = dict(x_m=0, y_m=0, yaw_deg=0)
        now = [0.0]
        def tick(seconds):
            now[0] += seconds
        with patch('robot_control.time.monotonic', side_effect=lambda: now[0]), patch('robot_control.time.sleep', side_effect=tick):
            c.turn_fetch(2)
        self.assertGreaterEqual(now[0], 1.86)
        self.assertLess(now[0], 1.95)
        self.assertEqual(c.robot.velocity, (0, 0, -.015))
        self.assertFalse(c.base_enabled)
        c.robot.zero.assert_called()
        self.assertIn('rechecking', c.status)
        now[0] = 0
        with patch('robot_control.time.monotonic', side_effect=lambda: now[0]), patch('robot_control.time.sleep', side_effect=tick):
            c.turn_fetch(2, dict(turn_speed_rad_s=.02, turn_duration_scale=.5))
        self.assertGreaterEqual(now[0], .87)
        self.assertLess(now[0], .93)
        self.assertEqual(c.robot.velocity, (0, 0, -.02))
        self.assertEqual(qualitative_alignment(dict(arm='aligned_perfectly', front='right_medium'),
            dict(turn_step_deg=1)), dict(x_mm=0, turn_deg=.5))
        settings = dict(turn_s_speed_rad_s=.018, turn_s_duration_s=.2,
                        turn_m_speed_rad_s=.022, turn_m_duration_s=.4,
                        turn_l_speed_rad_s=.025, turn_l_duration_s=.7)
        for size, degrees in (('s', .5), ('m', 1), ('l', 2)):
            now[0] = 0
            with patch('robot_control.time.monotonic', side_effect=lambda: now[0]), patch('robot_control.time.sleep', side_effect=tick):
                c.turn_fetch(degrees, settings)
            self.assertAlmostEqual(c.robot.velocity[2], -settings['turn_' + size + '_speed_rad_s'])
            self.assertGreaterEqual(now[0], settings['turn_' + size + '_duration_s'])
            self.assertLess(now[0], settings['turn_' + size + '_duration_s'] + .051)
        for bad in (0, True, float('nan'), 16):
            with self.assertRaises(ValueError):
                calibration_config(dict(turn_s_duration_s=bad))

    def test_grasp_height_records_only_measured_z_without_motion_or_capture(self):
        c = fake_control()
        c.arm.get_coords_info.return_value = [111, 222, 12.5]
        with patch('robot_control.wait_arm'), patch('robot_control.arm_deadline', return_value=nullcontext()):
            self.assertEqual(c.execute(dict(type='capture_grasp_height')), dict(grasp_z_mm=12.5))
        c.capture.assert_not_called()
        c.arm.set_coords.assert_not_called()
        c.arm.set_angles.assert_not_called()

    def test_mapping_limits_validation_and_compression(self):
        self.assertEqual(alignment_command({'x_mm': 2, 'turn_deg': 1}, None), {'X': 2, 'turn_deg': 1})
        self.assertEqual(alignment_command({'x_mm': -2, 'turn_deg': -1}, None), {'X': -2, 'turn_deg': -1})
        self.assertEqual(alignment_command({'x_mm': 0, 'turn_deg': 0}, None), {'X': 0, 'turn_deg': 0})
        self.assertEqual(alignment_command({'x_mm': 2, 'turn_deg': -2}, {'max_step_mm': 1}), {'X': 1, 'turn_deg': -2})
        for bad in (None, {}, {'x_mm': None, 'turn_deg': 0}, {'x_mm': True, 'turn_deg': 0},
                    {'x_mm': float('nan'), 'turn_deg': 0}, {'x_mm': 10.1, 'turn_deg': 0},
                    {'x_mm': .05, 'turn_deg': 0}, {'x_mm': 0, 'turn_deg': 0, 'grip': True}):
            with self.assertRaises(ValueError):
                alignment_command(bad, None)
        with self.assertRaises(ValueError):
            calibration_config({'max_step_mm': 11})
        self.assertEqual(pickup_delta('X', 1), (0, 1, 0))
        self.assertEqual(pickup_delta('Y', 1), (-1, 0, 0))
        cv2 = SimpleNamespace(INTER_AREA=3, IMWRITE_JPEG_QUALITY=1,
            resize=Mock(return_value='small'), imencode=Mock(return_value=(True, array.array('B', [1, 2]))))
        self.assertTrue(vision_image(SimpleNamespace(shape=(480, 640, 3)), cv2).startswith('data:image/jpeg;base64,'))
        self.assertEqual(cv2.resize.call_args.args[1], (480, 360))
        cv2.imencode.assert_called_once_with('.jpg', 'small', [1, 60])
        cv2.resize.reset_mock()
        cv2.imencode.reset_mock()
        frame = SimpleNamespace(shape=(1080, 1920, 3))
        vision_image(frame, cv2, original_resolution=True)
        cv2.resize.assert_not_called()
        cv2.imencode.assert_called_once_with('.jpg', frame, [1, 60])
        c = fake_control()
        c.arm.get_coords_info.side_effect = [[0, 200, 0], [0, 201, 0]]
        c.arm.get_angles_info.return_value = [90, 0, 0]
        with patch('robot_control.arm_deadline', return_value=nullcontext()):
            c.arm_step('X', 1, pickup=True)
        c.arm.set_coords.assert_called_once_with([0, 201, 0], 30)

    async def test_minimal_provider_request(self):
        images = dict(front='data:image/jpeg;base64,AQ==', arm='data:image/jpeg;base64,Ag==')
        provider = AsyncMock(return_value=dict(front='aligned_perfectly'))
        history = [dict(images=images, commanded=dict(X=1, turn_deg=-1))] * 2
        trace = {}
        with patch('web_backend.resolve.request_json', provider):
            await locate_pickup(images, dict(name='cup', appearance='red'), 'fake-key', images['front'], history, prompt='Edited prompt', trace=trace)
        body = provider.call_args.args[0]
        self.assertEqual(provider.call_args.args[2], 30)
        self.assertEqual(body['reasoning'], {'enabled': False})
        self.assertEqual(body['max_tokens'], 64)
        self.assertTrue(body['response_format']['json_schema']['strict'])
        self.assertEqual([part['image_url']['detail'] for part in body['messages'][1]['content']
                          if part['type'] == 'image_url'], ['high'] * 4)
        labels = [part['text'] for part in body['messages'][1]['content'] if part['type'] == 'text']
        self.assertTrue(any('Uploaded item reference' in label for label in labels))
        self.assertFalse(any('GOAL' in label for label in labels))
        self.assertTrue(any('"X": 1' in label and '"turn_deg": -1' in label for label in labels))
        self.assertTrue(trace['messages'][0]['content'].startswith('Edited prompt'))
        self.assertEqual(json.loads(trace['result_json']), provider.return_value)
        self.assertTrue(trace['aligned_perfectly'])

    def test_closed_loop_stages_never_mix_motors(self):
        c = fake_control()
        c.task_id, c.arm_step, c.turn_fetch = 'task', Mock(), Mock()
        c.capture.return_value = {'image': 'compressed'}
        with tempfile.TemporaryDirectory() as folder:
            c.directory = Path(folder)
            results = iter([dict(x_mm=0, turn_deg=1), dict(x_mm=0, turn_deg=0),
                            dict(x_mm=1, y_mm=-1, turn_deg=0), dict(x_mm=0, y_mm=0, turn_deg=0)])
            requests = []
            def respond(path, value):
                requests.append(value)
                atomic_json(path, value)
                result = next(results)
                atomic_json(c.directory / 'fetch_vision_response.json', dict(id=value['id'], result=result,
                    aligned_perfectly=not any(result.values())))
            with patch('robot_control.atomic_json', side_effect=respond):
                c.calibrate_fetch({'vision_item': {'name': 'cup'}}, None)
            self.assertEqual([r['stage'] for r in requests], ['base', 'base', 'arm', 'arm'])
            self.assertEqual(requests[2]['history'], [])
            self.assertEqual(c.capture.call_args_list, [call('front', vision=True)] * 2 + [call('arm', vision=True)] * 2)
            c.turn_fetch.assert_called_once_with(1, calibration_config(None))
            self.assertEqual(c.arm_step.call_args_list, [call('X', 1, pickup=True, distance=1), call('Y', -1, pickup=True, distance=1)])
            self.assertFalse(list(c.directory.iterdir()))
            results = iter([dict(x_mm=1, turn_deg=0)])
            with patch('robot_control.atomic_json', side_effect=respond), self.assertRaisesRegex(ValueError, 'violates active stage'):
                c.calibrate_fetch({'vision_item': {'name': 'cup'}}, None)
            results = iter([dict(x_mm=0, turn_deg=0), dict(x_mm=0, y_mm=0, turn_deg=1)])
            with patch('robot_control.atomic_json', side_effect=respond), self.assertRaisesRegex(ValueError, 'violates active stage'):
                c.calibrate_fetch({'vision_item': {'name': 'cup'}}, None)

    async def test_arm_goal_labels_and_y_direction(self):
        images = dict(arm='data:image/jpeg;base64,AQ==')
        trace = {}
        for label, expected in [('plus_medium', 5), ('minus_small', -1)]:
            with patch('web_backend.resolve.request_json', AsyncMock(return_value=dict(x='aligned_perfectly', y=label))):
                moves = await locate_pickup(images, dict(name='cup', appearance='red'), 'key', images['arm'], [],
                    stage='arm', goal=images['arm'], trace=trace)
            self.assertEqual(moves, dict(x_mm=0, y_mm=expected, turn_deg=0))
            self.assertFalse(trace['aligned_perfectly'])
            self.assertEqual(trace['stage'], 'arm')
            self.assertIn('exposes MORE', trace['messages'][0]['content'])
            self.assertTrue(any('GOAL:' in p.get('text', '') for p in trace['messages'][1]['content']))

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
            cameras = {}
            for camera in ('front', 'arm'):
                photo = index.add(image, dict(kind='observation', source='ros', camera_id=camera,
                    map_id=grid['map_id'], map_revision=1, frame='map', captured_at_s=time.time(),
                    base_pose=dict(x_m=0, y_m=0, yaw_rad=0)), index.value['revision'])
                index.image_path(photo['id']).with_suffix('.jpg').write_bytes(image)
                cameras[camera] = photo['id']
            index.save_grasp_goal(item_id, cameras, index.value['revision'], grid['map_id'])
            entered, release = asyncio.Event(), asyncio.Event()

            async def provider(body, key, timeout):
                entered.set()
                await release.wait()
                return dict(front='left_large')

            with patch('web_backend.server.load_key', return_value=('configured', 'private-test-key')), patch('web_backend.resolve.request_json', side_effect=provider):
                app = create_app(ui, root / 'stations.json', hardware_dir=bridge)
                hardware = app[EDITOR].hardware
                hardware.refresh = Mock()
                hardware.control = dict(phase='idle', stamp_s=time.time())
                async with TestClient(TestServer(app)) as client:
                    settings = await (await client.get('/api/fetch-settings')).json()
                    values = {**settings['values'], 'max_step_mm': .5, 'prompt': 'Edited prompt'}
                    self.assertEqual((await client.put('/api/fetch-settings', json=dict(expected_revision=0, values={**values, 'max_step_mm': 11}))).status, 400)
                    saved = await (await client.put('/api/fetch-settings', json=dict(expected_revision=0, values=values))).json()
                    self.assertEqual(saved['revision'], 1)
                    self.assertEqual(json.loads((ui / 'fetch_settings.json').read_text()), saved)
                    self.assertEqual((await client.put('/api/fetch-settings', json=dict(expected_revision=0, values=values))).status, 409)
                    hardware.command = AsyncMock(return_value=dict(status='accepted'))
                    turn = dict(id=str(uuid.uuid4()), type='fetch_turn_test', size='s', fetch_revision=1,
                        map_id=grid['map_id'], expected_revision=app[EDITOR].zones['revision'], fetch_settings=dict(turn_speed_rad_s=100))
                    self.assertEqual((await client.post('/api/commands', json=turn)).status, 200)
                    self.assertEqual(hardware.command.call_args.args[0]['fetch_settings'], values)
                    self.assertEqual((await client.post('/api/commands', json=dict(turn, fetch_revision=0))).status, 400)
                    self.assertEqual((await client.post('/api/commands', json=dict(turn, size='invalid'))).status, 400)
                    command = dict(id=str(uuid.uuid4()), type='fetch_test', preview=True, map_id=grid['map_id'],
                        expected_revision=app[EDITOR].zones['revision'], item_id=item_id, index_revision=index.value['revision'], fetch_revision=1,
                        fetch_settings=dict(max_step_mm=100))
                    self.assertEqual((await client.post('/api/commands', json=command)).status, 200)
                    forwarded = hardware.command.call_args.args[0]
                    self.assertEqual(forwarded['fetch_settings'], values)
                    self.assertEqual(forwarded['vision_item']['reference_photo_id'], reference['id'])
                    self.assertNotIn('goal', forwarded['vision_item'])
                    self.assertNotIn('station', forwarded)
                    command['fetch_revision'] = 0
                    self.assertEqual((await client.post('/api/commands', json=command)).status, 400)
                    hardware.control.update(phase='calibrating', boot_id='boot', task_id='task', stop_epoch=0)
                    self.assertEqual((await client.put('/api/fetch-settings', json=dict(expected_revision=1, values=values))).status, 409)
                    images = dict(front='data:image/jpeg;base64,AQ==', arm='data:image/jpeg;base64,Ag==')
                    value = dict(id='request', task_id='task', boot_id='boot', stop_epoch=0, map_id=grid['map_id'],
                        expires_at_s=time.time()+20, round=1, preview=True, prompt=values['prompt'], limits=values,
                        images=images, history=[], item=dict(name='Cup', appearance='Red', reference_photo_id=reference['id']))
                    atomic_json(bridge / 'fetch_vision_request.json', value)
                    try:
                        await asyncio.wait_for(entered.wait(), 2)
                        log = await (await client.get('/api/fetch-log')).json()
                        self.assertEqual(log['rounds'][0]['status'], 'requesting')
                        self.assertEqual(len([p for p in log['rounds'][0]['messages'][1]['content'] if p['type'] == 'image_url']), 2)
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
                    self.assertEqual(json.loads(entry['result_json']), dict(front='left_large'))
                    self.assertEqual(entry['correction'], dict(X=0, turn_deg=-2))
                    self.assertTrue(entry['messages'][0]['content'].startswith(values['prompt']))
                    entered.clear()
                    release.clear()
                    atomic_json(bridge / 'fetch_vision_request.json', dict(value, id='request-2', round=2))
                    await asyncio.wait_for(entered.wait(), 2)
                    hardware.stop = AsyncMock(return_value=dict(status='completed'))
                    self.assertEqual((await client.post('/api/stop')).status, 200)
                    cleared = await (await client.get('/api/fetch-log')).json()
                    self.assertEqual(cleared['rounds'], [])
                    release.set()
                    for _ in range(40):
                        finished = await (await client.get('/api/fetch-log')).json()
                        if finished['revision'] > cleared['revision']:
                            break
                        await asyncio.sleep(.025)
                    self.assertEqual((await (await client.get('/api/fetch-log')).json())['rounds'], [])
                    self.assertGreater(finished['revision'], cleared['revision'])
                    response = json.loads((bridge / 'fetch_vision_response.json').read_text())
                    self.assertNotEqual(response['id'], 'request-2')
                restarted = create_app(ui, root / 'stations.json', hardware_dir=bridge)
                async with TestClient(TestServer(restarted)) as client:
                    self.assertEqual(await (await client.get('/api/fetch-settings')).json(), saved)
                    self.assertEqual((await (await client.get('/api/fetch-log')).json())['rounds'], [])


if __name__ == '__main__':
    unittest.main()
