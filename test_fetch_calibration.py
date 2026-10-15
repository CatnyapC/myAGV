import array
import ast
import base64
from contextlib import nullcontext
from pathlib import Path
from types import SimpleNamespace
import tempfile
import asyncio
import json
import time
import threading
import unittest
import uuid
from unittest.mock import ANY, AsyncMock, Mock, call, patch
from navigation import Navigation
from aiohttp.test_utils import TestClient, TestServer

from fetch_calibration import alignment_command, calibration_config, measured_arm_alignment, pickup_delta, qualitative_alignment, vision_image
from test_robot_control import fake_control
from robot_control import Stopped
from robot_safety import fresh, number
from P340.keyboard_control import LIMITS
from web_backend.resolve import locate_pickup
from web_backend.map_data import demo_map
from web_backend.photos import PhotoIndex, demo_frame
from web_backend.server import EDITOR, create_app
from web_backend.storage import atomic_bytes, atomic_json


class FetchCalibrationTest(unittest.IsolatedAsyncioTestCase):
    def test_align_waits_for_sensor_refresh_without_discarding_response(self):
        c = fake_control()
        c.task_id = 'same-align-task'
        c.robot.ros = Mock(is_shutdown=Mock(return_value=False))
        c.capture.return_value = dict(image='compressed')
        ready = [True]
        c.robot.sensors_ready = lambda: ready[0]
        with tempfile.TemporaryDirectory() as folder:
            c.directory = Path(folder)
            def respond(path, value):
                atomic_json(path, value)
                atomic_json(c.directory / 'fetch_vision_response.json',
                    dict(id=value['id'], result=dict(x_mm=0, turn_deg=0), aligned_perfectly=True))
                ready[0] = False
                c.watchdog()
            with patch('robot_control.atomic_json', side_effect=respond), \
                 patch('robot_control.time.sleep', side_effect=lambda _: ready.__setitem__(0, True)):
                c.calibrate_fetch(dict(preview=True, vision_item=dict(name='cup')), None)
        self.assertIsNone(c.stop_pending)
        self.assertEqual(c.stop_epoch, 0)
        self.assertEqual(c.task_id, 'same-align-task')
        c.robot.nav.wait_stopped.assert_any_call(timeout=1)
        c.capture.assert_called_once_with('front', vision=True)
        c.arm.set_coords.assert_not_called()

    def test_grasp_wire_commands_keep_clamp_and_send_one_full_target_per_move(self):
        from pymycobot.ultraArmP340 import ultraArmP340
        from pymycobot.error import calibration_parameters
        c = fake_control()
        coords, frames = [0,150,100,27], []
        arm = ultraArmP340.__new__(ultraArmP340)
        arm.lock, arm.debug, arm.calibration_parameters = threading.Lock(), False, calibration_parameters
        arm._serial_port, arm._respone = Mock(), Mock()
        arm.get_coords_info = Mock(side_effect=lambda: list(coords))
        c.arm, c.read_arm = arm, Mock()

        def write(frame):
            frames.append(frame)
            if frame.startswith(b'G0 '):
                values = {part[0]: float(part[1:]) for part in frame.decode().split()[1:]}
                coords[:] = [values[axis] for axis in 'XYZE']

        arm._serial_port.write.side_effect = write
        packet = dict(type='grasp_test', item_id='cup', action='grasp', grasp_z_mm=-62.28)
        with patch('robot_control.wait_arm', return_value=[90,10,20]), \
             patch('robot_control.arm_deadline', return_value=nullcontext()), \
             patch('robot_control.time.sleep'), patch('robot_control.atomic_json'):
            c.execute(packet)
            self.assertEqual(frames, [b'M25 A100 F500\r', b'G90\r',
                b'G0 X0 Y150 Z-62.28 E27 F30\r', b'M25 A0 F500\r', b'G90\r',
                b'G0 X0.0 Y150.0 Z100.0 E27.0 F30\r'])
            frames.clear()
            c.execute(dict(packet, action='place'))
            self.assertEqual(frames, [b'G90\r', b'G0 X0.0 Y150.0 Z-62.28 E27.0 F30\r',
                b'M25 A100 F500\r', b'G90\r', b'G0 X0.0 Y150.0 Z100.0 E27.0 F30\r'])

    def test_grasp_holds_until_separate_place_even_after_config_reload(self):
        c = fake_control()
        coords, actions = [0, 150, 100, 27], []
        c.arm.get_coords_info.side_effect = lambda: list(coords)
        c.read_arm = Mock()
        c.localized = False

        def move(target, speed):
            self.assertEqual(target[:2], [0, 150])
            self.assertEqual(target[3], 27)
            coords[:] = target
            actions.append(('z', target[2]))

        c.arm.set_coords.side_effect = move
        c.grip = Mock(side_effect=lambda value: actions.append(('grip', value)))
        with tempfile.TemporaryDirectory() as folder, patch('robot_control.wait_arm', return_value=[90, 10, 20]), \
             patch('robot_control.arm_deadline', return_value=nullcontext()):
            c.config_path = Path(folder) / 'robot_config.json'
            packet = dict(type='grasp_test', item_id='cup', action='grasp', grasp_z_mm=12.5)
            self.assertEqual(c.execute(packet), {})
            self.assertEqual(actions, [('grip', 100), ('z', 12.5), ('grip', 0), ('z', 100)])
            self.assertEqual(c.snapshot()['grasp_test_item_id'], 'cup')
            self.assertTrue(c.snapshot()['grasp_test_toggle_available'])
            self.assertEqual(coords, [0, 150, 100, 27])
            c.config = json.loads(c.config_path.read_text())
            actions.clear()
            self.assertEqual(c.execute(dict(packet, action='place', grasp_z_mm=999)), {})
            self.assertEqual(actions, [('z', 12.5), ('grip', 100), ('z', 100)])
            self.assertIsNone(c.snapshot()['grasp_test_item_id'])
            self.assertNotIn('grasp_test_hold', json.loads(c.config_path.read_text()))
        self.assertEqual(coords, [0, 150, 100, 27])
        self.assertFalse(c.base_enabled)
        c.capture.assert_not_called()
        c.robot.nav.go_to.assert_not_called()
        c.arm.set_angles.assert_not_called()

    def test_full_z_travel_checks_guard_without_issuing_intermediate_targets(self):
        c = fake_control()
        c.arm.get_coords_info.side_effect = [[0,150,100,27], [0,150,100,27], [0,150,50,27]]
        guard = Mock(side_effect=[None, None, None, Stopped('Map changed during travel')])
        with patch('robot_control.arm_deadline', return_value=nullcontext()), \
             patch('robot_control.time.sleep'), self.assertRaises(Stopped):
            c.move_pickup_z(-62.28, guard)
        c.arm.set_coords.assert_called_once_with([0,150,-62.28,27], 30)
        c.arm.set_gripper_state.assert_not_called()
        c.arm.set_gripper_release.assert_not_called()

    def test_grasp_place_rejects_invalid_heights_feedback_and_pose_before_gripping(self):
        for target, feedback, angles, error in ((None, [0,150,35], [90,10,20], ValueError),
                (True, [0,150,35], [90,10,20], ValueError),
                (float('nan'), [0,150,35], [90,10,20], ValueError),
                (LIMITS['Z'][0], [0,150,35], [90,10,20], ValueError),
                (35, [0,150,35], [90,10,20], ValueError),
                (40, [0,150,35], [90,10,20], ValueError),
                (12.5, None, [90,10,20], RuntimeError),
                (12.5, [0,150,float('nan')], [90,10,20], ValueError),
                (12.5, [150,0,35], [90,10,20], ValueError),
                (12.5, [0,150,35], [110,10,20], ValueError)):
            with self.subTest(target=target, feedback=feedback, angles=angles):
                c = fake_control()
                c.arm.get_coords_info.return_value = feedback
                c.grip = Mock()
                with patch('robot_control.wait_arm', return_value=angles), \
                     patch('robot_control.arm_deadline', return_value=nullcontext()), self.assertRaises(error):
                    c.execute(dict(type='grasp_test', item_id='cup', action='grasp', grasp_z_mm=target))
                c.grip.assert_not_called()
                c.arm.set_coords.assert_not_called()

    def test_grasp_place_stops_without_release_or_return_motion(self):
        for failure in ('stop', 'holding', 'lease', 'sensors', 'map', 'zones', 'unreached'):
            with self.subTest(failure=failure):
                c = fake_control()
                c.arm.get_coords_info.return_value = [0,150,35]
                c.arm_step = Mock()

                def interrupt(value):
                    if failure == 'stop': c.stop_pending = 'stop'
                    if failure == 'holding' and value == 0: c.stop_pending = 'stop'
                    if failure == 'lease': c.deadline = 0
                    if failure == 'sensors': c.robot.sensors_ready = lambda: False
                    if failure == 'map': c.robot.grid['map_id'] = 'changed'
                    if failure == 'zones': c.robot.zones['revision'] += 1

                c.grip = Mock(side_effect=interrupt)
                if failure == 'holding':
                    c.arm_step.side_effect = lambda axis, direction, **kwargs: setattr(c.arm.get_coords_info,
                        'return_value', [0,150,c.arm.get_coords_info.return_value[2] + direction * kwargs['distance']])
                error = Stopped if failure in ('stop', 'holding', 'lease') else TimeoutError if failure == 'unreached' else Exception
                with patch('robot_control.wait_arm', return_value=[90,10,20]), \
                     patch('robot_control.arm_deadline', return_value=nullcontext()), \
                     patch('robot_control.atomic_json'), self.assertRaises(error):
                    c.execute(dict(type='grasp_test', item_id='cup', action='grasp', grasp_z_mm=12.5))
                if failure == 'holding':
                    self.assertEqual(c.grip.call_args_list, [call(100), call(0)])
                    self.assertEqual(c.arm.get_coords_info.return_value, [0,150,12.5])
                else:
                    c.grip.assert_called_once_with(100)
                    if failure != 'unreached': c.arm_step.assert_not_called()
                self.assertFalse(c.base_enabled)
                c.robot.zero.assert_called()

    def test_grasp_place_refuses_stale_actions_and_changed_pickup_position(self):
        hold = dict(item_id='cup', grasp_z_mm=12.5, coords=[0,150,35], map_id='map', zone_revision=2)
        for action, item_id, stored, feedback in (('place', 'cup', None, [0,150,35]),
                ('grasp', 'cup', hold, [0,150,35]), ('place', 'other', hold, [0,150,35]),
                ('place', 'cup', {**hold, 'map_id':'old'}, [0,150,35]),
                ('place', 'cup', {**hold, 'zone_revision':1}, [0,150,35]),
                ('place', 'cup', hold, [2,150,35]), (None, 'cup', None, [0,150,35])):
            with self.subTest(action=action, item_id=item_id, stored=stored, feedback=feedback):
                c = fake_control()
                c.config['grasp_test_hold'] = stored
                c.arm.get_coords_info.return_value = feedback
                c.grip = Mock()
                with patch('robot_control.wait_arm', return_value=[90,10,20]), \
                     patch('robot_control.arm_deadline', return_value=nullcontext()), self.assertRaises(ValueError):
                    c.execute(dict(type='grasp_test', action=action, item_id=item_id, grasp_z_mm=12.5))
                c.grip.assert_not_called()
                c.arm.set_coords.assert_not_called()

        c = fake_control()
        c.config['grasp_test_hold'] = hold
        c.grip = Mock()
        with self.assertRaisesRegex(ValueError, 'Place the test item'):
            c.execute(dict(type='fetch_test'))
        with patch('robot_control.atomic_json'):
            c.execute(dict(type='gripper', value=100))
        c.grip.assert_called_once_with(100)
        self.assertNotIn('grasp_test_hold', c.config)

    async def test_grasp_place_api_uses_saved_height_without_llm_and_rejects_stale_selection(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bridge, ui = root / 'ros', root / 'ui'
            grid = {**demo_map(), 'map_id': 'test-map'}
            atomic_json(bridge / 'map.json', grid)
            index = PhotoIndex(ui, root / 'stations.json')
            item_ids = []
            for name in ('Cup', 'Bottle'):
                photo = index.add(demo_frame('front'), dict(kind='reference', source='phone'), index.value['revision'])
                index.edit(photo['id'], dict(expected_revision=index.value['revision'], item_id='new', name=name, confirmed=True), grid['map_id'])
                item_ids.append(index.value['items'][-1]['id'])
            index.save_grasp_height(item_ids[0], 12.5, index.value['revision'])
            with patch('web_backend.server.load_key', return_value=('not_configured', '')), \
                 patch('web_backend.server.load_token', return_value=('not_configured', '')):
                app = create_app(ui, root / 'stations.json', hardware_dir=bridge)
                hardware = app[EDITOR].hardware
                hardware.refresh = Mock()
                hardware.control = dict(phase='idle', stamp_s=time.time(), metric_alignment_available=True, grasp_test_toggle_available=True)
                hardware.command = AsyncMock(return_value=dict(status='accepted'))
                async with TestClient(TestServer(app)) as client:
                    command = dict(id=str(uuid.uuid4()), type='grasp_test', action='grasp', map_id=grid['map_id'],
                        expected_revision=app[EDITOR].zones['revision'], item_id=item_ids[0],
                        index_revision=index.value['revision'], grasp_z_mm=999)
                    self.assertEqual((await client.post('/api/commands', json=command)).status, 200)
                    self.assertEqual(hardware.command.call_args.args[0]['grasp_z_mm'], 12.5)
                    hardware.command.reset_mock()
                    for changes in (dict(index_revision=0), dict(item_id='missing'), dict(item_id=item_ids[1]), dict(action=None)):
                        self.assertEqual((await client.post('/api/commands', json={**command, **changes})).status, 400)
                    hardware.control['grasp_test_toggle_available'] = False
                    self.assertEqual((await client.post('/api/commands', json=command)).status, 400)
                    hardware.command.assert_not_called()

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
        c.localized = False
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

    def test_local_rotation_keeps_safety_checks_without_pose_confirmation(self):
        c = fake_control()
        c.localized = False
        with self.assertRaisesRegex(RuntimeError, 'confirm localization'):
            c.travel_guard({})
        c.robot.nav.get_odom_pose.side_effect = [dict(x_m=0, y_m=0, yaw_deg=0),
            dict(x_m=0, y_m=0, yaw_deg=0), dict(x_m=0, y_m=0, yaw_deg=-2)]
        with patch('robot_control.time.sleep'):
            c.turn_fetch(2, local_alignment=True)
        self.assertEqual(c.robot.velocity[:2], (0, 0))
        self.assertLess(c.robot.velocity[2], 0)
        c.robot.exclusive = False
        with self.assertRaisesRegex(RuntimeError, 'exclusive output'):
            c.travel_guard({}, local_alignment=True)
        self.assertFalse(c.localized)

    def test_local_camera_still_requires_stopped_hardware(self):
        source = ast.parse(Path('ros_web_bridge.py').read_text())
        main = next(node for node in source.body if isinstance(node, ast.FunctionDef) and node.name == 'main')
        capture = next(node for node in main.body if isinstance(node, ast.FunctionDef) and node.name == 'capture')
        namespace = dict(control=SimpleNamespace(localized=False), robot=SimpleNamespace(grid={'map_id': 'map'},
            nav=SimpleNamespace(wait_stopped=Mock(side_effect=RuntimeError('standstill check')))))
        exec(compile(ast.Module(body=[capture], type_ignores=[]), 'ros_web_bridge.py', 'exec'), namespace)
        with self.assertRaisesRegex(RuntimeError, 'standstill check'):
            namespace['capture']('arm', vision=True)
        with self.assertRaisesRegex(RuntimeError, 'standstill check'):
            namespace['capture']('arm', grasp_goal=True)
        for kwargs in ({}, dict(vision=True, update_key='map-update')):
            with self.assertRaisesRegex(ValueError, 'localization'):
                namespace['capture']('arm', **kwargs)

    def test_arm_goal_z_is_bound_to_stationary_image_acquisition(self):
        source = ast.parse(Path('ros_web_bridge.py').read_text())
        main = next(n for n in source.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
        capture = next(n for n in main.body if isinstance(n, ast.FunctionDef) and n.name == 'capture')
        for teleop, after_z in [(t, z) for t in (False, True) for z in (40.2, 41, float('nan'), None)]:
            with self.subTest(teleop=teleop, after_z=after_z):
                arm = Mock()
                arm.get_coords_info.side_effect = [[0,150,40], [0,150,after_z] if after_z is not None else None]
                saved = Mock()
                namespace = dict(control=SimpleNamespace(localized=True, arm=arm, read_arm=lambda: [90,30,40]),
                    robot=SimpleNamespace(grid=dict(map_id='map'), nav=Mock(), stopped=lambda: True),
                    wait_arm=Mock(return_value=[90,30,40]), arm_deadline=lambda _: nullcontext(),
                    number=number, LIMITS=LIMITS, fresh=fresh, frame_lock=threading.Lock(),
                    time=SimpleNamespace(time=lambda: 100, monotonic=lambda: 0),
                    frames=dict(arm=('pixels', dict(captured_at_s=100, map_id='map'))),
                    cv2=Mock(), vision_image=Mock(return_value='data:image/jpeg;base64,AQ=='),
                    base64=base64, uuid=uuid, directory=Path('unused'), atomic_bytes=Mock(), atomic_json=saved)
                namespace['cv2'].imencode.return_value = (True, Mock(tobytes=lambda: b'png'))
                if teleop:
                    namespace['control'] = SimpleNamespace(localized=False, phase='teleop', arm=None, read_arm=Mock(),
                        teleop_feedback=Mock(side_effect=[dict(angles=[90,30,40], coords=[0,150,40]),
                            dict(angles=[90,30,40], coords=[0,150,after_z] if after_z is not None else None)]))
                exec(compile(ast.Module(body=[capture], type_ignores=[]), 'ros_web_bridge.py', 'exec'), namespace)
                if after_z == 40.2:
                    namespace['capture']('arm', grasp_goal=True, teleop_observation=teleop)
                    self.assertEqual(saved.call_args.args[1]['arm_z_mm'], 40.2)
                    self.assertEqual(saved.call_args.args[1]['arm_angles_deg'], [90,30,40])
                else:
                    with self.assertRaises((ValueError, RuntimeError)):
                        namespace['capture']('arm', grasp_goal=True, teleop_observation=teleop)
                    saved.assert_not_called()
                if teleop:
                    arm.get_coords_info.assert_not_called()
                    namespace['control'].read_arm.assert_not_called()
                    namespace['wait_arm'].assert_not_called()

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
                    {'x_mm': float('nan'), 'turn_deg': 0}, {'x_mm': float('inf'), 'turn_deg': 0},
                    {'x_mm': .05, 'turn_deg': 0}, {'x_mm': 0, 'turn_deg': 0, 'grip': True}):
            with self.assertRaises(ValueError):
                alignment_command(bad, None)
        with self.assertRaises(ValueError):
            calibration_config({'max_step_mm': -1})
        self.assertEqual(alignment_command(dict(x_mm=60,y_mm=-12,turn_deg=0),None),dict(X=60,Y=-12,turn_deg=0))
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
        self.assertEqual(provider.call_args.args[2], 65)
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
        c.arm.get_coords_info.return_value = [0, 150, 30]
        c.capture.return_value = {'image': 'compressed'}
        with tempfile.TemporaryDirectory() as folder:
            c.directory = Path(folder)
            results = iter([dict(x_mm=0, turn_deg=1), dict(x_mm=0, turn_deg=0),
                            dict(x_mm=10, y_mm=-10, turn_deg=0), dict(x_mm=1, y_mm=-1, turn_deg=0), dict(x_mm=0, y_mm=0, turn_deg=0)])
            requests = []
            def respond(path, value):
                requests.append(value)
                atomic_json(path, value)
                result = next(results)
                atomic_json(c.directory / 'fetch_vision_response.json', dict(id=value['id'], result=result,
                    aligned_perfectly=not any(result.values())))
            with patch('robot_control.atomic_json', side_effect=respond):
                c.calibrate_fetch({'vision_item': {'name': 'cup', 'calibration_z_mm': 30}}, None)
            self.assertEqual([r['stage'] for r in requests], ['base', 'base', 'arm', 'arm', 'arm'])
            self.assertEqual(requests[2]['history'], [])
            self.assertEqual(c.capture.call_args_list, [call('front', vision=True)] * 2 + [call('arm', vision=True)] * 3)
            c.turn_fetch.assert_called_once_with(1, calibration_config(None), local_alignment=False)
            self.assertEqual(c.arm_step.call_args_list, [call('X', 1, pickup=True, distance=10,y_mm=-10,guard=ANY),
                call('X', 1, pickup=True, distance=1,y_mm=-1,guard=ANY)])
            self.assertFalse(list(c.directory.iterdir()))
            results = iter([dict(x_mm=1, turn_deg=0)])
            with patch('robot_control.atomic_json', side_effect=respond), self.assertRaisesRegex(ValueError, 'violates active stage'):
                c.calibrate_fetch({'vision_item': {'name': 'cup', 'calibration_z_mm': 30}}, None)
            results = iter([dict(x_mm=0, turn_deg=0), dict(x_mm=0, y_mm=0, turn_deg=1)])
            with patch('robot_control.atomic_json', side_effect=respond), self.assertRaisesRegex(ValueError, 'violates active stage'):
                c.calibrate_fetch({'vision_item': {'name': 'cup', 'calibration_z_mm': 30}}, None)

    def test_goal_z_is_reached_before_any_alignment_image(self):
        for start_z in (10, 65, 39, 39.42, 40.58, 41):
            with self.subTest(start_z=start_z), tempfile.TemporaryDirectory() as folder:
                c = fake_control()
                c.directory = Path(folder)
                coords = [0, 150, start_z]
                c.arm.get_coords_info.side_effect = lambda: list(coords)
                def step(axis, direction, **kwargs):
                    self.assertEqual((axis, kwargs['pickup']), ('Z', True))
                    coords[2] += direction * kwargs['distance']
                c.arm_step = Mock(side_effect=step)
                def capture(*args, **kwargs):
                    self.assertLessEqual(abs(coords[2] - 40), 1)
                    self.assertEqual(coords[:2], [0, 150])
                    raise RuntimeError('height verified before vision')
                c.capture.side_effect = capture
                with self.assertRaisesRegex(RuntimeError, 'height verified before vision'):
                    c.calibrate_fetch(dict(vision_item=dict(calibration_z_mm=40)), None)
                self.assertEqual(c.arm_step.call_count, 1 if start_z in (10, 65) else 0)
                self.assertFalse(list(c.directory.iterdir()))

    def test_missing_bad_or_unreached_goal_z_stops_before_vision(self):
        for target, feedback, error in ((None, [0,150,10], ValueError),
                (True, [0,150,10], ValueError), (float('nan'), [0,150,10], ValueError),
                (-140, [0,150,10], ValueError), (40, None, RuntimeError),
                (40, [0,150,float('nan')], ValueError), (40, [0,150,10], TimeoutError)):
            with self.subTest(target=target, feedback=feedback), tempfile.TemporaryDirectory() as folder:
                c = fake_control()
                c.directory, c.arm_step = Path(folder), Mock()
                c.arm.get_coords_info.return_value = feedback
                with self.assertRaises(error):
                    c.calibrate_fetch(dict(vision_item=dict(calibration_z_mm=target)), None)
                c.capture.assert_not_called()
        with tempfile.TemporaryDirectory() as folder:
            c = fake_control()
            c.directory, c.arm_step = Path(folder), Mock()
            c.stop_pending = 'stop'
            with self.assertRaises(Stopped):
                c.calibrate_fetch(dict(vision_item=dict(calibration_z_mm=40)), None)
            c.arm_step.assert_not_called()
            c.capture.assert_not_called()

    def test_preview_does_not_restore_z(self):
        c = fake_control()
        c.arm_step = Mock()
        c.capture.return_value = dict(image='compressed')
        with tempfile.TemporaryDirectory() as folder:
            c.directory = Path(folder)
            def respond(path, value):
                atomic_json(c.directory / 'fetch_vision_response.json', dict(id=value['id'],
                    result=dict(x_mm=0, turn_deg=0), aligned_perfectly=True))
            with patch('robot_control.atomic_json', side_effect=respond):
                c.calibrate_fetch(dict(preview=True, vision_item={}), None)
            c.arm.get_coords_info.assert_not_called()
            c.arm_step.assert_not_called()

    async def test_base_stagnation_requires_five_executed_rounds(self):
        images = dict(front='data:image/jpeg;base64,AQ==')
        previous = dict(images=images, commanded=dict(X=0, turn_deg=1))
        trace = {}
        with patch('web_backend.resolve.request_json', AsyncMock(return_value=dict(front='aligned_stalled'))):
            with self.assertRaisesRegex(ValueError, 'five executed'):
                await locate_pickup(images, dict(name='cup', appearance='red'), 'key', images['front'], [previous] * 4)
            moves = await locate_pickup(images, dict(name='cup', appearance='red'), 'key', images['front'], [previous] * 5, trace=trace)
        self.assertEqual(moves, dict(x_mm=0, turn_deg=0))
        self.assertTrue(trace['aligned_perfectly'])
        self.assertIn('aligned_stalled', trace['result_json'])

    async def test_arm_goal_labels_and_y_direction(self):
        images = dict(arm='data:image/jpeg;base64,AQ==')
        trace = {}
        for box, x, y in [([160, 750, 460, 1000], -10, -5), ([360, 865, 640, 1000], 0, .25), ([360, 840, 640, 1000], 0, 0)]:
            with patch('web_backend.resolve.request_json', AsyncMock(return_value=dict(goal=[360, 840, 640, 1000], current=box))):
                moves = await locate_pickup(images, dict(name='cup', appearance='red'), 'key', images['arm'], [],
                    stage='arm', goal=images['arm'], trace=trace, prompt='Previously saved prompt')
            self.assertEqual(moves, dict(x_mm=x, y_mm=y, turn_deg=0))
            self.assertEqual(trace['aligned_perfectly'], x == y == 0)
            self.assertEqual(trace['stage'], 'arm')
            instruction = trace['messages'][0]['content']
            self.assertTrue(instruction.startswith('Previously saved prompt'))
            self.assertIn('camera is mounted on the gripper', instruction)
            self.assertIn('x=500, bottom=1000', instruction)
            self.assertTrue(any('GOAL:' in p.get('text', '') for p in trace['messages'][1]['content']))
            self.assertEqual(trace['measurement']['current'], box)
            self.assertEqual([v['bounds'] for v in trace['measured_images']], [[360,840,640,1000], box])
        for bad in (dict(x='aligned_perfectly', y='aligned_perfectly'),
                    dict(goal=[420,840,700,1000], current=None),
                    dict(goal=[420,840,700,1000], current=[True,840,700,1000]),
                    dict(goal=[420,840,700,1000], current=[480,840,640,1000])):
            with patch('web_backend.resolve.request_json', AsyncMock(return_value=bad)), self.assertRaises(ValueError):
                await locate_pickup(images, dict(name='cup', appearance='red'), 'key', images['arm'], [], stage='arm', goal=images['arm'])

    def test_generic_bounds_keep_tight_top_tolerance_and_measured_direction(self):
        goal = [350,850,650,1000]
        for top, y in ((845,0), (855,0), (844,-.25), (856,.25), (827,-.25), (780,-5)):
            with self.subTest(top=top):
                moves = measured_arm_alignment(dict(goal=goal, current=[350,top,650,1000]), None)
                self.assertEqual(moves, dict(x_mm=0, y_mm=y, turn_deg=0))
        self.assertEqual(measured_arm_alignment(dict(goal=[350,400,650,600], current=[350,376,650,576]), None)['y_mm'], -.25)
        self.assertEqual(measured_arm_alignment(dict(goal=goal, current=[375,850,675,1000]), None)['x_mm'], .25)
        for current in ([300,850,700,1000], [350,850,650,940], None, [350,True,650,1000]):
            with self.subTest(current=current), self.assertRaises(ValueError):
                measured_arm_alignment(dict(goal=goal, current=current), None)

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
                    base_pose=dict(x_m=0, y_m=0, yaw_rad=0), **({'arm_z_mm': 35} if camera == 'arm' else {})), index.value['revision'])
                index.image_path(photo['id']).with_suffix('.jpg').write_bytes(image)
                cameras[camera] = photo['id']
            index.save_grasp_goal(item_id, cameras, index.value['revision'], grid['map_id'])
            entered, release = asyncio.Event(), asyncio.Event()
            provider_result = dict(front='left_large')
            fast_requests = []

            async def provider(body, key, timeout, trace=None, fast_mode=True):
                fast_requests.append(fast_mode)
                entered.set()
                await release.wait()
                return provider_result

            with patch('web_backend.server.load_key', return_value=('not_configured', '')), \
                 patch('web_backend.server.load_token', return_value=('configured', 'private-test-key')), \
                 patch('web_backend.resolve.request_json', side_effect=provider):
                app = create_app(ui, root / 'stations.json', hardware_dir=bridge)
                hardware = app[EDITOR].hardware
                hardware.refresh = Mock()
                hardware.control = dict(phase='idle', stamp_s=time.time(), metric_alignment_available=True)
                async with TestClient(TestServer(app)) as client:
                    from web_backend.codex_bridge import MODEL as CODEX_MODEL
                    preferences = await (await client.get('/api/settings')).json()
                    response = await client.put('/api/settings', json=dict(expected_revision=preferences['revision'],
                        values={**preferences['values'], 'llm_model': CODEX_MODEL, 'reasoning_effort': 'low'}))
                    self.assertEqual(response.status, 200)
                    settings = await (await client.get('/api/fetch-settings')).json()
                    values = {**settings['values'], 'max_step_mm': 0, 'prompt': 'Edited prompt'}
                    self.assertEqual((await client.put('/api/fetch-settings', json=dict(expected_revision=0, values={**values, 'max_step_mm': -1}))).status, 400)
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
                    self.assertEqual(forwarded['vision_item']['calibration_z_mm'], 35)
                    self.assertNotIn('blue_cap_goal', forwarded['vision_item'])
                    self.assertNotIn('goal', forwarded['vision_item'])
                    self.assertNotIn('station', forwarded)
                    command['fetch_revision'] = 0
                    self.assertEqual((await client.post('/api/commands', json=command)).status, 400)
                    hardware.control.update(phase='calibrating', boot_id='boot', task_id='task', stop_epoch=0)
                    self.assertEqual((await client.put('/api/fetch-settings', json=dict(expected_revision=1, values=values))).status, 409)
                    images = dict(front='data:image/jpeg;base64,AQ==', arm='data:image/jpeg;base64,Ag==')
                    value = dict(id='request', task_id='task', boot_id='boot', stop_epoch=0, map_id=grid['map_id'],
                        expires_at_s=time.time()+20, round=1, preview=True, prompt=values['prompt'], limits=values,
                        images=images, arm_position_mm=[0,150,35], history=[], item=dict(name='Cup', appearance='Red', calibration_z_mm=35, reference_photo_id=reference['id']))
                    atomic_json(bridge / 'fetch_vision_request.json', value)
                    try:
                        await asyncio.wait_for(entered.wait(), 2)
                        log = await (await client.get('/api/fetch-log')).json()
                        self.assertEqual(log['rounds'][0]['status'], 'requesting')
                        self.assertEqual(len([p for p in log['rounds'][0]['messages'][1]['content'] if p['type'] == 'image_url']), 2)
                        self.assertNotIn('private-test-key', json.dumps(log))
                        preferences = await (await client.get('/api/settings')).json()
                        response = await client.put('/api/settings', json=dict(expected_revision=preferences['revision'],
                            values={**preferences['values'], 'fast_mode': False}))
                        self.assertEqual(response.status, 200)
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
                    provider_result = dict(goal=[350,850,650,1000], current=[350,844,650,1000])
                    arm_request = dict(value, id='arm-request', stage='arm', round=2,
                        item=dict(value['item'], arm_goal_photo_id=cameras['arm']))
                    atomic_json(bridge / 'fetch_vision_request.json', arm_request)
                    await asyncio.wait_for(entered.wait(), 2)
                    for _ in range(40):
                        log = await (await client.get('/api/fetch-log')).json()
                        if log['rounds'][-1]['status'] != 'requesting':
                            break
                        await asyncio.sleep(.025)
                    entry = log['rounds'][-1]
                    self.assertEqual(entry['status'], 'returned')
                    self.assertEqual(entry['model'], CODEX_MODEL)
                    self.assertEqual(entry['measurement'], provider_result)
                    self.assertEqual(entry['correction'], dict(X=5, Y=0, turn_deg=0))
                    self.assertEqual([v['label'] for v in entry['measured_images']], ['GOAL','CURRENT'])
                    self.assertEqual(entry['measured_images'][0]['image'], index.reference_image(cameras['arm']))
                    async def arm_round(request, output):
                        nonlocal provider_result
                        entered.clear()
                        provider_result = output
                        atomic_json(bridge / 'fetch_vision_request.json', request)
                        await asyncio.wait_for(entered.wait(),2)
                        for _ in range(40):
                            latest = await (await client.get('/api/fetch-log')).json()
                            if latest['rounds'][-1]['status'] != 'requesting':
                                self.assertEqual(latest['rounds'][-1]['status'],'returned')
                                return latest['rounds'][-1]
                            await asyncio.sleep(.025)
                        self.fail('Metric round did not finish')
                    previous = dict(images=images,commanded=dict(X=5,Y=0,turn_deg=0))
                    await arm_round(dict(arm_request,id='probe-x',round=3,history=[previous],arm_position_mm=[0,155,35]),
                        dict(goal=[350,850,650,1000],current=[310,844,610,1000]))
                    await arm_round(dict(arm_request,id='probe-y',round=4,history=[previous,previous],arm_position_mm=[5,155,35]),
                        dict(goal=[350,850,650,1000],current=[310,894,610,1000]))
                    await arm_round(dict(arm_request,id='metric-done',round=5,history=[previous,previous],arm_position_mm=[.6,150,35]),
                        dict(x_mm=0,y_mm=0,aligned=True))
                    cached = json.loads((ui/'arm_metric_calibrations.json').read_text())
                    self.assertIn(json.dumps([cameras['arm'],35]),cached)
                    hardware.control['task_id'] = 'warm-task'
                    warm = await arm_round(dict(arm_request,id='warm-start',task_id='warm-task',round=1),
                        dict(x_mm=35,y_mm=-10,aligned=False))
                    self.assertTrue(warm['calibration_reused'])
                    self.assertEqual(warm['correction'],dict(X=35,Y=-10,turn_deg=0))
                    self.assertEqual(fast_requests[:5], [True] * 5)
                    self.assertFalse(fast_requests[5])
                    hardware.control['task_id'] = 'task'
                    entered.clear()
                    release.clear()
                    provider_result = dict(front='left_large')
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
                hardware = restarted[EDITOR].hardware
                hardware.refresh = Mock()
                hardware.control = dict(phase='calibrating',boot_id='boot',task_id='restart-task',stop_epoch=0,stamp_s=time.time())
                async with TestClient(TestServer(restarted)) as client:
                    self.assertEqual(await (await client.get('/api/fetch-settings')).json(), saved)
                    self.assertEqual((await (await client.get('/api/fetch-log')).json())['rounds'], [])
                    warm = await arm_round(dict(arm_request,id='restart-warm',task_id='restart-task',round=1),
                        dict(x_mm=35,y_mm=-10,aligned=False))
                    self.assertTrue(warm['calibration_reused'])
                    self.assertEqual(warm['correction'],dict(X=35,Y=-10,turn_deg=0))


if __name__ == '__main__':
    unittest.main()
