"""Bounded camera-guided base approach, shared vision IPC and coordinate updates."""
import asyncio
from copy import deepcopy
import json
import math
from pathlib import Path
import tempfile
import time
import unittest
import uuid
from unittest.mock import AsyncMock, Mock, patch

from aiohttp.test_utils import TestClient, TestServer
from robot_control import Stopped
from test_item_locations import open_map
from test_robot_control import fake_control
from web_backend.item_locations import DEFAULT_CONFIG, approach_step, catalog_signature
from web_backend.map_data import local_to_world
from web_backend.simulation import global_costmap
from web_backend.photos import PhotoIndex, demo_frame
from web_backend.server import EDITOR, create_app
from web_backend.storage import atomic_bytes, atomic_json


ITEM = dict(id=str(uuid.uuid4()), name='Cup', appearance='Red', front_goal_photo_id='front-goal')
POSE = dict(x_m=2, y_m=3, yaw_rad=0)
IMAGE = 'data:image/jpeg;base64,AQ=='


def result(**values):
    return dict(seen=True, arrived=False, forward_m=.2, turn_deg=0, bearing_deg=0,
                distance_m=1.5, uncertainty_m=.35, confidence=.9, goal=[300, 0, 700, 1000],
                current=[425, 500, 575, 1000], **values)


class ApproachTest(unittest.IsolatedAsyncioTestCase):
    def test_only_approach_skips_occupied_goal_guard(self):
        c = fake_control()
        c.robot.validate_goal.side_effect = ValueError('Goal clearance intersects an obstacle')
        c.phase = 'approaching_item'
        c.travel_guard(POSE)
        c.robot.validate_goal.assert_not_called()
        for phase in ('navigating', 'calibrating'):
            c.phase = phase
            with self.subTest(phase=phase), self.assertRaisesRegex(ValueError, 'Goal clearance'):
                c.travel_guard(POSE)

    def test_stale_approach_sensors_pause_without_cancelling_task(self):
        c = fake_control()
        c.phase, c.base_enabled, c.task_id = 'approaching_item', True, 'same-task'
        c.robot.ros = Mock(is_shutdown=Mock(return_value=False))
        ready = [False]
        c.robot.sensors_ready = lambda: ready[0]
        c.watchdog()
        self.assertFalse(c.base_enabled)
        self.assertIsNone(c.stop_pending)
        self.assertEqual(c.stop_epoch, 0)
        with patch('robot_control.time.sleep', side_effect=lambda _: ready.__setitem__(0, True)):
            c.travel_guard(POSE)
        self.assertEqual(c.task_id, 'same-task')
        self.assertEqual(c.phase, 'approaching_item')
        self.assertEqual(c.stop_epoch, 0)
        c.robot.nav.wait_stopped.assert_called_with(timeout=1)
        ready[0] = False
        with patch('robot_control.time.sleep', side_effect=lambda _: setattr(c, 'stop_pending', 'STOP')):
            with self.assertRaises(Stopped):
                c.wait_vision_sensors()

    def test_bounded_corrections_and_visible_off_center_item_can_finish(self):
        grid = open_map()
        goal, _ = approach_step({**result(), 'forward_m': .25, 'turn_deg': 15, 'bearing_deg': 40},
                                dict(base_pose=POSE), grid, [], ITEM, .25, DEFAULT_CONFIG)
        self.assertAlmostEqual(goal['x_m'], 2 + .25*math.cos(math.radians(15)))
        self.assertAlmostEqual(goal['yaw_rad'], math.radians(15))
        for forward in (.25001, .5, .63):
            with self.subTest(forward=forward):
                clamped = approach_step({**result(), 'forward_m': forward}, dict(base_pose=POSE), grid, [], ITEM, .25, DEFAULT_CONFIG)[0]
                self.assertAlmostEqual(clamped['x_m'], 2.25)
        for change in (dict(forward_m=.5, turn_deg=24), dict(turn_deg=15.01), dict(turn_deg=-15.01)):
            with self.subTest(change=change), self.assertRaisesRegex(ValueError, 'exceeds bounds'):
                approach_step({**result(), **change}, dict(base_pose=POSE), grid, [], ITEM, .25, DEFAULT_CONFIG)
        near = {**result(), 'distance_m': .2, 'current': [310, 0, 690, 1000]}
        self.assertIsNone(approach_step({**near, 'forward_m': .5}, dict(base_pose=POSE),
                                       grid, [], ITEM, .25, DEFAULT_CONFIG)[0])
        self.assertIsNone(approach_step({**near, 'forward_m': .005}, dict(base_pose=POSE),
                                       grid, [], ITEM, .25, DEFAULT_CONFIG)[0])
        self.assertIsNone(approach_step({**near, 'forward_m': .001, 'current': near['goal']}, dict(base_pose=POSE),
                                       grid, [], ITEM, .25, DEFAULT_CONFIG)[0])
        matched = {**result(), 'forward_m': 0, 'turn_deg': 0, 'bearing_deg': 60,
                   'current': [20, 0, 420, 1000]}
        self.assertIsNone(approach_step(matched, dict(base_pose=POSE), grid, [], ITEM, .25, DEFAULT_CONFIG)[0])

    def test_visual_match_skips_tiny_final_move_but_small_suggestion_alone_does_not(self):
        grid = open_map()
        near = {**result(), 'forward_m': .009, 'distance_m': .1, 'turn_deg': 15,
                'current': [30, 100, 374, 900]}
        self.assertIsNone(approach_step(near, dict(base_pose=POSE), grid, [], ITEM, .25, DEFAULT_CONFIG)[0])
        for change in (dict(distance_m=.5), dict(current=result()['current']), dict(current=[30, 200, 374, 900])):
            with self.subTest(change=change):
                self.assertIsNotNone(approach_step({**near, **change}, dict(base_pose=POSE), grid, [], ITEM, .25, DEFAULT_CONFIG)[0])

    def test_clipped_views_follow_normal_model_turn_handling(self):
        grid = open_map()
        for goal_box in ([300, 0, 700, 1000], [0, 0, 400, 1000], [600, 0, 1000, 1000]):
            for current, turn in (([0, 0, 200, 1000], 15), ([800, 0, 1000, 1000], -15)):
                with self.subTest(goal=goal_box, current=current):
                    proposal = {**result(), 'goal': goal_box, 'current': current, 'forward_m': 0, 'turn_deg': turn}
                    goal, _ = approach_step(proposal, dict(base_pose=POSE), grid, [], ITEM, .25, DEFAULT_CONFIG)
                    self.assertEqual((goal['x_m'], goal['y_m']), (POSE['x_m'], POSE['y_m']))
                    self.assertAlmostEqual(goal['yaw_rad'], math.radians(turn))

    def test_measured_approach_executes_small_targets_and_stops_on_guard_failures(self):
        for action in ('forward', 'reverse', 'reverse_clamp', 'reverse_obstacle', 'turn', 'clamp', 'drift', 'drift_obstacle', 'heading_drift', 'sensor_pause', 'overshoot', 'stop', 'sensors', 'zones', 'obstacle', 'timeout'):
            with self.subTest(action=action):
                c = fake_control()
                c.robot.grid, c.robot.zones = open_map(), dict(revision=2, zones=[])
                c.robot.costmaps = {}
                c.robot.ros = Mock(is_shutdown=Mock(return_value=False))
                c.phase = 'approaching_item'
                c.robot.validate_goal.side_effect = AssertionError('Approach must not check occupied goals')
                c.robot.velocity = (0, 0, 0)
                c.deadline = .45
                measured, now, commands = dict(x_m=2, y_m=3, yaw_deg=0), [0.], []
                c.robot.nav.get_pose.side_effect = lambda **kw: dict(measured)
                c.robot.nav.get_odom_pose.side_effect = lambda: dict(measured)
                c.robot.zero.side_effect = lambda: setattr(c.robot, 'velocity', (0, 0, 0))
                def tick(seconds):
                    now[0] += seconds
                    c.deadline = now[0] + .45
                    velocity = c.robot.velocity
                    commands.append(velocity)
                    if action.startswith('drift') and len(commands) == 1:
                        measured['y_m'] += .03
                    if action == 'overshoot' and len(commands) == 1:
                        measured['x_m'] += .01
                    if action == 'heading_drift' and len(commands) == 1:
                        measured['yaw_deg'] += 3
                    if action != 'timeout':
                        measured['yaw_deg'] += math.degrees(velocity[2]*seconds)
                        measured['x_m'] += velocity[0]*seconds*math.cos(math.radians(measured['yaw_deg']))
                        measured['y_m'] += velocity[0]*seconds*math.sin(math.radians(measured['yaw_deg']))
                    if action == 'stop' or (action == 'sensors' and now[0] >= .1): c.stop_pending = 'STOP'
                    if action == 'sensors': c.robot.sensors_ready = lambda: False
                    if action == 'sensor_pause': c.robot.sensors_ready = lambda: now[0] >= 10
                    if action == 'zones': c.robot.zones['revision'] += 1
                    if action in ('obstacle', 'drift_obstacle'): c.robot.grid['cells'][30*80+21] = 100
                    if action == 'reverse_obstacle': c.robot.grid['cells'][30*80+19] = 100
                distance = -.63 if action == 'reverse_clamp' else -.1 if action == 'reverse_obstacle' else -.006 if action == 'reverse' else .63 if action == 'clamp' else .25 if action.startswith('drift') else .006
                turn = math.radians(-4 if action.startswith('drift') else 4.5 if action == 'turn' else 0)
                goal = dict(x_m=2 if action == 'turn' else 2+distance*math.cos(turn),
                            y_m=3 if action == 'turn' else 3+distance*math.sin(turn), yaw_rad=turn)
                with patch('robot_control.time.monotonic', side_effect=lambda: now[0]), patch('robot_control.time.sleep', side_effect=tick):
                    if action in ('forward', 'reverse', 'reverse_clamp', 'reverse_obstacle', 'turn', 'clamp', 'drift', 'drift_obstacle', 'heading_drift', 'sensor_pause', 'overshoot', 'obstacle'):
                        c.move_approach(goal, POSE)
                    else:
                        with self.assertRaises((Stopped, RuntimeError, ValueError)):
                            c.move_approach(goal, POSE)
                self.assertTrue(commands)
                if action == 'forward': self.assertGreaterEqual(measured['x_m'], 2.003)
                if action == 'turn': self.assertLess(abs(measured['yaw_deg']-4.5), .31)
                if action == 'clamp': self.assertAlmostEqual(measured['x_m'], 2.25, delta=.0031)
                if action.startswith('reverse'):
                    self.assertAlmostEqual(measured['x_m'], 2+max(distance, -.25), delta=.0031)
                    self.assertTrue(any(v[0] < 0 for v in commands))
                if action == 'drift':
                    remaining = (goal['x_m']-measured['x_m'])*math.cos(turn) + (goal['y_m']-measured['y_m'])*math.sin(turn)
                    self.assertLess(remaining, .0031)
                    self.assertLess(abs(measured['yaw_deg']-math.degrees(turn)), .31)
                    self.assertTrue(any(v[0] > 0 for v in commands))
                if action in ('forward', 'clamp', 'heading_drift'):
                    self.assertTrue(all(v[2] == 0 for v in commands))
                self.assertTrue(all(v[2] == 0 for v in commands if v[0] != 0))
                self.assertTrue(all(abs(v[0]) >= .01 for v in commands if v[0] != 0))
                if action == 'overshoot': self.assertEqual(len(commands), 1)
                self.assertEqual(c.robot.velocity, (0, 0, 0))
                self.assertFalse(c.base_enabled)
                c.robot.nav.go_to.assert_not_called()
                c.arm.set_coords.assert_not_called()
                c.arm.set_gripper_value.assert_not_called()

    def test_executor_rejects_unbounded_targets_without_velocity(self):
        for forward, turn in ((.5, 24), (0, 15.01), (0, -15.01)):
            with self.subTest(forward=forward, turn=turn):
                c = fake_control()
                c.robot.velocity = (0, 0, 0)
                goal = dict(x_m=2+forward, y_m=3, yaw_rad=math.radians(turn))
                with self.assertRaisesRegex(ValueError, 'exceeds bounds'):
                    c.move_approach(goal, POSE)
                self.assertEqual(c.robot.velocity, (0, 0, 0))

    def test_saved_goal_scale_controls_arrival_instead_of_coarse_stand_off(self):
        grid = open_map()
        close = {**result(), 'forward_m': .1, 'bearing_deg': 18, 'distance_m': .7}
        goal, _ = approach_step(close, dict(base_pose=POSE), grid, [], ITEM, .2, DEFAULT_CONFIG)
        self.assertAlmostEqual(goal['x_m'], 2.1)
        matched = {**close, 'forward_m': 0, 'arrived': True, 'current': close['goal']}
        self.assertIsNone(approach_step(matched, dict(base_pose=POSE), grid, [], ITEM, .2, DEFAULT_CONFIG)[0])
        for change, error in ((dict(forward_m=0), 'no useful progress'),
                              (dict(arrived=True, forward_m=0), 'saved front GOAL'),
                              (dict(arrived=True, forward_m=0, current=[300, 200, 700, 1000]), 'saved front GOAL'),
                              (dict(goal=None), 'bounds'), (dict(seen=False), 'lost'),
                              (dict(confidence=.69), 'confidence')):
            with self.subTest(change=change), self.assertRaisesRegex(ValueError, error):
                approach_step({**close, **change}, dict(base_pose=POSE), grid, [], ITEM, .2, DEFAULT_CONFIG)

    def test_too_close_can_reobserve_then_retreat_ignoring_occupancy(self):
        grid = open_map()
        close = {**result(), 'distance_m': .2, 'current': [240, 0, 760, 1000]}
        for forward in (0, .2):
            goal, _ = approach_step({**close, 'forward_m': forward}, dict(base_pose=POSE), grid, [], ITEM, .2, DEFAULT_CONFIG)
            self.assertEqual(goal, POSE)
        for forward in (-.1, -.5):
            goal, _ = approach_step({**close, 'forward_m': forward}, dict(base_pose=POSE), grid, [], ITEM, .2, DEFAULT_CONFIG)
            self.assertAlmostEqual(goal['x_m'], 1.97)
            self.assertEqual(goal['yaw_rad'], 0)
        blocked = deepcopy(grid)
        blocked['cells'][30*80+19] = 100
        goal, _ = approach_step({**close, 'forward_m': -.03}, dict(base_pose=POSE), blocked, [], ITEM, .2, DEFAULT_CONFIG)
        self.assertAlmostEqual(goal['x_m'], 1.97)

    def test_missing_front_goal_is_refused_before_capture_or_motion(self):
        c = fake_control()
        with self.assertRaisesRegex(ValueError, 'final GOAL'):
            c.approach_item(dict(location_config=DEFAULT_CONFIG, estimate={},
                                 vision_item={k: v for k, v in ITEM.items() if k != 'front_goal_photo_id'}))
        c.capture.assert_not_called()
        c.robot.nav.go_to.assert_not_called()

    def test_bounds_clearance_identity_arrival_and_camera_transform(self):
        grid = open_map()
        proposal = result()
        goal, estimate = approach_step(proposal, dict(base_pose=POSE), grid, [], ITEM, .25, DEFAULT_CONFIG)
        self.assertAlmostEqual(goal['x_m'], 2.2)
        self.assertAlmostEqual(estimate['x_m'], 3.5)
        with patch('web_backend.item_locations.global_costmap', wraps=global_costmap) as inflate:
            approach_step(proposal, dict(base_pose=POSE), grid, [], ITEM, .25, DEFAULT_CONFIG)
            self.assertLess(len(inflate.call_args.args[0]['cells']), 200)
        rotated = {**grid, 'origin': dict(x_m=2, y_m=1, yaw_rad=1.1)}
        x, y = local_to_world([2, 3], rotated['origin'])
        rotated_goal, _ = approach_step(proposal, dict(base_pose=dict(x_m=x, y_m=y, yaw_rad=1.1)),
                                       rotated, [], ITEM, .25, DEFAULT_CONFIG)
        expected = local_to_world([2.2, 3], rotated['origin'])
        self.assertAlmostEqual(rotated_goal['x_m'], expected[0])
        self.assertAlmostEqual(rotated_goal['y_m'], expected[1])
        for change in (dict(seen=False), dict(forward_m=-.1), dict(turn_deg=float('inf')), dict(forward_m=True),
                       dict(confidence=.69), dict(distance_m=float('nan')),
                       dict(arrived=True), dict(forward_m=0), dict(extra='grasp')):
            with self.subTest(change=change), self.assertRaises(ValueError):
                approach_step({**proposal, **change}, dict(base_pose=POSE), grid, [], ITEM, .25, DEFAULT_CONFIG)
        goal, _ = approach_step({**proposal, 'forward_m': 0, 'turn_deg': 15, 'bearing_deg': 30},
                               dict(base_pose=POSE), grid, [], ITEM, .25, DEFAULT_CONFIG)
        self.assertAlmostEqual(goal['yaw_rad'], math.radians(15))
        self.assertIsNone(approach_step({**proposal, 'arrived': True, 'forward_m': 0, 'distance_m': .9, 'current': result()['goal']},
                                       dict(base_pose=POSE), grid, [], ITEM, .25, DEFAULT_CONFIG)[0])
        for obstacle in (100, -1):
            blocked = deepcopy(grid)
            for x in (20, 22, 23):
                blocked['cells'][30 * grid['width'] + x] = obstacle
            goal, _ = approach_step(proposal, dict(base_pose=POSE), blocked, [], ITEM, .25, DEFAULT_CONFIG)
            self.assertAlmostEqual(goal['x_m'], 2.2)
        with self.assertRaises(ValueError):
            approach_step(proposal, dict(base_pose=POSE), grid, [dict(corners=[[2.2, 2.8], [2.4, 3.2]])], ITEM, .25, DEFAULT_CONFIG)
        edge = dict(x_m=7.95, y_m=3, yaw_rad=0)
        with self.assertRaises(ValueError):
            approach_step(proposal, dict(base_pose=edge), grid, [], ITEM, .25, DEFAULT_CONFIG)

    def test_hardware_recaptures_and_stops_at_sixteen_without_grasp(self):
        c = fake_control()
        c.robot.grid = open_map()
        c.robot.zones = dict(revision=2, zones=[])
        c.robot.costmaps = {}
        c.robot.costmaps['global'] = (time.monotonic(), 2, True, c.robot.grid)
        c.task_id, c.deadline = 'task', time.monotonic() + 60
        pose = dict(x_m=2, y_m=3, yaw_deg=0)
        c.robot.nav.get_pose.side_effect = lambda **kw: dict(pose)
        c.capture.side_effect = lambda *a, **k: dict(capture_id=str(uuid.uuid4()), image=IMAGE,
            base_pose=dict(x_m=pose['x_m'], y_m=pose['y_m'], yaw_rad=math.radians(pose['yaw_deg'])))
        c.move_approach = Mock(side_effect=lambda goal, source: pose.update(x_m=goal['x_m'], y_m=goal['y_m'], yaw_deg=math.degrees(goal['yaw_rad'])))
        packet = dict(location_config=DEFAULT_CONFIG, vision_item=ITEM, estimate={}, reference_photo_id='source', catalog_signature='catalog')
        with tempfile.TemporaryDirectory() as folder:
            c.directory = Path(folder)
            requests = []
            answers = iter([result(), {**result(), 'arrived': True, 'forward_m': 0, 'distance_m': .9, 'current': result()['goal']}])

            def respond(path, value):
                requests.append(value)
                atomic_json(path, value)
                atomic_json(c.directory / 'fetch_vision_response.json', dict(id=value['id'], result=next(answers)))

            with patch('robot_control.atomic_json', side_effect=respond):
                self.assertEqual(c.execute(dict(type='approach_item', **packet))['rounds'], 2)
            self.assertEqual(c.capture.call_count, 2)
            self.assertEqual(c.move_approach.call_count, 1)
            self.assertEqual(requests[1]['history'][0]['after_pose']['x_m'], 2.2)
            self.assertAlmostEqual(requests[1]['history'][0]['commanded']['forward_m'], .2)
            self.assertAlmostEqual(requests[1]['history'][0]['commanded']['turn_deg'], 0)
            c.arm.set_coords.assert_not_called()
            c.arm.set_gripper_value.assert_not_called()
            self.assertFalse(list(c.directory.iterdir()))
            c.capture.reset_mock(); c.move_approach.reset_mock()
            answers = iter([{**result(), 'forward_m': 0, 'distance_m': .2, 'current': result()['goal']}])
            with patch('robot_control.atomic_json', side_effect=respond):
                stopped = c.approach_item(packet)
            self.assertEqual(stopped['rounds'], 1)
            self.assertEqual(stopped['outcome'], 'arrived')
            c.move_approach.assert_not_called()
            self.assertEqual(c.capture.call_count, 1)
            c.arm.set_coords.assert_not_called()
            c.arm.set_gripper_value.assert_not_called()
            requests.clear()
            pose.update(x_m=2)
            c.capture.reset_mock(); c.move_approach.reset_mock()
            answers = iter([result()] * 16)
            with patch('robot_control.atomic_json', side_effect=respond), self.assertRaisesRegex(RuntimeError, '16 rounds'):
                c.approach_item(packet)
            self.assertEqual(len(requests), 16)
            self.assertEqual(c.move_approach.call_count, 15)
            self.assertEqual(len(requests[-1]['history']), 15)
            self.assertFalse(c.base_enabled)
            self.assertFalse(list(c.directory.iterdir()))
            c.move_approach.reset_mock()
            c.stop_pending = 'STOP'
            with self.assertRaises(Stopped):
                c.approach_item(packet)
            c.move_approach.assert_not_called()
            c.stop_pending = None
            c.deadline = time.monotonic() - 1
            with self.assertRaises(Stopped):
                c.approach_item(packet)
            c.deadline = time.monotonic() + 60
            c.robot.sensors_ready = lambda: False
            c.robot.ros = Mock(is_shutdown=Mock(return_value=False))
            with patch('robot_control.time.sleep', side_effect=lambda _: setattr(c, 'stop_pending', 'STOP')):
                with self.assertRaises(Stopped):
                    c.approach_item(packet)
            c.stop_pending = None
            c.robot.sensors_ready = lambda: True
            answers = iter([{**result(), 'seen': False}])
            with patch('robot_control.atomic_json', side_effect=respond), self.assertRaisesRegex(ValueError, 'lost'):
                c.approach_item(packet)
            c.move_approach.assert_not_called()
            answers = iter([result(), {**result(), 'forward_m': 0, 'current': result()['goal']}])
            requests.clear()

            def changed_pose(path, value):
                respond(path, value)
                if len(requests) == 1:
                    pose['x_m'] += .04

            with patch('robot_control.atomic_json', side_effect=changed_pose):
                self.assertEqual(c.approach_item(packet)['outcome'], 'arrived')
            self.assertEqual(len(requests), 2)
            self.assertAlmostEqual(requests[1]['base_pose']['x_m'], requests[0]['base_pose']['x_m']+.04)
            self.assertEqual(requests[1]['history'][-1]['commanded']['forward_m'], 0)
            c.move_approach.assert_not_called()
            answers = iter([result()])

            def changed_map(path, value):
                respond(path, value)
                c.robot.grid = {**c.robot.grid, 'map_id': 'different-map'}

            with patch('robot_control.atomic_json', side_effect=changed_map), self.assertRaisesRegex(ValueError, 'Map or no-go'):
                c.approach_item(packet)
            c.move_approach.assert_not_called()

    async def test_shared_worker_corrects_map_with_real_acquisition_and_executed_history(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            ui, bridge = root / 'ui', root / 'ros'
            grid = open_map()
            atomic_json(bridge / 'map.json', grid)
            index = PhotoIndex(ui, root / 'stations.json')
            image = demo_frame('front')
            reference = index.add(image, dict(kind='reference', source='phone'), 0)
            index.edit(reference['id'], dict(expected_revision=1, item_id='new', name='Cup', confirmed=True), grid['map_id'])
            item = index.value['items'][0]
            cameras = {}
            for camera in ('front', 'arm'):
                saved = index.add(image, dict(kind='observation', source='ros', camera_id=camera,
                    map_id=grid['map_id'], map_revision=1, frame='map', captured_at_s=time.time(), base_pose=POSE,
                    **(dict(arm_z_mm=35) if camera == 'arm' else {})), index.value['revision'])
                index.image_path(saved['id']).with_suffix('.jpg').write_bytes(image)
                cameras[camera] = saved['id']
            index.save_grasp_goal(item['id'], cameras, index.value['revision'], grid['map_id'])
            photo = index.add(image, dict(kind='observation', source='ros', camera_id='front', map_id=grid['map_id'],
                map_revision=1, frame='map', captured_at_s=time.time(), base_pose=POSE), index.value['revision'])
            signature = catalog_signature(index.value['items'])
            index.set_locations(photo['id'], dict(status='complete', catalog_signature=signature,
                estimates=[dict(item_id=item['id'], x_m=4, y_m=3, uncertainty_m=.8, confidence=.9)]), index.value['revision'])
            c = fake_control()
            c.directory, c.task_id, c.deadline = bridge, 'task', time.monotonic() + 60
            c.robot.grid, c.robot.zones = grid, dict(revision=0, zones=[])
            c.robot.costmaps = {}
            pose = dict(x_m=2, y_m=3, yaw_deg=0)
            c.robot.nav.get_pose.side_effect = lambda **kw: dict(pose)
            c.move_approach = Mock(side_effect=lambda goal, source: pose.update(x_m=goal['x_m'], y_m=goal['y_m'], yaw_deg=math.degrees(goal['yaw_rad'])))

            def capture(*args, **kwargs):
                capture_id = str(uuid.uuid4())
                metadata = dict(kind='observation', source='ros', camera_id='front', map_id=grid['map_id'], map_revision=1,
                    frame='map', captured_at_s=time.time(), capture_id=capture_id, update_key=kwargs['update_key'],
                    base_pose=dict(x_m=pose['x_m'], y_m=pose['y_m'], yaw_rad=math.radians(pose['yaw_deg'])))
                atomic_bytes(bridge / 'captures' / (capture_id + '.png'), image)
                atomic_bytes(bridge / 'captures' / (capture_id + '.jpg'), b'jpeg')
                atomic_json(bridge / 'captures' / (capture_id + '.json'), metadata)
                return dict(image=IMAGE, capture_id=capture_id, base_pose=metadata['base_pose'])

            c.capture.side_effect = capture
            bodies = []
            entered, release = asyncio.Event(), asyncio.Event()
            cancelling = goal_matching = False

            def compact(measurement):
                return {k: measurement[k] for k in ('forward_m', 'turn_deg', 'bearing_deg', 'distance_m', 'goal', 'current')}

            async def provider(body, key, timeout, fast_mode=True, trace=None):
                bodies.append(body)
                if goal_matching:
                    return compact({**result(), 'forward_m': 0, 'distance_m': .2, 'current': result()['goal']})
                if cancelling:
                    entered.set()
                    await release.wait()
                    return compact(result())
                return compact(result() if len(bodies) == 1 else {**result(), 'forward_m': 0, 'distance_m': .9, 'current': result()['goal']})

            CODEX_MODEL = 'codex-local/gpt-6.1-terra'
            atomic_json(ui / 'settings.json', dict(revision=0, values=dict(auto_align=False, manual_angle_deg=0,
                llm_model=CODEX_MODEL, reasoning_effort='low', fast_mode=True)))
            with patch('web_backend.server.load_token', return_value=('configured', 'private-key')), \
                 patch('web_backend.server.close_codex_session', new_callable=AsyncMock) as close_session, \
                 patch('web_backend.resolve.request_json', side_effect=provider):
                app = create_app(ui, root / 'stations.json', hardware_dir=bridge)
                hardware = app[EDITOR].hardware
                hardware.refresh = Mock(side_effect=lambda: setattr(hardware, 'control', c.snapshot()))
                hardware.refresh()
                hardware.cameras = dict(front=dict(stamp_s=time.time()))
                hardware.costmap = Mock(side_effect=AssertionError('Approach must not read the ROS costmap'))
                hardware.command = AsyncMock(return_value=dict(status='accepted'))
                async with TestClient(TestServer(app)) as client:
                    command = dict(id=str(uuid.uuid4()), type='approach_item', map_id=grid['map_id'], expected_revision=0,
                        index_revision=index.value['revision'], item_id=item['id'], vision_item=dict(id='injected'), location_config={})
                    saved_jpeg = index.image_path(cameras['front']).with_suffix('.jpg')
                    saved_jpeg.unlink()
                    missing_goal = await client.post('/api/commands', json=command)
                    self.assertEqual(missing_goal.status, 400)
                    self.assertIn('final GOAL', await missing_goal.text())
                    hardware.command.assert_not_called()
                    saved_jpeg.write_bytes(image)
                    self.assertEqual((await client.post('/api/commands', json=command)).status, 200)
                    forwarded = hardware.command.call_args.args[0]
                    self.assertEqual(forwarded['vision_item']['id'], item['id'])
                    self.assertEqual(forwarded['vision_item']['front_goal_photo_id'], cameras['front'])
                    self.assertEqual(forwarded['location_config'], DEFAULT_CONFIG)
                    self.assertEqual((await client.post('/api/commands', json={**command, 'index_revision': 0})).status, 400)
                    completed = await asyncio.wait_for(asyncio.to_thread(c.approach_item, forwarded), 4)
                    self.assertEqual(completed['rounds'], 2)
                    self.assertIn('FINAL GOAL', bodies[0]['messages'][1]['content'][3]['text'])
                    current = await (await client.get('/api/items')).json()
                    location = current['locations'][0]
                    self.assertAlmostEqual(location['x_m'], 3.1)
                    self.assertEqual(location['base_pose']['x_m'], 2.2)
                    self.assertNotEqual(location['photo_id'], photo['id'])
                    self.assertNotIn('station_link', current['items'][0])
                    self.assertEqual(len([p for p in bodies[1]['messages'][1]['content'] if p['type'] == 'image_url']), 1)
                    context = json.loads(bodies[1]['messages'][1]['content'][0]['text'])
                    self.assertEqual(context['last_executed']['after_pose']['x_m'], 2.2)
                    self.assertAlmostEqual(context['last_executed']['commanded']['forward_m'], .2)
                    self.assertEqual(bodies[0]['session']['id'], bodies[1]['session']['id'])
                    self.assertEqual([b['session']['start'] for b in bodies[:2]], [True, False])
                    self.assertEqual([b['session']['sequence'] for b in bodies[:2]], [1, 2])
                    close_session.assert_not_awaited()
                    fields = bodies[0]['response_format']['json_schema']['schema']['properties']
                    self.assertEqual(len(fields), 6)
                    self.assertEqual(fields['forward_m']['maximum'], .25)
                    self.assertEqual(fields['forward_m']['minimum'], -.25)
                    self.assertEqual(fields['turn_deg']['minimum'], -15)
                    self.assertEqual(fields['turn_deg']['maximum'], 15)
                    prompt = bodies[0]['messages'][0]['content']
                    self.assertIn('-0.25..0.25 meters', prompt)
                    self.assertNotIn('full remaining forward distance', prompt)
                    log = await (await client.get('/api/fetch-log')).json()
                    self.assertEqual([r['mode'] for r in log['rounds']], ['approach', 'approach'])
                    self.assertNotIn('private-key', json.dumps(log))
                    self.assertEqual(len(json.loads(log['rounds'][0]['result_json'])), 6)
                    self.assertAlmostEqual(log['rounds'][-1]['corrected_location']['x_m'], 3.1)
                    self.assertFalse((bridge / 'fetch_vision_response.json').exists())
                    cancelling = True
                    c.task_id, c.phase = 'cancelled-task', 'idle'
                    running = asyncio.create_task(asyncio.to_thread(c.approach_item, forwarded))
                    await asyncio.wait_for(entered.wait(), 2)
                    c.stop_pending, c.phase = 'STOP', 'fault'
                    c.stop_epoch += 1
                    with self.assertRaises(Stopped):
                        await asyncio.wait_for(running, 2)
                    release.set()
                    for _ in range(40):
                        log = await (await client.get('/api/fetch-log')).json()
                        if log['rounds'][0]['status'] != 'requesting':
                            break
                        await asyncio.sleep(.025)
                    self.assertEqual(log['rounds'][0]['status'], 'discarded')
                    current = await (await client.get('/api/items')).json()
                    self.assertAlmostEqual(current['locations'][0]['x_m'], 3.1)
                    self.assertEqual(current['photos'][-1]['location_analysis']['status'], 'cancelled')
                    self.assertFalse((bridge / 'fetch_vision_response.json').exists())
                    cancelling, goal_matching = False, True
                    c.stop_pending, c.phase, c.task_id = None, 'idle', 'goal-matched-task'
                    c.move_approach.reset_mock()
                    stopped = await asyncio.wait_for(asyncio.to_thread(c.approach_item, forwarded), 4)
                    self.assertEqual(stopped['outcome'], 'arrived')
                    self.assertEqual(stopped['rounds'], 1)
                    c.move_approach.assert_not_called()
                    log = await (await client.get('/api/fetch-log')).json()
                    self.assertEqual(log['rounds'][0]['status'], 'returned')
                    self.assertEqual(log['rounds'][0]['outcome'], 'arrived')
                    self.assertNotIn('error', log['rounds'][0])
                    current = await (await client.get('/api/items')).json()
                    self.assertAlmostEqual(current['locations'][0]['x_m'], stopped['estimate']['x_m'])
                self.assertAlmostEqual(PhotoIndex(ui, root / 'stations.json').snapshot(grid['map_id'])['locations'][0]['x_m'], stopped['estimate']['x_m'])


if __name__ == '__main__':
    unittest.main()
