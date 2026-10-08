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
from web_backend.photos import PhotoIndex, demo_frame
from web_backend.server import EDITOR, create_app
from web_backend.storage import atomic_bytes, atomic_json


ITEM = dict(id=str(uuid.uuid4()), name='Cup', appearance='Red')
POSE = dict(x_m=2, y_m=3, yaw_rad=0)
IMAGE = 'data:image/jpeg;base64,AQ=='


def result(**values):
    return dict(seen=True, arrived=False, forward_m=.2, turn_deg=0, bearing_deg=0,
                distance_m=1.5, uncertainty_m=.35, confidence=.9, **values)


class ApproachTest(unittest.IsolatedAsyncioTestCase):
    def test_bounds_clearance_identity_arrival_and_camera_transform(self):
        grid = open_map()
        proposal = result()
        goal, estimate = approach_step(proposal, dict(base_pose=POSE), grid, [], grid, ITEM, .25, DEFAULT_CONFIG)
        self.assertAlmostEqual(goal['x_m'], 2.2)
        self.assertAlmostEqual(estimate['x_m'], 3.5)
        for change in (dict(seen=False), dict(forward_m=.21), dict(turn_deg=16), dict(forward_m=True),
                       dict(confidence=.69), dict(distance_m=float('nan')), dict(distance_m=.8),
                       dict(arrived=True), dict(forward_m=0), dict(bearing_deg=30), dict(extra='grasp')):
            with self.subTest(change=change), self.assertRaises(ValueError):
                approach_step({**proposal, **change}, dict(base_pose=POSE), grid, [], grid, ITEM, .25, DEFAULT_CONFIG)
        goal, _ = approach_step({**proposal, 'forward_m': 0, 'turn_deg': 15, 'bearing_deg': 30},
                               dict(base_pose=POSE), grid, [], grid, ITEM, .25, DEFAULT_CONFIG)
        self.assertAlmostEqual(goal['yaw_rad'], math.radians(15))
        self.assertIsNone(approach_step({**proposal, 'arrived': True, 'forward_m': 0, 'distance_m': .9},
                                       dict(base_pose=POSE), grid, [], grid, ITEM, .25, DEFAULT_CONFIG)[0])
        for obstacle in (100, -1):
            blocked = deepcopy(grid)
            blocked['cells'][30 * grid['width'] + 23] = obstacle
            with self.assertRaises(ValueError):
                approach_step(proposal, dict(base_pose=POSE), blocked, [], grid, ITEM, .25, DEFAULT_CONFIG)
        dynamic = deepcopy(grid)
        dynamic['cells'][30 * grid['width'] + 22] = 100
        with self.assertRaises(ValueError):
            approach_step(proposal, dict(base_pose=POSE), grid, [], dynamic, ITEM, .25, DEFAULT_CONFIG)
        with self.assertRaises(ValueError):
            approach_step(proposal, dict(base_pose=POSE), grid, [dict(corners=[[2.2, 2.8], [2.4, 3.2]])], grid, ITEM, .25, DEFAULT_CONFIG)
        edge = dict(x_m=7.95, y_m=3, yaw_rad=0)
        with self.assertRaises(ValueError):
            approach_step(proposal, dict(base_pose=edge), grid, [], grid, ITEM, .25, DEFAULT_CONFIG)

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
        c.go = Mock(side_effect=lambda goal, **kw: pose.update(goal))
        packet = dict(location_config=DEFAULT_CONFIG, vision_item=ITEM, estimate={}, reference_photo_id='source', catalog_signature='catalog')
        with tempfile.TemporaryDirectory() as folder:
            c.directory = Path(folder)
            requests = []
            answers = iter([result(), {**result(), 'arrived': True, 'forward_m': 0, 'distance_m': .9}])

            def respond(path, value):
                requests.append(value)
                atomic_json(path, value)
                atomic_json(c.directory / 'fetch_vision_response.json', dict(id=value['id'], result=next(answers)))

            with patch('robot_control.atomic_json', side_effect=respond):
                self.assertEqual(c.execute(dict(type='approach_item', **packet))['rounds'], 2)
            self.assertEqual(c.capture.call_count, 2)
            self.assertEqual(c.go.call_count, 1)
            self.assertTrue(c.go.call_args.kwargs['precise'])
            self.assertEqual(requests[1]['history'][0]['after_pose']['x_m'], 2.2)
            self.assertEqual(requests[1]['history'][0]['commanded'], dict(forward_m=.2, turn_deg=0))
            c.arm.set_coords.assert_not_called()
            c.arm.set_gripper_value.assert_not_called()
            self.assertFalse(list(c.directory.iterdir()))
            requests.clear()
            pose.update(x_m=2)
            c.capture.reset_mock(); c.go.reset_mock()
            answers = iter([result()] * 16)
            with patch('robot_control.atomic_json', side_effect=respond), self.assertRaisesRegex(RuntimeError, '16 rounds'):
                c.approach_item(packet)
            self.assertEqual(len(requests), 16)
            self.assertEqual(c.go.call_count, 15)
            self.assertEqual(len(requests[-1]['history']), 15)
            self.assertFalse(c.base_enabled)
            self.assertFalse(list(c.directory.iterdir()))
            c.go.reset_mock()
            c.stop_pending = 'STOP'
            with self.assertRaises(Stopped):
                c.approach_item(packet)
            c.go.assert_not_called()
            c.stop_pending = None
            c.deadline = time.monotonic() - 1
            with self.assertRaises(Stopped):
                c.approach_item(packet)
            c.deadline = time.monotonic() + 60
            c.robot.sensors_ready = lambda: False
            with self.assertRaisesRegex(RuntimeError, 'fresh sensors'):
                c.approach_item(packet)
            c.robot.sensors_ready = lambda: True
            answers = iter([{**result(), 'seen': False}])
            with patch('robot_control.atomic_json', side_effect=respond), self.assertRaisesRegex(ValueError, 'lost'):
                c.approach_item(packet)
            c.go.assert_not_called()
            answers = iter([result()])

            def changed_pose(path, value):
                respond(path, value)
                pose['x_m'] += .04

            with patch('robot_control.atomic_json', side_effect=changed_pose), self.assertRaisesRegex(RuntimeError, 'pose changed'):
                c.approach_item(packet)
            c.go.assert_not_called()
            answers = iter([result()])

            def changed_map(path, value):
                respond(path, value)
                c.robot.grid = {**c.robot.grid, 'map_id': 'different-map'}

            with patch('robot_control.atomic_json', side_effect=changed_map), self.assertRaisesRegex(ValueError, 'Map or no-go'):
                c.approach_item(packet)
            c.go.assert_not_called()

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
            photo = index.add(image, dict(kind='observation', source='ros', camera_id='front', map_id=grid['map_id'],
                map_revision=1, frame='map', captured_at_s=time.time(), base_pose=POSE), index.value['revision'])
            signature = catalog_signature(index.value['items'])
            index.set_locations(photo['id'], dict(status='complete', catalog_signature=signature,
                estimates=[dict(item_id=item['id'], x_m=4, y_m=3, uncertainty_m=.8, confidence=.9)]), index.value['revision'])
            c = fake_control()
            c.directory, c.task_id, c.deadline = bridge, 'task', time.monotonic() + 60
            c.robot.grid, c.robot.zones = grid, dict(revision=0, zones=[])
            c.robot.costmaps = {'global': (time.monotonic(), 0, True, grid)}
            pose = dict(x_m=2, y_m=3, yaw_deg=0)
            c.robot.nav.get_pose.side_effect = lambda **kw: dict(pose)
            c.go = Mock(side_effect=lambda goal, **kw: pose.update(goal))

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
            cancelling = False

            async def provider(body, key, timeout):
                bodies.append(body)
                if cancelling:
                    entered.set()
                    await release.wait()
                    return result()
                return result() if len(bodies) == 1 else {**result(), 'arrived': True, 'forward_m': 0, 'distance_m': .9}

            with patch('web_backend.server.load_key', return_value=('configured', 'private-key')), patch('web_backend.resolve.request_json', side_effect=provider):
                app = create_app(ui, root / 'stations.json', hardware_dir=bridge)
                hardware = app[EDITOR].hardware
                hardware.refresh = Mock(side_effect=lambda: setattr(hardware, 'control', c.snapshot()))
                hardware.refresh()
                hardware.cameras = dict(front=dict(stamp_s=time.time()))
                hardware.costmap = Mock(return_value=grid)
                hardware.command = AsyncMock(return_value=dict(status='accepted'))
                async with TestClient(TestServer(app)) as client:
                    command = dict(id=str(uuid.uuid4()), type='approach_item', map_id=grid['map_id'], expected_revision=0,
                        index_revision=index.value['revision'], item_id=item['id'], vision_item=dict(id='injected'), location_config={})
                    self.assertEqual((await client.post('/api/commands', json=command)).status, 200)
                    forwarded = hardware.command.call_args.args[0]
                    self.assertEqual(forwarded['vision_item']['id'], item['id'])
                    self.assertEqual(forwarded['location_config'], DEFAULT_CONFIG)
                    self.assertEqual((await client.post('/api/commands', json={**command, 'index_revision': 0})).status, 400)
                    completed = await asyncio.wait_for(asyncio.to_thread(c.approach_item, forwarded), 4)
                    self.assertEqual(completed['rounds'], 2)
                    current = await (await client.get('/api/items')).json()
                    location = current['locations'][0]
                    self.assertAlmostEqual(location['x_m'], 3.1)
                    self.assertEqual(location['base_pose']['x_m'], 2.2)
                    self.assertNotEqual(location['photo_id'], photo['id'])
                    self.assertNotIn('station_link', current['items'][0])
                    self.assertEqual(len([p for p in bodies[1]['messages'][1]['content'] if p['type'] == 'image_url']), 3)
                    context = json.loads(bodies[1]['messages'][1]['content'][0]['text'])
                    self.assertEqual(context['history'][0]['after_pose']['x_m'], 2.2)
                    self.assertEqual(context['history'][0]['commanded']['forward_m'], .2)
                    log = await (await client.get('/api/fetch-log')).json()
                    self.assertEqual([r['mode'] for r in log['rounds']], ['approach', 'approach'])
                    self.assertNotIn('private-key', json.dumps(log))
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
                self.assertAlmostEqual(PhotoIndex(ui, root / 'stations.json').snapshot(grid['map_id'])['locations'][0]['x_m'], 3.1)


if __name__ == '__main__':
    unittest.main()
