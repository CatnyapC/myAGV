"""Map-guided fetch composition, complete arm targets and trusted item calibration."""
from contextlib import nullcontext
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
from web_backend.item_locations import DEFAULT_CONFIG, catalog_signature
from web_backend.photos import PhotoIndex, demo_frame
from web_backend.server import EDITOR, create_app
from web_backend.storage import atomic_json


def fetch_packet():
    return dict(type='fetch', item_id='cup', pickup_angles=[90, 20, 30, 27], grasp_z_mm=12.5,
        vision_item=dict(id='cup', calibration_z_mm=35), fetch_settings={}, location_config=DEFAULT_CONFIG,
        estimate=dict(x_m=5, y_m=3, confidence=.9, uncertainty_m=.35,
                      base_pose=dict(x_m=3.2, y_m=2.7, yaw_rad=.4)))


class MapFetchTest(unittest.TestCase):
    def test_fetch_composes_approach_align_grasp_return_and_place_with_full_targets(self):
        c = fake_control()
        c.config['transport_angles'] = [0, 10, 20, 27]
        c.robot.grid, c.robot.zones = open_map(), dict(revision=2, zones=[])
        c.robot.costmaps = {'global': (time.monotonic(), 2, True, c.robot.grid)}
        start = dict(x_m=2, y_m=3, yaw_deg=0)
        pose, angles, coords, events = dict(start), [0, 10, 20, 27], [20, 80, 100, 27], []
        c.robot.nav.get_pose.side_effect = lambda **kw: dict(pose)
        c.read_arm = Mock(side_effect=lambda: list(angles))
        c.arm.get_coords_info.side_effect = lambda: list(coords)

        def joints(target, speed):
            events.append(('joints', list(target)))
            angles[:] = target
            coords[:] = [20, 80, 100, 27] if target[0] == 0 else [0, 150, 35, 27] if target[1] == 20 else [1, 160, 35, 27]

        def coordinates(target, speed):
            self.assertEqual(target[3], 27)
            events.append(('coords', list(target)))
            coords[:] = target

        def go(goal, **kwargs):
            events.append(('go', dict(goal)))
            self.assertTrue(kwargs['precise'])
            pose.update(goal)

        def align(packet, config):
            self.assertFalse(packet['preview'])
            events.append(('align',))
            angles[:] = [90, 21, 30, 27]
            coords[:] = [1, 160, 35, 27]

        c.arm.set_angles.side_effect, c.arm.set_coords.side_effect = joints, coordinates
        c.grip = Mock(side_effect=lambda value: events.append(('grip', value)))
        c.go = Mock(side_effect=go)
        def approach(packet):
            self.assertEqual(pose, dict(x_m=3.2, y_m=2.7, yaw_deg=math.degrees(.4)))
            events.append(('approach',))
            return dict(outcome='arrived')
        c.approach_item = Mock(side_effect=approach)
        c.calibrate_fetch = Mock(side_effect=align)
        with tempfile.TemporaryDirectory() as folder, patch('robot_control.arm_deadline', return_value=nullcontext()), \
             patch('robot_control.wait_arm', side_effect=lambda *a, **kw: list(angles)):
            c.config_path = Path(folder) / 'robot_config.json'
            c.execute(dict(fetch_packet(), preview=True))
            self.assertNotIn('grasp_test_hold', json.loads(c.config_path.read_text()))
        self.assertEqual([e[0] for e in events], ['joints', 'go', 'approach', 'grip', 'joints', 'align',
            'grip', 'coords', 'grip', 'coords', 'joints', 'go', 'joints', 'coords', 'grip', 'coords', 'joints'])
        self.assertEqual([e[1] for e in events if e[0] == 'joints'],
            [c.config['transport_angles'], [90, 20, 30, 27], c.config['transport_angles'], [90, 21, 30, 27], c.config['transport_angles']])
        self.assertEqual([e[1] for e in events if e[0] == 'coords'],
            [[1, 160, 12.5, 27], [1, 160, 35, 27], [1, 160, 12.5, 27], [1, 160, 35, 27]])
        self.assertEqual(c.go.call_args.args[0], start)
        self.assertEqual(c.go.call_args_list[0].args[0],
                         dict(x_m=3.2, y_m=2.7, yaw_deg=math.degrees(.4)))
        self.assertEqual(c.go.call_count, 2)
        self.assertFalse(c.base_enabled)
        c.robot.nav.go_to_pickup.assert_not_called()

    def test_fetch_preflight_and_stage_failures_stop_subsequent_actions(self):
        for failure in ('camera', 'joints', 'height', 'confidence', 'approach', 'stand_off', 'align', 'return', 'stop', 'map'):
            with self.subTest(failure=failure):
                c, packet = fake_control(), fetch_packet()
                c.config['transport_angles'] = [0, 10, 20, 27]
                c.robot.grid, c.robot.zones = open_map(), dict(revision=2, zones=[])
                c.robot.costmaps = {'global': (time.monotonic(), 2, True, c.robot.grid)}
                c.robot.nav.get_pose.return_value = dict(x_m=2, y_m=3, yaw_deg=0)
                c.fold, c.go, c.grip, c.move_arm, c.test_grasp = Mock(), Mock(), Mock(), Mock(), Mock()
                c.approach_item, c.calibrate_fetch = Mock(return_value=dict(outcome='arrived')), Mock()
                if failure == 'camera': c.capture.side_effect = RuntimeError('camera unavailable')
                if failure == 'joints': packet['pickup_angles'] = [90, 20, 30]
                if failure == 'height': packet['grasp_z_mm'] = 35
                if failure == 'confidence': packet['estimate']['confidence'] = .6
                if failure == 'approach': c.approach_item.side_effect = RuntimeError('item lost')
                if failure == 'stand_off': c.approach_item.return_value = dict(outcome='stand_off')
                if failure == 'align': c.calibrate_fetch.side_effect = RuntimeError('alignment failed')
                if failure == 'return': c.go.side_effect = [None, RuntimeError('return blocked')]
                if failure == 'stop': c.approach_item.side_effect = lambda packet: setattr(c, 'stop_pending', 'STOP')
                if failure == 'map': c.approach_item.side_effect = lambda packet: c.robot.zones.update(revision=3)
                with patch('robot_control.wait_arm', return_value=[90, 20, 30, 27]), self.assertRaises((ValueError, RuntimeError, Stopped)):
                    c.fetch(packet)
                if failure in ('camera', 'joints', 'height', 'confidence'):
                    c.fold.assert_not_called()
                    c.go.assert_not_called()
                if failure != 'return': c.test_grasp.assert_not_called()
                else: self.assertEqual([call.args[0]['action'] for call in c.test_grasp.call_args_list], ['grasp'])
                if failure == 'stand_off':
                    c.calibrate_fetch.assert_not_called()
                    c.grip.assert_not_called()
                    c.move_arm.assert_not_called()
                self.assertFalse(c.base_enabled)


class MapFetchAPITest(unittest.IsolatedAsyncioTestCase):
    async def test_fetch_uses_saved_map_location_goal_pose_and_height_without_station(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            ui, bridge, grid = root / 'ui', root / 'ros', open_map()
            atomic_json(bridge / 'map.json', grid)
            index, image = PhotoIndex(ui, root / 'stations.json'), demo_frame('front')
            reference = index.add(image, dict(kind='reference', source='phone'), 0)
            index.edit(reference['id'], dict(expected_revision=1, item_id='new', name='Cup', confirmed=True), grid['map_id'])
            item = index.value['items'][0]
            index.save_grasp_height(item['id'], 12.5, index.value['revision'])
            cameras = {}
            for camera in ('front', 'arm'):
                photo = index.add(image, dict(kind='observation', source='ros', camera_id=camera,
                    map_id=grid['map_id'], map_revision=1, frame='map', captured_at_s=time.time(),
                    base_pose=dict(x_m=2, y_m=3, yaw_rad=0),
                    **(dict(arm_z_mm=35, arm_angles_deg=[90, 20, 30, 27]) if camera == 'arm' else {})), index.value['revision'])
                index.image_path(photo['id']).with_suffix('.jpg').write_bytes(image)
                cameras[camera] = photo['id']
            index.save_grasp_goal(item['id'], cameras, index.value['revision'], grid['map_id'])
            index.set_locations(cameras['front'], dict(status='complete', catalog_signature=catalog_signature(index.value['items']),
                estimates=[dict(item_id=item['id'], x_m=5, y_m=3, uncertainty_m=.35, confidence=.9)]), index.value['revision'])
            observation_pose = dict(x_m=3.2, y_m=2.7, yaw_rad=.4)
            observation = index.add(image, dict(kind='observation', source='ros', camera_id='front',
                map_id=grid['map_id'], map_revision=1, frame='map', captured_at_s=time.time() + 1,
                base_pose=observation_pose), index.value['revision'])
            index.set_locations(observation['id'], dict(status='complete', catalog_signature=catalog_signature(index.value['items']),
                estimates=[dict(item_id=item['id'], x_m=4.5, y_m=3.5, uncertainty_m=.35, confidence=.9)]), index.value['revision'])
            approach = index.add(image, dict(kind='observation', source='ros', camera_id='front',
                map_id=grid['map_id'], map_revision=1, frame='map', captured_at_s=time.time() + 2,
                base_pose=dict(x_m=4.1, y_m=3.4, yaw_rad=.1), update_key='approach:previous-fetch:0'), index.value['revision'])
            index.set_locations(approach['id'], dict(status='complete', catalog_signature=catalog_signature(index.value['items']),
                estimates=[dict(item_id=item['id'], x_m=4.6, y_m=3.6, uncertainty_m=.35, confidence=.9)]), index.value['revision'])
            with patch('web_backend.server.load_key', return_value=('configured', 'private-test-key')):
                app = create_app(ui, root / 'stations.json', hardware_dir=bridge)
                hardware = app[EDITOR].hardware
                hardware.refresh = Mock()
                hardware.control = dict(phase='idle', stamp_s=time.time(), navigation_ready=True,
                                        item_approach_available=True, map_fetch_available=True)
                hardware.cameras = dict(front=dict(stamp_s=time.time()), arm=dict(stamp_s=time.time()))
                hardware.command = AsyncMock(return_value=dict(status='accepted'))
                async with TestClient(TestServer(app)) as client:
                    command = dict(id=str(uuid.uuid4()), type='fetch', map_id=grid['map_id'], expected_revision=0,
                        index_revision=index.value['revision'], item_id=item['id'], pickup_angles=[0, 0, 0],
                        grasp_z_mm=999, preview=True, estimate={}, location_config={})
                    response = await client.post('/api/commands', json=command)
                    self.assertEqual(response.status, 200, await response.text())
                    forwarded = hardware.command.call_args.args[0]
                    self.assertNotIn('station', forwarded)
                    self.assertEqual(forwarded['pickup_angles'], [90, 20, 30, 27])
                    self.assertEqual(forwarded['grasp_z_mm'], 12.5)
                    self.assertFalse(forwarded['preview'])
                    self.assertEqual(forwarded['location_config'], DEFAULT_CONFIG)
                    self.assertEqual(forwarded['estimate']['x_m'], 4.5)
                    self.assertEqual(forwarded['estimate']['base_pose'], observation_pose)
                    self.assertEqual(forwarded['reference_photo_id'], observation['id'])
                    self.assertEqual(forwarded['vision_item']['arm_goal_photo_id'], cameras['arm'])
                    self.assertEqual(forwarded['vision_item']['front_goal_photo_id'], cameras['front'])
                    self.assertEqual(forwarded['vision_item']['reference_photo_id'], reference['id'])
                    snapshot = await (await client.get('/api/items')).json()
                    self.assertEqual(snapshot['locations'][0]['photo_id'], approach['id'])
                    self.assertEqual(snapshot['locations'][0]['x_m'], 4.6)
                    self.assertTrue(snapshot['items'][0]['fetch_available'])
                    self.assertEqual(snapshot['items'][0]['station_status'], 'none')
                    hardware.command.reset_mock()
                    for changes in (dict(index_revision=0), dict(item_id='missing')):
                        self.assertEqual((await client.post('/api/commands', json={**command, **changes})).status, 400)
                    hardware.control['map_fetch_available'] = False
                    self.assertEqual((await client.post('/api/commands', json=command)).status, 400)
                    hardware.control['map_fetch_available'] = True
                    hardware.cameras.pop('arm')
                    self.assertEqual((await client.post('/api/commands', json=command)).status, 400)
                    hardware.cameras['arm'] = dict(stamp_s=time.time())
                    index.image_path(cameras['arm']).with_suffix('.jpg').unlink()
                    self.assertEqual((await client.post('/api/commands', json=command)).status, 400)
                    hardware.command.assert_not_called()
