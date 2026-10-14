import asyncio
import base64
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import AsyncMock, Mock, patch

import cv2
import numpy as np
from aiohttp.test_utils import TestClient, TestServer

from fetch_calibration import measured_arm_alignment
from web_backend.blue_cap import blue_cap_bounds
from web_backend.map_data import demo_map
from web_backend.photos import PhotoIndex
from web_backend.server import EDITOR, create_app
from web_backend.storage import atomic_json


def cap_image(y=510, second=False, denim=False):
    frame = np.full((480, 640, 3), 180, np.uint8)
    if denim:
        frame[:, :150] = (80, 35, 20)
    cv2.circle(frame, (320, y), 100, (210, 30, 10), -1)
    if second:
        cv2.circle(frame, (550, 300), 65, (210, 30, 10), -1)
    return cv2.imencode('.png', frame)[1].tobytes()


def saved_goal(root):
    grid = {**demo_map(), 'map_id': 'test-map'}
    atomic_json(root / 'ros/map.json', grid)
    index = PhotoIndex(root / 'ui', root / 'stations.json')
    image = cap_image()
    reference = index.add(image, dict(kind='reference', source='phone'), 0)
    index.edit(reference['id'], dict(expected_revision=1, item_id='new', name='Salt', confirmed=True), grid['map_id'])
    item_id = index.value['items'][0]['id']
    cameras = {}
    for camera in ('front', 'arm'):
        photo = index.add(image, dict(kind='observation', source='ros', camera_id=camera,
            map_id=grid['map_id'], map_revision=0, frame='map', captured_at_s=time.time(),
            base_pose=dict(x_m=0, y_m=0, yaw_rad=0)), index.value['revision'])
        index.image_path(photo['id']).with_suffix('.jpg').write_bytes(image)
        cameras[camera] = photo['id']
    index.save_grasp_goal(item_id, cameras, index.value['revision'], grid['map_id'])
    return index, grid, reference, cameras


class BlueCapTest(unittest.TestCase):
    def test_real_pixels_distinguish_full_cap_from_bottom_fragment_and_denim(self):
        goal = blue_cap_bounds(cap_image())
        current = blue_cap_bounds(cap_image(y=250, denim=True))
        self.assertEqual(goal[3], 1000)
        self.assertLess(current[3], 800)
        self.assertLess(current[1], 400)
        self.assertEqual(blue_cap_bounds(cap_image(denim=True)), goal)
        self.assertLess(measured_arm_alignment(dict(goal=goal, current=current), {}, blue_cap=True)['y_mm'], 0)
        self.assertFalse(any(measured_arm_alignment(dict(goal=goal, current=goal), {}, blue_cap=True).values()))

    def test_missing_ambiguous_and_invalid_images_stop(self):
        blank = cv2.imencode('.png', np.full((480, 640, 3), 180, np.uint8))[1].tobytes()
        for image in (blank, cap_image(second=True), b'broken', b''):
            with self.subTest(image_size=len(image)), self.assertRaises(ValueError):
                blue_cap_bounds(image)

    def test_single_frame_tolerance_and_size_guard(self):
        goal = [400, 850, 700, 1000]
        close = [420, 830, 740, 990]
        self.assertFalse(any(measured_arm_alignment(dict(goal=goal, current=close), {}, blue_cap=True).values()))
        self.assertTrue(any(measured_arm_alignment(dict(goal=goal, current=close), {}).values()))
        for current in ([400, 850, 700, 940], [340, 850, 760, 1000]):
            with self.assertRaises(ValueError):
                measured_arm_alignment(dict(goal=goal, current=current), {}, blue_cap=True)

    def test_confirmed_goal_persists_and_recapture_or_deletion_invalidates_it(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            index, grid, _, cameras = saved_goal(root)
            item_id = index.value['items'][0]['id']
            measurement = dict(photo_id=cameras['arm'], bounds=blue_cap_bounds(cap_image()))
            with self.assertRaises(ValueError):
                index.save_blue_cap_goal(item_id, dict(measurement, photo_id=cameras['front']), index.value['revision'])
            index.save_blue_cap_goal(item_id, measurement, index.value['revision'])
            index = PhotoIndex(root / 'ui', root / 'stations.json')
            self.assertEqual(index.value['items'][0]['blue_cap_goal'], measurement)
            index.save_grasp_goal(item_id, cameras, index.value['revision'], grid['map_id'])
            self.assertNotIn('blue_cap_goal', index.value['items'][0])
            index.save_blue_cap_goal(item_id, measurement, index.value['revision'])
            index.delete(cameras['arm'], index.value['revision'])
            self.assertNotIn('blue_cap_goal', index.value['items'][0])


class BlueCapAPITest(unittest.IsolatedAsyncioTestCase):
    async def test_preview_confirmation_staleness_and_local_worker_without_llm(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            index, grid, reference, cameras = saved_goal(root)
            item_id = index.value['items'][0]['id']
            provider = AsyncMock()
            with patch('web_backend.server.load_key', return_value=('configured', 'test-key')), patch('web_backend.resolve.request_json', provider):
                app = create_app(root / 'ui', root / 'stations.json', hardware_dir=root / 'ros')
                hardware = app[EDITOR].hardware
                hardware.refresh = Mock()
                hardware.control = dict(phase='idle', stamp_s=time.time())
                async with TestClient(TestServer(app)) as client:
                    path = f'/api/items/{item_id}/blue-cap-goal'
                    revision = index.value['revision']
                    self.assertEqual((await client.post(path, json=dict(expected_revision=revision, confirmed=True))).status, 400)
                    preview = await (await client.post(path, json=dict(expected_revision=revision))).json()
                    self.assertEqual(preview['photo_id'], cameras['arm'])
                    edited = await (await client.patch(f"/api/photos/{reference['id']}", json=dict(
                        expected_revision=revision, item_id=item_id, name='Salt', confirmed=True))).json()
                    revision = edited['revision']
                    self.assertEqual((await client.post(path, json=dict(expected_revision=revision, confirmed=True))).status, 400)
                    with patch('web_backend.server.blue_cap_bounds', wraps=blue_cap_bounds) as detector:
                        preview = await (await client.post(path, json=dict(expected_revision=revision))).json()
                        confirmed = await (await client.post(path, json=dict(expected_revision=revision, confirmed=True))).json()
                        detector.assert_called_once()
                    blue = confirmed['items'][0]['blue_cap_goal']
                    self.assertEqual(blue['bounds'], preview['bounds'])
                    self.assertEqual(blue['photo_id'], cameras['arm'])
                    hardware.control.update(phase='calibrating', boot_id='boot', task_id='task', stop_epoch=0)
                    value = dict(id='blue-request', task_id='task', boot_id='boot', stop_epoch=0, map_id=grid['map_id'],
                        expires_at_s=time.time() + 30, round=2, stage='arm', images=dict(arm='data:image/jpeg;base64,' +
                        base64.b64encode(cap_image(y=250)).decode()), history=[], limits={}, item=dict(id=item_id,
                        name='Salt', appearance='blue cap', reference_photo_id=reference['id'],
                        arm_goal_photo_id=cameras['arm'], blue_cap_goal=blue))
                    atomic_json(root / 'ros/fetch_vision_request.json', value)
                    for _ in range(80):
                        response_path = root / 'ros/fetch_vision_response.json'
                        if response_path.exists():
                            break
                        await asyncio.sleep(.025)
                    response = json.loads(response_path.read_text())
                    self.assertFalse(response['aligned_perfectly'])
                    self.assertLess(response['result']['y_mm'], 0)
                    provider.assert_not_awaited()
                    log = await (await client.get('/api/fetch-log')).json()
                    entry = log['rounds'][0]
                    self.assertEqual(entry['model'], 'OpenCV blue cap')
                    self.assertEqual(entry['measurement']['goal'], blue['bounds'])
                    self.assertLess(entry['measurement']['current'][3], 800)
                    self.assertEqual(len(entry['measured_images']), 2)
                    self.assertNotIn('messages', entry)
