"""Local photo checks: reference/observation separation, binding and persistence."""
from copy import deepcopy
import asyncio
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

from aiohttp.test_utils import TestClient, TestServer
from web_backend.photos import PhotoIndex, demo_frame, validate_png
from web_backend.server import EDITOR, create_app


class PhotoIndexTest(unittest.TestCase):
    def test_association_station_guards_and_atomic_storage(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            stations = root / 'stations.json'
            station = dict(base=dict(x_m=1, y_m=2, yaw_deg=0), arm_angles_deg=[0, 10, 20])
            stations.write_text(json.dumps({'cup': station}))
            index = PhotoIndex(root, stations)
            image = demo_frame('front')
            validate_png(image)
            for bad in (b'not png', image[:-1], image[:40] + bytes([image[40] ^ 1]) + image[41:]):
                with self.assertRaises(ValueError):
                    validate_png(bad)
            reference = index.add(image, dict(kind='reference', source='phone'), 0)
            self.assertFalse({'base_pose', 'map_id', 'captured_at_s'} & reference.keys())
            metadata = dict(kind='observation', source='simulation', camera_id='front', map_id='map-a',
                            map_revision=0, frame='map', captured_at_s=1.5,
                            base_pose=dict(x_m=3, y_m=4, yaw_rad=.5))
            observation = index.add(image, metadata, 1)
            label = dict(expected_revision=2, item_id='new', name='Red cup', appearance='Red with handle',
                         station_name='cup', confirmed=True)
            with self.assertRaisesRegex(ValueError, 'station confirmation'):
                index.edit(reference['id'], label, 'map-a')
            index.edit(reference['id'], {**label, 'station_confirmed': True}, 'map-a')
            item_id = index.value['items'][0]['id']
            self.assertEqual(index.snapshot('map-a')['items'][0]['station_status'], 'ready')
            self.assertEqual(json.loads(stations.read_text()), {'cup': station})
            association = dict(expected_revision=3, item_id=item_id, station_name='cup')
            with self.assertRaisesRegex(ValueError, 'item association'):
                index.edit(observation['id'], association, 'map-a')
            with self.assertRaisesRegex(ValueError, 'another map'):
                index.edit(observation['id'], {**association, 'confirmed': True}, 'map-b')
            index.edit(observation['id'], {**association, 'confirmed': True}, 'map-a')
            self.assertEqual(index.value['photos'][1]['item_id'], item_id)
            station['base']['x_m'] = 2
            stations.write_text(json.dumps({'cup': station}))
            self.assertEqual(index.snapshot('map-a')['items'][0]['station_status'], 'stale')
            self.assertFalse(index.snapshot('map-a')['items'][0]['fetch_available'])
            with self.assertRaisesRegex(ValueError, 'station confirmation'):
                index.edit(reference['id'], dict(expected_revision=4, item_id=item_id, confirmed=True, station_name='cup'), 'map-a')
            before = deepcopy(index.value)
            with patch('web_backend.photos.atomic_json', side_effect=OSError('disk unavailable')):
                with self.assertRaises(OSError):
                    index.add(image, dict(kind='reference', source='phone'), 4)
            self.assertEqual(index.value, before)
            self.assertEqual(len(list((root / 'images').glob('*.png'))), 2)
            restarted = PhotoIndex(root, stations)
            self.assertEqual(restarted.value, before)
            self.assertFalse(restarted.snapshot('map-b')['photos'][1]['map_matches'])
            with self.assertRaisesRegex(ValueError, 'index changed'):
                restarted.delete(reference['id'], 3)
            restarted.delete(reference['id'], 4)
            self.assertEqual(restarted.value['items'][0]['id'], item_id)
            restarted.image_path(observation['id']).unlink()
            self.assertFalse(restarted.snapshot('map-a')['photos'][0]['available'])
            restarted.delete(observation['id'], 5)
            self.assertEqual(restarted.value['items'], [])


class PhotoAPITest(unittest.IsolatedAsyncioTestCase):
    async def test_upload_capture_time_and_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            stations = Path(directory) / 'stations.json'
            stations.write_text('{}')
            app = create_app(directory, stations)
            image = demo_frame('arm')
            async with TestClient(TestServer(app)) as client:
                self.assertEqual((await client.get('/api/stations')).status, 200)
                response = await client.post('/api/photos/reference?expected_revision=0', data=image, headers={'Content-Type': 'image/png'})
                self.assertEqual(response.status, 200, await response.text())
                result = await response.json()
                reference = result['photos'][0]
                self.assertEqual(reference['kind'], 'reference')
                self.assertNotIn('base_pose', reference)
                self.assertNotIn('map_id', reference)
                self.assertEqual(await (await client.get(reference['image_url'])).read(), image)
                editor = app[EDITOR]
                before = editor.sim.telemetry()

                def delayed_frame(camera):
                    # A delayed frame/save must not substitute the later current pose.
                    editor.sim.pose = {**editor.sim.pose, 'yaw_rad': 1.2}
                    return image

                async with client.ws_connect('/api/events') as socket:
                    await socket.receive_json()
                    with patch('web_backend.server.demo_frame', side_effect=delayed_frame):
                        response = await client.post('/api/photos/capture', json=dict(map_id=before['map_id'], camera_id='arm', expected_revision=1))
                    self.assertEqual(response.status, 200, await response.text())
                    self.assertEqual((await socket.receive_json())['type'], 'photos')
                result = await response.json()
                observation = result['photos'][1]
                self.assertEqual(observation['base_pose'], before['pose'])
                self.assertGreaterEqual(observation['captured_at_s'], before['stamp_s'])
                self.assertLessEqual(observation['captured_at_s'], observation['saved_at_s'])
                self.assertNotEqual(observation['base_pose'], editor.sim.pose)
                self.assertEqual(observation['source'], 'simulation')
                for body in (dict(expected_revision=1, item_id='new', name='Cup', confirmed=True),
                             dict(expected_revision=2, item_id='missing', confirmed=True),
                             dict(expected_revision=2, name=[])):
                    self.assertEqual((await client.patch(f"/api/photos/{observation['id']}", json=body)).status, 400)
                editor.sim.phase = 'planning'
                self.assertEqual((await client.post('/api/photos/capture', json=dict(map_id=before['map_id'], camera_id='front', expected_revision=2))).status, 409)
                editor.sim.stop('Test stop')
                self.assertEqual((await client.post('/api/photos/reference?expected_revision=2', data=b'bad', headers={'Content-Type': 'image/png'})).status, 400)
                self.assertEqual((await client.post('/api/photos/reference?expected_revision=2', data=b'bad', headers={'Content-Type': 'text/plain'})).status, 400)
                self.assertEqual((await client.get('/api/photos/missing/image')).status, 404)
                entered, release = threading.Event(), threading.Event()

                def slow_frame(camera):
                    entered.set()
                    if not release.wait(3):
                        raise RuntimeError('Capture test timed out')
                    return image

                with patch('web_backend.server.demo_frame', side_effect=slow_frame):
                    pending = asyncio.create_task(client.post('/api/photos/capture', json=dict(map_id=before['map_id'], camera_id='front', expected_revision=2)))
                    try:
                        self.assertTrue(await asyncio.to_thread(entered.wait, 2))
                        self.assertEqual((await asyncio.wait_for(client.post('/api/stop'), .5)).status, 200)
                    finally:
                        release.set()
                    self.assertEqual((await pending).status, 200)
            async with TestClient(TestServer(create_app(directory, stations))) as client:
                restored = await (await client.get('/api/items')).json()
                self.assertEqual(restored['revision'], 3)
                self.assertEqual(restored['photos'][1]['base_pose'], observation['base_pose'])
                Path(directory, 'images', f"{reference['id']}.png").unlink()
                self.assertEqual((await client.get(reference['image_url'])).status, 404)
                response = await client.delete(f"/api/photos/{reference['id']}", json=dict(expected_revision=3))
                self.assertEqual(response.status, 200)
                self.assertEqual(len((await response.json())['photos']), 2)


if __name__ == '__main__':
    unittest.main()
