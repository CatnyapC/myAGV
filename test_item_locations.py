"""Visual estimate boundaries, reachable previews and asynchronous API lifecycle."""
import asyncio
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from aiohttp.test_utils import TestClient, TestServer
from web_backend.item_locations import DEFAULT_CONFIG, approach_goal, estimate_items, project_estimates
from web_backend.map_data import local_to_world
from web_backend.photos import PhotoIndex, demo_frame
from web_backend.server import create_app
from web_backend.simulation import global_costmap, navigation_grid, plan_path


def open_map():
    return dict(map_id='test-map', revision=1, frame='map', width=80, height=60,
                resolution_m=.1, origin=dict(x_m=0, y_m=0, yaw_rad=0), cells=[0] * 4800)


class LocationGeometryTest(unittest.TestCase):
    def test_projection_validates_identity_numbers_and_camera_frame(self):
        grid = open_map()
        photo = dict(base_pose=dict(x_m=3, y_m=3, yaw_rad=math.pi / 2))
        items = [dict(id='cup', name='Cup', appearance='Red')]
        config = {**DEFAULT_CONFIG, 'camera_forward_m': .2, 'camera_yaw_deg': -90}
        result = dict(estimates=[dict(item_id='cup', bearing_deg=0, distance_m=1,
                                     uncertainty_m=.2, confidence=.9)])
        estimate = project_estimates(result, photo, grid, items, config)[0]
        self.assertAlmostEqual(estimate['x_m'], 4)
        self.assertAlmostEqual(estimate['y_m'], 3.2)
        self.assertEqual(estimate['uncertainty_m'], .35)
        for field, value in [('item_id', 'invented'), ('distance_m', True), ('bearing_deg', 90),
                             ('distance_m', float('nan')), ('confidence', 2), ('uncertainty_m', -.1)]:
            invalid = deepcopy(result)
            invalid['estimates'][0][field] = value
            with self.assertRaises(ValueError):
                project_estimates(invalid, photo, grid, items, config)
        duplicate = dict(estimates=result['estimates'] * 2)
        with self.assertRaises(ValueError):
            project_estimates(duplicate, photo, grid, items, config)
        result['estimates'][0]['distance_m'] = 6
        self.assertEqual(project_estimates(result, photo, grid, items, config), [])

    def test_approach_excludes_walls_unknown_and_no_go_and_faces_item(self):
        grid = open_map()
        pose = dict(x_m=1.5, y_m=3, yaw_rad=0)
        estimate = dict(x_m=5, y_m=3, uncertainty_m=.35, confidence=.9, base_pose=pose)
        goal = approach_goal(grid, [], None, pose, estimate, .2, DEFAULT_CONFIG)
        self.assertGreaterEqual(math.hypot(goal['x_m']-5, goal['y_m']-3), .8)
        self.assertAlmostEqual(goal['yaw_rad'], math.atan2(3-goal['y_m'], 5-goal['x_m']))
        self.assertTrue(plan_path(global_costmap(grid, .2), pose, goal))
        rotated = deepcopy(grid)
        rotated['origin'] = dict(x_m=-3, y_m=2, yaw_rad=math.pi / 2)
        p = local_to_world([1.5, 3], rotated['origin'])
        t = local_to_world([5, 3], rotated['origin'])
        rp = dict(x_m=p[0], y_m=p[1], yaw_rad=math.pi / 2)
        rg = approach_goal(rotated, [], None, rp, dict(estimate, x_m=t[0], y_m=t[1], base_pose=rp), .2, DEFAULT_CONFIG)
        self.assertTrue(plan_path(global_costmap(rotated, .2), rp, rg))
        for wall_value in (100, -1):
            blocked = deepcopy(grid)
            for y in range(60):
                blocked['cells'][y*80+30] = wall_value
            with self.assertRaises(ValueError):
                approach_goal(blocked, [], None, pose, estimate, .2, DEFAULT_CONFIG)
        zones = [dict(corners=[[2.9, 0], [3.1, 0], [3.1, 6], [2.9, 6]])]
        with self.assertRaises(ValueError):
            approach_goal(grid, zones, None, pose, estimate, .2, DEFAULT_CONFIG)
        self.assertEqual(navigation_grid(grid, zones)['cells'][3*80+30], 100)
        with self.assertRaises(ValueError):
            approach_goal(grid, [], None, pose, dict(estimate, confidence=.69), .2, DEFAULT_CONFIG)
        dynamic = deepcopy(grid)
        for y in range(60):
            dynamic['cells'][y*80+30] = 100
        with self.assertRaises(ValueError):
            approach_goal(grid, [], dynamic, pose, estimate, .2, DEFAULT_CONFIG)


class LocationAPITest(unittest.IsolatedAsyncioTestCase):
    async def test_model_receives_both_images_and_only_validated_measurements_survive(self):
        from aiohttp import web
        photo = dict(base_pose=dict(x_m=1.5, y_m=3, yaw_rad=0))
        items = [dict(id='cup', name='Cup', appearance='Red')]
        async def provider(body, key, timeout):
            self.assertEqual(len([e for e in body['messages'][1]['content'] if e['type'] == 'image_url']), 2)
            context = json.loads(body['messages'][1]['content'][0]['text'])
            self.assertEqual(context['captured_base_pose'], photo['base_pose'])
            self.assertEqual(context['map']['map_id'], 'test-map')
            return dict(estimates=[dict(item_id='cup', bearing_deg=0, distance_m=1, uncertainty_m=.4, confidence=.9)])
        with patch('web_backend.resolve.request_json', provider):
            result = await estimate_items(demo_frame('front'), demo_frame('front'), open_map(), photo, items, 'fake', DEFAULT_CONFIG)
            self.assertEqual(result[0]['x_m'], 2.5)
        async def malicious(body, key, timeout):
            return dict(estimates=[dict(item_id='cup', bearing_deg=0, distance_m=1, uncertainty_m=.4, confidence=.9, command='go')])
        with patch('web_backend.resolve.request_json', malicious):
            with self.assertRaises(web.HTTPBadGateway):
                await estimate_items(demo_frame('front'), demo_frame('front'), open_map(), photo, items, 'fake', DEFAULT_CONFIG)

    async def test_queue_persistence_approach_revisions_stop_and_provider_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            store = PhotoIndex(root, root / 'stations.json')
            ref = store.add(demo_frame('front'), dict(kind='reference', source='phone'), 0)
            store.edit(ref['id'], dict(expected_revision=1, item_id='new', name='Cup', appearance='Red', confirmed=True), 'test-map')
            item_id = store.value['items'][0]['id']
            photo = store.add(demo_frame('front'), dict(kind='observation', source='simulation', camera_id='front',
                        captured_at_s=1, map_id='test-map', map_revision=1, frame='map',
                        base_pose=dict(x_m=1.5, y_m=3, yaw_rad=0)), 2)
            mode, entered, release = 'ok', asyncio.Event(), asyncio.Event()

            async def infer(image, map_image, map_info, observation, items, key, config):
                self.assertTrue(image.startswith(b'\x89PNG'))
                self.assertTrue(map_image.startswith(b'\x89PNG'))
                self.assertEqual(observation['base_pose']['x_m'], 1.5)
                self.assertEqual(map_info['map_id'], 'test-map')
                if mode == 'stall':
                    entered.set()
                    await release.wait()
                if mode == 'error':
                    raise ValueError('Invalid model estimate')
                return [dict(item_id=item_id, x_m=5, y_m=3, uncertainty_m=.35, confidence=.9)]

            async def wait_status(client, status):
                for _ in range(100):
                    index = await (await client.get('/api/items')).json()
                    if index['photos'][1].get('location_analysis', {}).get('status') == status:
                        return index
                    await asyncio.sleep(.05)
                self.fail('Location worker did not finish')

            with patch.dict(os.environ, {'OPENROUTER_API_KEY': 'fake', 'CREDENTIALS_DIRECTORY': ''}), \
                 patch('web_backend.server.demo_map', open_map), patch('web_backend.server.estimate_items', infer):
                async with TestClient(TestServer(create_app(root, root / 'stations.json'))) as client:
                    response = await client.post('/api/item-locations/estimate', json=dict(map_id='test-map', expected_revision=3))
                    self.assertEqual(response.status, 202)
                    index = await wait_status(client, 'complete')
                    self.assertEqual(index['locations'][0]['item_id'], item_id)
                    # The model must never create a confirmed item/station association.
                    self.assertNotIn('item_id', index['photos'][1])
                    self.assertNotIn('station_link', index['items'][0])
                    payload = dict(map_id='test-map', expected_revision=index['revision'], zone_revision=0,
                                   item_id=item_id, photo_id=photo['id'])
                    response = await client.post('/api/item-locations/approach', json=payload)
                    self.assertEqual(response.status, 200, await response.text())
                    goal = (await response.json())['goal']
                    self.assertLess(goal['x_m'], 5)
                    stale = await client.post('/api/item-locations/approach', json={**payload, 'zone_revision': 1})
                    self.assertEqual(stale.status, 409)
                    stale = await client.post('/api/item-locations/approach', json={**payload, 'expected_revision': 0})
                    self.assertEqual(stale.status, 409)
                    wrong_map = await client.post('/api/item-locations/approach', json={**payload, 'map_id': 'old'})
                    self.assertEqual(wrong_map.status, 409)
                    self.assertEqual(PhotoIndex(root, root / 'stations.json').snapshot('test-map')['locations'][0]['x_m'], 5)
                    self.assertEqual(PhotoIndex(root, root / 'stations.json').snapshot('another-map')['locations'], [])
                    mode = 'stall'
                    await client.post('/api/item-locations/estimate', json=dict(map_id='test-map', expected_revision=index['revision']))
                    await asyncio.wait_for(entered.wait(), 3)
                    await client.post('/api/stop')
                    release.set()
                    index = await wait_status(client, 'cancelled')
                    self.assertEqual(index['locations'], [])
                    mode = 'error'
                    await client.post('/api/item-locations/estimate', json=dict(map_id='test-map', expected_revision=index['revision']))
                    index = await wait_status(client, 'error')
                    self.assertEqual(index['locations'], [])
                    self.assertIn('Invalid model estimate', index['photos'][1]['location_analysis']['error'])


if __name__ == '__main__':
    unittest.main()
