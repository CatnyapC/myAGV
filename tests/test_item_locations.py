"""Visual estimate boundaries, reachable previews and asynchronous API lifecycle."""
import asyncio
import base64
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import tempfile
import time
import unittest
import uuid
from unittest.mock import AsyncMock, Mock, patch

from aiohttp.test_utils import TestClient, TestServer
from web_backend.item_locations import DEFAULT_CONFIG, approach_goal, estimate_items, project_estimates, validate_approach_path
from web_backend.map_data import local_to_world
from web_backend.photos import PhotoIndex, demo_frame
from web_backend.server import EDITOR, create_app
from web_backend.storage import atomic_json
from web_backend.simulation import global_costmap, navigation_grid, plan_path


def open_map():
    return dict(map_id='test-map', revision=1, frame='map', width=80, height=60,
                resolution_m=.1, origin=dict(x_m=0, y_m=0, yaw_rad=0), cells=[0] * 4800)


class LocationGeometryTest(unittest.TestCase):
    def test_navigation_rejects_blocked_endpoints_but_visual_approach_ignores_obstacles(self):
        start = dict(x_m=1.55, y_m=3.05, yaw_rad=0)
        goal = dict(x_m=4.25, y_m=2.75, yaw_rad=0)
        estimate = dict(confidence=.9, base_pose=goal)
        for label, endpoint in (('Start pose', start), ('Goal pose', goal)):
            for value in (100, -1):
                grid = open_map()
                # An adjacent hazard must block the footprint even when its center is free.
                index = math.floor(endpoint['y_m'] / .1) * 80 + math.floor(endpoint['x_m'] / .1) + 1
                grid['cells'][index] = value
                with self.subTest(label=label, value=value), self.assertRaisesRegex(ValueError, f'{label} blocked by global costmap'):
                    approach_goal(grid, [], None, start, estimate, .2, DEFAULT_CONFIG)
                validate_approach_path(grid, [], start, goal, .2)

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

    def test_approach_returns_capture_pose_and_excludes_walls_unknown_and_no_go(self):
        grid = open_map()
        pose = dict(x_m=1.5, y_m=3, yaw_rad=0)
        capture_pose = dict(x_m=4.2, y_m=2.7, yaw_rad=.4)
        estimate = dict(x_m=5, y_m=3, uncertainty_m=.35, confidence=.9, base_pose=capture_pose)
        goal = approach_goal(grid, [], None, pose, estimate, .2, DEFAULT_CONFIG)
        self.assertEqual(goal, capture_pose)
        self.assertTrue(plan_path(global_costmap(grid, .2), pose, goal))
        rotated = deepcopy(grid)
        rotated['origin'] = dict(x_m=-3, y_m=2, yaw_rad=math.pi / 2)
        p = local_to_world([1.5, 3], rotated['origin'])
        t = local_to_world([5, 3], rotated['origin'])
        captured = local_to_world([4.2, 2.7], rotated['origin'])
        rp = dict(x_m=p[0], y_m=p[1], yaw_rad=math.pi / 2)
        rotated_capture = dict(x_m=captured[0], y_m=captured[1], yaw_rad=math.pi / 2 + .4)
        rg = approach_goal(rotated, [], None, rp, dict(estimate, x_m=t[0], y_m=t[1], base_pose=rotated_capture), .2, DEFAULT_CONFIG)
        self.assertEqual(rg, rotated_capture)
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

        for invalid in (None, dict(x_m=4, y_m=3), dict(capture_pose, yaw_rad=math.nan),
                        dict(capture_pose, x_m=9), dict(capture_pose, yaw_rad=4)):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                approach_goal(grid, [], None, pose, dict(estimate, base_pose=invalid), .2, DEFAULT_CONFIG)
        for cell_value in (100, -1):
            blocked = deepcopy(grid)
            blocked['cells'][27 * 80 + 42] = cell_value
            with self.subTest(cell_value=cell_value), self.assertRaises(ValueError):
                approach_goal(blocked, [], None, pose, estimate, .2, DEFAULT_CONFIG)


class LocationAPITest(unittest.IsolatedAsyncioTestCase):
    async def test_multiview_payload_omits_raw_grid_and_projects_each_chosen_pose(self):
        from fetch_calibration import FETCH_VISION_TIMEOUT_S
        from web_backend.codex_bridge import MODEL, inputs
        photo = dict(id='last', base_pose=dict(x_m=4, y_m=4, yaw_rad=math.pi / 2))
        views = [dict(id='first', base_pose=dict(x_m=1.5, y_m=3, yaw_rad=0),
                      image='data:image/png;base64,' + base64.b64encode(demo_frame('front')).decode())]
        items = [dict(id=name, name=name, appearance=name) for name in ('cup', 'tea')]
        grid = {**open_map(), 'raw_cells': [0] * 400000}
        trace = {}
        invalid_view = None

        async def provider(body, key, timeout, **kwargs):
            self.assertEqual(timeout, FETCH_VISION_TIMEOUT_S)
            inputs(body)  # The previous raw_cells payload exceeded this boundary.
            context = json.loads(body['messages'][1]['content'][0]['text'])
            self.assertNotIn('raw_cells', context['map'])
            self.assertNotIn('cells', context['map'])
            self.assertEqual(len([p for p in body['messages'][1]['content'] if p['type'] == 'image_url']), 3)
            return dict(estimates=[dict(item_id=name, view_index=invalid_view if invalid_view is not None else i, bearing_deg=0, distance_m=1,
                                       uncertainty_m=.4, confidence=.9) for i, name in enumerate(('cup', 'tea'))])

        with patch('web_backend.resolve.request_json', provider):
            result = await estimate_items(demo_frame('front'), demo_frame('front'), grid, photo, items,
                                          'fake', DEFAULT_CONFIG, model=MODEL, views=views, trace=trace)
        self.assertEqual([e['photo_id'] for e in result], ['first', 'last'])
        self.assertAlmostEqual(result[0]['x_m'], 2.5)
        self.assertAlmostEqual(result[1]['y_m'], 5)
        self.assertIn('view_index', trace['result_json'])
        from aiohttp import web
        for invalid_view in (-1, 2, True):
            with self.subTest(view_index=invalid_view), patch('web_backend.resolve.request_json', provider):
                with self.assertRaises(web.HTTPBadGateway):
                    await estimate_items(demo_frame('front'), demo_frame('front'), grid, photo, items,
                                         'fake', DEFAULT_CONFIG, model=MODEL, views=views)

    async def test_waypoint_captures_automatically_start_one_multiview_turn_and_enable_approach(self):
        from web_backend.codex_bridge import MODEL, inputs
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            ui, bridge = root / 'ui', root / 'ros'
            grid = open_map()
            atomic_json(bridge / 'map.json', grid)
            atomic_json(ui / 'settings.json', dict(revision=0, values=dict(auto_align=False,
                        manual_angle_deg=0, llm_model=MODEL, reasoning_effort='low', fast_mode=True)))
            store = PhotoIndex(ui, root / 'stations.json')
            ref = store.add(demo_frame('front'), dict(kind='reference', source='phone'), 0)
            store.edit(ref['id'], dict(expected_revision=1, item_id='new', name='Cup', appearance='Red', confirmed=True), grid['map_id'])
            item_id = store.value['items'][0]['id']
            poses = [dict(x_m=1.5, y_m=3, yaw_rad=0), dict(x_m=2, y_m=3, yaw_rad=0)]
            captures = [str(uuid.uuid4()), str(uuid.uuid4())]

            def capture(i):
                pose = poses[i]
                path = bridge / 'captures' / captures[i]
                atomic_json(path.with_suffix('.json'), dict(kind='observation', source='ros', camera_id='front',
                            capture_id=captures[i], captured_at_s=time.time(), map_id=grid['map_id'], map_revision=1,
                            frame='map', base_pose=pose, current=True,
                            update_key='%.2f:%.2f:%.2f' % (pose['x_m'], pose['y_m'], pose['yaw_rad'])))
                path.with_suffix('.png').write_bytes(demo_frame('front'))

            calls = []
            async def provider(body, key, timeout, trace=None, **kwargs):
                calls.append(body)
                inputs(body)
                self.assertEqual(len([p for p in body['messages'][1]['content'] if p['type'] == 'image_url']), 3)
                trace['codex'] = dict(thread_id='item-session', turn_id='turn', service_tier='priority')
                return dict(estimates=[dict(item_id=item_id, view_index=0, bearing_deg=0,
                                           distance_m=3, uncertainty_m=.4, confidence=.9)])

            with patch('web_backend.server.load_token', return_value=('configured', 'fake')), \
                 patch('web_backend.resolve.request_json', provider):
                app = create_app(ui, root / 'stations.json', hardware_dir=bridge)
                hardware = app[EDITOR].hardware
                hardware.refresh, hardware.stop = Mock(), AsyncMock()
                hardware.costmap = Mock(return_value=grid)
                hardware.control = dict(phase='navigating', navigation_ready=True, zones_ready=True,
                                        map_update=dict(active=True, completed=1, goals=poses))
                async with TestClient(TestServer(app)) as client:
                    capture(0)
                    await asyncio.sleep(.65)
                    self.assertEqual(calls, [])
                    hardware.control.update(phase='idle', map_update=dict(active=False, state='complete', completed=2, goals=poses))
                    await asyncio.sleep(.55)
                    self.assertEqual(calls, [])  # Second acquisition has not been imported yet.
                    capture(1)
                    for _ in range(100):
                        index = await (await client.get('/api/items')).json()
                        if index['locations']:
                            break
                        await asyncio.sleep(.05)
                    self.assertEqual(len(calls), 1)
                    self.assertEqual(index['locations'][0]['x_m'], 4.5)
                    self.assertEqual(index['locations'][0]['base_pose'], poses[0])
                    self.assertTrue(all(p['location_analysis']['status'] == 'complete' for p in index['photos'] if p['kind'] == 'observation'))
                    log = await (await client.get('/api/fetch-log')).json()
                    self.assertEqual(log['rounds'][0]['mode'], 'locations')
                    self.assertEqual(log['rounds'][0]['codex']['thread_id'], 'item-session')
                    hardware.control['stamp_s'] = time.time()
                    hardware.snapshot = dict(stamp_s=time.time(), pose_stamp_s=time.time(), frame='map',
                                             map_id=grid['map_id'], pose=poses[1])
                    location = index['locations'][0]
                    response = await client.post('/api/item-locations/approach', json=dict(map_id=grid['map_id'],
                        expected_revision=index['revision'], zone_revision=0, item_id=item_id, photo_id=location['photo_id']))
                    self.assertEqual(response.status, 200, await response.text())
                    self.assertEqual((await response.json())['goal'], poses[0])

                    payload = dict(map_id=grid['map_id'], expected_revision=index['revision'])
                    response = await client.post('/api/item-locations/clear', json={**payload, 'expected_revision': 0})
                    self.assertEqual(response.status, 400)
                    response = await client.post('/api/item-locations/clear', json={**payload, 'map_id': 'old'})
                    self.assertEqual(response.status, 409)
                    hardware.control['phase'] = 'navigating'
                    response = await client.post('/api/item-locations/clear', json=payload)
                    self.assertEqual(response.status, 409)
                    hardware.control['phase'] = 'update_paused'
                    points = await (await client.get('/api/observation-points')).json()
                    response = await client.post('/api/item-locations/clear', json=payload)
                    self.assertEqual(response.status, 200, await response.text())
                    cleared = await response.json()
                    self.assertEqual(cleared['locations'], [])
                    self.assertEqual(cleared['items'], index['items'])
                    self.assertEqual([p['id'] for p in cleared['photos']], [p['id'] for p in index['photos']])
                    self.assertTrue(all(p['available'] for p in cleared['photos']))
                    self.assertEqual(await (await client.get('/api/observation-points')).json(), points)
                    self.assertEqual(PhotoIndex(ui, root / 'stations.json').snapshot(grid['map_id'])['locations'], [])
                    await asyncio.sleep(.65)
                    self.assertEqual(len(calls), 1)  # Old ROS frames must not be estimated again.
                    captures = [str(uuid.uuid4()), str(uuid.uuid4())]
                    capture(0)
                    capture(1)
                    for _ in range(100):
                        fresh = await (await client.get('/api/items')).json()
                        if fresh['locations']:
                            break
                        await asyncio.sleep(.05)
                    self.assertEqual(len(calls), 2)
                    self.assertEqual(fresh['locations'][0]['x_m'], 4.5)
                    self.assertNotIn(fresh['locations'][0]['photo_id'], [p['id'] for p in index['photos']])

    async def test_model_receives_both_images_and_only_validated_measurements_survive(self):
        from aiohttp import web
        photo = dict(base_pose=dict(x_m=1.5, y_m=3, yaw_rad=0))
        items = [dict(id='cup', name='Cup', appearance='Red')]
        async def provider(body, key, timeout, fast_mode=True, trace=None):
            self.assertEqual(len([e for e in body['messages'][1]['content'] if e['type'] == 'image_url']), 2)
            context = json.loads(body['messages'][1]['content'][0]['text'])
            self.assertEqual(context['captured_base_pose'], photo['base_pose'])
            self.assertEqual(context['map']['map_id'], 'test-map')
            return dict(estimates=[dict(item_id='cup', bearing_deg=0, distance_m=1, uncertainty_m=.4, confidence=.9)])
        with patch('web_backend.resolve.request_json', provider):
            result = await estimate_items(demo_frame('front'), demo_frame('front'), open_map(), photo, items, 'fake', DEFAULT_CONFIG)
            self.assertEqual(result[0]['x_m'], 2.5)
        async def malicious(body, key, timeout, fast_mode=True, trace=None):
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

            async def infer(image, map_image, map_info, observation, items, key, config, model=None, fast_mode=True, views=(), trace=None):
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
                    self.assertEqual(goal, photo['base_pose'])
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

                    mode = 'stall'
                    entered.clear()
                    release.clear()
                    await client.post('/api/item-locations/estimate', json=dict(map_id='test-map', expected_revision=index['revision']))
                    await asyncio.wait_for(entered.wait(), 3)
                    running = await (await client.get('/api/items')).json()
                    response = await client.post('/api/item-locations/clear', json=dict(map_id='test-map', expected_revision=running['revision']))
                    self.assertEqual(response.status, 200, await response.text())
                    release.set()
                    for _ in range(100):
                        log = await (await client.get('/api/fetch-log')).json()
                        if log['rounds'][-1]['status'] == 'cancelled':
                            break
                        await asyncio.sleep(.05)
                    self.assertEqual(log['rounds'][-1]['status'], 'cancelled')
                    self.assertEqual((await (await client.get('/api/items')).json())['locations'], [])
                    self.assertEqual(PhotoIndex(root, root / 'stations.json').snapshot('test-map')['locations'], [])


if __name__ == '__main__':
    unittest.main()
