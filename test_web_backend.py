"""Necessary local checks only: coordinates, map raster, API edits and persistence."""
import json
import math
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer, make_mocked_request
from web_backend.map_data import demo_map, dominant_angle, local_to_world, render_map, world_to_local
from web_backend.server import DEFAULT_SETTINGS, boundary, create_app
from web_backend.resolve import MODELS


class MapGeometryTest(unittest.TestCase):
    def test_rotation_round_trip_and_wall_alignment(self):
        grid = demo_map()
        angle = dominant_angle(grid)
        self.assertAlmostEqual(math.degrees(angle), 21, delta=1)
        metadata, image = render_map(grid, dict(auto_align=True, manual_angle_deg=0), 3, angle)
        self.assertTrue(image.startswith(b'\x89PNG\r\n\x1a\n'))
        self.assertEqual(grid['origin']['yaw_rad'], math.radians(17))
        for local in ([0, 0], [2.45, 1.15], [12, 10]):
            world = local_to_world(local, grid['origin'])
            view = world_to_local(world, metadata['display']['origin'])
            back = local_to_world(view, metadata['display']['origin'])
            for actual, expected in zip(back, world):
                self.assertAlmostEqual(actual, expected, places=9)


class EditorAPITest(unittest.IsolatedAsyncioTestCase):
    async def test_explicit_lan_origin(self):
        async def handler(request):
            return web.json_response({'ok': True})

        origin = 'http://192.168.1.20:5173'
        headers = {'Host': '192.168.1.20:5173', 'Origin': origin}
        with patch.dict(os.environ, {'MYAGV_WEB_ORIGIN': ''}):
            with self.assertRaises(web.HTTPForbidden):
                await boundary(make_mocked_request('GET', '/api/state', headers=headers), handler)
        with patch.dict(os.environ, {'MYAGV_WEB_ORIGIN': origin}):
            response = await boundary(make_mocked_request('GET', '/api/state', headers=headers), handler)
            self.assertEqual(response.status, 200)
            for invalid in ({'Host': 'evil.test:5173'}, {'Host': '192.168.1.20:5174'},
                            {'Origin': 'http://evil.test'}, {'Origin': origin.replace('http:', 'https:')}):
                with self.assertRaises(web.HTTPForbidden):
                    await boundary(make_mocked_request('GET', '/api/state', headers={**headers, **invalid}), handler)

    async def test_edits_revisions_and_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            async with TestClient(TestServer(create_app(directory))) as client:
                state = await (await client.get('/api/state')).json()
                self.assertFalse(state['motion_available'])
                metadata = await (await client.get('/api/map')).json()
                self.assertEqual((await client.get(metadata['png_url'])).status, 200)
                grid = demo_map()
                corners = [local_to_world(p, grid['origin']) for p in [[1, 1], [2, 1], [2, 2], [1, 2]]]
                command = dict(id='add-one', type='zone_add', map_id=grid['map_id'], expected_revision=0, corners=corners)
                response = await client.post('/api/commands', json=command)
                self.assertEqual(response.status, 200, await response.text())
                result = await response.json()
                zone_id = result['zones']['zones'][0]['id']
                duplicate = await (await client.post('/api/commands', json=command)).json()
                self.assertEqual(result, duplicate)
                self.assertEqual((await client.post('/api/commands', json={**command, 'type': 'zone_delete'})).status, 409)
                self.assertEqual((await client.post('/api/commands', json={**command, 'id': 'stale'})).status, 409)
                bad = {**command, 'id': 'bad', 'expected_revision': 1, 'corners': [[999, 999]] * 4}
                self.assertEqual((await client.post('/api/commands', json=bad)).status, 400)
                values = {**DEFAULT_SETTINGS, 'auto_align': False, 'manual_angle_deg': -26}
                async with client.ws_connect('/api/events') as socket:
                    await socket.receive_json()
                    response = await client.put('/api/settings', json=dict(expected_revision=0, values=values))
                    self.assertEqual(response.status, 200)
                    self.assertEqual((await socket.receive_json())['type'], 'settings')
                self.assertEqual((await client.get(metadata['png_url'])).status, 409)
                current = await (await client.get('/api/map')).json()
                self.assertAlmostEqual(current['display']['origin']['yaw_rad'], math.radians(-26))
                self.assertEqual((await client.get('/api/state', headers={'Origin': 'http://evil.test'})).status, 403)
            async with TestClient(TestServer(create_app(directory))) as client:
                saved = await (await client.get('/api/no-go-zones')).json()
                self.assertEqual(saved['zones'][0]['corners'], corners)
                self.assertEqual((await (await client.get('/api/settings')).json())['values'], values)
                delete = dict(id='delete-one', type='zone_delete', map_id=grid['map_id'], expected_revision=1, zone_id=zone_id)
                response = await client.post('/api/commands', json=delete)
                self.assertEqual(response.status, 200)
                self.assertEqual((await response.json())['zones']['zones'], [])
                self.assertEqual(json.loads((Path(directory) / 'no_go_zones.json').read_text())['zones'], [])

    async def test_llm_settings_migration_validation_and_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'settings.json'
            path.write_text(json.dumps(dict(revision=3, values=dict(auto_align=False, manual_angle_deg=12))))
            async with TestClient(TestServer(create_app(directory))) as client:
                preferences = await (await client.get('/api/settings')).json()
                self.assertEqual(preferences['values']['reasoning_effort'], 'off')
                self.assertEqual(preferences['values']['manual_angle_deg'], 12)
                before_map = await (await client.get('/api/map')).json()
                for model in MODELS:
                    for effort in ('off', 'low', 'high', 'max'):
                        values = {**preferences['values'], 'llm_model': model, 'reasoning_effort': effort}
                        response = await client.put('/api/settings', json=dict(expected_revision=preferences['revision'], values=values))
                        self.assertEqual(response.status, 200, await response.text())
                        preferences = await response.json()
                        self.assertEqual(json.loads(path.read_text())['values'], values)
                        state = await (await client.get('/api/state')).json()
                        self.assertEqual((state['llm']['model'], state['llm']['reasoning_effort']), (model, effort))
                self.assertEqual(await (await client.get('/api/map')).json(), before_map)
                self.assertEqual((await client.get(before_map['png_url'])).status, 200)
                self.assertEqual((await client.get(f"/api/global-costmap.png?view_revision={before_map['display']['view_revision']}&zone_revision=0")).status, 200)
                saved = path.read_text()
                for invalid in ({'reasoning_effort': 'medium'}, {'reasoning_effort': []}, {'llm_model': 'invented/model'}):
                    response = await client.put('/api/settings', json=dict(expected_revision=preferences['revision'], values={**values, **invalid}))
                    self.assertEqual(response.status, 400)
                    self.assertEqual(path.read_text(), saved)
                self.assertEqual((await client.put('/api/settings', json=dict(expected_revision=3, values=values))).status, 409)
            async with TestClient(TestServer(create_app(directory))) as client:
                restored = await (await client.get('/api/settings')).json()
                self.assertEqual(restored['values'], values)
                self.assertEqual(restored['revision'], preferences['revision'])


if __name__ == '__main__':
    unittest.main()
