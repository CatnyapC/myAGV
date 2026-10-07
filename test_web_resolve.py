"""Mock OpenRouter contract and resolver guards; no real key or paid request."""
import asyncio
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer
from web_backend.map_data import demo_map
from web_backend.photos import PhotoIndex, demo_frame
from web_backend.resolve import MODEL, MODELS, load_key, validate_result
from web_backend.server import create_app


class ResolverValidationTest(unittest.TestCase):
    def test_credentials_and_result_boundary(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch.dict(os.environ, {'CREDENTIALS_DIRECTORY': directory, 'OPENROUTER_API_KEY': 'ignored'}, clear=True):
                self.assertEqual(load_key(), ('unavailable', ''))
                Path(directory, 'llm_api_key').write_text('sk-test\n')
                self.assertEqual(load_key(), ('configured', 'sk-test'))
                Path(directory, 'llm_api_key').write_text('bad\nheader')
                self.assertEqual(load_key(), ('unavailable', ''))
        with patch.dict(os.environ, {}, clear=True):
            self.assertEqual(load_key(), ('not_configured', ''))
        for invalid in ({'status': 'matched', 'item_ids': ['unknown']},
                        {'status': 'matched', 'item_ids': []},
                        {'status': 'ambiguous', 'item_ids': ['a', 'a']},
                        {'status': 'not_found', 'item_ids': ['a']},
                        {'status': 'matched', 'item_ids': ['a'], 'goal': [1, 2]},
                        {'status': 'matched', 'item_ids': [{}]}):
            with self.assertRaises(web.HTTPBadGateway):
                validate_result(invalid, {'a', 'b'})
        self.assertEqual(validate_result(dict(status='ambiguous', item_ids=['a', 'b']), {'a', 'b'})['status'], 'ambiguous')


class ResolverAPITest(unittest.IsolatedAsyncioTestCase):
    async def test_mock_provider_selection_errors_and_cancellation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            stations = root / 'stations.json'
            station = dict(base=dict(x_m=1, y_m=2, yaw_deg=30), arm_angles_deg=[0, 10, 20])
            stations.write_text(json.dumps({'cup': station}))
            index, image = PhotoIndex(root, stations), demo_frame('front')
            ref = index.add(image, dict(kind='reference', source='phone'), 0)
            index.edit(ref['id'], dict(expected_revision=1, item_id='new', name='Red cup', appearance='Red with handle',
                                      station_name='cup', confirmed=True, station_confirmed=True), demo_map()['map_id'])
            item_id = index.value['items'][0]['id']
            obs = index.add(image, dict(kind='observation', source='simulation', camera_id='front',
                                       map_id=demo_map()['map_id'], map_revision=0, frame='map', captured_at_s=1,
                                       base_pose=dict(x_m=4, y_m=5, yaw_rad=0)), 2)
            index.edit(obs['id'], dict(expected_revision=3, item_id=item_id, station_name='cup', confirmed=True), demo_map()['map_id'])
            other = index.add(image, dict(kind='reference', source='phone'), 4)
            index.edit(other['id'], dict(expected_revision=5, item_id='new', name='Other red cup', appearance='Red', confirmed=True), demo_map()['map_id'])
            other_id = index.value['items'][1]['id']
            mode, entered, release = 'matched', asyncio.Event(), asyncio.Event()
            expected_model, expected_effort = MODEL, 'off'
            calls = []

            async def provider(request):
                body = await request.json()
                calls.append(body)
                self.assertEqual(request.headers['Authorization'], 'Bearer fake-server-key')
                self.assertEqual(body['model'], expected_model)
                self.assertEqual(body['provider'], dict(sort='latency', require_parameters=True))
                self.assertEqual(body['response_format']['type'], 'json_schema')
                self.assertEqual(body['reasoning'], dict(enabled=False) if expected_effort == 'off' else dict(effort=expected_effort, exclude=True))
                self.assertEqual(body['max_tokens'], {'off': 384, 'low': 2048, 'high': 4096, 'max': 8192}[expected_effort])
                catalog = json.loads(body['messages'][1]['content'])['items']
                self.assertTrue(all(set(i) == {'id', 'name', 'appearance'} for i in catalog))
                if mode == 'stall':
                    entered.set()
                    await release.wait()
                if mode == 'timeout':
                    await asyncio.sleep(.15)
                if mode == 'error':
                    return web.json_response(dict(error='echo fake-server-key'), status=401)
                result = dict(status='ambiguous', item_ids=[item_id, other_id]) if mode == 'ambiguous' else dict(status='not_found', item_ids=[]) if mode == 'none' else dict(status='matched', item_ids=['invented' if mode == 'invalid' else item_id])
                return web.json_response(dict(choices=[dict(finish_reason='stop', message=dict(content=json.dumps(result)))]))

            mock = web.Application()
            mock.router.add_post('/chat', provider)
            async with TestServer(mock) as upstream:
                with patch.dict(os.environ, {'OPENROUTER_API_KEY': 'fake-server-key', 'CREDENTIALS_DIRECTORY': ''}), patch('web_backend.resolve.API_URL', str(upstream.make_url('/chat'))):
                    async with TestClient(TestServer(create_app(root, stations))) as client:
                        payload = dict(text='把红色杯子拿来', map_id=demo_map()['map_id'], expected_revision=6)
                        state = await (await client.get('/api/state')).json()
                        self.assertEqual(state['llm']['status'], 'configured')
                        self.assertNotIn('fake-server-key', json.dumps(state))
                        result = await (await client.post('/api/resolve', json=payload)).json()
                        self.assertEqual(result['item_ids'], [item_id])
                        self.assertEqual(result['photo_id'], obs['id'])
                        self.assertEqual(result['goal']['x_m'], station['base']['x_m'])
                        self.assertNotEqual(result['goal']['x_m'], obs['base_pose']['x_m'])
                        after = await (await client.get('/api/state')).json()
                        self.assertIsNone(after['navigation']['goal'])  # Selection never starts navigation.
                        for mode in ('ambiguous', 'none'):
                            result = await (await client.post('/api/resolve', json=payload)).json()
                            self.assertEqual(result['status'], 'ambiguous' if mode == 'ambiguous' else 'not_found')
                            self.assertNotIn('goal', result)
                        for mode in ('invalid', 'error'):
                            response = await client.post('/api/resolve', json=payload)
                            self.assertEqual(response.status, 502)
                            self.assertNotIn('fake-server-key', await response.text())
                        mode = 'timeout'
                        with patch('web_backend.resolve.TIMEOUT_S', .03):
                            self.assertEqual((await client.post('/api/resolve', json=payload)).status, 504)
                        call_count = len(calls)
                        self.assertEqual((await client.post('/api/resolve', json={**payload, 'expected_revision': 5})).status, 409)
                        self.assertEqual((await client.post('/api/resolve', json={**payload, 'text': 'x'*501})).status, 400)
                        self.assertEqual(len(calls), call_count)
                        mode = 'matched'
                        preferences = await (await client.get('/api/settings')).json()
                        for expected_model in MODELS:
                            for expected_effort in ('off', 'low', 'high', 'max'):
                                values = {**preferences['values'], 'llm_model': expected_model, 'reasoning_effort': expected_effort}
                                response = await client.put('/api/settings', json=dict(expected_revision=preferences['revision'], values=values))
                                self.assertEqual(response.status, 200)
                                preferences = await response.json()
                                result = await (await client.post('/api/resolve', json=payload)).json()
                                self.assertEqual(result['status'], 'matched')
                                self.assertEqual(result['settings_revision'], preferences['revision'])
                        call_count = len(calls)
                        mode = 'stall'
                        pending = asyncio.create_task(client.post('/api/resolve', json=payload))
                        try:
                            await asyncio.wait_for(entered.wait(), 1)
                            self.assertEqual((await client.post('/api/resolve', json=payload)).status, 429)
                            self.assertEqual(len(calls), call_count + 1)
                            self.assertEqual((await asyncio.wait_for(client.post('/api/stop'), .5)).status, 200)
                        finally:
                            release.set()
                        self.assertEqual((await pending).status, 409)
                        entered.clear(); release.clear()
                        pending = asyncio.create_task(client.post('/api/resolve', json=payload))
                        try:
                            await asyncio.wait_for(entered.wait(), 1)
                            values = {**preferences['values'], 'reasoning_effort': 'low'}
                            response = await client.put('/api/settings', json=dict(expected_revision=preferences['revision'], values=values))
                            self.assertEqual(response.status, 200)
                        finally:
                            release.set()
                        self.assertEqual((await pending).status, 409)
                        expected_effort = 'low'
                        entered.clear(); release.clear()
                        pending = asyncio.create_task(client.post('/api/resolve', json=payload))
                        try:
                            await asyncio.wait_for(entered.wait(), 1)
                            response = await client.patch(f"/api/photos/{ref['id']}", json=dict(expected_revision=6, item_id=item_id, name='Red cup', appearance='Red with handle', station_name='cup', confirmed=True))
                            self.assertEqual(response.status, 200)
                        finally:
                            release.set()
                        self.assertEqual((await pending).status, 409)
                        payload['expected_revision'] = 7
                        mode = 'matched'
                        station['base']['x_m'] = 2
                        stations.write_text(json.dumps({'cup': station}))
                        result = await (await client.post('/api/resolve', json=payload)).json()
                        self.assertNotIn('goal', result)
                        self.assertIn('blocked_reason', result)
                        Path(root, 'images', f"{obs['id']}.png").unlink()
                        result = await (await client.post('/api/resolve', json=payload)).json()
                        self.assertEqual(result['blocked_reason'], 'No confirmed observation on this map')
            with patch.dict(os.environ, {'OPENROUTER_API_KEY': '', 'CREDENTIALS_DIRECTORY': ''}):
                async with TestClient(TestServer(create_app(root, stations))) as client:
                    self.assertEqual((await client.post('/api/resolve', json=payload)).status, 503)


if __name__ == '__main__':
    unittest.main()
