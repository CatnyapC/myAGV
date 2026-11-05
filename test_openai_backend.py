"""Direct OpenAI catalog and inference contracts; no paid requests."""
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer
from web_backend.resolve import load_key, openai_models, request_json
from web_backend.server import create_app, DEFAULT_SETTINGS


class OpenAIBackendTest(unittest.IsolatedAsyncioTestCase):
    async def test_catalog_routing_settings_and_restart(self):
        calls = []
        failed = False

        async def models(request):
            self.assertEqual(request.headers['Authorization'], 'Bearer private-key')
            if failed:
                return web.Response(status=401, text='private-key')
            return web.json_response(dict(data=[dict(id='gpt-test'), dict(id='embedding-test'), dict(id='gpt-test')]))

        async def complete(request):
            self.assertEqual(request.headers['Authorization'], 'Bearer private-key')
            body = await request.json()
            calls.append(body)
            self.assertEqual(body['model'], 'gpt-test')
            self.assertEqual(body['max_completion_tokens'], 8192)
            self.assertFalse({'provider', 'reasoning', 'temperature', 'max_tokens', 'session'} & body.keys())
            return web.json_response(dict(choices=[dict(finish_reason='stop', message=dict(content='{"ok":true}'))]))

        upstream = web.Application()
        upstream.router.add_get('/models', models)
        upstream.router.add_post('/chat/completions', complete)
        with tempfile.TemporaryDirectory() as directory:
            async with TestServer(upstream) as mock:
                with patch.dict(os.environ, {'OPENAI_API_KEY': 'private-key', 'OPENROUTER_API_KEY': 'router-key'}, clear=True), patch('web_backend.resolve.OPENAI_URL', str(mock.make_url('')).rstrip('/')):
                    self.assertEqual(load_key('openai_api_key', 'OPENAI_API_KEY'), ('configured', 'private-key'))
                    self.assertEqual(load_key(), ('configured', 'router-key'))
                    async with TestClient(TestServer(create_app(directory))) as client:
                        settings = await (await client.get('/api/settings')).json()
                        direct = [m for m in settings['llm_models'] if m['id'].startswith('openai/')]
                        self.assertEqual([m['id'] for m in direct], ['openai/embedding-test', 'openai/gpt-test'])
                        self.assertEqual(direct[0]['efforts'], ['auto'])
                        values = dict(DEFAULT_SETTINGS, llm_model='openai/gpt-test', reasoning_effort='auto')
                        response = await client.put('/api/settings', json=dict(expected_revision=0, values=values))
                        self.assertEqual(response.status, 200, await response.text())
                        state = await (await client.get('/api/state')).json()
                        self.assertEqual(state['llm']['status'], 'configured')
                        self.assertNotIn('private-key', json.dumps(state) + json.dumps(settings))
                        response = await client.put('/api/settings', json=dict(expected_revision=1, values=dict(values, llm_model='openai/not-listed')))
                        self.assertEqual(response.status, 400)
                    body = dict(model='openai/gpt-test', messages=[], provider={}, reasoning={}, temperature=0, max_tokens=64, session={})
                    self.assertEqual(await request_json(body, 'private-key', 2), dict(ok=True))
                    self.assertEqual(len(calls), 1)
                    self.assertIn('provider', body)
                    failed = True
                    with self.assertRaises(web.HTTPBadGateway) as error:
                        await openai_models('private-key')
                    self.assertNotIn('private-key', error.exception.text)
                    async with TestClient(TestServer(create_app(directory))) as client:
                        settings = await (await client.get('/api/settings')).json()
                        self.assertEqual(settings['values']['llm_model'], 'openai/gpt-test')
                        self.assertTrue(settings['llm_models'])
                        state = await (await client.get('/api/state')).json()
                        self.assertEqual(state['llm']['status'], 'unavailable')

    async def test_missing_key_and_credential_precedence(self):
        self.assertEqual(await openai_models(''), [])
        with tempfile.TemporaryDirectory() as directory:
            with patch.dict(os.environ, {'CREDENTIALS_DIRECTORY': directory, 'OPENAI_API_KEY': 'ignored'}, clear=True):
                self.assertEqual(load_key('openai_api_key', 'OPENAI_API_KEY'), ('unavailable', ''))
                Path(directory, 'openai_api_key').write_text('private-key\n')
                self.assertEqual(load_key('openai_api_key', 'OPENAI_API_KEY'), ('configured', 'private-key'))
                Path(directory, 'openai_api_key').write_text('bad\nheader')
                self.assertEqual(load_key('openai_api_key', 'OPENAI_API_KEY'), ('unavailable', ''))


if __name__ == '__main__':
    unittest.main()
