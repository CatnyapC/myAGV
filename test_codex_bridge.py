"""Codex adapter contracts, failure handling, and provider routing without inference."""
import asyncio
import base64
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import AsyncMock, Mock, patch

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer
from web_backend import codex_bridge as bridge
from web_backend.photos import demo_frame
from web_backend.resolve import locate_pickup, request_json
from web_backend.server import create_app


def body():
    return dict(model=bridge.MODEL, stream=False,
        messages=[dict(role='system', content='Measure the supplied images.'),
                  dict(role='user', content=[dict(type='text', text='GOAL'),
                      dict(type='image_url', image_url=dict(url='data:image/png;base64,' + base64.b64encode(demo_frame('arm')).decode()))])],
        response_format=dict(json_schema=dict(schema=dict(type='object', properties=dict(ok=dict(type='boolean')),
            required=['ok'], additionalProperties=False))))


class CodexBridgeTest(unittest.IsolatedAsyncioTestCase):
    def test_inputs_and_credentials_are_bounded(self):
        instructions, content, schema, effort = bridge.inputs(body())
        self.assertEqual(content[1]['type'], 'image')
        self.assertEqual(effort, 'low')
        for mutate in (lambda b: b.update(model='arbitrary'),
                       lambda b: b.update(reasoning=dict(effort='off')),
                       lambda b: b['messages'][1]['content'][1]['image_url'].update(url='https://example.com/image.jpg'),
                       lambda b: b['messages'][1]['content'][1]['image_url'].update(url='data:image/png;base64,AQ=='),
                       lambda b: b.update(stream=True)):
            value = body()
            mutate(value)
            with self.assertRaises(ValueError):
                bridge.inputs(value)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'token'
            with patch.dict(os.environ, dict(MYAGV_CODEX_TOKEN_FILE=str(path))):
                self.assertEqual(bridge.load_token(), ('unavailable', ''))
                path.write_text('a' * 64)
                self.assertEqual(bridge.load_token(), ('configured', 'a' * 64))
                path.write_text('short')
                self.assertEqual(bridge.load_token(), ('unavailable', ''))

    async def test_rpc_completion_failure_timeout_and_cancellation_kill_process(self):
        for mode in ('completed', 'failed', 'tool', 'timeout', 'cancelled'):
            with self.subTest(mode=mode):
                queue = asyncio.Queue()
                sent = []
                process = Mock(returncode=None)
                process.wait = AsyncMock(return_value=0)
                process.stdin.drain = AsyncMock()
                process.stdout.readline = queue.get

                def write(raw):
                    message = json.loads(raw)
                    sent.append(message)
                    identity = message.get('id')
                    if identity is None:
                        return
                    result = {1: {}, 2: dict(account=dict(type='chatgpt')),
                              3: dict(thread=dict(id='thread')), 4: dict(turn=dict(id='turn'))}[identity]
                    queue.put_nowait(json.dumps(dict(id=identity, result=result)).encode())
                    if identity == 4 and mode not in ('timeout', 'cancelled'):
                        item = dict(type='commandExecution') if mode == 'tool' else dict(type='agentMessage', phase='final_answer', text='{"ok":true}')
                        queue.put_nowait(json.dumps(dict(method='item/completed', params=dict(threadId='thread', item=item))).encode())
                        queue.put_nowait(json.dumps(dict(method='turn/completed', params=dict(threadId='thread', turn=dict(status=mode)))).encode())

                process.stdin.write = write
                with patch.object(bridge, 'codex_command', return_value=['codex']), \
                     patch.object(bridge.asyncio, 'create_subprocess_exec', AsyncMock(return_value=process)), \
                     patch.object(bridge, 'TIMEOUT_S', .03):
                    if mode == 'completed':
                        result = await bridge.infer(body())
                        self.assertEqual(json.loads(result['choices'][0]['message']['content']), dict(ok=True))
                        self.assertTrue(sent[3]['params']['ephemeral'])
                        self.assertEqual(sent[3]['params']['sandbox'], 'read-only')
                        self.assertEqual(sent[3]['params']['serviceTier'], 'fast')
                        self.assertEqual(sent[4]['params']['serviceTier'], 'fast')
                        self.assertEqual(sent[4]['params']['outputSchema'], body()['response_format']['json_schema']['schema'])
                    elif mode == 'cancelled':
                        task = asyncio.create_task(bridge.infer(body()))
                        while len(sent) < 5:
                            await asyncio.sleep(0)
                        task.cancel()
                        with self.assertRaises(asyncio.CancelledError):
                            await task
                    else:
                        with self.assertRaises(TimeoutError if mode == 'timeout' else RuntimeError):
                            await bridge.infer(body())
                process.kill.assert_called_once()
                process.wait.assert_awaited_once()

    async def test_http_auth_provider_routing_and_failures(self):
        token = 'private-test-token-' + 'a' * 32
        result = dict(choices=[dict(finish_reason='stop', message=dict(content='{"ok":true}'))])
        with patch.object(bridge, 'load_token', return_value=('configured', token)), \
             patch.object(bridge, 'infer', AsyncMock(return_value=result)) as infer:
            async with TestClient(TestServer(bridge.create_app())) as client:
                self.assertEqual((await client.post('/v1/chat/completions', json=body())).status, 401)
                headers = dict(Authorization='Bearer ' + token)
                self.assertEqual((await client.post('/v1/chat/completions', json=body(), headers={**headers, 'Origin': 'http://localhost'})).status, 401)
                self.assertEqual(infer.await_count, 0)
                with patch.dict(os.environ, dict(MYAGV_CODEX_URL=str(client.make_url('/v1/chat/completions')))):
                    self.assertEqual(await request_json(body(), token, 1), dict(ok=True))
                    infer.side_effect = TimeoutError()
                    with self.assertRaises(web.HTTPBadGateway) as error:
                        await request_json(body(), token, 1)
                    self.assertNotIn(token, error.exception.text)
                with patch.dict(os.environ, dict(MYAGV_CODEX_URL='http://remote.example/v1/chat/completions')):
                    with self.assertRaises(web.HTTPServiceUnavailable):
                        await request_json(body(), token, 1)

    async def test_settings_select_codex_without_openrouter_key(self):
        with tempfile.TemporaryDirectory() as directory, \
             patch('web_backend.server.load_key', return_value=('not_configured', '')), \
             patch('web_backend.server.load_token', return_value=('configured', 'private-token')):
            async with TestClient(TestServer(create_app(directory, Path(directory) / 'stations.json'))) as client:
                settings = await (await client.get('/api/settings')).json()
                values = {**settings['values'], 'llm_model': bridge.MODEL, 'reasoning_effort': 'low'}
                response = await client.put('/api/settings', json=dict(expected_revision=settings['revision'], values=values))
                self.assertEqual(response.status, 200)
                state = await (await client.get('/api/state')).json()
                self.assertEqual(state['llm']['status'], 'configured')
                self.assertEqual(state['llm']['model'], bridge.MODEL)
                self.assertNotIn('private-token', json.dumps(state))
                values['reasoning_effort'] = 'off'
                self.assertEqual((await client.put('/api/settings', json=dict(expected_revision=1, values=values))).status, 400)

    async def test_alignment_passes_selected_model_and_preserves_measurement_guard(self):
        trace = {}
        with patch('web_backend.resolve.request_json', AsyncMock(return_value=dict(goal=[350,780,650,1000], current=[400,600,700,1000]))) as request:
            moves = await locate_pickup(dict(arm='data:image/jpeg;base64,AQ=='), dict(name='item', appearance='plain'),
                'private-token', 'data:image/jpeg;base64,AQ==', [], stage='arm', goal='data:image/jpeg;base64,AQ==',
                model=bridge.MODEL, effort='medium', trace=trace)
        self.assertEqual(request.call_args.args[0]['model'], bridge.MODEL)
        self.assertEqual(request.call_args.args[0]['reasoning'], dict(effort='medium'))
        self.assertEqual(trace['model'], bridge.MODEL)
        self.assertFalse(trace['aligned_perfectly'])
        self.assertGreater(moves['x_mm'], 0)
        self.assertLess(moves['y_mm'], 0)


if __name__ == '__main__':
    unittest.main()
