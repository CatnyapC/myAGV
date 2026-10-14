"""Loopback-only Chat Completions adapter for the local Codex ChatGPT login."""
import asyncio
import base64
import hmac
import json
import os
from pathlib import Path
import shutil
import tempfile
import tomllib

from aiohttp import web

MODEL = 'codex-local/gpt-6.1-sol'
EFFORTS = ('low', 'medium', 'high', 'max')
TIMEOUT_S = 28


def load_token():
    try:
        path = os.environ.get('MYAGV_CODEX_TOKEN_FILE')
        token = Path(path).read_text().strip() if path else ''
        if not token:
            return 'not_configured', ''
        if not 32 <= len(token) <= 1024 or any(not 33 <= ord(c) <= 126 for c in token):
            return 'unavailable', ''
        return 'configured', token
    except (OSError, UnicodeError):
        return 'unavailable', ''


def inputs(body):
    if body.get('model') != MODEL or body.get('stream') is not False:
        raise ValueError('Unsupported Codex model or streaming mode')
    messages = body['messages']
    if len(messages) != 2 or [m['role'] for m in messages] != ['system', 'user']:
        raise ValueError('Expected system and user messages')
    instructions = messages[0]['content']
    if not isinstance(instructions, str) or not 1 <= len(instructions) <= 16000:
        raise ValueError('Invalid instructions')
    content = messages[1]['content']
    if isinstance(content, str):
        content = [dict(type='text', text=content)]
    if not isinstance(content, list) or not 1 <= len(content) <= 32:
        raise ValueError('Invalid user content')
    result, images = [], 0
    for item in content:
        if item['type'] == 'text' and isinstance(item['text'], str) and len(item['text']) <= 100000:
            result.append(dict(type='text', text=item['text']))
        elif item['type'] == 'image_url':
            url = item['image_url']['url']
            if not isinstance(url, str) or len(url) > 6 * 1024 * 1024:
                raise ValueError('Invalid image size')
            prefix, encoded = url.split(',', 1)
            if prefix not in ('data:image/jpeg;base64', 'data:image/png;base64'):
                raise ValueError('Only inline JPEG/PNG images are accepted')
            data = base64.b64decode(encoded, validate=True)
            if not data.startswith(b'\xff\xd8\xff' if 'jpeg' in prefix else b'\x89PNG\r\n\x1a\n'):
                raise ValueError('Invalid image content')
            result.append(dict(type='image', url=url))
            images += 1
        else:
            raise ValueError('Unsupported user content')
    if images > 8:
        raise ValueError('Too many images')
    schema = body['response_format']['json_schema']['schema']
    if not isinstance(schema, dict) or schema.get('type') != 'object' or len(json.dumps(schema)) > 65536:
        raise ValueError('An object output schema is required')
    effort = body.get('reasoning', {}).get('effort', 'low')
    if effort not in EFFORTS:
        raise ValueError('Unsupported Codex reasoning effort')
    return instructions, result, schema, effort


def codex_command():
    binary = shutil.which('codex')
    if not binary:
        raise RuntimeError('Codex CLI not installed')
    command = [binary, 'app-server', '--listen', 'stdio://']
    config = dict(model_provider='openai', web_search='disabled', project_doc_max_bytes=0)
    for feature in ('shell_tool', 'unified_exec', 'apply_patch_freeform', 'apps', 'multi_agent',
                    'hooks', 'remote_plugin', 'shell_snapshot', 'memories'):
        config['features.' + feature] = False
    # Disable inherited integrations without changing the user's Codex config/login.
    path = Path(os.environ.get('CODEX_HOME', str(Path.home() / '.codex'))) / 'config.toml'
    saved = tomllib.loads(path.read_text()) if path.is_file() else {}
    for section in ('mcp_servers', 'plugins'):
        for name in saved.get(section, {}):
            config[section + '.' + name + '.enabled'] = False
    for name, value in config.items():
        command.extend(['-c', name + '=' + json.dumps(value)])
    return command


async def infer(body):
    instructions, content, schema, effort = inputs(body)
    environment = dict(os.environ)
    for name in ('OPENAI_API_KEY', 'CODEX_API_KEY', 'CODEX_ACCESS_TOKEN'):
        environment.pop(name, None)
    # shortcut: one isolated app-server per request; keep it warm if startup dominates latency.
    with tempfile.TemporaryDirectory(prefix='myagv-codex-') as directory:
        process = await asyncio.create_subprocess_exec(*codex_command(), cwd=directory, env=environment,
            stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.DEVNULL, limit=32 * 1024 * 1024)
        try:
            async def send(method, params, identity=None):
                message = dict(method=method, params=params)
                if identity is not None:
                    message['id'] = identity
                process.stdin.write(json.dumps(message).encode() + b'\n')
                await process.stdin.drain()

            async def receive():
                line = await process.stdout.readline()
                if not line:
                    raise RuntimeError('Codex app-server disconnected')
                message = json.loads(line)
                if 'method' in message and 'id' in message:
                    raise RuntimeError('Codex requested an unsupported tool or approval')
                if 'error' in message:
                    raise RuntimeError('Codex rejected the inference request')
                return message

            async def rpc(identity, method, params):
                await send(method, params, identity)
                while True:
                    message = await receive()
                    if message.get('id') == identity:
                        return message['result']

            async with asyncio.timeout(TIMEOUT_S):
                await rpc(1, 'initialize', dict(clientInfo=dict(name='myagv_vision', title='myAGV Vision', version='1.0')))
                await send('initialized', {})
                account = await rpc(2, 'account/read', dict(refreshToken=False))
                if (account.get('account') or {}).get('type') != 'chatgpt':
                    raise RuntimeError('Sign in to Codex CLI with ChatGPT first')
                thread = await rpc(3, 'thread/start', dict(model=body['model'].removeprefix('codex-local/'), cwd=directory,
                    ephemeral=True, approvalPolicy='never', sandbox='read-only',
                    baseInstructions=instructions,
                    developerInstructions='Use only the supplied text and images. Never call tools, execute commands, access files or control hardware. Return only the requested JSON.',
                    serviceName='myagv_vision'))
                thread_id = thread['thread']['id']
                await rpc(4, 'turn/start', dict(threadId=thread_id, input=content, effort=effort, outputSchema=schema))
                answer = None
                while True:
                    message = await receive()
                    params = message.get('params', {})
                    if params.get('threadId') != thread_id:
                        continue
                    if message.get('method') == 'item/completed':
                        item = params['item']
                        if item['type'] == 'agentMessage' and item.get('phase') != 'commentary':
                            answer = item['text']
                        elif item['type'] not in ('userMessage', 'agentMessage', 'reasoning'):
                            raise RuntimeError('Codex returned an unsupported tool action')
                    if message.get('method') == 'turn/completed':
                        if params['turn']['status'] != 'completed' or not answer:
                            code = (params['turn'].get('error') or {}).get('codexErrorInfo', 'unknown')
                            code = code if isinstance(code, str) and code.isalnum() else 'unknown'
                            raise RuntimeError('Codex inference failed (' + code + ')')
                        if len(answer.encode()) > 65536 or not isinstance(json.loads(answer), dict):
                            raise RuntimeError('Invalid Codex JSON result')
                        return dict(choices=[dict(finish_reason='stop', message=dict(content=answer))])
        finally:
            if process.returncode is None:
                process.kill()
            await process.wait()


def create_app():
    status, token = load_token()
    if status != 'configured':
        raise RuntimeError('Set MYAGV_CODEX_TOKEN_FILE to a protected token file')
    lock = asyncio.Lock()

    @web.middleware
    async def boundary(request, handler):
        if request.headers.get('Origin') or not hmac.compare_digest(request.headers.get('Authorization', '').encode(), ('Bearer ' + token).encode()):
            raise web.HTTPUnauthorized(text='Codex bridge authorization required')
        try:
            return await handler(request)
        except (ValueError, KeyError, TypeError, AttributeError):
            raise web.HTTPBadRequest(text='Invalid Codex bridge request') from None
        except TimeoutError:
            raise web.HTTPGatewayTimeout(text='Codex timed out; retry explicitly') from None
        except RuntimeError as exc:
            raise web.HTTPBadGateway(text=str(exc)) from None
        except OSError:
            raise web.HTTPBadGateway(text='Codex process unavailable') from None

    async def complete(request):
        if lock.locked():
            raise web.HTTPTooManyRequests(text='Codex inference already running')
        async with lock:
            return web.json_response(await infer(await request.json()))

    async def health(request):
        return web.json_response(dict(status='configured', model=MODEL))

    app = web.Application(middlewares=[boundary], client_max_size=32 * 1024 * 1024)
    app.router.add_post('/v1/chat/completions', complete)
    app.router.add_get('/healthz', health)
    return app


if __name__ == '__main__':
    web.run_app(create_app(), host='127.0.0.1', port=9008, access_log=None, handler_cancellation=True)
