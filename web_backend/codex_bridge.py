"""Loopback-only Chat Completions adapter for the local Codex ChatGPT login."""
import asyncio
import base64
import hmac
import json
import os
import re
from pathlib import Path
import shutil
import tempfile
import tomllib

from aiohttp import web
from fetch_calibration import FETCH_VISION_TIMEOUT_S

MODEL = 'codex-local/gpt-6.1-sol'
EFFORTS = ('low', 'medium', 'high', 'max')
TIMEOUT_S = FETCH_VISION_TIMEOUT_S - 5


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
    config = dict(model_provider='openai', service_tier='fast', web_search='disabled', project_doc_max_bytes=0)
    config['features.fast_mode'] = True
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


class CodexConversation:
    def __init__(self):
        self.process = None
        self.directory = None
        self.thread_id = None
        self.instructions = None
        self.identity = 0
        self.sequence = None
        self.last_used = 0
        self.service_tier = None
        self.phase = 'startup'
        self.turn_id = None
        self.last_event = None

    async def close(self):
        process, self.process = self.process, None
        if process:
            if process.returncode is None:
                process.kill()
            await process.wait()
        if self.directory:
            self.directory.cleanup()
            self.directory = None

    async def send(self, method, params, identity=None):
        message = dict(method=method, params=params)
        if identity is not None:
            message['id'] = identity
        self.process.stdin.write(json.dumps(message).encode() + b'\n')
        await self.process.stdin.drain()

    async def receive(self):
        line = await self.process.stdout.readline()
        if not line:
            raise RuntimeError('Codex app-server disconnected')
        message = json.loads(line)
        self.last_event = message.get('method', 'rpc_response')
        if 'method' in message and 'id' in message:
            raise RuntimeError('Codex requested an unsupported tool or approval')
        if 'error' in message:
            raise RuntimeError('Codex rejected the inference request')
        return message

    async def rpc(self, method, params):
        self.phase = method
        self.identity += 1
        identity = self.identity
        await self.send(method, params, identity)
        while True:
            message = await self.receive()
            if message.get('id') == identity:
                return message['result']

    async def run(self, body):
        instructions, content, schema, effort = inputs(body)
        async with asyncio.timeout(TIMEOUT_S):
            if self.process is None:
                environment = dict(os.environ)
                for name in ('OPENAI_API_KEY', 'CODEX_API_KEY', 'CODEX_ACCESS_TOKEN'):
                    environment.pop(name, None)
                self.directory = tempfile.TemporaryDirectory(prefix='myagv-codex-')
                self.process = await asyncio.create_subprocess_exec(*codex_command(), cwd=self.directory.name, env=environment,
                    stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.DEVNULL, limit=32 * 1024 * 1024)
                await self.rpc('initialize', dict(clientInfo=dict(name='myagv_vision', title='myAGV Vision', version='1.0')))
                await self.send('initialized', {})
                account = await self.rpc('account/read', dict(refreshToken=False))
                if (account.get('account') or {}).get('type') != 'chatgpt':
                    raise RuntimeError('Sign in to Codex CLI with ChatGPT first')
                thread = await self.rpc('thread/start', dict(model=body['model'].removeprefix('codex-local/'), cwd=self.directory.name,
                    ephemeral=True, approvalPolicy='never', sandbox='read-only', serviceTier='fast',
                    baseInstructions=instructions,
                    developerInstructions='Use only the supplied text and images. Never call tools, execute commands, access files or control hardware. Return only the requested JSON.',
                    serviceName='myagv_vision'))
                self.thread_id = thread['thread']['id']
                self.service_tier = thread.get('serviceTier')
                self.instructions = instructions
            elif instructions != self.instructions:
                raise ValueError('Session instructions changed; restart ALIGN')
            turn = await self.rpc('turn/start', dict(threadId=self.thread_id, input=content, effort=effort,
                outputSchema=schema, serviceTier='fast'))
            turn_id = turn['turn']['id']
            self.turn_id = turn_id
            self.phase = 'inference'
            answer, usage = None, None
            while True:
                message = await self.receive()
                params = message.get('params', {})
                if params.get('threadId') != self.thread_id:
                    continue
                if params.get('turnId', turn_id) != turn_id:
                    continue
                if message.get('method') == 'thread/tokenUsage/updated':
                    usage = params.get('tokenUsage', {}).get('last')
                if message.get('method') == 'item/completed':
                    item = params['item']
                    if item['type'] == 'agentMessage' and item.get('phase') != 'commentary':
                        answer = item['text']
                    elif item['type'] not in ('userMessage', 'agentMessage', 'reasoning'):
                        raise RuntimeError('Codex returned an unsupported tool action')
                if message.get('method') == 'turn/completed':
                    if params['turn']['id'] != turn_id:
                        continue
                    if params['turn']['status'] != 'completed' or not answer:
                        raise RuntimeError('Codex inference failed; restart ALIGN')
                    if len(answer.encode()) > 65536 or not isinstance(json.loads(answer), dict):
                        raise RuntimeError('Invalid Codex JSON result')
                    self.last_used = asyncio.get_running_loop().time()
                    result = dict(choices=[dict(finish_reason='stop', message=dict(content=answer))],
                        codex=dict(thread_id=self.thread_id, turn_id=turn_id, service_tier=self.service_tier))
                    if usage:
                        result['usage'] = {k: v for k, v in usage.items() if type(v) is int and v >= 0}
                    return result


async def infer(body, conversation=None):
    owned = conversation is None
    conversation = conversation or CodexConversation()
    started = asyncio.get_running_loop().time()
    try:
        return await conversation.run(body)
    except TimeoutError:
        diagnostic = dict(event='codex_timeout', phase=conversation.phase,
            last_event=conversation.last_event, thread_id=conversation.thread_id, turn_id=conversation.turn_id,
            timeout_s=TIMEOUT_S, elapsed_s=round(asyncio.get_running_loop().time()-started,2))
        print(json.dumps({k:v for k,v in diagnostic.items() if v is not None}), flush=True)
        await conversation.close()
        raise
    except BaseException:
        await conversation.close()
        raise
    finally:
        if owned:
            await conversation.close()


def create_app():
    status, token = load_token()
    if status != 'configured':
        raise RuntimeError('Set MYAGV_CODEX_TOKEN_FILE to a protected token file')
    lock = asyncio.Lock()
    sessions = {}

    async def close_session(identity):
        conversation = sessions.pop(identity, None)
        if conversation:
            await conversation.close()

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
            body = await request.json()
            session = body.get('session')
            if session is None:
                return web.json_response(await infer(body))
            identity, sequence = session['id'], session['sequence']
            if (not isinstance(identity, str) or not re.fullmatch(r'[A-Za-z0-9:_-]{1,160}', identity)
                    or type(sequence) is not int or not 1 <= sequence <= 33 or type(session['start']) is not bool):
                raise ValueError('Invalid ALIGN session')
            if session['start']:
                if identity in sessions:
                    raise web.HTTPConflict(text='ALIGN session already exists')
                for old in list(sessions):
                    await close_session(old)
                sessions[identity] = CodexConversation()
            conversation = sessions.get(identity)
            if conversation is None or (conversation.sequence is not None and sequence != conversation.sequence + 1):
                raise web.HTTPConflict(text='ALIGN session missing or sequence changed; restart ALIGN')
            try:
                result = await infer(body, conversation)
                conversation.sequence = sequence
                return web.json_response(result)
            except BaseException:
                await close_session(identity)
                raise

    async def delete_session(request):
        await close_session(request.match_info['id'])
        return web.Response(status=204)

    async def cleanup(application):
        async def reap():
            while True:
                await asyncio.sleep(1)
                now = asyncio.get_running_loop().time()
                for identity, conversation in list(sessions.items()):
                    if not lock.locked() and now - conversation.last_used > 180:
                        await close_session(identity)
        task = asyncio.create_task(reap())
        try:
            yield
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
            for identity in list(sessions):
                await close_session(identity)

    async def health(request):
        return web.json_response(dict(status='configured', model=MODEL))

    app = web.Application(middlewares=[boundary], client_max_size=32 * 1024 * 1024)
    app.router.add_post('/v1/chat/completions', complete)
    app.router.add_delete('/v1/sessions/{id}', delete_session)
    app.router.add_get('/healthz', health)
    app.cleanup_ctx.append(cleanup)
    return app


if __name__ == '__main__':
    web.run_app(create_app(), host='127.0.0.1', port=9008, access_log=None, handler_cancellation=True)
