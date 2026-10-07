"""Loopback-only Demo API. No robot motion or ROS connection is available."""
import argparse
import asyncio
from contextlib import suppress
import json
import math
import os
from pathlib import Path
import secrets
import tempfile
import time
import uuid
from urllib.parse import urlsplit

from aiohttp import web
from .map_data import demo_map, dominant_angle, render_map, validate_rectangle
from .simulation import Simulation, plan_path, validate_pose

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SETTINGS = dict(auto_align=True, manual_angle_deg=0)


def atomic_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(dir=path.parent, prefix='.write-')
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(value, stream, allow_nan=False, indent=2)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def load_json(path, fallback):
    return json.loads(path.read_text()) if path.exists() else fallback


def validate_settings(value):
    if not isinstance(value, dict) or set(value) != set(DEFAULT_SETTINGS):
        raise ValueError('Invalid settings fields')
    angle = value['manual_angle_deg']
    if type(value['auto_align']) is not bool or isinstance(angle, bool) or not isinstance(angle, (int, float)) or not math.isfinite(angle) or not -180 <= angle <= 180:
        raise ValueError('Invalid display settings')
    return value


class Editor:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.grid = demo_map()
        self.settings = load_json(self.directory / 'settings.json', dict(revision=0, values=DEFAULT_SETTINGS.copy()))
        validate_settings(self.settings['values'])
        self.zones = load_json(self.directory / 'no_go_zones.json', dict(map_id=self.grid['map_id'], revision=0, zones=[]))
        if self.zones['map_id'] != self.grid['map_id']:
            raise ValueError('Saved zones belong to another map; explicit review required')
        for zone in self.zones['zones']:
            validate_rectangle(zone['corners'], self.grid)
        self.detected = dominant_angle(self.grid)
        self.metadata, self.png = render_map(self.grid, self.settings['values'], self.settings['revision'], self.detected)
        self.lease = None
        self.expires = 0
        self.sockets = set()
        self.lock = asyncio.Lock()
        self.replies = {}  # Bounded retry cache; duplicate command IDs never add another zone.
        self.sim = Simulation(self.grid, self.zones)
        self.cost_png = None

    def state(self):
        return dict(robot_id='local-demo', demo=True, phase=self.sim.phase, motion_available=False,
                    simulation_available=True, cameras_available=False, localization_available=False,
                    zone_enforcement='simulated-global', operator_active=bool(self.lease and time.monotonic() < self.expires),
                    navigation=self.sim.telemetry())

    def leased(self, request):
        return self.lease and time.monotonic() < self.expires and secrets.compare_digest(request.headers.get('X-Control-Lease', ''), self.lease)

    def require_lease(self, request):
        if not self.leased(request):
            raise web.HTTPForbidden(text='Operator lease required')

    async def emit(self, kind):
        for socket in tuple(self.sockets):
            if not socket.closed:
                try:
                    message = dict(type=kind)
                    if kind == 'telemetry':
                        message['state'] = self.state()
                    await asyncio.wait_for(socket.send_json(message), .2)
                except (ConnectionError, asyncio.TimeoutError):
                    self.sockets.discard(socket)
                    if socket.operator_lease and socket.operator_lease == self.lease:
                        self.sim.stop('Telemetry unavailable')
                        self.lease, self.expires = None, 0
                    with suppress(ConnectionError, asyncio.TimeoutError):
                        await asyncio.wait_for(socket.close(code=1001, drain=False), .2)


EDITOR = web.AppKey('editor', Editor)


@web.middleware
async def boundary(request, handler):
    if request.url.host not in ('localhost', '127.0.0.1', '::1'):
        raise web.HTTPForbidden(text='Local demo only')
    origin = request.headers.get('Origin')
    if origin:
        parsed = urlsplit(origin)
        allowed = {request.host, '127.0.0.1:5173', 'localhost:5173'}
        if parsed.scheme != 'http' or parsed.netloc not in allowed:
            raise web.HTTPForbidden(text='Origin rejected')
    try:
        response = await handler(request)
    except ValueError as exc:
        response = web.json_response(dict(error=str(exc)), status=400)
    except (TypeError, KeyError):
        response = web.json_response(dict(error='Invalid request fields'), status=400)
    except OSError:
        response = web.json_response(dict(error='Storage operation failed; changes were not saved'), status=500)
    except web.HTTPException as exc:
        response = web.json_response(dict(error=exc.text), status=exc.status)
    response.headers['Cache-Control'] = 'no-store' if request.path.startswith('/api/') else 'no-cache'
    response.headers['X-Content-Type-Options'] = 'nosniff'
    return response


def create_app(directory=ROOT / 'web_runtime'):
    app = web.Application(middlewares=[boundary], client_max_size=32 * 1024)
    editor = Editor(directory)
    app[EDITOR] = editor

    async def state(request):
        return web.json_response(editor.state())

    async def map_info(request):
        return web.json_response(editor.metadata)

    async def map_image(request):
        if request.query.get('view_revision') != str(editor.metadata['display']['view_revision']):
            raise web.HTTPConflict(text='Display revision changed; reload map')
        return web.Response(body=editor.png, content_type='image/png')

    async def zones(request):
        return web.json_response(editor.zones)

    async def costmap(request):
        grid = editor.sim.navigation if request.path == '/api/navigation-map' else editor.sim.costmap
        return web.json_response({**grid, 'zone_revision': editor.sim.revision, 'source': 'simulation'})

    async def costmap_image(request):
        view, zone = editor.settings['revision'], editor.sim.revision
        if request.query.get('view_revision') != str(view) or request.query.get('zone_revision') != str(zone):
            raise web.HTTPConflict(text='Costmap revision changed; reload')
        if not editor.cost_png or editor.cost_png[0] != (view, zone):
            _, png = await asyncio.to_thread(render_map, editor.sim.costmap, editor.settings['values'], view, editor.detected,
                                             {-1: b'\x00\x00\x00\x00', 0: b'\x00\x00\x00\x00', 100: b'\xe5\x48\x4d\x60'})
            if (view, zone) != (editor.settings['revision'], editor.sim.revision):
                raise web.HTTPConflict(text='Costmap revision changed; reload')
            editor.cost_png = ((view, zone), png)
        return web.Response(body=editor.cost_png[1], content_type='image/png')

    async def settings(request):
        if request.method == 'GET':
            return web.json_response(editor.settings)
        data = await request.json()
        async with editor.lock:
            editor.require_lease(request)
            if data['expected_revision'] != editor.settings['revision']:
                raise web.HTTPConflict(text='Settings changed; reload before editing')
            values = validate_settings(data['values'])
            saved = dict(revision=editor.settings['revision'] + 1, values=values)
            metadata, png = await asyncio.to_thread(render_map, editor.grid, values, saved['revision'], editor.detected)
            # The thread yielded; recheck ownership before committing a shared edit.
            editor.require_lease(request)
            atomic_json(editor.directory / 'settings.json', saved)
            editor.settings, editor.metadata, editor.png = saved, metadata, png
        await editor.emit('settings')
        return web.json_response(saved)

    async def control(request):
        async with editor.lock:
            action = request.match_info['action']
            if action == 'claim':
                if editor.lease and time.monotonic() < editor.expires:
                    raise web.HTTPConflict(text='Another operator owns editing')
                if editor.lease:
                    editor.sim.stop('Lease expired')
                editor.lease = secrets.token_urlsafe(32)
            else:
                editor.require_lease(request)
            if action == 'release':
                editor.sim.stop('Ownership released')
                editor.lease, editor.expires = None, 0
                reply = dict(released=True)
            else:
                editor.expires = time.monotonic() + 30
                reply = dict(lease=editor.lease, expires_in_s=30)
        await editor.emit('state')
        return web.json_response(reply)

    async def commands(request):
        data = await request.json()
        async with editor.lock:
            editor.require_lease(request)
            command_id = data['id']
            if not isinstance(command_id, str) or not 1 <= len(command_id) <= 100:
                raise ValueError('Invalid command ID')
            cache_key = (editor.lease, command_id)
            fingerprint = json.dumps(data, sort_keys=True, allow_nan=False)
            if cache_key in editor.replies:
                old, reply = editor.replies[cache_key]
                if old != fingerprint:
                    raise web.HTTPConflict(text='Command ID reused with different arguments')
                return web.json_response(reply)
            if data['map_id'] != editor.grid['map_id'] or data['expected_revision'] != editor.zones['revision']:
                raise web.HTTPConflict(text='Map or zones changed; reload before editing')
            kind = data['type']
            if kind == 'cancel':
                editor.sim.stop('Cancelled')
            elif kind in ('navigate', 'demo_reset'):
                if editor.sim.phase != 'idle':
                    raise web.HTTPConflict(text='Stop the simulated task first')
                if kind == 'demo_reset':
                    editor.sim.reset()
                else:
                    if data.get('frame') != editor.grid['frame']:
                        raise ValueError('Goal frame must match the map frame')
                    goal = validate_pose(data['goal'])
                    generation = editor.sim.generation
                    editor.sim.phase, editor.sim.status = 'planning', 'Planning'
                    try:
                        path = await asyncio.to_thread(plan_path, editor.sim.costmap, dict(editor.sim.pose), goal)
                        editor.require_lease(request)
                        if generation != editor.sim.generation:
                            raise web.HTTPConflict(text='Navigation cancelled while planning')
                        editor.sim.start(goal, path)
                    except BaseException:
                        if generation == editor.sim.generation:
                            editor.sim.stop('Planning failed')
                        raise
            elif kind not in ('zone_add', 'zone_delete'):
                raise web.HTTPBadRequest(text='Unsupported Demo command')
            if kind in ('navigate', 'cancel', 'demo_reset'):
                reply = dict(id=command_id, status='running' if kind == 'navigate' else 'completed', navigation=editor.sim.telemetry())
                editor.replies[cache_key] = fingerprint, reply
                if len(editor.replies) > 200:
                    del editor.replies[next(iter(editor.replies))]
                await editor.emit('telemetry')
                return web.json_response(reply)
            if editor.sim.phase != 'idle':
                raise web.HTTPConflict(text='No-go edits require a stopped simulator')
            current = list(editor.zones['zones'])
            if data['type'] == 'zone_add':
                if len(current) >= 100:
                    raise web.HTTPBadRequest(text='Zone limit reached')
                current.append(dict(id=str(uuid.uuid4()), corners=validate_rectangle(data['corners'], editor.grid)))
            elif data['type'] == 'zone_delete':
                current = [z for z in current if z['id'] != data['zone_id']]
                if len(current) == len(editor.zones['zones']):
                    raise web.HTTPNotFound(text='Zone not found')
            saved = dict(map_id=editor.grid['map_id'], revision=editor.zones['revision'] + 1, zones=current)
            navigation, global_grid = await asyncio.to_thread(editor.sim.rebuild, saved, False)
            editor.require_lease(request)
            atomic_json(editor.directory / 'no_go_zones.json', saved)
            editor.zones = saved
            editor.sim.navigation, editor.sim.costmap, editor.sim.revision = navigation, global_grid, saved['revision']
            editor.sim.goal = None
            reply = dict(id=command_id, status='completed', zones=saved, enforcement='simulated-global')
            editor.replies[cache_key] = fingerprint, reply
            if len(editor.replies) > 200:
                del editor.replies[next(iter(editor.replies))]
        await editor.emit('zones')
        return web.json_response(reply)

    async def stop(request):
        editor.sim.stop('Stopped')
        await editor.emit('telemetry')
        return web.json_response(dict(status='completed', reason='Simulation stopped; hardware is not connected'))

    async def events(request):
        socket = web.WebSocketResponse(heartbeat=20)
        socket.operator_lease = None
        await socket.prepare(request)
        editor.sockets.add(socket)
        await socket.send_json(dict(type='state'))
        try:
            async for message in socket:
                if message.type == web.WSMsgType.TEXT:
                    try:
                        data = json.loads(message.data)
                        token = data.get('lease')
                        if data.get('type') == 'operator' and isinstance(token, str) and editor.lease and time.monotonic() < editor.expires and secrets.compare_digest(token, editor.lease):
                            socket.operator_lease = token
                    except (ValueError, AttributeError):
                        pass
        finally:
            editor.sockets.discard(socket)
            if socket.operator_lease and socket.operator_lease == editor.lease:
                editor.sim.stop('Operator disconnected')
                editor.lease, editor.expires = None, 0
                await editor.emit('telemetry')
        return socket

    async def shutdown(application):
        for socket in tuple(editor.sockets):
            await socket.close(code=1001, message=b'Service stopping')

    async def lease_watch(application):
        async def expire():
            while True:
                await asyncio.sleep(.1)
                async with editor.lock:
                    expired = bool(editor.lease and time.monotonic() >= editor.expires)
                    if expired:
                        editor.sim.stop('Lease expired')
                        editor.lease, editor.expires = None, 0
                    running = editor.sim.phase == 'running'
                    if running:
                        editor.sim.advance(.1)
                if expired or running:
                    await editor.emit('telemetry')
        task = asyncio.create_task(expire())
        yield
        task.cancel()
        with suppress(asyncio.CancelledError):
            await task

    async def static(request):
        dist = ROOT / 'web' / 'dist'
        path = (dist / request.match_info['path']).resolve()
        if not path.is_relative_to(dist.resolve()):
            raise web.HTTPForbidden(text='Invalid asset path')
        if path.is_file():
            return web.FileResponse(path)
        if request.match_info['path'] == '' and (dist / 'index.html').is_file():
            return web.FileResponse(dist / 'index.html')
        raise web.HTTPNotFound(text='Build frontend first, or use the PM2 development frontend on port 5173')

    app.router.add_get('/api/state', state)
    app.router.add_get('/api/map', map_info)
    app.router.add_get('/api/map.png', map_image)
    app.router.add_get('/api/no-go-zones', zones)
    app.router.add_get('/api/navigation-map', costmap)
    app.router.add_get('/api/global-costmap', costmap)
    app.router.add_get('/api/global-costmap.png', costmap_image)
    app.router.add_get('/api/settings', settings)
    app.router.add_put('/api/settings', settings)
    app.router.add_post('/api/control/{action:claim|renew|release}', control)
    app.router.add_post('/api/commands', commands)
    app.router.add_post('/api/stop', stop)
    app.router.add_get('/api/events', events)
    app.router.add_get('/{path:.*}', static)
    app.on_shutdown.append(shutdown)
    app.cleanup_ctx.append(lease_watch)
    return app


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8791)
    parser.add_argument('--data-dir', type=Path, default=ROOT / 'web_runtime')
    args = parser.parse_args()
    web.run_app(create_app(args.data_dir), host='127.0.0.1', port=args.port)
