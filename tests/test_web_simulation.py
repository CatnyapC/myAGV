import asyncio
import math
import random
import tempfile
import unittest

from aiohttp.test_utils import TestClient, TestServer

from web_backend.map_data import demo_map, local_to_world, world_to_local
from web_backend.server import create_app
from web_backend.simulation import Simulation, cell_index, global_costmap, navigation_grid, plan_path, segment_clear


class SimulationTest(unittest.TestCase):
    def test_inflation_matches_geometric_clearance_without_changing_source(self):
        rng = random.Random(42)
        width, height = 17, 13
        for density in (0, .05, .5, .95, 1):
            source = [rng.choice((-1, 1, 50, 99, 100)) if rng.random() < density else 0
                      for _ in range(width * height)]
            obstacles = [(i % width, i // width) for i, value in enumerate(source) if value != 0]
            for resolution in (.05, .13, .4):
                for radius in (0, .2, .61):
                    grid = dict(width=width, height=height, resolution_m=resolution,
                                origin=dict(x_m=-2, y_m=4, yaw_rad=.29), cells=list(source))
                    margin = radius + math.sqrt(2) * resolution
                    expected = list(source)
                    for i, value in enumerate(source):
                        if value != 0:
                            continue
                        x, y = i % width, i // width
                        boundary = min(x + .5, width - x - .5, y + .5, height - y - .5) * resolution
                        if boundary <= margin or any(math.hypot((x-ox)*resolution, (y-oy)*resolution) <= margin
                                                     for ox, oy in obstacles):
                            expected[i] = 100
                    with self.subTest(density=density, resolution=resolution, radius=radius):
                        self.assertEqual(global_costmap(grid, radius), {**grid, 'cells': expected})
                        self.assertEqual(grid['cells'], source)

    def test_path_errors_identify_blocked_or_outside_endpoint(self):
        grid = dict(width=12, height=12, resolution_m=.1,
                    origin=dict(x_m=-2, y_m=4, yaw_rad=.29), cells=[0] * 144)
        def pose(x, y):
            wx, wy = local_to_world([x, y], grid['origin'])
            return dict(x_m=wx, y_m=wy)
        start, goal = pose(.45, .45), pose(.85, .85)
        for label, endpoint, index in (('Start pose', start, 4 * 12 + 4), ('Goal pose', goal, 8 * 12 + 8)):
            for cost in (-1, 100):
                blocked = {**grid, 'cells': list(grid['cells'])}
                blocked['cells'][index] = cost
                with self.subTest(label=label, cost=cost), self.assertRaises(ValueError) as error:
                    plan_path(blocked, start, goal)
                self.assertIn(f'{label} blocked by global costmap', str(error.exception))
                self.assertIn(f"x={endpoint['x_m']:.3f} m, y={endpoint['y_m']:.3f} m", str(error.exception))
                self.assertIn(f'cell {index % 12}, {index // 12}', str(error.exception))
                self.assertIn(f'cost={cost}', str(error.exception))
        for label, a, b in (('Start pose', pose(-.05, .45), goal), ('Goal pose', start, pose(1.25, .85))):
            with self.subTest(label=label), self.assertRaisesRegex(ValueError, f'{label} outside map bounds'):
                plan_path(grid, a, b)

    def test_direct_route_has_no_grid_heading_changes(self):
        grid = dict(map_id='test', frame='map', width=60, height=40, resolution_m=.1,
                    origin=dict(x_m=-2, y_m=4, yaw_rad=.29), cells=[0]*2400)
        def pose(x, y):
            wx, wy = local_to_world([x, y], grid['origin'])
            return dict(x_m=wx, y_m=wy, yaw_rad=.29)
        start, goal = pose(.83, .92), pose(4.37, 2.16)
        heading = math.atan2(goal['y_m']-start['y_m'], goal['x_m']-start['x_m'])
        start['yaw_rad'] = goal['yaw_rad'] = heading
        sim = Simulation(grid, dict(revision=0, zones=[]))
        path = plan_path(sim.costmap, start, goal)
        self.assertEqual(path, [[start['x_m'], start['y_m']], [goal['x_m'], goal['y_m']]])
        sim.pose = dict(start)
        sim.start(goal, path)
        for _ in range(100):
            sim.advance(.1)
            self.assertAlmostEqual(sim.pose['yaw_rad'], heading)
            if sim.phase == 'idle':
                break
        self.assertEqual(sim.status, 'Arrived')
        # Same-cell goals must not visit the cell center first.
        nearby = pose(.84, .93)
        self.assertEqual(plan_path(sim.costmap, start, nearby),
                         [[start['x_m'], start['y_m']], [nearby['x_m'], nearby['y_m']]])
        # A valid detour exists, but the direct diagonal touches an occupied corner.
        tiny = {**grid, 'width': 3, 'height': 3, 'cells': [0,100,0,0,0,0,0,0,0]}
        a, b = pose(.05, .05), pose(.25, .25)
        route = plan_path(tiny, a, b)
        self.assertFalse(segment_clear(tiny, route[0], route[-1]))
        self.assertGreater(len(route), 2)
        self.assertTrue(all(segment_clear(tiny, p, q) for p, q in zip(route, route[1:])))
        # Edge grazing and unknown cells must also reject shortcuts.
        edge = [local_to_world(p, grid['origin']) for p in [(.1, .02), (.1, .08)]]
        self.assertFalse(segment_clear(tiny, *edge))
        unknown = {**tiny, 'cells': [0,-1,0,0,0,0,0,0,0]}
        self.assertFalse(segment_clear(unknown, route[0], route[-1]))

    def test_costmap_detour_delete_and_simulated_arrival(self):
        grid = dict(map_id='test', frame='map', revision=1, width=40, height=30, resolution_m=.1,
                    origin=dict(x_m=-2, y_m=4, yaw_rad=.29), cells=[0]*1200)
        def pose(x, y, yaw=0):
            wx, wy = local_to_world([x, y], grid['origin'])
            return dict(x_m=wx, y_m=wy, yaw_rad=yaw)
        start, goal = pose(.85, 1.55, .29), pose(3.15, 1.55, -1)
        corners = [local_to_world(p, grid['origin']) for p in [(1.8,1), (2.2,1), (2.2,2), (1.8,2)]]
        zones = dict(revision=1, zones=[dict(id='zone', corners=corners)])
        navigation = navigation_grid(grid, zones['zones'])
        self.assertEqual(grid['cells'], [0]*1200)  # Never overwrite the source.
        self.assertEqual(navigation['cells'][15*40+20], 100)
        costmap = global_costmap(navigation)
        route = plan_path(costmap, start, goal)
        for point in route:
            cell_index(costmap, dict(x_m=point[0], y_m=point[1]))
        samples = [[p[0]+(q[0]-p[0])*t/100, p[1]+(q[1]-p[1])*t/100]
                   for p, q in zip(route, route[1:]) for t in range(101)]
        for point in samples:
            cell_index(costmap, dict(x_m=point[0], y_m=point[1]))
        projected = [world_to_local(p, grid['origin']) for p in samples]
        self.assertTrue(any(1.8 < x < 2.2 and (y < .6 or y > 2.4) for x, y in projected))
        with self.assertRaisesRegex(ValueError, 'blocked'):
            plan_path(costmap, start, pose(2, 1.5))
        restored = navigation_grid(grid, [])
        self.assertEqual(restored['cells'], grid['cells'])
        direct = plan_path(global_costmap(restored), start, goal)
        self.assertEqual(direct, [[start['x_m'], start['y_m']], [goal['x_m'], goal['y_m']]])
        self.assertLess(len(direct), len(route))
        barrier = [local_to_world(p, grid['origin']) for p in [(1.8,0), (2.2,0), (2.2,3), (1.8,3)]]
        with self.assertRaisesRegex(ValueError, 'No path'):
            plan_path(global_costmap(navigation_grid(grid, [dict(corners=barrier)])), start, goal)
        tiny = {**grid, 'width': 2, 'height': 2, 'cells': [0,100,100,0]}
        with self.assertRaisesRegex(ValueError, 'No path'):
            plan_path(tiny, pose(.05,.05), pose(.15,.15))  # No diagonal corner cutting.
        sim = Simulation(grid, zones)
        sim.pose = dict(start)
        sim.start(goal, route)
        for _ in range(2000):
            sim.advance(.1)
            cell_index(sim.costmap, sim.pose)
            if sim.phase == 'idle':
                break
        self.assertEqual(sim.status, 'Arrived')
        self.assertAlmostEqual(sim.pose['x_m'], goal['x_m'])
        self.assertAlmostEqual(sim.pose['y_m'], goal['y_m'])
        self.assertLess(abs(sim.pose['yaw_rad'] - goal['yaw_rad']), .04)
        self.assertEqual(sim.telemetry()['path'], [])


class SimulationAPITest(unittest.IsolatedAsyncioTestCase):
    async def test_navigation_guards_stop_cancel_and_disconnect(self):
        asyncio.get_running_loop().set_debug(False)
        with tempfile.TemporaryDirectory() as directory:
            async with TestClient(TestServer(create_app(directory))) as client:
                state = await (await client.get('/api/state')).json()
                self.assertFalse(state['motion_available'])
                self.assertEqual(state['navigation']['source'], 'simulation')
                grid = demo_map()
                nav = state['navigation']
                px, py = nav['pose']['x_m'], nav['pose']['y_m']
                overlap = dict(id='overlap', type='zone_add', map_id=grid['map_id'], expected_revision=0,
                               corners=[[px-.1,py-.1],[px+.1,py-.1],[px+.1,py+.1],[px-.1,py+.1]])
                self.assertEqual((await client.post('/api/commands', json=overlap)).status, 400)
                global_grid = await (await client.get('/api/global-costmap')).json()
                free = [(i, value) for i, value in enumerate(global_grid['cells']) if value == 0 and grid['cells'][i] == 0]
                goal_index = max(free, key=lambda entry: entry[0])[0]
                r = grid['resolution_m']
                gx, gy = local_to_world([(goal_index % grid['width'] + .5)*r, (goal_index // grid['width'] + .5)*r], grid['origin'])
                command = dict(id='go', type='navigate', map_id=grid['map_id'], frame='map', expected_revision=0,
                               goal=dict(x_m=gx, y_m=gy, yaw_rad=-.5))
                bad = {**command, 'id': 'bad', 'goal': dict(x_m=999, y_m=999, yaw_rad=0)}
                self.assertEqual((await client.post('/api/commands', json=bad)).status, 400)
                async with client.ws_connect('/api/events') as socket:
                    await socket.receive_json()
                    response = await client.post('/api/commands', json=command)
                    self.assertEqual(response.status, 200, await response.text())
                    self.assertEqual((await response.json())['status'], 'running')
                    packet = await socket.receive_json(timeout=2)
                    while packet['type'] != 'telemetry':
                        packet = await socket.receive_json(timeout=2)
                    self.assertTrue(packet['state']['navigation']['moving'])
                    self.assertGreater(len(packet['state']['navigation']['path']), 2)
                    async with client.ws_connect('/api/events') as second_socket:
                        await second_socket.receive_json()
                    state = await (await client.get('/api/state')).json()
                    self.assertEqual(state['phase'], 'running')
                    self.assertEqual((await client.post('/api/commands', json=overlap)).status, 409)
                    repeat = await client.post('/api/commands', json=command)
                    self.assertEqual(repeat.status, 200)
                    await client.post('/api/stop')
                    stopped = await (await client.get('/api/state')).json()
                    self.assertEqual(stopped['phase'], 'idle')
                    self.assertEqual(stopped['navigation']['path'], [])
                    command['id'] = 'go-again'
                    self.assertEqual((await client.post('/api/commands', json=command)).status, 200)
                # Closing the last UI connection stops navigation, including across reconnects.
                for _ in range(20):
                    state = await (await client.get('/api/state')).json()
                    if state['phase'] == 'idle':
                        break
                    await asyncio.sleep(.01)
                self.assertEqual(state['navigation']['status'], 'UI disconnected')
                self.assertFalse(state['navigation']['moving'])
                async with client.ws_connect('/api/events') as socket:
                    await socket.receive_json()
                    # A retried command cannot restart navigation after reconnecting.
                    self.assertEqual((await client.post('/api/commands', json=command)).status, 200)
                    state = await (await client.get('/api/state')).json()
                    self.assertEqual(state['phase'], 'idle')
                    self.assertEqual(state['navigation']['path'], [])
                    command['id'] = 'go-cancel'
                    self.assertEqual((await client.post('/api/commands', json=command)).status, 200)
                    cancel = dict(id='cancel', type='cancel', map_id=grid['map_id'], expected_revision=0)
                    self.assertEqual((await client.post('/api/commands', json=cancel)).status, 200)
                    state = await (await client.get('/api/state')).json()
                    self.assertEqual(state['navigation']['status'], 'Cancelled')
                    self.assertEqual(state['navigation']['path'], [])
                    self.assertFalse(state['navigation']['moving'])
                image = await client.get('/api/global-costmap.png?view_revision=0&zone_revision=0')
                self.assertEqual(image.status, 200)
                png = await image.read()
                self.assertEqual(png[25], 6)  # Transparent RGBA costmap.
                self.assertEqual((await client.get('/api/global-costmap.png?view_revision=0&zone_revision=99')).status, 409)


if __name__ == '__main__':
    unittest.main()
