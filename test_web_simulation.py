import asyncio
import math
import tempfile
import unittest

from aiohttp.test_utils import TestClient, TestServer

from web_backend.map_data import demo_map, local_to_world, world_to_local
from web_backend.server import create_app
from web_backend.simulation import Simulation, cell_index, global_costmap, navigation_grid, plan_path


class SimulationTest(unittest.TestCase):
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
        projected = [world_to_local(p, grid['origin']) for p in route]
        self.assertTrue(any(1.8 < x < 2.2 and (y < .6 or y > 2.4) for x, y in projected))
        with self.assertRaisesRegex(ValueError, 'blocked'):
            plan_path(costmap, start, pose(2, 1.5))
        restored = navigation_grid(grid, [])
        self.assertEqual(restored['cells'], grid['cells'])
        direct = plan_path(global_costmap(restored), start, goal)
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
                free = [(i, value) for i, value in enumerate(global_grid['cells']) if value == 0]
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
