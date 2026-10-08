"""Map reset recovery and paused SLAM snapshots, without ROS or movement."""
from copy import deepcopy
from contextlib import nullcontext
import asyncio
import itertools
import json
from pathlib import Path
import subprocess
import tempfile
import threading
import time as clock
from time import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from aiohttp.test_utils import TestClient, TestServer

from robot_control import Control
from robot_ros import RobotROS
from web_backend.map_data import demo_map, render_map
from web_backend.server import EDITOR, create_app
from web_backend.storage import atomic_json


def robot_at(root):
    robot = RobotROS.__new__(RobotROS)
    robot.directory = root / 'ros'
    robot.directory.mkdir()
    robot.lock = threading.RLock()
    robot.ros, robot.nav, robot.maps, robot.zero = Mock(), Mock(), Mock(), Mock()
    robot.ros.get_param.return_value = 'run-1'
    robot.source = SimpleNamespace(header=SimpleNamespace(seq=0, stamp=0), data=[])
    robot.grid = demo_map()
    robot.zones = dict(map_id=robot.grid['map_id'], revision=0, zones=[])
    robot.costmaps = {}
    robot.map_saved, robot.costmap_saved = None, 0
    robot.pending_map = None
    robot.control = Control.__new__(Control)
    c = robot.control
    c.robot, c.directory = robot, robot.directory
    c.config = dict(small_obstacle_m=0)
    c.zones_path, c.stations_path = root / 'hardware_no_go_zones.json', root / 'stations.json'
    c.phase, c.localized, c.stop_pending, c.base_enabled = 'idle', True, None, False
    c.request_stop = Mock()
    return robot


class MapResetTest(unittest.TestCase):
    def test_map_writer_retains_costmap_sample_age_and_skips_stale_feedback(self):
        with tempfile.TemporaryDirectory() as directory:
            robot = robot_at(Path(directory))
            robot.set_map(robot.source, robot.grid, robot.zones)
            received = clock.monotonic() - .7
            robot.costmaps['global'] = (received, 0, True, robot.grid)
            robot.write_maps()
            saved = json.loads((robot.directory / 'global_costmap.json').read_text())
            self.assertAlmostEqual(time() - saved['stamp_s'], .7, delta=.2)
            robot.costmaps['global'] = (clock.monotonic() - 5, 0, True, robot.grid)
            with patch('robot_ros.atomic_json') as write:
                robot.write_maps()
                write.assert_not_called()

    def test_slam_pose_auto_confirmation_requires_map_pose_and_fresh_sensors(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            robot = robot_at(root)
            robot.control.localized = False
            robot.sensors_ready = Mock(return_value=True)
            atomic_json(root / 'mapping.json', dict(session_id='session'))
            pose = dict(x_m=0., y_m=0., yaw_rad=0.)
            robot.confirm_slam_pose(pose)
            self.assertFalse(robot.control.localized)  # Saved AMCL maps retain manual confirmation.
            robot.grid.update(map_id='slam-session-run', revision=0)
            robot.confirm_slam_pose(pose)
            self.assertFalse(robot.control.localized)
            robot.grid['revision'] = 1
            robot.confirm_slam_pose(None)
            self.assertFalse(robot.control.localized)
            robot.sensors_ready.return_value = False
            robot.confirm_slam_pose(pose)
            self.assertFalse(robot.control.localized)
            robot.sensors_ready.return_value = True
            robot.confirm_slam_pose(pose)
            self.assertTrue(robot.control.localized)

    def test_reset_survives_passive_ui_release_but_honors_explicit_stop(self):
        with tempfile.TemporaryDirectory() as directory:
            c = robot_at(Path(directory)).control
            c.phase = 'resetting_map'
            self.assertEqual(c.receive(dict(op='stop', passive=True))['status'], 'ignored')
            c.request_stop.assert_not_called()
            self.assertEqual(c.receive(dict(op='stop'))['status'], 'stopping')
            c.request_stop.assert_called_once()

    def test_slam_snapshot_changes_only_on_commit_and_survives_bridge_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            robot = robot_at(root)
            atomic_json(root / 'mapping.json', dict(session_id='session'))
            grid = demo_map()
            with patch('robot_ros.grid_dict', side_effect=lambda _: deepcopy(grid)):
                robot.receive_map(robot.source)
                self.assertEqual(robot.grid['map_id'], 'slam-session-run-1')
                self.assertEqual(robot.grid['revision'], 1)
                robot.control.localized = True
                robot.control.request_stop.reset_mock()
                robot.costmaps = {'global': (clock.monotonic(), 0, True, robot.grid)}
                published_at = robot.published_at
                original = deepcopy(robot.grid)
                grid['cells'][100] = 0 if grid['cells'][100] else 100
                robot.receive_map(robot.source)
                self.assertEqual(robot.grid, original)
                robot.write_maps()
                self.assertEqual(json.loads((robot.directory / 'map.json').read_text()), original)
                grid['cells'][101] = 0 if grid['cells'][101] else 100
                robot.receive_map(robot.source)
                self.assertEqual(robot.grid, original)
                robot.commit_map_update()
                self.assertEqual(robot.grid['revision'], 2)
                self.assertTrue(robot.control.localized)
                robot.control.request_stop.assert_not_called()
                self.assertFalse(robot.costmaps)  # Pause waits for both costmaps to consume the new snapshot.
                self.assertGreater(robot.published_at, published_at)
                robot.receive_map(robot.source)
                self.assertEqual(robot.grid['revision'], 2)
                robot.write_maps()
                self.assertEqual(json.loads((robot.directory / 'map.json').read_text())['cells'], grid['cells'])
                saved = deepcopy(robot.grid)
                grid['cells'][102] = 0 if grid['cells'][102] else 100
                robot.grid = None  # A bridge restart must not publish unpublished scans.
                robot.receive_map(robot.source)
                self.assertEqual(robot.grid, saved)
                robot.commit_map_update()
                self.assertEqual(robot.grid['revision'], 3)
                self.assertEqual(robot.grid['cells'], grid['cells'])
                robot.control.localized = True
                robot.control.request_stop.reset_mock()
                atomic_json(robot.control.zones_path, robot.zones)
                robot.control.stations_path.write_text('old stations')
                robot.ros.get_param.return_value = 'run-2'
                robot.receive_map(robot.source)
                self.assertEqual(robot.grid['map_id'], 'slam-session-run-2')
                self.assertFalse(robot.control.localized)
                self.assertFalse(robot.control.stations_path.exists())
                self.assertFalse(robot.control.zones_path.exists())
                robot.control.request_stop.assert_called_once()

    def test_reset_requires_confirmation_and_archives_or_restores_data(self):
        for failure in (None, 'start', 'scan'):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                robot = robot_at(root)
                c = robot.control
                c.reset_map = lambda: Control.reset_map(c)
                saved = root / 'room.yaml'
                files = [saved, saved.with_suffix('.pgm'), c.stations_path, c.zones_path, robot.directory / 'map.json']
                for path in files:
                    path.write_text('original ' + path.name)
                starts = []
                def process(command, **_):
                    if command[1] == 'start':
                        starts.append(command)
                        if failure == 'start' and len(starts) == 1:
                            raise subprocess.CalledProcessError(1, command)
                        if failure != 'scan' or len(starts) > 1:
                            robot.grid['revision'] = 1
                            marker = root / 'mapping.json'
                            if marker.exists():
                                robot.grid['map_id'] = 'slam-' + json.loads(marker.read_text())['session_id'] + '-run-1'
                    return subprocess.CompletedProcess(command, 0)
                with self.assertRaisesRegex(ValueError, 'Confirm reset'):
                    Control.execute(c, dict(type='reset_map'))
                clock = patch('robot_control.time.monotonic', side_effect=itertools.count(0, 21).__next__) if failure == 'scan' else nullcontext()
                with patch.dict('os.environ', MYAGV_MAP_FILE=str(saved)), patch('robot_control.shutil.which', return_value='/bin/pm2'), patch('robot_control.subprocess.run', side_effect=process), clock:
                    if failure:
                        with self.assertRaises(subprocess.CalledProcessError if failure == 'start' else RuntimeError):
                            Control.execute(c, dict(type='reset_map', confirmed=True))
                        self.assertFalse((root / 'mapping.json').exists())
                        for path in files:
                            self.assertEqual(path.read_text(), 'original ' + path.name)
                    else:
                        result = Control.execute(c, dict(type='reset_map', confirmed=True))
                        self.assertTrue(result['mapping_mode'])
                        manifest = json.loads((Path(result['backup']) / 'manifest.json').read_text())
                        for archived, original in manifest.items():
                            self.assertEqual((Path(result['backup']) / archived).read_text(), 'original ' + Path(original).name)
                        self.assertFalse(c.stations_path.exists())
                        self.assertFalse(c.localized)
                        self.assertTrue(all(cell == -1 for cell in robot.grid['cells']))
                robot.nav.go_to.assert_not_called()
                robot.nav.wait_stopped.assert_called_once()


class LiveMapAPITest(unittest.IsolatedAsyncioTestCase):
    async def test_slow_map_render_does_not_freeze_controller_or_pose_updates(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            grid = demo_map()
            atomic_json(root / 'ros' / 'map.json', grid)
            app = create_app(root / 'ui', stations_path=root / 'stations.json', hardware_dir=root / 'ros')
            rendering, release = threading.Event(), threading.Event()
            def slow_render(*args, **kwargs):
                rendering.set()
                release.wait(3)
                return render_map(*args, **kwargs)
            with patch('web_backend.server.render_map', side_effect=slow_render):
                async with TestClient(TestServer(app)) as client:
                    try:
                        atomic_json(root / 'ros' / 'map.json', {**grid, 'revision': 2})
                        self.assertTrue(await asyncio.to_thread(rendering.wait, 2))
                        pose = dict(x_m=9., y_m=2., yaw_rad=0.)
                        atomic_json(root / 'ros' / 'state.json', dict(map_id=grid['map_id'], frame='map', pose=pose, stamp_s=time(), pose_stamp_s=time()))
                        atomic_json(root / 'ros' / 'control.json', dict(phase='working', stamp_s=time()))
                        atomic_json(root / 'ros' / 'zones.json', dict(map_id=grid['map_id'], revision=0, zones=[]))
                        for _ in range(20):
                            state = await (await client.get('/api/state')).json()
                            if state['phase'] == 'working':
                                break
                            await asyncio.sleep(.05)
                        self.assertEqual(state['phase'], 'working')
                        self.assertEqual(state['navigation']['pose'], pose)
                        self.assertFalse(release.is_set())
                    finally:
                        release.set()

    async def test_map_and_pose_stream_while_reset_command_is_pending(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            grid = demo_map()
            atomic_json(root / 'ros' / 'map.json', grid)
            app = create_app(root / 'ui', stations_path=root / 'stations.json', hardware_dir=root / 'ros')
            started, release = asyncio.Event(), asyncio.Event()
            async def reset(*args, **kwargs):
                started.set()
                await release.wait()
                return dict(status='completed', mapping_mode=True)
            async with TestClient(TestServer(app)) as client:
                with patch.object(app[EDITOR].hardware, 'command', side_effect=reset):
                    pending = asyncio.create_task(client.post('/api/commands', json=dict(type='reset_map', confirmed=True, map_id=grid['map_id'], expected_revision=0)))
                    try:
                        await asyncio.wait_for(started.wait(), 2)
                        grid = {**grid, 'map_id': 'slam-fresh-session'}
                        atomic_json(root / 'ros' / 'map.json', grid)
                        pose = dict(x_m=0., y_m=0., yaw_rad=0.)
                        atomic_json(root / 'ros' / 'state.json', dict(map_id=grid['map_id'], frame='map', pose=pose, stamp_s=time(), pose_stamp_s=time()))
                        atomic_json(root / 'ros' / 'control.json', dict(phase='resetting_map', stamp_s=time()))
                        atomic_json(root / 'ros' / 'zones.json', dict(map_id=grid['map_id'], revision=0, zones=[]))
                        for _ in range(40):
                            info = await (await client.get('/api/map')).json()
                            if info['map_id'] == grid['map_id']:
                                break
                            await asyncio.sleep(.05)
                        self.assertEqual(info['map_id'], grid['map_id'])
                        state = await (await client.get('/api/state')).json()
                        self.assertEqual(state['navigation']['pose'], pose)
                        self.assertEqual(state['phase'], 'resetting_map')
                        self.assertFalse(pending.done())
                    finally:
                        release.set()
                        response = await pending
                        self.assertEqual(response.status, 200)

    async def test_new_revision_refreshes_png_without_changing_map_session(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            grid = demo_map()
            atomic_json(root / 'ros' / 'map.json', grid)
            app = create_app(root / 'ui', stations_path=root / 'stations.json', hardware_dir=root / 'ros')
            original_png = app[EDITOR].png
            async with TestClient(TestServer(app)) as client:
                grid = {**grid, 'cells': list(grid['cells']), 'revision': grid['revision'] + 1}
                grid['cells'][100] = 0 if grid['cells'][100] else 100
                atomic_json(root / 'ros' / 'map.json', grid)
                for _ in range(40):
                    response = await client.get('/api/map')
                    info = await response.json()
                    if info['revision'] == grid['revision']:
                        break
                    await asyncio.sleep(.05)
                self.assertEqual(info['revision'], grid['revision'])
                self.assertEqual(info['map_id'], grid['map_id'])
                self.assertEqual(app[EDITOR].hardware.generation, 0)
                self.assertNotEqual(await (await client.get(info['png_url'])).read(), original_png)


if __name__ == '__main__':
    unittest.main()
