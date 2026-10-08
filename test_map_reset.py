"""Map reset recovery and continuous SLAM updates, without ROS or movement."""
from copy import deepcopy
from contextlib import nullcontext
import asyncio
import itertools
import json
from pathlib import Path
import subprocess
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from aiohttp.test_utils import TestClient, TestServer

from robot_control import Control
from robot_ros import RobotROS
from web_backend.map_data import demo_map
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
    robot.control = Control.__new__(Control)
    c = robot.control
    c.robot, c.directory = robot, robot.directory
    c.zones_path, c.stations_path = root / 'hardware_no_go_zones.json', root / 'stations.json'
    c.phase, c.localized, c.stop_pending, c.base_enabled = 'idle', True, None, False
    c.request_stop = Mock()
    return robot


class MapResetTest(unittest.TestCase):
    def test_live_updates_keep_map_identity_localization_and_costmap_feedback(self):
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
                robot.costmaps = {'global': 'fresh approved feedback'}
                grid['cells'][100] = 0 if grid['cells'][100] else 100
                robot.receive_map(robot.source)
                self.assertEqual(robot.grid['revision'], 2)
                self.assertTrue(robot.control.localized)
                robot.control.request_stop.assert_not_called()
                self.assertIn('global', robot.costmaps)
                robot.receive_map(robot.source)
                self.assertEqual(robot.grid['revision'], 2)
                self.assertEqual(json.loads((robot.directory / 'map.json').read_text())['cells'], grid['cells'])
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
