"""Single-map boot recovery and durable pause acknowledgements, without ROS."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

from map_persistence import load_saved_map, map_digest, prepare_localization, save_map, validate_saved_map
from test_map_reset import robot_at
from test_robot_control import fake_control, packet
from web_backend.map_data import demo_map
from web_backend.storage import atomic_json


class MapPersistenceTest(unittest.TestCase):
    def test_boot_exports_raw_cells_orientation_and_preserves_identity(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            directory = root / 'ros'
            grid = dict(frame='map', map_id='slam-session-run-1', revision=4,
                        width=3, height=2, resolution_m=.05,
                        origin=dict(x_m=1.2, y_m=-2.3, yaw_rad=.4),
                        cells=[0] * 6, raw_cells=[-1, 0, 100, 100, 0, -1])
            grid['content_hash'] = map_digest(grid)
            save_map(directory, grid)
            atomic_json(root / 'mapping.json', dict(session_id='session'))
            yaml, mapping = prepare_localization(directory, root / 'missing.yaml')
            self.assertFalse(mapping)
            self.assertFalse((root / 'mapping.json').exists())
            self.assertIn('origin: [1.2, -2.3, 0.4]', Path(yaml).read_text())
            self.assertEqual((root / 'saved_map.pgm').read_bytes(),
                             b'P5\n3 2\n255\n' + bytes([0, 254, 205, 205, 254, 0]))
            self.assertEqual(load_saved_map(directory), grid)
            self.assertEqual(prepare_localization(directory, 'unused'), (yaml, False))

    def test_boot_adopts_legacy_snapshot_and_new_reset_starts_slam(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            directory = root / 'ros'
            self.assertEqual(prepare_localization(directory, 'room.yaml'), ('room.yaml', False))
            atomic_json(root / 'mapping.json', dict(session_id='new'))
            atomic_json(directory / 'map.json', {**demo_map(), 'revision': 0})
            self.assertEqual(prepare_localization(directory, 'room.yaml'), ('room.yaml', True))
            grid = {**demo_map(), 'map_id': 'slam-new-run'}
            atomic_json(directory / 'map.json', grid)
            _, mapping = prepare_localization(directory, 'room.yaml')
            self.assertFalse(mapping)
            self.assertEqual(json.loads((root / 'saved_map.json').read_text()),
                             {**grid, 'content_hash': map_digest(grid)})

    def test_corruption_and_export_failure_cannot_start_or_discard_slam_marker(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            directory = root / 'ros'
            grid = {**demo_map(), 'map_id': 'slam-session-run'}
            grid['content_hash'] = map_digest(grid)
            save_map(directory, grid)
            marker = root / 'mapping.json'
            atomic_json(marker, dict(session_id='session'))
            with patch('map_persistence.atomic_bytes', side_effect=OSError('disk full')):
                with self.assertRaisesRegex(OSError, 'disk full'):
                    prepare_localization(directory, 'room.yaml')
            self.assertTrue(marker.exists())
            broken = deepcopy(grid)
            broken['cells'][100] = 0 if broken['cells'][100] else 100
            atomic_json(root / 'saved_map.json', broken)
            with self.assertRaisesRegex(ValueError, 'checksum'):
                prepare_localization(directory, 'room.yaml')
            self.assertTrue(marker.exists())

    def test_invalid_geometry_identity_and_cells_are_rejected(self):
        for values in (dict(width=-1), dict(height=True), dict(revision=0), dict(map_id=''),
                       dict(resolution_m=float('nan')), dict(origin=dict(x_m=0)),
                       dict(cells=[101]), dict(raw_cells=[False])):
            with self.subTest(values=values), self.assertRaises(ValueError):
                validate_saved_map({**demo_map(), **values})

    def test_restart_retains_revision_zones_stations_and_requires_localization(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            robot = robot_at(root)
            grid = {**demo_map(), 'map_id': 'slam-session-run', 'revision': 7}
            grid['content_hash'] = map_digest(grid)
            robot.set_map(robot.source, grid, dict(map_id=grid['map_id'], revision=3, zones=[]))
            atomic_json(robot.control.zones_path, robot.zones)
            robot.control.stations_path.write_text('taught station')
            atomic_json(root / 'mapping.json', dict(session_id='session'))
            prepare_localization(robot.directory, 'room.yaml')
            robot.grid, robot.control.localized = None, False
            raw = {key: grid[key] for key in ('frame', 'width', 'height', 'resolution_m', 'origin', 'cells')}
            raw['origin'] = {**raw['origin'], 'yaw_rad': raw['origin']['yaw_rad'] + 1e-12}
            with patch('robot_ros.grid_dict', return_value=raw):
                robot.receive_map(robot.source)
                robot.receive_map(robot.source)
            self.assertEqual(robot.grid['map_id'], grid['map_id'])
            self.assertEqual(robot.grid['revision'], 7)
            self.assertEqual(robot.zones['revision'], 3)
            self.assertEqual(robot.control.stations_path.read_text(), 'taught station')
            self.assertFalse(robot.control.localized)
            raw['cells'] = [0] * len(raw['cells'])
            with patch('robot_ros.grid_dict', return_value=raw):
                robot.receive_map(robot.source)
            self.assertIsNone(robot.grid)
            self.assertEqual(load_saved_map(robot.directory)['revision'], 7)

    def test_failed_checkpoint_keeps_previous_map_and_pending_update(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            robot = robot_at(root)
            robot.set_map(robot.source, robot.grid, robot.zones)
            old = deepcopy(robot.grid)
            before = (root / 'saved_map.json').read_bytes()
            candidate = {**demo_map(), 'revision': old['revision'] + 1}
            candidate['cells'][100] = 0 if candidate['cells'][100] else 100
            robot.pending_map = robot.source, candidate
            with patch('web_backend.storage.os.replace', side_effect=OSError('disk full')):
                with self.assertRaisesRegex(OSError, 'disk full'):
                    robot.commit_map_update()
            self.assertEqual(robot.grid, old)
            self.assertIsNotNone(robot.pending_map)
            self.assertEqual((root / 'saved_map.json').read_bytes(), before)
            self.assertEqual(robot.map_save_error, 'disk full')
            robot.commit_map_update()
            self.assertEqual(load_saved_map(robot.directory)['cells'], candidate['cells'])
            self.assertEqual(json.loads((robot.directory / 'map.json').read_text()), robot.grid)
            self.assertEqual(robot.map_save_error, '')

    def test_pause_ack_waits_for_storage_and_failure_never_completes(self):
        for failure in (False, True):
            with self.subTest(failure=failure):
                c = fake_control()
                c.owner, c.task_id, c.phase = 'tab', 'update', 'navigating'
                c.robot.grid['revision'] = 4
                c.robot.ros = Mock(is_shutdown=Mock(return_value=False))
                c.map_update = dict(active=True, map_id='map', zone_revision=2, deadline=time.monotonic() + 60,
                                    pause_requested=False, pause_reason='', storage_error='', state='updating')
                request = {**packet(), 'type': 'update_pause', 'task_id': c.task_id}
                self.assertEqual(c.receive(request)['status'], 'accepted')
                self.assertEqual(c.receive(dict(op='result', id=request['id']))['status'], 'pending')
                def commit():
                    self.assertNotIn(request['id'], c.results)
                    if failure:
                        raise OSError('disk full')
                c.robot.commit_map_update.side_effect = commit
                def resume(_seconds):
                    self.assertEqual(c.results[request['id']]['status'], 'completed')
                    self.assertEqual(c.results[request['id']]['map_revision'], 4)
                    c.map_update['pause_requested'] = False
                with patch('robot_control.time.sleep', side_effect=resume):
                    if failure:
                        with self.assertRaisesRegex(OSError, 'disk full'):
                            c.wait_update()
                        c.stop_hardware('disk full')
                        self.assertEqual(c.results[request['id']]['status'], 'cancelled')
                        with self.assertRaisesRegex(RuntimeError, 'Map storage failed'):
                            c.travel_guard({})
                    else:
                        c.wait_update()


if __name__ == '__main__':
    unittest.main()
