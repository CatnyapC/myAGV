"""Sensor freshness stays consistent across concurrent ROS callbacks."""
from types import SimpleNamespace
import threading
import unittest
from unittest.mock import MagicMock, Mock, patch

from robot_ros import RobotROS
from robot_safety import zones_visible


class SensorFreshnessTest(unittest.TestCase):
    def setUp(self):
        self.robot = RobotROS.__new__(RobotROS)
        self.scan = SimpleNamespace(header=SimpleNamespace(stamp=10),
                                    range_min=.1, range_max=10, ranges=[1.])
        self.odom = SimpleNamespace(header=SimpleNamespace(stamp=10))
        self.robot.scan = (10, self.scan)
        self.robot.nav = SimpleNamespace(odom=(10, self.odom))
        self.robot.ros = SimpleNamespace(Time=MagicMock())
        self.robot.ros.Time.now.return_value.__sub__.return_value.to_sec.return_value = .1

    def test_callback_after_clock_read_does_not_mark_new_sample_as_future(self):
        def clock():
            self.robot.scan = (10.11, self.scan)
            self.robot.nav.odom = (10.11, self.odom)
            return 10.1

        with patch('robot_ros.time.monotonic', side_effect=clock):
            self.assertTrue(self.robot.sensors_ready())

    def test_scan_replacement_does_not_mix_validated_sample_and_ranges(self):
        def ros_clock():
            self.robot.scan = (10.1, SimpleNamespace(ranges=[]))
            stamp = MagicMock()
            stamp.__sub__.return_value.to_sec.return_value = .1
            return stamp

        self.robot.ros.Time.now.side_effect = ros_clock
        with patch('robot_ros.time.monotonic', return_value=10.1):
            self.assertTrue(self.robot.sensors_ready())

    def test_sensor_delays_up_to_one_second_remain_ready(self):
        for age in (.51, .73, 1):
            with self.subTest(age=age), patch('robot_ros.time.monotonic', return_value=10 + age):
                self.robot.ros.Time.now.return_value.__sub__.return_value.to_sec.return_value = age
                self.assertTrue(self.robot.sensors_ready())

    def test_missing_stale_future_and_invalid_samples_still_fail_closed(self):
        for sensor in ('scan', 'odom'):
            for sample in (None, (9, self.scan), (11, self.scan)):
                with self.subTest(sensor=sensor, sample=sample):
                    owner = self.robot if sensor == 'scan' else self.robot.nav
                    setattr(owner, sensor, sample)
                    with patch('robot_ros.time.monotonic', return_value=10.1):
                        self.assertFalse(self.robot.sensors_ready())
                    setattr(owner, sensor, (10, self.scan if sensor == 'scan' else self.odom))
        with patch('robot_ros.time.monotonic', return_value=10.1):
            for age in (1.01, -.11):
                self.robot.ros.Time.now.return_value.__sub__.return_value.to_sec.return_value = age
                self.assertFalse(self.robot.sensors_ready())
            self.robot.ros.Time.now.return_value.__sub__.return_value.to_sec.return_value = .1
            self.scan.ranges = [float('inf'), float('nan'), .01, 11]
            self.assertFalse(self.robot.sensors_ready())


class FetchMapFreezeTest(unittest.TestCase):
    def setUp(self):
        self.robot = RobotROS.__new__(RobotROS)
        self.robot.lock = threading.RLock()
        self.robot.ros, self.robot.maps = MagicMock(), Mock()
        self.robot.ros.Time.now.return_value.to_sec.return_value = 100
        self.robot.source = SimpleNamespace(header=SimpleNamespace())
        self.robot.grid = dict(frame='map', width=2, height=2, resolution_m=.1,
                               origin=dict(x_m=0, y_m=0, yaw_rad=0), map_id='map', revision=1,
                               cells=[0, 0, 100, -1])
        self.robot.zones = dict(map_id='map', revision=2, zones=[])
        self.robot.fetch_map = None
        self.robot.zones_ready = Mock(return_value=True)
        self.robot.sync_obstacle_layers = Mock()
        self.observed = [100, 99, 100, 0]
        self.robot.costmaps = {'global': (10, 2, True, {**self.robot.grid, 'cells': self.observed})}

    def test_snapshot_keeps_initial_obstacles_without_double_inflation_and_restores(self):
        modes = []
        self.robot.sync_obstacle_layers.side_effect = lambda: modes.append(self.robot.fetch_map is None)
        self.robot.freeze_fetch_map()
        self.assertEqual(self.robot.fetch_map['cells'], [100, 0, 100, -1])
        self.assertEqual(self.robot.maps.publish.call_args.args[0].data, [100, 0, 100, 0])
        # The first fresh costmap may still contain the previous static layer.
        self.assertFalse(zones_visible({**self.robot.grid, 'cells': [100, 0, 99, 0]}, self.robot.zone_points))
        self.assertTrue(zones_visible(self.robot.derived, self.robot.zone_points))
        self.assertEqual(self.robot.costmaps, {})
        self.observed[:] = [0, 100, 100, 0]
        self.robot.apply_zones(self.robot.zones)
        self.assertEqual(self.robot.derived['cells'], [100, 0, 100, 0])
        self.robot.restore_fetch_map()
        self.assertIsNone(self.robot.fetch_map)
        self.assertEqual(self.robot.derived['cells'], [0, 0, 100, 0])
        self.assertEqual(self.robot.grid['cells'], [0, 0, 100, -1])
        self.assertEqual(modes, [False, True])

    def test_partial_freeze_can_restore_and_invalid_geometry_never_starts(self):
        self.robot.sync_obstacle_layers.side_effect = [RuntimeError('layer failed'), None]
        with self.assertRaisesRegex(RuntimeError, 'layer failed'):
            self.robot.freeze_fetch_map()
        self.robot.restore_fetch_map()
        self.assertIsNone(self.robot.fetch_map)
        self.assertEqual(self.robot.derived['cells'], [0, 0, 100, 0])
        self.robot.costmaps = {'global': (10, 2, True, {**self.robot.grid, 'width': 3})}
        with self.assertRaisesRegex(RuntimeError, 'geometry'):
            self.robot.freeze_fetch_map()
        self.assertIsNone(self.robot.fetch_map)

    def test_queued_pre_freeze_costmap_cannot_satisfy_wait(self):
        self.robot.published_at, self.robot.costmap_after = 10, 100
        self.robot.costmaps, self.robot.costmap_received, self.robot.zone_points = {}, {}, []
        self.robot.ros.Time.now.return_value.__sub__.return_value.to_sec.return_value = .1
        grid = dict(width=2, height=2, resolution_m=.1, cells=[0]*4)
        for stamp, accepted in ((99.9, False), (100.1, True)):
            msg = SimpleNamespace(header=SimpleNamespace(frame_id='map', stamp=SimpleNamespace(to_sec=lambda: stamp)),
                                  info=SimpleNamespace(width=2, height=2))
            with patch('robot_ros.grid_dict', return_value=grid), \
                    patch('robot_ros.time.monotonic', return_value=10.5):
                self.robot.receive_costmap('global', msg)
            self.assertEqual('global' in self.robot.costmaps, accepted)

    def test_obstacle_layers_recover_after_restart_and_disable_only_during_fetch(self):
        settings = {side: False for side in ('global', 'local')}
        self.robot.ros.get_param.side_effect = lambda key, default: settings[key.split('/')[2].split('_')[0]]
        def client(prefix, **kwargs):
            side = prefix.split('/')[2].split('_')[0]
            instance = Mock()
            def update(values):
                settings[side] = values['enabled']
                return values
            instance.update_configuration.side_effect = update
            return instance
        with patch.dict('sys.modules', {'dynamic_reconfigure.client': SimpleNamespace(Client=client)}):
            RobotROS.sync_obstacle_layers(self.robot)
            self.assertEqual(settings, {'global': True, 'local': True})
            self.robot.fetch_map = self.robot.grid
            RobotROS.sync_obstacle_layers(self.robot)
            self.assertEqual(settings, {'global': False, 'local': False})
            self.robot.fetch_map = None
            RobotROS.sync_obstacle_layers(self.robot)
            self.assertEqual(settings, {'global': True, 'local': True})
