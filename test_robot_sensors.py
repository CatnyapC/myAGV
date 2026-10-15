"""Sensor freshness stays consistent across concurrent ROS callbacks."""
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

from robot_ros import RobotROS


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
