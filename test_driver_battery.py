"""Battery stream patch and ROS callback checks; no physical movement."""
import unittest
from types import SimpleNamespace

from install_driver_battery import patch
from robot_ros import RobotROS


class BatteryDriverTest(unittest.TestCase):
    def test_patch_reads_existing_frame_without_serial_commands(self):
        source = '''#include "myagv_odometry/myAGV.h"
pub_v = n.advertise<std_msgs::Int8>("Voltage", 1000);
unsigned char buf[27] = {0};
vx = (static_cast<double>(buf[index]) - 128.0) * 0.01;
'''
        updated = patch(source)
        self.assertEqual(patch(updated), updated)
        self.assertIn('"/myagv/battery_voltage", 1', updated)
        self.assertIn('(buf[16] > buf[17] ? buf[16] : buf[17]) / 10.0f', updated)
        self.assertNotIn('boost::asio::write', updated)
        for invalid in (source.replace('buf[27]', 'buf[16]'), source.replace('vx =', 'vy =')):
            with self.assertRaises(ValueError):
                patch(invalid)

    def test_callback_replaces_invalid_voltage(self):
        robot = SimpleNamespace(battery=None)
        RobotROS.receive_battery(robot, SimpleNamespace(data=12.3))
        self.assertEqual(robot.battery['voltage_v'], 12.3)
        self.assertGreater(robot.battery['stamp_s'], 0)
        for value in (0, -1, 26, float('nan'), float('inf')):
            RobotROS.receive_battery(robot, SimpleNamespace(data=value))
            self.assertIsNone(robot.battery)
