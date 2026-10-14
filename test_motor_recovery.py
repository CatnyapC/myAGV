"""Explicit motor fault recovery, cancellation and measured stop gates without hardware."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

from install_driver_watchdog import patch_recovery
from robot_ros import RobotROS
from test_robot_control import fake_control


LEGACY = '''void MyAGV::restoreRun()
{
    int res = 0;
    while (res != 1) { std::cin >> res; }
    restore();
}

bool MyAGV::readSpeed() { return true; }
'''


class MotorRecoveryTest(unittest.TestCase):
    def robot(self, fault=True):
        robot = RobotROS.__new__(RobotROS)
        robot.control = fake_control()
        robot.control.deadline = time.monotonic() + 60
        robot.driver_watchdog = robot.exclusive = True
        robot.zero, robot.nav, robot.ros = Mock(), Mock(), Mock()
        robot.ros.is_shutdown.return_value = False
        self.params = {'/myagv_web/motor_overcurrent': fault, '/myagv_web/driver_recovery': 1}
        robot.ros.get_param.side_effect = lambda name, default: self.params.get(name, default)
        robot.ros.set_param.side_effect = lambda name, value: self.params.update({name: value})
        return robot

    def test_reset_only_for_latched_fault_then_requires_new_measured_stop(self):
        for fault in (False, True):
            robot = self.robot(fault)
            def acknowledge(_seconds):
                self.assertTrue(self.params['/myagv_web/motor_restore_requested'])
                self.params['/myagv_web/motor_overcurrent'] = False
            def measured_stop():
                self.assertFalse(self.params['/myagv_web/motor_overcurrent'])
            robot.nav.wait_stopped.side_effect = measured_stop
            with patch('robot_ros.time.sleep', side_effect=acknowledge):
                robot.recover_base_stop()
            robot.zero.assert_called_once()
            robot.nav.cancel.assert_called_once()
            robot.nav.wait_stopped.assert_called_once()
            if fault:
                self.assertEqual([call.args for call in robot.ros.set_param.call_args_list],
                    [('/myagv_web/motor_restore_requested', True), ('/myagv_web/motor_restore_requested', False)])
            else:
                robot.ros.set_param.assert_not_called()

    def test_timeout_and_stop_revoke_reset_without_clearing_motion_lock(self):
        robot = self.robot()
        robot.control.deadline = 160
        with patch('robot_ros.time.monotonic', side_effect=[100, 100, 100, 106]):
            with self.assertRaisesRegex(TimeoutError, 'did not acknowledge'):
                robot.recover_base_stop()
        self.assertFalse(self.params['/myagv_web/motor_restore_requested'])
        robot.nav.wait_stopped.assert_not_called()
        robot = self.robot()
        def stop(_seconds):
            robot.control.stop_pending = 'STOP'
        with patch('robot_ros.time.sleep', side_effect=stop):
            with self.assertRaisesRegex(RuntimeError, 'stopped'):
                robot.recover_base_stop()
        self.assertFalse(self.params['/myagv_web/motor_restore_requested'])
        robot.nav.wait_stopped.assert_not_called()

    def test_unsafe_or_uninstalled_recovery_never_sends_reset(self):
        for condition in ('exclusive', 'driver_watchdog', 'lease', 'patch'):
            robot = self.robot()
            if condition in ('exclusive', 'driver_watchdog'):
                setattr(robot, condition, False)
            elif condition == 'lease':
                robot.control.deadline = 0
            else:
                self.params['/myagv_web/driver_recovery'] = 0
            with self.assertRaises(RuntimeError):
                robot.recover_base_stop()
            robot.ros.set_param.assert_not_called()

    def test_physical_confirmation_and_arm_feedback_still_gate_recovery(self):
        control = fake_control()
        control.connect_arm = Mock()
        control.grip = Mock()
        with self.assertRaises(ValueError):
            control.execute(dict(type='recover_stop', confirmed=False))
        control.robot.recover_base_stop.assert_not_called()
        with patch('robot_control.wait_arm', return_value=[90, 30, 40]):
            control.execute(dict(type='recover_stop', confirmed=True))
        control.robot.recover_base_stop.assert_called_once()
        self.assertFalse(control.arm_homed)
        self.assertFalse(control.base_enabled)
        control.arm.set_coords.assert_not_called()
        control.grip.assert_not_called()

    def test_recovery_watchdog_allows_missing_odometry_but_keeps_lease_guard(self):
        control = fake_control()
        control.phase = 'recovering_stop'
        control.robot.sensors_ready = Mock(return_value=False)
        control.request_stop = Mock()
        control.watchdog()
        control.request_stop.assert_not_called()
        control.deadline = 0
        control.watchdog()
        control.request_stop.assert_called_with('Control heartbeat expired')

    def test_patch_refuses_unknown_input_and_is_idempotent(self):
        source = patch_recovery(LEGACY)
        self.assertEqual(patch_recovery(source), source)
        with self.assertRaises(ValueError):
            patch_recovery(LEGACY.replace('std::cin', 'custom_input'))

    @unittest.skipUnless(shutil.which('c++'), 'C++ compiler unavailable')
    def test_generated_driver_waits_for_request_and_never_resumes_motion(self):
        harness = r'''
#include <atomic>
#include <cassert>
#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <thread>
namespace ros {
std::atomic<bool> running{true};
bool ok() { return running; }
struct WallDuration {
    double seconds;
    WallDuration(double s): seconds(s) {}
    void sleep() { std::this_thread::sleep_for(std::chrono::milliseconds(2)); }
};
}
struct Params {
    std::mutex mutex;
    std::map<std::string, bool> values;
    void setParam(std::string key, bool value) { std::lock_guard<std::mutex> lock(mutex); values[key]=value; }
    void getParam(std::string key, bool &value) { std::lock_guard<std::mutex> lock(mutex); value=values[key]; }
};
struct MyAGV {
    Params n;
    std::atomic<int> resets{0}, zeros{0};
    void restore() { ++resets; }
    void writeSpeed(double x, double y, double z) { assert(x==0 && y==0 && z==0); ++zeros; }
    void restoreRun();
    bool readSpeed();
};
'''
        harness += patch_recovery(LEGACY)
        harness += r'''
void wait_latched(MyAGV &driver) {
    for (int i=0; i<1000; ++i) {
        bool fault=false;
        driver.n.getParam("/myagv_web/motor_overcurrent",fault);
        if (fault) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    assert(false);
}
int main() {
    MyAGV driver;
    driver.n.setParam("/myagv_web/motor_restore_requested",true);
    std::thread worker([&] { driver.restoreRun(); });
    wait_latched(driver);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    assert(driver.resets==0);
    driver.n.setParam("/myagv_web/motor_restore_requested",true);
    worker.join();
    bool fault=true, requested=true;
    driver.n.getParam("/myagv_web/motor_overcurrent",fault);
    driver.n.getParam("/myagv_web/motor_restore_requested",requested);
    assert(driver.resets==1 && driver.zeros==2 && !fault && !requested);
    MyAGV shutting_down;
    std::thread stopped([&] { shutting_down.restoreRun(); });
    wait_latched(shutting_down);
    ros::running=false;
    stopped.join();
    assert(shutting_down.resets==0 && shutting_down.zeros==1);
}
'''
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'driver.cpp'
            binary = Path(directory) / 'driver'
            source.write_text(harness)
            subprocess.run(['c++', '-std=c++11', '-pthread', str(source), '-o', str(binary)], check=True, capture_output=True)
            subprocess.run([str(binary)], check=True, timeout=5)
