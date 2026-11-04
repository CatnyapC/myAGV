"""Explicit motor fault recovery, cancellation and measured stop gates without hardware."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from install_driver_watchdog import patch_recovery, patch_speed_guard
from robot_control import Stopped
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
    def test_angular_fault_stops_once_and_disables_motion(self):
        robot = RobotROS.__new__(RobotROS)
        robot.ros = Mock()
        params = {'/myagv_web/driver_watchdog': 1, '/myagv_web/angular_speed_fault': True}
        robot.ros.get_param.side_effect = lambda key, default=None: params.get(key, default)
        robot.ros.get_name.return_value = '/myagv_web_control'
        robot.control = Mock()
        robot.directory = Path('/tmp/myagv-test')
        robot.angular_speed_fault, robot.amcl_uri, robot.clearance = False, None, None
        master = Mock()
        master.getSystemState.return_value = ([], [], [])
        with patch.dict(sys.modules, rosgraph=SimpleNamespace(Master=Mock(return_value=master))):
            robot.check_publishers()
            robot.check_publishers()
        self.assertFalse(robot.driver_watchdog)
        robot.control.request_stop.assert_called_once()
        self.assertIn('angular overspeed', robot.control.request_stop.call_args.args[0])

    @unittest.skipUnless(shutil.which('c++'), 'C++ compiler unavailable')
    def test_angular_guard_latches_zero_without_changing_valid_commands(self):
        original = '''#include "myagv_odometry/myAGV.h"
void MyAGV::execute(double linearX, double linearY, double angularZ) {
    writeSpeed(linearX, linearY, angularZ);
    readSpeed(); // easy to report error
}
'''
        guarded = patch_speed_guard(original)
        self.assertEqual(patch_speed_guard(guarded), guarded)
        with self.assertRaises(ValueError):
            patch_speed_guard(original.replace('readSpeed();', 'otherRead();'))
        harness = r'''
#include <array>
#include <cassert>
#include <limits>
#include <string>
#include <vector>
#define ROS_ERROR(...) ((void)0)
struct MyAGV {
    struct Params { bool fault=false; void setParam(const char*, bool v) { fault=v; } } n;
    double vtheta=0;
    double currentTime=1, lastTime=0;
    bool fresh=true;
    std::vector<std::array<double,3>> commands;
    void writeSpeed(double x,double y,double z) { commands.push_back({{x,y,z}}); }
    bool readSpeed() { return fresh; }
    void publisherOdom(double) {}
    void execute(double,double,double);
};
'''
        harness += guarded.replace('#include "myagv_odometry/myAGV.h"', '')
        harness += r'''
int main(int argc,char** argv) {
    MyAGV driver;
    std::string mode=argv[1];
    double turn=.03;
    driver.vtheta=.1;
    if(mode=="reverse") { turn=-.03; driver.vtheta=-.1; }
    if(mode=="overspeed") driver.vtheta=.83;
    if(mode=="reverse_overspeed") { turn=-.03; driver.vtheta=-.83; }
    if(mode=="stale") { driver.vtheta=.83; driver.fresh=false; }
    if(mode=="nan") turn=std::numeric_limits<double>::quiet_NaN();
    if(mode=="inf") turn=std::numeric_limits<double>::infinity();
    driver.execute(.02,0,turn);
    bool fault=mode=="overspeed" || mode=="reverse_overspeed" || mode=="nan" || mode=="inf";
    assert(driver.n.fault==fault);
    if(fault) {
        assert((driver.commands.back()==std::array<double,3>{{0,0,0}}));
        driver.vtheta=0;
        driver.execute(.1,.1,-.1);
        assert((driver.commands.back()==std::array<double,3>{{0,0,0}}));
    } else if(mode=="stale") assert((driver.commands.back()==std::array<double,3>{{0,0,0}}));
    else assert((driver.commands.back()==std::array<double,3>{{.02,0,turn}}));
}
'''
        with tempfile.TemporaryDirectory() as directory:
            source, binary = Path(directory) / 'guard.cpp', Path(directory) / 'guard'
            modern = original.replace('''    writeSpeed(linearX, linearY, angularZ);
    readSpeed(); // easy to report error''', '''    double dt = currentTime - lastTime;
    if (true == readSpeed()) {
        writeSpeed(linearX, linearY, angularZ);
        publisherOdom(dt);
    }
    lastTime = currentTime;''')
            modern_harness = harness.replace(guarded.replace('#include "myagv_odometry/myAGV.h"', ''),
                                             patch_speed_guard(modern).replace('#include "myagv_odometry/myAGV.h"', ''))
            for layout, code in (('legacy', harness), ('modern', modern_harness)):
                source.write_text(code)
                subprocess.run(['c++', '-std=c++11', str(source), '-o', str(binary)], check=True, capture_output=True)
                for mode in ('normal', 'reverse', 'overspeed', 'reverse_overspeed', 'stale', 'nan', 'inf'):
                    with self.subTest(layout=layout, mode=mode):
                        subprocess.run([str(binary), mode], check=True, timeout=5)

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

    def test_failed_or_cancelled_recovery_stays_locked_even_if_later_stop_feedback_arrives(self):
        for failure in (TimeoutError('Reset not acknowledged'), Stopped('STOP')):
            control = fake_control()
            control.robot.recover_base_stop.side_effect = failure
            with self.assertRaises(type(failure)):
                control.execute(dict(type='recover_stop', confirmed=True))
            with patch('robot_control.arm_deadline'), patch('robot_control.wait_arm', return_value=[90, 30, 40]):
                control.stop_hardware(str(failure))
            self.assertEqual(control.phase, 'fault')
            self.assertFalse(control.arm_homed)

    def test_patch_refuses_unknown_input_and_is_idempotent(self):
        source = patch_recovery(LEGACY)
        self.assertEqual(patch_recovery(source), source)
        with self.assertRaises(ValueError):
            patch_recovery(LEGACY.replace('std::cin', 'custom_input'))
        newer = LEGACY.replace('bool MyAGV::readSpeed()', 'void MyAGV::clearSerialBuffer() {}\n\nbool MyAGV::readSpeed()')
        self.assertIn('void MyAGV::clearSerialBuffer() {}', patch_recovery(newer))

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
