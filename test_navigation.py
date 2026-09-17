import copy
import json
import math
from pathlib import Path
import signal
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

import fetch_demo
import navigation
from teleop_control import Controller, parse_args, record_station


POSE = {"x_m": 1.2, "y_m": 0.5, "yaw_deg": 90}
STATION = {"base": POSE, "arm_angles_deg": [0, 20, 30]}


class FakeArm:
    def __init__(self, events, angles):
        self.events, self.angles = events, list(angles)

    def get_angles_info(self):
        self.events.append(("read_arm",))
        return list(self.angles)

    def is_moving_end(self):
        return 1

    def set_angles(self, angles, speed):
        self.events.append(("arm", list(angles)))
        self.angles = list(angles)

    def set_gripper_state(self, value, speed):
        self.events.append(("grip", value))

    def set_jog_stop(self):
        self.events.append(("stop_arm",))

    def set_mode(self, mode):
        self.events.append(("mode", mode))


class StationsTest(unittest.TestCase):
    def test_record_overwrite_manual_edit_and_invalid_feedback(self):
        with tempfile.TemporaryDirectory() as folder, patch("navigation.time.sleep"):
            path = Path(folder) / "stations.json"
            args = parse_args(["--p340-port", "unused", "--arm-homed", "--stations", str(path)])
            arm = FakeArm([], [0, 20, 95, 40])
            c = Controller(Mock(), arm, {"i": (1, 0, 0, 0)}, args)
            nav = Mock()
            nav.get_pose.return_value = dict(POSE)
            c.record = lambda: record_station(c, nav, lambda: "cup")
            c.handle("i", 1)
            c.handle("p", 2)
            self.assertFalse(c.base_moving)
            self.assertEqual(navigation.load_stations(path)["cup"]["arm_angles_deg"], arm.angles)
            navigation.save_station("other", STATION, path)
            arm.angles[1] = 15
            c.mode = "ARM"
            c.handle("p", 3)
            self.assertIn("other", navigation.load_stations(path))
            self.assertEqual(navigation.load_stations(path)["cup"]["arm_angles_deg"][1], 15)
            original = path.read_bytes()
            arm.angles[0] = float("nan")
            with self.assertRaises(ValueError):
                c.handle("p", 4)
            self.assertEqual(path.read_bytes(), original)
            c.arm_homed = False
            c.handle("p", 5)
            self.assertEqual(path.read_bytes(), original)
            # Same schema can be edited without the recorder.
            data = json.loads(original)
            data["cup"]["base"]["yaw_deg"] = -90
            path.write_text(json.dumps(data))
            self.assertEqual(navigation.load_stations(path)["cup"]["base"]["yaw_deg"], -90)
            before = path.read_bytes()
            with patch("navigation.os.replace", side_effect=OSError("disk error")), self.assertRaises(OSError):
                navigation.save_station("new", STATION, path)
            self.assertEqual(path.read_bytes(), before)
            self.assertEqual(list(Path(folder).iterdir()), [path])

    def test_pickup_axis_required_but_old_records_can_be_replaced(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "stations.json"
            old = dict(STATION, arm_angles_deg=[75, 20, 30])
            path.write_text(json.dumps({"cup": old, "other": old}))
            with self.assertRaisesRegex(ValueError, "J1"):
                navigation.save_station("cup", old, path)
            navigation.save_station("cup", STATION, path)
            self.assertEqual(navigation.load_stations(path)["other"], old)
            self.assertEqual(navigation.pickup_angles([0.5, 20, 30]), [0, 20, 30])

    def test_record_checks_map_before_arm_and_reports_progress(self):
        args = parse_args(["--p340-port", "unused", "--arm-homed"])
        c = Controller(Mock(), Mock(), {}, args)
        nav, ask_name = Mock(), Mock(return_value="")
        c.record = lambda: record_station(c, nav, ask_name)
        nav.get_pose.side_effect = RuntimeError("No map transform")
        with patch("teleop_control.print") as output, patch("teleop_control.wait_arm") as wait:
            with self.assertRaisesRegex(RuntimeError, "No map transform"):
                c.handle("p", 1)
            output.assert_any_call("Recording station: stopping motion...", flush=True)
            wait.assert_not_called()
            ask_name.assert_not_called()
            nav.get_pose.side_effect = None
            nav.get_pose.return_value = dict(POSE)
            wait.return_value = [10, 20, 30]
            c.handle("p", 2)
            wait.assert_called_once_with(c.arm, timeout=5)
            ask_name.assert_called_once()
            output.assert_any_call("Recording cancelled")

    def test_idle_pose_needs_stable_angles_not_motion_end_reply(self):
        arm = Mock()
        arm.get_angles_info.side_effect = [
            [0, 10, 20], [5, 10, 20], [10, 10, 20],
            [10, 10, 20], [10, 10, 20], [10, 10, 20],
        ]
        arm.is_moving_end.side_effect = AssertionError("Idle jog has no completion reply")
        with patch("navigation.time.sleep"):
            self.assertEqual(navigation.wait_arm(arm, timeout=5), [10, 10, 20])
        self.assertEqual(arm.get_angles_info.call_count, 6)
        arm.is_moving_end.assert_not_called()

    def test_target_pose_completes_without_motion_end_reply(self):
        for feedback in (
            [[10, 20, 30]] * 4,  # Already at transport pose: no movement event.
            [[0, 20, 30], [5, 20, 30]] + [[10, 20, 30]] * 4,
        ):
            with self.subTest(feedback=feedback):
                arm = Mock()
                arm.get_angles_info.side_effect = feedback
                arm.is_moving_end.side_effect = AssertionError("No completion reply")
                with patch("navigation.time.sleep"):
                    self.assertEqual(navigation.wait_arm(arm, [10, 20, 30]), [10, 20, 30])
                self.assertEqual(arm.get_angles_info.call_count, len(feedback))
                arm.is_moving_end.assert_not_called()

    def test_target_pose_rejects_wrong_or_unstable_angles(self):
        for feedback in (
            [[0, 20, 30]] * 4,  # Stationary, but never reached target.
            [[9.5, 20, 30], [10.5, 20, 30]] * 2,  # Near target, still moving.
        ):
            with self.subTest(feedback=feedback):
                arm = Mock()
                arm.get_angles_info.side_effect = feedback
                with patch("navigation.time.sleep"), patch(
                    "navigation.time.monotonic", side_effect=[0, 0, 0.2, 0.4, 0.6, 1.1]
                ), self.assertRaisesRegex(TimeoutError, "requested pose"):
                    navigation.wait_arm(arm, [10, 20, 30], timeout=1)

    def test_joint_three_uses_p340_sdk_range(self):
        from pymycobot.error import RobotLimit
        limits = RobotLimit.robot_limit["ultraArmP340"]
        self.assertEqual(navigation.JOINT_LIMITS,
                         list(zip(limits["angles_min"], limits["angles_max"])))
        for angle in (-5, 70.1, 95, 110):
            with self.subTest(angle=angle):
                self.assertEqual(navigation.validate_angles([0, 20, angle]), [0, 20, angle])
        for angle in (-5.1, 110.1):
            with self.subTest(angle=angle), self.assertRaisesRegex(
                ValueError, "J3 angle .*expected -5..110 degrees"
            ):
                navigation.validate_angles([0, 20, angle])

    def test_invalid_json_and_angles_refused(self):
        for value in (None, [], [1, 2], [0, 100, 0], [True, 0, 0], [0, 0, float("inf")]):
            with self.subTest(value=value), self.assertRaises(ValueError):
                navigation.validate_angles(value)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "stations.json"
            path.write_text("broken json")
            with self.assertRaises(ValueError):
                navigation.save_station("cup", STATION, path)
            self.assertEqual(path.read_text(), "broken json")

    def test_sdk_nonresponse_is_bounded_and_handler_restored(self):
        previous = signal.getsignal(signal.SIGALRM)
        with self.assertRaises(TimeoutError):
            with navigation.arm_deadline(0.01):
                # The SDK retries under except Exception; timeout must escape it.
                try:
                    signal.pause()
                except Exception:
                    self.fail("Deadline was swallowed by an SDK retry")
        self.assertEqual(signal.getsignal(signal.SIGALRM), previous)


def goal():
    return NS(target_pose=NS(header=NS(), pose=NS(position=NS(), orientation=NS())))


class NavigationTest(unittest.TestCase):
    def setUp(self):
        self.clock = 0.0
        self.sleep_patch = patch("navigation.time.sleep", side_effect=self.sleep)
        self.clock_patch = patch("navigation.time.monotonic", side_effect=lambda: self.clock)
        self.sleep_patch.start()
        self.clock_patch.start()
        self.addCleanup(self.sleep_patch.stop)
        self.addCleanup(self.clock_patch.stop)
        self.nav = navigation.Navigation.__new__(navigation.Navigation)
        self.nav.client = Mock()
        self.nav.client.wait_for_server.return_value = True
        self.nav.publisher = Mock()
        self.nav.ros = NS(Time=NS(now=lambda: 123), Duration=lambda v: v, is_shutdown=lambda: False)
        self.nav.goal_type, self.nav.twist_type = goal, lambda: "zero"
        self.nav.active = False
        self.nav.get_pose = Mock(return_value=dict(POSE))
        self.nav.wait_stopped = Mock()

    def sleep(self, seconds):
        self.clock += seconds

    def test_map_goal_quaternion_and_arrival(self):
        self.nav.client.get_state.return_value = 3
        self.nav.go_to(POSE)
        sent = self.nav.client.send_goal.call_args[0][0].target_pose
        self.assertEqual(sent.header.frame_id, "map")
        self.assertEqual(sent.pose.position.x, 1.2)
        self.assertAlmostEqual(sent.pose.orientation.z, 2 ** -0.5)
        self.nav.wait_stopped.assert_called_once()
        self.nav.client.cancel_goal.assert_not_called()

    def test_launch_planner_success_is_inside_arrival_tolerance(self):
        launch = ET.parse(Path(__file__).with_name("navigation_fetch.launch")).getroot()
        self.assertEqual(launch[1].tag, "include")
        self.assertEqual(launch[1].get("file"),
                         "$(find myagv_navigation)/launch/navigation_active.launch")
        self.assertTrue(all(node.tag == "param" for node in list(launch)[2:]))
        params = {node.get("name"): float(node.get("value")) for node in launch.findall("param")
                  if node.get("value") not in ("true", "false")}
        prefix = "/move_base/TrajectoryPlannerROS/"
        # Simulate move_base stopping at its configured acceptance boundary.
        self.nav.client.get_state.return_value = 3
        self.nav.get_pose.return_value = dict(
            POSE, x_m=POSE["x_m"] + params[prefix + "xy_goal_tolerance"],
            yaw_deg=POSE["yaw_deg"] + math.degrees(params[prefix + "yaw_goal_tolerance"]))
        self.nav.go_to(POSE)
        self.assertLessEqual(params[prefix + "trans_stopped_vel"], 0.01)
        self.assertLessEqual(params[prefix + "theta_stopped_vel"], 0.02)
        # Final alignment and sampled trajectories must share compatible speeds.
        turn_max = params[prefix + "max_rotational_vel"]
        turn_min = params[prefix + "min_in_place_vel_theta"]
        self.assertGreater(turn_min, 0)
        self.assertLessEqual(turn_min, turn_max)
        self.assertEqual(params[prefix + "max_vel_theta"], turn_max)
        self.assertEqual(params[prefix + "min_vel_theta"], -turn_max)
        # At the installed 5 Hz controller rate, the minimum turn step must not
        # jump across the entire heading acceptance window.
        self.assertLess(turn_min / 5, 2 * params[prefix + "yaw_goal_tolerance"])

    def test_timeout_cancels_and_waits_for_ack(self):
        self.nav.client.get_state.return_value = 1
        self.nav.client.cancel_goal.side_effect = lambda: setattr(self.nav.client.get_state, "return_value", 2)
        with self.assertRaises(TimeoutError):
            self.nav.go_to(POSE, timeout=0.1)
        self.nav.client.cancel_goal.assert_called_once()
        self.assertFalse(self.nav.active)
        self.nav.publisher.publish.assert_called_with("zero")

    def test_unconfirmed_cancel_and_interruption(self):
        self.nav.active = True
        self.nav.client.get_state.return_value = 1
        with self.assertRaisesRegex(RuntimeError, "unconfirmed"):
            self.nav.cancel()
        self.assertTrue(self.nav.active)
        self.nav.client.get_state.side_effect = [KeyboardInterrupt(), 2]
        with self.assertRaises(KeyboardInterrupt):
            self.nav.go_to(POSE)
        self.assertFalse(self.nav.active)

    def test_failed_goal_or_bad_arrival_never_succeeds(self):
        self.nav.client.get_state.return_value = 4
        with self.assertRaisesRegex(RuntimeError, "Navigation failed"):
            self.nav.go_to(POSE)
        self.nav.wait_stopped.assert_not_called()
        self.nav.client.get_state.return_value = 3
        self.nav.get_pose.return_value = dict(POSE, x_m=9)
        with self.assertRaisesRegex(RuntimeError, "tolerance"):
            self.nav.go_to(POSE)

    def test_missing_localization_does_not_send_goal(self):
        self.nav.get_pose.side_effect = RuntimeError("No map")
        with self.assertRaises(RuntimeError):
            self.nav.go_to(POSE)
        self.nav.client.send_goal.assert_not_called()

    def test_standstill_requires_fresh_stable_odom(self):
        nav = self.nav
        del nav.wait_stopped
        nav.odom = None
        class Stamp:
            def __sub__(self, other):
                return NS(to_sec=lambda: 0)
        nav.ros.Time.now = Stamp
        with self.assertRaises(TimeoutError):
            nav.wait_stopped(timeout=0.2)
        def update(seconds):
            self.clock += seconds
            msg = NS(header=NS(stamp=Stamp()), twist=NS(twist=NS(
                linear=NS(x=0, y=0), angular=NS(z=0))))
            nav._odom(msg)
        with patch("navigation.time.sleep", side_effect=update):
            nav.wait_stopped()

    def test_tf_pose_and_stale_transform(self):
        nav = self.nav
        del nav.get_pose
        class Stamp:
            def __init__(self, seconds=0):
                self.seconds = seconds
            @staticmethod
            def now():
                return Stamp(10)
            def __sub__(self, other):
                return NS(to_sec=lambda: self.seconds - other.seconds)
        nav.ros.Time = Stamp
        nav.tf_errors = (LookupError,)
        nav.tf = Mock()
        transform = NS(header=NS(stamp=Stamp(10)), transform=NS(
            translation=NS(x=1.2, y=0.5), rotation=NS(x=0, y=0, z=2**-0.5, w=2**-0.5)))
        nav.tf.lookup_transform.return_value = transform
        self.assertAlmostEqual(nav.get_pose()["yaw_deg"], 90)
        transform.header.stamp = Stamp(8)
        with self.assertRaisesRegex(ValueError, "stale"):
            nav.get_pose()


class FetchTest(unittest.TestCase):
    def run_demo(self, start, angles, fail_trip=None):
        events = []
        arm = FakeArm(events, angles)
        nav = Mock()
        nav.get_pose.side_effect = lambda: (events.append(("read_base",)) or dict(start))
        trips = []
        def go(pose, timeout):
            trips.append(dict(pose))
            events.append(("go", dict(pose)))
            if len(trips) == fail_trip:
                raise TimeoutError("blocked")
        nav.go_to.side_effect = go
        nav.go_to_pickup.side_effect = go
        args = NS(arm_speed=30, grip_speed=500, grip_wait=1.5, clamp=0, release=100, nav_timeout=120)
        station = copy.deepcopy(STATION)
        if len(angles) == 4:
            station["arm_angles_deg"].append(45)
        transport = [0, 10, 10] + ([0] if len(angles) == 4 else [])
        with patch("navigation.time.sleep"):
            if fail_trip:
                with self.assertRaises(TimeoutError):
                    fetch_demo.fetch(nav, arm, station, transport, args)
            else:
                fetch_demo.fetch(nav, arm, station, transport, args)
        return events, trips, nav

    def test_each_run_returns_to_its_own_base_and_all_arm_angles(self):
        for x, angle in ((0, 20), (2, 40)):
            start = {"x_m": x, "y_m": -1, "yaw_deg": -90}
            angles = [angle, 10, 20, -45]
            events, trips, nav = self.run_demo(start, angles)
            self.assertEqual(trips, [POSE, start])
            self.assertLess(events.index(("read_base",)), events.index(("mode", 0)))
            moves = [event for event in events if event[0] in ("arm", "grip", "go")]
            self.assertEqual(moves[-2:], [("arm", angles), ("grip", 100)])
            self.assertEqual([e for e in moves if e[0] == "grip"], [("grip", 100), ("grip", 0), ("grip", 100)])
            nav.cancel.assert_not_called()

    def test_nav_failure_blocks_grasp_or_final_release(self):
        for fail_trip in (1, 2):
            events, trips, nav = self.run_demo(POSE, [0, 10, 20], fail_trip)
            grips = [e for e in events if e[0] == "grip"]
            self.assertEqual(grips, [] if fail_trip == 1 else [("grip", 100), ("grip", 0)])
            nav.cancel.assert_called_once()
            self.assertEqual(events[-1], ("stop_arm",))

    def test_missing_transport_and_homing_are_rejected_before_hardware(self):
        for extra in ([], ["--arm-homed"], ["--transport-angles", "0", "10", "10"]):
            with patch("sys.stderr"), self.assertRaises(SystemExit):
                fetch_demo.parse_args(["cup", "--p340-port", "unused"] + extra)

    def test_nonzero_pickup_or_transport_axis_refused_before_motion(self):
        for station, transport in ((dict(STATION, arm_angles_deg=[75, 20, 30]), [0, 10, 10]),
                                   (STATION, [75, 10, 10])):
            nav, arm = Mock(), Mock()
            with self.assertRaisesRegex(ValueError, "J1"):
                fetch_demo.fetch(nav, arm, station, transport, NS())
            self.assertEqual(arm.mock_calls, [])
            self.assertEqual(nav.mock_calls, [])

    def test_snapshot_failure_and_joint_mismatch_do_not_move(self):
        for angles in ([0, 10, 20, 30], [float("nan"), 10, 20]):
            events = []
            nav = Mock()
            nav.get_pose.return_value = dict(POSE)
            with patch("navigation.time.sleep"), self.assertRaises(ValueError):
                fetch_demo.fetch(nav, FakeArm(events, angles), STATION, [0, 10, 10], NS())
            self.assertFalse(any(e[0] in ("arm", "grip", "mode") for e in events))
            nav.go_to.assert_not_called()

    def test_arm_feedback_failure_does_not_release_or_drive_again(self):
        events = []
        arm = FakeArm(events, [0, 10, 20])
        nav = Mock()
        nav.get_pose.return_value = dict(POSE)
        nav.go_to.side_effect = lambda *args: setattr(arm, "angles", [float("nan"), 0, 0])
        args = NS(arm_speed=30, grip_speed=500, grip_wait=1.5, clamp=0, release=100, nav_timeout=120)
        # Fail when sending the grasp pose, after the initial open, before clamp/return.
        original = arm.set_angles
        def broken(angles, speed):
            original(angles, speed)
            if angles == STATION["arm_angles_deg"]:
                arm.angles = [float("nan"), 0, 0]
        arm.set_angles = broken
        with patch("navigation.time.sleep"), self.assertRaises(ValueError):
            fetch_demo.fetch(nav, arm, STATION, [0, 10, 10], args)
        self.assertEqual([e for e in events if e[0] == "grip"], [("grip", 100)])
        nav.go_to_pickup.assert_called_once()
        nav.go_to.assert_not_called()


if __name__ == "__main__":
    unittest.main()
