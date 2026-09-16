import time
import unittest
from unittest.mock import Mock, patch

from teleop_control import Controller, keyboard_loop, parse_args


class TeleopTest(unittest.TestCase):
    def setUp(self):
        self.events = []
        self.publisher = Mock()
        self.publisher.update.side_effect = lambda *v: self.events.append(("base", v))
        self.arm = Mock()
        self.arm.get_coords_info.return_value = [180, 0, 80]
        self.arm.set_jog_stop.side_effect = lambda: self.events.append(("arm_stop",))
        self.arm.set_gripper_state.side_effect = lambda *v: self.events.append(("grip", v))
        args = parse_args(["--p340-port", "unused", "--arm-homed"])
        self.controller = Controller(self.publisher, self.arm, {"i": (1, 0, 0, 0), "J": (0, 1, 0, 0)}, args)

    def test_drive_timeout_and_gripper_stop_order(self):
        c = self.controller
        c.handle("J", 1)
        self.assertEqual(self.events[-1], ("base", (0, 1, 0, 0, 0.05, 0.3)))
        c.tick(1.61)
        self.assertFalse(c.base_moving)
        self.assertEqual(self.events[-1], ("base", (0, 0, 0, 0, 0, 0)))
        c.handle("i", 2)
        c.handle("g", 2.1)
        self.assertEqual(self.events[-2:], [("base", (0, 0, 0, 0, 0, 0)), ("grip", (0, 500))])
        c.handle("r", 3)
        self.arm.set_gripper_state.assert_called_with(100, 500)

    def test_mode_switch_and_arm_timeout(self):
        c = self.controller
        c.handle("i", 1)
        c.handle("\t", 1.1)
        self.assertFalse(c.base_moving)
        c.handle("k", 2)
        self.arm.set_jog_coord.assert_called_with(3, 0, 30)
        c.tick(2.61)
        self.assertIsNone(c.active_move)
        c.handle("w", 3)
        c.handle("\t", 3.1)
        self.assertEqual(c.mode, "BASE")
        self.assertIsNone(c.active_move)
        self.assertEqual(self.arm.set_jog_stop.call_count, 2)

    def test_sideways_mount_keys_arrows_and_joint_limits(self):
        c = self.controller
        c.mode = "ARM"
        # Vehicle forward/back/left/right for the 90-degree CCW arm mounting.
        for keys, axis, direction in ((["w", "\x1b[A"], 2, 0),
                                      (["s", "\x1b[B"], 2, 1),
                                      (["a", "\x1b[D"], 1, 0),
                                      (["d", "\x1b[C"], 1, 1),
                                      (["k"], 3, 0), (["j"], 3, 1)):
            for key in keys:
                with self.subTest(key=key):
                    c.stop()
                    self.arm.set_jog_coord.reset_mock()
                    c.handle(key, 1)
                    self.arm.set_jog_coord.assert_called_once_with(axis, direction, 30)
        c.stop()
        self.arm.set_jog_coord.reset_mock()
        self.arm.get_coords_info.return_value = [180, -365, 80]
        c.handle("w", 2)  # Forward now checks the arm's lower Y limit.
        self.arm.set_jog_coord.assert_not_called()
        c.handle("s", 3)  # Reverse moves away from that limit.
        self.arm.set_jog_coord.assert_called_once_with(2, 1, 30)

    def test_unhomed_and_outward_limit_motion_refused(self):
        c = self.controller
        c.handle("\t", 0)
        c.arm_homed = False
        c.handle("w", 1)
        self.arm.set_jog_coord.assert_not_called()
        c.arm_homed = True
        self.arm.get_coords_info.return_value = [180, 0, 130]
        c.handle("k", 2)
        self.arm.set_jog_coord.assert_not_called()
        c.handle("j", 3)  # Moving away from upper Z limit is allowed.
        self.arm.set_jog_coord.assert_called_once_with(3, 1, 30)

    def test_stalled_x_jog_stops_and_allows_reverse(self):
        c = self.controller
        c.mode = "ARM"
        c.handle("a", 10)  # X- can stall before the rectangular XYZ limit.
        self.arm.get_coords_info.return_value = [175, 0, 80]
        for now in (10.3, 10.6, 10.9, 11.2, 11.5, 11.9):
            c.handle("a", now)
            c.tick(now)
        self.assertIsNone(c.active_move)
        self.arm.set_jog_stop.assert_called_once()
        # Key repeat must not restart the stalled command or query it again.
        self.arm.get_coords_info.reset_mock()
        c.handle("a", 12)
        self.arm.get_coords_info.assert_not_called()
        self.arm.set_jog_coord.assert_called_once_with(1, 0, 30)
        c.handle("d", 12.1)
        self.assertEqual(c.active_move, ("X", 1))
        self.arm.set_jog_coord.assert_called_with(1, 1, 30)
        c.handle("\t", 12.2)
        self.assertEqual(c.mode, "BASE")

    def test_slow_progress_does_not_trigger_stall(self):
        c = self.controller
        c.mode = "ARM"
        c.handle("a", 10)
        for step in range(1, 13):
            now = 10 + step * 0.3
            self.arm.get_coords_info.return_value = [180 - step * 0.2, 0, 80]
            c.handle("a", now)
            c.tick(now)
        self.assertEqual(c.active_move, ("X", -1))
        self.arm.set_jog_stop.assert_not_called()

    def test_reached_x_limit_blocks_repeat_but_not_reverse(self):
        c = self.controller
        c.mode = "ARM"
        c.handle("a", 10)
        self.arm.get_coords_info.return_value = [-356, 0, 80]
        c.tick(10.1)
        self.assertIsNone(c.active_move)
        c.handle("a", 10.2)
        self.arm.set_jog_coord.assert_called_once_with(1, 0, 30)
        c.handle("d", 10.3)
        self.assertEqual(c.active_move, ("X", 1))

    def test_feedback_failure_stops_motion_and_propagates(self):
        c = self.controller
        c.handle("\t", 0)
        c.handle("w", 10)
        self.arm.get_coords_info.return_value = None
        with patch("teleop_control.time.monotonic", return_value=10.1):
            with self.assertRaisesRegex(RuntimeError, "Cannot read arm coordinates"):
                keyboard_loop(c, lambda _: None, lambda: False)
        self.arm.set_jog_stop.assert_called_once()
        self.assertEqual(self.events[-2][0], "base")

    def test_eof_and_serial_failure_stop_chassis(self):
        c = self.controller
        c.handle("i", 1)
        keyboard_loop(c, lambda _: "", lambda: False)
        self.assertFalse(c.base_moving)
        c.handle("i", 2)
        self.arm.set_gripper_state.side_effect = OSError("serial disconnected")
        with self.assertRaises(OSError):
            keyboard_loop(c, lambda _: "g", lambda: False)
        self.assertFalse(c.base_moving)
        self.assertEqual(self.events[-1], ("base", (0, 0, 0, 0, 0, 0)))

    def test_invalid_speed_and_timeout(self):
        for option, value in [("--speed", "nan"), ("--key-timeout", "0"),
                              ("--grip-speed", "1501"), ("--arm-timeout", "nan"),
                              ("--arm-timeout", "0.6")]:
            with self.subTest(option=option), patch("sys.stderr"), self.assertRaises(SystemExit):
                parse_args(["--p340-port", "unused", option, value])

    def test_blocked_jog_read_and_command_trigger_stop(self):
        def blocked(*_args):
            time.sleep(0.8)
            return [180, 0, 80]

        c = self.controller
        c.mode = "ARM"
        c.args.key_timeout = 0.1
        c.args.arm_timeout = 0.05
        for operation in ("get_coords_info", "set_jog_coord"):
            with self.subTest(operation=operation):
                self.arm.reset_mock(side_effect=True)
                self.arm.get_coords_info.return_value = [180, 0, 80]
                if operation == "get_coords_info":
                    c.handle("w", time.monotonic())
                    key = None  # Stall a feedback poll while the arm is jogging.
                else:
                    key = "w"  # Stall the acknowledgement after sending jog.
                getattr(self.arm, operation).side_effect = blocked
                c.last_poll = 0
                started = time.monotonic()
                if operation == "get_coords_info":
                    c.tick(time.monotonic())  # Recoverable poll timeout stops jogging.
                else:
                    with self.assertRaises(TimeoutError):
                        keyboard_loop(c, lambda _: key, lambda: False)
                self.assertLess(time.monotonic() - started, 0.4)
                self.arm.set_jog_stop.assert_called_once()
                self.assertIsNone(c.active_move)
                self.publisher.update.assert_called_with(0, 0, 0, 0, 0, 0)

    def test_near_key_expiry_skips_read_then_stops_on_time(self):
        c = self.controller
        c.mode = "ARM"
        c.handle("w", 10)
        self.arm.get_coords_info.reset_mock()
        c.tick(10.59)
        self.arm.get_coords_info.assert_not_called()
        self.assertIsNotNone(c.active_move)
        c.tick(10.61)
        self.assertIsNone(c.active_move)
        self.arm.set_jog_stop.assert_called_once()

    def test_feedback_slower_than_old_limit_is_accepted(self):
        c = self.controller
        c.mode = "ARM"
        c.handle("w", time.monotonic())
        def delayed_feedback():
            time.sleep(0.3)
            return [180, 0, 80]
        self.arm.get_coords_info.side_effect = delayed_feedback
        c.tick(time.monotonic())
        self.assertIsNotNone(c.active_move)
        self.arm.set_jog_stop.assert_not_called()
        c.stop()

    def test_blocked_stop_acknowledgement_is_also_bounded(self):
        c = self.controller
        c.mode = "ARM"
        c.handle("w", time.monotonic())
        self.arm.set_jog_stop.side_effect = lambda: time.sleep(0.8)
        started = time.monotonic()
        with self.assertRaises(TimeoutError):
            keyboard_loop(c, lambda _: "", lambda: False)
        self.assertLess(time.monotonic() - started, 0.6)
        self.publisher.update.assert_called_with(0, 0, 0, 0, 0, 0)


if __name__ == "__main__":
    unittest.main()
