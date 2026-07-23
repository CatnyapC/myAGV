import threading
import unittest
from unittest.mock import Mock, patch

import command_control


class CommandControlTest(unittest.TestCase):
    def test_safety_validation(self):
        for value in ("nan", "inf", "-inf"):
            with self.assertRaises(ValueError):
                command_control.parse_float(value, "value")
        with self.assertRaises(ValueError):
            command_control.check_grip_speed(0)
        with self.assertRaises(ValueError):
            command_control.check_seconds(command_control.MAX_MOVE_SECONDS + 1)

    def test_failed_homing_is_retried(self):
        arm = Mock()
        arm.go_zero.side_effect = [RuntimeError("failed"), None]
        state = {
            "arm": None,
            "p340_port": "unused",
            "p340_baud": 115200,
            "no_zero": False,
            "arm_homed": False,
        }

        with patch.object(command_control, "ultraArmP340", return_value=arm):
            with self.assertRaises(RuntimeError):
                command_control.get_arm(state)
            command_control.get_arm(state)

        self.assertTrue(state["arm_homed"])
        self.assertEqual(arm.go_zero.call_count, 2)

    def test_move_can_be_stopped(self):
        started = threading.Event()
        stopped = threading.Event()
        agv = Mock()

        def move(_speed, _seconds):
            started.set()
            stopped.wait(1)

        agv.go_ahead.side_effect = move
        agv.stop.side_effect = stopped.set
        thread = command_control.run_move(agv, ["forward"], 10, 1)
        self.assertTrue(started.wait(0.5))

        command_control.stop_agv({"agv": agv, "move_thread": thread})
        thread.join(0.5)

        self.assertFalse(thread.is_alive())
        self.assertTrue(stopped.is_set())


if __name__ == "__main__":
    unittest.main()
