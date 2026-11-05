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

    def test_move_stops_after_pulse(self):
        agv = Mock()
        command_control.run_move(agv, ["forward"], 10, 1)
        agv.go_ahead.assert_called_once_with(10, 1)
        agv.stop.assert_called_once()


if __name__ == "__main__":
    unittest.main()
