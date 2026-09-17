"""ROS1 chassis teleop plus P340 arm/gripper; run with ROS's Python."""

import argparse
import importlib.util
import math
from pathlib import Path
import signal
import sys
import termios
import time
import tty

from P340 import keyboard_control as arm_keys
from navigation import Navigation, STATIONS, arm_deadline, save_station, wait_arm


HELP = """
p: stop and record item station + arm angles (map localization required)
Tab: switch BASE / ARM (stops motion first)
v: enter/leave PICKUP teaching mode (J1 near zero, arm reaches left)
PICKUP: w/s base forward/back at 3 cm/s; a/d arm extend/retract; k/j height
PICKUP: q/e base turn left/right at 0.05 rad/s; h home
BASE: i/, forward/back; j/l turn; J/L strafe; u/o/m/. arcs
ARM:  w/s Y-/Y+; a/d X-/X+; k/j Z+/Z-; arrows = XY (unrestricted)
ARM:  h home (required before jogging unless --arm-homed)
Both: g close gripper; r open; +/- adjust active mode speed
Space: stop chassis + arm motion, keep gripper holding
Ctrl-C: quit. Hold/repeat motion keys; inactivity stops motion.
"""


class Controller:
    def __init__(self, publisher, arm, bindings, args):
        self.publisher, self.arm, self.bindings = publisher, arm, bindings
        self.args = args
        self.mode = "BASE"
        self.arm_homed = args.arm_homed
        self.active_move = None
        self.last_motion = 0.0
        self.last_poll = 0.0
        self.base_moving = False
        self.record = None

    def stop(self):
        self.publisher.update(0, 0, 0, 0, 0, 0)
        self.base_moving = False
        if self.active_move:
            with arm_deadline(self.args.arm_timeout):
                self.arm.set_jog_stop()
            self.active_move = None

    def arm_can_move(self, move):
        coords = self.arm.get_coords_info()
        if not coords or len(coords) < 3:
            raise RuntimeError("Cannot read arm coordinates; motion stopped")
        values = [float(v) for v in coords[:3]]
        if not all(math.isfinite(v) for v in values):
            raise RuntimeError("Invalid arm coordinates; motion stopped")
        axis, sign = move
        low, high = arm_keys.LIMITS[axis]
        if self.mode == "PICKUP":
            # Native X is radial reach when J1=0. Never jog native Y here.
            if values[0] <= 0 or abs(math.degrees(math.atan2(values[1], values[0]))) > 1:
                print("Pickup arm must face left at J1=0; home before teaching")
                return False
            if axis == "X":
                low = 0  # Do not retract through the arm's rotation axis.
        value = values["XYZ".index(axis)]
        return value < high - 5 if sign > 0 else value > low + 5

    def handle(self, key, now):
        if key == "\x03":
            return False
        if key == "p":
            print("Recording station: stopping motion...", flush=True)
            self.stop()
            if not self.arm_homed:
                print("Home arm before recording (h in ARM mode)")
            elif self.record is not None:
                self.record()
        elif key == "v":
            self.stop()
            self.mode = "BASE" if self.mode == "PICKUP" else "PICKUP"
            print("mode=" + self.mode)
        elif key == "\t":
            self.stop()
            self.mode = "ARM" if self.mode == "BASE" else "BASE"
            print("mode=" + self.mode)
        elif key in {"g", "r"}:
            self.stop()
            value = self.args.clamp if key == "g" else self.args.release
            with arm_deadline():
                self.arm.set_gripper_state(value, self.args.grip_speed)
            print("gripper=" + str(value))
        elif key in {"+", "-"}:
            self.stop()
            factor = 1.1 if key == "+" else 1 / 1.1
            if self.mode == "BASE":
                self.args.speed = min(0.2, max(0.01, self.args.speed * factor))
                self.args.turn = min(1.0, max(0.05, self.args.turn * factor))
                print("speed=%.3f m/s turn=%.3f rad/s" % (self.args.speed, self.args.turn))
            else:
                self.args.arm_speed = min(200, max(1, self.args.arm_speed + (5 if key == "+" else -5)))
                print("arm speed=" + str(self.args.arm_speed))
        elif self.mode in {"ARM", "PICKUP"} and key == "h":
            self.stop()
            self.arm_homed = False
            print("Homing arm; wait for completion")
            with arm_deadline(60):
                self.arm.go_zero()
            self.arm_homed = True
            print("Arm homed")
        elif self.mode == "PICKUP" and key in {"w", "s", "q", "e"}:
            if self.active_move:
                self.stop()
            x = {"w": 1, "s": -1}.get(key, 0)
            turn = {"q": 1, "e": -1}.get(key, 0)
            self.publisher.update(x, 0, 0, turn, 0.03, 0.05)
            self.base_moving = True
            self.last_motion = now
        elif self.mode == "BASE" and key in self.bindings and self.bindings[key][2] == 0:
            self.publisher.update(*self.bindings[key], self.args.speed, self.args.turn)
            self.base_moving = True
            self.last_motion = now
        elif self.mode in {"ARM", "PICKUP"} and arm_keys.key_move(key):
            if not self.arm_homed:
                self.stop()
                print("Press h to home the arm first")
                return True
            move = arm_keys.key_move(key)
            if self.mode == "PICKUP":
                move = {"a": ("X", 1), "d": ("X", -1),
                        "k": ("Z", 1), "j": ("Z", -1)}.get(key)
                if move is None:
                    self.stop()
                    return True
            # Keep the existing unrestricted ARM keys for placement poses.
            elif move[0] == "X":
                move = ("Y", move[1])
            elif move[0] == "Y":
                move = ("X", -move[1])
            if move != self.active_move:
                self.stop()
                # Bound both the preflight read and the command acknowledgement.
                with arm_deadline(self.args.arm_timeout):
                    if not self.arm_can_move(move):
                        print("Arm coordinate limit")
                        return True
                    # Cleanup also stops a partially failed command.
                    self.active_move = move
                    arm_keys.start_jog(self.arm, None, move, self.args.arm_speed)
            self.last_motion = now
        else:
            self.stop()
        return True

    def tick(self, now):
        if (self.base_moving or self.active_move) and now - self.last_motion >= self.args.key_timeout:
            self.stop()
        if self.active_move and now - self.last_poll >= 0.25:
            remaining = self.args.key_timeout - (now - self.last_motion)
            # Leave the key watchdog running; don't start a read with a tiny budget.
            if remaining < self.args.arm_timeout:
                return
            self.last_poll = now
            try:
                with arm_deadline(self.args.arm_timeout):
                    can_move = self.arm_can_move(self.active_move)
            except TimeoutError:
                self.stop()
                print("Arm feedback timed out; stopped. Press a motion key to retry.")
                return
            if not can_move:
                self.stop()
                print("Arm coordinate limit")


def keyboard_loop(controller, read_key, is_shutdown):
    try:
        while not is_shutdown():
            key = read_key(0.05)
            now = time.monotonic()
            if key is not None:
                if key == "" or not controller.handle(key, now):
                    break
            controller.tick(time.monotonic())
    finally:
        controller.stop()


def record_station(controller, nav, ask_name):
    """Capture measured poses while stopped; only then prompt for the item name."""
    if not controller.arm_homed:
        raise RuntimeError("Home arm before recording")
    print("Recording: waiting for stationary base / odom (up to 5s)...", flush=True)
    nav.wait_stopped()
    print("Recording: reading map position (up to 3s)...", flush=True)
    pose = nav.get_pose()
    print("Recording: waiting for stable arm angles...", flush=True)
    angles = wait_arm(controller.arm, timeout=5)
    station = {"base": pose, "arm_angles_deg": angles}
    print("Measured pose: %s" % station)
    name = ask_name().strip()
    if name:
        save_station(name, station, controller.args.stations)
        print("Saved %s: %s" % (name, station))
    else:
        print("Recording cancelled")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--p340-port", required=True)
    parser.add_argument("--stations", type=Path, default=STATIONS)
    parser.add_argument("--arm-homed", action="store_true", help="arm was already homed; allow jogging")
    parser.add_argument("--speed", type=float, default=0.05, help="chassis m/s, max 0.2")
    parser.add_argument("--turn", type=float, default=0.3, help="chassis rad/s, max 1")
    parser.add_argument("--arm-speed", type=int, default=30)
    parser.add_argument("--grip-speed", type=int, default=500)
    parser.add_argument("--clamp", type=int, default=0)
    parser.add_argument("--release", type=int, default=100)
    parser.add_argument("--key-timeout", type=float, default=0.6)
    parser.add_argument("--arm-timeout", type=float, default=0.4,
                        help="jog SDK timeout in seconds; must be below --key-timeout")
    args = parser.parse_args(argv)
    for name, low, high in [("speed", 0.01, 0.2), ("turn", 0.05, 1.0),
                            ("arm_speed", 1, 200), ("grip_speed", 1, 1500),
                            ("clamp", 0, 100), ("release", 0, 100),
                            ("key_timeout", 0.1, 2.0), ("arm_timeout", 0.05, 2.0)]:
        value = getattr(args, name)
        if not math.isfinite(value) or not low <= value <= high:
            parser.error("%s must be in %s..%s" % (name, low, high))
    if args.arm_timeout >= args.key_timeout:
        parser.error("--arm-timeout must be below --key-timeout")
    return args


def main():
    import rospy

    args = parse_args(rospy.myargv()[1:])
    if not sys.stdin.isatty():
        raise RuntimeError("Interactive terminal required (use ssh -t)")
    path = Path(__file__).parent / "vendor/teleop_twist_keyboard/teleop_twist_keyboard.py"
    if not path.is_file():
        raise RuntimeError("Clone ros-teleop/teleop_twist_keyboard into vendor/teleop_twist_keyboard first")
    spec = importlib.util.spec_from_file_location("upstream_teleop", path)
    upstream = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(upstream)
    upstream.stamped = False
    rospy.init_node("myagv_p340_teleop")

    def interrupt(_signum, _frame):
        raise KeyboardInterrupt

    for signum in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, interrupt)

    settings = termios.tcgetattr(sys.stdin)
    publisher = upstream.PublishThread(10)
    controller = None
    try:
        publisher.wait_for_subscribers()
        with arm_deadline():
            arm = arm_keys.ultraArmP340(args.p340_port, 115200)
        controller = Controller(publisher, arm, upstream.moveBindings, args)
        nav = None

        def ask_name():
            termios.tcsetattr(sys.stdin, termios.TCSAFLUSH, settings)
            try:
                return input("Item name (same name overwrites; Enter cancels): ")
            finally:
                tty.setcbreak(sys.stdin.fileno())
                termios.tcflush(sys.stdin, termios.TCIFLUSH)

        def record():
            nonlocal nav
            try:
                if nav is None:
                    print("Recording: connecting to ROS pose feedback...", flush=True)
                    nav = Navigation()
                record_station(controller, nav, ask_name)
            except (ValueError, RuntimeError, OSError) as exc:
                print("Not saved: %s" % exc)

        controller.record = record
        controller.stop()
        print(HELP)
        print("mode=BASE")
        tty.setcbreak(sys.stdin.fileno())
        keyboard_loop(controller, arm_keys.read_key, rospy.is_shutdown)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            publisher.stop()
            if controller is not None:
                controller.stop()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == "__main__":
    main()
