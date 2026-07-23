import argparse
import glob
import math
import os
import shlex
import signal
import threading
import time

from serial.tools import list_ports
try:
    from pymycobot.mycobot import MyAgv
except ImportError:
    from pymycobot.myagv import MyAgv
from pymycobot.ultraArmP340 import ultraArmP340


BAUD = 115200
AGV_PORT = "/dev/ttyAMA2"
REACH_SPEED = 30
GRIP_SPEED = 500
MOVE_SPEED = 10
GO_SPEED = 0.3
MOVE_SECONDS = 1.0
MAX_MOVE_SECONDS = 10.0
ZERO_TIMEOUT_SECONDS = 60
CLAMP_VALUE = 0
RELEASE_VALUE = 100
COORD_TOLERANCE_MM = 2.0
APPROACH_TIMEOUT = 1.0
APPROACH_DELTA_MM = 1.0
MOVE_SETTLE_MARGIN = 1.0

LIMITS = {
    "X": (-360.0, 365.55),
    "Y": (-365.55, 365.55),
    "Z": (-140.0, 130.0),
}

MOVES = {
    "forward": "go_ahead",
    "f": "go_ahead",
    "fwd": "go_ahead",
    "back": "retreat",
    "b": "retreat",
    "bwd": "retreat",
    "backward": "retreat",
    "left": "pan_left",
    "l": "pan_left",
    "right": "pan_right",
    "r": "pan_right",
    "cw": "clockwise_rotation",
    "clockwise": "clockwise_rotation",
    "ccw": "counterclockwise_rotation",
    "counterclockwise": "counterclockwise_rotation",
}

HELP = """
commands:
  move DIRECTION [speed] [seconds]       manual chassis move pulse
  stop                                   stop chassis movement
  reach X Y Z [speed]                   move P340 TCP to absolute mm coordinate
  coord AXIS VALUE [speed]               move one P340 axis: X/Y/Z
  where                                  print current P340 x y z theta
  zero                                   P340 go_zero()
  speed reach|grip|move|go VALUE         set default speed
  mode abs|rel                           set P340 coordinate mode
  speedmode const|accel                  set P340 speed mode
  angle ID DEGREE [speed]                set P340 angle, id 1..3
  joints J1 J2 J3 [speed]                set P340 joints 1..3
  joints4 J1 J2 J3 J4 [speed]            set P340 joints 1..4
  angles                                 print P340 angles
  clamp [value] [speed]                  close gripper, default 0 500
  release [value] [speed]                open gripper, default 100 500
  grip VALUE [speed]                     set gripper value, 0..100
  gripstop                               set_gripper_release()
  power                                  P340 power_on()
  servorelease                           release all P340 servos
  go                                     reserved for navigation, not implemented
  help                                   show this text
  quit                                   exit

move directions:
  forward/f/fwd back/b/bwd left/l right/r cw ccw
""".strip()


def find_p340_port():
    if os.environ.get("P340_PORT"):
        return os.environ["P340_PORT"]

    for port in list_ports.comports():
        if "usb" in port.device.lower():
            return port.device

    patterns = [
        "/dev/ttyUSB*",
        "/dev/ttyACM*",
        "/dev/cu.usbserial*",
        "/dev/cu.wchusbserial*",
        "/dev/cu.SLAB_USBtoUART*",
    ]
    for pattern in patterns:
        matches = sorted(glob.glob(pattern))
        if matches:
            return matches[0]
    return "/dev/ttyUSB0"


def print_ports():
    for port in list_ports.comports():
        print(f"{port.device}\t{port.description}\t{port.hwid}")


def parse_float(value, name):
    try:
        value = float(value)
    except ValueError as exc:
        raise ValueError(f"{name} must be number") from exc
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite number")
    return value


def parse_int(value, name):
    try:
        return int(value)
    except ValueError as exc:
        raise ValueError(f"{name} must be int") from exc


def check_range(axis, value):
    low, high = LIMITS[axis]
    if value < low or value > high:
        raise ValueError(f"{axis} out of range: {value} not in {low}..{high} mm")


def check_reach_speed(speed):
    if speed < 1 or speed > 200:
        raise ValueError("reach speed must be 1..200")


def check_grip_speed(speed):
    if speed < 1 or speed > 1500:
        raise ValueError("grip speed must be 1..1500")


def check_move_speed(speed):
    if speed < 1 or speed > 127:
        raise ValueError("move speed must be 1..127")


def check_go_speed(speed):
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError("go speed must be > 0")


def check_seconds(seconds):
    if not 0 < seconds <= MAX_MOVE_SECONDS:
        raise ValueError(f"seconds must be > 0 and <= {MAX_MOVE_SECONDS:g}")


def check_grip(value, speed):
    if value < 0 or value > 100:
        raise ValueError("grip value must be 0..100")
    check_grip_speed(speed)


def print_coords(arm):
    coords = arm.get_coords_info()
    if not coords:
        print("no coords")
        return

    labels = ["x", "y", "z", "theta"]
    print(" ".join(f"{label}={float(value):.1f}" for label, value in zip(labels, coords)))


def read_coords(arm):
    coords = arm.get_coords_info()
    if not coords or len(coords) < 3:
        return None
    return [float(value) for value in coords[:3]]


def target_error(coords, targets):
    return max(abs(coords[index] - target) for index, target in targets.items())


def wait_done(arm, start_coords, targets, speed):
    start_error = target_error(start_coords, targets)
    if start_error <= COORD_TOLERANCE_MM:
        return

    time.sleep(APPROACH_TIMEOUT)
    coords = read_coords(arm)
    if coords is None or target_error(coords, targets) >= start_error - APPROACH_DELTA_MM:
        raise TimeoutError("move did not approach target")
    if target_error(coords, targets) <= COORD_TOLERANCE_MM:
        return

    distance = sum(
        (start_coords[index] - target) ** 2 for index, target in targets.items()
    ) ** 0.5
    time.sleep(max(0.0, distance / speed - APPROACH_TIMEOUT) + MOVE_SETTLE_MARGIN)
    coords = read_coords(arm)
    if coords is None or target_error(coords, targets) > COORD_TOLERANCE_MM:
        raise TimeoutError("move did not reach target")


def move_coords(arm, values, speed, range_check, coord_mode, wait):
    values = [parse_float(value, name) for value, name in zip(values, ["x", "y", "z"])]
    start_coords = read_coords(arm)
    if start_coords is None:
        raise RuntimeError("could not read current coordinates")
    coords = values
    if coord_mode == "rel":
        coords = [start_coords[index] + value for index, value in enumerate(values)]
    if range_check:
        for axis, value in zip("XYZ", coords):
            check_range(axis, value)
    check_reach_speed(speed)
    arm.set_mode(0)
    arm.set_coords(coords, speed)
    if wait:
        wait_done(arm, start_coords, dict(enumerate(coords)), speed)
    print_coords(arm)


def move_axis(arm, axis, value, speed, range_check, coord_mode, wait):
    start_coords = read_coords(arm)
    if start_coords is None:
        raise RuntimeError("could not read current coordinates")
    coords = start_coords.copy()
    check_reach_speed(speed)
    axis_index = "XYZ".index(axis)
    coords[axis_index] = coords[axis_index] + value if coord_mode == "rel" else value
    if range_check:
        check_range(axis, coords[axis_index])
    arm.set_mode(0)
    arm.set_coords(coords, speed)
    if wait:
        wait_done(arm, start_coords, {axis_index: coords[axis_index]}, speed)
    print_coords(arm)


def run_zero(arm):
    def timeout(_signum, _frame):
        raise TimeoutError(f"go_zero() timed out after {ZERO_TIMEOUT_SECONDS}s")

    previous_handler = signal.signal(signal.SIGALRM, timeout)
    signal.alarm(ZERO_TIMEOUT_SECONDS)
    try:
        arm.go_zero()
    finally:
        signal.alarm(0)
        signal.signal(signal.SIGALRM, previous_handler)


def get_arm(state, home=True):
    if state["arm"] is None:
        state["arm"] = ultraArmP340(state["p340_port"], state["p340_baud"])
    if home and not state["no_zero"] and not state["arm_homed"]:
        run_zero(state["arm"])
        state["arm_homed"] = True
        time.sleep(0.5)
    return state["arm"]


def get_agv(state):
    if state["agv"] is None:
        state["agv"] = MyAgv(state["agv_port"], state["agv_baud"])
        state["agv"].stop()
    return state["agv"]


def run_move(agv, args, default_speed, default_seconds):
    if len(args) not in {1, 2, 3}:
        raise ValueError("usage: move DIRECTION [speed] [seconds]")

    direction = args[0].lower()
    method_name = MOVES.get(direction)
    if method_name is None:
        raise ValueError("direction must be forward/f/fwd, back/b/bwd, left/l, right/r, cw, or ccw")

    speed = parse_int(args[1], "speed") if len(args) >= 2 else default_speed
    seconds = parse_float(args[2], "seconds") if len(args) == 3 else default_seconds
    check_move_speed(speed)
    check_seconds(seconds)

    print(f"move {direction} speed={speed} seconds={seconds:g}")

    def move():
        try:
            getattr(agv, method_name)(speed, seconds)
        except Exception as exc:
            print(f"error: {exc}")
        finally:
            agv.stop()

    thread = threading.Thread(target=move, daemon=True)
    thread.start()
    return thread


def stop_agv(state):
    if state["agv"] is None:
        return
    state["agv"].stop()
    thread = state["move_thread"]
    if thread is not None and thread.is_alive():
        thread.join(0.2)
        state["agv"].stop()


def run_speed(args, state):
    if len(args) != 2:
        raise ValueError("usage: speed reach|grip|move|go VALUE")
    kind = args[0].lower()
    if kind in {"reach", "grip", "move"}:
        speed = parse_int(args[1], "speed")
    else:
        speed = parse_float(args[1], "speed")

    if kind == "reach":
        check_reach_speed(speed)
        state["reach_speed"] = speed
    elif kind == "grip":
        check_grip_speed(speed)
        state["grip_speed"] = speed
    elif kind == "move":
        check_move_speed(speed)
        state["move_speed"] = speed
    elif kind == "go":
        check_go_speed(speed)
        state["go_speed"] = speed
    else:
        raise ValueError("usage: speed reach|grip|move|go VALUE")
    print(f"speed {kind}={speed:g}")


def run_command(line, state):
    parts = shlex.split(line)
    if not parts:
        return True

    command = parts[0].lower()
    args = parts[1:]
    reach_speed = state["reach_speed"]

    if command in {"q", "quit", "exit"}:
        return False
    if command in {"h", "help", "?"}:
        print(HELP)
    elif command == "move":
        thread = state["move_thread"]
        if thread is not None and thread.is_alive():
            raise ValueError("chassis already moving; type stop")
        state["move_thread"] = run_move(
            get_agv(state), args, state["move_speed"], state["move_seconds"]
        )
    elif command == "stop":
        stop_agv(state)
        print("stopped")
    elif command == "go":
        raise NotImplementedError("go is reserved for navigation")
    elif command == "reach":
        if len(args) not in {3, 4}:
            raise ValueError("usage: reach X Y Z [speed]")
        move_speed = parse_int(args[3], "speed") if len(args) == 4 else reach_speed
        move_coords(get_arm(state), args[:3], move_speed, state["range_check"], state["coord_mode"], state["wait"])
    elif command == "coord":
        if len(args) not in {2, 3}:
            raise ValueError("usage: coord AXIS VALUE [speed]")
        axis = args[0].upper()
        if axis not in LIMITS:
            raise ValueError("axis must be X, Y, or Z")
        value = parse_float(args[1], "value")
        move_speed = parse_int(args[2], "speed") if len(args) == 3 else reach_speed
        move_axis(get_arm(state), axis, value, move_speed, state["range_check"], state["coord_mode"], state["wait"])
    elif command == "where":
        print_coords(get_arm(state))
    elif command == "zero":
        arm = get_arm(state, home=False)
        run_zero(arm)
        state["arm_homed"] = True
        time.sleep(0.5)
        print("zeroed")
    elif command == "speed":
        run_speed(args, state)
    elif command == "mode":
        if len(args) != 1 or args[0].lower() not in {"abs", "rel"}:
            raise ValueError("usage: mode abs|rel")
        state["coord_mode"] = args[0].lower()
        print(f"mode={args[0].lower()}")
    elif command == "speedmode":
        if len(args) != 1 or args[0].lower() not in {"const", "accel"}:
            raise ValueError("usage: speedmode const|accel")
        mode = 0 if args[0].lower() == "const" else 2
        get_arm(state).set_speed_mode(mode)
        print(f"speedmode={args[0].lower()}")
    elif command == "angle":
        if len(args) not in {2, 3}:
            raise ValueError("usage: angle ID DEGREE [speed]")
        joint_id = parse_int(args[0], "id")
        degree = parse_float(args[1], "degree")
        move_speed = parse_int(args[2], "speed") if len(args) == 3 else reach_speed
        if joint_id < 1 or joint_id > 3:
            raise ValueError("id must be 1..3")
        check_reach_speed(move_speed)
        arm = get_arm(state)
        arm.set_angle(joint_id, degree, move_speed)
        print(arm.get_angles_info())
    elif command in {"joints", "joints4"}:
        joint_count = 4 if command == "joints4" else 3
        if len(args) not in {joint_count, joint_count + 1}:
            usage = "joints4 J1 J2 J3 J4 [speed]" if command == "joints4" else "joints J1 J2 J3 [speed]"
            raise ValueError(f"usage: {usage}")
        degrees = [parse_float(value, f"j{i + 1}") for i, value in enumerate(args[:joint_count])]
        move_speed = parse_int(args[joint_count], "speed") if len(args) > joint_count else reach_speed
        check_reach_speed(move_speed)
        arm = get_arm(state)
        arm.set_angles(degrees, move_speed)
        print(arm.get_angles_info())
    elif command == "angles":
        print(get_arm(state).get_angles_info())
    elif command == "clamp":
        value = parse_int(args[0], "value") if len(args) >= 1 else CLAMP_VALUE
        grip_speed = parse_int(args[1], "speed") if len(args) >= 2 else state["grip_speed"]
        check_grip(value, grip_speed)
        get_arm(state).set_gripper_state(value, grip_speed)
        print(f"grip={value}")
    elif command == "release":
        value = parse_int(args[0], "value") if len(args) >= 1 else RELEASE_VALUE
        grip_speed = parse_int(args[1], "speed") if len(args) >= 2 else state["grip_speed"]
        check_grip(value, grip_speed)
        get_arm(state).set_gripper_state(value, grip_speed)
        print(f"grip={value}")
    elif command == "grip":
        if len(args) not in {1, 2}:
            raise ValueError("usage: grip VALUE [speed]")
        value = parse_int(args[0], "value")
        grip_speed = parse_int(args[1], "speed") if len(args) == 2 else state["grip_speed"]
        check_grip(value, grip_speed)
        get_arm(state).set_gripper_state(value, grip_speed)
        print(f"grip={value}")
    elif command == "gripstop":
        get_arm(state).set_gripper_release()
        print("gripper released")
    elif command == "power":
        get_arm(state).power_on()
        print("powered")
    elif command == "servorelease":
        get_arm(state).release_all_servos()
        print("servos released")
    else:
        raise ValueError("unknown command; type help")

    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--p340-port", default=find_p340_port())
    parser.add_argument("--p340-baud", type=int, default=BAUD)
    parser.add_argument("--agv-port", default=os.environ.get("AGV_PORT", AGV_PORT))
    parser.add_argument("--agv-baud", type=int, default=BAUD)
    parser.add_argument("--reach-speed", type=int, default=REACH_SPEED)
    parser.add_argument("--grip-speed", type=int, default=GRIP_SPEED)
    parser.add_argument("--move-speed", type=int, default=MOVE_SPEED)
    parser.add_argument("--go-speed", type=float, default=GO_SPEED)
    parser.add_argument("--move-seconds", type=float, default=MOVE_SECONDS)
    parser.add_argument("--no-zero", action="store_true", help="skip first P340 command go_zero()")
    parser.add_argument("--wait", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--no-wait", action="store_true", help="return prompt before P340 coordinate movement ends")
    parser.add_argument("--no-range-check", action="store_true")
    parser.add_argument("--list-ports", action="store_true")
    args = parser.parse_args()

    if args.list_ports:
        print_ports()
        return

    check_reach_speed(args.reach_speed)
    check_grip_speed(args.grip_speed)
    check_move_speed(args.move_speed)
    check_go_speed(args.go_speed)
    check_seconds(args.move_seconds)

    state = {
        "arm": None,
        "agv": None,
        "move_thread": None,
        "p340_port": args.p340_port,
        "p340_baud": args.p340_baud,
        "agv_port": args.agv_port,
        "agv_baud": args.agv_baud,
        "reach_speed": args.reach_speed,
        "grip_speed": args.grip_speed,
        "move_speed": args.move_speed,
        "go_speed": args.go_speed,
        "move_seconds": args.move_seconds,
        "no_zero": args.no_zero,
        "arm_homed": False,
        "range_check": not args.no_range_check,
        "coord_mode": "abs",
        "wait": not args.no_wait,
    }

    print("myAGV main command control")
    print("type help.")

    try:
        while True:
            try:
                line = input("myagv> ")
            except EOFError:
                break

            try:
                if not run_command(line, state):
                    break
            except Exception as exc:
                print(f"error: {exc}")
    except KeyboardInterrupt:
        print()
    finally:
        stop_agv(state)


if __name__ == "__main__":
    main()
