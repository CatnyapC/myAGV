import argparse
import glob
import os
import shlex
import time

from serial.tools import list_ports
from pymycobot.ultraArmP340 import ultraArmP340


BAUD = 115200
MOVE_SPEED = 30
GRIP_SPEED = 500
CLAMP_VALUE = 0
RELEASE_VALUE = 100
MOVE_START_DELAY = 0.5
MOVE_POLL_INTERVAL = 0.1
MOVE_SETTLE_READS = 2
MOVE_TIMEOUT = 60.0

LIMITS = {
    "X": (-360.0, 365.55),
    "Y": (-365.55, 365.55),
    "Z": (-140.0, 130.0),
}

HELP = """
commands:
  go X Y Z [speed]         move TCP to absolute mm coordinate
  coord AXIS VALUE [speed] move one axis: X/Y/Z
  where                    print current x y z theta
  zero                     go_zero()
  speed VALUE              set default move speed, 1..200
  mode abs|rel             set_mode(0/1)
  speedmode const|accel    set_speed_mode(0/2)
  angle ID DEGREE [speed]  set_angle(id, degree, speed), id 1..4
  joints J1 J2 J3 [speed]   set_angles([j1,j2,j3], speed)
  joints4 J1 J2 J3 J4 [speed]  set_angles([j1,j2,j3,j4], speed)
  angles                   print get_angles_info()
  clamp [value] [speed]    close gripper, default 0 500
  release [value] [speed]  open gripper, default 100 500
  grip VALUE [speed]       set_gripper_state(value, speed)
  gripstop                 set_gripper_release()
  power                    power_on()
  servorelease             release_all_servos()
  wait                     wait until is_moving_end() == 1
  help                     show this text
  quit                     exit

coordinate scale:
  millimeters. official ranges:
  X -360..365.55, Y -365.55..365.55, Z -140..130
""".strip()


def find_port():
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
        return float(value)
    except ValueError as exc:
        raise ValueError(f"{name} must be number") from exc


def parse_int(value, name):
    try:
        return int(value)
    except ValueError as exc:
        raise ValueError(f"{name} must be int") from exc


def check_range(axis, value):
    low, high = LIMITS[axis]
    if value < low or value > high:
        raise ValueError(f"{axis} out of range: {value} not in {low}..{high} mm")


def check_speed(speed):
    if speed < 1 or speed > 200:
        raise ValueError("speed must be 1..200")


def check_grip(value, speed):
    if value < 0 or value > 100:
        raise ValueError("grip value must be 0..100")
    if speed < 0 or speed > 1500:
        raise ValueError("grip speed must be 0..1500")


def print_coords(arm):
    coords = arm.get_coords_info()
    if not coords:
        print("no coords")
        return

    labels = ["x", "y", "z", "theta"]
    print(" ".join(f"{label}={float(value):.1f}" for label, value in zip(labels, coords)))


def wait_done(arm):
    time.sleep(MOVE_START_DELAY)
    deadline = time.monotonic() + MOVE_TIMEOUT
    settled_reads = 0
    while settled_reads < MOVE_SETTLE_READS:
        if time.monotonic() > deadline:
            raise TimeoutError("move did not finish")
        if arm.is_moving_end() == 1:
            settled_reads += 1
        else:
            settled_reads = 0
        time.sleep(MOVE_POLL_INTERVAL)


def move_coords(arm, values, speed, range_check, coord_mode, wait):
    coords = [parse_float(value, name) for value, name in zip(values, ["x", "y", "z"])]
    if range_check and coord_mode == "abs":
        for axis, value in zip("XYZ", coords):
            check_range(axis, value)
    check_speed(speed)
    arm.set_coords(coords, speed)
    if wait:
        wait_done(arm)
    print_coords(arm)


def run_command(arm, line, state):
    parts = shlex.split(line)
    if not parts:
        return True

    command = parts[0].lower()
    args = parts[1:]
    speed = state["speed"]

    if command in {"q", "quit", "exit"}:
        return False
    if command in {"h", "help", "?"}:
        print(HELP)
    elif command == "go":
        if len(args) not in {3, 4}:
            raise ValueError("usage: go X Y Z [speed]")
        move_speed = parse_int(args[3], "speed") if len(args) == 4 else speed
        move_coords(arm, args[:3], move_speed, state["range_check"], state["coord_mode"], state["wait"])
    elif command == "coord":
        if len(args) not in {2, 3}:
            raise ValueError("usage: coord AXIS VALUE [speed]")
        axis = args[0].upper()
        if axis not in LIMITS:
            raise ValueError("axis must be X, Y, or Z")
        value = parse_float(args[1], "value")
        move_speed = parse_int(args[2], "speed") if len(args) == 3 else speed
        if state["range_check"] and state["coord_mode"] == "abs":
            check_range(axis, value)
        check_speed(move_speed)
        arm.set_coord(axis, value, move_speed)
        if state["wait"]:
            wait_done(arm)
        print_coords(arm)
    elif command == "where":
        print_coords(arm)
    elif command == "zero":
        arm.go_zero()
        time.sleep(0.5)
        print("zeroed")
    elif command == "speed":
        if len(args) != 1:
            raise ValueError("usage: speed VALUE")
        speed = parse_int(args[0], "speed")
        check_speed(speed)
        state["speed"] = speed
        print(f"speed={speed}")
    elif command == "mode":
        if len(args) != 1 or args[0].lower() not in {"abs", "rel"}:
            raise ValueError("usage: mode abs|rel")
        mode = 0 if args[0].lower() == "abs" else 1
        arm.set_mode(mode)
        state["coord_mode"] = args[0].lower()
        print(f"mode={args[0].lower()}")
    elif command == "speedmode":
        if len(args) != 1 or args[0].lower() not in {"const", "accel"}:
            raise ValueError("usage: speedmode const|accel")
        mode = 0 if args[0].lower() == "const" else 2
        arm.set_speed_mode(mode)
        print(f"speedmode={args[0].lower()}")
    elif command == "angle":
        if len(args) not in {2, 3}:
            raise ValueError("usage: angle ID DEGREE [speed]")
        joint_id = parse_int(args[0], "id")
        degree = parse_float(args[1], "degree")
        move_speed = parse_int(args[2], "speed") if len(args) == 3 else speed
        if joint_id < 1 or joint_id > 4:
            raise ValueError("id must be 1..4")
        check_speed(move_speed)
        arm.set_angle(joint_id, degree, move_speed)
        if state["wait"]:
            wait_done(arm)
        print(arm.get_angles_info())
    elif command in {"joints", "joints4"}:
        joint_count = 4 if command == "joints4" else 3
        if len(args) not in {joint_count, joint_count + 1}:
            usage = "joints4 J1 J2 J3 J4 [speed]" if command == "joints4" else "joints J1 J2 J3 [speed]"
            raise ValueError(f"usage: {usage}")
        degrees = [parse_float(value, f"j{i + 1}") for i, value in enumerate(args[:joint_count])]
        move_speed = parse_int(args[joint_count], "speed") if len(args) > joint_count else speed
        check_speed(move_speed)
        arm.set_angles(degrees, move_speed)
        if state["wait"]:
            wait_done(arm)
        print(arm.get_angles_info())
    elif command == "angles":
        print(arm.get_angles_info())
    elif command == "clamp":
        value = parse_int(args[0], "value") if len(args) >= 1 else CLAMP_VALUE
        grip_speed = parse_int(args[1], "speed") if len(args) >= 2 else state["grip_speed"]
        check_grip(value, grip_speed)
        arm.set_gripper_state(value, grip_speed)
        print(f"grip={value}")
    elif command == "release":
        value = parse_int(args[0], "value") if len(args) >= 1 else RELEASE_VALUE
        grip_speed = parse_int(args[1], "speed") if len(args) >= 2 else state["grip_speed"]
        check_grip(value, grip_speed)
        arm.set_gripper_state(value, grip_speed)
        print(f"grip={value}")
    elif command == "grip":
        if len(args) not in {1, 2}:
            raise ValueError("usage: grip VALUE [speed]")
        value = parse_int(args[0], "value")
        grip_speed = parse_int(args[1], "speed") if len(args) == 2 else state["grip_speed"]
        check_grip(value, grip_speed)
        arm.set_gripper_state(value, grip_speed)
        print(f"grip={value}")
    elif command == "gripstop":
        arm.set_gripper_release()
        print("gripper released")
    elif command == "power":
        arm.power_on()
        print("powered")
    elif command == "servorelease":
        arm.release_all_servos()
        print("servos released")
    elif command == "wait":
        wait_done(arm)
        print("done")
    else:
        raise ValueError("unknown command; type help")

    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=find_port())
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--speed", type=int, default=MOVE_SPEED)
    parser.add_argument("--grip-speed", type=int, default=GRIP_SPEED)
    parser.add_argument("--no-zero", action="store_true", help="skip startup go_zero()")
    parser.add_argument("--wait", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--no-wait", action="store_true", help="return prompt before move commands finish")
    parser.add_argument("--no-range-check", action="store_true")
    parser.add_argument("--list-ports", action="store_true")
    args = parser.parse_args()

    if args.list_ports:
        print_ports()
        return

    state = {
        "speed": args.speed,
        "grip_speed": args.grip_speed,
        "range_check": not args.no_range_check,
        "coord_mode": "abs",
        "wait": not args.no_wait,
    }
    check_speed(state["speed"])

    arm = ultraArmP340(args.port, args.baud)
    if not args.no_zero:
        arm.go_zero()
        time.sleep(0.5)

    print(f"P340 command control on {args.port}")
    print("type help. coordinates are mm.")

    while True:
        try:
            line = input("p340> ")
        except EOFError:
            break

        try:
            if not run_command(arm, line, state):
                break
        except Exception as exc:
            print(f"error: {exc}")


if __name__ == "__main__":
    main()
