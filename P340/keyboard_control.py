import argparse
import glob
import os
import select
import sys
import termios
import time
import tty

from serial.tools import list_ports
from pymycobot.ultraArmP340 import ultraArmP340


BAUD = 115200
MOVE_SPEED = 30
SPEED_STEP = 5
GRIP_SPEED = 500
CLAMP_VALUE = 0
RELEASE_VALUE = 100
RANGE_POLL_INTERVAL = 0.25

ARROWS = {
    "\x1b[D": ("Y", 1),   # left
    "\x1b[C": ("Y", -1),  # right
    "\x1b[A": ("X", -1),  # up
    "\x1b[B": ("X", 1),   # down
    "\x1bOD": ("Y", 1),
    "\x1bOC": ("Y", -1),
    "\x1bOA": ("X", -1),
    "\x1bOB": ("X", 1),
}

KEY_MOVES = {
    **ARROWS,
    "a": ("Y", 1),
    "d": ("Y", -1),
    "w": ("X", -1),
    "s": ("X", 1),
    "k": ("Z", 1),
    "j": ("Z", -1),
}

AXES = {"X": 1, "Y": 2, "Z": 3}
LIMITS = {
    "X": (-360.0, 365.55),
    "Y": (-365.55, 365.55),
    "Z": (-140.0, 130.0),
}


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


def read_key(timeout=0.05):
    if not select.select([sys.stdin], [], [], timeout)[0]:
        return None

    key = sys.stdin.read(1)
    if key != "\x1b":
        return key

    end = time.monotonic() + 0.1
    while time.monotonic() < end and select.select([sys.stdin], [], [], 0.01)[0]:
        key += sys.stdin.read(1)
    return key


def key_move(key):
    if key in KEY_MOVES:
        return KEY_MOVES[key]
    if key and key.startswith("\x1b") and key[-1] in "ABCD":
        return {
            "D": ("Y", 1),
            "C": ("Y", -1),
            "A": ("X", -1),
            "B": ("X", 1),
        }[key[-1]]
    return None


def print_coords(arm):
    coords = arm.get_coords_info()
    if coords and len(coords) >= 3:
        print(
            f"x={float(coords[0]):.1f} "
            f"y={float(coords[1]):.1f} "
            f"z={float(coords[2]):.1f}"
        )


def start_jog(arm, active_move, move, speed, force=False):
    if active_move == move and not force:
        return active_move

    if active_move:
        arm.set_jog_stop()
        print_coords(arm)

    axis, sign = move
    if axis == "Z":
        direction = 0 if sign > 0 else 1
    else:
        direction = 1 if sign > 0 else 0
    arm.set_jog_coord(AXES[axis], direction, speed)
    print(f"jog {axis}{'+' if sign > 0 else '-'}")
    return move


def stop_jog(arm, active_move):
    if active_move:
        arm.set_jog_stop()
        print("stop")
        print_coords(arm)
    return None


def change_speed(arm, active_move, speed, delta, default_speed):
    speed = default_speed if delta == 0 else min(200, max(1, speed + delta))
    print(f"speed={speed}")
    if active_move:
        start_jog(arm, active_move, active_move, speed, force=True)
    return speed


def at_limit(arm, move):
    axis, sign = move
    coords = arm.get_coords_info()
    if not coords or len(coords) < 3:
        return False

    value = float(coords["XYZ".index(axis)])
    low, high = LIMITS[axis]
    return value >= high if sign > 0 else value <= low


def print_help(port):
    print(f"P340 keyboard control on {port}")
    print("press once: w/s or up/down = X, a/d or left/right = Y, k/j = Z")
    print("space: stop arm and gripper")
    print("n: clamp, m: release")
    print("q/e/r: slower/faster/reset speed")
    print("h: go zero, x: quit")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=find_port())
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--speed", type=int, default=MOVE_SPEED)
    parser.add_argument("--speed-step", type=int, default=SPEED_STEP)
    parser.add_argument("--grip-speed", type=int, default=GRIP_SPEED)
    parser.add_argument("--clamp", type=int, default=CLAMP_VALUE)
    parser.add_argument("--release", type=int, default=RELEASE_VALUE)
    parser.add_argument("--range-poll-interval", type=float, default=RANGE_POLL_INTERVAL)
    parser.add_argument("--zero", action="store_true")
    parser.add_argument("--list-ports", action="store_true")
    parser.add_argument("--debug-keys", action="store_true")
    args = parser.parse_args()

    if args.list_ports:
        print_ports()
        return

    arm = ultraArmP340(args.port, args.baud)
    if args.zero:
        arm.go_zero()
        time.sleep(0.5)

    active_move = None
    last_range_poll = 0.0
    move_speed = args.speed

    print_help(args.port)
    old_term = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            now = time.monotonic()
            key = read_key()
            if args.debug_keys and key:
                print(repr(key))

            if key == "x":
                break
            if key == "h":
                active_move = stop_jog(arm, active_move)
                arm.go_zero()
                time.sleep(0.5)
                print("zeroed")
            elif move := key_move(key):
                active_move = start_jog(arm, active_move, move, move_speed)
            elif key == " ":
                active_move = stop_jog(arm, active_move)
                arm.set_gripper_release()
                print("gripper stop")
            elif key == "q":
                move_speed = change_speed(arm, active_move, move_speed, -args.speed_step, args.speed)
            elif key == "e":
                move_speed = change_speed(arm, active_move, move_speed, args.speed_step, args.speed)
            elif key == "r":
                move_speed = change_speed(arm, active_move, move_speed, 0, args.speed)
            elif key == "n":
                arm.set_gripper_state(args.clamp, args.grip_speed)
                print("clamp")
            elif key == "m":
                arm.set_gripper_state(args.release, args.grip_speed)
                print("release")
            if active_move and now - last_range_poll > args.range_poll_interval:
                last_range_poll = now
                if at_limit(arm, active_move):
                    active_move = stop_jog(arm, active_move)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)
        active_move = stop_jog(arm, active_move)


if __name__ == "__main__":
    main()
