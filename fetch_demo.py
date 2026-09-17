"""Fetch one recorded item, returning to this run's initial base AND arm pose."""

import argparse
from pathlib import Path
import sys
import time

from navigation import (Navigation, STATIONS, arm_deadline, install_interrupts,
                        load_stations, number, pickup_angles, validate_angles,
                        validate_station, wait_arm)


# Set once after an empty, low-speed hardware check. Never guess mounted-arm clearance.
# Alternatively pass --transport-angles J1 J2 J3 [J4]. No dropoff configuration.
TRANSPORT_ANGLES = None


def move_arm(arm, angles, speed):
    with arm_deadline():
        arm.set_angles(validate_angles(angles), speed)
    wait_arm(arm, angles)


def grip(arm, value, args):
    with arm_deadline():
        arm.set_gripper_state(value, args.grip_speed)
    # ponytail: timed gripper completion, not object detection; add sensing if needed.
    time.sleep(args.grip_wait)


def fetch(nav, arm, station, transport, args):
    station = validate_station(station)
    transport = pickup_angles(transport)
    station["arm_angles_deg"] = pickup_angles(station["arm_angles_deg"])
    try:
        nav.wait_stopped()
        start_arm = wait_arm(arm)
        start_base = nav.get_pose()
        if not len(start_arm) == len(transport) == len(station["arm_angles_deg"]):
            raise ValueError("Start, transport and grasp must have the same joint count")
        print("Return pose captured: base=%s arm=%s" % (start_base, start_arm))
        # No homing here: preserve the user's initial placement pose.
        with arm_deadline():
            arm.set_mode(0)
        print("Transport pose; going to item")
        move_arm(arm, transport, args.arm_speed)
        nav.go_to_pickup(station["base"], args.nav_timeout)
        print("At item; grasping")
        grip(arm, args.release, args)
        move_arm(arm, station["arm_angles_deg"], args.arm_speed)
        grip(arm, args.clamp, args)
        move_arm(arm, transport, args.arm_speed)
        print("Returning to startup pose")
        nav.go_to(start_base, args.nav_timeout)
        move_arm(arm, start_arm, args.arm_speed)
        grip(arm, args.release, args)
        print("Done; gripper opened at startup pose (grasp success not sensed)")
    except BaseException:
        try:
            nav.cancel()
        except Exception as exc:
            print("Navigation cleanup failed: %s" % exc, file=sys.stderr)
        try:
            # Best effort only: SDK jog-stop is not a certified stop for queued moves.
            with arm_deadline():
                arm.set_jog_stop()
        except Exception as exc:
            print("Arm cleanup failed: %s" % exc, file=sys.stderr)
        raise


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("station")
    parser.add_argument("--p340-port", required=True)
    parser.add_argument("--arm-homed", action="store_true", required=True,
                        help="arm homed before positioning for this run; no power loss since")
    parser.add_argument("--stations", type=Path, default=STATIONS)
    parser.add_argument("--transport-angles", type=float, nargs="+", default=TRANSPORT_ANGLES)
    parser.add_argument("--arm-speed", type=int, default=30)
    parser.add_argument("--grip-speed", type=int, default=500)
    parser.add_argument("--grip-wait", type=float, default=1.5)
    parser.add_argument("--clamp", type=int, default=0)
    parser.add_argument("--release", type=int, default=100)
    parser.add_argument("--nav-timeout", type=float, default=120)
    from pickup_alignment import add_pickup_args
    add_pickup_args(parser)
    args = parser.parse_args(argv)
    try:
        pickup_angles(args.transport_angles)
    except ValueError as exc:
        parser.error("Set calibrated TRANSPORT_ANGLES or --transport-angles: %s" % exc)
    for key, low, high in (("arm_speed", 1, 200), ("grip_speed", 1, 1500),
                           ("grip_wait", 0.5, 10), ("clamp", 0, 100),
                           ("release", 0, 100), ("nav_timeout", 1, 3600)):
        value = getattr(args, key)
        if not number(value) or not low <= value <= high:
            parser.error("%s must be in %s..%s" % (key, low, high))
    if args.clamp >= args.release:
        parser.error("clamp must be less than release")
    return args


def main():
    args = parse_args()
    station = load_stations(args.stations)[args.station]
    pickup_angles(station["arm_angles_deg"])
    import rospy
    from pymycobot.ultraArmP340 import ultraArmP340

    rospy.init_node("myagv_fetch", disable_signals=True)
    install_interrupts()
    nav = Navigation(args.approach_distance, args.approach_speed, args.approach_clearance)
    with arm_deadline():
        arm = ultraArmP340(args.p340_port, 115200)
    try:
        fetch(nav, arm, station, args.transport_angles, args)
    finally:
        arm.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit("Interrupted; inspect robot before restarting")
    except Exception as exc:
        raise SystemExit(str(exc))
