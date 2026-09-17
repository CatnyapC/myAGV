"""ROS1 move_base navigation and shared station recording helpers."""

import argparse
from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import signal
import tempfile
import time


STATIONS = Path(__file__).with_name("stations.json")
# Conservative P340 product limits; J4 is the optional end-effector servo.
# Matches pymycobot 4.0.5 RobotLimit.robot_limit["ultraArmP340"].
JOINT_LIMITS = [(-150, 170), (-20, 90), (-5, 110), (-179, 179)]


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def validate_pose(pose):
    if not isinstance(pose, dict) or set(pose) != {"x_m", "y_m", "yaw_deg"}:
        raise ValueError("base requires x_m, y_m, yaw_deg")
    if not all(number(v) for v in pose.values()):
        raise ValueError("base values must be finite numbers")
    if not -180 <= pose["yaw_deg"] <= 180:
        raise ValueError("yaw_deg must be in -180..180")
    return dict(pose)


def validate_angles(angles):
    if not isinstance(angles, (list, tuple)) or len(angles) not in (3, 4):
        raise ValueError("Expected 3 or 4 P340 joint angles")
    for joint, (angle, (low, high)) in enumerate(zip(angles, JOINT_LIMITS), start=1):
        if not number(angle) or not low <= angle <= high:
            raise ValueError("Invalid P340 J%s angle %r; expected %s..%s degrees" % (joint, angle, low, high))
    return list(angles)


def pickup_angles(angles):
    """Accept measured home-axis tolerance; command the pickup axis at zero."""
    angles = validate_angles(angles)
    if abs(angles[0]) > 1:
        raise ValueError("Side pickup needs J1 within 1 degree of 0; home and re-teach this pose")
    angles[0] = 0.0
    return angles


def validate_station(station):
    if not isinstance(station, dict) or set(station) != {"base", "arm_angles_deg"}:
        raise ValueError("Station requires base and arm_angles_deg")
    return {"base": validate_pose(station["base"]),
            "arm_angles_deg": validate_angles(station["arm_angles_deg"])}


def load_stations(path=STATIONS):
    with open(path, encoding="utf-8") as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError("stations.json must contain an object keyed by item name")
    for name, station in data.items():
        if not name.strip() or name != name.strip():
            raise ValueError("Station names must be nonempty without surrounding spaces")
        validate_station(station)
    return data


def save_station(name, station, path=STATIONS):
    name = name.strip()
    if not name:
        raise ValueError("Station name is empty")
    station = validate_station(station)
    pickup_angles(station["arm_angles_deg"])
    path = Path(path)
    data = load_stations(path) if path.exists() else {}
    data[name] = station
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         delete=False) as stream:
            temporary = stream.name
            json.dump(data, stream, indent=2, ensure_ascii=False, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary and os.path.exists(temporary):
            os.unlink(temporary)


class _ArmDeadlineExpired(BaseException):
    """Bypass the SDK's broad except Exception retry loops."""


@contextmanager
def arm_deadline(seconds=3):
    """Bound SDK calls that can otherwise wait forever; main thread on Linux/macOS."""
    def expired(_signum, _frame):
        raise _ArmDeadlineExpired()

    previous = signal.signal(signal.SIGALRM, expired)
    try:
        signal.setitimer(signal.ITIMER_REAL, seconds)
        yield
    except _ArmDeadlineExpired:
        raise TimeoutError("P340 did not respond in time") from None
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, previous)


def read_arm_angles(arm):
    with arm_deadline():
        return validate_angles(arm.get_angles_info())


def wait_arm(arm, target=None, timeout=30):
    """Require stable measured angles and, for moves, agreement with the target."""
    if target is not None:
        print("Waiting for measured arm target: %s" % target, flush=True)
    deadline = time.monotonic() + timeout
    previous = None
    stable_reads = 0
    while time.monotonic() < deadline:
        angles = read_arm_angles(arm)
        if target is not None and len(angles) != len(target):
            raise ValueError("P340 feedback and target joint counts differ")
        # "Moving end" is not reliable for idle/no-op commands. Use feedback
        # for completion, as the original command controller does for XYZ moves.
        stable = previous is not None and len(previous) == len(angles) and all(
            abs(a - b) <= 0.2 for a, b in zip(angles, previous))
        reached = target is None or all(abs(a - b) <= 1 for a, b in zip(angles, target))
        stable_reads = stable_reads + 1 if stable else 0
        if stable_reads >= 3 and reached:
            if target is not None:
                print("Arm target reached: %s" % angles, flush=True)
            return angles
        previous = angles
        time.sleep(0.2)
    raise TimeoutError("P340 did not stop at the requested pose; target=%s last_angles=%s" % (target, previous))


class Navigation:
    def __init__(self, approach_distance=0.3, approach_speed=0.03, approach_clearance=0.25):
        import actionlib
        import rospy
        import tf2_ros
        from geometry_msgs.msg import Twist
        from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
        from nav_msgs.msg import Odometry

        self.ros, self.tf_errors = rospy, (tf2_ros.TransformException,)
        self.goal_type, self.twist_type = MoveBaseGoal, Twist
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf)
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.active = False
        self.odom = None
        self.subscriber = rospy.Subscriber("odom", Odometry, self._odom, queue_size=1)
        self.publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.approach_settings = (approach_distance, approach_speed, approach_clearance)
        self.aligner = None

    def _odom(self, message):
        self.odom = (time.monotonic(), message)

    def get_pose(self, timeout=3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                transform = self.tf.lookup_transform("map", "base_footprint", self.ros.Time(0))
                age = (self.ros.Time.now() - transform.header.stamp).to_sec()
                if not -0.1 <= age <= 1:
                    raise ValueError("Map localization is stale")
                p, q = transform.transform.translation, transform.transform.rotation
                values = [q.x, q.y, q.z, q.w]
                if not all(number(v) for v in values) or abs(sum(v*v for v in values) - 1) > 0.01:
                    raise ValueError("Invalid localization quaternion")
                yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
                return validate_pose({"x_m": p.x, "y_m": p.y, "yaw_deg": math.degrees(yaw)})
            except self.tf_errors:
                if self.ros.is_shutdown():
                    raise RuntimeError("ROS shutdown")
                time.sleep(0.05)
        raise RuntimeError("No map -> base_footprint transform; initialize localization first")

    def wait_stopped(self, timeout=5):
        start = time.monotonic()
        quiet_since = None
        while time.monotonic() - start < timeout:
            if self.ros.is_shutdown():
                raise RuntimeError("ROS shutdown")
            now = time.monotonic()
            sample = self.odom
            stopped = False
            if sample and sample[0] >= start and now - sample[0] <= 0.5:
                message = sample[1]
                age = (self.ros.Time.now() - message.header.stamp).to_sec()
                twist = message.twist.twist
                values = [twist.linear.x, twist.linear.y, twist.angular.z]
                stopped = (-0.1 <= age <= 0.5 and all(number(v) for v in values)
                           and math.hypot(*values[:2]) <= 0.01 and abs(values[2]) <= 0.02)
            quiet_since = (now if quiet_since is None else quiet_since) if stopped else None
            if quiet_since is not None and now - quiet_since >= 0.5:
                return
            time.sleep(0.05)
        raise TimeoutError("Base not stationary or fresh odometry unavailable")

    def cancel(self):
        if not self.active:
            return
        self.client.cancel_goal()
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            self.publisher.publish(self.twist_type())
            if self.client.get_state() in (2, 3, 4, 5, 8):
                self.active = False
                return
            time.sleep(0.1)
        raise RuntimeError("Navigation cancellation unconfirmed; use hardware stop")

    def go_to(self, pose, timeout=120):
        pose = validate_pose(pose)
        if not number(timeout) or timeout <= 0:
            raise ValueError("Navigation timeout must be positive and finite")
        if not self.client.wait_for_server(self.ros.Duration(5)):
            raise RuntimeError("move_base unavailable")
        self.get_pose()  # Refuse motion without current localization.
        goal = self.goal_type()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = self.ros.Time.now()
        goal.target_pose.pose.position.x = pose["x_m"]
        goal.target_pose.pose.position.y = pose["y_m"]
        half_yaw = math.radians(pose["yaw_deg"]) / 2
        goal.target_pose.pose.orientation.z = math.sin(half_yaw)
        goal.target_pose.pose.orientation.w = math.cos(half_yaw)
        self.active = True
        try:
            self.client.send_goal(goal)
            print("Navigation goal sent; waiting for move_base...", flush=True)
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if self.ros.is_shutdown():
                    raise RuntimeError("ROS shutdown")
                state = self.client.get_state()
                if state in (2, 3, 4, 5, 8, 9):
                    if state != 9:
                        self.active = False
                    if state != 3:
                        raise RuntimeError("Navigation failed: %s" % self.client.get_goal_status_text())
                    self.wait_stopped()
                    actual = self.get_pose()
                    distance = math.hypot(actual["x_m"] - pose["x_m"], actual["y_m"] - pose["y_m"])
                    angle = abs((actual["yaw_deg"] - pose["yaw_deg"] + 180) % 360 - 180)
                    if distance > 0.05 or angle > 5:
                        raise RuntimeError("Arrival outside 5 cm / 5 degree tolerance")
                    return
                time.sleep(0.05)
            raise TimeoutError("Navigation timed out")
        finally:
            self.cancel()

    def go_to_pickup(self, pose, timeout=120):
        if self.aligner is None:
            from pickup_alignment import PickupAlignment
            self.aligner = PickupAlignment(self, *self.approach_settings)
        self.aligner.approach(validate_pose(pose), timeout)


def install_interrupts():
    def interrupt(_signum, _frame):
        raise KeyboardInterrupt
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, interrupt)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("pose", "go", "roundtrip"))
    parser.add_argument("station", nargs="?")
    parser.add_argument("--stations", type=Path, default=STATIONS)
    from pickup_alignment import add_pickup_args
    add_pickup_args(parser)
    # Parse first so --help also works without ROS installed.
    args = parser.parse_args()
    if args.command != "pose" and not args.station:
        parser.error("station name required")
    target = None
    if args.station:
        station = load_stations(args.stations)[args.station]
        pickup_angles(station["arm_angles_deg"])
        target = station["base"]
    import rospy
    rospy.init_node("myagv_navigation_client", disable_signals=True)
    install_interrupts()
    nav = Navigation(args.approach_distance, args.approach_speed, args.approach_clearance)
    try:
        nav.wait_stopped()
        start = nav.get_pose()
        if args.command == "pose":
            print(json.dumps(start, indent=2))
        else:
            nav.go_to_pickup(target)
            if args.command == "roundtrip":
                nav.go_to(start)
    finally:
        nav.cancel()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit("Interrupted")
    except Exception as exc:
        raise SystemExit(str(exc))
