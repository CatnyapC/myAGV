"""Align beside a left-side pickup using slow longitudinal motion and small turns."""

import math
import time
from functools import partial


def bounded(value, low, high):
    import argparse
    try:
        value = float(value)
    except ValueError:
        raise argparse.ArgumentTypeError("Expected a number")
    if not math.isfinite(value) or not low <= value <= high:
        raise argparse.ArgumentTypeError("Expected %s..%s" % (low, high))
    return value


def add_pickup_args(parser):
    parser.add_argument("--approach-distance", type=partial(bounded, low=0.1, high=0.6), default=0.3,
                        help="staging distance along pickup heading, meters (default 0.3)")
    parser.add_argument("--approach-speed", type=partial(bounded, low=0.01, high=0.05), default=0.03,
                        help="forward/backward alignment speed, m/s (default 0.03)")
    parser.add_argument("--approach-clearance", type=partial(bounded, low=0.15, high=1.0), default=0.25,
                        help="clear circle radius around folded robot/load, meters (default 0.25)")


def staging_pose(pose, distance):
    yaw = math.radians(pose["yaw_deg"])
    return dict(pose, x_m=pose["x_m"] - distance * math.cos(yaw),
                y_m=pose["y_m"] - distance * math.sin(yaw))


def corridor_error(actual, target):
    yaw = math.radians(target["yaw_deg"])
    dx, dy = target["x_m"] - actual["x_m"], target["y_m"] - actual["y_m"]
    return (dx * math.cos(yaw) + dy * math.sin(yaw),
            -dx * math.sin(yaw) + dy * math.cos(yaw),
            (target["yaw_deg"] - actual["yaw_deg"] + 180) % 360 - 180)


def corridor_clear(grid, x, y, yaw, direction, radius):
    """Conservative swept circle over the next 10 cm; unknown/outside is blocked."""
    info = grid.info
    q = info.origin.orientation
    if abs(q.x) + abs(q.y) + abs(q.z) > 1e-6 or abs(q.w - 1) > 1e-6:
        raise RuntimeError("Pickup alignment requires an unrotated local costmap grid")
    res = info.resolution
    if res <= 0 or len(grid.data) != info.width * info.height:
        raise RuntimeError("Invalid pickup alignment costmap")
    ox, oy = info.origin.position.x, info.origin.position.y
    # Include half a cell diagonal so an occupied cell edge cannot be missed.
    margin = radius + res / math.sqrt(2)
    for step in (0, 0.05, 0.1):
        cx = x + direction * step * math.cos(yaw)
        cy = y + direction * step * math.sin(yaw)
        left, right = math.floor((cx - margin - ox) / res), math.floor((cx + margin - ox) / res)
        bottom, top = math.floor((cy - margin - oy) / res), math.floor((cy + margin - oy) / res)
        if left < 0 or bottom < 0 or right >= info.width or top >= info.height:
            return False
        for row in range(bottom, top + 1):
            for col in range(left, right + 1):
                px, py = ox + (col + 0.5) * res, oy + (row + 0.5) * res
                if math.hypot(px - cx, py - cy) <= margin and grid.data[row * info.width + col] != 0:
                    return False
    return True


class PickupAlignment:
    def __init__(self, nav, distance=0.3, speed=0.03, clearance=0.25):
        from nav_msgs.msg import OccupancyGrid
        from sensor_msgs.msg import LaserScan
        for value, low, high in ((distance, 0.1, 0.6), (speed, 0.01, 0.05), (clearance, 0.15, 1.0)):
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError("Pickup alignment distance/speed/clearance outside allowed range")
        self.nav, self.distance, self.speed, self.clearance = nav, distance, speed, clearance
        self.grid = self.scan = None
        self.grid_sub = nav.ros.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid,
                                           self._grid, queue_size=1)
        self.scan_sub = nav.ros.Subscriber("scan", LaserScan, self._scan, queue_size=1)

    def _grid(self, msg):
        self.grid = (time.monotonic(), msg)

    def _scan(self, msg):
        self.scan = (time.monotonic(), msg)

    def exclusive(self):
        import rosgraph
        publishers = rosgraph.Master(self.nav.ros.get_name()).getSystemState()[0]
        topic = self.nav.ros.resolve_name("cmd_vel")
        allowed = {self.nav.ros.get_name(), "/move_base"}
        for name, nodes in publishers:
            if name == topic and set(nodes) - allowed:
                raise RuntimeError("Close other cmd_vel controllers before pickup alignment: %s" % (set(nodes) - allowed))
        # No direct velocity control while any navigation goal is pending/active.
        if self.nav.client.get_state() in (0, 1, 6, 7):
            raise RuntimeError("Another navigation goal is active; pickup alignment refused")

    def check_clear(self, direction):
        nav = self.nav
        for sample, age_limit, label in ((self.grid, 1.0, "local costmap"),
                                         (self.scan, 0.5, "LiDAR"), (nav.odom, 0.5, "odometry")):
            if sample is None:
                raise RuntimeError("Pickup alignment needs fresh " + label)
            received, msg = sample
            age = (nav.ros.Time.now() - msg.header.stamp).to_sec()
            if time.monotonic() - received > age_limit or not -0.1 <= age <= age_limit:
                raise RuntimeError("Pickup alignment stopped: stale " + label)
        scan = self.scan[1]
        if not any(math.isfinite(r) and scan.range_min <= r <= scan.range_max for r in scan.ranges):
            raise RuntimeError("Pickup alignment stopped: no usable LiDAR returns")
        grid = self.grid[1]
        tf = nav.tf.lookup_transform(grid.header.frame_id, "base_footprint", nav.ros.Time(0))
        if not -0.1 <= (nav.ros.Time.now() - tf.header.stamp).to_sec() <= 0.5:
            raise RuntimeError("Pickup alignment stopped: stale costmap transform")
        p, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        if not corridor_clear(grid, p.x, p.y, yaw, direction, self.clearance):
            raise RuntimeError("Pickup corridor blocked or unknown; stopped")

    def align(self, target):
        nav = self.nav
        self.exclusive()
        deadline = time.monotonic() + self.distance / self.speed + 20
        best = float("inf")
        progress_at = time.monotonic()
        try:
            while time.monotonic() < deadline:
                if nav.ros.is_shutdown():
                    raise RuntimeError("ROS shutdown")
                actual = nav.get_pose(timeout=0.1)
                along, across, angle = corridor_error(actual, target)
                if abs(across) > 0.02 or abs(angle) > 15:
                    raise RuntimeError("Pickup misaligned (>2 cm sideways / 15 degrees); re-stage")
                if abs(along) > self.distance + 0.06:
                    raise RuntimeError("Pickup localization jump or approach too long")
                if abs(along) <= 0.01 and abs(angle) <= 1:
                    break
                # Correct heading first, then translate in either direction.
                # Small overshoots can reverse; no large turns near the item.
                direction = 0 if abs(angle) > 1 else (1 if along > 0 else -1)
                self.check_clear(direction)
                error = abs(along) + 0.2 * abs(math.radians(angle))
                if error < best - 0.005:
                    best, progress_at = error, time.monotonic()
                elif time.monotonic() - progress_at > 3:
                    raise RuntimeError("Pickup stopped: no measured progress")
                cmd = nav.twist_type()
                cmd.linear.x = direction * self.speed
                if direction == 0:
                    cmd.angular.z = math.copysign(min(0.05, max(0.03, abs(math.radians(angle)))), angle)
                nav.publisher.publish(cmd)
                time.sleep(0.1)
            else:
                raise TimeoutError("Pickup alignment timed out")
        finally:
            # The stock chassis has no independent command watchdog.
            for _ in range(3):
                nav.publisher.publish(nav.twist_type())
                time.sleep(0.05)
        nav.wait_stopped()
        along, across, angle = corridor_error(nav.get_pose(timeout=0.1), target)
        if abs(along) > 0.02 or abs(across) > 0.02 or abs(angle) > 2:
            raise RuntimeError("Pickup final pose outside 2 cm / 2 degree tolerance")

    def approach(self, target, timeout=120):
        self.exclusive()
        actual = self.nav.get_pose()
        # Choose the closer longitudinal staging point, then approach forwards
        # or backwards. The saved heading always keeps the object on the left.
        candidates = [staging_pose(target, distance) for distance in (self.distance, -self.distance)]
        stage = min(candidates, key=lambda p: math.hypot(p["x_m"] - actual["x_m"],
                                                       p["y_m"] - actual["y_m"]))
        print("Side-pickup staging pose: %s" % stage, flush=True)
        self.nav.go_to(stage, timeout)
        print("Aligning beside item: forward/backward with small heading corrections", flush=True)
        self.align(target)
