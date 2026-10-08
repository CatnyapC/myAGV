"""ROS sensing, virtual obstacles and the sole WebUI chassis velocity publisher."""
from copy import deepcopy
import hashlib
import json
import math
import threading
import time
import uuid

from navigation import Navigation
from robot_safety import fresh, footprint_clear, zones_visible
from web_backend.map_data import local_to_world
from web_backend.simulation import navigation_grid
from web_backend.storage import atomic_json


def grid_dict(message):
    info, q = message.info, message.info.origin.orientation
    return dict(frame=message.header.frame_id, width=info.width, height=info.height,
                resolution_m=info.resolution,
                origin=dict(x_m=info.origin.position.x, y_m=info.origin.position.y,
                            yaw_rad=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))),
                cells=list(message.data))


class RobotROS:
    def __init__(self, directory):
        import rospy
        from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
        from nav_msgs.msg import OccupancyGrid, Path
        from sensor_msgs.msg import LaserScan

        self.ros, self.directory = rospy, directory
        self.Twist, self.InitialPose = Twist, PoseWithCovarianceStamped
        self.nav = Navigation()
        self.nav.publisher.unregister()
        self.nav.publisher = rospy.Publisher('/myagv/input_vel', Twist, queue_size=1)
        self.output = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.initial = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.maps = rospy.Publisher('/navigation_map', OccupancyGrid, queue_size=1, latch=True)
        self.lock = threading.RLock()
        self.source = self.grid = self.derived = None
        self.zones = None
        self.zone_points = []
        self.published_at = float('inf')
        self.costmaps = {}
        self.costmap_received = {}
        self.costmap_saved = 0
        self.scan = None
        self.path = []
        self.path_at = 0
        self.velocity = (0, 0, 0)
        self.velocity_at = 0
        self.control = None
        self.driver_watchdog = False
        self.exclusive = False
        self.last_check = 0
        self.clearance = None
        self.layers_ok = False
        self.amcl_uri = None
        self.subscribers = [
            rospy.Subscriber('/myagv/input_vel', Twist, self.receive_velocity, queue_size=1),
            rospy.Subscriber('/scan', LaserScan, self.receive_scan, queue_size=1),
            rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.receive_path, queue_size=1),
        ]
        for name in ('global', 'local'):
            self.subscribers.append(rospy.Subscriber('/move_base/%s_costmap/costmap' % name,
                OccupancyGrid, lambda msg, name=name: self.receive_costmap(name, msg), queue_size=1, buff_size=4_000_000))
        self.timer = rospy.Timer(rospy.Duration(.05), self.tick)
        rospy.on_shutdown(self.zero)

    def zero(self):
        self.velocity = (0, 0, 0)
        self.velocity_at = 0
        self.output.publish(self.Twist())

    def receive_velocity(self, message):
        if not self.control or self.control.phase not in ('navigating', 'approach') or not self.control.base_enabled:
            return
        values = message.linear.x, message.linear.y, message.angular.z
        if all(math.isfinite(v) for v in values):
            self.velocity, self.velocity_at = values, time.monotonic()

    def receive_scan(self, message):
        self.scan = (time.monotonic(), message)

    def receive_path(self, message):
        if message.header.frame_id == 'map':
            self.path = [[p.pose.position.x, p.pose.position.y] for p in message.poses][::2]
            self.path_at = time.monotonic()

    def set_map(self, source, grid, zones):
        with self.lock:
            self.source, self.grid = deepcopy(source), grid
        self.apply_zones(zones)

    def receive_map(self, message):
        try:
            grid = grid_dict(message)
            if grid['frame'] != 'map' or not 0 < grid['width'] * grid['height'] <= 2_000_000 or len(grid['cells']) != grid['width'] * grid['height'] or not math.isfinite(grid['resolution_m']) or grid['resolution_m'] <= 0:
                raise ValueError('Invalid map geometry/frame')
            grid['cells'] = [-1 if c < 0 else 100 if c >= 50 else 0 for c in grid['cells']]
            digest = hashlib.sha256(json.dumps(grid, sort_keys=True, allow_nan=False).encode()).hexdigest()
            with self.lock:
                marker = self.directory.parent / 'mapping.json'
                mapping = marker.exists()
                map_id = ('slam-' + json.loads(marker.read_text())['session_id'] + '-' + self.ros.get_param('/gmapping/map_session')) if mapping else 'ros-' + digest[:16]
                previous = self.grid or {}
                if previous.get('map_id') == map_id and previous.get('content_hash') == digest:
                    return
                same_map = previous.get('map_id') == map_id
                grid.update(map_id=map_id, revision=previous['revision'] + 1 if same_map else 1, content_hash=digest)
                if not same_map:
                    if self.control.phase != 'resetting_map':
                        self.control.request_stop('Map changed')
                    self.control.localized = False
                zones = self.zones if same_map else dict(map_id=map_id, revision=0, zones=[])
                if not same_map and self.control.zones_path.exists():
                    zones = json.loads(self.control.zones_path.read_text())
                    if zones['map_id'] != map_id:
                        if not mapping:
                            raise ValueError('Saved no-go zones belong to another map; archive/review the saved file first')
                        backup = self.directory.parent / 'map_backups' / str(uuid.uuid4())
                        backup.mkdir(parents=True)
                        for path in (self.control.zones_path, self.control.stations_path):
                            if path.exists():
                                path.rename(backup / path.name)
                        zones = dict(map_id=map_id, revision=0, zones=[])
                self.set_map(message, grid, zones)
                atomic_json(self.directory / 'map.json', grid)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.grid = None
            self.control.request_stop(str(exc))
            self.ros.logerr('Map rejected: %s', exc)

    def apply_zones(self, zones):
        if self.grid is None or zones['map_id'] != self.grid['map_id']:
            raise ValueError('No-go zones belong to a different map')
        derived = navigation_grid(self.grid, zones['zones'])
        mask = navigation_grid({**self.grid, 'cells': [0] * len(self.grid['cells'])}, zones['zones'])
        r, width = self.grid['resolution_m'], self.grid['width']
        points = [local_to_world([((i % width)+.5)*r, ((i//width)+.5)*r], self.grid['origin'])
                  for i, value in enumerate(mask['cells']) if value == 100]
        message = deepcopy(self.source)
        message.data = derived['cells']
        message.header.stamp = self.ros.Time.now()
        message.header.seq = zones['revision']
        with self.lock:
            zones_changed = self.zones != zones
            self.zones, self.derived, self.zone_points = zones, derived, points
            if zones_changed:
                self.published_at = time.monotonic()
                self.costmaps = {}
        self.maps.publish(message)
        atomic_json(self.directory / 'navigation_map.json', {**derived, 'zone_revision': zones['revision']})

    def receive_costmap(self, name, message):
        if message.header.frame_id != 'map' or message.info.width * message.info.height > 2_000_000:
            return
        grid = grid_dict(message)
        if len(grid['cells']) != grid['width'] * grid['height'] or grid['resolution_m'] <= 0:
            return
        with self.lock:
            revision = self.zones['revision'] if self.zones else -1
            published = self.published_at
            points = self.zone_points
        # ponytail: inspect all zone cells; use a native mask if very large zones hurt throughput.
        valid = zones_visible(grid, points)
        age = (self.ros.Time.now() - message.header.stamp).to_sec()
        self.costmap_received[name] = dict(age_s=round(age, 3), mask_applied=valid)
        with self.lock:
            if published == self.published_at and time.monotonic() > published + .4 and -.1 <= age <= 2:
                self.costmaps[name] = (time.monotonic(), revision, valid, grid)
        if name == 'global' and self.grid and time.monotonic() - self.costmap_saved > .5:
            self.costmap_saved = time.monotonic()
            atomic_json(self.directory / 'global_costmap.json', {**grid, 'map_id': self.grid['map_id'],
                        'revision': revision, 'zone_revision': revision, 'stamp_s': time.time()})

    def zones_ready(self):
        now = time.monotonic()
        if not self.zones or not self.layers_ok:
            return False
        return all(name in self.costmaps and fresh(self.costmaps[name][0], now, 2 if name == 'global' else 1)
                   and self.costmaps[name][1] == self.zones['revision'] and self.costmaps[name][2]
                   for name in ('global', 'local'))

    def sensors_ready(self):
        now = time.monotonic()
        for sample, limit in ((self.scan, .5), (self.nav.odom, .5)):
            if not sample or not fresh(sample[0], now, limit):
                return False
            age = (self.ros.Time.now() - sample[1].header.stamp).to_sec()
            if not -.1 <= age <= limit:
                return False
        return any(math.isfinite(r) and self.scan[1].range_min <= r <= self.scan[1].range_max
                   for r in self.scan[1].ranges)

    def validate_goal(self, pose, radius):
        if not self.zones_ready():
            raise RuntimeError('Waiting for both ROS costmaps to apply no-go zones')
        if not footprint_clear(self.derived, pose, radius) or not footprint_clear(self.costmaps['global'][3], pose, radius):
            raise ValueError('Goal clearance intersects a wall, unknown space or no-go zone')

    def stopped(self):
        sample = self.nav.odom
        if not sample or not fresh(sample[0], time.monotonic(), .5):
            return False
        twist = sample[1].twist.twist
        return math.hypot(twist.linear.x, twist.linear.y) <= .01 and abs(twist.angular.z) <= .02

    def confirm_slam_pose(self, pose):
        if (pose is not None and self.grid and self.grid.get('revision', 0) > 0
                and self.grid['map_id'].startswith('slam-')
                and (self.directory.parent / 'mapping.json').exists() and self.sensors_ready()):
            self.control.localized = True

    def tick(self, _event):
        now = time.monotonic()
        control = self.control
        if control and control.phase == 'teleop':
            return
        if control:
            control.watchdog()
        allowed = bool(control and control.base_allowed() and self.driver_watchdog and self.exclusive
                       and self.sensors_ready() and fresh(self.velocity_at, now, .4))
        message = self.Twist()
        if allowed:
            message.linear.x = max(-.1, min(.1, self.velocity[0]))
            message.linear.y = max(-.1, min(.1, self.velocity[1]))
            message.angular.z = max(-.3, min(.3, self.velocity[2]))
        self.output.publish(message)

    def check_publishers(self):
        import rosgraph
        self.driver_watchdog = self.ros.get_param('/myagv_web/driver_watchdog', 0) == 1
        master = rosgraph.Master(self.ros.get_name())
        amcl_uri = master.lookupNode('/gmapping' if (self.directory.parent / 'mapping.json').exists() else '/amcl')
        if self.amcl_uri and amcl_uri != self.amcl_uri and self.control:
            self.control.localized = False
            self.control.request_stop('Localization process restarted; reconfirm pose')
        self.amcl_uri = amcl_uri
        publishers = master.getSystemState()[0]
        self.exclusive = all(set(nodes) <= ({self.ros.get_name()} if topic == '/cmd_vel' else {self.ros.get_name(), '/move_base'})
                             for topic, nodes in publishers if topic in ('/cmd_vel', '/myagv/input_vel'))
        self.layers_ok = True
        for side in ('global', 'local'):
            prefix = '/move_base/%s_costmap/' % side
            if self.ros.get_param(prefix + 'static_layer/map_topic', '') != '/navigation_map' or not self.ros.get_param(prefix + 'static_layer/enabled', False):
                self.layers_ok = False
            if self.clearance is not None and (self.ros.get_param(prefix + 'footprint', '') not in ('', '[]') or
                    abs(self.ros.get_param(prefix + 'robot_radius', 0) - self.clearance) > .001):
                self.clearance = None

    def set_clearance(self, radius):
        from dynamic_reconfigure.client import Client
        for side in ('global', 'local'):
            client = Client('/move_base/%s_costmap' % side, timeout=2)
            try:
                config = client.update_configuration(dict(footprint='[]', robot_radius=radius))
            finally:
                client.close()
            if abs(config['robot_radius'] - radius) > .001 or config['footprint'] not in ('[]', ''):
                raise RuntimeError('ROS costmap rejected measured footprint')
        self.clearance = radius

    def clearance_ready(self, radius):
        return self.clearance == radius

    def initial_pose(self, pose):
        if (self.directory.parent / 'mapping.json').exists():
            raise RuntimeError('Live SLAM determines the map pose; verify and confirm it instead')
        message = self.InitialPose()
        message.header.frame_id, message.header.stamp = 'map', self.ros.Time.now()
        message.pose.pose.position.x, message.pose.pose.position.y = pose['x_m'], pose['y_m']
        half = pose['yaw_rad'] / 2
        message.pose.pose.orientation.z, message.pose.pose.orientation.w = math.sin(half), math.cos(half)
        message.pose.covariance[0] = message.pose.covariance[7] = .25
        message.pose.covariance[35] = math.radians(15) ** 2
        self.initial.publish(message)
