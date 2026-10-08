"""Single P340 owner and cancellable WebUI tasks. No motion at startup."""
from collections import OrderedDict
from contextlib import suppress
import fcntl
import json
import math
import os
from pathlib import Path
import queue
import shutil
import signal
import socketserver
import subprocess
import threading
import time
import uuid

from navigation import arm_deadline, load_stations, pickup_angles, save_station, validate_angles, wait_arm
from fetch_calibration import FETCH_VISION_TIMEOUT_S, alignment_command, calibration_config, pickup_delta
from P340.keyboard_control import LIMITS
from robot_safety import fresh, lease_valid, manual_vector, number, footprint_clear
from web_backend.map_data import validate_rectangle
from web_backend.photos import digest_station
from web_backend.item_locations import approach_step, location_config
from web_backend.simulation import validate_pose
from web_backend.storage import atomic_json


class Stopped(BaseException):
    pass


class UpdatePaused(BaseException):
    pass


class Control:
    def __init__(self, robot, directory, capture):
        self.robot, self.directory, self.capture = robot, Path(directory), capture
        self.directory.mkdir(parents=True, exist_ok=True)
        self.lock_file = (self.directory / 'control.lock').open('a')
        fcntl.flock(self.lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.boot_id = str(uuid.uuid4())
        self.phase, self.status = 'idle', 'Ready; initialize localization before automatic travel'
        self.task_id = self.owner = None
        self.deadline = 0
        self.manual_lease = False
        self.stop_epoch = 0
        self.input = None
        self.manual_args = {}
        self.stop_pending = None
        self.executing = False
        self.observing = False
        self.stopping = False
        self.confirmation = None
        self.localized = False
        self.arm = None
        self.arm_homed = False
        self.home_cancelled = False
        self.angles = None
        self.arm_stamp = 0
        self.arm_error = ''
        self.teleop_pid = None
        self.goal = None
        self.base_enabled = False
        self.map_update = None
        self.queue = queue.Queue(maxsize=1)
        self.results, self.requests = OrderedDict(), OrderedDict()
        self.config_path = self.directory.parent / 'robot_config.json'
        self.config = json.loads(self.config_path.read_text()) if self.config_path.exists() else dict(transport_angles=None, clearance_m=.20)
        number(self.config['clearance_m'], .15, 1, 'Measured clearance radius')
        number(self.config.get('small_obstacle_m', .15), 0, .5, 'Small obstacle size')
        if self.config['transport_angles'] is not None:
            validate_angles(self.config['transport_angles'])
        self.stations_path = Path(__file__).with_name('stations.json')
        self.zones_path = self.directory.parent / 'hardware_no_go_zones.json'
        robot.control = self
        robot.nav.guard = self.travel_guard
        signal.signal(signal.SIGUSR1, self.interrupt)
        self.receive_lock = threading.Lock()
        self.start_socket()

    def interrupt(self, _signum, _frame):
        if self.executing and not self.stopping:
            raise Stopped(self.stop_pending or 'Stopped')

    def start_socket(self):
        control = self

        class Handler(socketserver.StreamRequestHandler):
            def handle(self):
                self.request.settimeout(.5)
                try:
                    line = self.rfile.readline(65537)
                    if len(line) > 65536 or not line.endswith(b'\n'):
                        raise ValueError('Invalid control packet')
                    packet = json.loads(line)
                    if packet.get('op') == 'stop':
                        result = control.receive(packet)
                    else:
                        with control.receive_lock:
                            result = control.receive(packet)
                except (ValueError, KeyError, TypeError, RuntimeError, OSError) as exc:
                    result = dict(error=str(exc))
                self.wfile.write((json.dumps(result, allow_nan=False) + '\n').encode())

        class Server(socketserver.ThreadingUnixStreamServer):
            daemon_threads = True

        path = self.directory / 'control.sock'
        if path.exists():
            path.unlink()
        self.server = Server(str(path), Handler)
        os.chmod(path, 0o600)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def receive(self, packet):
        if not isinstance(packet, dict):
            raise ValueError('Invalid control packet')
        kind = packet.get('op')
        if kind == 'result':
            return self.results.get(packet['id'], dict(status='pending'))
        if kind == 'stop':
            if packet.get('passive') is True and self.phase in ('teleop', 'handoff', 'resetting_map'):
                return dict(status='ignored', reason='Map reset keeps hardware stopped' if self.phase == 'resetting_map' else 'TELEOP owns hardware')
            if (packet.get('passive') is True and self.phase == 'idle' and not self.executing
                    and not self.stopping and not self.stop_pending and not self.base_enabled and self.queue.empty()):
                return dict(status='ignored', reason='Hardware already idle')
            self.request_stop(packet.get('reason', 'Stopped'))
            return dict(status='stopping')
        if kind == 'teleop':
            pid, active = packet.get('pid'), packet.get('active')
            if type(pid) is not int or pid <= 0 or type(active) is not bool:
                raise ValueError('Valid TELEOP process and ownership action required')
            command_id = str(uuid.UUID(packet['id']))
            if not active and self.teleop_pid is None:
                return dict(status='completed')
            if active and self.phase not in ('idle', 'fault'):
                raise RuntimeError('Stop the WebUI task before starting TELEOP')
            if not active and self.teleop_pid != pid:
                raise RuntimeError('Another TELEOP process owns hardware')
            if self.executing or self.stopping or self.stop_pending or not self.queue.empty():
                raise RuntimeError('Hardware handoff busy; retry after stopping')
            if active:
                os.kill(pid, 0)
            self.queue.put_nowait(dict(id=command_id, op='teleop', pid=pid,
                                       type='teleop_acquire' if active else 'teleop_release', stop_epoch=self.stop_epoch))
            return dict(status='accepted', id=command_id)
        if packet.get('boot_id') != self.boot_id or not lease_valid(packet.get('deadline'), time.monotonic()):
            raise ValueError('Expired controller session; reconnect and retry explicitly')
        if kind == 'heartbeat':
            if packet.get('session_id') == self.owner:
                self.deadline = min(packet['deadline'], time.monotonic() + .45) if self.manual_lease else packet['deadline']
                self.input = packet.get('input')
            return dict(status='ok')
        if kind not in ('command', 'observe_goal'):
            raise ValueError('Unknown control packet')
        command_id = str(uuid.UUID(packet['id']))
        fingerprint = json.dumps({k: v for k, v in packet.items() if k not in ('deadline', 'boot_id')}, sort_keys=True)
        if command_id in self.requests:
            if self.requests[command_id] != fingerprint:
                raise ValueError('Command ID reused with different arguments')
            return self.results.get(command_id, dict(status='accepted', id=command_id))
        if packet.get('stop_epoch') != self.stop_epoch:
            raise ValueError('Stop state changed; refresh and issue a new action')
        command = packet.get('type')
        if kind == 'observe_goal':
            if command != 'capture_grasp_goal' or self.phase != 'teleop' or self.observing:
                raise RuntimeError('TELEOP camera capture unavailable or already running')
            if packet.get('map_id') != (self.robot.grid or {}).get('map_id') or not self.localized:
                raise ValueError('Current map and localization required')
            self.requests[command_id] = fingerprint
            self.observing = True

            def observe():
                try:
                    captures = {}
                    for camera in ('front', 'arm'):
                        if self.phase != 'teleop' or self.stop_epoch != packet['stop_epoch']:
                            raise RuntimeError('TELEOP changed during camera capture')
                        captures[camera] = self.capture(camera, grasp_goal=True, teleop_observation=True)['capture_id']
                    if self.phase != 'teleop' or self.stop_epoch != packet['stop_epoch']:
                        raise RuntimeError('TELEOP changed during camera capture')
                    self.finish(packet, dict(status='completed', captures=captures))
                except Exception as exc:
                    self.finish(packet, dict(status='failed', error=str(exc)))
                finally:
                    self.observing = False

            threading.Thread(target=observe, daemon=True).start()
            return dict(status='accepted', id=command_id)
        if command in ('teleop_acquire', 'teleop_release'):
            raise ValueError('TELEOP handoff requires the local socket')
        if command in ('update_pause', 'update_start') and (command == 'update_start' or (self.map_update or {}).get('active')):
            update = self.map_update
            if not update or not update['active'] or packet.get('task_id') != self.task_id or packet.get('session_id') != self.owner:
                raise ValueError('Current map update and owning tab required')
            if packet.get('map_id') != update['map_id'] or packet.get('expected_revision') != update['zone_revision']:
                raise ValueError('Map or no-go revision changed')
            if command == 'update_start':
                if self.phase != 'update_paused':
                    raise ValueError('Wait for measured update pause before starting updates')
                if update['storage_error']:
                    raise ValueError('Repair photo storage, then STOP and start a new update')
                if time.monotonic() >= update['deadline']:
                    update['deadline'] = time.monotonic() + update['limit_s']
                update['pause_requested'] = False
            else:
                update['pause_requested'], update['pause_reason'] = True, 'Map updating paused by operator'
                self.base_enabled = False
                self.robot.zero()
                self.robot.nav.client.cancel_all_goals()
            self.requests[command_id] = fingerprint
            self.finish(packet, dict(status='completed'))
            return self.results[command_id]
        if self.phase in ('review_grasp', 'verify_grasp') and packet.get('session_id') == self.owner:
            if command == 'confirm' and packet.get('task_id') == self.task_id and packet.get('stage') == self.phase:
                self.confirmation = self.phase
                return dict(status='accepted')
            if command != 'align_step' or self.phase != 'review_grasp':
                raise RuntimeError('Confirm the current grasp stage or stop')
        elif (self.phase != 'idle' and not (self.phase == 'fault' and command == 'recover_stop')) or self.executing or not self.queue.empty():
            raise RuntimeError('Robot is busy; stop before starting another action')
        if packet.get('map_id') != (self.robot.grid or {}).get('map_id'):
            raise ValueError('Map changed; reload before issuing commands')
        if self.robot.zones is None or packet.get('expected_revision') != self.robot.zones['revision']:
            raise ValueError('No-go revision changed; reload before issuing commands')
        if not isinstance(packet.get('session_id'), str) or not packet['session_id']:
            raise ValueError('Live control session required')
        self.manual_lease = command == 'manual'
        self.owner = packet['session_id']
        self.deadline = min(packet['deadline'], time.monotonic() + .45) if self.manual_lease else packet['deadline']
        self.requests[command_id] = fingerprint
        try:
            self.queue.put_nowait(packet)
        except queue.Full:
            self.requests.pop(command_id, None)
            raise RuntimeError('Another action is queued') from None
        return dict(status='accepted', id=command_id)

    def request_stop(self, reason):
        if self.teleop_pid is not None:
            with suppress(ProcessLookupError):
                os.kill(self.teleop_pid, signal.SIGTERM)
            self.teleop_pid = None
        self.stop_epoch += 1
        self.base_enabled = False
        self.deadline = 0
        self.input = None
        self.robot.zero()
        self.robot.nav.client.cancel_all_goals()
        if not self.stopping:
            first = self.stop_pending is None
            self.stop_pending = str(reason)
            if first and self.executing and not self.stopping:
                os.kill(os.getpid(), signal.SIGUSR1)

    def watchdog(self):
        if self.phase in ('idle', 'fault', 'teleop', 'handoff', 'resetting_map') or self.stopping:
            return
        reason = None
        if not lease_valid(self.deadline, time.monotonic()):
            reason = 'Control heartbeat expired'
        elif not self.robot.driver_watchdog or not self.robot.exclusive:
            reason = 'Driver watchdog unavailable or competing chassis publisher'
        elif not self.robot.sensors_ready():
            reason = 'LiDAR or odometry stale'
        elif self.base_enabled and self.phase != 'manual' and not self.robot.zones_ready():
            reason = 'ROS no-go enforcement unavailable'
        if reason:
            self.request_stop(reason)
        update = self.map_update
        if update and update['active'] and not update['pause_requested'] and time.monotonic() >= update['deadline']:
            update['pause_requested'], update['pause_reason'] = True, 'Update time limit reached; map retained'
            self.base_enabled = False
            self.robot.zero()
            self.robot.nav.client.cancel_all_goals()

    def base_allowed(self):
        return self.base_enabled and not self.stop_pending and lease_valid(self.deadline, time.monotonic())

    def require_arm(self, homed=True):
        if self.arm is None:
            raise RuntimeError('P340 unavailable: ' + self.arm_error)
        if homed and not self.arm_homed:
            raise RuntimeError('Home arm or explicitly confirm already homed')

    def read_arm(self):
        self.require_arm(False)
        with arm_deadline(.4):
            self.angles = validate_angles(self.arm.get_angles_info())
        self.arm_stamp = time.monotonic()
        return self.angles

    def connect_arm(self):
        from pymycobot.ultraArmP340 import ultraArmP340
        if self.arm:
            return
        with arm_deadline(3):
            self.arm = ultraArmP340(os.environ.get('MYAGV_P340_PORT', '/dev/ttyUSB0'), 115200)
            self.arm._serial_port.exclusive = True
        self.arm_homed = False
        self.read_arm()
        self.arm_error = ''

    def travel_guard(self, pose):
        if not lease_valid(self.deadline, time.monotonic()) or self.stop_pending:
            raise Stopped('Control lease expired')
        update = self.map_update
        if update and update['active']:
            if not self.robot.grid or self.robot.grid['map_id'] != update['map_id'] or self.robot.zones['revision'] != update['zone_revision']:
                raise Stopped('Update map or no-go revision changed')
            if not update['pause_requested'] and time.monotonic() >= update['deadline']:
                update['pause_requested'], update['pause_reason'] = True, 'Update time limit reached; map retained'
                self.base_enabled = False
                self.robot.zero()
                self.robot.nav.client.cancel_all_goals()
            if update['pause_requested']:
                raise UpdatePaused()
        if not self.localized:
            raise RuntimeError('Set and confirm localization first')
        if not self.robot.driver_watchdog or not self.robot.exclusive or not self.robot.sensors_ready():
            raise RuntimeError('Chassis watchdog, exclusive output and fresh sensors required')
        self.robot.nav.get_pose(timeout=.1)
        if not self.robot.clearance_ready(self.config['clearance_m']):
            raise RuntimeError('Planner footprint does not match measured clearance')
        self.robot.validate_goal(pose, self.config['clearance_m'])

    def fold(self):
        self.require_arm()
        if self.config['transport_angles'] is None:
            raise RuntimeError('Record a measured folded transport pose and clearance first')
        self.phase, self.status = 'transport', 'Folding arm for travel'
        self.move_arm(self.config['transport_angles'])

    def move_arm(self, angles):
        self.require_arm()
        target, start = validate_angles(angles), self.read_arm()
        if len(target) != len(start):
            raise ValueError('Arm joint counts differ')
        # M15 only promises jog stop. Bound each absolute command so cancellation
        # stops issuing targets, even if firmware completes the current waypoint.
        steps = max(1, math.ceil(max(abs(a-b) for a,b in zip(start,target)) / 2))
        for index in range(1, steps+1):
            if self.map_update and self.map_update['active']:
                self.wait_update()
            if self.stop_pending or not lease_valid(self.deadline, time.monotonic()):
                raise Stopped('Arm task lease expired')
            waypoint = [a+(b-a)*index/steps for a,b in zip(start,target)]
            with arm_deadline(.4):
                self.arm.set_angles(waypoint, 30)
            self.angles = wait_arm(self.arm, waypoint, timeout=5)
            self.arm_stamp = time.monotonic()

    def grip(self, value):
        self.require_arm()
        number(value, 0, 100, 'Gripper value')
        with arm_deadline(.4):
            self.arm.set_gripper_state(value, 500)
        time.sleep(1.5)

    def arm_step(self, axis, direction, pickup=False, distance=1):
        self.require_arm()
        if axis not in LIMITS or direction not in (-1, 1):
            raise ValueError('Invalid arm step')
        number(distance, .001, 1, 'Arm step distance')
        with arm_deadline(.4):
            coords = self.arm.get_coords_info()
        if not coords or len(coords) < 3 or not all(type(v) in (float, int) and math.isfinite(v) for v in coords[:3]):
            raise RuntimeError('Arm coordinate feedback unavailable')
        coords = list(coords[:3])
        if pickup and (coords[1] <= 5 or abs(math.degrees(math.atan2(coords[1], coords[0])) - 90) > 15):
            raise ValueError('Pickup arm must face forward near J1=90')
        delta = pickup_delta(axis, direction * distance) if pickup else tuple(
            direction * distance if a == axis else 0 for a in 'XYZ')
        coords = [value + step for value, step in zip(coords, delta)]
        if any(not LIMITS[a][0] + 5 <= coords[i] <= LIMITS[a][1] - 5 for i, a in enumerate('XYZ')) or (pickup and coords[1] <= 5):
            raise ValueError('Arm workspace limit')
        if self.stop_pending or not lease_valid(self.deadline, time.monotonic()):
            raise Stopped('Arm step cancelled or lease expired')
        with arm_deadline(.4):
            self.arm.set_mode(0)
            self.arm.set_coords(coords, 30)
        end = time.monotonic() + 3
        while time.monotonic() < end:
            if self.stop_pending or not lease_valid(self.deadline, time.monotonic()):
                raise Stopped('Arm step cancelled or lease expired')
            with arm_deadline(.4):
                actual = self.arm.get_coords_info()
            if actual and len(actual) >= 3 and all(type(a) in (int, float) and math.isfinite(a) and abs(a-b) <= .5 for a, b in zip(actual[:3], coords)):
                self.read_arm()
                return
            time.sleep(.05)
        raise TimeoutError('Arm step did not reach its measured target')

    def finish(self, packet, result):
        command_id = packet['id']
        self.results[command_id] = dict(id=command_id, **result)
        print(json.dumps(dict(event='command_result', time_s=time.time(), type=packet.get('type'),
                              **({'goal': packet['goal']} if 'goal' in packet else {}),
                              **self.results[command_id]), allow_nan=False), flush=True)
        while len(self.results) > 200:
            old, _ = self.results.popitem(last=False)
            self.requests.pop(old, None)

    def stop_hardware(self, reason):
        if self.teleop_pid is not None:
            with suppress(ProcessLookupError):
                os.kill(self.teleop_pid, signal.SIGTERM)
            self.teleop_pid = None
        was_homing = self.phase == 'homing'
        self.home_cancelled = self.home_cancelled or was_homing
        self.stopping = True
        self.phase, self.status = 'stopping', 'Stop unconfirmed'
        self.base_enabled = False
        self.robot.zero()
        self.robot.nav.client.cancel_all_goals()
        if was_homing:
            self.arm_homed = False
        arm_stopped = False
        confirmed = False
        try:
            if self.arm:
                with arm_deadline(.4):
                    self.arm.set_jog_stop()
                self.angles = wait_arm(self.arm, timeout=2)
                self.arm_stamp = time.monotonic()
                arm_stopped = True
            self.robot.nav.wait_stopped(timeout=2)
            confirmed = arm_stopped and not self.home_cancelled
            self.status = reason + ('; stopped' if confirmed else '; arm stop unconfirmed')
        except (RuntimeError, ValueError, OSError, TimeoutError) as exc:
            self.status = 'Stop unconfirmed: ' + str(exc)
            self.arm_homed = False
        finally:
            self.robot.nav.active = False
            self.goal = None
            self.input = None
            self.deadline = 0
            self.stop_pending = None
            self.phase = 'idle' if confirmed else 'fault'
            self.stopping = False
            while not self.queue.empty():
                with suppress(queue.Empty):
                    packet = self.queue.get_nowait()
                    self.finish(packet, dict(status='cancelled', error='Cancelled before execution'))

    def go(self, pose, precise=False):
        self.travel_guard(pose)
        self.phase, self.status, self.goal = 'navigating', 'Navigating', pose
        self.base_enabled = True
        try:
            self.robot.nav.go_to(pose, position_tolerance=.03 if precise else .10, yaw_tolerance=5 if precise else 10)
        finally:
            self.base_enabled = False
            self.robot.zero()
            self.goal = None

    def reset_map(self):
        pm2 = shutil.which('pm2')
        if not pm2:
            raise RuntimeError('PM2 unavailable; cannot restart mapping')
        self.base_enabled = False
        self.robot.zero()
        self.robot.nav.client.cancel_all_goals()
        self.robot.nav.wait_stopped()
        self.phase, self.status = 'resetting_map', 'Resetting map; waiting for fresh SLAM scans'
        self.localized = False
        marker = self.directory.parent / 'mapping.json'
        session_id = str(uuid.uuid4())
        backup = self.directory.parent / 'map_backups' / session_id
        saved_map = Path(os.environ.get('MYAGV_MAP_FILE', str(Path.home() / 'maps' / 'room.yaml')))
        paths = [self.zones_path, self.stations_path, marker, saved_map, saved_map.with_suffix('.pgm')]
        paths += [self.directory / name for name in ('map.json', 'navigation_map.json', 'global_costmap.json', 'zones.json', 'state.json', 'captures')]
        moved = []
        marker_written = False
        previous = self.robot.source, self.robot.grid, self.robot.zones
        def process(action):
            subprocess.run([pm2, action, 'myagv-localization'], check=True, capture_output=True, timeout=15)
        try:
            process('stop')
            with self.robot.lock:
                backup.mkdir(parents=True)
                for index, path in enumerate(paths):
                    if path.exists():
                        destination = backup / (str(index) + '-' + path.name)
                        path.rename(destination)
                        moved.append((path, destination))
                atomic_json(backup / 'manifest.json', {str(dest.name): str(path) for path, dest in moved})
                atomic_json(marker, dict(session_id=session_id))
                marker_written = True
                blank = {**previous[1], 'map_id': 'slam-' + session_id, 'revision': 0,
                         'cells': [-1] * len(previous[1]['cells'])}
                blank.pop('content_hash', None)
                blank.pop('raw_cells', None)
                self.robot.set_map(previous[0], blank, dict(map_id=blank['map_id'], revision=0, zones=[]))
                atomic_json(self.directory / 'map.json', blank)
                self.robot.amcl_uri = self.robot.clearance = None
            if self.stop_pending:
                raise Stopped('Map reset stopped')
            process('start')
            end = time.monotonic() + 20
            while not self.robot.grid or not self.robot.grid['map_id'].startswith('slam-' + session_id + '-') or self.robot.grid['revision'] == 0:
                if self.stop_pending:
                    raise Stopped('Map reset stopped')
                if time.monotonic() >= end:
                    raise RuntimeError('Fresh SLAM map unavailable; old map restored')
                time.sleep(.1)
        except BaseException:
            with suppress(Exception):
                process('stop')
            with self.robot.lock:
                if marker_written:
                    marker.unlink(missing_ok=True)
                for path, destination in reversed(moved):
                    destination.replace(path)
                self.robot.set_map(*previous)
            with suppress(Exception):
                process('start')
            raise
        finally:
            self.robot.zero()
        return dict(backup=str(backup), mapping_mode=True)

    def review(self, stage):
        self.base_enabled = False
        self.robot.zero()
        self.phase = stage
        self.status = 'Inspect live arm view, adjust and confirm grasp' if stage == 'review_grasp' else 'Grasp unverified; confirm possession before return'
        self.confirmation = None
        end = time.monotonic() + 300
        while time.monotonic() < end:
            if self.confirmation == stage:
                self.capture('arm', persist=False)  # A stale/missing camera cannot satisfy review.
                self.robot.nav.wait_stopped()
                return
            try:
                packet = self.queue.get(timeout=.05)
            except queue.Empty:
                continue
            try:
                if packet['type'] != 'align_step' or stage != 'review_grasp':
                    raise ValueError('Only bounded arm alignment is allowed during grasp review')
                self.arm_step(packet.get('axis'), packet.get('direction'), pickup=True)
                self.finish(packet, dict(status='completed'))
            except Exception as exc:
                self.finish(packet, dict(status='failed', error=str(exc)))
                raise
        raise TimeoutError('Grasp confirmation timed out')

    def execute(self, packet):
        kind = packet['type']
        if kind == 'teleop_acquire':
            self.robot.zero()
            self.robot.nav.client.cancel_all_goals()
            self.robot.nav.wait_stopped()
            if self.arm:
                self.arm.close()
            self.arm, self.angles, self.arm_stamp = None, None, 0
            self.arm_homed, self.home_cancelled, self.localized = False, True, False
            self.owner, self.deadline, self.input, self.base_enabled = None, 0, None, False
            self.teleop_pid = packet['pid']
            self.phase, self.status = 'teleop', 'TELEOP owns hardware; WebUI monitoring only'
            self.arm_error = ''
        elif kind == 'teleop_release':
            self.teleop_pid = None
            self.phase, self.status = 'fault', 'TELEOP finished; verify stopped hardware before resuming WebUI'
        elif kind == 'recover_stop':
            if packet.get('confirmed') is not True:
                raise ValueError('Physically verify stopped hardware before clearing the fault')
            self.connect_arm()
            self.robot.nav.wait_stopped()
            self.angles = wait_arm(self.arm, timeout=3)
            self.arm_stamp = time.monotonic()
            self.arm_homed, self.home_cancelled = False, False
        elif kind == 'initial_pose':
            self.robot.nav.wait_stopped()
            self.localized = False
            self.robot.initial_pose(validate_pose(packet['goal']))
        elif kind == 'confirm_localization':
            self.robot.nav.get_pose()
            if not self.robot.sensors_ready():
                raise RuntimeError('Fresh LiDAR and odometry required')
            self.localized = True
        elif kind == 'connect_arm':
            self.connect_arm()
        elif kind == 'reset_map':
            if packet.get('confirmed') is not True:
                raise ValueError('Confirm reset of map, no-go zones and taught stations')
            return self.reset_map()
        elif kind == 'update_pause':
            self.base_enabled = False
            self.robot.zero()
            self.robot.nav.cancel()
            self.robot.nav.wait_stopped()
            self.robot.commit_map_update()
            return dict(map_revision=self.robot.grid['revision'])
        elif kind == 'obstacle_filter':
            minimum = number(packet.get('minimum_m'), 0, .5, 'Small obstacle size')
            self.robot.nav.wait_stopped()
            self.config['small_obstacle_m'] = minimum
            atomic_json(self.config_path, self.config)
            with self.robot.lock:
                grid = {**self.robot.grid, 'revision': self.robot.grid['revision'] + 1}
                pending = self.robot.pending_map
                self.robot.set_map(self.robot.source, grid, self.robot.zones)
                if pending:
                    self.robot.pending_map = (pending[0], {**pending[1], 'revision': grid['revision'] + 1})
                self.robot.costmaps = {}
                self.robot.published_at = time.monotonic()
            return dict(minimum_m=minimum)
        elif kind in ('home', 'confirm_homed'):
            self.require_arm(False)
            self.robot.nav.wait_stopped()
            if kind == 'home':
                self.phase, self.status, self.arm_homed = 'homing', 'Homing arm', False
                with arm_deadline(60):
                    self.arm.go_zero()
            self.angles = wait_arm(self.arm, timeout=60 if kind == 'home' else 5)
            self.arm_stamp, self.arm_homed = time.monotonic(), True
        elif kind == 'gripper':
            self.robot.nav.wait_stopped()
            self.grip(packet['value'])
        elif kind == 'manual':
            if packet.get('mode') not in ('BASE', 'ARM', 'PICKUP'):
                raise ValueError('Choose BASE, ARM or PICKUP mode')
            self.phase, self.status = 'manual', 'Hold to move; manual base motion is outside no-go enforcement'
            self.manual_args = packet
            self.input = packet.get('input')
        elif kind == 'transport':
            self.robot.nav.wait_stopped()
            self.fold()
        elif kind == 'transport_record':
            self.require_arm()
            self.robot.nav.wait_stopped()
            radius = number(packet.get('clearance_m'), .15, 1, 'Measured folded clearance radius')
            if packet.get('measured') is not True:
                raise ValueError('Explicit measured-clearance confirmation required')
            angles = validate_angles(wait_arm(self.arm, timeout=5))
            self.robot.set_clearance(radius)
            self.robot.nav.approach_settings = (.3, .03, radius)
            self.robot.nav.aligner = None
            self.config = {**self.config, 'transport_angles': angles, 'clearance_m': radius}
            atomic_json(self.config_path, self.config)
        elif kind == 'teach':
            if not self.localized:
                raise ValueError('Confirm localization before teaching')
            self.require_arm()
            name = packet.get('name')
            if not isinstance(name, str) or not 1 <= len(name.strip()) <= 100:
                raise ValueError('Station name must contain 1..100 characters')
            name = name.strip()
            records = load_stations(self.stations_path)
            previous = records.get(name)
            if previous and (packet.get('overwrite') is not True or packet.get('station_digest') != digest_station(previous)):
                raise ValueError('Confirm overwrite of the current station revision')
            self.robot.nav.wait_stopped()
            pose = self.robot.nav.get_pose()
            angles = pickup_angles(wait_arm(self.arm, timeout=5))
            if load_stations(self.stations_path).get(name) != previous:
                raise ValueError('Station changed while recording; retry')
            save_station(name, dict(base=pose, arm_angles_deg=angles), self.stations_path)
            return dict(station=name)
        elif kind == 'capture':
            self.robot.nav.wait_stopped()
            return self.capture(packet['camera_id'])
        elif kind == 'capture_grasp_height':
            self.require_arm()
            self.robot.nav.wait_stopped()
            wait_arm(self.arm, timeout=3)
            with arm_deadline(.4):
                coords = self.arm.get_coords_info()
            if not coords or len(coords) < 3:
                raise RuntimeError('Arm coordinate feedback unavailable')
            return dict(grasp_z_mm=number(coords[2], *LIMITS['Z'], 'Arm Z feedback'))
        elif kind == 'capture_grasp_goal':
            self.require_arm(False)
            self.robot.nav.wait_stopped()
            before = wait_arm(self.arm, timeout=3)
            captures = {camera: self.capture(camera, grasp_goal=True)['capture_id'] for camera in ('front', 'arm')}
            after = wait_arm(self.arm, timeout=3)
            if len(before) != len(after) or any(abs(a-b) > .5 for a, b in zip(before, after)):
                raise RuntimeError('Arm moved while recording grasp goal')
            return dict(captures=captures)
        elif kind == 'fetch_turn_test':
            size = packet.get('size')
            if size not in ('s', 'm', 'l'):
                raise ValueError('Choose S, M or L rotation test')
            config = calibration_config(packet['fetch_settings'])
            self.phase, self.status = 'calibrating', 'Right rotation test: ' + size.upper()
            self.turn_fetch(config['turn_step_deg'] * {'s': .25, 'm': .5, 'l': 1}[size], config)
        elif kind == 'fetch_test':
            self.require_arm()
            if not self.localized or type(packet.get('preview')) is not bool:
                raise ValueError('Confirmed localization and explicit test mode required')
            self.robot.nav.wait_stopped()
            angles = wait_arm(self.arm, timeout=3)
            if not packet['preview']:
                pickup_angles(angles)
            self.calibrate_fetch(packet, packet['fetch_settings'])
        elif kind == 'zones':
            self.robot.nav.wait_stopped()
            zones = packet.get('zones')
            if not isinstance(zones, list) or len(zones) > 100:
                raise ValueError('At most 100 no-go zones are allowed')
            for zone in zones:
                uuid.UUID(zone['id'])
                validate_rectangle(zone['corners'], self.robot.grid)
            from web_backend.simulation import navigation_grid
            derived = navigation_grid(self.robot.grid, zones)
            if not footprint_clear(derived, self.robot.nav.get_pose(), self.config['clearance_m']):
                raise ValueError('No-go zone intersects current robot clearance')
            saved = dict(map_id=self.robot.grid['map_id'], revision=self.robot.zones['revision'] + 1, zones=zones)
            atomic_json(self.zones_path, saved)
            self.robot.apply_zones(saved)
            return dict(zones=saved)
        elif kind == 'navigate':
            pose = validate_pose(packet['goal'])
            pose = dict(x_m=pose['x_m'], y_m=pose['y_m'], yaw_deg=math.degrees(pose['yaw_rad']))
            self.go(pose)
        elif kind == 'fetch':
            self.fetch(packet)
        elif kind == 'approach_item':
            return self.approach_item(packet)
        elif kind == 'update_plan':
            self.map_update = {**self.plan_update(packet), 'active': False, 'state': 'planned',
                               'map_id': self.robot.grid['map_id'], 'completed': 0, 'round': 0, 'captures': 0, 'remaining_s': 0}
            return dict(goals=self.map_update['goals'])
        elif kind == 'map_update':
            return self.run_update(packet)
        elif kind == 'survey':
            goals = packet.get('goals')
            if not isinstance(goals, list) or not 1 <= len(goals) <= 20:
                raise ValueError('Survey requires 1..20 viewing poses')
            poses = [validate_pose(p) for p in goals]
            for pose in poses:
                self.travel_guard(pose)
            self.capture('front', persist=False)
            self.fold()
            captures = []
            for pose in poses:
                self.go(dict(x_m=pose['x_m'], y_m=pose['y_m'], yaw_deg=math.degrees(pose['yaw_rad'])))
                captures.append(self.capture('front'))
            return dict(captures=captures)
        else:
            raise ValueError('Unsupported hardware command')
        return {}

    def plan_update(self, packet):
        goals = packet.get('goals')
        if not isinstance(goals, list) or not 1 <= len(goals) <= 20:
            raise ValueError('Select 1..20 viewing poses')
        poses = [validate_pose(p) for p in goals]
        return dict(goals=poses, capped=False)

    def wait_update(self):
        update = self.map_update
        if not update or not update['active']:
            return
        if self.stop_pending or not lease_valid(self.deadline, time.monotonic()):
            raise Stopped('Map update control expired')
        if time.monotonic() >= update['deadline']:
            update['pause_requested'], update['pause_reason'] = True, 'Update time limit reached; map retained'
        if not update['pause_requested']:
            return
        self.base_enabled = False
        self.robot.zero()
        self.robot.nav.cancel()
        self.robot.nav.wait_stopped()
        self.robot.commit_map_update()
        self.phase, self.status = 'update_paused', update['pause_reason']
        update['state'] = 'paused'
        while update['pause_requested']:
            if self.stop_pending or not lease_valid(self.deadline, time.monotonic()) or self.robot.ros.is_shutdown():
                raise Stopped('Map update cancelled while paused')
            if not self.robot.grid or self.robot.grid['map_id'] != update['map_id'] or self.robot.zones['revision'] != update['zone_revision']:
                raise Stopped('Update map or no-go revision changed')
            time.sleep(.05)
        self.phase, self.status, update['state'] = 'updating_map', 'Updating map observations', 'updating'

    def run_update(self, packet):
        limit_s = number(packet.get('limit_s', 600), 60, 3600, 'Update window seconds')
        started = time.monotonic()
        plan = self.plan_update(packet)
        self.map_update = update = {**plan, 'map_id': self.robot.grid['map_id'],
            'zone_revision': self.robot.zones['revision'], 'map_revision': self.robot.grid.get('revision'), 'active': True, 'state': 'updating',
            'deadline': started + limit_s, 'limit_s': limit_s, 'pause_requested': False,
            'pause_reason': '', 'storage_error': '', 'completed': 0, 'round': 1, 'captures': 0}
        try:
            while True:
                self.wait_update()
                pose = update['goals'][update['completed']]
                try:
                    self.execute(dict(type='navigate', goal=pose))
                except UpdatePaused:
                    self.wait_update()
                    continue
                self.wait_update()
                self.phase, self.status = 'updating_map', 'Refreshing map observation'
                # Bounded handoff prevents the API/storage backlog from filling the disk.
                if len(list((self.directory / 'captures').glob('*.json'))) >= 20:
                    update['storage_error'] = 'Photo import backlog; repair storage before restarting updates'
                    update['pause_requested'], update['pause_reason'] = True, update['storage_error']
                    continue
                key = '%.2f:%.2f:%.2f' % (pose['x_m'], pose['y_m'], pose['yaw_rad'])
                self.capture('front', update_key=key)
                update['captures'] += 1
                update['completed'] += 1
                if update['completed'] == len(update['goals']):
                    update['pause_requested'], update['pause_reason'] = True, 'Go queue completed; map updated'
                    self.wait_update()
                    update['completed'] = 0
                    update['round'] += 1
        finally:
            update['active'], update['state'] = False, 'interrupted'

    def approach_item(self, packet):
        config = location_config(packet['location_config'])
        item, estimate = packet['vision_item'], packet['estimate']
        map_id, revision = self.robot.grid['map_id'], self.robot.zones['revision']
        request_path = self.directory / 'fetch_vision_request.json'
        response_path = self.directory / 'fetch_vision_response.json'
        history = []

        def current():
            if self.stop_pending or not lease_valid(self.deadline, time.monotonic()):
                raise Stopped('Item approach cancelled or lease expired')
            if self.robot.grid['map_id'] != map_id or self.robot.zones['revision'] != revision:
                raise ValueError('Map or no-go zones changed during item approach')
            self.travel_guard(self.robot.nav.get_pose())

        try:
            for round_index in range(16):
                current()
                self.phase, self.status = 'approaching_item', 'Item approach round %d/16: stopped camera check' % (round_index + 1)
                self.base_enabled = False
                self.robot.zero()
                self.robot.nav.wait_stopped()
                capture = self.capture('front', vision=True, update_key='approach:%s:%d' % (self.task_id, round_index))
                pose = capture['base_pose']
                request_id = str(uuid.uuid4())
                atomic_json(request_path, dict(id=request_id, mode='approach', boot_id=self.boot_id, task_id=self.task_id,
                    stop_epoch=self.stop_epoch, map_id=map_id, zone_revision=revision, expires_at_s=time.time() + 20,
                    item=item, estimate=estimate, reference_photo_id=packet['reference_photo_id'], base_pose=pose, capture_id=capture['capture_id'],
                    images=dict(front=capture['image']), history=history, round=round_index + 1,
                    clearance_m=self.config['clearance_m'], catalog_signature=packet['catalog_signature']))
                end = time.monotonic() + 20
                while True:
                    current()
                    try:
                        response = json.loads(response_path.read_text())
                    except (OSError, ValueError):
                        response = {}
                    if response.get('id') == request_id:
                        if response.get('error'):
                            raise RuntimeError(response['error'])
                        break
                    if time.monotonic() >= end:
                        raise RuntimeError('Item approach vision timed out')
                    time.sleep(.05)
                current()
                feedback = self.robot.nav.get_pose()
                feedback = dict(feedback, yaw_rad=math.radians(feedback['yaw_deg']))
                yaw_error = math.atan2(math.sin(feedback['yaw_rad'] - pose['yaw_rad']), math.cos(feedback['yaw_rad'] - pose['yaw_rad']))
                if math.hypot(feedback['x_m'] - pose['x_m'], feedback['y_m'] - pose['y_m']) > .03 or abs(yaw_error) > math.radians(5):
                    raise RuntimeError('Base pose changed after approach photo')
                costmap = self.robot.costmaps['global'][3]
                goal, corrected = approach_step(response['result'], dict(base_pose=pose), self.robot.grid,
                    self.robot.zones['zones'], costmap, item, self.config['clearance_m'], config)
                estimate = dict(corrected, base_pose=pose)
                if goal is None:
                    return dict(rounds=round_index + 1, estimate=corrected)
                if round_index == 15:
                    raise RuntimeError('Item approach did not converge after 16 rounds')
                self.go(dict(x_m=goal['x_m'], y_m=goal['y_m'], yaw_deg=math.degrees(goal['yaw_rad'])), precise=True)
                current()
                self.robot.nav.wait_stopped()
                after = self.robot.nav.get_pose()
                after = dict(after, yaw_rad=math.radians(after['yaw_deg']))
                history.append(dict(images=dict(front=capture['image']), base_pose=pose, after_pose=after,
                                    commanded={k: response['result'][k] for k in ('forward_m', 'turn_deg')}, estimate=corrected))
        finally:
            self.base_enabled = False
            self.robot.zero()
            request_path.unlink(missing_ok=True)
            response_path.unlink(missing_ok=True)

    def turn_fetch(self, degrees, config=None):
        config = calibration_config(config)
        number(degrees, -2, 2, 'Fetch base turn')
        self.robot.nav.wait_stopped()
        start = self.robot.nav.get_odom_pose()
        map_id, revision = self.robot.grid['map_id'], self.robot.zones['revision']
        target = math.radians(start['yaw_deg'] - degrees)
        speed = config['turn_speed_rad_s']
        end = time.monotonic() + config['turn_duration_scale'] * abs(math.radians(degrees)) / speed
        try:
            while True:
                pose = self.robot.nav.get_odom_pose()
                if self.robot.grid['map_id'] != map_id or self.robot.zones['revision'] != revision:
                    raise Stopped('Map or no-go zones changed during fetch rotation')
                self.travel_guard(self.robot.nav.get_pose())
                error = math.atan2(math.sin(target - math.radians(pose['yaw_deg'])),
                                   math.cos(target - math.radians(pose['yaw_deg'])))
                if abs(error) <= math.radians(.15) or error * degrees >= 0:
                    return
                if time.monotonic() >= end:
                    self.status = 'Rotation pulse finished; rechecking camera alignment'
                    return
                self.base_enabled = True
                self.robot.velocity = (0, 0, -math.copysign(speed, degrees))
                self.robot.velocity_at = time.monotonic()
                time.sleep(.05)
        finally:
            self.base_enabled = False
            self.robot.zero()
            self.robot.nav.wait_stopped()

    def calibrate_fetch(self, packet, config):
        config = calibration_config(config)
        self.phase, self.status = 'calibrating', 'Aligning pickup with front and arm cameras'
        self.base_enabled = False
        self.robot.zero()
        request_path = self.directory / 'fetch_vision_request.json'
        response_path = self.directory / 'fetch_vision_response.json'
        travel, turned = 0, 0
        history = []
        map_id, revision = self.robot.grid['map_id'], self.robot.zones['revision']

        def current():
            if self.stop_pending or not lease_valid(self.deadline, time.monotonic()):
                raise Stopped('Fetch alignment cancelled or lease expired')
            if self.robot.grid['map_id'] != map_id or self.robot.zones['revision'] != revision:
                raise ValueError('Map or no-go zones changed during fetch alignment')
            if not self.robot.sensors_ready():
                raise RuntimeError('Fresh sensors required during fetch alignment')

        try:
            for round_index in range(33):
                current()
                self.status = 'Round %d: capturing front and arm cameras' % (round_index + 1)
                self.robot.nav.wait_stopped()
                images = {camera: self.capture(camera, vision=True)['image'] for camera in ('front', 'arm')}
                request_id = str(uuid.uuid4())
                atomic_json(request_path, dict(id=request_id, boot_id=self.boot_id, task_id=self.task_id,
                    stop_epoch=self.stop_epoch, map_id=map_id, expires_at_s=time.time() + FETCH_VISION_TIMEOUT_S + 5,
                    item=packet['vision_item'], images=images, history=history,
                    round=round_index + 1, preview=packet.get('preview', False), limits=config,
                    prompt=packet.get('fetch_settings', {}).get('prompt')))
                self.status = 'Round %d: waiting for LLM' % (round_index + 1)
                end = time.monotonic() + FETCH_VISION_TIMEOUT_S + 5
                while True:
                    current()
                    try:
                        response = json.loads(response_path.read_text())
                    except (OSError, ValueError):
                        response = {}
                    if response.get('id') == request_id:
                        if response.get('error'):
                            raise RuntimeError(response['error'])
                        moves = alignment_command(response.get('result'), config)
                        break
                    if time.monotonic() >= end:
                        raise TimeoutError('Fetch vision calibration timed out')
                    time.sleep(.05)
                if packet.get('preview'):
                    return
                if response.get('aligned_perfectly') is True and not any(moves.values()):
                    return
                if not any(moves.values()):
                    raise ValueError('Both cameras must explicitly report aligned_perfectly')
                if round_index == 32:
                    raise RuntimeError('Fetch cameras did not converge after 32 corrections')
                if moves['turn_deg']:
                    turned += abs(moves['turn_deg'])
                    if turned > 45:
                        raise ValueError('Fetch rotation budget exhausted')
                    current()
                    self.status = 'Aligning left/right with base rotation'
                    self.turn_fetch(moves['turn_deg'], config)
                    moves['X'] = 0  # Rotation changes the arm view; recapture before advancing.
                travel += abs(moves['X'])
                if travel > config['max_total_mm']:
                    raise ValueError('Fetch alignment travel budget exhausted')
                for axis, delta in [('X', moves['X'])]:
                    remaining = abs(delta)
                    while remaining >= .001:
                        current()
                        step = min(1, remaining)
                        self.arm_step(axis, 1 if delta > 0 else -1, pickup=True, distance=step)
                        remaining -= step
                history = (history + [dict(images=images, commanded=moves)])[-2:]
        finally:
            self.base_enabled = False
            self.robot.zero()
            request_path.unlink(missing_ok=True)
            response_path.unlink(missing_ok=True)

    def fetch(self, packet):
        records = load_stations(self.stations_path)
        station = records.get(packet.get('station'))
        if station is None or digest_station(station) != packet.get('station_digest'):
            raise ValueError('Station missing or changed; reconfirm item association')
        target = pickup_angles(station['arm_angles_deg'])
        calibration = packet.get('fetch_settings', self.config.get('fetch_calibration'))
        if packet.get('vision_item'):
            calibration = calibration_config(calibration)
        self.travel_guard(station['base'])
        self.require_arm()
        self.robot.nav.wait_stopped()
        start_arm, start_base = wait_arm(self.arm, timeout=5), self.robot.nav.get_pose()
        transport = self.config['transport_angles']
        if transport is None:
            raise ValueError('Record transport pose first')
        if not len(start_arm) == len(target) == len(transport):
            raise ValueError('Arm joint counts differ')
        self.travel_guard(start_base)
        self.capture('arm', persist=False)
        self.fold()
        self.phase, self.status, self.base_enabled = 'approach', 'Approaching taught pickup pose', True
        try:
            self.robot.nav.go_to_pickup(station['base'])
        finally:
            self.base_enabled = False
            self.robot.zero()
        self.grip(100)
        self.move_arm(target)
        if packet.get('vision_item'):
            self.calibrate_fetch(packet, calibration)
        self.review('review_grasp')
        self.grip(0)
        self.fold()
        self.review('verify_grasp')
        self.go(start_base)
        self.phase, self.status = 'placing', 'Restoring startup arm pose'
        self.move_arm(start_arm)
        self.grip(100)

    def manual_tick(self):
        if not isinstance(self.input, dict):
            self.request_stop('Manual input released')
            return
        mode, key = self.manual_args['mode'], self.input.get('key')
        if self.input.get('mode') != mode:
            self.request_stop('Manual mode changed')
            return
        if mode in ('BASE', 'PICKUP') and key in ('forward', 'back', 'left', 'right', 'ccw', 'cw'):
            values = manual_vector(mode, key, self.manual_args.get('speed', .05), self.manual_args.get('turn', .15))
            self.base_enabled = True
            self.robot.velocity, self.robot.velocity_at = values, time.monotonic()
        else:
            self.base_enabled = False
            self.robot.zero()
            if mode not in ('ARM', 'PICKUP') or key not in ('X+', 'X-', 'Y+', 'Y-', 'Z+', 'Z-'):
                raise ValueError('Invalid arm input')
            self.arm_step(key[0], 1 if key[1] == '+' else -1, pickup=mode == 'PICKUP')

    def snapshot(self):
        ready = self.phase not in ('teleop', 'handoff') and self.robot.driver_watchdog and self.robot.exclusive and self.robot.sensors_ready()
        update = dict(self.map_update) if self.map_update else None
        if update and update.get('active'):
            update['remaining_s'] = max(0, math.ceil(update['deadline'] - time.monotonic()))
        return dict(boot_id=self.boot_id, stop_epoch=self.stop_epoch, stamp_s=time.time(), phase=self.phase, status=self.status, map_update=update,
                    item_approach_available=True,
                    task_id=self.task_id, arm_available=self.arm is not None, arm_homed=self.arm_homed,
                    arm_angles=self.angles if fresh(self.arm_stamp, time.monotonic(), 2) else None,
                    arm_error=self.arm_error, transport_angles=self.config['transport_angles'],
                    clearance_m=self.config['clearance_m'], localized=self.localized,
                    small_obstacle_m=self.config.get('small_obstacle_m', .15),
                    mapping_mode=(self.directory.parent / 'mapping.json').exists(),
                    driver_watchdog=self.robot.driver_watchdog, exclusive=self.robot.exclusive,
                    sensors_ready=self.robot.sensors_ready(), motion_available=ready,
                    navigation_ready=ready and self.localized and self.robot.zones_ready() and self.robot.clearance_ready(self.config['clearance_m']),
                    costmap_feedback=self.robot.costmap_received, layers_configured=self.robot.layers_ok,
                    planner_footprint_ready=self.robot.clearance_ready(self.config['clearance_m']),
                    zones_ready=self.robot.zones_ready(), zone_revision=self.robot.zones['revision'] if self.robot.zones else -1,
                    goal=self.goal, moving=self.base_enabled, stop_confirmed=self.phase == 'idle' and self.status.endswith('; stopped'))

    def run(self):
        last_poll = 0
        try:
            self.connect_arm()
        except Exception as exc:
            self.arm_error = str(exc)
            if self.arm:
                with suppress(Exception):
                    self.arm.close()
            self.arm = None
        while not self.robot.ros.is_shutdown():
            packet = None
            try:
                if self.stop_pending:
                    self.stop_hardware(self.stop_pending)
                if self.phase == 'teleop' and self.teleop_pid is not None:
                    try:
                        os.kill(self.teleop_pid, 0)
                    except ProcessLookupError:
                        self.teleop_pid = None
                        self.stop_hardware('TELEOP disconnected; verify stopped hardware')
                if self.phase == 'manual':
                    self.executing = True
                    self.manual_tick()
                else:
                    try:
                        packet = self.queue.get(timeout=.05)
                    except queue.Empty:
                        packet = None
                    if packet:
                        self.executing = True
                        handoff = packet.get('op') == 'teleop'
                        if (not handoff and not lease_valid(self.deadline, time.monotonic())) or self.stop_pending or packet.get('stop_epoch') != self.stop_epoch:
                            raise Stopped('Expired queued command')
                        self.task_id = packet['id']
                        self.phase, self.status = 'handoff' if handoff else 'working', packet['type']
                        result = self.execute(packet)
                        self.finish(packet, dict(status='completed', **result))
                        if self.phase not in ('manual', 'teleop', 'fault'):
                            self.phase, self.status = 'idle', 'Completed: ' + packet['type']
                    elif self.arm and time.monotonic() - last_poll > 1:
                        last_poll = time.monotonic()
                        self.read_arm()
                time.sleep(.03)
            except (Stopped, Exception) as exc:
                if packet:
                    self.finish(packet, dict(status='cancelled' if isinstance(exc, Stopped) else 'failed', error=str(exc)))
                self.stop_hardware(str(exc))
                if not packet and self.arm and not isinstance(exc, Stopped):
                    self.arm_error, self.arm_homed = str(exc), False
                    with suppress(Exception):
                        self.arm.close()
                    self.arm = None
            finally:
                self.executing = False
        self.stop_hardware('Controller shutdown')
        self.server.shutdown()
        if self.arm:
            self.arm.close()
