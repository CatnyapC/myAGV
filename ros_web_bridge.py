"""ROS/serial owner. Web API exchanges snapshots and bounded Unix-socket commands."""
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import threading
import time
import uuid

from navigation import wait_arm
from robot_control import Control
from robot_ros import RobotROS, grid_dict
from robot_safety import fresh
from web_backend.storage import atomic_bytes, atomic_json


def main():
    import cv2
    import rospy
    import tf2_ros
    from nav_msgs.msg import OccupancyGrid

    directory = Path(os.environ.get('MYAGV_HARDWARE_DIR', 'web_runtime/ros'))
    directory.mkdir(parents=True, exist_ok=True)
    rospy.init_node('myagv_web_control', disable_signals=True)
    robot = RobotROS(directory)
    frames, frame_lock = {}, threading.Lock()

    def pose_at(stamp):
        transform = robot.nav.tf.lookup_transform('map', 'base_footprint', stamp, rospy.Duration(.2))
        p, q = transform.transform.translation, transform.transform.rotation
        return dict(x_m=p.x, y_m=p.y, yaw_rad=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

    def capture(camera_id, persist=True):
        if camera_id not in ('front', 'arm') or not robot.grid or not control.localized:
            raise ValueError('Camera and confirmed localization required')
        robot.nav.wait_stopped()
        before = wait_arm(control.arm, timeout=3) if camera_id == 'arm' and control.arm else None
        if camera_id == 'arm' and before is None:
            raise RuntimeError('Arm feedback required for acquisition')
        started, end = time.time(), time.monotonic() + 2
        frame = None
        while time.monotonic() < end:
            with frame_lock:
                current = frames.get(camera_id)
                if current and current[1]['captured_at_s'] >= started:
                    frame, metadata = current
                    break
            time.sleep(.02)
        if frame is None or not fresh(metadata['captured_at_s'], time.time(), 1):
            raise RuntimeError('Fresh camera frame with acquisition-time transform unavailable')
        if not robot.stopped() or metadata['map_id'] != robot.grid['map_id']:
            raise RuntimeError('Robot or map changed during acquisition')
        if before is not None:
            after = control.read_arm()
            if len(before) != len(after) or any(abs(a-b) > .5 for a, b in zip(before, after)):
                raise RuntimeError('Arm moved during acquisition')
            metadata = dict(metadata, arm_angles_deg=after)
        if not persist:
            return {}
        ok, encoded = cv2.imencode('.png', frame)
        if not ok:
            raise RuntimeError('Camera encoding failed')
        capture_id = str(uuid.uuid4())
        metadata = dict(metadata, capture_id=capture_id)
        atomic_bytes(directory / 'captures' / (capture_id + '.png'), encoded.tobytes())
        atomic_json(directory / 'captures' / (capture_id + '.json'), metadata)
        return dict(capture_id=capture_id)

    control = Control(robot, directory, capture)
    robot.nav.approach_settings = (.3, .03, control.config['clearance_m'])

    def shutdown(signum, _frame):
        rospy.signal_shutdown('signal %s' % signum)
        control.request_stop('Service shutdown')

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    def receive_map(message):
        try:
            grid = grid_dict(message)
            if grid['frame'] != 'map' or not 0 < grid['width'] * grid['height'] <= 2_000_000 or len(grid['cells']) != grid['width'] * grid['height'] or not math.isfinite(grid['resolution_m']) or grid['resolution_m'] <= 0:
                raise ValueError('Invalid map geometry/frame')
            grid['cells'] = [-1 if c < 0 else 100 if c >= 50 else 0 for c in grid['cells']]
            digest = hashlib.sha256(json.dumps(grid, sort_keys=True, allow_nan=False).encode()).hexdigest()
            grid.update(map_id='ros-' + digest[:16], revision=1)
            if robot.grid and robot.grid['map_id'] == grid['map_id']:
                return
            control.request_stop('Map changed')
            control.localized = False
            zones = dict(map_id=grid['map_id'], revision=0, zones=[])
            if control.zones_path.exists():
                zones = json.loads(control.zones_path.read_text())
                if zones['map_id'] != grid['map_id']:
                    raise ValueError('Saved no-go zones belong to another map; archive/review the saved file first')
            robot.set_map(message, grid, zones)
            atomic_json(directory / 'map.json', grid)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            robot.grid = None
            control.request_stop(str(exc))
            rospy.logerr('Map rejected: %s', exc)

    rospy.Subscriber('/map', OccupancyGrid, receive_map, queue_size=1)

    def camera(camera_id, device):
        while not rospy.is_shutdown():
            stream = cv2.VideoCapture(device, cv2.CAP_V4L2)
            stream.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            stream.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            last = 0
            try:
                while not rospy.is_shutdown() and stream.isOpened():
                    started = time.monotonic()
                    ok, frame = stream.read()
                    stamp, ros_stamp = time.time(), rospy.Time.now()
                    if not ok:
                        break
                    if stamp - last < .25:
                        continue  # Drain the device continuously; never label buffered frames as new.
                    last = stamp
                    if time.monotonic() - started > .5:
                        continue
                    ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
                    if ok:
                        atomic_bytes(directory / (camera_id + '.jpg'), encoded.tobytes())
                        atomic_json(directory / (camera_id + '.json'), dict(stamp_s=stamp))
                    grid = robot.grid
                    if not grid or not robot.stopped():
                        continue
                    try:
                        pose = pose_at(ros_stamp)
                    except tf2_ros.TransformException:
                        continue
                    metadata = dict(kind='observation', source='ros', captured_at_s=stamp,
                                    timestamp_basis='host_frame_receipt', base_pose=pose, camera_id=camera_id,
                                    map_id=grid['map_id'], map_revision=grid['revision'], frame='map')
                    with frame_lock:
                        frames[camera_id] = (frame.copy(), metadata)
            except (cv2.error, OSError) as exc:
                rospy.logwarn_throttle(10, '%s camera: %s', camera_id, exc)
            finally:
                stream.release()
            time.sleep(1)

    def monitor():
        while not rospy.is_shutdown():
            try:
                robot.check_publishers()
                if robot.clearance is None:
                    robot.set_clearance(control.config['clearance_m'])
            except Exception as exc:
                robot.layers_ok = False
                rospy.logwarn_throttle(10, 'ROS planner check pending: %s', exc)
            time.sleep(1)

    def snapshots():
        while not rospy.is_shutdown():
            try:
                pose, stamp = None, None
                try:
                    p = robot.nav.get_pose(timeout=.05)
                    pose = dict(x_m=p['x_m'], y_m=p['y_m'], yaw_rad=math.radians(p['yaw_deg']))
                    stamp = time.time()
                except (RuntimeError, TimeoutError, ValueError):
                    pass
                grid = robot.grid or {}
                atomic_json(directory / 'state.json', dict(stamp_s=time.time(), pose=pose, pose_stamp_s=stamp,
                            frame=grid.get('frame'), map_id=grid.get('map_id'),
                            path=robot.path if fresh(robot.path_at, time.monotonic(), 2) and control.base_enabled else []))
                atomic_json(directory / 'control.json', control.snapshot())
                if robot.zones:
                    atomic_json(directory / 'zones.json', robot.zones)
            except Exception as exc:
                rospy.logerr_throttle(5, 'Snapshot error: %s', exc)
            time.sleep(.15)

    for camera_id in ('front', 'arm'):
        device = os.environ.get('MYAGV_%s_DEVICE' % camera_id.upper())
        if device:
            threading.Thread(target=camera, args=(camera_id, device), daemon=True).start()
    threading.Thread(target=monitor, daemon=True).start()
    threading.Thread(target=snapshots, daemon=True).start()
    control.run()


if __name__ == '__main__':
    main()
