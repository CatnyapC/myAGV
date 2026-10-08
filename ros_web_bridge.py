"""Read-only ROS1 and camera snapshots for the WebUI; run with system Python."""
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time

from web_backend.storage import atomic_bytes, atomic_json


def main():
    import cv2
    import rospy
    import tf2_ros
    from nav_msgs.msg import OccupancyGrid

    directory = Path(os.environ.get('MYAGV_HARDWARE_DIR', 'web_runtime/ros'))
    rospy.init_node('myagv_web_reader', disable_signals=False)
    transforms = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(transforms)
    map_info = dict(frame='map', map_id=None)

    def receive_map(message):
        width, height, resolution = message.info.width, message.info.height, message.info.resolution
        if not 0 < width * height <= 2_000_000 or not math.isfinite(resolution) or resolution <= 0:
            rospy.logerr_throttle(10, 'Web map exceeds 2 million cells or has invalid resolution')
            return
        pose = message.info.origin
        q = pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        grid = dict(frame=message.header.frame_id, width=width, height=height, resolution_m=resolution,
                    origin=dict(x_m=pose.position.x, y_m=pose.position.y, yaw_rad=yaw),
                    cells=[-1 if cell < 0 else 100 if cell >= 50 else 0 for cell in message.data])
        digest = hashlib.sha256(json.dumps(grid, sort_keys=True, allow_nan=False).encode()).hexdigest()
        grid.update(map_id='ros-' + digest[:16], revision=1)
        atomic_json(directory / 'map.json', grid)
        map_info.update(frame=grid['frame'], map_id=grid['map_id'])

    rospy.Subscriber('/map', OccupancyGrid, receive_map, queue_size=1)

    def camera(camera_id, device):
        while not rospy.is_shutdown():
            capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            try:
                while not rospy.is_shutdown() and capture.isOpened():
                    ok, frame = capture.read()
                    stamp = time.time()
                    if not ok:
                        break
                    ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
                    if ok:
                        atomic_bytes(directory / f'{camera_id}.jpg', encoded.tobytes())
                        atomic_json(directory / f'{camera_id}.json', dict(stamp_s=stamp))
                    time.sleep(.25)
            except (cv2.error, OSError) as exc:
                rospy.logwarn_throttle(10, '%s camera error: %s', camera_id, exc)
            finally:
                capture.release()
            rospy.logwarn_throttle(10, '%s camera unavailable on %s', camera_id, device)
            time.sleep(3)

    for camera_id in ('front', 'arm'):
        device = os.environ.get(f'MYAGV_{camera_id.upper()}_DEVICE')
        if device:
            threading.Thread(target=camera, args=(camera_id, device), daemon=True).start()

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        pose, pose_stamp = None, None
        try:
            transform = transforms.lookup_transform(map_info['frame'], 'base_footprint', rospy.Time(0))
            age = (rospy.Time.now() - transform.header.stamp).to_sec()
            if -1 <= age <= 2:
                p, q = transform.transform.translation, transform.transform.rotation
                pose = dict(x_m=p.x, y_m=p.y, yaw_rad=math.atan2(2 * (q.w * q.z + q.x * q.y),
                                                               1 - 2 * (q.y * q.y + q.z * q.z)))
                pose_stamp = time.time() - max(0, age)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        atomic_json(directory / 'state.json', dict(stamp_s=time.time(), pose=pose, pose_stamp_s=pose_stamp,
                                                   **map_info))
        rate.sleep()


if __name__ == '__main__':
    main()
