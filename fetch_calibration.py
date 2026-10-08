"""Dual-camera pickup calibration; logical X is forward, Y is right, in mm."""
import base64
import math


def calibration_config(value):
    if value is None:
        value = {}
    if not isinstance(value, dict):
        raise ValueError('Invalid fetch_calibration configuration')
    # ponytail: conservative image-width gains; tune against measured mm travel on the robot.
    result = dict(front_target_x=.5, arm_target_x=.5, tolerance=.025, max_step_mm=2,
                  max_total_mm=20, front_mm_per_width=20, arm_mm_per_width=20)
    result.update(value)
    for key in ('front_target_x', 'arm_target_x', 'tolerance', 'max_step_mm',
                'max_total_mm', 'front_mm_per_width', 'arm_mm_per_width'):
        number = result.get(key)
        if type(number) not in (int, float) or not math.isfinite(number):
            raise ValueError('Invalid fetch calibration: ' + key)
    if not all(0 <= result[k] <= 1 for k in ('front_target_x', 'arm_target_x')):
        raise ValueError('Camera target X must be normalized to 0..1')
    if not .005 <= result['tolerance'] <= .1 or not 0 < result['max_step_mm'] <= 2:
        raise ValueError('Invalid fetch alignment tolerance or step limit')
    if not result['max_step_mm'] <= result['max_total_mm'] <= 30:
        raise ValueError('Invalid fetch alignment travel budget')
    if not all(0 < abs(result[k]) <= 1000 for k in ('front_mm_per_width', 'arm_mm_per_width')):
        raise ValueError('Measured signed camera gains must be nonzero')
    return result


def alignment_command(result, config):
    config = calibration_config(config)
    if not isinstance(result, dict) or set(result) != {'front_x', 'arm_x'}:
        raise ValueError('Invalid model alignment JSON')
    moves = {}
    for camera, axis in (('arm', 'X'), ('front', 'Y')):
        x = result[camera + '_x']
        if x is None:
            raise ValueError('Pickup target missing or ambiguous in ' + camera + ' camera')
        if type(x) not in (int, float) or not math.isfinite(x) or not 0 <= x <= 1:
            raise ValueError('Invalid model camera X')
        error = x - config[camera + '_target_x']
        delta = error * config[camera + '_mm_per_width']
        limit = config['max_step_mm']
        moves[axis] = 0 if abs(error) <= config['tolerance'] else max(-limit, min(limit, delta))
    return moves


def pickup_delta(axis, delta):
    """J1=90: logical forward/right maps to native +Y/-X."""
    if axis not in ('X', 'Y', 'Z'):
        raise ValueError('Invalid pickup axis')
    return {'X': (0, delta, 0), 'Y': (-delta, 0, 0), 'Z': (0, 0, delta)}[axis]


def vision_image(frame, cv2):
    """Reuse bridge OpenCV; preserve the camera's mounted orientation."""
    height, width = frame.shape[:2]
    scale = min(1, 480 / max(width, height))
    if scale < 1:
        frame = cv2.resize(frame, (max(1, round(width * scale)), max(1, round(height * scale))),
                           interpolation=cv2.INTER_AREA)
    ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
    if not ok or len(encoded) > 160 * 1024:
        raise RuntimeError('Fetch camera compression failed')
    return 'data:image/jpeg;base64,' + base64.b64encode(encoded.tobytes()).decode('ascii')
