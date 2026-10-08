"""Dual-camera pickup calibration; arm X is forward in mm; base turn is clockwise in degrees."""
import base64
import math

FETCH_VISION_TIMEOUT_S = 30


def calibration_config(value):
    if value is None:
        value = {}
    if not isinstance(value, dict):
        raise ValueError('Invalid fetch_calibration configuration')
    result = {key: value.get(key, default) for key, default in (
        ('max_step_mm', 10), ('turn_speed_rad_s', .015),
        ('turn_duration_scale', .8), ('turn_step_deg', 2))}
    for key in result:
        number = result.get(key)
        if type(number) not in (int, float) or not math.isfinite(number):
            raise ValueError('Invalid fetch calibration: ' + key)
    if not .1 <= result['max_step_mm'] <= 10:
        raise ValueError('Invalid fetch alignment step limit')
    for key, low, high in (('turn_speed_rad_s', .005, .1), ('turn_duration_scale', .1, 1.5), ('turn_step_deg', .4, 2)):
        if not low <= result[key] <= high:
            raise ValueError('Invalid fetch calibration: ' + key)
    for size, fraction in (('s', .25), ('m', .5), ('l', 1)):
        duration = result['turn_duration_scale'] * math.radians(result['turn_step_deg'] * fraction) / result['turn_speed_rad_s']
        for suffix, default, low, high in (('speed_rad_s', result['turn_speed_rad_s'], .005, .1),
                                           ('duration_s', max(.05, duration), .05, 15)):
            key = 'turn_' + size + '_' + suffix
            number = value.get(key, default)
            if type(number) not in (int, float) or not math.isfinite(number) or not low <= number <= high:
                raise ValueError('Invalid fetch calibration: ' + key)
            result[key] = number
    return result


def alignment_command(result, config):
    config = calibration_config(config)
    if not isinstance(result, dict) or set(result) not in ({'x_mm', 'turn_deg'}, {'x_mm', 'y_mm', 'turn_deg'}):
        raise ValueError('Invalid model alignment JSON')
    moves = {}
    for field, axis in [('x_mm', 'X'), ('turn_deg', 'turn_deg')] + ([('y_mm', 'Y')] if 'y_mm' in result else []):
        delta = result[field]
        if delta is None:
            raise ValueError('Pickup target missing or ambiguous')
        if type(delta) not in (int, float) or not math.isfinite(delta) or abs(delta) > (2 if axis == 'turn_deg' else 10):
            raise ValueError('Invalid model arm adjustment')
        if 0 < abs(delta) < .1:
            raise ValueError('Model arm adjustment is below measurable step size')
        limit = 2 if axis == 'turn_deg' else config['max_step_mm']
        moves[axis] = max(-limit, min(limit, delta))
    return moves


def qualitative_alignment(result, config):
    config = calibration_config(config)
    if not isinstance(result, dict) or set(result) != {'arm', 'front'}:
        raise ValueError('Invalid visual alignment labels')
    steps = {'left_large': -2, 'left_medium': -1, 'left_small': -.5,
             'aligned_perfectly': 0, 'right_small': .5, 'right_medium': 1, 'right_large': 2}
    moves = {}
    for camera, field in (('arm', 'x_mm'), ('front', 'turn_deg')):
        label = result[camera]
        if label == 'unknown':
            raise ValueError('Pickup target missing or ambiguous: ' + camera)
        if not isinstance(label, str) or label not in steps:
            raise ValueError('Invalid visual alignment label')
        limit = config['max_step_mm'] if camera == 'arm' else config['turn_step_deg']
        if camera == 'arm':
            step = math.copysign({2: 10, 1: 5, .5: 1, 0: 0}[abs(steps[label])], steps[label])
        else:
            step = steps[label] * config['turn_step_deg'] / 2
        moves[field] = max(-limit, min(limit, step))
    alignment_command(moves, config)
    return moves


def measured_arm_alignment(result, config):
    config = calibration_config(config)
    for box in result.values():
        if (not isinstance(box, list) or len(box) != 4
                or any(type(v) is not int or not 0 <= v <= 1000 for v in box)
                or box[0] >= box[2] or box[1] >= box[3]):
            raise ValueError('Arm target visible bounds unavailable or invalid')
    goal, current = result['goal'], result['current']
    x = (current[0] + current[2] - goal[0] - goal[2]) / 2
    goal_height, height = goal[3] - goal[1], current[3] - current[1]
    y = height - goal_height  # More exposed => Y+.
    width_ratio = (current[2] - current[0]) / (goal[2] - goal[0])
    area_ratio = width_ratio * height / goal_height
    if abs(x) <= 20 and abs(y) <= 20 and (abs(width_ratio - 1) > .10 or abs(area_ratio - 1) > .15
            or abs(current[1] - goal[1]) > 20 or abs(current[3] - goal[3]) > 20):
        raise ValueError('Arm GOAL size or pose still differs; correction direction unclear')
    def step(error):
        if abs(error) <= 20:
            return 0
        size = 10 if abs(error) > 150 else 5 if abs(error) > 60 else 1
        return math.copysign(min(config['max_step_mm'], size), error)
    return dict(x_mm=step(x), y_mm=step(y), turn_deg=0)


def pickup_delta(axis, delta):
    """J1=90: logical forward/right maps to native +Y/-X."""
    if axis not in ('X', 'Y', 'Z'):
        raise ValueError('Invalid pickup axis')
    return {'X': (0, delta, 0), 'Y': (-delta, 0, 0), 'Z': (0, 0, delta)}[axis]


def vision_image(frame, cv2, original_resolution=False):
    """Reuse bridge OpenCV; preserve the camera's mounted orientation."""
    height, width = frame.shape[:2]
    scale = 1 if original_resolution else min(1, 480 / max(width, height))
    if scale < 1:
        frame = cv2.resize(frame, (max(1, round(width * scale)), max(1, round(height * scale))),
                           interpolation=cv2.INTER_AREA)
    ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
    if not ok or len(encoded) > (4 * 1024 * 1024 if original_resolution else 160 * 1024):
        raise RuntimeError('Fetch camera compression failed')
    return 'data:image/jpeg;base64,' + base64.b64encode(encoded.tobytes()).decode('ascii')
