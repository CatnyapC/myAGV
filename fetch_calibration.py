"""Dual-camera pickup calibration; arm X is forward in mm; base turn is clockwise in degrees."""
import base64
import math

FETCH_VISION_TIMEOUT_S = 30


def calibration_config(value):
    if value is None:
        value = {}
    if not isinstance(value, dict):
        raise ValueError('Invalid fetch_calibration configuration')
    result = {key: value.get(key, default) for key, default in (('max_step_mm', 2), ('max_total_mm', 20))}
    for key in result:
        number = result.get(key)
        if type(number) not in (int, float) or not math.isfinite(number):
            raise ValueError('Invalid fetch calibration: ' + key)
    if not .1 <= result['max_step_mm'] <= 2:
        raise ValueError('Invalid fetch alignment step limit')
    if not result['max_step_mm'] <= result['max_total_mm'] <= 30:
        raise ValueError('Invalid fetch alignment travel budget')
    return result


def alignment_command(result, config):
    config = calibration_config(config)
    if not isinstance(result, dict) or set(result) != {'x_mm', 'turn_deg'}:
        raise ValueError('Invalid model alignment JSON')
    moves = {}
    for field, axis in (('x_mm', 'X'), ('turn_deg', 'turn_deg')):
        delta = result[field]
        if delta is None:
            raise ValueError('Pickup target missing or ambiguous')
        if type(delta) not in (int, float) or not math.isfinite(delta) or abs(delta) > 2:
            raise ValueError('Invalid model arm adjustment')
        if 0 < abs(delta) < .1:
            raise ValueError('Model arm adjustment is below measurable step size')
        limit = config['max_step_mm'] if axis == 'X' else 2
        moves[axis] = max(-limit, min(limit, delta))
    return moves


def qualitative_alignment(result, config):
    config = calibration_config(config)
    if not isinstance(result, dict) or set(result) != {'arm', 'front'}:
        raise ValueError('Invalid visual alignment labels')
    steps = {'left_large': -2, 'left_medium': -1, 'left_small': -.5,
             'aligned': 0, 'right_small': .5, 'right_medium': 1, 'right_large': 2}
    moves = {}
    for camera, field in (('arm', 'x_mm'), ('front', 'turn_deg')):
        label = result[camera]
        if label == 'unknown':
            raise ValueError('Pickup target missing or ambiguous: ' + camera)
        if not isinstance(label, str) or label not in steps:
            raise ValueError('Invalid visual alignment label')
        limit = config['max_step_mm'] if camera == 'arm' else 2
        moves[field] = max(-limit, min(limit, steps[label]))
    alignment_command(moves, config)
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
