"""Dual-camera pickup calibration; arm X is forward in mm; base turn is clockwise in degrees."""
import base64
import math

FETCH_VISION_TIMEOUT_S = 65
FETCH_VISION_RETRIES = 5


def calibration_config(value):
    if value is None:
        value = {}
    if not isinstance(value, dict):
        raise ValueError('Invalid fetch_calibration configuration')
    result = {key: value.get(key, default) for key, default in (
        ('max_step_mm', 0), ('turn_speed_rad_s', .015),
        ('turn_duration_scale', .8), ('turn_step_deg', 2))}
    for key in result:
        number = result.get(key)
        if type(number) not in (int, float) or not math.isfinite(number):
            raise ValueError('Invalid fetch calibration: ' + key)
    if result['max_step_mm'] < 0:
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
        if type(delta) not in (int, float) or not math.isfinite(delta) or (axis == 'turn_deg' and abs(delta) > 2):
            raise ValueError('Invalid model arm adjustment')
        if 0 < abs(delta) < .1:
            raise ValueError('Model arm adjustment is below measurable step size')
        limit = 2 if axis == 'turn_deg' else config['max_step_mm']
        moves[axis] = max(-limit, min(limit, delta)) if limit else delta
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
        limit = (config['max_step_mm'] or 10) if camera == 'arm' else config['turn_step_deg']
        if camera == 'arm':
            step = math.copysign({2: 10, 1: 5, .5: 1, 0: 0}[abs(steps[label])], steps[label])
        else:
            step = steps[label] * config['turn_step_deg'] / 2
        moves[field] = max(-limit, min(limit, step))
    alignment_command(moves, config)
    return moves


def visible_bounds(box):
    if (not isinstance(box, list) or len(box) != 4
            or any(type(v) is not int or not 0 <= v <= 1000 for v in box)
            or box[0] >= box[2] or box[1] >= box[3]):
        raise ValueError('Arm target visible bounds unavailable or invalid')
    return [(box[0] + box[2]) / 2, box[1]]


def metric_arm_alignment(result, config):
    if not isinstance(result, dict) or set(result) != {'x_mm', 'y_mm', 'aligned'} or type(result['aligned']) is not bool:
        raise ValueError('Invalid metric alignment result')
    values = [result[k] for k in ('x_mm', 'y_mm')]
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
        raise ValueError('Arm metric error missing, ambiguous or out of range')
    if result['aligned'] and math.hypot(*values) >= 1:
        raise ValueError('Model alignment flag contradicts its one-millimeter error')
    # Check the full residual before an optional travel limit can make a large error look small.
    aligned = sum(abs(v) for v in values) < 1.25
    limit = calibration_config(config)['max_step_mm']
    moves = [0 if aligned or abs(v) < .1 else max(-limit, min(limit, v)) if limit else v for v in values]
    return dict(x_mm=moves[0], y_mm=moves[1], turn_deg=0), aligned


class MetricArmCalibration:
    """Learn a local 2x2 image/millimeter map using measured, independent axis moves."""
    def __init__(self, saved=None):
        self.goal = None
        self.pending = None
        self.columns = []
        self.inverse = None
        self.z = None
        self.reused = saved is not None
        if saved is not None:
            visible_bounds(saved['goal_bounds'])
            matrix = saved['image_error_to_remaining_mm']
            if (not isinstance(matrix, list) or len(matrix) != 2
                    or any(not isinstance(row, list) or len(row) != 2 for row in matrix)
                    or any(type(v) not in (int, float) or not math.isfinite(v) for row in matrix for v in row)):
                raise ValueError('Invalid saved ARM metric calibration')
            a, b = matrix[0]
            c, d = matrix[1]
            determinant = a*d-b*c
            if not math.isfinite(determinant) or abs(determinant) <= 0 or abs(determinant) < .25 * math.hypot(a,c) * math.hypot(b,d):
                raise ValueError('Saved ARM metric calibration is ambiguous')
            self.goal = list(saved['goal_bounds'])
            self.inverse = [list(row) for row in matrix]

    def context(self):
        return dict(phase='metric' if self.inverse else 'calibrate', goal_bounds=self.goal,
                    image_error_to_remaining_mm=self.inverse)

    def observe(self, result, position, config):
        if not isinstance(result, dict) or set(result) != {'goal', 'current'}:
            raise ValueError('Invalid calibration bounds')
        visible_bounds(result['goal'])
        feature = visible_bounds(result['current'])
        if self.goal is None:
            self.goal = list(result['goal'])
        target = visible_bounds(self.goal)
        if (not isinstance(position, list) or len(position) != 3 or
                any(type(v) not in (int, float) or not math.isfinite(v) for v in position)):
            raise ValueError('Measured ARM position required for millimeter calibration')
        if self.z is None:
            self.z = position[2]
        if abs(position[2] - self.z) > 1:
            raise ValueError('ARM Z changed during millimeter calibration')
        limit = calibration_config(config)['max_step_mm'] or 20
        box = result['current']
        if self.pending:
            axis, before, old_feature, requested = self.pending
            delta = [position[1] - before[1], before[0] - position[0]]
            if abs(delta[1-axis]) > .5 or abs(delta[axis]) < 1.5 or delta[axis] * requested <= 0:
                raise ValueError('Calibration probe did not reach a measurable independent displacement')
            shift = [feature[i] - old_feature[i] for i in range(2)]
            if max(abs(v) for v in shift) < 5 or box[0] == 0 or box[2] == 1000 or box[1] == 0:
                raise ValueError('Calibration target clipped or image response too small')
            self.columns.append([v / delta[axis] for v in shift])
            self.pending = None
        if len(self.columns) == 2:
            (a, c), (b, d) = self.columns
            determinant = a*d - b*c
            if abs(determinant) < .25 * math.hypot(a, c) * math.hypot(b, d):
                raise ValueError('ARM image/millimeter calibration is ambiguous')
            self.inverse = [[-d/determinant, b/determinant], [c/determinant, -a/determinant]]
            error = [feature[i] - target[i] for i in range(2)]
            residual = [sum(row[i] * error[i] for i in range(2)) for row in self.inverse]
            raw = dict(x_mm=residual[0], y_mm=residual[1], aligned=math.hypot(*residual) < 1)
            moves, aligned = metric_arm_alignment(raw, config)
            return moves, aligned, raw
        # A clipped fragment has no reliable midpoint for a metric probe; bring it into view first.
        if box[0] == 0 or box[2] == 1000 or box[1] == 0:
            axis = 0 if box[0] == 0 or box[2] == 1000 else 1
            error = feature[axis] - target[axis]
            moves = [0, 0]
            moves[axis] = math.copysign(min(limit, 20 if axis == 0 else 5), error)
            return dict(x_mm=moves[0], y_mm=moves[1], turn_deg=0), False, None
        if limit < 2:
            raise ValueError('Use an ARM step limit of at least 2 mm for automatic metric calibration')
        axis = len(self.columns)
        distance = math.copysign(min(5, limit), feature[axis] - target[axis])
        self.pending = axis, list(position), feature, distance
        moves = [0, 0]
        moves[axis] = distance
        return dict(x_mm=moves[0], y_mm=moves[1], turn_deg=0), False, None


def measured_arm_alignment(result, config):
    config = calibration_config(config)
    for box in result.values():
        visible_bounds(box)
    goal, current = result['goal'], result['current']
    x = (current[0] + current[2] - goal[0] - goal[2]) / 2
    goal_height, height = goal[3] - goal[1], current[3] - current[1]
    # Measured on hardware: logical Y+ moves the visible target upward in the image.
    y = current[1] - goal[1]
    tolerance, top_tolerance = 20, 5
    width_ratio = (current[2] - current[0]) / (goal[2] - goal[0])
    area_ratio = width_ratio * height / goal_height
    if abs(x) <= tolerance and abs(y) <= top_tolerance and (abs(width_ratio - 1) > .10
            or abs(area_ratio - 1) > .15 or abs(current[3] - goal[3]) > tolerance):
        raise ValueError('Arm GOAL size or pose still differs; correction direction unclear')
    def step(error, threshold):
        if abs(error) <= threshold:
            return 0
        size = .25 if abs(error) <= 30 else 1
        size = 10 if abs(error) > 150 else 5 if abs(error) > 60 else size
        return math.copysign(min(config['max_step_mm'] or 10, size), error)
    return dict(x_mm=step(x, tolerance), y_mm=step(y, top_tolerance), turn_deg=0)


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
