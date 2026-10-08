"""Pure checks shared by the ROS controller and its hardware-free tests."""
import math

from web_backend.map_data import world_to_local


def fresh(stamp, now, limit):
    return type(stamp) in (int, float) and math.isfinite(stamp) and 0 <= now - stamp <= limit


def number(value, low, high, label):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError('%s must be in %s..%s' % (label, low, high))
    return value


def lease_valid(deadline, now):
    return type(deadline) in (int, float) and now < deadline <= now + .6


def footprint_clear(grid, pose, radius):
    """Conservative circular envelope, including touched cells and unknown space."""
    x, y = world_to_local([pose['x_m'], pose['y_m']], grid['origin'])
    r, width, height = grid['resolution_m'], grid['width'], grid['height']
    margin = radius + r / math.sqrt(2)
    left, right = math.floor((x - margin) / r), math.floor((x + margin) / r)
    bottom, top = math.floor((y - margin) / r), math.floor((y + margin) / r)
    if left < 0 or bottom < 0 or right >= width or top >= height:
        return False
    return all(grid['cells'][row * width + col] == 0
               for row in range(bottom, top + 1) for col in range(left, right + 1)
               if math.hypot((col + .5) * r - x, (row + .5) * r - y) <= margin)


def zones_visible(grid, points):
    """Every virtual obstacle inside this costmap must remain lethal."""
    r, width, height = grid['resolution_m'], grid['width'], grid['height']
    for point in points:
        x, y = world_to_local(point, grid['origin'])
        col, row = math.floor(x / r), math.floor(y / r)
        if 0 <= col < width and 0 <= row < height and grid['cells'][row * width + col] < 100:
            return False
    return True


def manual_vector(mode, key, speed=.05, turn=.15):
    number(speed, .01, .1, 'Base speed')
    number(turn, .03, .3, 'Turn speed')
    if mode == 'PICKUP':
        speed, turn = .03, .05
    bindings = {'forward': (1, 0, 0), 'back': (-1, 0, 0), 'left': (0, 1, 0),
                'right': (0, -1, 0), 'ccw': (0, 0, 1), 'cw': (0, 0, -1)}
    if mode not in ('BASE', 'PICKUP') or key not in bindings or (mode == 'PICKUP' and key in ('left', 'right')):
        raise ValueError('Invalid base input')
    x, y, yaw = bindings[key]
    return x * speed, y * speed, yaw * turn
