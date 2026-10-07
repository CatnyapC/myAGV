"""Synthetic OccupancyGrid and a display-only aligned raster (no ROS imports)."""
import math
import struct
import zlib
from collections import Counter


def rotate(x, y, angle):
    c, s = math.cos(angle), math.sin(angle)
    return c * x - s * y, s * x + c * y


def world_to_local(point, origin):
    return rotate(point[0] - origin['x_m'], point[1] - origin['y_m'], -origin['yaw_rad'])


def local_to_world(point, origin):
    x, y = rotate(*point, origin['yaw_rad'])
    return [x + origin['x_m'], y + origin['y_m']]


def demo_map():
    width, height, resolution = 240, 200, 0.05
    cells = []
    # An intentionally rotated room, doorway, divider and two pieces of furniture.
    for row in range(height):
        for col in range(width):
            x, y = rotate((col + .5) * resolution - 6, (row + .5) * resolution - 5,
                          -math.radians(21))
            inside = abs(x) < 4 and abs(y) < 3
            wall = inside and (abs(x) > 3.88 or abs(y) > 2.88)
            divider = abs(x - 1) < .07 and -.8 < y < 2.9
            furniture = (-2.8 < x < -1.4 and .7 < y < 1.5) or (1.9 < x < 2.9 and -2 < y < -.9)
            cells.append(100 if wall or (inside and (divider or furniture)) else 0 if inside else -1)
    return dict(map_id='demo-room-v1', revision=1, frame='map', width=width, height=height,
                resolution_m=resolution, origin=dict(x_m=-3.2, y_m=1.4, yaw_rad=math.radians(17)),
                cells=cells)


def dominant_angle(grid):
    points = [(i % grid['width'], i // grid['width']) for i, cell in enumerate(grid['cells']) if cell == 100]
    if len(points) < 40:
        return None
    # ponytail: bounded orthogonal projection vote, not arbitrary-room segmentation.
    # Use a line detector if real non-Manhattan maps need more than manual fallback.
    points = points[::max(1, len(points) // 4000)]
    scores = []
    for degrees in range(90):
        angle = math.radians(degrees)
        xs, ys = Counter(), Counter()
        for x, y in points:
            u, v = rotate(x, y, -angle)
            xs[round(u)] += 1
            ys[round(v)] += 1
        scores.append(sum(n * n for n in xs.values()) + sum(n * n for n in ys.values()))
    best = max(range(90), key=scores.__getitem__)
    median = sorted(scores)[45]
    if scores[best] < median * 1.3:
        return None
    return math.radians((best + 45) % 90 - 45)


def png_rgb(width, height, rows, alpha=False):
    def chunk(kind, payload):
        return struct.pack('!I', len(payload)) + kind + payload + struct.pack('!I', zlib.crc32(kind + payload))
    header = struct.pack('!2I5B', width, height, 8, 6 if alpha else 2, 0, 0, 0)
    return b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', header) + chunk(b'IDAT', zlib.compress(rows)) + chunk(b'IEND', b'')


def render_map(grid, settings, view_revision, detected_angle, colors=None):
    source = grid['origin']
    angle = source['yaw_rad'] + (detected_angle or 0) if settings['auto_align'] else math.radians(settings['manual_angle_deg'])
    corners = [local_to_world([x, y], source) for x, y in
               [(0, 0), (grid['width'] * grid['resolution_m'], 0),
                (grid['width'] * grid['resolution_m'], grid['height'] * grid['resolution_m']),
                (0, grid['height'] * grid['resolution_m'])]]
    projected = [rotate(*p, -angle) for p in corners]
    low_x, low_y = min(p[0] for p in projected), min(p[1] for p in projected)
    high_x, high_y = max(p[0] for p in projected), max(p[1] for p in projected)
    r = grid['resolution_m']
    width, height = math.ceil((high_x - low_x) / r), math.ceil((high_y - low_y) / r)
    ox, oy = rotate(low_x, low_y, angle)
    origin = dict(x_m=ox, y_m=oy, yaw_rad=angle)
    rows = bytearray()
    alpha = colors is not None
    colors = colors or {-1: b'\x1b\x1b\x1d', 0: b'\x34\x34\x38', 100: b'\xb4\xb4\xb8'}
    # PNG is top-down; OccupancyGrid is bottom-up. Reverse Y here, exactly once.
    for row in reversed(range(height)):
        rows.append(0)
        for col in range(width):
            p = local_to_world([(col + .5) * r, (row + .5) * r], origin)
            u, v = world_to_local(p, source)
            x, y = math.floor(u / r), math.floor(v / r)
            value = grid['cells'][y * grid['width'] + x] if 0 <= x < grid['width'] and 0 <= y < grid['height'] else -1
            rows.extend(colors[value])
    metadata = {k: v for k, v in grid.items() if k != 'cells'}
    metadata['display'] = dict(width_m=width * r, height_m=height * r, origin=origin,
                               view_revision=view_revision, alignment_available=detected_angle is not None)
    metadata['png_url'] = f'/api/map.png?view_revision={view_revision}'
    return metadata, png_rgb(width, height, rows, alpha=alpha)


def validate_rectangle(corners, grid):
    if not isinstance(corners, list) or len(corners) != 4:
        raise ValueError('Four ordered corners required')
    clean = []
    for point in corners:
        if not isinstance(point, list) or len(point) != 2:
            raise ValueError('Invalid corner')
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in point):
            raise ValueError('Corners must be finite numbers')
        u, v = world_to_local(point, grid['origin'])
        if not (-1e-7 <= u <= grid['width'] * grid['resolution_m'] + 1e-7 and
                -1e-7 <= v <= grid['height'] * grid['resolution_m'] + 1e-7):
            raise ValueError('Zone outside map bounds')
        clean.append(point)
    edges = [(clean[(i + 1) % 4][0] - p[0], clean[(i + 1) % 4][1] - p[1]) for i, p in enumerate(clean)]
    lengths = [math.hypot(*e) for e in edges]
    if min(lengths) < grid['resolution_m']:
        raise ValueError('Zone must be at least one map cell wide and high')
    for i, edge in enumerate(edges):
        other = edges[(i + 1) % 4]
        if abs(edge[0] * other[0] + edge[1] * other[1]) > lengths[i] * lengths[(i + 1) % 4] * 1e-6:
            raise ValueError('Zone must be a rectangle')
        opposite = edges[(i + 2) % 4]
        if math.hypot(edge[0] + opposite[0], edge[1] + opposite[1]) > 1e-6:
            raise ValueError('Zone must have ordered rectangle corners')
    return clean
