"""Deterministic local navigation harness; never imports ROS or drives hardware."""
import heapq
import math
import time

from .map_data import local_to_world, world_to_local

LENGTH_M, WIDTH_M, CLEARANCE_M = .32, .28, .05
RADIUS_M = math.hypot(LENGTH_M / 2, WIDTH_M / 2) + CLEARANCE_M


def intersects(a, b):
    """Separating-axis test; touching cells are conservatively occupied."""
    for polygon in (a, b):
        for i, p in enumerate(polygon):
            q = polygon[(i + 1) % len(polygon)]
            axis = (p[1] - q[1], q[0] - p[0])
            pa = [v[0] * axis[0] + v[1] * axis[1] for v in a]
            pb = [v[0] * axis[0] + v[1] * axis[1] for v in b]
            if max(pa) < min(pb) - 1e-9 or max(pb) < min(pa) - 1e-9:
                return False
    return True


def navigation_grid(grid, zones):
    cells = [0 if cell == -1 else cell for cell in grid['cells']]
    r, width, height = grid['resolution_m'], grid['width'], grid['height']
    for zone in zones:
        polygon = [world_to_local(p, grid['origin']) for p in zone['corners']]
        lo_x = max(0, math.floor(min(p[0] for p in polygon) / r) - 1)
        hi_x = min(width - 1, math.floor(max(p[0] for p in polygon) / r))
        lo_y = max(0, math.floor(min(p[1] for p in polygon) / r) - 1)
        hi_y = min(height - 1, math.floor(max(p[1] for p in polygon) / r))
        for y in range(lo_y, hi_y + 1):
            for x in range(lo_x, hi_x + 1):
                square = [(x*r, y*r), ((x+1)*r, y*r), ((x+1)*r, (y+1)*r), (x*r, (y+1)*r)]
                if intersects(polygon, square):
                    cells[y * width + x] = 100
    return {**grid, 'cells': cells}


def global_costmap(grid):
    r, width, height = grid['resolution_m'], grid['width'], grid['height']
    # Extra cell diagonal covers off-center poses and cell-to-cell segments.
    radius = RADIUS_M + math.sqrt(2) * r
    reach = math.ceil(radius / r)
    offsets = [(dx, dy) for dy in range(-reach, reach + 1) for dx in range(-reach, reach + 1)
               if math.hypot(dx*r, dy*r) <= radius]
    cells = list(grid['cells'])
    for y in range(height):
        for x in range(width):
            if cells[y*width+x] == 0 and min((x+.5)*r, (width-x-.5)*r, (y+.5)*r, (height-y-.5)*r) <= radius:
                cells[y*width+x] = 100  # The chassis cannot extend beyond known map bounds.
    for i, value in enumerate(grid['cells']):
        if value == 0:
            continue
        x, y = i % width, i // width
        for dx, dy in offsets:
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height and cells[ny * width + nx] == 0:
                cells[ny * width + nx] = 100
    return {**grid, 'cells': cells}


def validate_pose(value):
    if not isinstance(value, dict) or set(value) != {'x_m', 'y_m', 'yaw_rad'}:
        raise ValueError('Pose requires x_m, y_m and yaw_rad')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value.values()):
        raise ValueError('Pose must contain finite numbers')
    if abs(value['yaw_rad']) > math.pi:
        raise ValueError('Heading must be between -pi and pi radians')
    return dict(value)


def cell_index(grid, pose):
    u, v = world_to_local([pose['x_m'], pose['y_m']], grid['origin'])
    x, y = math.floor(u / grid['resolution_m']), math.floor(v / grid['resolution_m'])
    if not (0 <= x < grid['width'] and 0 <= y < grid['height']):
        raise ValueError('Pose outside map bounds')
    index = y * grid['width'] + x
    if grid['cells'][index] != 0:
        raise ValueError('Pose blocked by global costmap')
    return index


def segment_clear(grid, start, end):
    """Check the entire segment, including cells touched at edges/corners."""
    segment = [world_to_local(p, grid['origin']) for p in (start, end)]
    r, width, height = grid['resolution_m'], grid['width'], grid['height']
    lo_x = max(0, math.floor(min(p[0] for p in segment) / r) - 1)
    hi_x = min(width - 1, math.floor(max(p[0] for p in segment) / r) + 1)
    lo_y = max(0, math.floor(min(p[1] for p in segment) / r) - 1)
    hi_y = min(height - 1, math.floor(max(p[1] for p in segment) / r) + 1)
    for y in range(lo_y, hi_y + 1):
        for x in range(lo_x, hi_x + 1):
            if grid['cells'][y * width + x] == 0:
                continue
            square = [(x*r, y*r), ((x+1)*r, y*r), ((x+1)*r, (y+1)*r), (x*r, (y+1)*r)]
            if intersects(segment, square):
                return False
    return True


def simplify_path(grid, points):
    # ponytail: greedy bounding-box scans suit the demo; use grid ray traversal for larger maps.
    route, index = [points[0]], 0
    while index < len(points) - 1:
        next_index = len(points) - 1
        while next_index > index + 1 and not segment_clear(grid, points[index], points[next_index]):
            next_index -= 1
        route.append(points[next_index])
        index = next_index
    return route


def plan_path(grid, start, goal):
    first, last = cell_index(grid, start), cell_index(grid, goal)
    width, height, cells = grid['width'], grid['height'], grid['cells']
    def heuristic(index):
        return math.hypot(index % width - last % width, index // width - last // width)
    frontier, scores, parents = [(heuristic(first), 0, first)], {first: 0}, {}
    # ponytail: bounded small demo map; production delegates planning to ROS1 move_base.
    while frontier:
        _, score, current = heapq.heappop(frontier)
        if score != scores[current]:
            continue
        if current == last:
            route = [last]
            while route[-1] != first:
                route.append(parents[route[-1]])
            r = grid['resolution_m']
            points = [local_to_world([((i % width) + .5)*r, ((i // width) + .5)*r], grid['origin']) for i in reversed(route)]
            return simplify_path(grid, [[start['x_m'], start['y_m']], *points, [goal['x_m'], goal['y_m']]])
        x, y = current % width, current // width
        for dx, dy in ((1,0), (-1,0), (0,1), (0,-1), (1,1), (1,-1), (-1,1), (-1,-1)):
            nx, ny = x + dx, y + dy
            if not (0 <= nx < width and 0 <= ny < height):
                continue
            nxt = ny * width + nx
            if cells[nxt] != 0 or (dx and dy and (cells[y * width + nx] != 0 or cells[ny * width + x] != 0)):
                continue
            cost = score + math.hypot(dx, dy)
            if cost < scores.get(nxt, math.inf):
                scores[nxt], parents[nxt] = cost, current
                heapq.heappush(frontier, (cost + heuristic(nxt), cost, nxt))
    raise ValueError('No path in global costmap')


class Simulation:
    def __init__(self, grid, zones):
        self.source = grid
        self.phase, self.status = 'idle', 'Ready'
        self.pose, self.goal, self.path = None, None, []
        self.index, self.generation = 1, 0
        self.rebuild(zones)
        self.reset()

    def rebuild(self, zones, commit=True):
        navigation = navigation_grid(self.source, zones['zones'])
        costmap = global_costmap(navigation)
        if self.pose is not None:
            try:
                cell_index(costmap, self.pose)
            except ValueError as exc:
                raise ValueError('Zone overlaps the robot clearance envelope') from exc
        if commit:
            self.navigation, self.costmap, self.revision = navigation, costmap, zones['revision']
        return navigation, costmap

    def reset(self):
        width, height, r = self.source['width'], self.source['height'], self.source['resolution_m']
        free = [i for i, v in enumerate(self.costmap['cells']) if v == 0]
        if not free:
            raise ValueError('No safe simulated starting pose')
        index = min(free, key=lambda i: (i % width - width*.35)**2 + (i // width - height*.4)**2)
        x, y = local_to_world([(index % width + .5)*r, (index // width + .5)*r], self.source['origin'])
        self.pose = dict(x_m=x, y_m=y, yaw_rad=self.source['origin']['yaw_rad'])
        self.goal = None
        self.stop('Ready')

    def stop(self, status='Stopped'):
        self.generation += 1
        self.phase, self.status, self.path = 'idle', status, []

    def start(self, goal, path):
        self.goal, self.path, self.index = goal, path, 1
        self.phase, self.status = 'running', 'Navigating'

    def advance(self, seconds):
        if self.phase != 'running':
            return
        point = self.path[self.index] if self.index < len(self.path) else None
        distance = math.hypot(point[0]-self.pose['x_m'], point[1]-self.pose['y_m']) if point else 0
        if point and distance <= 1e-7:
            self.index += 1
            return
        desired = math.atan2(point[1]-self.pose['y_m'], point[0]-self.pose['x_m']) if distance > 1e-7 else self.goal['yaw_rad']
        error = (desired - self.pose['yaw_rad'] + math.pi) % (2*math.pi) - math.pi
        turn = max(-seconds*1.8, min(seconds*1.8, error))
        self.pose['yaw_rad'] = (self.pose['yaw_rad'] + turn + math.pi) % (2*math.pi) - math.pi
        if abs(error) > .04:
            return
        if point:
            step = min(distance, .6*seconds)
            if distance > 1e-7:
                self.pose['x_m'] += (point[0]-self.pose['x_m'])*step/distance
                self.pose['y_m'] += (point[1]-self.pose['y_m'])*step/distance
            if distance <= step + 1e-7:
                self.index += 1
        else:
            self.stop('Arrived')

    def telemetry(self):
        return dict(map_id=self.source['map_id'], frame=self.source['frame'], stamp_s=time.time(),
                    source='simulation', pose=dict(self.pose), goal=self.goal,
                    moving=self.phase == 'running', phase=self.phase, status=self.status,
                    path=([[self.pose['x_m'], self.pose['y_m']], *self.path[self.index:]] if self.path else []),
                    footprint=dict(length_m=LENGTH_M, width_m=WIDTH_M),
                    costmap=dict(ready=True, applied_zone_revision=self.revision, clearance_m=CLEARANCE_M))
