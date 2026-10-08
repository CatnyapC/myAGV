"""Remove small isolated returns before mapping, masking and inflation."""
import math


def filter_grid(grid, minimum):
    raw = grid.get('raw_cells', grid['cells'])
    cells = list(raw)
    if minimum > 0:
        width, height = grid['width'], grid['height']
        remaining = {i for i, v in enumerate(cells) if v == 100}
        while remaining:
            first = remaining.pop()
            component, pending = [first], [first]
            while pending:
                i = pending.pop()
                x, y = i % width, i // width
                for yy in range(max(0, y-1), min(height, y+2)):
                    for xx in range(max(0, x-1), min(width, x+2)):
                        j = yy*width + xx
                        if j in remaining:
                            remaining.remove(j)
                            pending.append(j)
                            component.append(j)
            xs, ys = [i % width for i in component], [i // width for i in component]
            span = max(max(xs)-min(xs)+1, max(ys)-min(ys)+1)*grid['resolution_m']
            if span < minimum:
                for i in component:
                    cells[i] = 0
    return {**grid, 'raw_cells': raw, 'cells': cells}


def filter_scan(scan, minimum):
    ranges = list(scan.ranges)
    if minimum <= 0:
        return ranges
    points = {}
    for i, r in enumerate(ranges):
        if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
            a = scan.angle_min + i*scan.angle_increment
            points[i] = (r*math.cos(a), r*math.sin(a))
    def adjacent(a, b):
        # Allow normal angular spacing at long range, but split depth jumps.
        gap = max(.1, 3*min(ranges[a], ranges[b])*abs(scan.angle_increment))
        return math.dist(points[a], points[b]) <= gap
    groups = []
    for i in points:
        if groups and i == groups[-1][-1]+1 and adjacent(groups[-1][-1], i):
            groups[-1].append(i)
        else:
            groups.append([i])
    if (len(groups) > 1 and groups[0][0] == 0 and groups[-1][-1] == len(ranges)-1
            and abs(len(ranges)*scan.angle_increment) > 6.2 and adjacent(groups[-1][-1], 0)):
        groups[0] = groups.pop() + groups[0]
    for group in groups:
        xs, ys = zip(*(points[i] for i in group))
        span = math.hypot(max(xs)-min(xs), max(ys)-min(ys))
        span += min(ranges[i] for i in group)*abs(scan.angle_increment)
        if span < minimum:
            for i in group:
                ranges[i] = float('inf')
    return ranges
