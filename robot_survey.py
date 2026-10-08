"""Bounded viewing poses in the current connected, footprint-clear map region."""
from collections import deque
import math

from robot_safety import number
from web_backend.map_data import local_to_world, world_to_local


def update_goals(grid, start, radius, spacing_m=1, limit=200):
    number(spacing_m, .5, 3, 'Viewing spacing')
    number(radius, .15, 1, 'Measured clearance radius')
    width, height, r = grid['width'], grid['height'], grid['resolution_m']
    if not 0 < width * height <= 2_000_000 or len(grid['cells']) != width * height or not math.isfinite(r) or r <= 0:
        raise ValueError('Invalid update map')
    reach = math.ceil(radius / r + 1)
    # ponytail: square clearance is conservative; circular erosion if narrow passages matter.
    horizontal, clear = bytearray(width * height), bytearray(width * height)
    for y in range(height):
        row = y * width
        blocked = sum(c != 0 for c in grid['cells'][row:row + 2 * reach + 1])
        for x in range(reach, width - reach):
            horizontal[row + x] = blocked == 0
            if x + reach + 1 < width:
                blocked += (grid['cells'][row + x + reach + 1] != 0) - (grid['cells'][row + x - reach] != 0)
    for x in range(reach, width - reach):
        blocked = sum(not horizontal[y * width + x] for y in range(min(height, 2 * reach + 1)))
        for y in range(reach, height - reach):
            clear[y * width + x] = blocked == 0
            if y + reach + 1 < height:
                blocked += (not horizontal[(y + reach + 1) * width + x]) - (not horizontal[(y - reach) * width + x])
    u, v = world_to_local([start['x_m'], start['y_m']], grid['origin'])
    sx, sy = math.floor(u / r), math.floor(v / r)
    if not (0 <= sx < width and 0 <= sy < height) or not clear[sy * width + sx]:
        raise ValueError('Current pose lacks clear reachable space for map updating')
    pending = deque([sy * width + sx])
    seen = bytearray(width * height)
    seen[pending[0]] = 1
    step, sites = max(1, math.ceil(spacing_m / r)), {}
    while pending:
        index = pending.popleft()
        x, y = index % width, index // width
        tile = (x // step, y // step)
        distance = (x - (tile[0] + .5) * step) ** 2 + (y - (tile[1] + .5) * step) ** 2
        if tile not in sites or distance < sites[tile][0]:
            sites[tile] = (distance, x, y)
        for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
            if 0 <= nx < width and 0 <= ny < height:
                nxt = ny * width + nx
                if clear[nxt] and not seen[nxt]:
                    seen[nxt] = 1
                    pending.append(nxt)
    ordered = sorted(sites, key=lambda p: (p[1], p[0] if p[1] % 2 == 0 else -p[0]))
    # Four viewing headings per site; spread the bounded sample across the whole region.
    cap = limit // 4
    capped = len(ordered) > cap
    if capped:
        ordered = [ordered[i * len(ordered) // cap] for i in range(cap)]
    first = min(range(len(ordered)), key=lambda i: (sites[ordered[i]][1] - sx) ** 2 + (sites[ordered[i]][2] - sy) ** 2)
    ordered = ordered[first:] + ordered[:first]
    goals = []
    for tile in ordered:
        _, x, y = sites[tile]
        px, py = local_to_world([(x + .5) * r, (y + .5) * r], grid['origin'])
        for turn in range(4):
            yaw = (grid['origin']['yaw_rad'] + turn * math.pi / 2 + math.pi) % (2 * math.pi) - math.pi
            goals.append(dict(x_m=px, y_m=py, yaw_rad=yaw))
    return dict(goals=goals, capped=capped, spacing_m=spacing_m)
