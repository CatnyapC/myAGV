"""Remove TrajectoryPlanner's clockwise sampling bias during in-place turns."""
from pathlib import Path
import sys


def patch(source):
    start = source.find('//next we want to generate trajectories for rotating in place')
    end = source.find('//do we have a legal trajectory', start)
    if start < 0 or end < 0:
        raise ValueError('Unexpected planner source; refused partial patch')
    rotation = source[start:end]
    old = 'double ahead_gdist = goal_map_(cell_x, cell_y).target_dist;'
    new = ('double ahead_gdist = headingDiff(cell_x, cell_y, x_r, y_r, th_r);'
           ' // MYAGV_SHORTEST_TURN')
    if rotation.count(new) == 1 and old not in rotation:
        return source
    if rotation.count(old) != 1:
        raise ValueError('Unexpected rotation score; refused partial patch')
    return source[:start] + rotation.replace(old, new) + source[end:]


if __name__ == '__main__':
    path = Path(sys.argv[1])
    original = path.read_text()
    updated = patch(original)
    if updated != original:
        backup = path.with_suffix(path.suffix + '.before-shortest-turn')
        if backup.exists():
            raise SystemExit('Backup exists; inspect previous installation first')
        backup.write_text(original)
        path.write_text(updated)
    print('Shortest-turn source ready; rebuild base_local_planner and restart navigation')
