"""Score path-directed turns and preserve TrajectoryPlanner collision failures."""
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
    if rotation.count(new) != 1 or old in rotation:
        if rotation.count(old) != 1:
            raise ValueError('Unexpected rotation score; refused partial patch')
        source = source[:start] + rotation.replace(old, new) + source[end:]
    anchor = '      //the point is legal... add it to the trajectory'
    heading = ('      // MYAGV_ENDPOINT_HEADING: score forward arcs without dense time sampling.\n'
               '      if (!heading_scoring_ && i == num_steps - 1) {\n'
               '        heading_diff = headingDiff(cell_x, cell_y, x_i, y_i, theta_i);\n'
               '        if (heading_diff == DBL_MAX) { traj.cost_ = -2.0; return; }\n'
               '      }\n\n')
    cost = 'cost = path_distance_bias_ * path_dist + goal_dist * goal_distance_bias_ + occdist_scale_ * occ_cost;'
    scored = cost[:-1] + ' + 0.3 * heading_diff;'
    if not (source.count(heading + anchor) == 1 and source.count(scored) == 1 and cost not in source):
        if source.count(anchor) != 1 or source.count(cost) != 1 or 'MYAGV_ENDPOINT_HEADING' in source:
            raise ValueError('Unexpected trajectory score; refused partial patch')
        source = source.replace(anchor, heading + anchor).replace(cost, scored)
    unsafe = 'if(best_traj->cost_ == -1.0)\n      best_traj->cost_ = 1.0;'
    safe = '// MYAGV_COLLISION_STOP: keep failed collision checks invalid.'
    if source.count(safe) == 1 and unsafe not in source:
        return source
    if source.count(unsafe) != 1:
        raise ValueError('Unexpected escape collision check; refused partial patch')
    return source.replace(unsafe, safe)


def patch_goal_stop(source):
    start = source.find('  bool TrajectoryPlannerROS::stopWithAccLimits(')
    end = source.find('  bool TrajectoryPlannerROS::rotateToGoal(', start)
    if start < 0 or end < 0:
        raise ValueError('Unexpected goal stop function; refused partial patch')
    original = source[start:end]
    updated = (original[:original.find('{') + 1] + '\n'
               '    // MYAGV_MEASURED_STOP: command zero; the caller waits for stopped odometry before turning.\n'
               '    cmd_vel.linear.x = 0.0;\n'
               '    cmd_vel.linear.y = 0.0;\n'
               '    cmd_vel.angular.z = 0.0;\n'
               '    return true;\n'
               '  }\n\n')
    if original != updated and not all(token in original for token in (
            'double vx = sign(robot_vel.pose.position.x)', 'double vth = sign(vel_yaw)',
            'bool valid_cmd = tc_->checkTrajectory', 'return false;')):
        raise ValueError('Unexpected goal deceleration; refused partial patch')
    return source[:start] + updated + source[end:]


if __name__ == '__main__':
    path = Path(sys.argv[1])
    changes = []
    for source, transform in ((path, patch), (path.with_name('trajectory_planner_ros.cpp'), patch_goal_stop)):
        original = source.read_text()
        updated = transform(original)
        if updated != original:
            backup = source.with_suffix(source.suffix + '.before-shortest-turn-v2')
            if backup.exists():
                raise SystemExit('Backup exists; inspect previous installation first')
            changes.append((source, backup, original, updated))
    for source, backup, original, updated in changes:
        backup.write_text(original)
        source.write_text(updated)
    print('Navigation source ready; rebuild base_local_planner and restart navigation')
