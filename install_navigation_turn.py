"""Score path-directed turns and preserve TrajectoryPlanner collision failures."""
from pathlib import Path
import sys


def replace_once(source, old, new):
    if source.count(new) == 1:
        return source
    if source.count(old) != 1:
        raise ValueError('Unexpected navigation source; refused partial patch: ' + old[:70])
    return source.replace(old, new)


def patch_header(source):
    return replace_once(source, '      bool escaping_;',
                        '      bool myagv_following_ = false; // MYAGV_POLYLINE_FOLLOW\n      bool escaping_;')


def patch_global(source):
    source = replace_once(source, '#include <global_planner/planner_core.h>',
                          '#include <global_planner/planner_core.h>\n#include "myagv_polyline.h"')
    return replace_once(source, '    // add orientations if needed',
                        '    plan = myagv::simplify(*costmap_, plan); // MYAGV_POLYLINE\n\n    // add orientations if needed')


def patch(source):
    source = replace_once(source, '#include <base_local_planner/trajectory_planner.h>',
                          '#include <base_local_planner/trajectory_planner.h>\n#include "myagv_polyline.h"')
    source = replace_once(source, '    global_plan_.resize(new_plan.size());',
        '    if (new_plan.empty()) myagv_following_ = false;\n'
        '    global_plan_.resize(new_plan.size());')
    source = replace_once(source, '    //we want to sample the velocity space regularly',
        '    // MYAGV_POLYLINE_FOLLOW: align within 45 degrees; keep following until error exceeds 60.\n'
        '    unsigned int heading_x, heading_y;\n'
        '    double error = DBL_MAX;\n'
        '    if (costmap_.worldToMap(x, y, heading_x, heading_y))\n'
        '      error = headingDiff(heading_x, heading_y, x, y, theta);\n'
        '    // findBestPath stores measured yaw as float.\n'
        '    if (error > M_PI / 3.0 + 1e-6) myagv_following_ = false;\n'
        '    else if (error <= M_PI / 4.0 + 1e-6) myagv_following_ = true;\n'
        '    // Feedback outside the speed limits must not invert the sampling interval.\n'
        '    min_vel_x = min(min_vel_x, max_vel_x);\n'
        '    min_vel_theta = max(min_vel_th_, min(max_vel_th_, min_vel_theta));\n'
        '    max_vel_theta = max(min_vel_theta, max(min_vel_th_, min(max_vel_th_, max_vel_theta)));\n\n'
        '    //we want to sample the velocity space regularly')
    signature = '  double TrajectoryPlanner::headingDiff('
    start = source.find(signature)
    end = source.find('  //calculate the cost of a ray-traced line', start)
    if start < 0 or end < 0:
        raise ValueError('Unexpected heading function; refused partial patch')
    original = source[start:end]
    updated = ('  double TrajectoryPlanner::headingDiff(int cell_x, int cell_y, double x, double y, double heading){\n'
        '    // MYAGV_SEGMENT_HEADING: target the current straight segment, using exact world coordinates.\n'
        '    if (global_plan_.empty()) return DBL_MAX;\n'
        '    unsigned int gx, gy;\n'
        '    for (int i = myagv::segmentEnd(global_plan_, x, y); i >= 0; --i) {\n'
        '      const auto& p = global_plan_[i].pose.position;\n'
        '      if (hypot(p.x-x, p.y-y) > 1e-6 && costmap_.worldToMap(p.x, p.y, gx, gy) &&\n'
        '          lineCost(cell_x, gx, cell_y, gy) >= 0)\n'
        '        return fabs(angles::shortest_angular_distance(heading, atan2(p.y-y, p.x-x)));\n'
        '    }\n'
        '    return DBL_MAX;\n'
        '  }\n\n')
    if original != updated and not all(token in original for token in ('global_plan_.size() - 1', 'lineCost(', 'mapToWorld(')):
        raise ValueError('Unexpected heading score; refused partial patch')
    source = source[:start] + updated + source[end:]
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
    legacy_gate = ('      // MYAGV_TURN_FIRST: reject forward motion until the visible path is within 30 degrees.\n'
            '      if (i == 0 && vx_samp > 0.0 &&\n'
            '          headingDiff(cell_x, cell_y, x_i, y_i, theta_i) > M_PI / 6.0) {\n'
            '        traj.cost_ = -2.0; return;\n'
            '      }\n\n')
    gate = ('      // MYAGV_TURN_FIRST: use the measured-pose hysteresis for every forward candidate.\n'
            '      if (i == 0 && vx_samp > 0.0 && !myagv_following_) {\n'
            '        traj.cost_ = -2.0; return;\n'
            '      }\n\n')
    for previous_gate in (legacy_gate, legacy_gate.replace('30 degrees', '20 degrees').replace('M_PI / 6.0', 'M_PI / 9.0')):
        if source.count(previous_gate + heading + anchor) == 1:
            source = source.replace(previous_gate + heading + anchor, gate + heading + anchor)
    if source.count(gate + heading + anchor) != 1:
        if source.count(heading + anchor) != 1 or 'MYAGV_TURN_FIRST' in source:
            raise ValueError('Unexpected forward heading gate; refused partial patch')
        source = source.replace(heading + anchor, gate + heading + anchor)
    source = replace_once(source, '    for(int i = 0; i < vtheta_samples_; ++i) {',
        '    const bool keep_forward = best_traj->cost_ >= 0 && best_traj->xv_ > 0; // MYAGV_KEEP_FORWARD\n'
        '    for(int i = 0; !keep_forward && i < vtheta_samples_; ++i) {')
    fallback = "    //and finally, if we can't do anything else, we want to generate trajectories that move backwards slowly"
    disabled = ('    // MYAGV_NO_ESCAPE: zero escape speed means stop and replan, not a valid stationary trajectory.\n'
                '    if (backup_vel_ >= 0.0) {\n'
                '      escaping_ = false;\n'
                '      best_traj->cost_ = -1.0;\n'
                '      return *best_traj;\n'
                '    }\n\n')
    if source.count(disabled + fallback) != 1:
        if source.count(fallback) != 1 or 'MYAGV_NO_ESCAPE' in source:
            raise ValueError('Unexpected escape fallback; refused partial patch')
        source = source.replace(fallback, disabled + fallback)
    unsafe = 'if(best_traj->cost_ == -1.0)\n      best_traj->cost_ = 1.0;'
    safe = '// MYAGV_COLLISION_STOP: keep failed collision checks invalid.'
    if source.count(safe) == 1 and unsafe not in source:
        return source
    if source.count(unsafe) != 1:
        raise ValueError('Unexpected escape collision check; refused partial patch')
    return source.replace(unsafe, safe)


def patch_goal_stop(source):
    source = replace_once(source, '    global_plan_ = orig_global_plan;',
        '    tc_->updatePlan({}); // Reset tracking for a new/replanned goal, not rolling-window updates.\n'
        '    global_plan_ = orig_global_plan;')
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
    helper = Path(__file__).with_name('myagv_polyline.h').read_text()
    changes = []
    global_path = path.parents[2] / 'global_planner/src/planner_core.cpp'
    header = path.parents[1] / 'include/base_local_planner/trajectory_planner.h'
    for source, transform in ((path, patch), (path.with_name('trajectory_planner_ros.cpp'), patch_goal_stop),
                              (header, patch_header), (global_path, patch_global)):
        original = source.read_text()
        updated = transform(original)
        if updated != original:
            backup = source.with_suffix(source.suffix + '.before-shortest-turn-v6')
            if backup.exists():
                raise SystemExit('Backup exists; inspect previous installation first')
            changes.append((source, backup, original, updated))
    for source, backup, original, updated in changes:
        backup.write_text(original)
        source.write_text(updated)
    for directory in (path.parent, global_path.parent):
        (directory / 'myagv_polyline.h').write_text(helper)
    print('Navigation source ready; rebuild trajectory_planner_ros and global_planner, then restart navigation')
