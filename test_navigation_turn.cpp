// Offline regression: no ROS goals or velocity commands are published.
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/trajectory_planner.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <cassert>
#include <cmath>
#include <iostream>

int main(int argc, char** argv) {
  ros::init(argc, argv, "myagv_navigation_regression", ros::init_options::AnonymousName);
  ros::Time::init();
  // The private stop helper must not feed measured overspeed back as another drive command.
  base_local_planner::TrajectoryPlannerROS wrapper;
  wrapper.dsrv_ = nullptr;  // The upstream default constructor leaves this unset until initialize().
  for (double vx : {-.2, 0., .02, .11, .3}) {
    for (double wz : {-.4, 0., .03, .3}) {
      geometry_msgs::PoseStamped pose, velocity;
      pose.pose.orientation.w = 1;
      velocity.pose.position.x = vx;
      velocity.pose.position.y = -.02;
      velocity.pose.orientation.z = sin(wz / 2);
      velocity.pose.orientation.w = cos(wz / 2);
      geometry_msgs::Twist command;
      command.linear.x = .09;
      command.linear.y = -.02;
      command.angular.z = -.3;
      assert(wrapper.stopWithAccLimits(pose, velocity, command));
      assert(command.linear.x == 0 && command.linear.y == 0 && command.angular.z == 0);
    }
  }
  costmap_2d::Costmap2D map(120, 120, .05, -3, -3, 0);
  base_local_planner::CostmapModel world(map);
  std::vector<geometry_msgs::Point> footprint;
  for (int i = 0; i < 16; ++i) {
    geometry_msgs::Point p;
    p.x = .2 * cos(i * 2 * M_PI / 16);
    p.y = .2 * sin(i * 2 * M_PI / 16);
    footprint.push_back(p);
  }
  for (bool blocked : {false, true}) {
    if (blocked) {
      for (unsigned int x = 0; x < map.getSizeInCellsX(); ++x)
        for (unsigned int y = 0; y < map.getSizeInCellsY(); ++y)
          map.setCost(x, y, costmap_2d::LETHAL_OBSTACLE);
    }
    for (double offset : {0., .025}) {
      for (double bearing : {0., M_PI_2, M_PI, -M_PI_2}) {
        for (int error : {-170, -135, -90, -45, -31, -29, -21, -19, 0, 19, 21, 29, 31, 45, 90, 135, 170}) {
          // Match web_hardware.launch; meter_scoring scales distance biases by .05.
          base_local_planner::TrajectoryPlanner planner(
              world, map, footprint, .2, .2, .3, 1.5, .025, 8, 20,
              .6 * .05, .8 * .05, .01, 0.0, .05, .1, M_PI_2,
              false, .05, .02, .1, -.1, .03, 0.0);
          std::vector<geometry_msgs::PoseStamped> plan;
          for (int i = 0; i <= 40; ++i) {
            geometry_msgs::PoseStamped p;
            p.pose.position.x = offset + i * .05 * cos(bearing);
            p.pose.position.y = offset + i * .05 * sin(bearing);
            p.pose.orientation.w = 1;
            plan.push_back(p);
          }
          planner.updatePlan(plan);
          geometry_msgs::PoseStamped pose, velocity, output;
          pose.pose.position.x = pose.pose.position.y = offset;
          double yaw = bearing + error * M_PI / 180;
          pose.pose.orientation.z = sin(yaw / 2);
          pose.pose.orientation.w = cos(yaw / 2);
          velocity.pose.orientation.w = 1;
          if (!blocked && error == 0) {
            // A disabled escape must request replanning, not silently keep a zero-speed goal alive.
            planner.escaping_ = true;
            planner.stuck_left = planner.stuck_right = true;
            auto failed = planner.findBestPath(pose, velocity, output);
            assert(failed.cost_ < 0 && !planner.escaping_);
            assert(output.pose.position.x == 0 && output.pose.position.y == 0);
            assert(output.pose.orientation.z == 0 && output.pose.orientation.w == 1);
            planner.stuck_left = planner.stuck_right = false;
          }
          auto trajectory = planner.findBestPath(pose, velocity, output);
          if (blocked) {
            assert(trajectory.cost_ < 0);
          } else {
            if (error != 0 && trajectory.thetav_ * error >= 0)
              std::cerr << "Wrong turn: bearing=" << bearing << " error=" << error
                        << " offset=" << offset << " wz=" << trajectory.thetav_ << '\n';
            assert(trajectory.cost_ >= 0);
            if (error != 0) assert(trajectory.thetav_ * error < 0);
            if (std::abs(error) > 30) assert(trajectory.xv_ == 0);
            else assert(planner.checkTrajectory(offset, offset, yaw, 0, 0, 0, .03, 0, 0));
            // Heading tolerance also applies while already translating.
            assert(planner.checkTrajectory(offset, offset, yaw, .08, 0, 0, .03, 0, 0)
                   == (std::abs(error) <= 30));
            assert(std::abs(trajectory.xv_) <= .050001);
            assert(std::abs(trajectory.thetav_) <= .100001);
          }
        }
      }
    }
  }
  // A virtual lookahead point can be blocked while the actual rotating footprint is clear.
  map.resetMap(0, 0, 120, 120);
  for (unsigned x = 0; x < 120; ++x)
    for (unsigned y = 0; y < 120; ++y) {
      double wx, wy;
      map.mapToWorld(x, y, wx, wy);
      if (std::hypot(wx + .23, wy + .23) < .1)
        map.setCost(x, y, costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    }
  for (double lookahead : {.325, 0.0}) {
    base_local_planner::TrajectoryPlanner planner(world, map, footprint,
        .2, .2, .3, 1.5, .025, 8, 20, .6*.05, .8*.05, .01,
        lookahead, .05, .1, M_PI_2, false, .05, .02, .1, -.1, .03, 0.0);
    std::vector<geometry_msgs::PoseStamped> plan;
    for (int i = 0; i <= 20; ++i) {
      geometry_msgs::PoseStamped p;
      p.pose.position.x = i * .05;
      p.pose.orientation.w = 1;
      plan.push_back(p);
    }
    planner.updatePlan(plan);
    geometry_msgs::PoseStamped pose, velocity, output;
    pose.pose.orientation.z = sin(-3*M_PI/8);
    pose.pose.orientation.w = cos(-3*M_PI/8);
    velocity.pose.orientation.w = 1;
    auto trajectory = planner.findBestPath(pose, velocity, output);
    assert(planner.checkTrajectory(0, 0, -3*M_PI/4, 0, 0, 0, 0, 0, .1));
    assert((trajectory.cost_ >= 0) == (lookahead == 0));
    if (lookahead == 0) assert(trajectory.xv_ == 0 && trajectory.thetav_ > 0);
  }
  std::cout << "Measured-stop, turn-first, shortest-turn, nearby-obstacle and collision regressions passed\n";
}
