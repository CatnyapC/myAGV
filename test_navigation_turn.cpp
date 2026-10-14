// Offline regression: no ROS goals or velocity commands are published.
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/trajectory_planner.h>
#include <cassert>
#include <cmath>
#include <iostream>

int main() {
  ros::Time::init();
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
        for (int error : {-170, -135, -90, -45, 45, 90, 135, 170}) {
          // Match web_hardware.launch; meter_scoring scales distance biases by .05.
          base_local_planner::TrajectoryPlanner planner(
              world, map, footprint, .2, .2, .3, 1.5, .025, 8, 20,
              .6 * .05, .8 * .05, .01, .325, .05, .1, M_PI_2,
              false, .1, .02, .1, -.1, .03, -.02);
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
          auto trajectory = planner.findBestPath(pose, velocity, output);
          if (blocked) {
            assert(trajectory.cost_ < 0);
          } else {
            if (trajectory.thetav_ * error >= 0)
              std::cerr << "Wrong turn: bearing=" << bearing << " error=" << error
                        << " offset=" << offset << " wz=" << trajectory.thetav_ << '\n';
            assert(trajectory.cost_ >= 0 && trajectory.thetav_ * error < 0);
            assert(std::abs(trajectory.thetav_) <= .100001);
          }
        }
      }
    }
  }
  std::cout << "Shortest-turn and collision regressions passed\n";
}
