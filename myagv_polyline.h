#ifndef MYAGV_POLYLINE_H
#define MYAGV_POLYLINE_H
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <geometry_msgs/PoseStamped.h>

namespace myagv {
using Plan = std::vector<geometry_msgs::PoseStamped>;

// Supercover traversal checks both cells at diagonal crossings; inflated cells stay blocked.
inline bool clearSegment(const costmap_2d::Costmap2D& map,
                         const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
  unsigned ax, ay, bx, by;
  if (!map.worldToMap(a.x, a.y, ax, ay) || !map.worldToMap(b.x, b.y, bx, by)) return false;
  auto clear = [&](int x, int y) {
    return x >= 0 && y >= 0 && x < int(map.getSizeInCellsX()) && y < int(map.getSizeInCellsY())
        && map.getCost(x, y) < costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  };
  double gx = (a.x-map.getOriginX())/map.getResolution(), gy = (a.y-map.getOriginY())/map.getResolution();
  double dx = (b.x-a.x)/map.getResolution(), dy = (b.y-a.y)/map.getResolution();
  auto endpoint_clear = [&](double px, double py) {
    bool edge_x = std::abs(px-std::round(px)) < 1e-9, edge_y = std::abs(py-std::round(py)) < 1e-9;
    int cx = edge_x ? std::round(px) : std::floor(px), cy = edge_y ? std::round(py) : std::floor(py);
    return clear(cx, cy) && (!edge_x || clear(cx-1, cy)) && (!edge_y || clear(cx, cy-1))
        && (!(edge_x && edge_y) || clear(cx-1, cy-1));
  };
  if (!endpoint_clear(gx, gy) || !endpoint_clear(gx+dx, gy+dy)) return false;
  int x = ax, y = ay, sx = (dx > 0)-(dx < 0), sy = (dy > 0)-(dy < 0);
  double inf = std::numeric_limits<double>::infinity();
  double tx = sx ? (x+(sx > 0)-gx)/dx : inf, ty = sy ? (y+(sy > 0)-gy)/dy : inf;
  double step_x = sx ? std::abs(1/dx) : inf, step_y = sy ? std::abs(1/dy) : inf;
  for (;;) {
    if (!clear(x, y)) return false;
    if (!sy && std::abs(gy-std::round(gy)) < 1e-9 && !clear(x, y-1)) return false;
    if (!sx && std::abs(gx-std::round(gx)) < 1e-9 && !clear(x-1, y)) return false;
    if (x == int(bx) && y == int(by)) return true;
    if (std::min(tx, ty) >= 1-1e-9) return true;
    if (std::abs(tx-ty) < 1e-9) {
      if (!clear(x+sx, y) || !clear(x, y+sy)) return false;
      x += sx; y += sy; tx += step_x; ty += step_y;
    } else if (tx < ty) { x += sx; tx += step_x; }
    else { y += sy; ty += step_y; }
  }
}

inline Plan simplify(const costmap_2d::Costmap2D& map, const Plan& plan) {
  if (plan.size() < 3) return plan;
  Plan result{plan.front()};
  for (size_t i = 0; i+1 < plan.size();) {
    size_t next = plan.size()-1;
    while (next > i+1 && !clearSegment(map, plan[i].pose.position, plan[next].pose.position)) --next;
    const auto& a = plan[i].pose.position;
    const auto& b = plan[next].pose.position;
    int steps = std::max(1, int(std::ceil(std::hypot(b.x-a.x, b.y-a.y)/map.getResolution())));
    // Keep dense samples for ROS distance fields, with exact straight segments and corners.
    for (int step = 1; step < steps; ++step) {
      auto p = plan[i];
      p.pose.position.x = a.x+(b.x-a.x)*step/steps;
      p.pose.position.y = a.y+(b.y-a.y)*step/steps;
      result.push_back(p);
    }
    result.push_back(plan[next]);
    i = next;
  }
  return result;
}

inline size_t segmentEnd(const Plan& plan, double x, double y) {
  if (plan.empty()) return 0;
  size_t nearest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i+1 < plan.size(); ++i) {
    const auto& a = plan[i].pose.position;
    const auto& b = plan[i+1].pose.position;
    double dx = b.x-a.x, dy = b.y-a.y, length2 = dx*dx+dy*dy;
    if (length2 < 1e-12) continue;
    double t = std::max(0., std::min(1., ((x-a.x)*dx+(y-a.y)*dy)/length2));
    double distance = std::hypot(x-a.x-t*dx, y-a.y-t*dy);
    if (distance < best) { best = distance; nearest = i; }
  }
  for (;;) {
    size_t end = std::min(nearest+1, plan.size()-1);
    const auto& a = plan[nearest].pose.position;
    const auto& b = plan[end].pose.position;
    double direction = std::atan2(b.y-a.y, b.x-a.x);
    while (end+1 < plan.size()) {
      const auto& p = plan[end].pose.position;
      const auto& q = plan[end+1].pose.position;
      double angle = std::atan2(q.y-p.y, q.x-p.x)-direction;
      if (std::hypot(q.x-p.x, q.y-p.y) > 1e-6 &&
          std::abs(std::atan2(std::sin(angle), std::cos(angle))) > M_PI/180.) break;
      ++end;
    }
    const auto& corner = plan[end].pose.position;
    if (end+1 == plan.size() || std::hypot(corner.x-x, corner.y-y) > .15) return end;
    nearest = end;
  }
}
}
#endif
