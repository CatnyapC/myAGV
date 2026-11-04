// Offline geometry regressions; no goals or motor commands are published.
#include "myagv_polyline.h"
#include <cassert>
#include <iostream>
#include <random>

geometry_msgs::PoseStamped pose(double x, double y) {
  geometry_msgs::PoseStamped p;
  p.pose.position.x = x; p.pose.position.y = y; p.pose.orientation.w = 1;
  return p;
}

int main() {
  costmap_2d::Costmap2D map(80, 80, .05, -1, -1, 0);
  myagv::Plan curved{pose(0,0), pose(.2,.1), pose(.5,.3), pose(.8,.1), pose(1,0)};
  curved.back().pose.orientation.z = .5;
  auto straight = myagv::simplify(map, curved);
  assert(straight.front().pose.position.x == 0 && straight.back().pose.position.x == 1);
  assert(straight.back().pose.orientation.z == .5);
  for (const auto& p : straight) assert(p.pose.position.y == 0);
  assert(myagv::segmentEnd(straight, .2, .02) == straight.size()-1);

  // An L-shaped path keeps its current corner until the chassis is close enough.
  myagv::Plan bent{pose(0,0), pose(.5,0), pose(1,0), pose(1,.5), pose(1,1)};
  assert(myagv::segmentEnd(bent, .2, .02) == 2);
  assert(myagv::segmentEnd(bent, .9, .02) == 4);
  assert(myagv::segmentEnd(bent, 1, .4) == 4);
  assert(myagv::simplify(map, {}).empty());
  assert(myagv::simplify(map, {pose(0,0)}).size() == 1);

  // Inflation, no-go/lethal and unknown cells all prevent shortcuts.
  for (unsigned char blocked : {costmap_2d::INSCRIBED_INFLATED_OBSTACLE,
                                costmap_2d::LETHAL_OBSTACLE, costmap_2d::NO_INFORMATION}) {
    map.resetMap(0, 0, 80, 80);
    for (unsigned y = 0; y < 34; ++y) map.setCost(40, y, blocked);
    assert(!myagv::clearSegment(map, pose(0,0).pose.position, pose(2,0).pose.position));
    auto around = myagv::simplify(map, {pose(0,0),pose(.5,1),pose(1.5,1),pose(2,0)});
    bool turned = false;
    for (size_t i = 1; i < around.size(); ++i) {
      if (!myagv::clearSegment(map, around[i-1].pose.position, around[i].pose.position)) {
        std::cerr << "Blocked simplified edge: " << around[i-1].pose.position.x << "," << around[i-1].pose.position.y
                  << " -> " << around[i].pose.position.x << "," << around[i].pose.position.y << '\n';
        assert(false);
      }
      turned = turned || around[i].pose.position.y > .5;
    }
    assert(turned);
  }
  map.resetMap(0, 0, 80, 80);
  map.setCost(21, 20, costmap_2d::LETHAL_OBSTACLE);
  assert(!myagv::clearSegment(map, pose(.025,.025).pose.position, pose(.075,.075).pose.position));
  assert(!myagv::clearSegment(map, pose(-2,0).pose.position, pose(0,0).pose.position));
  // Compare grid traversal with independent closed-rectangle intersection, including grid-line endpoints.
  costmap_2d::Costmap2D small(20, 20, .05, 0, 0, 0);
  std::mt19937 rng(7);
  std::vector<std::pair<int,int>> obstacles;
  for (int x=0; x<20; ++x) for (int y=0; y<20; ++y) if (rng()%9 == 0) {
    small.setCost(x,y,costmap_2d::LETHAL_OBSTACLE);
    obstacles.emplace_back(x,y);
  }
  for (int trial=0; trial<1000; ++trial) {
    auto a=pose((1+rng()%39)*.025,(1+rng()%39)*.025).pose.position;
    auto b=pose((1+rng()%39)*.025,(1+rng()%39)*.025).pose.position;
    bool clear=true;
    for (auto cell:obstacles) {
      double lo=0, hi=1;
      for (int axis=0; axis<2; ++axis) {
        double origin=axis?a.y:a.x, delta=axis?b.y-a.y:b.x-a.x;
        double edge=(axis?cell.second:cell.first)*.05;
        if (std::abs(delta)<1e-12) { if (origin<edge-1e-10 || origin>edge+.05+1e-10) hi=-1; }
        else {
          double t0=(edge-origin)/delta, t1=(edge+.05-origin)/delta;
          lo=std::max(lo,std::min(t0,t1)); hi=std::min(hi,std::max(t0,t1));
        }
      }
      if (lo<=hi+1e-10) { clear=false; break; }
    }
    assert(myagv::clearSegment(small,a,b)==clear);
  }
  std::cout << "Polyline shortcuts, corner tracking, inflation and boundary checks passed\n";
}
