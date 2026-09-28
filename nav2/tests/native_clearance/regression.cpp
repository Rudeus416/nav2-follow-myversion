// 【内容标注】用途：导航离线回归：regression。
// 对应用户需求：R13 R14 R22（原话及追溯边界见 nav2/CODE_GUIDE.md）。
// 添加/修改逻辑：对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明。
// 本次仅加注释；需求关联不是精确创建/提交记录。
// Offline regression using the installed Nav2 collision checker and Hybrid A*.
// No ROS nodes, topics, actions or chassis commands are created.
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include "nav2_smac_planner/a_star.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"
using namespace nav2_smac_planner;
int main(int argc, char ** argv) {
  if (argc != 3) return 2; // radius, output path
  const double radius = std::stod(argv[1]), resolution = .05;
  nav2_costmap_2d::Costmap2D map(44, 84, resolution, 0, 0, 0);
  std::vector<std::pair<int,int>> obstacles;
  for (int y=0; y<84; ++y) for (int x=0; x<44; ++x)
    if (x==0 || x==43 || y==0 || y==83 || (y==40 && x>=21 && x<=26))
      obstacles.emplace_back(x,y);
  // Same exponential field as native InflationLayer.computeCost; .232 is the
  // padded footprint inscribed radius. No replacement planner is used here.
  auto cost=[](double distance)->unsigned char {
    if (distance==0) return 254;
    if (distance<=.232) return 253;
    return static_cast<unsigned char>(252*std::exp(-3*(distance-.232)));
  };
  for (int y=0; y<84; ++y) for (int x=0; x<44; ++x) {
    double d=100;
    for (auto [ox,oy]:obstacles) d=std::min(d,std::hypot(x-ox,y-oy)*resolution);
    map.setCost(x,y,d<=radius ? cost(d) : 0);
  }
  GridCollisionChecker checker(&map,72,nullptr);
  nav2_costmap_2d::Footprint footprint;
  for (auto [x,y]:std::vector<std::pair<double,double>>{{.297,.232},{.297,-.232},{-.297,-.232},{-.297,.232}}) {
    geometry_msgs::msg::Point p; p.x=x;p.y=y;footprint.push_back(p);
  }
  checker.setFootprint(footprint,false,cost(std::hypot(.297,.232)));
  // Pose just below bar: center free, rectangular body intersects the obstacle.
  bool collision=checker.inCollision(23.f,37.f,18.f,false);
  if (collision != (radius>0)) throw std::runtime_error("collision shortcut regression");
  SearchInfo info{};
  info.minimum_turning_radius=.4/resolution; info.non_straight_penalty=1.2;
  info.reverse_penalty=2; info.cost_penalty=3.5;info.retrospective_penalty=.015;
  info.analytic_expansion_ratio=3.5; info.analytic_expansion_max_length=1.6/resolution;
  AStarAlgorithm<NodeHybrid> planner(MotionModel::DUBIN,info);
  int max_iterations=1000000,iterations=0;
  planner.initialize(false,max_iterations,1000,2.,401,72);
  planner.setCollisionChecker(&checker);
  planner.setStart(23,7,18);planner.setGoal(23,72,18);
  NodeHybrid::CoordinateVector path;
  if (!planner.createPath(path,iterations,3.f)) throw std::runtime_error("no route around bar");
  std::ofstream out(argv[2]);
  for(auto p:path) out << (p.x+.5)*resolution << ' ' << (p.y+.5)*resolution << ' ' << p.theta*2*M_PI/72 << '\n';
  std::cout<<"radius="<<radius<<" overlapping_body_detected="<<collision<<" path_poses="<<path.size()<<" iterations="<<iterations<<std::endl;
}
