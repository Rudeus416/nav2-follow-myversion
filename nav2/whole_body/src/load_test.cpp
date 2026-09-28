// 【内容标注】用途：整车插件离线验证。
// 对应用户需求：R13 R14 R22（原话及追溯边界见 nav2/CODE_GUIDE.md）。
// 添加/修改逻辑：验证几何/硬缓冲/插件加载相关回归；不发送真实小车运动指令。
// 本次仅加注释；需求关联不是精确创建/提交记录。
// 验证三个类可经 pluginlib 从本地 install 加载；不配置或启动导航节点。
#include <pluginlib/class_loader.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <dwb_core/trajectory_critic.hpp>
#include <iostream>
int main(){
 pluginlib::ClassLoader<nav2_core::GlobalPlanner> planners("nav2_core","nav2_core::GlobalPlanner");
 pluginlib::ClassLoader<nav2_costmap_2d::Layer> layers("nav2_costmap_2d","nav2_costmap_2d::Layer");
 pluginlib::ClassLoader<dwb_core::TrajectoryCritic> critics("dwb_core","dwb_core::TrajectoryCritic");
 auto p=planners.createSharedInstance("visual_car_nav2::Planner");
 auto l=layers.createSharedInstance("visual_car_nav2::HardBuffer");
 auto c=critics.createSharedInstance("visual_car_nav2::WholeBodyCritic");
 std::cout<<"PASS three local plugin types load without starting ROS or robot"<<std::endl;
}
