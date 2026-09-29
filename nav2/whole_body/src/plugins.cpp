// 【内容标注】用途：自有 Nav2 插件实现。
// 对应用户需求：R13 R14 R23（原话及追溯边界见 nav2/CODE_GUIDE.md）。
// 添加/修改逻辑：HardBuffer 硬禁区、Planner 整车全局规划、WholeBodyCritic 实时轨迹评价。
// 本次仅加注释；需求关联不是精确创建/提交记录。
// Nav2 插件适配层：仅注册本项目自己的类，不修改系统 Nav2 源码。
#include "search.hpp"
#include <memory>
#include "nav2_core/global_planner.hpp"
#include "nav2_core/exceptions.hpp"
#include "dwb_core/trajectory_critic.hpp"
#include "dwb_core/exceptions.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "pluginlib/class_list_macros.hpp"
namespace visual_car_nav2 {
// Costmap2DROS 提供的是配置并添加 footprint_padding 后的车身轮廓。
// 【职责 / R13 R14】footprint：读取 Nav2 已包含 padding 的完整车身轮廓。
inline Polygon footprint(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap){Polygon out;for(auto p:costmap->getRobotFootprint())out.emplace_back(p.x,p.y);return out;}
// 【职责 / R13】pose：把 ROS 位姿转换为平面位置和偏航角。
inline Pose pose(const geometry_msgs::msg::Pose &p){auto q=p.orientation;return {p.position.x,p.position.y,std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))};}
// 插件一：可关闭、半径可为零的紫色硬禁区层，必须放在所有障碍层之后。
// 【职责 / R12 R14】HardBuffer：作为最后一层生成可关闭、可调半径的紫色硬禁区。
class HardBuffer: public nav2_costmap_2d::Layer {
 double radius_=0.;rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_;
public:
 // 【职责 / R12 R14】onInitialize：声明并校验参数，注册持地图锁的原子动态更新。
 void onInitialize() override {
   auto node=node_.lock();if(!node)throw std::runtime_error("Missing node");
   declareParameter("enabled",rclcpp::ParameterValue(true));declareParameter("inflation_radius",rclcpp::ParameterValue(0.));
   node->get_parameter(name_+".enabled",enabled_);node->get_parameter(name_+".inflation_radius",radius_);
   if(!std::isfinite(radius_)||radius_<0||radius_>1.5)throw std::runtime_error("Hard buffer radius must be 0..1.5m");
   // 动态参数与地图更新共用地图锁，避免更新半径时与地图线程交错。
   callback_=node->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &params){
     std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*layered_costmap_->getCostmap()->getMutex());
     auto answer=rcl_interfaces::msg::SetParametersResult();answer.successful=true;double radius=radius_;bool enabled=enabled_;
     for(auto &p:params){if(p.get_name()==name_+".inflation_radius"){
       if(p.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE || !std::isfinite(p.as_double())||p.as_double()<0||p.as_double()>1.5){answer.successful=false;answer.reason="radius must be 0..1.5m";return answer;}radius=p.as_double();
     }else if(p.get_name()==name_+".enabled"){
       if(p.get_type()!=rclcpp::ParameterType::PARAMETER_BOOL){answer.successful=false;return answer;}enabled=p.as_bool();}}
     // 参数改变后标记地图未更新，下一次 updateCosts 完成才恢复 current。
     if(radius_!=radius || enabled_!=enabled)current_=false;
     radius_=radius;enabled_=enabled;return answer;
   });current_=true;
 }
 // 即使半径缩小/开关关闭，也请求整图重建，防止旧紫色区域残留。
 // 【职责 / R12 R14】updateBounds：参数缩小或关闭时也请求整图重建，防旧紫色残留。
 void updateBounds(double,double,double,double *minx,double *miny,double *maxx,double *maxy)override{
   auto m=layered_costmap_->getCostmap();*minx=std::min(*minx,m->getOriginX());*miny=std::min(*miny,m->getOriginY());
   *maxx=std::max(*maxx,m->getOriginX()+m->getSizeInCellsX()*m->getResolution());*maxy=std::max(*maxy,m->getOriginY()+m->getSizeInCellsY()*m->getResolution());
 }
 // 【职责 / R12 R14】updateCosts：在所有障碍层之后应用硬缓冲并标记本轮更新完成。
 void updateCosts(nav2_costmap_2d::Costmap2D &m,int,int,int,int)override {if(enabled_)hardBuffer(m,radius_);current_=true;}
 // 【职责 / R12 R14】reset / isClearable：重置后等待重算，禁止外部清除破坏硬禁区语义。
 void reset()override{current_=false;}
 bool isClearable()override{return false;}
};
// 插件二：全局规划器。Nav2 的 ComputePathToPose/NavigateToPose 调用 createPlan。
// 【职责 / R13 R14】Planner：把整车朝向搜索接入 Nav2 GlobalPlanner 接口。
class Planner: public nav2_core::GlobalPlanner {
 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;rclcpp_lifecycle::LifecycleNode::SharedPtr node_;double turning_=.4,seconds_=3.;int limit_=150000;
public:
 // 【职责 / R13 R14】configure：保存节点/地图并读取转弯半径与搜索时限。
 void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,std::string name,std::shared_ptr<tf2_ros::Buffer>,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map)override{
   node_=parent.lock();map_=map;
   if(!node_->has_parameter(name+".minimum_turning_radius"))node_->declare_parameter(name+".minimum_turning_radius",.4);
   if(!node_->has_parameter(name+".max_planning_time"))node_->declare_parameter(name+".max_planning_time",3.);
   node_->get_parameter(name+".minimum_turning_radius",turning_);node_->get_parameter(name+".max_planning_time",seconds_);
   if(!std::isfinite(turning_)||turning_<.05||!std::isfinite(seconds_)||seconds_<=0)throw std::runtime_error("Invalid whole-body planner parameters");
 }
 // 【职责 / R13】activate / deactivate / cleanup：遵循插件生命周期并在清理时释放节点和地图。
 void activate()override{} void deactivate()override{} void cleanup()override{map_.reset();node_.reset();}
 // 【职责 / R13 R14】createPlan：锁定一致地图，运行整车搜索并输出包含朝向的完整路径。
 nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,const geometry_msgs::msg::PoseStamped &goal)override{
   if(start.header.frame_id!=map_->getGlobalFrameID()||goal.header.frame_id!=map_->getGlobalFrameID())throw nav2_core::PlannerException("Path frame mismatch");
   // 持地图锁获取一致的规划输入；起点、终点和地图必须同一坐标系。
   auto *m=map_->getCostmap();std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*m->getMutex());Collision collision(*m,footprint(map_));
   std::vector<Pose> points;try{points=search(collision,pose(start.pose),pose(goal.pose),turning_,seconds_,limit_);}catch(const std::exception &e){throw nav2_core::PlannerException(e.what());}
   // 每个样本同时输出位置和朝向，预览/执行都使用这一完整路径。
   nav_msgs::msg::Path path;path.header.frame_id=map_->getGlobalFrameID();path.header.stamp=node_->now();
   for(auto p:points){geometry_msgs::msg::PoseStamped out;out.header=path.header;out.pose.position.x=p.x;out.pose.position.y=p.y;out.pose.orientation.z=std::sin(p.a/2);out.pose.orientation.w=std::cos(p.a/2);path.poses.push_back(out);}return path;
 }
};
// 插件三：DWB 的硬碰撞否决器。必须放在 critics 列表第一项。
// 不生成速度，只检查原生 DWB 生成的候选轨迹；碰撞即抛出 IllegalTrajectory。
// 【职责 / R13 R14】WholeBodyCritic：否决 DWB 候选中任何单点或扫掠车身碰撞。
class WholeBodyCritic: public dwb_core::TrajectoryCritic {
 Pose start_{};Polygon foot_;std::unique_ptr<Collision> collision_;
public:
 // 【职责 / R13 R14】getScale：保持非零硬否决权重，避免配置权重绕过碰撞检查。
 double getScale() const override{return 1.;} // 固定非零 scale，防止通过调权重意外跳过整车碰撞检查。
 // 【职责 / R13 R14】prepare：每个控制周期重新取得车身和地图，并建立本周期占用索引。
 bool prepare(const geometry_msgs::msg::Pose2D &p,const nav_2d_msgs::msg::Twist2D &,const geometry_msgs::msg::Pose2D &,const nav_2d_msgs::msg::Path2D &)override{start_={p.x,p.y,p.theta};foot_=footprint(costmap_ros_);if(foot_.size()<3)return false;collision_=std::make_unique<Collision>(*costmap_ros_->getCostmap(),foot_);collision_->indexOccupancy();return true;}
 // 从当前位姿开始，逐段检查整个候选；安全轨迹返回 0，交给其他 critic 评分。
 // 【职责 / R13 R14】scoreTrajectory：从当前车位逐段检查候选轨迹，碰撞即抛出非法轨迹。
 double scoreTrajectory(const dwb_msgs::msg::Trajectory2D &traj)override{
   if(!collision_)throw dwb_core::IllegalTrajectoryException(name_,"Collision checker not prepared");
   auto &collision=*collision_;Pose prev=start_;
   if(!collision.free(prev)||traj.poses.empty())throw dwb_core::IllegalTrajectoryException(name_,"Whole body touches obstacle or hard buffer");
   for(auto p:traj.poses){Pose next{p.x,p.y,p.theta};if(!collision.swept(prev,next))throw dwb_core::IllegalTrajectoryException(name_,"Swept footprint touches obstacle or hard buffer");prev=next;}return 0.;
 }
};
}
// 对应 plugins.xml 中三个不同基类的导出项。
PLUGINLIB_EXPORT_CLASS(visual_car_nav2::Planner,nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(visual_car_nav2::HardBuffer,nav2_costmap_2d::Layer)
PLUGINLIB_EXPORT_CLASS(visual_car_nav2::WholeBodyCritic,dwb_core::TrajectoryCritic)
