// 【内容标注】用途：整车插件离线验证。
// 对应用户需求：R13 R14 R22（原话及追溯边界见 nav2/CODE_GUIDE.md）。
// 添加/修改逻辑：验证几何/硬缓冲/插件加载相关回归；不发送真实小车运动指令。
// 本次仅加注释；需求关联不是精确创建/提交记录。
// 在隔离 DDS 测试域中使用合成障碍，验证动态关闭/缩小缓冲后不残留紫色。
// 测试没有任何底盘速度发布器；环境隔离参数见 build.sh。
#include <pluginlib/class_loader.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <iostream>
#include <stdexcept>
// This synthetic obstacle never subscribes or publishes any robot topic.
class Obstacle:public nav2_costmap_2d::Layer{
public:
 void onInitialize()override{enabled_=true;current_=true;}
 void updateBounds(double,double,double,double *x0,double *y0,double *x1,double *y1)override{*x0=0;*y0=0;*x1=2;*y1=2;}
 void updateCosts(nav2_costmap_2d::Costmap2D &m,int,int,int,int)override{m.setCost(20,20,254);}
 void reset()override{}bool isClearable()override{return false;}
};
void require(bool b,const char *message){if(!b)throw std::runtime_error(message);}
int main(int argc,char **argv){
 rclcpp::init(argc,argv);
 {
 auto node=std::make_shared<nav2_util::LifecycleNode>("whole_body_layer_test");
 auto group=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
 pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader("nav2_costmap_2d","nav2_costmap_2d::Layer");
 nav2_costmap_2d::LayeredCostmap layers("test_frame",false,false);layers.resizeMap(40,40,.05,0,0);
 auto obstacle=std::make_shared<Obstacle>();layers.addPlugin(obstacle);obstacle->initialize(&layers,"obstacle",nullptr,node,group);
 auto buffer=loader.createSharedInstance("visual_car_nav2::HardBuffer");layers.addPlugin(buffer);buffer->initialize(&layers,"inflation",nullptr,node,group);
 auto map=layers.getCostmap();
 auto set=[&](bool enabled,double radius){auto result=node->set_parameters_atomically({rclcpp::Parameter("inflation.enabled",enabled),rclcpp::Parameter("inflation.inflation_radius",radius)});require(result.successful,"parameter change rejected");layers.updateMap(1,1,0);require(map->getCost(20,20)==254,"real obstacle removed");};
 set(true,0.);require(map->getCost(22,20)==0,"zero radius adds purple");
 set(true,.2);require(map->getCost(22,20)==252,"radius fails to add purple");
 set(true,0.);require(map->getCost(22,20)==0,"zero radius leaves stale purple");
 set(true,.2);set(false,.2);require(map->getCost(22,20)==0,"disabled leaves stale purple");
 set(true,.2);require(map->getCost(22,20)==252,"re-enable fails");
 auto result=node->set_parameters_atomically({rclcpp::Parameter("inflation.inflation_radius",-1.)});require(!result.successful,"invalid radius accepted");
 layers.updateMap(1,1,0);require(map->getCost(22,20)==252,"invalid parameter changed map");
 std::cout<<"PASS layered costmap: zero, grow, shrink to zero, disable, enable, invalid radius"<<std::endl;
 }
 rclcpp::shutdown();
}
