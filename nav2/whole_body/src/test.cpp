// 【内容标注】用途：整车插件离线验证。
// 对应用户需求：R13 R14 R22（原话及追溯边界见 nav2/CODE_GUIDE.md）。
// 添加/修改逻辑：验证几何/硬缓冲/插件加载相关回归；不发送真实小车运动指令。
// 需求关联用于用途追溯；本次增加长地图窗口回归，不修改搜索算法。
// 离线回归：合成地图中验证绕障、零缓冲窄通道、转角和未知区域拒绝。
// 不创建 ROS 节点，不发送速度；末尾耗时只代表合成空地图的几何检查。
#include "search.hpp"
#include <iostream>
void require(bool v,const char *s){if(!v)throw std::runtime_error(s);}
// 【回归 / R13 R20 R22】重现 4.78m 终点整车跨出旧 10m 窗口；
// 新全局窗口允许长图路线，静态墙/图外边界及绕障扫掠仍然有效。
void long_map_window_regression(){
 using namespace visual_car_nav2;
 Polygon foot{{.297,.232},{.297,-.232},{-.297,-.232},{-.297,.232}};
 Pose start{0,0,0},reported{4.78,.22,std::atan2(.22,4.78)};
 nav2_costmap_2d::Costmap2D local(200,200,.05,-4.95,-4.95,0);
 Collision old(local,foot);
 require(old.free(start),"old window start unexpectedly invalid");
 require(!old.free(reported),"reported endpoint must reproduce old local/global window failure");
 // 18 x 10 m is the map-mode global size for the current 8.4 x 2.2 m map.
 nav2_costmap_2d::Costmap2D global(360,200,.05,-8.95,-4.95,0);
 for(unsigned int y=0;y<global.getSizeInCellsY();++y)for(unsigned int x=0;x<global.getSizeInCellsX();++x){
   double wx,wy;global.mapToWorld(x,y,wx,wy);
   if(wx<-.3 || wx>8.1 || wy<-.9 || wy>1.2)global.setCost(x,y,254);
 }
 Collision clear(global,foot);
 auto check=[&](Collision &collision,Pose goal){
   auto path=search(collision,start,goal,.4,3.,150000);
   require(path.size()>2,"long map missing route");
   for(size_t i=1;i<path.size();++i)require(collision.swept(path[i-1],path[i]),"long map unsafe sweep");
   require(std::hypot(path.back().x-goal.x,path.back().y-goal.y)<1e-6,"long map did not reach requested goal");
   std::cout<<"PASS long-map goal="<<goal.x<<","<<goal.y<<" poses="<<path.size()<<std::endl;
 };
 check(clear,reported);
 require(!clear.free({8.,0,0}),"larger window must not erase the static map boundary");
 // A hand-drawn transverse obstacle forces a detour on a route beyond local coverage.
 for(unsigned int y=0;y<global.getSizeInCellsY();++y)for(unsigned int x=0;x<global.getSizeInCellsX();++x){
   double wx,wy;global.mapToWorld(x,y,wx,wy);
   if(wx>2.95 && wx<3.05 && wy>-.10 && wy<.45)global.setCost(x,y,254);
 }
 Collision blocked(global,foot);
 require(!blocked.free({3.,.2,0}),"far-route hand-drawn obstacle disappeared");
 check(blocked,{6.7,.22,std::atan2(.22,6.7)});
}

int main(){
 long_map_window_regression();
 using namespace visual_car_nav2;
 Polygon foot{{.297,.232},{.297,-.232},{-.297,-.232},{-.297,.232}};
 // 同一横向细障碍，在不同紫色半径下均验证返回路径的每一段扫掠。
 for(double radius:{0.,.10,.25}){
  nav2_costmap_2d::Costmap2D map(64,104,.05,0,0,0);
  for(int x=0;x<64;++x){map.setCost(x,0,254);map.setCost(x,103,254);}for(int y=0;y<104;++y){map.setCost(0,y,254);map.setCost(63,y,254);}
  for(int x=30;x<=36;++x)map.setCost(x,50,254);
  hardBuffer(map,radius);Collision c(map,foot);
  require(!c.free({1.65,2.4,M_PI/2}),"thin obstacle inside footprint missed");
  auto path=search(c,{1.625,.7,M_PI/2},{1.625,4.5,M_PI/2},.4,8.,150000);
  require(path.size()>2,"missing route");for(size_t i=1;i<path.size();++i)require(c.swept(path[i-1],path[i]),"returned unsafe segment");
  std::cout<<"PASS route radius="<<radius<<" poses="<<path.size()<<std::endl;
 }
 nav2_costmap_2d::Costmap2D map(40,40,.05,0,0,0);Collision c(map,foot);
 map.setCost(20,20,252);require(!c.free({1.,1.,0}),"purple must block full body");
 require(!c.swept({.5,1.,0},{1.6,1.,0}),"sweep skips thin obstacle");
 map.setCost(20,20,0);map.setCost(25,25,254);
 require(c.free({1.,1.,0})&&c.free({1.,1.,M_PI/2}),"rotation endpoints should be clear");
 require(!c.swept({1.,1.,0},{1.,1.,M_PI/2}),"rotation corner sweep missed obstacle");
 // A corridor wider than the padded car but narrower than car + purple must change feasibility.
 nav2_costmap_2d::Costmap2D narrow(30,80,.05,0,0,0);
 for(int y=0;y<80;++y){narrow.setCost(5,y,254);narrow.setCost(18,y,254);}
 Collision n(narrow,foot);
 auto path=search(n,{.60,.5,M_PI/2},{.60,3.5,M_PI/2},.4,3.,150000);
 require(path.size()>2,"zero buffer must allow a whole-body-clear narrow corridor");
 for(size_t i=1;i<path.size();++i)require(n.swept(path[i-1],path[i]),"narrow corridor collision");
 auto original=narrow;hardBuffer(narrow,0.);
 for(unsigned int y=0;y<80;++y)for(unsigned int x=0;x<30;++x)require(narrow.getCost(x,y)==original.getCost(x,y),"radius zero changed map");
 hardBuffer(narrow,.10);
 require(!n.free({.60,.5,M_PI/2}),"full body must not enter purple beside corridor");
 bool rejected=false;try{search(n,{.60,.5,M_PI/2},{.60,3.5,M_PI/2},.4,3.,150000);}catch(const std::runtime_error &){rejected=true;}
 require(rejected,"blocked narrow corridor was accepted");
 // No center-only checks: even a single unknown cell inside the body is blocking.
 map.setCost(25,25,0);map.setCost(20,20,255);require(!c.free({1.,1.,0}),"unknown cell accepted");
 std::cout<<"PASS zero-buffer narrow passage; full-body purple rejection; unknown rejection"<<std::endl;
 nav2_costmap_2d::Costmap2D open_map(200,200,.05,-5,-5,0);Collision open_collision(open_map,foot);
 // Indexed shortcut must agree with the original checker near walls/unknown.
 Collision indexed(map,foot);indexed.indexOccupancy();
 for(int x=6;x<32;++x)for(int y=6;y<32;++y){
   Pose a{x*.05,y*.05,.3},b{x*.05+.04,y*.05+.03,.5};
   require(c.swept(a,b)==indexed.swept(a,b),"indexed sweep differs from exact checker");
 }
 open_collision.indexOccupancy();
 auto began=std::chrono::steady_clock::now();
 for(int v=0;v<15;++v)for(int a=0;a<20;++a){Pose previous{0,0,0};for(int i=1;i<=40;++i){double t=i*.05;double yaw=(a-9.5)/19.*t;Pose next{v*.3/14*t,0,yaw};require(open_collision.swept(previous,next),"clear candidate rejected");previous=next;}}
 std::cout<<"BENCH 300 local candidates / 40 segments: "<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count()<<" ms"<<std::endl;
 std::cout<<"PASS filled footprint, purple veto, translation and rotation sweep"<<std::endl;
}
