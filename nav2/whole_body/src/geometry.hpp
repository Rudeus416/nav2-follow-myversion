// 【内容标注】用途：整车几何与硬缓冲基础。
// 对应用户需求：R13 R14 R23（原话及追溯边界见 nav2/CODE_GUIDE.md）。
// 添加/修改逻辑：SAT 多边形碰撞、轨迹扫掠、占用索引及硬膨胀；未知/障碍/紫色不得进入。
// 本次仅加注释；需求关联不是精确创建/提交记录。
// 整车几何检查：全局搜索和局部执行共用本文件，紫色为硬禁区。
#pragma once
#include <cmath>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <opencv2/imgproc.hpp>
#include "nav2_costmap_2d/costmap_2d.hpp"
namespace visual_car_nav2 {
// 地图坐标中的车身中心位姿：x/y 单位米，a 为偏航角（弧度）。
struct Pose {double x,y,a;};
// 【职责 / R13 R14】wrap：把任意偏航角归一到最短角差范围。
inline double wrap(double a){return std::atan2(std::sin(a),std::cos(a));}
using Polygon=std::vector<cv::Point2d>;
// 用分离轴定理判断凸多边形与完整栅格方块是否相交。
// 检查方块而非格子中心；边界接触也视作碰撞。margin 是额外的数值/扫掠余量。
// 【职责 / R13 R14】overlaps：用分离轴定理判断完整凸车身与完整栅格方块是否接触。
inline bool overlaps(const Polygon &p,double x,double y,double r,double margin) {
  std::vector<cv::Point2d> axes{{1,0},{0,1}};
  for(size_t i=0;i<p.size();++i){auto d=p[(i+1)%p.size()]-p[i];double n=std::hypot(d.x,d.y);if(n>1e-12)axes.push_back({-d.y/n,d.x/n});}
  for(auto axis:axes){double lo=1e30,hi=-1e30;for(auto v:p){double q=v.dot(axis);lo=std::min(lo,q);hi=std::max(hi,q);}
    double center=(x+r/2)*axis.x+(y+r/2)*axis.y,extent=r/2*(std::abs(axis.x)+std::abs(axis.y));
    if(hi+margin<center-extent || lo-margin>center+extent)return false;
  }return true;
}
// foot 必须是按边界顺序排列的凸车身轮廓，调用方传入已含 padding 的轮廓。
// radius 是轮廓外接圆半径，只用于采样和余量计算，不把矩形替换成圆。
// 【职责 / R13 R14】Collision：统一提供全局规划和局部控制使用的整车及扫掠碰撞规则。
class Collision {
public:
 nav2_costmap_2d::Costmap2D &map;Polygon foot;double radius=0;
 std::vector<unsigned int> occupied_prefix;
 // Only build for a DWB cycle while its costmap is locked; never reuse across cycles.
 // 【职责 / R13 R14】indexOccupancy：为同一 DWB 周期建立占用前缀和，仅加速可证明的空区域。
 void indexOccupancy(){
   unsigned w=map.getSizeInCellsX(),h=map.getSizeInCellsY();occupied_prefix.assign((w+1)*(h+1),0);
   for(unsigned y=0;y<h;++y){unsigned row=0;for(unsigned x=0;x<w;++x){
     row+=map.getCost(x,y)>0;occupied_prefix[(y+1)*(w+1)+x+1]=occupied_prefix[y*(w+1)+x+1]+row;
   }}
 }
 // 【职责 / R13 R14】emptyBounds：仅在严格位于图内且包围盒无任何占用时允许快速跳过。
 bool emptyBounds(double x0,double y0,double x1,double y1)const{
   if(occupied_prefix.empty())return false;
   double ox=map.getOriginX(),oy=map.getOriginY(),r=map.getResolution();
   int w=map.getSizeInCellsX(),h=map.getSizeInCellsY();
   if(x0<=ox||y0<=oy||x1>=ox+w*r||y1>=oy+h*r)return false;
   int l=std::max(0,int(std::floor((x0-ox)/r))-1),b=std::max(0,int(std::floor((y0-oy)/r))-1);
   int u=std::min(w,int(std::floor((x1-ox)/r))+1),t=std::min(h,int(std::floor((y1-oy)/r))+1);
   auto sum=[&](int x,int y){return occupied_prefix[y*(w+1)+x];};
   return sum(u,t)+sum(l,b)==sum(l,t)+sum(u,b);
 }

 // 【职责 / R13 R14】Collision 构造：校验凸轮廓并计算扫掠采样所需外接半径。
 Collision(nav2_costmap_2d::Costmap2D &m,Polygon f):map(m),foot(std::move(f)){
   if(foot.size()<3)throw std::runtime_error("Invalid footprint");
   for(auto v:foot){if(!std::isfinite(v.x)||!std::isfinite(v.y))throw std::runtime_error("Invalid footprint");radius=std::max(radius,std::hypot(v.x,v.y));}
 }
 // 将底盘坐标系的轮廓，按平移和朝向转换到代价地图坐标系。
 // 【职责 / R13 R14】polygon：把底盘轮廓按中心位姿转换到代价地图坐标。
 Polygon polygon(Pose p)const {Polygon out;double c=std::cos(p.a),s=std::sin(p.a);for(auto v:foot)out.push_back({p.x+c*v.x-s*v.y,p.y+s*v.x+c*v.y});return out;}
 // 包围盒先裁定图外，再检查区域内所有非零格与填充多边形的重叠。
 // 这能发现落在车身内部的细小障碍，而不仅仅检查四条边或中心点。
 // 【职责 / R13 R14】freePolygon：拒绝越界，并检查多边形与所有非零代价格的完整面积重叠。
 bool freePolygon(const Polygon &p,double margin=0)const {
   double minx=1e30,miny=1e30,maxx=-1e30,maxy=-1e30;
   for(auto v:p){if(!std::isfinite(v.x)||!std::isfinite(v.y))return false;minx=std::min(minx,v.x);miny=std::min(miny,v.y);maxx=std::max(maxx,v.x);maxy=std::max(maxy,v.y);}
   double ox=map.getOriginX(),oy=map.getOriginY(),r=map.getResolution();
   if(minx-margin<=ox || miny-margin<=oy || maxx+margin>=ox+map.getSizeInCellsX()*r || maxy+margin>=oy+map.getSizeInCellsY()*r)return false;
   int x0=std::max(0,int(std::floor((minx-margin-ox)/r))-1),y0=std::max(0,int(std::floor((miny-margin-oy)/r))-1);
   int x1=std::min(int(map.getSizeInCellsX())-1,int(std::floor((maxx+margin-ox)/r))),y1=std::min(int(map.getSizeInCellsY())-1,int(std::floor((maxy+margin-oy)/r)));
   for(int y=y0;y<=y1;++y)for(int x=x0;x<=x1;++x)
     if(map.getCost(x,y)>0 && overlaps(p,ox+x*r,oy+y*r,r,margin))return false;
   return true;
 }
 // 【职责 / R13 R14】free：检查单个位姿下整车轮廓是否可通行。
 bool free(Pose p)const{return freePolygon(polygon(p));}
 // 检查两个相邻路径位姿之间的平移与最短角度旋转。
 // 自适应细分后取相邻车身轮廓的凸包，覆盖中间位置；非有限输入拒绝。
 // 【职责 / R13 R14】swept：自适应细分相邻位姿并检查平移、转弯期间完整车身扫掠。
 bool swept(Pose a,Pose b)const {
   if(!std::isfinite(a.x)||!std::isfinite(a.y)||!std::isfinite(a.a)||!std::isfinite(b.x)||!std::isfinite(b.y)||!std::isfinite(b.a))return false;
   double da=wrap(b.a-a.a);int n=std::max(1,int(std::ceil((std::hypot(b.x-a.x,b.y-a.y)+radius*std::abs(da))/(map.getResolution()*.25))));
   if(n>20000)return false;
   // Entire swept body lies inside the center-segment box expanded by its radius.
   // Skip polygon allocation only if EVERY cell in that conservative box is zero.
   double bound=radius+map.getResolution()*.25+1e-5;
   if(emptyBounds(std::min(a.x,b.x)-bound,std::min(a.y,b.y)-bound,
                  std::max(a.x,b.x)+bound,std::max(a.y,b.y)+bound))return true;
   auto previous=polygon(a);
   for(int i=1;i<=n;++i){double t=double(i)/n;auto next=polygon({a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,a.a+da*t});
     std::vector<cv::Point2f> points;for(auto v:previous)points.emplace_back(v.x,v.y);for(auto v:next)points.emplace_back(v.x,v.y);
     std::vector<cv::Point2f> hull;cv::convexHull(points,hull);Polygon poly;for(auto v:hull)poly.emplace_back(v.x,v.y);
     // 旋转的圆弧可能突出端点凸包；加入弓高余量及浮点运算余量。
     double margin=radius*(1-std::cos(std::abs(da)/(2*n)))+1e-6;
     if(!freePolygon(poly,margin))return false;previous=std::move(next);
   }return true;
 }
};
// 最后一道地图层：从致命/未知单元计算欧氏距离，把半径内自由格设为 252。
// 252 映射到网页 OccupancyGrid 的紫色区；碰撞器将其作为禁入区。
// 半径为 0 不添加格子；旧紫色由 LayeredCostmap 重建底图时清除。
// 【职责 / R12 R14】hardBuffer：把致命/未知格的欧氏硬缓冲写成禁入代价；半径为零不新增格。
inline void hardBuffer(nav2_costmap_2d::Costmap2D &map,double radius){
 if(radius<=0)return;
 int w=map.getSizeInCellsX(),h=map.getSizeInCellsY();cv::Mat mask(h,w,CV_8U,cv::Scalar(1));bool any=false;
 for(int y=0;y<h;++y)for(int x=0;x<w;++x)if(map.getCost(x,y)>=254){mask.at<unsigned char>(y,x)=0;any=true;}
 if(!any)return;
 cv::Mat distance;cv::distanceTransform(mask,distance,cv::DIST_L2,cv::DIST_MASK_PRECISE);
 for(int y=0;y<h;++y)for(int x=0;x<w;++x)if(map.getCost(x,y)==0 && distance.at<float>(y,x)*map.getResolution()<=radius+1e-9)map.setCost(x,y,252);
}
}
