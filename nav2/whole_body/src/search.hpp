// 【内容标注】用途：带朝向的整车绕障搜索。
// 对应用户需求：R13 R14 R23（原话及追溯边界见 nav2/CODE_GUIDE.md）。
// 添加/修改逻辑：Dijkstra 启发与前进运动原语/Dubins 连接；整段扫掠复核，预算耗尽拒绝返回假路线。
// 本次仅加注释；需求关联不是精确创建/提交记录。
// 基于 (x, y, yaw) 的前进绕障搜索；调用方需在搜索期间保持地图稳定。
#pragma once
#include "geometry.hpp"
#include <queue>
#include <unordered_map>
#include <chrono>
#include <ompl/base/spaces/DubinsStateSpace.h>
namespace visual_car_nav2 {
// 输入：带障碍/紫色的代价地图、完整车身、起终点、转弯半径、时间/节点预算。
// 输出：按行驶顺序排列的密集位姿；无已验证解时抛出异常，不回退到中心线。
inline std::vector<Pose> search(Collision &collision,Pose start,Pose goal,double turning,double seconds,int limit){
 if(!collision.free(start))throw std::runtime_error("Start footprint touches obstacle, buffer or map edge");
 if(!collision.free(goal))throw std::runtime_error("Goal footprint touches obstacle, buffer or map edge");
 auto &map=collision.map;const double r=map.getResolution();const int bins=72;
 // 连续位置保留在节点中；哈希键按地图单元和 72 个朝向桶去重。
 auto key=[&](Pose p){unsigned int x,y;if(!map.worldToMap(p.x,p.y,x,y))return int64_t(-1);int a=int(std::llround((wrap(p.a)+M_PI)/(2*M_PI)*bins))%bins;return (int64_t(y)*map.getSizeInCellsX()+x)*bins+a;};
 // 从终点反向运行二维八邻域 Dijkstra，为搜索提供绕障启发代价。
 // 它不考虑车宽，也不替代整车碰撞检查；不据此承诺连续空间的最短路径。
 int w=map.getSizeInCellsX(),h=map.getSizeInCellsY();std::vector<double> dist(w*h,1e30);
 using Cell=std::pair<double,int>;std::priority_queue<Cell,std::vector<Cell>,std::greater<Cell>> dq;
 unsigned int gx,gy;map.worldToMap(goal.x,goal.y,gx,gy);dist[gy*w+gx]=0;dq.push({0,int(gy*w+gx)});
 while(!dq.empty()){auto [d,i]=dq.top();dq.pop();if(d!=dist[i])continue;int x=i%w,y=i/w;
 for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){int nx=x+dx,ny=y+dy;if((!dx&&!dy)||nx<0||ny<0||nx>=w||ny>=h||map.getCost(nx,ny)>0)continue;
 double nd=d+std::hypot(dx,dy)*r;int j=ny*w+nx;if(nd<dist[j]){dist[j]=nd;dq.push({nd,j});}}}
 auto heuristic=[&](Pose p){unsigned int x,y;if(!map.worldToMap(p.x,p.y,x,y))return 1e30;return std::max(std::hypot(goal.x-p.x,goal.y-p.y),dist[y*w+x]);};
 // edge 保留父节点到本节点之间已检查过的全部样本，回溯时不能丢掉弯道。
 struct Node{Pose p;double g;int parent;std::vector<Pose> edge;};std::vector<Node> nodes{{start,0,-1,{start}}};
 using Entry=std::pair<double,int>;std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> open;
 std::unordered_map<int64_t,double> best;best[key(start)]=0;open.push({heuristic(start),0});
 // OMPL 只生成满足最小转弯半径的前进连接曲线；是否碰撞由本项目检查。
 ompl::base::DubinsStateSpace space(turning,false);
 auto *s=space.allocState(),*g=space.allocState(),*v=space.allocState();
 // RAII：成功返回或异常退出时均释放 OMPL 临时状态。
 struct Free{ompl::base::StateSpace &space;ompl::base::State *s,*g,*v;~Free(){space.freeState(s);space.freeState(g);space.freeState(v);}} guard{space,s,g,v};
 auto set=[](ompl::base::State *state,Pose p){auto *q=state->as<ompl::base::SE2StateSpace::StateType>();q->setX(p.x);q->setY(p.y);q->setYaw(p.a);};set(g,goal);
 auto begin=std::chrono::steady_clock::now();int expanded=0;
 // 原语至少跨越约一个格子的对角线，角度增量对齐朝向桶。
 const double angle=std::ceil((std::sqrt(2.)*r/turning)/(2*M_PI/bins))*(2*M_PI/bins);
 const double step=turning*angle;
 // 预算耗尽表示本次未找到已验证路线，不意味着物理空间一定绝对无路。
 // 时间检查在搜索循环每 64 次进行；不是整个服务调用的严格实时上限。
 while(!open.empty() && expanded++<limit){
 if(expanded%64==0 && std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()>seconds)throw std::runtime_error("Whole-body planning time limit; no verified path");
 int index=open.top().second;open.pop();Node current=nodes[index];if(current.g>best[key(current.p)]+1e-8)continue;
 // 接近终点时尝试短 Dubins 连接，沿线逐段通过 swept 才能接入终点。
 if(std::hypot(current.p.x-goal.x,current.p.y-goal.y)<1.5){
   set(s,current.p);double length=space.distance(s,g);
   if(length<2.0){int n=std::max(1,int(std::ceil(length/(r*.2))));std::vector<Pose> tail;Pose prev=current.p;bool valid=true;
     for(int i=1;i<=n;++i){space.interpolate(s,g,double(i)/n,v);auto *q=v->as<ompl::base::SE2StateSpace::StateType>();Pose p{q->getX(),q->getY(),q->getYaw()};
       // 保留每个已检查样本，不用未经检查的长直线替代弯曲路径。
       if(!collision.swept(prev,p)){valid=false;break;}tail.push_back(p);prev=p;}
     if(valid){std::vector<int> chain;for(int i=index;i>=0;i=nodes[i].parent)chain.push_back(i);std::reverse(chain.begin(),chain.end());std::vector<Pose> path;for(int i:chain)path.insert(path.end(),nodes[i].edge.begin(),nodes[i].edge.end());path.insert(path.end(),tail.begin(),tail.end());return path;}
   }
 }
 // 扩展右转、直行、左转三个前进原语；本实现不搜索倒车/原地旋转。
 for(int turn=-1;turn<=1;++turn){
   Pose prev=current.p,next=current.p;std::vector<Pose> edge;bool valid=true;int n=std::max(1,int(std::ceil(step/(r*.2))));
   for(int i=1;i<=n;++i){double d=step*i/n;
     if(turn==0)next={current.p.x+d*std::cos(current.p.a),current.p.y+d*std::sin(current.p.a),current.p.a};
     else {double a=current.p.a+turn*d/turning;next={current.p.x+turn*turning*(std::sin(a)-std::sin(current.p.a)),current.p.y-turn*turning*(std::cos(a)-std::cos(current.p.a)),wrap(a)};}
     if(!collision.swept(prev,next)){valid=false;break;}edge.push_back(next);prev=next;
   }
   if(!valid)continue;auto k=key(next);double cost=current.g+step*(turn?1.1:1.0);auto it=best.find(k);
   if(k<0 || (it!=best.end() && it->second<=cost))continue;
   // g 为路长加轻微转弯惩罚；优先队列按 g + 启发代价取下一节点。
   best[k]=cost;nodes.push_back({next,cost,index,std::move(edge)});open.push({cost+heuristic(next),int(nodes.size()-1)});
 }
 }
 throw std::runtime_error("No whole-body collision-free path within search limits");
}
}
