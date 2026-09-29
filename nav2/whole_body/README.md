# 整车绕障路径规划代码说明

这是 visual_car 项目新增的 ROS 2 Humble / Nav2 扩展。它根据地图、完整车身轮廓、起点和终点，搜索可供整车通过的前进路线；并在局部控制阶段继续检查候选轨迹。源码含中文注释，没有修改系统 Nav2、相机模型或底盘驱动。

## 1. 本模块实现的规则

- 紫色半径为 **0** 或缓冲关闭：不生成紫色区，但车身仍须避开墙、障碍、手绘禁区和地图外区域。
- 紫色半径 **大于 0**：紫色格也属于硬禁区，整个车身（含 padding）均不能接触，而不是只检查中心线。
- 检查车身内部和边缘，也检查相邻位姿之间的平移、旋转扫掠。
- 搜索没有找到已验证解、起终点车身碰撞、坐标系不一致等情况均返回失败，不退回未检查的直线。
- 本模块依据地图判断可通行性；视觉/雷达如何识别实物和生成地图，不在本模块内。

当前配置车身长 0.494 m、宽 0.364 m，四周 padding 0.05 m；检查轮廓约为 0.594 × 0.464 m。这里须填写实际整车外廓，轴距/轮距不足以证明车壳和伸出附件被覆盖。关闭紫色不会关闭 padding。

## 2. 文件导航与建议阅读顺序

| 文件 | 作用 |
| --- | --- |
| `src/geometry.hpp` | 车身坐标变换、完整多边形碰撞、扫掠检查、紫色缓冲生成 |
| `src/search.hpp` | 带朝向的前进搜索、运动原语、Dubins 终点连接、完整路径回溯 |
| `src/plugins.cpp` | 将几何和搜索接入 Nav2 的三种插件接口 |
| `plugins.xml` | 三个类与 Nav2 基类的 pluginlib 注册 |
| `CMakeLists.txt`、`package.xml` | 本地 ROS 包构建及依赖 |
| `build.sh` | 编译、离线测试、安装到本目录、插件加载与图层测试 |
| `src/test.cpp` | 合成地图的完整车身与扫掠回归测试 |
| `src/load_test.cpp` | 三种插件的动态加载测试 |
| `src/layer_test.cpp` | 真实 LayeredCostmap 接口下的缓冲开关/缩小测试 |

交付包另外保留 `nav2/params.yaml`、`nav2/follow.launch.py`、`nav2/path_clearance.py`、`nav2/route_preview.py`、`nav2/point_navigation.py`、`nav2/inflation_control.py` 和 `frontend/nav2-inflation.js`，用于查看现有项目的接入方式。**这些接入文件不是完整网页/机器人项目，不能只靠此代码包启动原网页。** 原项目的 `motion`、`navigation`、地图快照、视觉服务、底盘和 ROS 话题仍由原项目提供。

## 3. 数据流程

```text
静态墙 / 手绘禁区 / 已开启的视觉或雷达障碍
                  ↓ 已合并的 costmap
          HardBuffer（必须是最后一层）
                  ↓ 添加可选的紫色硬禁区
   Planner → search → Collision.free / Collision.swept
                  ↓ nav_msgs/Path，包含位置与朝向
    停车预览及开始前 Python 完整车身复核（项目接入）
                  ↓ 用户明确开始行驶
             Nav2 FollowPath / DWB
                  ↓ 每条候选轨迹先检查
              WholeBodyCritic
                  ↓ 仅无碰撞候选交给其他 critic 评分
         原有控制门控 → 底盘速度接口
```

规划插件本身不发布底盘速度、不解除强停。DWB 生成候选速度，`WholeBodyCritic` 只负责否决碰撞轨迹；最终速度仍经过原项目的控制门控。

## 4. 几何检查怎么做

### `overlaps()`：完整格子与凸车身相交

使用分离轴定理（SAT），检查矩形格子的轴和车身各边法线。任一轴上投影分离则不碰撞，否则碰撞；边界接触也拒绝。检查的是格子的完整方块，不只检查格子中心或车身周边。

`Collision` 接收已包含 padding 的、按边界顺序排列的**凸多边形**。当前配置为矩形，不支持把凹形 footprint 原样当作凸形使用。

### `free()` / `freePolygon()`：单个位姿

把车身从底盘坐标系变换到地图坐标系，检查是否超出地图，再遍历包围盒内的非零代价格与车身是否相交。原始代价 252（紫色）、254（障碍）、255（未知）均被拒绝。

### `swept()`：两个位姿之间

根据位移、转角和车身外接圆半径细分，单步控制在约 1/4 地图格的运动尺度。取相邻车身轮廓凸包，加上旋转弓高和数值余量，检查中间扫过的区域。外接圆用于细分/余量计算，不会把整车碰撞形状替换为圆。

该函数检查相邻位姿之间的线性中心位置与最短角度插值；搜索返回密集样本，局部执行再次检查 DWB 的候选轨迹。实际轨迹跟踪误差仍须通过车身边距、定位精度和低速实测确认。

### `hardBuffer()`：紫色区域

从致命/未知单元做欧氏距离变换，在设定半径内把自由格写成代价 252。距离基于栅格中心，半径按地图分辨率离散。半径 0 不增加格子。

`HardBuffer.updateBounds()` 始终请求整图更新；底层地图重建后再添加当前半径，因此缩小到 0 或关闭后不会留下旧紫色格。不要对已经膨胀过的数组反复直接调用函数来模拟缩小，必须先恢复原始障碍底图。

## 5. 路径搜索怎么做

入口：

```cpp
std::vector<Pose> search(
    Collision &collision, Pose start, Pose goal,
    double turning, double seconds, int limit);
```

1. 检查起点和终点的完整车身；碰撞直接拒绝。
2. 从终点反向运行二维八邻域 Dijkstra，提供绕障启发代价。它只指导搜索顺序，不替代车身检查，也不保证连续空间的最短路径。
3. 搜索状态包含连续 `(x, y, yaw)`；按地图单元和 72 个朝向桶去重。
4. 扩展直行、左转、右转三个前进原语，沿原语逐段检查完整车身扫掠。转弯略加代价以减少无必要转弯。
5. 距终点较近时，用 OMPL 的 Dubins 曲线尝试连接；曲线仍逐段碰撞检查。
6. 沿父节点回溯，保留全部已检查样本，不能把弯道简化为未经检查的长直线。
7. 没有解或预算耗尽就抛出错误。

当前不搜索倒车或原地旋转；最小转弯半径默认 0.4 m，是待实测参数。规划预算默认 3 秒、最多 150000 次循环；时间每 64 次循环检查一次，且不包含之前的二维预计算，因此不是服务调用的严格实时期限。离散搜索/预算失败也不等于物理空间绝对没有路。

## 6. Nav2 插件配置

三个类必须配套使用：

| 扩展点 | 插件类 | 必要配置 |
| --- | --- | --- |
| 全局规划 | `visual_car_nav2::Planner` | `GridBased.plugin` |
| 地图缓冲 | `visual_car_nav2::HardBuffer` | 两张地图的 `inflation.plugin`，排列在所有障碍层之后 |
| 局部轨迹 | `visual_car_nav2::WholeBodyCritic` | `WholeBody.class`，`WholeBody` 放在 DWB critics 第一项 |

关键配置片段（完整项目示例见交付包 `nav2/params.yaml`）：

```yaml
# planner_server.ros__parameters
planner_plugins: [GridBased]
GridBased:
  plugin: visual_car_nav2::Planner
  minimum_turning_radius: 0.4
  max_planning_time: 3.0

# controller_server.ros__parameters.FollowPath
critics: [WholeBody, RotateToGoal, Oscillation, PathAlign, PathDist, GoalDist]
WholeBody.class: visual_car_nav2::WholeBodyCritic
WholeBody.scale: 1.0

# 两张 costmap 各自的 ros__parameters；保留原障碍层，在末尾加 inflation
footprint: '[[0.247,0.182],[0.247,-0.182],[-0.247,-0.182],[-0.247,0.182]]'
footprint_padding: 0.05
inflation:
  plugin: visual_car_nav2::HardBuffer
  enabled: true
  inflation_radius: 0.0
```

以上片段来自不同节点/层级，**不能原样当成一份完整 ROS 参数文件**。两张地图的 footprint、padding、缓冲参数和障碍来源要一致。原项目 `follow.launch.py` 负责按 radar/map/both 组合各层，并校验插件类型。

DWB 的 `WholeBodyCritic.getScale()` 固定返回 1，避免通过设置零权重关闭碰撞否决。合格轨迹返回 0，由其他 critic 选择更合适的跟踪速度；不合格轨迹抛出 `IllegalTrajectoryException`。

## 7. 编译与测试

要求：Linux、ROS 2 Humble、C++17、CMake/编译器，以及可发现的 `ament_cmake`、`nav2_core`、`nav2_costmap_2d`、`dwb_core`、`pluginlib`、OpenCV core/imgproc、OMPL 开发依赖。本机环境已验证；换机器需先准备这些依赖。

在原项目根目录或解压包根目录执行：

```bash
bash nav2/whole_body/build.sh
```

R41 构建环境修复：脚本在干净子进程中加载 `/opt/ros/humble`，固定系统编译器、Python、`fmt` 和 `spdlog`，禁用 CMake 用户包注册表及 Python 用户包目录。每次只重建自己的 `CMakeCache.txt` 和 `CMakeFiles/`；源文件和旧安装目录不会被整体删除，故障日志在下一次测试前保留。构建锁避免两个本脚本同时重写缓存。此环境隔离不改变当前终端或原视觉服务，也不要求退出 Conda。

若启动提示“先编译本项目整车导航插件”，而测试日志出现 `miniconda3/lib/libstdc++.so.6: GLIBCXX_3.4.30 not found`，原因是旧构建选中了 Conda 的 `fmt/spdlog` 并写入运行库路径，测试进程未能加载，安装步骤尚未执行。直接用更新后的 `bash nav2/whole_body/build.sh` 重新构建。不要跳过测试或替换系统/Conda 库来掩盖问题。

脚本会在 `nav2/whole_body/build/` 构建，安装到 `nav2/whole_body/install/`，不写系统目录。先做 C++ 几何/搜索测试，再做插件加载和图层测试。图层测试使用 `ROS_DOMAIN_ID=231`、localhost 和合成网格，没有底盘速度发布器。编译测试不会启动原网页或真实导航任务。

单独复跑几何测试：

```bash
source /opt/ros/humble/setup.bash
ctest --test-dir nav2/whole_body/build --output-on-failure
```

Python 预览校验回归（需要当前 Python 环境中的 numpy、opencv）：

```bash
PYTHONDONTWRITEBYTECODE=1 python -B -m unittest discover \
  -s nav2/tests -p 'test_whole_body_preview.py'
```

C++ 覆盖：0/0.10/0.25 m 缓冲下绕过横障碍、零缓冲窄通道、紫色使通道不再可通行、车身内部细障碍、平移跨障碍、转角扫掠、未知格拒绝。图层测试覆盖开启、关闭、缩小到零、重新开启和非法半径拒绝。合成空地图耗时打印不代表完整 Nav2 的实时性能承诺。

## 8. 现有项目的接入和网页控制

`follow.launch.py` 只为 Nav2 子进程增加本地 `install` 的 `AMENT_PREFIX_PATH` 和 `LD_LIBRARY_PATH`。在其他工程使用时，应把本包的安装目录加入该工程启动环境，例如先执行：

```bash
source /opt/ros/humble/setup.bash
source nav2/whole_body/install/local_setup.bash
```

随后由该工程自己的 Nav2 启动文件加载上述参数；本包没有独立的定位、传感器或底盘 bringup。

原项目网页强停后，通过 `GET/POST /api/nav2/inflation` 读取/设置 `enabled`、`radius`。后端分别写两张地图，再读回确认；失败保留导航拦截，成功清除旧预览。`frontend/nav2-inflation.js` 使用原网页已有 DOM，不是独立 HTML 页面。

`route_preview.py` 从 `view.snapshot()` 获取当前车身、位姿、静态图与局部代价图，交给 `path_clearance.py` 做整车复核；`point_navigation.py` 在开始前再次复核并检查地图版本、起点和预览时效，随后发送完整路径到 `FollowPath`。实际 Nav2 执行还会用最新局部地图检查 DWB 轨迹。

本次交付整理只增加注释/说明并打包，没有重新调整算法行为。新环境集成后应先停车预览，再在实际尺寸和定位已确认的条件下低速验证。


## 9. 人物终点接入

原项目新增 `nav2/person_navigation.py`：在用户已启用自动跟随的前提下，把选定人物的距离、方位通过 `nav2_follow.follow_goal()` 转换为 odom 下的接近终点，保留设置的跟随距离。随后显式调用本包 `GridBased` 规划器、共用 Python 完整车身复核，再以 `FollowPath` 执行同一完整路径。

它复用现有跟随 ID 和授权按钮，不改识别模型，不发送底盘速度，也不自动解除强停。规划与复核异步执行，完成后重新检查目标 ID、数据新鲜度、车位和运动授权；停止或失去目标会使旧结果失效。测试见 `nav2/tests/test_person_navigation.py`，完整操作说明见上级 `nav2/README.md` 的“人物作为目标的整车路线执行”。此前交付 ZIP 已按要求删除，本次没有重新生成。


## 10. 转弯执行与人物位置稳定性

导航限速由 `nav2/motion_geometry.py::limit_twist` 同比例处理线速度和角速度，保持路径曲率；`params.yaml` 中 DWB 上限同步为 0.18 m/s、0.4 rad/s。`person_position()` 将相机测量转换为 odom 中的人物位置，人物跟随据此滤波并判断移动，避免小车绕行引起停车点变化而误判目标移动。回归场景见 `nav2/tests/test_motion_geometry.py`。

相机视野约束尚未加入搜索。普通实时跟随仍要求人物可见；明确点击执行的已锁定段按第 13 节规则处理，不因人物暂时出画取消。整车碰撞与传感器检查独立保留。


## 11. 分段人物跟随

人物小幅移动时保持当前路径终点；成功到达后，清空旧位置滤波样本，重新确认到达后采集的 3 帧人物数据，再决定下一段终点。此状态逻辑位于 `nav2_follow.py` 与 `nav2/person_navigation.py`，不改变整车路径算法。普通实时跟随仍处理人物明显移动、丢失和超时；明确锁定段采用第 13 节规则，到达前不重新定位人物，障碍及传感器故障仍停车。


## 12. 人物占用区与显示稳定性

人物红色占用区保留。`motion_geometry.person_stopping_candidates()` 为人物停车点提供有限个接近侧备选位置；`person_navigation.py` 和 `nav2_follow.py` 的人物预览在规划失败后串行尝试，最终路线仍需完整车身复核，不修改本包碰撞判定或原生 Nav2。手动选点不使用备选位置。

人物掉帧时，`person_map.py` 的显示缓存仅保留源帧起 3 秒内的最后位置，前端淡色标注；显示缓存本身不授权运动；明确“记录人物点并规划”后，锁定段可按第 13 节单独执行。停止后灰色历史路线仅供排查，不能执行。候选共用 5 秒期限，选中 ID 改变、目标过期或强停会阻止执行。测试覆盖显示缓存隔离、候选不缩短跟随距离、不超过 3 米、重试期间停车和人工终点不被替换。


## 13. 连续追踪与固定分段

勾选连续追踪后，本段执行使用固定路径门控，途中不更新人物终点；到达回调解除本段门控，要求到达后新的 3 帧人物数据才计算下一段。1 米以内等待，离开后续段，结束追踪关闭授权。途中视觉近距障碍、整车碰撞及传感器保护保留，异常停止后锁住自动续段。R31 同时支持单次明确锁定路线：点击“记录人物点并规划 → 开始追踪”后，当前人物暂时不可见或已移动不影响同一路线的启动与执行；单次到达或失败后结束，连续成功到达才等待新的 3 帧续段。锁定点/ID/相机配置、30 秒预览、车位、整车碰撞和传感器仍须有效。原实时跟随入口不受此豁免影响，也不改变本包的路径搜索或碰撞规则。实现见 `nav2_follow.py`、`person_navigation.py`、`person_execution.py`，回归见 `tests/test_continuous_follow.py`。


## 14. 固定人物终点的遇障绕行（R32）

新增 `../obstacle_replan.py` 管理停车、等待旧动作终态、原终点重新规划及整车复核；`../replan_support.py` 接入新鲜全局地图、同时间戳车身轮廓和全量视觉候选；`../replan_safety.py` 检查完整车身及转角扫掠。绕行使用本包原 `GridBased` 和原紫色禁区规则，不修改 C++ 搜索/控制插件。

全量视觉点独立于小地图仅显示最近候选；实际输出速度还需通过采集延迟和制动行程内的整车扫掠检查。首次危险立即停车，只有新路线复核成功才允许沿安全方向绕开。取消或故障不会自动复活。单段最多 3 次；取消确认与规划复核各限 5 秒。完整操作及限制见上级 README 的“遇障停车后自动绕行原终点”。

## 自有代码内容标注与需求来源

参见 [代码用途、用户原话和修改逻辑总索引](../CODE_GUIDE.md)。源码中的 `【内容标注】` 与 `【职责 / Rxx】` 对应索引编号；本次只增加注释，不改变执行逻辑。无法确认的早期创建轮次不作归属推断。
