# 自有导航代码用途与需求索引

本索引记录当前代码与本会话用户指令的对应关系。**不是逐次提交的审计记录**：仅凭当前文件和对话，无法证明每个函数最早由哪一轮创建，也无法完整恢复历次中间实现。需求原话可确认，早期创建轮次未确认；表中“添加/修改逻辑”描述当前最终实现，不虚构历史 diff。需求编号是本次人工整理的索引，不是平台消息 ID。

R24 标注轮次只插入注释和文档，不改变默认开关、参数、代码语句、接口或运行行为。后续 R25 自检的行为修复与尚存限制见 [自检报告](AUDIT.md)。原注释和原函数文档字符串保留；注释中的“安全/可通过”指当前模型及检查条件下的判断，不等于实车验证。 R42 再次执行全量注释审计，补齐后续 R35～R41 模块的函数职责和本文件遗漏的文件索引；本轮同样不修改可执行语句。公开对话、诊断依据、撤回项和限制见 [对话、公开分析与修改审计记录](CONVERSATION_AUDIT.md)。

## 用户指令索引

<a id="r01"></a>

### R01：地图模式

> 添加，并且设置为雷达模式和地图模式可以选择开启，可以单独开启一个也可以同步开启

<a id="r02"></a>

### R02：地图展示

> 地图可不可以在网页前端呈现，需要标注的有地图，小车当前位置，小车预计路线

<a id="r03"></a>

### R03：手绘禁区

> 可不可以在网页添加一个现场绘制大概地图的功能，不止矩形，跟随鼠标或者手指绘画

<a id="r04"></a>

### R04：地图外边界

> 不是你这样添加，就在上传的地图四周添加红色区域就行了

<a id="r05"></a>

### R05：虚拟墙

> 黄色虚拟墙不需要新增一个视频口，合并在红色虚拟墙的那个窗口里面

<a id="r06"></a>

### R06：虚拟墙按需刷新

> 虚拟墙改成绘制时更新，没更新虚拟墙或者小车没动时不需要一直更新虚拟墙

<a id="r07"></a>

### R07：视觉障碍选择

> 视觉探测障碍物改为选择开启，在网页端添加可选项

<a id="r08"></a>

### R08：最近视觉障碍

> 先修复墙体，然后调整视觉障碍，只选取最近的可能障碍物，当然所有的改动都在nav2的导航里

<a id="r09"></a>

### R09：语义检测

> 你没有改成任何东西，你看看yolo模型有没有判定方法

<a id="r10"></a>

### R10：视觉位置校验

> 显示错误，真正的最近遮挡物和标注并不一样

<a id="r11"></a>

### R11：选点行驶

> 预览路径终点之后可不可以开始跑预计路径

<a id="r12"></a>

### R12：紫色缓冲

> 我就是需要彻底关闭它，并且添加一个滑动来改变紫色区域大小的功能

<a id="r13"></a>

### R13：整车通行

> 你查看一下nav2，能不能实现的是，还是可以把紫色膨胀区改为0，但是我需要的是小车计算一个可以整车安全通过障碍物和虚拟障碍区，达到终点的功能

<a id="r14"></a>

### R14：整车避让紫色区

> 同时，如果增加紫色膨胀区，计算的就是小车全身不碰到紫色膨胀区，安全通过障碍物以及虚拟障碍区

<a id="r15"></a>

### R15：人物标点

> 已选中的人物改成橙色三角形

<a id="r16"></a>

### R16：历史人物点

> 看到目标后，点击规划路径后并未产生路径，你添加的历史人物点呢，在点击追踪人物的产生路径之后，立马在最近人物闪烁处生成一个历史最近人物点，往那里规划

<a id="r17"></a>

### R17：人物路线执行

> 成功规划，在追踪人物的规划旁边添加一个开始追踪，让小车行动

<a id="r18"></a>

### R18：连续追踪

> 接下来增加连续追踪人物模式，即在完成第一次追踪到过往路径，在目标点附近后，立刻定位人物位置，生成下一个位置，直到人物位置距离小车很近不需要继续移动，比如在1m范围内。人物一旦离开1m范围继续追踪，直到结束追踪模式。注意在追踪的时候除非有突然障碍物出现，那么立刻停车，其他时候不要重新定位并计算新路径

<a id="r19"></a>

### R19：近距保护和平顺性

> 优化你的近距离障碍物监测机制，同时我还需要你优化小车行动的丝滑度，并告诉我你怎么做的

<a id="r20"></a>

### R20：地图加长

> 生成一张之前地图两倍长度的新地图并替换之前的旧地图

<a id="r21"></a>

### R21：启动失败排查

> 修改你写的nav2文件，查找你写的代码的bug，依旧不要去修改代码库，只能修改你自己写的代码

<a id="r22"></a>

### R22：性能与故障回归

> 小车又不动了，在修改一个bug时注意是否与其他的代码冲突，不要因为修改一个bug产生新的bug

<a id="r23"></a>

### R23：源码说明

> 把你刚刚写的计算小车绕路路径的代码整合发给我，并配上readme和代码间注释

<a id="r24"></a>

### R24：本次内容标注

> 接下来给所有你写的代码做上内容标注，哪里是干什么的，是我的哪一个命令产生的，修改或者添加逻辑是什么

<a id="r25"></a>

### R25：全链路冲突检查与离线模拟

> 检查你的所有已写项目，检查是否存在冲突，模拟小车运行时是否会存在bug或者检测问题导致小车无法追踪，无法行动等问题

范围、已修复冲突、未修复的配置限制、测试命令见 [自检报告](AUDIT.md)。

<a id="r26"></a>

### R26：修复远端终点超出规划窗口

> 修复

上下文：用户的选点 `(4.78,0.22)` 报 `Goal footprint touches obstacle, buffer or map edge`，已查询确认为规划窗口截断完整车身。此轮同步修复全局覆盖范围与整路线快照复核；见 [README](README.md) 和 [后续自检记录](AUDIT.md)。

<a id="r27"></a>

### R27：自有导航面板的画面集中与操作分组

> 优化你写的代码的前端，视频输出放在一起，按键有序放好，不要杂乱成一团

> 撤回修改，不要修改别人的前端，你只用优化你自己写的代码的

按后续范围要求撤回全页工作台、固定强停栏及独立页面入口，继续使用原首页。布局模块只整理原 `.obstacle-map-panel` 内的自有导航控件：集中红黄虚拟墙和疑似障碍预览，分组人物追踪/地图选点操作，收纳参数。原实时相机画面、强停和旧页面其他区域保留原位。

模块移动原导航控件节点并保留 ID、表单、事件与禁用/隐藏状态；CSS 仅作用于导航面板。不修改原 HTML、通用样式、app.js、相机服务或导航执行逻辑。

<a id="r28"></a>

### R28：地图上下平移

> 给nav2地图的缩放功能增加一个地图上下滑动的功能

在自有地图工具栏增加上下滑块与箭头按钮，查看时支持鼠标/单指拖图。所有地图图层共用平移变换，选点与绘图使用同一逆变换；保留原有笔画边缘夹取。缩放/轮询保留视图偏移，重置恢复居中；选点和绘图手势优先。只调整自有导航显示，不修改地图数据或控制接口。

<a id="r29"></a>

### R29：左右平移与电脑显示确认

> 再加一个左右移动，另外，我现在只能在手机端看到修改，电脑网页端看不到

在上下控件旁增加独立的左右滑块和箭头按钮，查看时支持双轴及斜向拖动。绘制、图层裁剪和选点反算共享同一中心；两轴共用禁用、范围与重置规则，不产生控制请求。电脑显示问题已由用户确认“电脑看到了”，未因此改动原页面或相机服务。

<a id="r30"></a>

### R30：人物规划/启动无反应的错误反馈

> 检测刚刚报错原因并修正

用户确认操作为“记录人物点并规划 / 开始追踪”，表现为“启用开始追踪后无反应”。日志可证实人物接口返回 409，但旧日志没有保存 detail，不能断言具体由哪个条件拒绝。修复可证实的界面与诊断问题：前端直接显示共享前置检查结果；按钮请求中有反馈；操作错误不被轮询覆盖；后台保留原 HTTP 状态并记录拒绝原因。整车碰撞、时效、视觉开关和原子授权检查保持原顺序与标准。

<a id="r31"></a>

### R31：按已经锁定的人物位置执行本段

> 怎么会显示人物位置不新鲜呢，我之前的代码不是确定上次最新锁定位置开始行动吗，你把我的代码删了吗

历史人物点和预览路线没有删除；旧执行端同时要求实时人物新鲜且未离开参考点，R30 把该检查提前显示，仍与“先沿锁定路线行驶”的要求冲突。现在明确点击执行时只使用已锁定点和对应有效路线，不再由当前人物缺测/移动拒绝。仍检查同一 ID/相机配置、序号、30 秒预览、起点、完整路径和传感器；运行中单次及连续锁定段都不因人物变化重规划。单次终态不自动重启；连续成功到达后才读取新帧续段。原实时跟随入口保留自己的新鲜度规则。

<a id="r32"></a>

### R32：遇到新障碍后，保留原人物终点自动绕行

> 差什么补什么，但是只可以修改你写的nav2代码

承接已确认链路：锁定最近人物点并执行；遇障先停车，能绕行则保持原终点重新规划，不能绕行则停；连续模式到达后才重新定位人物，1 米内等待，取消后结束。新增状态机串行等待旧动作终态、同一终点规划、全地图/整车/全量视觉复核及实际速度扫掠；保留授权、传感器和取消检查，禁止迟到回调复活。单段 3 次上限及取消/规划超时防止持续抖动重试。首次近距保护仍立即停车，仅成功复核的新路线使用方向性整车门控。本轮不改原控制、相机、前端或安装库。

<a id="r33"></a>

### R33：历史点不限 3 秒，排查手动选点途中停止

> 1.小车不追踪目标，历史记录点不需要有3s时间，2.我手动选取目标点为什么又不跑了，之前的代码是可以跑起来的，这次又停在路中央了，你修改了什么

历史位置与短时实时图标分开保存：显式规划可用同一 ID/相机流的最近有效位置，不设年龄期限，不伪造新采集时间。连续下一段仍使用到达后的新观测。日志确认手动 FollowPath 约 23.5 秒后由接入层“里程计 TF 超时”取消；修复导航回调与 TF 共用单线程、等待导航锁会阻塞 TF 的结构耦合。自有独立 TF 接收线程保持原 0.5 秒门槛，新增源时间差诊断；绕行轮廓读取避开整图/编辑锁，重复、过期、关闭帧不重复读取。未声称旧日志能证明底盘 TF 源正常；不更改原控制/相机/前端/安装库。

<a id="r34"></a>

### R34：已接受人物路线却在起步后立即取消

> 小车依然无法启动，找出原因并修改

日志证实 FollowPath 被接受后 0.651 秒，因视觉障碍数据超时被接入层取消。`replan_support.monitor_route` 的通用异常处理吞掉了原本应短暂停车等待的 `VisionStale`。新增类型分流并复用原 1.2 秒零速/源年龄 2 秒上限；速度扫掠同样处理计算期间过期，恢复前重查完整证据并等待新指令。同帧同路径不反复计算整条路线，但每次仍检查时效。历史点规则、真实障碍、TF、授权、取消保护不放宽。仅修改自有导航代码及测试/文档。

## 文件与当前逻辑

每个源码文件顶部有同样的需求编号；主要函数前用 `【职责 / Rxx】` 标明职责。测试文件按测试模块标注，详细断言见测试名称和测试体。C++ 几何与搜索文件保留原有逐段算法注释，再补需求关联。

| 文件 | 干什么 | 对应需求 | 添加或修改的逻辑 |
| --- | --- | --- | --- |
| [frontend/nav2-panel.js](../frontend/nav2-panel.js) | 自有导航面板布局与预览分区 | [R27](#r27) [R28](#r28) [R29](#r29) | 仅整理原导航面板内的预览、操作和设置；保留状态门控，不移动原实时视频/强停/运动面板，不主动发送控制请求；收纳地图平移控件 |
| [frontend/nav2-panel.css](../frontend/nav2-panel.css) | 自有导航面板响应式与触控样式 | [R27](#r27) [R28](#r28) [R29](#r29) | 限定导航面板作用域；复选框同行、按钮换行、手机布局和等比地图，不改变旧页面其他区域；新增上下/左右平移控件与手势样式 |
| [frontend/nav2-inflation.js](../frontend/nav2-inflation.js) | 紫色缓冲开关与半径滑块 | [R12](#r12) [R14](#r14) | 区分未应用设置与后端确认状态，强停时提交半径并更新提示 |
| [frontend/nav2-vision-toggle.js](../frontend/nav2-vision-toggle.js) | 视觉障碍选择与估距校验界面 | [R07](#r07) [R10](#r10) | 开关视觉层、显示诊断、提交停车图像选点和实测距离校验 |
| [frontend/obstacle-map.js](../frontend/obstacle-map.js) | 导航小地图与选点/人物操作面板 | [R02](#r02) [R03](#r03) [R11](#r11) [R15](#r15) [R16](#r16) [R17](#r17) [R18](#r18) [R27](#r27) [R28](#r28) [R29](#r29) [R30](#r30) [R31](#r31) | 绘制地图、路线和人物；编辑禁区；锁定历史点；启动/结束连续追踪；加载面板内布局模块，不直接发布速度；同步平移投影、选点和绘画逆变换 |
| [frontend/route-preview.js](../frontend/route-preview.js) | 停车视觉疑似障碍预览面板 | [R08](#r08) | 停车时拉取预览；切换状态后丢弃旧请求和图像 URL |
| [frontend/virtual-wall.js](../frontend/virtual-wall.js) | 同窗红黄虚拟墙网页显示 | [R05](#r05) [R06](#r06) | 墙体/车位或设置改变时刷新；请求代际丢弃过期响应；释放图像资源 |
| [nav2/boundary_map.py](boundary_map.py) | 单地图模式的外侧禁区边框 | [R04](#r04) | 在原图四周外侧增加一格占用，并按地图朝向平移原点，保留内部净空间 |
| [nav2/clearance_worker.py](clearance_worker.py) | 隔离进程中的整车路径复核 | [R13](#r13) [R14](#r14) [R22](#r22) | 限制并发与超时，避免 Python 几何计算阻塞视觉和控制线程；失败不放行 |
| [nav2/continuous_segment.py](continuous_segment.py) | 连续追踪与原暂停入口的兼容适配 | [R18](#r18) [R22](#r22) [R25](#r25) [R31](#r31) [R32](#r32) | 只对已授权、已复核的连续或显式锁定段豁免人物掉帧取消；保留传感器检查，关闭时恢复原方法 |
| [nav2/tests/check_frontend_layout.py](tests/check_frontend_layout.py) | 自有导航面板离线浏览器回归 | [R27](#r27) [R28](#r28) [R29](#r29) [R30](#r30) [R31](#r31) | 本地模拟全部接口，校验桌面/手机布局、面板外原节点位置、画笔选点、预览、按钮单次请求与强停禁用状态，不连接实车；验证鼠标/触摸平移及坐标、图层一致性 |
| [nav2/tests/run_regressions.py](tests/run_regressions.py) | 集中离线回归入口 | [R22](#r22) [R25](#r25) | 汇集自有导航及已有导航接入测试，不启动真实控制 |
| [nav2/tests/test_continuous_integration.py](tests/test_continuous_integration.py) | 原暂停方法与连续段集成回归 | [R18](#r18) [R25](#r25) [R31](#r31) | 只读抽取原 MotionManager 方法，验证掉帧、强停、ID 变更和传感器故障 |
| [nav2/tests/test_locked_person_segment.py](tests/test_locked_person_segment.py) | 锁定历史人物路线运行回归 | [R31](#r31) | 只读抽取原控制入口，验证人物缺测/移动不改线、ID/流/障碍故障停车、单次终态结束、连续到达新帧续段及迟到清理不覆盖新授权 |
| [nav2/tests/test_execution_watchdog.py](tests/test_execution_watchdog.py) | 异步执行故障回归 | [R22](#r22) [R25](#r25) | 模拟无应答、迟到接受、取消失败、终态丢失及立即完成 |
| [nav2/tests/test_map_configuration_limits.py](tests/test_map_configuration_limits.py) | 真实地图配置限制复现 | [R13](#r13) [R20](#r20) [R25](#r25) | 加载当前地图、起点和车身；起点余量限制仍复现，远端窗口用例已改为验证全局修复 |
| [nav2/tests/test_continuous_goal_tolerance.py](tests/test_continuous_goal_tolerance.py) | 连续跟随到达容差限制复现 | [R18](#r18) [R25](#r25) | 模拟人物 1.10 米时短段被判到达后重复规划；记录当前限制 |
| [nav2/depth_mailbox.py](depth_mailbox.py) | 最新深度结果的轻量读取桥 | [R22](#r22) | 保留原回调，额外保存最新结果引用；按流/配置过滤，关闭时恢复回调 |
| [nav2/follow.launch.py](follow.launch.py) | 自有 Nav2 节点与插件启动装配 | [R01](#r01) [R13](#r13) [R21](#r21) | 选择雷达/地图/叠加模式；加载自有插件；配置静态图、坐标变换和生命周期启动 |
| [nav2/follow.xml](follow.xml) | 单终点行为树 | [R11](#r11) [R18](#r18) | 组织计算路线与跟随路线的动作；参数和 Python 接入共同决定使用方式 |
| [nav2/follow_through_poses.xml](follow_through_poses.xml) | 多位姿行为树 | [R11](#r11) [R13](#r13) | 组织多位姿路线规划和执行；当前是否调用取决于所用导航动作 |
| [nav2/inflation_control.py](inflation_control.py) | 紫色硬缓冲参数网页控制 | [R12](#r12) [R14](#r14) [R21](#r21) | 强停下写两张代价地图的参数并读回；不确定状态阻止执行，成功使旧预览失效 |
| [nav2/map_window.py](map_window.py) | 按真实地图扩大全局规划窗口 | [R20](#r20) [R26](#r26) | 计算旋转后整图跨度，map/both 自动扩大 global 并发布完整快照；不改 local、原点或占用 |
| [nav2/tests/test_map_window.py](tests/test_map_window.py) | 全局范围与模式隔离回归 | [R26](#r26) | 覆盖真实长图、旋转地图、大配置、坏输入与资源上限 |
| [nav2/tests/test_global_costmap_snapshot.py](tests/test_global_costmap_snapshot.py) | 独立全局快照回归 | [R26](#r26) | 全量/增量校验、时效和写时复制；默认网页不携带全局大栅格 |
| [nav2/tests/test_route_global_validation.py](tests/test_route_global_validation.py) | 长路线整车复核回归 | [R13](#r13) [R26](#r26) | 同一终点旧局部窗失败、完整全局通过；远端障碍/未知/紫色/缺图/图外继续拒绝 |
| [nav2/map_startup.py](map_startup.py) | 静态地图启动就绪检查 | [R01](#r01) [R21](#r21) | 等待地图相关服务和节点就绪，失败退出以阻止不完整导航启动 |
| [nav2/maps/corridor.yaml](maps/corridor.yaml) | 默认手绘走廊地图元数据 | [R01](#r01) [R20](#r20) | 保留分辨率/原点与默认图片路径；对应 2.1m 净宽、8.4m 长地图 |
| [nav2/motion_geometry.py](motion_geometry.py) | 人物世界坐标、停车候选和速度几何 | [R13](#r13) [R16](#r16) [R18](#r18) [R19](#r19) | 相机测量转 odom；生成保留跟随距离的停车点；同步缩放速度保持曲率，增加加速过渡 |
| [nav2/near_obstacle.py](near_obstacle.py) | 近距障碍停车与恢复判定 | [R19](#r19) | 保留 0.70m 最低停车距离并增加延迟/速度提前量；危险立即进入，恢复需余量和三帧 |
| [nav2/params.yaml](params.yaml) | Nav2 规划器、控制器与代价地图参数 | [R01](#r01) [R13](#r13) [R14](#r14) [R19](#r19) [R22](#r22) | 配置自有整车规划/评价插件、车身轮廓、速度/进展检查和障碍图层 |
| [nav2/path_clearance.py](path_clearance.py) | Python 完整车身碰撞校验 | [R13](#r13) [R14](#r14) | 检查完整多边形及沿路径的扫掠区域，拒绝障碍、紫色区、未知区和越界 |
| [nav2/person_execution.py](person_execution.py) | 开始追踪按钮的执行事务 | [R17](#r17) [R18](#r18) [R30](#r30) [R31](#r31) | 停车复核锁定点/ID/序号/时效/车位后原子授权，发送同一路径；失败恢复停车 |
| [nav2/person_map.py](person_map.py) | 人物图标、历史点和人物网页接口 | [R15](#r15) [R16](#r16) [R17](#r17) [R18](#r18) [R30](#r30) [R31](#r31) | 同帧人物与历史 TF 定位；短时缓存和点击锁定点分离；接入规划、启动、结束按钮 |
| [nav2/obstacle_replan.py](obstacle_replan.py) | 固定终点遇障重规划状态机 | [R32](#r32) | 等待旧动作终态、原终点规划/复核、授权与回调代际；无路/取消/超时不重启 |
| [nav2/pose_listener.py](pose_listener.py) | 独立导航 TF 接收与清理 | [R33](#r33) | 专用节点/执行线程，避免导航锁等待堵住 TF；原定位时效门槛不变 |
| [nav2/tests/test_point_mode_isolation.py](tests/test_point_mode_isolation.py) | 手动选点执行隔离回归 | [R33](#r33) | 原人物暂停入口、视觉短时等待、TF 超时与障碍停车；不依赖人物绕行数据 |
| [nav2/tests/test_pose_listener.py](tests/test_pose_listener.py) | TF 调度隔离回归 | [R33](#r33) | 接收与控制回调隔离、退出与资源清理；不启动车辆 |
| [nav2/replan_support.py](replan_support.py) | 地图、完整视觉点与控制接入 | [R32](#r32) | 全局图须在遇障后更新；轮廓使用自身时间戳 TF；锁外剩余路线检查与实际命令门控 |
| [nav2/replan_safety.py](replan_safety.py) | 全量视觉整车扫掠几何 | [R32](#r32) | 5 cm 完整占用格、原 padding 车身与旋转扫掠；制动/延迟模型内的实际速度检查 |
| [nav2/tests/test_obstacle_replan.py](tests/test_obstacle_replan.py) | 遇障状态机离线回归 | [R32](#r32) | 同终点、取消终态、迟到回调、单次/连续衔接与失败后禁止自动重启 |
| [nav2/tests/test_replan_visual_pause.py](tests/test_replan_visual_pause.py) | 人物路线视觉等待集成回归 | [R34](#r34) | 真实自有状态机、模拟 ROS 动作：短暂过期保留任务、新指令恢复、持续超时/TF/障碍/取消、计算期间过期与同帧去重 |
| [nav2/tests/test_replan_support.py](tests/test_replan_support.py) | 传感器/地图接入回归 | [R32](#r32) | 全量证据、地图更新、轮廓同帧 TF、锁外几何、实际命令与路线进度 |
| [nav2/tests/test_replan_safety.py](tests/test_replan_safety.py) | 整车视觉扫掠回归 | [R32](#r32) | 细杆、转角、全量/图外点、制动时效及与原车身校验器差分 |
| [nav2/person_navigation.py](person_navigation.py) | 人物路线规划、复核和执行状态机 | [R13](#r13) [R16](#r16) [R17](#r17) [R18](#r18) [R31](#r31) [R32](#r32) | 有限候选串行规划，复核同一路径后 FollowPath；处理取消/超时；连续模式固定本段 |
| [nav2/point_navigation.py](point_navigation.py) | 人工地图选点的执行接口 | [R11](#r11) [R13](#r13) [R22](#r22) | 复核地图版本、起点、路线及视觉时效，再执行既有预览；不依赖人物跟踪 |
| [nav2/route_preview.py](route_preview.py) | 选点规划与视觉预览接口集合 | [R02](#r02) [R08](#r08) [R11](#r11) [R13](#r13) [R32](#r32) | 接入共用整车校验器、人物接口和人工选点；生成停车视觉预览候选 |
| [nav2/semantic_obstacles.py](semantic_obstacles.py) | 独立语义物体检测工作进程 | [R09](#r09) [R08](#r08) | 提取 YOLO 实例区域辅助最近候选选择；独立模型/进程，不替换原人物检测链路 |
| [nav2/startup_watchdog.py](startup_watchdog.py) | Nav2 启动进度监测 | [R21](#r21) | 观察启动进度并限制无进展等待，输出失败诊断；具体首次新增轮次无法从对话确认 |
| [nav2/tests/native_clearance/CMakeLists.txt](tests/native_clearance/CMakeLists.txt) | 导航离线回归：CMakeLists | [R13](#r13) [R14](#r14) [R22](#r22) | 对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明 |
| [nav2/tests/native_clearance/check_paths.py](tests/native_clearance/check_paths.py) | 导航离线回归：check_paths | [R13](#r13) [R14](#r14) [R22](#r22) | 对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明 |
| [nav2/tests/native_clearance/regression.cpp](tests/native_clearance/regression.cpp) | 导航离线回归：regression | [R13](#r13) [R14](#r14) [R22](#r22) | 对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明 |
| [nav2/tests/test_clearance_worker.py](tests/test_clearance_worker.py) | 导航离线回归：clearance_worker | [R13](#r13) [R22](#r22) | 对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明 |
| [nav2/tests/test_continuous_follow.py](tests/test_continuous_follow.py) | 导航离线回归：continuous_follow | [R18](#r18) | 对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明 |
| [nav2/tests/test_depth_mailbox.py](tests/test_depth_mailbox.py) | 导航离线回归：depth_mailbox | [R22](#r22) | 对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明 |
| [nav2/tests/test_motion_geometry.py](tests/test_motion_geometry.py) | 导航离线回归：motion_geometry | [R13](#r13) [R18](#r18) [R19](#r19) | 对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明 |
| [nav2/tests/test_near_and_ramp.py](tests/test_near_and_ramp.py) | 导航离线回归：near_and_ramp | [R19](#r19) | 对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明 |
| [nav2/tests/test_person_execution.py](tests/test_person_execution.py) | 导航离线回归：person_execution | [R17](#r17) [R18](#r18) [R30](#r30) [R31](#r31) | 对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明 |
| [nav2/tests/test_person_map.py](tests/test_person_map.py) | 导航离线回归：person_map | [R15](#r15) [R16](#r16) [R30](#r30) [R31](#r31) | 对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明 |
| [nav2/tests/test_person_navigation.py](tests/test_person_navigation.py) | 导航离线回归：person_navigation | [R16](#r16) [R17](#r17) [R18](#r18) | 对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明 |
| [nav2/tests/test_point_start_idle_stop.py](tests/test_point_start_idle_stop.py) | 导航离线回归：point_start_idle_stop | [R11](#r11) [R22](#r22) | 对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明 |
| [nav2/tests/test_safety_config.py](tests/test_safety_config.py) | 导航离线回归：safety_config | [R13](#r13) [R14](#r14) | 对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明 |
| [nav2/tests/test_semantic_obstacles.py](tests/test_semantic_obstacles.py) | 导航离线回归：semantic_obstacles | [R09](#r09) | 对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明 |
| [nav2/tests/test_vision_calibration.py](tests/test_vision_calibration.py) | 导航离线回归：vision_calibration | [R10](#r10) | 对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明 |
| [nav2/tests/test_vision_filtering.py](tests/test_vision_filtering.py) | 导航离线回归：vision_filtering | [R08](#r08) | 对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明 |
| [nav2/tests/test_vision_freshness.py](tests/test_vision_freshness.py) | 导航离线回归：vision_freshness | [R22](#r22) | 对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明 |
| [nav2/tests/test_vision_scheduler.py](tests/test_vision_scheduler.py) | 导航离线回归：vision_scheduler | [R22](#r22) | 对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明 |
| [nav2/tests/test_whole_body_preview.py](tests/test_whole_body_preview.py) | 导航离线回归：whole_body_preview | [R13](#r13) [R14](#r14) | 对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明 |
| [nav2/virtual_wall.py](virtual_wall.py) | 红黄虚拟墙相机叠加 | [R05](#r05) [R06](#r06) | 投影手绘边界和视觉候选到同一相机预览；保持原相机视频功能独立 |
| [nav2/vision_calibration.py](vision_calibration.py) | 视觉估距比例的停车校验接口 | [R10](#r10) | 选图像点与实测距离校验深度比例；仅导航视觉路径使用，不宣称单目距离绝对准确 |
| [nav2/vision_layer.py](vision_layer.py) | 视觉障碍候选与代价地图生成 | [R08](#r08) [R09](#r09) [R10](#r10) [R19](#r19) [R22](#r22) [R32](#r32) | 串行读取深度；变换/筛选最近候选；合并静态墙；独立使用全部候选做近距停车 |
| [nav2/vision_toggle.py](vision_toggle.py) | 视觉障碍层网页开关与状态 | [R07](#r07) [R22](#r22) | 独立开关视觉参与导航；管理视觉工作线程、数据时效和诊断快照 |
| [nav2/whole_body/CMakeLists.txt](whole_body/CMakeLists.txt) | 自有整车插件构建与注册 | [R13](#r13) [R23](#r23) | 声明依赖、构建/安装或插件注册；具体见本文件指令，不改系统 Nav2 源码 |
| [nav2/whole_body/build.sh](whole_body/build.sh) | 隔离构建、测试与安装自有整车插件 | [R13](#r13) [R23](#r23) [R41](#r41) | 在干净子进程固定系统 ROS/C++/fmt/spdlog，清除自有污染缓存；CTest、插件加载和隔离图层通过后才安装 |
| [nav2/whole_body/package.xml](whole_body/package.xml) | 自有整车插件构建与注册 | [R13](#r13) [R23](#r23) | 声明依赖、构建/安装或插件注册；具体见本文件指令，不改系统 Nav2 源码 |
| [nav2/whole_body/plugins.xml](whole_body/plugins.xml) | 自有整车插件构建与注册 | [R13](#r13) [R23](#r23) | 声明依赖、构建/安装或插件注册；具体见本文件指令，不改系统 Nav2 源码 |
| [nav2/whole_body/src/geometry.hpp](whole_body/src/geometry.hpp) | 整车几何与硬缓冲基础 | [R13](#r13) [R14](#r14) [R23](#r23) | SAT 多边形碰撞、轨迹扫掠、占用索引及硬膨胀；未知/障碍/紫色不得进入 |
| [nav2/whole_body/src/layer_test.cpp](whole_body/src/layer_test.cpp) | 整车插件离线验证 | [R13](#r13) [R14](#r14) [R22](#r22) | 验证几何/硬缓冲/插件加载相关回归；不发送真实小车运动指令 |
| [nav2/whole_body/src/load_test.cpp](whole_body/src/load_test.cpp) | 整车插件离线验证 | [R13](#r13) [R14](#r14) [R22](#r22) | 验证几何/硬缓冲/插件加载相关回归；不发送真实小车运动指令 |
| [nav2/whole_body/src/plugins.cpp](whole_body/src/plugins.cpp) | 自有 Nav2 插件实现 | [R13](#r13) [R14](#r14) [R23](#r23) | HardBuffer 硬禁区、Planner 整车全局规划、WholeBodyCritic 实时轨迹评价 |
| [nav2/whole_body/src/search.hpp](whole_body/src/search.hpp) | 带朝向的整车绕障搜索 | [R13](#r13) [R14](#r14) [R23](#r23) | Dijkstra 启发与前进运动原语/Dubins 连接；整段扫掠复核，预算耗尽拒绝返回假路线 |
| [nav2/whole_body/src/test.cpp](whole_body/src/test.cpp) | 整车插件离线验证 | [R13](#r13) [R14](#r14) [R22](#r22) | 验证几何/硬缓冲/插件加载相关回归；不发送真实小车运动指令 |
| [nav2_follow.py](../nav2_follow.py) | Nav2 与原运动控制的导航接入层 | [R11](#r11) [R13](#r13) [R16](#r16) [R17](#r17) [R18](#r18) [R19](#r19) [R22](#r22) [R31](#r31) [R32](#r32) | 分离规划/复核/执行；管理取消代际、传感器时效、连续分段目标和保曲率速度输出；不另建底盘发布通道 |
| [nav2_map_edit.py](../nav2_map_edit.py) | 手绘禁区存储与静态地图合成 | [R03](#r03) [R04](#r04) | 校验矩形/笔画及地图版本，把禁区叠加到原地图，支持撤销和清除 |
| [nav2_map_publisher.py](../nav2_map_publisher.py) | 编辑地图的 ROS 发布桥 | [R03](#r03) [R04](#r04) | 接收底图并重载手绘禁区，发布导航使用的编辑后地图 |
| [obstacle_view.py](../obstacle_view.py) | 网页所需地图与机器人图层快照 | [R02](#r02) [R03](#r03) [R22](#r22) | 订阅地图/轮廓/雷达/路径并转换坐标，按数据时效组织只读快照，接入手绘编辑 |
| [tests/test_nav2_follow.py](../tests/test_nav2_follow.py) | 导航相关已有回归：test_nav2_follow | [R02](#r02) [R03](#r03) [R08](#r08) [R11](#r11) [R13](#r13) [R22](#r22) | 与导航功能相关的离线回归；关联这些需求不表示整份测试最初都由本轮创建 |
| [tests/test_nav2_map_edit.py](../tests/test_nav2_map_edit.py) | 导航相关已有回归：test_nav2_map_edit | [R02](#r02) [R03](#r03) [R08](#r08) [R11](#r11) [R13](#r13) [R22](#r22) | 与导航功能相关的离线回归；关联这些需求不表示整份测试最初都由本轮创建 |
| [tests/test_nav2_vision_pause.py](../tests/test_nav2_vision_pause.py) | 导航相关已有回归：test_nav2_vision_pause | [R02](#r02) [R03](#r03) [R08](#r08) [R11](#r11) [R13](#r13) [R22](#r22) | 与导航功能相关的离线回归；关联这些需求不表示整份测试最初都由本轮创建 |
| [tests/test_nav2_wall_merge.py](../tests/test_nav2_wall_merge.py) | 导航相关已有回归：test_nav2_wall_merge | [R02](#r02) [R03](#r03) [R08](#r08) [R11](#r11) [R13](#r13) [R22](#r22) | 与导航功能相关的离线回归；关联这些需求不表示整份测试最初都由本轮创建 |
| [tests/test_obstacle_view.py](../tests/test_obstacle_view.py) | 导航相关已有回归：test_obstacle_view | [R02](#r02) [R03](#r03) [R08](#r08) [R11](#r11) [R13](#r13) [R22](#r22) | 与导航功能相关的离线回归；关联这些需求不表示整份测试最初都由本轮创建 |
| [tests/test_route_preview.py](../tests/test_route_preview.py) | 导航相关已有回归：test_route_preview | [R02](#r02) [R03](#r03) [R08](#r08) [R11](#r11) [R13](#r13) [R22](#r22) | 与导航功能相关的离线回归；关联这些需求不表示整份测试最初都由本轮创建 |
| [tests/test_virtual_wall.py](../tests/test_virtual_wall.py) | 导航相关已有回归：test_virtual_wall | [R02](#r02) [R03](#r03) [R08](#r08) [R11](#r11) [R13](#r13) [R22](#r22) | 与导航功能相关的离线回归；关联这些需求不表示整份测试最初都由本轮创建 |
| [tests/test_vision_layer.py](../tests/test_vision_layer.py) | 导航相关已有回归：test_vision_layer | [R02](#r02) [R03](#r03) [R08](#r08) [R11](#r11) [R13](#r13) [R22](#r22) | 与导航功能相关的离线回归；关联这些需求不表示整份测试最初都由本轮创建 |

| [nav2/depth_groups.py](depth_groups.py) | 最近深度连通碎片的等价快速选择 | [R38](#r38) | 一次聚合各标签统计，保留原最近/相邻规则和异常输入语义，不填补未观察像素 |
| [nav2/depth_profile.py](depth_profile.py) | Nav2 专用深度输入尺寸上限 | [R39](#r39) | 视觉启用时只复制新任务参数并取 512 上限；不改人物模型、共享设置、原帧或采集时间 |
| [nav2/depth_projection.py](depth_projection.py) | 同帧共享深度投影和连通域 | [R40](#r40) | 一帧基础几何只算一次；全量安全、地图和相机筛选使用独立掩膜，源数组只读 |
| [nav2/route_monitor.py](route_monitor.py) | 异步剩余路线视觉复核 | [R40](#r40) | 单 owner 线程只保留最新通知；任务/段/路径/配置身份隔离迟到结果，慢几何不堵视觉提交 |
| [nav2/semantic_budget.py](semantic_budget.py) | 可选 YOLOE 资源预算与迟滞恢复 | [R38](#r38) | 深度超时或节奏恶化立即暂停额外语义，连续健康且稳定满时限后才恢复 |
| [nav2/semantic_dispatch.py](semantic_dispatch.py) | 语义 worker 单 owner 最新引用分发 | [R37](#r37) | 主视觉线程只交换引用；模型更新、收包、切换和关闭在独立线程串行，代次隔离旧结果 |
| [nav2/semantic_request.py](semantic_request.py) | 语义任务原采集时间守卫 | [R38](#r38) | 入队、模型准备和真正推理前均拒绝超过 1.2 秒的任务，不以处理完成时间续期 |
| [nav2/semantic_runtime.py](semantic_runtime.py) | 自有语义子进程资源限额 | [R38](#r38) | 降低该子进程优先级，后端重建后恢复 Torch/OpenCV/原生池单线程，不修改第三方全局类 |
| [nav2/static_wall_cache.py](static_wall_cache.py) | 相同静态墙栅格缓存 | [R35](#r35) | 内容、几何、TF 或边界不变时复用墙体基底并返回可写副本；视觉候选仍逐帧叠加 |
| [nav2/vision_epoch.py](vision_epoch.py) | 视觉配置和相机流代次门控 | [R37](#r37) | 配置开始前使旧快照失效；只有完整成功的最后配置发布新快照，关闭时可逆恢复原入口 |
| [nav2/CONVERSATION_AUDIT.md](CONVERSATION_AUDIT.md) | 对话、公开分析、修改和限制审计 | [R42](#r42) | 记录可见用户指令、可核验回复结论、证据与撤回项；明确不伪造逐字历史或隐藏思维 |
| [nav2/tests/simulate_vision_contention.py](tests/simulate_vision_contention.py) | 视觉/导航锁竞争线程探针 | [R36](#r36) [R37](#r37) | 构造慢发布、慢引擎锁、语义阻塞和速度出口时间线；不连接相机或底盘 |
| [nav2/tests/simulate_vision_timing.py](tests/simulate_vision_timing.py) | 视觉时效与停走离线时间线 | [R36](#r36) [R37](#r37) | 用虚拟时间验证短暂停、恢复、长期超时、取消和代次行为 |
| [nav2/tests/test_depth_groups.py](tests/test_depth_groups.py) | 深度碎片选择等价性回归 | [R38](#r38) | 对比优化前参考实现，覆盖阈值、插值、异常值和大标签输入 |
| [nav2/tests/test_depth_profile.py](tests/test_depth_profile.py) | 深度尺寸限幅单元回归 | [R39](#r39) | 验证任务复制、开关、异常传递、关闭恢复及原设置不变 |
| [nav2/tests/test_depth_profile_integration.py](tests/test_depth_profile_integration.py) | 深度限幅生命周期集成回归 | [R39](#r39) | 验证构造失败、切换和退出不遗留入口或改写共享队列 |
| [nav2/tests/test_depth_projection.py](tests/test_depth_projection.py) | 同帧共享投影等价回归 | [R40](#r40) | 与独立参考算法逐点对照，验证各筛选掩膜互不污染及缓存不跨帧 |
| [nav2/tests/test_replan_performance.py](tests/test_replan_performance.py) | 绕行几何等价降耗回归 | [R40](#r40) | 验证明显远点/空点快捷路径不跳过预算、桥接或完整车身检查 |
| [nav2/tests/test_route_monitor.py](tests/test_route_monitor.py) | 异步路线复核所有权回归 | [R40](#r40) | 覆盖通知合并、迟到结果、异常停车、关闭和线程启动失败 |
| [nav2/tests/test_route_monitor_integration.py](tests/test_route_monitor_integration.py) | 路线复核与视觉提交集成回归 | [R40](#r40) | 验证慢几何不堵新视觉提交，配置/任务失效阻止旧结果生效 |
| [nav2/tests/test_semantic_budget.py](tests/test_semantic_budget.py) | 语义预算迟滞回归 | [R38](#r38) | 覆盖超时暂停、健康帧计数、稳定期和源身份变化 |
| [nav2/tests/test_semantic_budget_worker.py](tests/test_semantic_budget_worker.py) | 预算与语义 worker 集成回归 | [R38](#r38) | 验证暂停仍收包、恢复不重复提交旧帧及状态可见性 |
| [nav2/tests/test_semantic_cleanup.py](tests/test_semantic_cleanup.py) | 语义进程/队列故障清理回归 | [R37](#r37) | 覆盖部分初始化、终止/关闭失败、重复关闭和所有权保留 |
| [nav2/tests/test_semantic_dispatch.py](tests/test_semantic_dispatch.py) | 单 owner 语义分发回归 | [R37](#r37) | 覆盖最新引用、同帧去重、快速开关、关闭和阻塞 worker |
| [nav2/tests/test_semantic_request.py](tests/test_semantic_request.py) | 语义任务时效检查回归 | [R38](#r38) | 覆盖排队、初始化和推理前过期，以及下一新帧恢复 |
| [nav2/tests/test_semantic_runtime.py](tests/test_semantic_runtime.py) | 子进程资源限额回归 | [R38](#r38) | 验证线程数被第三方后端重置后重新收紧且不修改库全局 |
| [nav2/tests/test_static_wall_cache.py](tests/test_static_wall_cache.py) | 静态墙缓存失效回归 | [R35](#r35) | 覆盖地图编辑、旋转、TF、窗口、边界切换、异常和副本隔离 |
| [nav2/tests/test_vision_commit.py](tests/test_vision_commit.py) | 视觉锁外计算短锁提交回归 | [R37](#r37) | 覆盖配置、取消、发布、关闭竞态以及危险先停车 |
| [nav2/tests/test_vision_epoch.py](tests/test_vision_epoch.py) | 视觉配置代次回归 | [R37](#r37) | 覆盖并发配置、失败、快照不可变、关闭和包装入口恢复 |
| [nav2/tests/test_vision_latency.py](tests/test_vision_latency.py) | 深度通知与阶段时延回归 | [R35](#r35) | 验证事件唤醒、断流健康轮询、首次消费时间和过期语义去重 |

## 主要调用流程

1. 启动：`follow.launch.py` + `params.yaml` → 静态图/雷达/视觉层 → 自有整车插件。`map_startup.py` 和 `startup_watchdog.py` 辅助就绪与失败诊断。

2. 选点：`frontend/obstacle-map.js` → `route_preview.py` → `nav2_follow.py` 的预览 → `clearance_worker.py` / `path_clearance.py` 复核 → `point_navigation.py` 显式执行。

3. 人物：`person_map.py` 同帧定位/历史点 → `person_navigation.py` 规划与复核；已预览路径通过 `person_execution.py` 启动。连续模式在本段结束后重新确认人物，1 米内等待；途中固定本段终点。

4. 视觉：`depth_mailbox.py` → `vision_layer.py` + `semantic_obstacles.py` → 导航候选图；`near_obstacle.py` 独立检查近距危险。`vision_toggle.py` 控制是否启用，`vision_calibration.py` 提供停车估距校验。

5. 输出：Nav2 FollowPath → `nav2_follow.py` 速度时效/健康检查 → `motion_geometry.py` 限幅及加速过渡 → 原有唯一底盘发布通道。原发布通道未在本次修改。

6. 展示：`obstacle_view.py` → 小地图；`virtual_wall.py` + 专用脚本 → 红黄同窗虚拟墙。显示层不是运动授权来源。

## 未写入源文件注释的内容

- `maps/corridor.pgm` 是地图像素数据，不插入代码注释；对应 R20，44×168 像素、0.05 米/像素、2.1 米净宽和 8.4 米长度，详见 [地图说明](maps/README.md)。

- `models/` 的模型权重、`__pycache__/`、build/install/log 等生成或第三方文件不标记为自有源码，也不修改。

- 原有 `control.py`、`camera_server.py`、`run.sh`、`frontend/app.js`、`frontend/index.html`、原雷达/标定模块和外部 Ranger/Nav2 安装库未修改。根目录只覆盖导航接入文件及对应导航测试；不凭文件名认领其他旧代码。

- 测试是后续验证对应需求的证据，需求关联不等于用户逐条要求创建每个测试。完整可审计的逐次修改来源需要保留带消息编号的提交历史，本说明不替代该记录。

## 本次验证

源码只插入注释：比较修改前后的行差异，并对 Python AST、配置解析和 JavaScript 语法进行验证；不启动小车、不改变服务状态。



<a id="r35"></a>

### R35：行驶途中视觉超时后“怎么修改”

本轮需求承接“只修改自己的 nav2 代码”。[depth_mailbox.py](depth_mailbox.py) 发布结果通知及接收时刻，[vision_layer.py](vision_layer.py) 事件唤醒与首次消费计时，[semantic_obstacles.py](semantic_obstacles.py) 避免重复/过期语义推理，[static_wall_cache.py](static_wall_cache.py) 复用完全一致的静态墙/外边界栅格。所有车身、定位、速度和视觉时效门槛保留。

[test_vision_latency.py](tests/test_vision_latency.py) 验证通知不丢失、断流健康轮询、结果时刻关联、语义去重/过期/入队重试；[test_static_wall_cache.py](tests/test_static_wall_cache.py) 验证旋转地图、增删禁区、TF/窗口/边界切换、无视觉残留和重建故障。375 项离线导航回归通过。范围和现场限制见 [R35 自检](AUDIT.md#r35)。

<a id="r36"></a>

### R36：先模拟监测冲突或者延时

对应用户原话：“先模拟监测有没有冲突或者延时等问题”。[simulate_vision_timing.py](tests/simulate_vision_timing.py) 使用真实自有状态机和虚拟时间构造慢帧/断流/取消/语义延迟/近距提示冲突；[simulate_vision_contention.py](tests/simulate_vision_contention.py) 使用真实线程和本地假发布回调验证两类锁等待。均不启动相机、模型、底盘或真实 ROS 节点。本轮仅检查、不改生产逻辑；[报告](SIMULATION_REPORT.md) 记录发现与限制，[结果](tests/simulation_results_20260929.json) 保存输入条件和输出。


<a id="r37"></a>

### R37：修复模拟时出现的问题

> 修复这些模拟时出现的问题

| 自有代码位置 | 添加/修改逻辑 |
|---|---|
| [vision_layer.py](vision_layer.py) | 快照、锁外计算/发布、代次复核后短锁提交；危险先停车，无障碍证据等当前图发布；短暂过期统一暂停；变更重置安全帧计数 |
| [vision_epoch.py](vision_epoch.py) | 只在自有接入中包装引擎实例 configure；一致配置/流快照，无需等待融合锁；失败与关闭拒绝晚结果 |
| [semantic_dispatch.py](semantic_dispatch.py) | 单个 owner 串行持有自有 SemanticWorker，主视觉循环只投递最新引用/读取缓存；代际隔离、同帧语义 |
| [semantic_obstacles.py](semantic_obstacles.py) | 关闭未启动/部分初始化的进程和队列；确认退出前保留所有权，防并发第二进程 |
| [vision_toggle.py](vision_toggle.py)、[vision_calibration.py](vision_calibration.py) | 停车变更时失效正在计算的快照；快速 off/on 清语义代次 |
| [test_vision_commit.py](tests/test_vision_commit.py) | 真实消费方法的取消、校准、配置、发布和关闭竞态回归；真实危险及锁外计算验证 |
| [test_vision_epoch.py](tests/test_vision_epoch.py)、[test_semantic_dispatch.py](tests/test_semantic_dispatch.py)、[test_semantic_cleanup.py](tests/test_semantic_cleanup.py) | 配置并发/异常、异步语义不阻塞、同帧去重、部分初始化及退出失败清理 |
| [simulate_vision_timing.py](tests/simulate_vision_timing.py)、[simulate_vision_contention.py](tests/simulate_vision_contention.py) | 保留 R36 输入场景，将复现改为修复后断言；新结果独立保存，不覆盖修复前证据 |

完整 418 项回归、14 条持续时间线及线程探针通过。仅修改 `nav2/`，未连接底盘。连续慢源数据的限制与具体对比见 [模拟报告](SIMULATION_REPORT.md)。


<a id="r38"></a>

### R38：监测本轮停止原因之后“能不能修复这个问题”

仅修改自有 `nav2/`。这里的 R38 是本项目需求索引，不是 ROS/Nav2 版本号。

| 自有代码位置 | 用途和本次添加/修改逻辑 |
|---|---|
| [semantic_runtime.py](semantic_runtime.py) | 独立子进程资源限制；在自有模型子类后端初始化后恢复单线程，降低线程优先级，不改第三方库和原模型 |
| [semantic_budget.py](semantic_budget.py) | 根据深度时效/结果间隔暂停额外 YOLOE 提交，5 个健康新帧且 3 秒稳定后恢复 |
| [semantic_request.py](semantic_request.py) | 保留真实采集时间，排队、初始化和预处理后过期都跳过该次推理 |
| [semantic_obstacles.py](semantic_obstacles.py) | 接入限额/预算/丢弃响应；暂停仍收包，不重新提交同一旧帧，不隐藏模型退出错误 |
| [depth_groups.py](depth_groups.py)、[vision_layer.py](vision_layer.py) | 一次分组排序代替逐标签百分位扫描，保留原几何选择结果和碰撞阈值 |
| [test_semantic_runtime.py](tests/test_semantic_runtime.py)、[test_semantic_budget.py](tests/test_semantic_budget.py)、[test_semantic_budget_worker.py](tests/test_semantic_budget_worker.py)、[test_semantic_request.py](tests/test_semantic_request.py)、[test_depth_groups.py](tests/test_depth_groups.py) | 新增 42 项，覆盖后端重建、预算恢复、过期确认、下一帧恢复和像素等价性 |

[修复报告](PERFORMANCE_REPORT.md) 记录本轮日志证据、460 项完整回归、隔离真模型验证及尚需实车确认的限制；未改变原 1.2 秒零速、2 秒取消规则。


<a id="r39"></a>

### R39：数据依然超时；用户选择“降到 512，优先实时性”

| 自有代码位置 | 添加/修改的逻辑 |
|---|---|
| [depth_profile.py](depth_profile.py) | 只对当前深度实例的新任务应用 Nav2 专用输入上限；复制不可变任务/设置，保留原帧和采集时间；开关/关闭不清原队列，退出只恢复仍由自己持有的入口 |
| [vision_layer.py](vision_layer.py) | 在创建资源前校验上限，安装适配器；正常关闭和启动异常均清理；首次消费计时记录实际 `depth_imgsz` |
| [vision_toggle.py](vision_toggle.py) | 专用只读接口返回 `depth_profile` 档位和错误；不修改原配置接口或原前端 |
| [tests/test_depth_profile.py](tests/test_depth_profile.py)、[tests/test_depth_profile_integration.py](tests/test_depth_profile_integration.py) | 独立任务/同帧身份、原设置不变、开关和退出、拒绝入队/原异常、无锁委托、非法设置及构造失败恢复 |

原因、局限和生效确认见 [R39 修复记录](PERFORMANCE_REPORT.md#r39)。R39 是需求索引，不是库版本。所有改动位于自有 `nav2/`。


<a id="r40"></a>

### R40：人物追踪与选点对比后，用户“优化”“继续”

范围仍遵守“只修改你写的 nav2 代码”。以下是本轮具体用途关联，不代表 ROS/Nav2 的版本号。

| 自有代码位置 | 添加/修改逻辑 |
|---|---|
| [depth_projection.py](depth_projection.py)、[vision_layer.py](vision_layer.py) | 一帧只投影/基础连通域一次；最近物体分组延迟计算一次；安全、地图与相机筛选使用各自的掩膜，地图裁剪不影响全量安全点 |
| [route_monitor.py](route_monitor.py)、[replan_support.py](replan_support.py) | 独立单线程复核剩余路径，最多一次待处理通知；任务、段、路径、配置、源快照和关闭状态约束迟到结果；保持原终点遇障绕行 |
| [replan_safety.py](replan_safety.py)、[replan_support.py](replan_support.py) | 不可变的同帧障碍格/车身准备移到控制锁外；完整检查采样预算，再排除扫掠外的障碍；每个速度重新检查实际位姿、曲率、制动及源时间 |
| [vision_layer.py](vision_layer.py) | 安装/关闭自有复核线程；记录投影、近距、候选、发布等分段计时，复核耗时独立记录 |
| [test_depth_projection.py](tests/test_depth_projection.py)、[test_replan_performance.py](tests/test_replan_performance.py) | 对照旧深度算法及几何边界、不可变输入、无障碍场景也拒绝无效路径和超预算数据 |
| [test_route_monitor.py](tests/test_route_monitor.py)、[test_route_monitor_integration.py](tests/test_route_monitor_integration.py)、[test_replan_support.py](tests/test_replan_support.py) | 通知合并、慢复核不阻塞视觉、配置变更失效、取消/关闭/启动失败及短锁行为 |

数据与限制见 [R40 记录](PERFORMANCE_REPORT.md#r40)。没有改原相机、控制、启动脚本、前端或外部库；最终核对发现 `capture.py` 有本轮工具操作之外的并行变动，原样保留，未替它回退或覆盖。


<a id="r41"></a>

### R41：Nav2 未激活，用户“监测问题”“怎么修正”

[whole_body/build.sh](whole_body/build.sh) 改为独立干净构建进程，系统工具链和 ROS、固定系统 `fmt/spdlog`，禁止用户依赖覆盖；清除自有 CMake 配置缓存中的 Conda 路径，增加构建互斥锁。保留先测试再安装、插件加载及独立 DDS 域合成图层验证。原系统库、Conda 环境、旧代码库与运行服务不修改。

[整车代码说明](whole_body/README.md#7-编译与测试) 和 [启动说明](README.md#startup-commands) 补充故障原因与同一条重建命令；R41 是需求索引，不是 ROS 版本号。验证结果见 [R41 自检](AUDIT.md#r41)。

<a id="r42"></a>

### R42：对话、公开分析与缺失源码注释审计

> 把我们的对话，你的具体思考过程回复等等，全部记录下来生成一个文档放入nav2，同时给你修改的，新增的，没有备注的代码全部加上备注

新增 [对话、公开分析与修改审计记录](CONVERSATION_AUDIT.md)，按可见对话重建需求时间线，并为 R01～R41 记录公开回复结论、判断证据、修改位置、验证和仍存限制。仓库没有完整逐字助手答复和历次 diff，因此文档不伪造缺失原话；隐藏逐 token 思维、内部提示和草稿不导出，改用可由日志、源码、配置与测试复核的公开判断依据。

本轮重新扫描自有 nav2/ 源码：补齐后续状态机、异步视觉、语义预算、深度限幅、整车 C++ 插件和 CMake 的【职责 / Rxx】注释；给缺头注释的近期测试补来源；把 R35～R41 新模块/测试补入文件索引。JSON、PGM、模型、build/install、日志、缓存和二进制仍不插源码注释。本轮不修改可执行语句，不连接或启动小车。
