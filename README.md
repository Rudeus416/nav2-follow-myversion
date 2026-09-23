# visual_car：照片流行人跟随

相机与毫米波雷达的同步采集、人工外参标定说明见 [CALIBRATION.md](CALIBRATION.md)。

`control.py` 是运行入口。它沿用 `test.py` 的 YOLO26 实例分割、BoT-SORT + ReID、YOLO26 米制深度图、人物测距和距离 PID / 水平角度 PD 流程，但独立实现，没有导入或调用 `test.py`。海康相机订阅和网页采集由 `camera_server.py` 提供。运行时采用六条逻辑流水线：相机把带单调递增帧号的 BGR 放入多帧 `C` 缓冲；Detection 与 BoT-SORT 因 Ultralytics 封装而暂时共用一个 Tracking Worker；该 Worker 在调用 `model.track()` **之前**先发布同帧 DepthRequest，使 Depth 与 Detection/Tracking 并行；两种结果通过帧号和配置版本做非阻塞精确 Join；Tracking 每次更新转向，同帧深度完成后更新距离与线速度；最后由独立显示线程绘制缩略图和 JPEG。相机原图 JPEG 与结果图都使用单槽最新帧邮箱，慢速显示或编码只替换旧任务，不阻塞控制和 ROS 图像回调。

## 启动

推荐使用统一管理脚本。首次执行前赋予权限，之后不带参数即可启动全部服务：

```bash
cd /home/metaiot/Workspace/visual_car
chmod +x run.sh
./run.sh
```

检查任务是否完整运行、查看日志和安全结束任务：

```bash
./run.sh status
./run.sh logs
./run.sh stop
```

脚本启动后默认锁存强制停止，需要在网页确认环境安全后点击“解除强制停止”。`stop` 会先锁存并发送零速度，再依次关闭视觉控制、相机子进程和 Ranger 驱动；CAN 接口保持启用，方便下次启动。

也可以按下面的原始命令分别启动：

```bash
source /opt/ros/humble/setup.bash
source /home/metaiot/Workspace/dataCollectToolkit/ros2_driver_ws/install/setup.bash
cd /home/metaiot/Workspace/visual_car
/home/metaiot/miniconda3/envs/follow_demo/bin/python -B control.py \
  --fps 5 --camera-fps 30 --process-fps 15 --depth-fps 5 --port 8890
```

打开 `http://127.0.0.1:8890/`，局域网用 Jetson IP。服务默认监听 `0.0.0.0:8890`，同一局域网内可直接访问 `http://<Jetson-IP>:8890/`；当前页面没有账号认证，只要网络可达就能操作。相机若由别的终端启动，加 `--no-launch`。若 Ranger 底盘使用不同的速度话题，用 `--cmd-vel-topic` 配置；默认以 50 Hz 发布 `/cmd_vel` 上的 ROS 2 `geometry_msgs/msg/Twist`，只设置 `linear.x` 和 `angular.z`，其余字段保持零。

Ranger 驱动位于 `/home/metaiot/Workspace/RANGER/ranger_ws`，需要在另一个终端先启动 CAN 和底盘节点：

```bash
source /opt/ros/humble/setup.bash
source /home/metaiot/Workspace/RANGER/ranger_ws/install/setup.bash
sudo ip link set can0 up type can bitrate 500000
ros2 launch ranger_bringup ranger_mini_v3.launch.py port_name:=can0
```

启动后用 `ros2 topic info /cmd_vel -v` 检查是否出现 `ranger_base_node` 订阅者，用 `ip -details link show can0` 检查 CAN 是否为 `UP/ERROR-ACTIVE`。Jetson 上的 `control.py` 发布 ROS 消息，`ranger_base_node` 在决策计算单元接收消息并通过 `ugv_sdk` 和 `can0` 转成 CAN 指令，CAN 指令才直接发送给线控底盘。

把已有照片按时间模拟成一段视频，可用 `--no-launch --replay-existing`；回放只是给处理线程提供照片，并不隔离 `/cmd_vel`。程序启动时强制停止默认锁存，但只做回放验证仍应使用独立 `ROS_DOMAIN_ID` 和假的速度话题，避免碰到实际底盘。

## 跟随和手动控制

页面默认自动模式、人物 ID 0、相机高度 0.50 m、跟随距离 1.00 m。程序启动即锁存强制停止；解除后还要点击“开始跟随”才可能输出自动运动。选择新的 ID 或应用新参数会增加配置版本、清除未配对结果并发送零速度；只有新版本的同帧 Tracking/Depth 结果才能恢复更新。每个 Tracking 结果都会推进单调控制帧水位；目标不存在时立即发零速度，之后到达的旧帧深度结果不能覆盖这次停止。距离不可用或深度命令超过三个周期（限制在 0.25～1.0 秒）时同样保持零速度。按“结束跟随”也立即停下。按模式键切到手动后，按住四个方向键会按配置加速度逐步增加前进、后退或左右角速度；松开立即停，浏览器断开时 0.35 秒心跳超时停。

页面显示**实际发布**的 `linear.x` 与 `angular.z`，并分别提供相机送入跟踪频率、原图留存频率、跟踪频率上限、深度频率上限、检测器最低置信度、BoT-SORT 高低分阈值、新建轨迹阈值、ReID 最低 IoU 门槛、PID 增益、速度与加速度上限等参数。运行状态会显示 ROS 实际输入 FPS、实际跟踪/深度 FPS、真实帧间隔、跳帧数、C/N/M1/M2/T/F 缓冲大小、Join 等待/过期数量、配置版本，以及 Tracking、控制和显示三类时延。保存配置使用当前页面的输入值，写到 `data/param_configs/YYYYMMDD_HHMMSS_ffffff.json`。参数通过版本化快照从下一任务生效；已经在途的旧版本结果会被拒绝，不会重新写入控制。旧配置中的 `detection_confidence` 会在加载时兼容为 `detector_min_conf`。

图片默认采用“仅内存”模式：原图 JPEG 按 `--max-frames` 在进程内存保留，默认 20 张；结果 JPEG 在内存保留最近 10 张，页面各显示最近 10 张，进程退出后这些内存图片消失。单纯用于相机网页预览的 JPEG 会先缩至 1280 像素宽；到达原图留存周期时仍编码并保留完整分辨率。点击页面的存储方式按钮可切换为“内存 + data目录”，此后原图额外写到 `data/img_raw`，结果额外写到 `data/img_processed`；再次切回仅内存不会删除以前的磁盘文件。启动 `control.py` 时仍会把旧 `data/frame_*.jpg` 移到 `data/img_raw`。磁盘少于 50 MB 时自动退回内存保留并报告写盘错误。参数配置会保存当前存储方式。

## 毫米波辅助测距

跟随页面的“毫米波雷达”默认关闭。要启用或调整任何雷达设置，先按“强制停止”；后端会拒绝未锁存强制停止时的雷达配置请求。勾选“使用并启动毫米波雷达”后应用设置，服务会用固定的 3TX profile 启动雷达；已有外部雷达驱动时勾选“仅订阅”。最大/最小距离、帧周期、两个 CFAR 门限、峰值合并、串口和话题均可在页面设置。改变需要下发到设备的参数会停止并重新启动本服务管理的雷达驱动。校准页面保存的外参文件可在跟随页面读取；直接改六个外参数值需先勾选“允许手动调整以下六个外参”。配置文件会保存雷达设置。跟随任务持续接收点云，原图留存 FPS 和采集时长仍由已有相机采集栏控制。

相机与雷达均使用 ROS `header.stamp`，时间戳为零时才退回本机接收时间。每张参与跟踪的图像在 Tracking/Depth 同帧合并时选择缓冲区中时间戳最近的雷达点云，不设置时间差阈值；状态栏显示实际时间差。雷达点使用校准外参、相机内参及畸变投影到左侧图像，右侧仍为深度图。人物框中的投影点优先按实例掩码筛选，再取距离最集中的一组点，计算这些点在相机水平 `(x,z)` 平面上的平均距离，作为 PID 距离输入。框中没有雷达点时，使用单目距离乘以比例系数；该系数由同时可得的雷达/单目距离比值通过一维 Kalman 滤波更新。尚无有效配对时系数为 1，状态栏会显示尚未校正。雷达关闭时保持原有单目测距方式。

雷达点云可能很稀疏；单个点仍可能是杂波，外参误差也会使背景点落入人物框。启用自动运动前，应在不同距离和方位复核外参、投影与测距，并观察状态栏的时间差和目标距离来源。当前选择最近点云不使用时间差门限，因此传感器延迟或短时掉帧会增加测距误差。

雷达配置开始时会结束当前跟随；配置完成后，解除强制停止并重新点击“开始跟随”才会恢复自动运动。

## 标定与跟踪

`camera_calibration.yaml` 复制自 dataCollectToolkit 的海康相机 `camera_info.yaml`，标定尺寸 3072×2048，内参为 `fx=1859.588329`、`fy=1869.678526`、`cx=1493.542388`、`cy=1151.062343`，五项 `plumb_bob` 畸变为 `[-0.048256, 0.073359, 0.017038, -0.008814, 0]`。代码按照片尺寸缩放内参。控制分支保留网络输入尺寸的低分辨率实例掩码，去掉 letterbox 填充后把前景采样点映射回原图坐标，再使用原相机内参去畸变和采样深度。默认“全身深度图”模式使用人物实例掩码下半区域的深度，做中位数/MAD 去离群，得到以相机为原点、水平 `(x,z)` 平面上的距离；相机高度不参与这一模式。脚底模式在水平相机假设下用离地高度与脚底射线求地面交点，脚被裁切或射线接近地平线时距离可能无效。单目米制深度需要在实际相机上做距离尺度校验，尤其是 1 m 控制点。

相机帧号和公开人物 ID 都单调递增，不主动回绕。每条 TrackingResult 携带相机帧号、单调时间戳、真实帧间隔、跳过的相机帧数和配置版本。项目会用“真实帧间隔 ÷ 配置处理周期”更新 BoT-SORT Kalman 状态转移和过程不确定度；稀疏光流 GMC 直接估计两张实际选中图像之间的相机运动。BoT-SORT 同时使用 IoU 与 ReID 外观匹配；丢失轨迹与公开 ID 的保留时间按真实时间约 3 秒计算，不再随实际处理 FPS 改变。

页面上的“持续身份认证”默认关闭。开启后，系统会为人物维护会话内逻辑 ID 和一个只收录高质量 ReID 特征的小型 gallery；只有当前选中的跟随目标会在 BoT-SORT 分配新内部 ID 后尝试重认。候选必须是目标丢失附近新出现的轨迹，并通过外观距离、位置、尺度、候选唯一性和连续 3 个 Tracking 结果确认，才会重新绑定原逻辑 ID。已在目标丢失前稳定存在的旁观者不会被抢占为目标。目标丢失、候选不明确、确认中或 3 秒重认超时时，底盘保持停车；确认成功后仍要等新 ID 对应的同帧深度结果才能恢复线速度。该机制只在当前进程会话内有效，不跨程序重启保存人物生物特征。

默认使用直线距离和水平居中控制；新增可选的毫米波 Nav2 绕行跟随，通过 `NAV2_ENABLED=1 ./run.sh start` 启用，依赖、雷达到底盘 TF、轮廓配置及限制见 [Nav2 说明](nav2/README.md)。实车启动非零速度前，应先核对 Ranger 运动模式、速度话题、正负方向、相机安装方向和深度尺度。
