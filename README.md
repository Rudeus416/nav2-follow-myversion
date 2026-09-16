# visual_car：照片流行人跟随

`control.py` 是运行入口。它沿用 `test.py` 的 YOLO26 实例分割、BoT-SORT + ReID、YOLO26 米制深度图、人物测距和距离 PID / 水平角度 PD 流程，但独立实现，没有导入或调用 `test.py`。海康相机订阅和网页采集由 `camera_server.py` 提供。同一张内存 BGR 图像直接进入容量为 1 的控制队列，不等待 JPEG 写盘再读取；控制线程完成检测、跟踪、深度、测距和动作更新后，立即处理下一张最新图。绘图线程从另一个容量为 1 的队列读取最新处理结果，按 `test.py` 的样式生成前端画面，绘图跟不上时只跳过旧的显示结果，不阻塞动作更新。

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
/home/metaiot/miniconda3/envs/follow_demo/bin/python -B control.py --fps 5 --process-fps 3 --port 8890
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

把已有照片按时间模拟成一段视频，可用 `--no-launch --replay-existing`；回放只是给处理线程提供照片，并不隔离 `/cmd_vel`。只做回放验证时，使用独立 `ROS_DOMAIN_ID` 或单独速度话题，避免碰到实际底盘。

## 跟随和手动控制

页面默认自动模式、人物 ID 0、相机高度 0.50 m、跟随距离 1.00 m。点击“开始跟随”后才输出自动运动。选择新的 ID 会清空旧控制命令，等新目标在下一处理帧出现后继续跟随；目标不存在、距离不可用或图像命令超时（最多 2 秒）时立即发零速度。按“结束跟随”也立即停下。按模式键切到手动后，按住四个方向键会按配置加速度逐步增加前进、后退或左右角速度；松开立即停，浏览器断开时 0.35 秒心跳超时停。强制停止为锁存状态，在自动和手动模式都持续发零速度；必须明确点“解除强制停止”才能恢复。

页面显示**实际发布**的 `linear.x` 与 `angular.z`，并提供采集频率、处理采样频率、检测阈值、PID 增益、速度与加速度上限等参数。页面分别显示控制时延和结果画面时延。保存配置使用当前页面的输入值，写到 `data/param_configs/YYYYMMDD_HHMMSS_ffffff.json`；应用配置会更新运行参数。自己启动的相机在下一次启动时会使用已加载的曝光与增益；外部启动的相机在运行时加载配置才能调用其参数服务。

图片默认采用“仅内存”模式：原图 JPEG 按 `--max-frames` 在进程内存保留，默认 100 张；结果 JPEG 在内存保留最近 10 张，页面各显示最近 10 张，进程退出后这些内存图片消失。点击页面的存储方式按钮可切换为“内存 + data目录”，此后原图额外写到 `data/img_raw`，结果额外写到 `data/img_processed`；再次切回仅内存不会删除以前的磁盘文件。启动 `control.py` 时仍会把旧 `data/frame_*.jpg` 移到 `data/img_raw`。磁盘少于 50 MB 时自动退回内存保留并报告写盘错误。参数配置会保存当前存储方式。

## 标定与跟踪

`camera_calibration.yaml` 复制自 dataCollectToolkit 的海康相机 `camera_info.yaml`，标定尺寸 3072×2048，内参为 `fx=1859.588329`、`fy=1869.678526`、`cx=1493.542388`、`cy=1151.062343`，五项 `plumb_bob` 畸变为 `[-0.048256, 0.073359, 0.017038, -0.008814, 0]`。代码按照片尺寸缩放内参。控制分支保留网络输入尺寸的低分辨率实例掩码，去掉 letterbox 填充后把前景采样点映射回原图坐标，再使用原相机内参去畸变和采样深度。默认“全身深度图”模式使用人物实例掩码下半区域的深度，做中位数/MAD 去离群，得到以相机为原点、水平 `(x,z)` 平面上的距离；相机高度不参与这一模式。脚底模式在水平相机假设下用离地高度与脚底射线求地面交点，脚被裁切或射线接近地平线时距离可能无效。单目米制深度需要在实际相机上做距离尺度校验，尤其是 1 m 控制点。

公开人物 ID 从 0 开始单调递增，不回收。BoT-SORT 使用 Kalman 运动预测、稀疏光流相机运动补偿、IoU 与 ReID 外观匹配；丢失轨迹最多保留 45 个**已处理帧**，若重新关联成功会继续同一 ID。若未关联成功，就分配新 ID；丢失期间即使跟踪器内部仍记住轨迹，底盘也立即停。遮挡太久、外观变化大或相机快速运动时不能保证重连，系统不会自动切换到别的人。

目前只实现直线距离和水平居中控制，没有障碍物观测和避障规划。实车启动非零速度前，应先核对 Ranger 运动模式、速度话题、正负方向、相机安装方向和深度尺度。
