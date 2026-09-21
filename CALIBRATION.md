# 相机与毫米波雷达人工外参标定

`capture.py` 采集海康相机 `/hk_camera/image_raw` 和 TI 毫米波雷达处理后的点云
`/ti_mmwave/radar_scan_pcl`。图像以 JPEG 保存，点云以 NPZ 保存；每个雷达帧
匹配时间戳最近、且时间差不超过阈值的相机图像。同一张图像可以匹配多个雷达帧。

## 采集

雷达默认需要 `/dev/ttyACM0` 命令口、`/dev/ttyACM1` 数据口，以及连接 DCA1000
的网卡地址 `192.168.33.30`。先连接硬件，检查 `ls /dev/ttyACM*` 和 `ip -br addr`。
默认启动脚本会启动雷达联合驱动和海康相机；不要再对同一硬件启动第二套驱动。

```bash
cd /home/metaiot/Workspace/visual_car
source /opt/ros/humble/setup.bash
source /home/metaiot/Workspace/dataCollectToolkit/ros2_driver_ws/install/setup.bash
/home/metaiot/miniconda3/envs/follow_demo/bin/python -B capture.py
```

不需要在命令行配置参数时，直接运行上面的命令，不加额外选项即可。请使用
`follow_demo` 环境的 Python；系统 Python 的 NumPy 版本可能使雷达驱动依赖检查失败。
`--name` 默认是 `None`，脚本会用启动时间生成不重复的采集目录；如果把它改成固定名字，第二次运行前需换名。

在 [capture.py](capture.py) 的 `parse_args()` 中修改 `default=`，保存后下次启动生效。雷达源文件路径写在文件顶部的 `DEFAULT_RADAR_PROFILE`，也可以把 `--radar-profile` 的 `default=` 改成另一个 `Path(...)`。当前默认值如下，命令行同名选项仍可临时覆盖：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `--name` | `None` | 自动用启动时间命名目录 |
| `--duration` | `3.0` | 采集秒数 |
| `--camera-fps` | `5.0` | 每秒最多保存 5 张相机图像 |
| `--pair-threshold-ms` | `120.0` | 雷达帧与最近图像允许的最大时间差 |
| `--radar-frame-period-ms` | `200.0` | 雷达约每秒输出 5 帧 |
| `--radar-profile` | `xWR1843_profile_1280.cfg` | 使用实测成功的 2TX 配置源文件 |
| `--min-range-m` / `--max-range-m` | `0.0` / `6.0` | 雷达板输出点的距离范围，米 |
| `--cfar-range-db` / `--cfar-doppler-db` | `10.0` / `10.0` | 距离维和速度维检测阈值，dB |
| `--cfar-peak-grouping` | `"none"` | 不合并相邻峰值，点数可能增加，也可能有重复检测 |
| `--external-camera` / `--external-radar` | `False` / `False` | 默认由脚本启动两个传感器 |
| `--camera-topic` / `--radar-topic` | `CAMERA_TOPIC` / `RADAR_TOPIC` | 订阅的话题名，常量定义在文件顶部 |
| `--cli-port` / `--data-port` | `/dev/ttyACM0` / `/dev/ttyACM1` | 雷达命令口和点云数据口 |
| `--startup-timeout` | `45.0` | 等待传感器启动的最长秒数 |
| `--jpeg-quality` / `--queue-size` | `95` / `16` | 图像质量和写盘队列容量 |

`--cfar-peak-grouping` 可选 `"both"`、`"range"`、`"doppler"`、`"none"`。阈值越低，弱回波越多，噪声也可能增加；`"none"` 会产生很多相邻检测。`--min-range-m` 可改为 `0.3` 排除近场点，但这一值尚未完成独立硬件复测。把 `--external-radar` 的 `default` 改为 `True` 后，脚本只订阅已经运行的雷达，内部设置的 profile、距离、CFAR 和帧周期均不会应用到外部雷达。

`--max-range-m` 修改本次快照 profile 中距离维度的 `cfarFovCfg` 上限，不能突破当前
`profileCfg` 与硬件本身的探测能力。生成的 `radar_profile.cfg` 和 `radar_params.yaml`
保存在本次采集目录，不修改 dataCollectToolkit 中的原始配置。
采集快照默认将雷达帧周期设为 200 ms（约 5 FPS），并让 UART 只发送检测点；
原始 profile 的 50 ms 帧周期和全部热图输出在本机实测导致点云数据口无连续输出。
可用 `--radar-frame-period-ms` 修改帧周期。更短的周期需给雷达采样和板上处理留足时间。
图像默认最多存储 5 FPS，可通过 `--camera-fps` 修改；雷达点云按收到的帧保存。

### 墙面点云稀疏时的实测调参

2026-09-19 在本机让雷达面向约 2.7 m 处的墙面，使用 200 ms 帧周期、6 m 距离上限，对每种配置短采集后得到：

| 配置 | 每帧平均点数 | 1–4 m 每帧平均点数 | 零调整投影落入图像 | 高度 z |
| --- | ---: | ---: | ---: | --- |
| 原 2TX，两个 CFAR 阈值 15 dB，两维峰值合并 | 8 | 5 | 5 | 全 0 |
| 原 2TX，阈值 10 dB，两维峰值合并 | 11.8 | 7 | 6.8 | 全 0 |
| 原 2TX，阈值 15 dB，仅速度维峰值合并 | 25 | 12 | 12 | 全 0 |
| 原 2TX，阈值 12 dB，仅速度维峰值合并 | 38 | 25 | 17.7 | 全 0 |
| 3TX `AWR1843_slam.cfg`，阈值 8 dB，仅速度维峰值合并 | 24.3 | 10 | 18.3 | 有非零高度 |

图像内点数按现有相机内参、畸变和默认轴向变换离线计算；尚未套入物理外参。这个数值只用于比较同一场景中的配置效果。

当前默认是 2TX、10 dB 且不做峰值合并，会保留较多弱回波和相邻检测。若想减少重复点，可改用 12 dB、`doppler` 峰值合并；这组参数曾在本机实测。约 0.05–0.23 m 的近场串扰点可以在标定时忽略；`--min-range-m 0.3` 可在新的采集中由雷达板直接排除这些点。

如果校准需要高度信息，改用 `xWR1843_profile_3tx4rx_20hz.cfg`。2026-09-20 本机使用该 profile、200 ms 帧周期、6 m 距离上限、10 dB 双维 CFAR 门限和不合并峰值，实测 5 秒得到 20 帧、3370 个点，其中 3369 个点的 Z 非零（约 -1.90 至 +1.65 m）。通过网页再采 3 秒得到 5 帧、851 个点，Z 均非零；这些点满足 `range = sqrt(x²+y²+z²)`，对 0.3 m 外的点计算出的俯仰角约为 -26.2° 至 +34.9°。这证明俯仰角估计已进入点云；绝对高度精度仍需用已知高度的反射目标独立标定。

此前一次网页尝试报 `numpy._CopyMode`，发生在雷达驱动导入阶段，尚未向雷达下发 3TX 配置。网页服务现在使用 `follow_demo` Python 启动采集，已通过实测。`AWR1843_slam.cfg` 虽曾产生非零 Z，但禁用了 LVDS，当前联合采集脚本会拒绝它。降低 CFAR 阈值会让弱回波和噪声一同进入点云；关闭两个维度的峰值合并可能产生相邻速度单元的重复检测和近场点。

平整墙面往往只给少数强反射，板上 CFAR 导出的是检测峰值，不会自动生成覆盖墙面的密集深度图。`--cfar-peak-grouping` 可选 `both`、`range`、`doppler`、`none`；`--radar-profile` 可选择 Toolkit 中的其它配置。每次采集保存的 `radar_profile.cfg` 是实际下发给雷达的参数快照。

如果相机已由 `run.sh` 或其他进程启动，加 `--external-camera`。如果雷达已由其他
进程启动，加 `--external-radar`；此时脚本中的雷达配置默认值会被忽略，也不能在命令行
指定雷达调参选项。两路都已启动时，可同时指定两个 `--external-*` 选项。
若 `run.sh` 同时控制底盘，采集标定目标前请在原页面结束跟随并保持强制停止。
脚本等到相机首帧和雷达点云话题发布者出现后开始计时。若没有收到点云，仍保存图像并在
`manifest.json` 的 `warnings` 中记录；请检查雷达前是否有明显反射的标定目标。

输出结构：

```text
data/data_calib/hall_01/
├── camera/              # 按 ROS 时间戳命名的 JPEG
├── radar/               # 按 ROS 时间戳命名的 NPZ，含 x/y/z/range/velocity 等
├── pairs/               # 每组配对一个 JSON，记录相对文件路径和时间差
├── manifest.json        # 采集参数、帧列表、队列丢弃数和错误
├── radar_profile.cfg    # 本次雷达 profile 快照（自行启动雷达时没有）
├── radar_params.yaml    # 本次 ROS 参数快照（自行启动雷达时没有）
├── radar.log             # 本次启动日志（自行启动雷达时没有）
└── camera.log            # 本次启动日志（自行启动相机时没有）
```

时间匹配使用 ROS 消息 `header.stamp`；只有该值为零时才退回本机接收时间。
相机驱动和雷达点云节点当前均在发布时写入 ROS 时间。`manifest.json` 还保留本机
接收时间。JPEG 编码在独立线程完成，雷达点云使用另一条写入队列；若写盘跟不上，队列会丢弃部分帧并计入
`dropped_queue`，应检查采集批次的统计信息。

## 打开标定页面

另开终端启动页面服务；网页发起采集时服务会为 `capture.py` 加载 ROS 环境：

```bash
cd /home/metaiot/Workspace/visual_car
/home/metaiot/miniconda3/envs/follow_demo/bin/python -B calibration_server.py --port 8891
```

本机打开 `http://127.0.0.1:8891/`。需要局域网访问时，将启动参数改为
`--host 0.0.0.0`，用主机 IP 打开。页面从 `data/data_calib` 读取批次和配对。

页面右列下方的“启动相机与毫米波雷达采集”可填写 `capture.py` 的全部采集参数。
“雷达检测参数 → 雷达天线配置”提供 1TX、2TX、3TX 三种 profile；2TX 是默认值，
3TX 用于输出俯仰角和非零 Z。1TX 使用本仓库的 `radar_profiles/1tx4rx_capture.cfg`，
它从已验证的 2TX 配置改成单 TX；Toolkit 原有的 1TX profile 在本机启动后没有连续
点云，故不作为页面选项。实测本仓库 1TX profile 在 5 秒采集中保存了 15 帧、2668 个点。
即使 3TX 文件名带 `20hz`，页面的 200 ms 帧周期也会
覆盖为约 5 FPS。网页服务启动采集时默认使用本机 `follow_demo` Python；需要改用其他
环境时可设置 `VISUAL_CAR_CAPTURE_PYTHON` 环境变量。
点击“开始采集”后会显示进程输出和状态；“停止采集”会请求脚本保存已收到的数据并停止传感器。
采集名称留空时按启动时间命名。勾选“雷达已由其他进程启动”后，雷达 profile、
距离、CFAR、帧周期和串口配置会停用，也不会传给脚本。采集结束后批次列表自动刷新；
也可用 01 区域的“刷新批次”按钮手动检查新批次，该操作保留当前选中的批次和配对。
修改网页采集服务端代码后需要重启 `calibration_server.py`。

先选一个配对，再在图像中拖动雷达点。04 的六个参数与三维雷达点云窗口使用同一
坐标系：X 前、Y 左、Z 上。偏航绕雷达 Z 轴，俯仰绕雷达 Y 轴，横滚绕雷达 X 轴；
正方向均遵循右手定则。旋转模式下，水平/垂直拖动分别调偏航和俯仰；按住 Shift
水平拖动调横滚。平移模式下，水平拖动调雷达 Y，垂直拖动调雷达 Z；按住 Shift
垂直拖动调雷达 X。左侧 04 区域也可直接输入六个数值，完整 4×4 矩阵随操作更新。
切换配对时参数保持不变。

雷达消息采用 ROS 坐标系（X 前、Y 左、Z 上），相机使用光学坐标系（X 右、Y 下、
Z 前）。因此零调节量仍包含**已知的坐标轴换轴**。页面投影使用
`camera_calibration.yaml` 中的内参与五项 `plumb_bob` 畸变系数；点的颜色由
Matplotlib `magma` 色卡表示雷达原始距离。

图像下方的“三维雷达点云”窗口显示当前所选配对的全部原始雷达检测点，
不受外参及相机视野裁剪影响。拖动该窗口可旋转观察视角，滚轮缩放，双击恢复
默认视角；这些操作不会修改标定参数。窗口的 X/Y/Z 轴分别为雷达坐标系的
向前/向左/向上，刻度单位是米，点色同样使用 Matplotlib `magma` 表示距离。
当前默认 2TX 配置输出的 Z 坐标可能全部为 0，页面会提示这批点实际位于平面上。

“保存参数”首次创建 `data/data_transfer/<时间戳>.json`；加载某个已存参数后，
该按钮会更新同一个文件，文件名不变。“另存为新文件”会创建新的时间戳文件。
新版文件中的六个参数按雷达坐标系保存，矩阵满足
`P_camera_optical = A · (Rz(yaw) · Ry(pitch) · Rx(roll) · P_radar_ros + t_radar)`，
其中 `A` 是固定的雷达轴到相机光学轴的换轴矩阵；页面显示及文件保存的 4×4 矩阵
以相机光学坐标为输出。加载旧版 `schema_version: 1` 参数文件时，页面会自动换算六个参数，
维持原有 4×4 矩阵和投影；再次保存会在原文件名下写入新版格式。

人工投影能帮助初调外参，但单个平面或单个距离上的稀疏点不能唯一约束所有
六个自由度。建议在多个距离、方位和高度放置反射目标，切换多组配对检查对齐，
并记录采集时的传感器安装状态。跟随页面可选择并读取这里保存的外参文件；
读取后按“强制停止”并应用雷达设置，跟随测距才会使用该外参。
