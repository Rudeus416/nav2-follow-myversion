# 【内容标注】用途：自有 Nav2 节点与插件启动装配。
# 对应用户需求：R01 R13 R21 R26（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：选择雷达/地图/叠加模式；加载自有插件；配置静态图、坐标变换和生命周期启动。
# 需求关联用于用途追溯；本次修复长地图规划窗口，见 nav2/AUDIT.md。
"""Mapless Humble Nav2; all controller output is gated by visual_car_control."""
import json
import os
import tempfile
import yaml
import math
import importlib.util
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler, EmitEvent, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


# 【职责 / R01 R13 R21】planar_radar_pose：从雷达标定关系提取平面位姿。
def planar_radar_pose(calibration, camera_pose):
    """Drop height/tilt from saved radar->camera FLU adjustment, then compose."""
    if calibration.get('schema_version') != 2 or calibration.get('parameter_frame') != 'radar_ros':
        raise ValueError('Planar TF requires schema_version=2, parameter_frame=radar_ros')
    x, y, yaw = camera_pose
    t = calibration['translation_m']
    angle = math.radians(calibration['adjustment_euler_deg']['yaw'])
    if not all(math.isfinite(v) for v in (x, y, yaw, t['x'], t['y'], angle)):
        raise ValueError('Non-finite planar mounting parameter')
    return (x + math.cos(yaw)*t['x'] - math.sin(yaw)*t['y'],
            y + math.sin(yaw)*t['x'] + math.cos(yaw)*t['y'], yaw + angle)


# 【职责 / R01 R13 R21】radar_tf：组织雷达坐标变换参数。
def radar_tf(context):
    if LaunchConfiguration('publish_radar_tf').perform(context).lower() != 'true':
        return []
    calibration = json.loads(Path(LaunchConfiguration('radar_calibration').perform(context)).read_text())
    camera = tuple(float(LaunchConfiguration(key).perform(context))
                   for key in ('camera_x', 'camera_y', 'camera_yaw'))
    x, y, yaw = planar_radar_pose(calibration, camera)
    return [Node(package='tf2_ros', executable='static_transform_publisher',
                 name='visual_car_planar_radar_tf', output='screen', arguments=[
                     '--x', str(x), '--y', str(y), '--z', '0',
                     '--roll', '0', '--pitch', '0', '--yaw', str(yaw),
                     '--frame-id', 'base_link', '--child-frame-id',
                     LaunchConfiguration('radar_frame').perform(context)])]


# 【职责 / R01 R13 R21】navigation_nodes：按环境变量配置地图/雷达图层、插件及节点。
def navigation_nodes(context):
    root = Path(__file__).resolve().parent
    mode = os.environ.get('NAV2_OBSTACLE_MODE', 'radar')
    if mode not in ('radar', 'map', 'both'):
        raise ValueError('NAV2_OBSTACLE_MODE must be radar, map or both')
    config = yaml.safe_load(Path(LaunchConfiguration('params_file').perform(context)).read_text())
    map_file = None
    window = None
    if mode != 'radar':
        map_file = os.environ.get('NAV2_MAP_FILE', str(root / 'maps/corridor.yaml'))
        # Load by path: ros2 launch need not run from this project's directory.
        spec = importlib.util.spec_from_file_location('visual_car_map_window', root / 'map_window.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        window = module.configure_global_map_window(config, mode, map_file, map_to_odom_yaw=math.pi/2)
    prefix = root / 'whole_body/install'
    if not (prefix / 'lib/libvisual_car_whole_body.so').is_file():
        raise ValueError('先编译本项目整车导航插件：bash nav2/whole_body/build.sh')
    plugin_env = {key: str(prefix / subdir) + os.pathsep + os.environ.get(key, '')
                  for key, subdir in (('AMENT_PREFIX_PATH', ''), ('LD_LIBRARY_PATH', 'lib'))}
    if config['planner_server']['ros__parameters']['GridBased']['plugin'] != 'visual_car_nav2::Planner':
        raise ValueError('NAV2_PARAMS 必须使用 visual_car_nav2::Planner 整车规划器')
    controller = config['controller_server']['ros__parameters']['FollowPath']
    if (controller['plugin'] != 'dwb_core::DWBLocalPlanner'
            or controller['critics'][0] != 'WholeBody'
            or controller.get('WholeBody.class') != 'visual_car_nav2::WholeBodyCritic'):
        raise ValueError('NAV2_PARAMS 必须使用 WholeBody 作为第一项轨迹碰撞检查')
    for key in ('local_costmap', 'global_costmap'):
        costmap = config[key][key]['ros__parameters']
        costmap['plugins'] = (['static_layer'] if mode != 'radar' else []) + (['obstacles'] if mode != 'map' else []) + ['inflation']
        costmap['static_layer'] = {'plugin': 'nav2_costmap_2d::StaticLayer', 'map_topic': '/visual_car/edited_map', 'map_subscribe_transient_local': True, 'use_maximum': True}
        if mode == 'map':
            # Boundary precedes the final combined vision/static layer.
            costmap['plugins'].insert(-1, 'map_boundary')
            costmap['map_boundary'] = {'plugin': 'nav2_costmap_2d::StaticLayer',
                'map_topic': '/visual_car/map_boundary',
                'map_subscribe_transient_local': True, 'use_maximum': True,
                'track_unknown_space': False}
        costmap['plugins'].insert(-1, 'vision_layer')
        costmap['vision_layer'] = {'plugin': 'nav2_costmap_2d::StaticLayer',
            'map_topic': '/visual_car/vision_costmap', 'map_subscribe_transient_local': True,
            'use_maximum': True, 'track_unknown_space': False,
            'enabled': os.environ.get('NAV2_VISION_OBSTACLES', '0') == '1'}
        if costmap['inflation']['plugin'] != 'visual_car_nav2::HardBuffer':
            raise ValueError('NAV2_PARAMS 必须使用 HardBuffer 紫色禁入缓冲层')
        costmap['obstacles']['radar']['topic'] = os.environ.get('NAV2_RADAR_TOPIC', '/ti_mmwave/radar_scan_pcl')
    with tempfile.NamedTemporaryFile(mode='w', prefix='visual_car_nav2_', suffix='.yaml', delete=False) as f:
        yaml.safe_dump(config, f)
        params = f.name
    extra_nodes = []
    if mode != 'radar':
        extra_nodes = [ExecuteProcess(cmd=['/usr/bin/python3', str(root.parent / 'nav2_map_publisher.py')], output='screen'),
                       Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
                            parameters=[{'yaml_filename': map_file, 'frame_id': 'map'}]),
                       Node(package='tf2_ros', executable='static_transform_publisher', name='visual_car_map_origin', output='screen',
                            arguments=['--x', '1.2', '--y', '0.3', '--z', '0', '--yaw', str(math.pi/2),
                                       '--frame-id', 'map', '--child-frame-id', 'odom'])]

    if mode == 'map':
        extra_nodes.append(ExecuteProcess(cmd=['/usr/bin/python3', str(root / 'boundary_map.py')], output='screen'))

    servers = [('nav2_controller', 'controller_server'),
               ('nav2_planner', 'planner_server'),
               ('nav2_bt_navigator', 'bt_navigator')]
    nodes = extra_nodes
    if window is not None:
        local = config['local_costmap']['local_costmap']['ros__parameters']
        nodes.insert(0, LogInfo(msg=(
            f'整图规划窗口：global {window["width"]}×{window["height"]} 米；'
            f'local {local["width"]}×{local["height"]} 米（保持配置）；'
            '全局代价图发布完整快照，真实地图边界与起点不变')))
    if mode != 'map':
        nodes.extend(radar_tf(context))
    for package, name in servers:
        overrides = ({'default_nav_to_pose_bt_xml': str(root / 'follow.xml'),
                      'default_nav_through_poses_bt_xml': str(root / 'follow_through_poses.xml')}
                     if name == 'bt_navigator' else {})
        nodes.append(Node(package=package, executable=name, name=name,
                          output='screen', parameters=[params, overrides], additional_env=plugin_env,
                          remappings=[('cmd_vel', '/visual_car/nav2_cmd_vel')]))
    manager = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                   name='visual_car_nav2_lifecycle', output='screen',
                   sigterm_timeout='3', sigkill_timeout='3',
                   parameters=[{'autostart': True, 'bond_timeout': 4.0,
                                'node_names': [name for _, name in servers]}])
    watchdog = ExecuteProcess(cmd=['/usr/bin/python3', str(root / 'startup_watchdog.py')], output='screen')
    # 【职责 / R01 R13 R21】watched_exit：监控被管理进程退出并联动启动结果。
    def watched_exit(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Nav2 启动未推进；查看 startup_watchdog 的节点状态'))]
        return []
    nodes.append(RegisterEventHandler(OnProcessExit(target_action=watchdog, on_exit=watched_exit)))
    if mode != 'radar':
        startup = ExecuteProcess(cmd=['/usr/bin/python3', str(root / 'map_startup.py')], output='screen')
        # 【职责 / R01 R13 R21】map_ready：地图就绪后继续后续导航启动。
        def map_ready(event, context):
            if event.returncode == 0:
                return [manager, watchdog]
            return [EmitEvent(event=Shutdown(reason='地图启动失败，停止 Nav2；查看 map_startup 日志'))]
        nodes.extend([RegisterEventHandler(OnProcessExit(target_action=startup, on_exit=map_ready)), startup])
    else:
        nodes.extend([manager, watchdog])
    return nodes


# 【职责 / R01 R13 R21】generate_launch_description：组装启动过程、就绪检查和退出处理。
def generate_launch_description():
    root = Path(__file__).resolve().parent
    nodes = []
    for name, default in (
        ('params_file', str(root / 'params.yaml')),
        ('camera_x', '0'), ('camera_y', '0'), ('camera_yaw', '0'),
        ('radar_frame', 'ti_mmwave_0'), ('publish_radar_tf', 'true'),
        ('radar_calibration', str(root.parent / 'data/data_transfer/20260920_160622_946624.json')),
    ):
        nodes.append(DeclareLaunchArgument(name, default_value=default))
    nodes.append(OpaqueFunction(function=navigation_nodes))
    return LaunchDescription(nodes)
