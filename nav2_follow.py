# 【内容标注】用途：Nav2 与原运动控制的导航接入层。
# 对应用户需求：R11 R13 R16 R17 R18 R19 R22 R31 R32 R33 R34（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：分离规划/复核/执行；管理取消代际、传感器时效、连续分段目标和保曲率速度输出；不另建底盘发布通道。
# 需求关联用于追溯用途，不代表精确创建/提交记录；后续自检修复见 nav2/AUDIT.md。
"""Nav2 moving-goal adapter. Nav2 never publishes directly to the chassis."""
from __future__ import annotations

import math
import logging
import os
import threading
import time
from collections import deque
from nav2.motion_geometry import person_position, limit_twist, person_stopping_candidates, CurvatureRamp
from nav2.replan_support import NearObstacle, recovery, check_vision as check_visual_health, guard_command


# 【职责 / R11 R13 R16 R17 R18 R19 R22】VisionStale：标识视觉数据过期异常，供调用方区分短暂等待和其他故障。
class VisionStale(RuntimeError):
    """No fresh visual obstacle measurement; distinct from a detected hazard."""
    def __init__(self, message, *, source_at=None):
        super().__init__(message)
        # R34: keep the actual expired evidence's timestamp. A newer status
        # stamp must not extend the grace for an older full-cloud snapshot.
        self.source_at = source_at


# 【职责 / R11 R13 R16 R17 R18 R19 R22】StableGoalFilter：累计连续人物世界坐标，检查三帧跨度和位置离散程度。
class StableGoalFilter:
    """Require a compact sequence of odom goals before using its mean."""

    # 【职责 / R11 R13 R16 R17 R18 R19 R22】__init__：初始化本类依赖与状态；副作用以原初始化语句为准。
    def __init__(self):
        self.samples = deque(maxlen=3)

    # 【职责 / R11 R13 R16 R17 R18 R19 R22】update：剔除间隔过大或位置跳变的旧样本，三帧稳定后返回平均位置/方向；不发送导航任务。
    def update(self, goal, now):
        if self.samples and (now - self.samples[-1][0] > 1.2 or
                any(math.hypot(goal[0] - g[0], goal[1] - g[1]) > 0.35
                    for _, g in self.samples)):
            self.samples.clear()
        self.samples.append((now, goal))
        if len(self.samples) < 3 or now - self.samples[0][0] < 0.2:
            return None
        return (sum(g[0] for _, g in self.samples) / 3,
                sum(g[1] for _, g in self.samples) / 3,
                math.atan2(sum(math.sin(g[2]) for _, g in self.samples),
                           sum(math.cos(g[2]) for _, g in self.samples)))


# 【职责 / R11 R13 R16 R17 R18 R19 R22】follow_goal：保留跟随距离并限制单段推进 3 米；连续模式使用精确 1 米等待阈值。
def follow_goal(distance, bearing, stand_off, robot_x, robot_y, robot_yaw,
                camera_x=0.0, camera_y=0.0, camera_yaw=0.0, *, tolerance=0.1):
    """Return an odom goal on the robot-to-person ray, or None inside stand-off."""
    values = (distance, bearing, stand_off, robot_x, robot_y, robot_yaw,
              camera_x, camera_y, camera_yaw)
    if not all(math.isfinite(v) for v in values) or distance <= 0 or stand_off <= 0:
        raise ValueError("Invalid follow goal geometry")
    x = camera_x + distance * math.cos(camera_yaw + bearing)
    y = camera_y + distance * math.sin(camera_yaw + bearing)
    radius = math.hypot(x, y)
    if radius <= stand_off + tolerance:
        return None
    heading = robot_yaw + math.atan2(y, x)
    travel = min(radius - stand_off, 3.0)  # Stay inside the rolling planning map.
    return (robot_x + travel * math.cos(heading),
            robot_y + travel * math.sin(heading), heading)


# 【职责 / R11 R13 R16 R17 R18 R19 R22】Nav2Follower：Nav2 与原运动控制的导航接入层的状态封装；各方法职责见下方标注。
class Nav2Follower:
    # 【职责 / R11 R13 R16 R17 R18 R19 R22】__init__：初始化本类依赖与状态；副作用以原初始化语句为准。
    def __init__(self, node, radar_topic, camera_pose, radar_frame="ti_mmwave_0"):
        from geometry_msgs.msg import Twist
        from nav2_msgs.action import NavigateToPose, ComputePathToPose, FollowPath
        from rclpy.action import ActionClient
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2
        from tf2_ros import Buffer
        from nav2.pose_listener import PoseListener
        self.obstacle_mode = os.environ.get('NAV2_OBSTACLE_MODE', 'radar')
        if self.obstacle_mode not in ('radar', 'map', 'both'):
            raise ValueError('Invalid NAV2_OBSTACLE_MODE')
        self.velocity_ramp = CurvatureRamp()
        self.vision_enabled = os.environ.get('NAV2_VISION_OBSTACLES', '0') == '1'
        self.vision_at = 0.0
        self.vision_error = '等待视觉障碍数据'
        self.vision_count = 0
        self.vision_near_blocked = False
        self.vision_safety = None
        self.node = node
        self.action_type = NavigateToPose
        self.client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        self.preview_type = ComputePathToPose
        self.preview_client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        self.path_action_type = FollowPath
        self.path_client = ActionClient(node, FollowPath, '/follow_path')
        from nav2.person_navigation import PersonNavigation
        self.person_navigation = PersonNavigation(self)
        self.preview_sequence = 0
        self.preview_handle = None
        self.preview = {'state': 'idle', 'message': '保持强停，识别目标后可预览路径', 'points': []}
        self.preview_at = 0.0
        self.tf = Buffer()
        self.listener = PoseListener(node, self.tf)
        self.camera_pose = camera_pose
        self.expected_radar_frame = radar_frame
        self.lock = threading.RLock()
        self.handle = None
        self.pending = False
        self.canceling = False
        self.generation = 0
        self.enabled = False
        self.fixed_goal_active = False
        self.command = (0.0, 0.0)
        self.command_at = self.radar_at = self.goal_at = 0.0
        self.radar_frame = ''
        self.goal = None
        self.person_anchor = None
        self.goal_filter = StableGoalFilter()
        self.error = '等待雷达、TF 和 Nav2'
        self.sub = node.create_subscription(Twist, '/visual_car/nav2_cmd_vel', self._command, 1)
        self.radar_sub = node.create_subscription(PointCloud2, radar_topic, self._radar,
                                                  qos_profile_sensor_data)
        self.preview_timer = node.create_timer(0.1, self._preview_timeout)

    # 【职责 / R11 R13 R16 R17 R18 R19 R22】clear_preview：递增预览序号并取消旧预览，阻止迟到响应覆盖新状态。
    def clear_preview(self):
        with self.lock:
            self.preview_sequence += 1
            if self.preview_handle is not None:
                self.preview_handle.cancel_goal_async()
                self.preview_handle = None
            self.preview_candidates = []
            self.preview = {'state': 'idle', 'message': '预览已清除，请重新计算', 'points': []}

    # 【职责 / R11 R13 R16 R17 R18 R19 R22】_preview_timeout：监管预览和人物规划期限；超时使请求失效。
    def _preview_timeout(self):
        if hasattr(self, 'person_navigation'):
            self.person_navigation.tick()
        with self.lock:
            self._execution_timeout()
            if self.preview['state'] == 'pending' and time.monotonic() - self.preview_at > 5:
                self.clear_preview()
                self.preview.update(state='error', message='规划超时：5 秒内未返回路径')

    # 【职责 / R18 R22】_execution_timeout：执行接受超时失效旧任务；未确认取消前保留隔离，不抢发新路线。
    def _execution_timeout(self):
        person = getattr(self, 'person_navigation', None)
        if (self.pending and self.enabled and not getattr(person, 'busy', False)
                and time.monotonic() - self.goal_at > 5.):
            message = '路线执行服务 5 秒未确认，已停车；等待旧请求取消确认，持续无响应请重启服务'
            self.stop(message)
            self.error = message
            if person is not None:
                person.state = 'error'; person.message = message
            # Keep pending=True: the server may still accept the old request.
            # A late acceptance must go through generation rejection/cancellation
            # before another action can share the untagged cmd_vel topic.

    # 【职责 / R11 R13 R16 R17 R18 R19 R22】preview_path：人物距离/方位生成停车预览，保留跟随距离。
    def preview_path(self, distance, bearing, stand_off):
        """Planner-only action: never sends NavigateToPose, FollowPath or velocity."""
        with self.lock:
            self.clear_preview()
            try:
                pose = self.healthy_pose()
                goal = follow_goal(distance, bearing, stand_off, *pose, *self.camera_pose)
                if goal is None:
                    self.preview.update(state='near', message='目标已在跟随距离内，无需前进')
                    return
                person = person_position(distance, bearing, *pose, *self.camera_pose)
                self.preview_candidates = person_stopping_candidates(goal, person, pose)[1:]
                self._request_preview(goal)
            except Exception as exc:
                self.preview.update(state='error', message=str(exc))

    # 【职责 / R11 R13 R16 R17 R18 R19 R22】preview_person_point：保存点击时的 odom 历史人物点，计算保持距离的停车候选；只规划不授权运动。
    def preview_person_point(self, point, stand_off):
        """R31: latch an odom observation for planning only.

        Execution requires a separate explicit start and complete path validation.

        Unlike a live marker this explicit click snapshot survives missing frames.
        Using its capture-time world position avoids reprojecting an old bearing
        with the robot's current pose. Person occupancy remains in the costmap.
        """
        with self.lock:
            self.person_target_marker = dict(point, position=list(point['position']),
                                             selected_at=time.monotonic())
            self.person_locked_target = self.person_target_marker
            self.clear_preview()
            self.person_marker_sequence = self.preview_sequence
            try:
                rx, ry, yaw = pose = self.healthy_pose()
                px, py = point['position']
                goal = follow_goal(math.hypot(px-rx,py-ry),
                                   math.atan2(py-ry,px-rx)-yaw,stand_off,*pose)
                if goal is None:
                    self.preview.update(state='near',message='已记录人物历史点；目标在跟随距离内，无需前进')
                    return
                self.preview_candidates = person_stopping_candidates(goal,(px,py),pose)[1:]
                self._request_preview(goal)
            except Exception as exc:
                self.preview.update(state='error',message=str(exc),points=[])

    # 【职责 / R11 R13 R16 R17 R18 R19 R22】preview_goal：人工地图终点预览，不替换成其他人物候选。
    def preview_goal(self, x, y):
        """Explicit map-point planner request, without navigation execution."""
        with self.lock:
            self.clear_preview()
            try:
                if not all(math.isfinite(v) for v in (x,y)):
                    raise ValueError('目标坐标无效')
                rx, ry, _ = self.healthy_pose()
                self._request_preview((x,y,math.atan2(y-ry,x-rx)))
            except Exception as exc:
                self.preview.update(state='error', message=str(exc), points=[])

    # 【职责 / R11 R13 R16 R17 R18 R19 R22】_request_preview：发送 ComputePathToPose；候选重试共用最初截止时间。
    def _request_preview(self, goal, retry=False):
        if not self.preview_client.server_is_ready():
            raise RuntimeError('Nav2 规划服务尚未就绪')
        request = self.preview_type.Goal()
        request.goal.header.frame_id = 'odom'
        request.goal.header.stamp = self.node.get_clock().now().to_msg()
        request.goal.pose.position.x, request.goal.pose.position.y = goal[:2]
        request.goal.pose.orientation.z = math.sin(goal[2]/2)
        request.goal.pose.orientation.w = math.cos(goal[2]/2)
        request.planner_id = 'GridBased'
        request.use_start = False
        self.preview.update(state='pending', message='正在计算停车预览路径')
        if not retry:
            self.preview_at = time.monotonic()
        sequence = self.preview_sequence
        self.preview_client.send_goal_async(request).add_done_callback(
            lambda f: self._preview_accepted(f, sequence))

    # 【职责 / R11 R13 R16 R17 R18 R19 R22】_preview_accepted：登记规划句柄；过期预览立即取消。
    def _preview_accepted(self, future, sequence):
        with self.lock:
            try:
                handle = future.result()
                if sequence != self.preview_sequence:
                    if handle.accepted:
                        handle.cancel_goal_async()
                    return
                if not handle.accepted:
                    raise RuntimeError('规划服务拒绝预览请求')
                self.preview_handle = handle
                handle.get_result_async().add_done_callback(lambda f: self._preview_result(f, sequence))
            except Exception as exc:
                if sequence == self.preview_sequence:
                    self.preview.update(state='error', message=str(exc), points=[])

    # 【职责 / R11 R13 R16 R17 R18 R19 R22】_preview_result：将较重路径复核交给后台线程，避免阻塞 ROS 回调。
    def _preview_result(self, future, sequence):
        # Do not run footprint rasterization on the ROS executor or under nav.lock.
        threading.Thread(target=self._validate_preview_result,
                         args=(future, sequence), daemon=True).start()

    # 【职责 / R11 R13 R16 R17 R18 R19 R22】_validate_preview_result：后台复核返回路线；有限尝试人物备选停车点，成功才记录可用完整路径。
    def _validate_preview_result(self, future, sequence):
        with self.lock:
            if sequence != self.preview_sequence:
                return
            self.preview_handle = None
        try:
            response = future.result()
            if response.status == 6:
                with self.lock:
                    if sequence != self.preview_sequence:
                        return
                    candidates = getattr(self, 'preview_candidates', [])
                    if candidates and time.monotonic()-self.preview_at < 5.:
                        self._request_preview(candidates.pop(0), retry=True)
                        self.preview['message'] = '原停车点不可达，正在尝试人物附近其他停车点'
                        return
            if response.status != 4:
                code = getattr(response.result, 'error_code', None)
                raise RuntimeError(f'规划失败（状态 {response.status}，错误码 {code}）；请查看 Nav2 日志')
            path = response.result.path
            if path.header.frame_id != 'odom' or not path.poses:
                raise RuntimeError('规划返回空路径或坐标系不是 odom')
            validator = getattr(self, 'validate_preview', None)
            if validator is not None:
                validator(path)
            step = max(1, math.ceil(len(path.poses)/600))
            points = [[p.pose.position.x, p.pose.position.y] for p in path.poses[::step]]
            points.append([path.poses[-1].pose.position.x, path.poses[-1].pose.position.y])
            if not all(math.isfinite(v) for p in points for v in p):
                raise RuntimeError('规划路径包含无效坐标')
            with self.lock:
                if sequence != self.preview_sequence:
                    return
                self.preview_full_path = path
                self.preview_at = time.monotonic()
                self.preview.update(state='ready', message='预览完成，尚未执行运动', points=points)
        except Exception as exc:
            with self.lock:
                if sequence == self.preview_sequence:
                    self.preview.update(state='error', message=str(exc), points=[])

    # 【职责 / R11 R13 R16 R17 R18 R19 R22】_command：接收 Nav2 速度但仅在有效动作中保存，不直接发布底盘指令。
    def _command(self, msg):
        with self.lock:
            if self.enabled and self.handle is not None and not self.canceling:
                values = (msg.linear.x, msg.angular.z)
                if all(math.isfinite(v) for v in values):
                    self.command = values
                    self.command_at = time.monotonic()

    # 【职责 / R11 R13 R16 R17 R18 R19 R22】_radar：检查雷达坐标系和源时间，地图单独模式不依赖雷达。
    def _radar(self, msg):
        if getattr(self, 'obstacle_mode', 'radar') == 'map':
            return
        # Receipt time alone must not make replayed/stale sensor data fresh.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self.node.get_clock().now().nanoseconds * 1e-9 - stamp
        with self.lock:
            if msg.header.frame_id != self.expected_radar_frame:
                self.radar_at = 0.0
                self.error = f'雷达坐标系不匹配：期望 {self.expected_radar_frame}，收到 {msg.header.frame_id}'
                self.stop(self.error)
                return
            if -0.1 <= age <= 0.6 and msg.header.frame_id:
                self.radar_at = time.monotonic()
                self.radar_frame = msg.header.frame_id

    # 【职责 / R11 R13 R16 R17 R18 R19 R22】check_vision：视觉已启用时检查障碍错误及源帧年龄，区分暂时过期与其他故障。
    def check_vision(self):
        task = recovery(self)
        # R32: only a revalidated detour may use directional whole-body checks;
        # sensor/TF/config errors never receive this narrow near-field exemption.
        check_visual_health(self, allow_near=bool(task and task.permits_near))

    # 【职责 / R11 R13 R16 R17 R18 R19 R22】healthy_pose：同时检查配置、视觉/雷达时效和 TF，返回新鲜车位；失败阻止导航。
    def healthy_pose(self, check_vision=True):
        if getattr(self, "inflation_config_error", ""):
            raise RuntimeError(self.inflation_config_error)
        if check_vision:
            self.check_vision()
        if self.camera_pose is None:
            raise RuntimeError('未配置相机相对 base_link 的 x、y、yaw；历史高度 0.5 m 不能代替安装外参')
        from rclpy.time import Time
        if getattr(self, 'obstacle_mode', 'radar') != 'map' and time.monotonic() - self.radar_at > 0.6:
            raise RuntimeError('雷达数据超时')
        transform = self.tf.lookup_transform('odom', 'base_link', Time())
        stamp = transform.header.stamp
        age = (self.node.get_clock().now().nanoseconds * 1e-9
               - stamp.sec - stamp.nanosec * 1e-9)
        if not -0.1 <= age <= 0.5:
            raise RuntimeError(f'里程计 TF 超时：源时间差 {age:.3f} 秒（允许 -0.100～0.500 秒）')
        if getattr(self, 'obstacle_mode', 'radar') != 'map':
            self.tf.lookup_transform('base_link', self.radar_frame, Time())
        t, q = transform.transform.translation, transform.transform.rotation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        return t.x, t.y, yaw

    # 【职责 / R11 R13 R16 R17 R18 R19 R22】stop：清零速度、失效旧任务并取消动作；连续追踪异常后锁住续段。
    def stop(self, reason='接入层请求停止'):
        with self.lock:
            task = recovery(self)
            recovery_active = bool(task is not None and task.active)
            if task is not None:
                task.invalidate()  # Any ordinary stop revokes automatic recovery.
            if getattr(self,'velocity_ramp',None) is not None:self.velocity_ramp.reset()
            # Repeated idle stop notifications are not new navigation transitions.
            invalidate = recovery_active or self.enabled or self.pending or self.handle is not None or self.canceling
            if getattr(self,'continuous_follow',False) and invalidate:
                self.continuous_blocked = True  # Fault/stop must not silently replan.
            self.fixed_goal_active = False
            if self.enabled or (self.pending and not getattr(self, 'stop_recorded_pending', False)):
                self.last_stop_reason = reason
                self.last_stop_at = time.monotonic()
                self.stop_recorded_pending = True
                logging.getLogger(__name__).warning('Nav2 停止/取消：%s', reason)
            if hasattr(self, 'person_navigation'):
                self.person_navigation.cancel()
            self.person_anchor = None
            self.goal_filter = StableGoalFilter()
            self.enabled = False
            self.command_at = 0.0
            self.command = (0.0, 0.0)
            self.goal = None
            if invalidate:
                self.generation += 1
            if self.handle is not None and not self.canceling:
                self.canceling = True
                try:
                    self.handle.cancel_goal_async().add_done_callback(self._canceled)
                except Exception as exc:
                    # Local zero/invalidation already took effect. Keep the
                    # handle quarantined until its terminal result is known.
                    self.error = '导航取消通信失败，保持停车等待终态：' + str(exc)

    # 【职责 / R11 R13 R16 R17 R18 R19 R22】_canceled：处理取消回应；终态回来前不抢发替代任务。
    def _canceled(self, future):
        with self.lock:
            try:
                response = future.result()
                if not response.goals_canceling:
                    # Wait for the terminal result; do not permit a new goal yet.
                    return
            except Exception as exc:
                self.error = str(exc)
                return
            # The result callback is the terminal cancellation acknowledgement.

    # 【职责 / R11 R13 R16 R17 R18 R19 R22】_result：处理终态；到达后清空旧滤波样本，开启下一段的新帧门槛。
    def _result(self, future, handle):
        with self.lock:
            if self.handle is not handle:
                return
            task = recovery(self)
            if task is not None and task.execution_result(future, handle):
                return  # Old canceled action is not an arrival of the new route.
            if task is not None:
                task.invalidate()
            if self.handle is handle:
                self.fixed_goal_active = False
                self.enabled = False
                self.handle = None
                self.canceling = False
                self.command_at = 0.0
                self.goal = None
            try:
                result = future.result()
                if result.status != 4 and getattr(self,'continuous_follow',False):
                    self.continuous_blocked = True
                if result.status not in (4, 5):
                    self.error = f'Nav2 任务结束，状态 {result.status}'
                person = getattr(self, 'person_navigation', None)
                if person is not None and person.state == 'executing':
                    single_locked = bool(person.locked_segment and not person.locked_segment['continuous'])
                    person.finish_locked_segment()
                    person.state = 'arrived' if result.status == 4 else 'error'
                    person.message = ('已抵达历史人物路线终点，本次追踪结束' if single_locked else
                                      '已抵达本段终点，重新确认所选人物（等待到达后 3 帧）') if result.status == 4 else f'人物路线执行结束：状态 {result.status}'
                    if result.status == 4:
                        # Do not reuse observations accumulated while traversing the old path.
                        person.arrived_at = time.monotonic()
                        self.person_locked_target = None
                        self.goal_filter = StableGoalFilter()
                        self.person_anchor = None
                        self.command = (0., 0.)
                        self.error = person.message
            except Exception as exc:
                # A failed result transport is not a confirmed terminal state.
                # Restore the handle for cancellation/quarantine; allowing a new
                # action here could mix an old controller's untagged velocities.
                self.handle = handle
                message = '导航终态读取失败，保持停车等待取消确认：' + str(exc)
                self.stop(message)
                self.error = message
                person = getattr(self, 'person_navigation', None)
                if person is not None:
                    person.state = 'error'; person.message = message

    # 【职责 / R11 R13 R16 R17 R18 R19 R22】_accepted：接收动作句柄；对迟到任务检查代际并取消，避免停止后复活。
    def _accepted(self, future, generation):
        with self.lock:
            self.pending = False
            try:
                handle = future.result()
                if not handle.accepted:
                    task = recovery(self)
                    if task is not None and task.holding:
                        # A rejected old request has no live controller; the
                        # recovery timer may now plan, while all output stays zero.
                        self.enabled = False
                        self.fixed_goal_active = True
                        return
                    if task is not None:
                        task.invalidate()
                    if getattr(self,'continuous_follow',False):self.continuous_blocked=True
                    self.fixed_goal_active = False
                    self.enabled = False
                    self.goal = None
                    self.error = 'Nav2 拒绝目标'
                    person = getattr(self, 'person_navigation', None)
                    if person is not None and generation == self.generation:
                        person.finish_locked_segment()
                        person.state = 'error'
                        person.message = self.error
                    return
                self.handle = handle
                handle.get_result_async().add_done_callback(lambda f: self._result(f, handle))
                if (self.handle is handle and not self.canceling
                        and (generation != self.generation or not self.enabled)):
                    # A ready result callback may already have completed this
                    # handle. Never re-enter canceling after its terminal state.
                    self.canceling = True
                    handle.cancel_goal_async().add_done_callback(self._canceled)
            except Exception as exc:
                # Transport failure is not a rejection: the server may have
                # accepted. Do not permit a replacement to race that action.
                if self.handle is None:
                    self.pending = True
                message = '导航执行应答失败，保持停车等待确认：' + str(exc)
                self.stop(message)
                self.error = message
                person = getattr(self, 'person_navigation', None)
                if person is not None:
                    person.state = 'error'; person.message = message

    # 【职责 / R11 R13 R16 R17 R18 R19 R22】update：仅在允许时获取下一段目标；连续模式途中冻结终点，到达后要求新帧；普通模式保持原更新规则。
    def update(self, distance, bearing, stand_off):
        """Called with a fresh, frame-matched visual measurement, never PID yaw."""
        with self.lock:
            try:
                task = recovery(self)
                if task is not None and task.holding:
                    return  # Recovery owns the old endpoint; no live-person retarget.
                # R31: the explicit historical segment owns this route until its
                # terminal state. New or missing person measurements cannot replan it.
                person = getattr(self, 'person_navigation', None)
                if getattr(person, 'locked_segment', None) is not None:
                    reason = person.locked_segment_error()
                    if reason:
                        self.error = reason
                        self.stop(reason)
                    return
                continuous = getattr(self,'continuous_follow',False)
                if continuous:
                    motion=getattr(self,'person_motion',None)
                    if motion is None or not motion.following or motion.estop or motion.mode!='auto':
                        return
                    if motion.settings.target_id != getattr(self,'continuous_target_id',motion.settings.target_id):
                        self.stop('连续追踪人物 ID 已改变，请重新启动')
                        self.continuous_blocked=True
                    if getattr(self,'continuous_blocked',False):
                        self.error='连续追踪已因停止或异常暂停，请重新规划并开始追踪'
                        return
                if continuous and (self.pending or self.canceling or self.handle is not None):
                    # Execute the latched segment. No target localization/filtering
                    # or goal changes until the action reports arrival.
                    # The 50 Hz velocity gate still checks TF and obstacle data.
                    # Checking here first would cancel at 1.2s vision age, bypassing
                    # velocity()'s existing zero-output grace until age 2s.
                    return
                pose = self.healthy_pose()
                if continuous:stand_off=1.0
                person = getattr(self, 'person_navigation', None)
                arrived_at = getattr(person, 'arrived_at', None)
                if arrived_at is not None:
                    # A frame delivered after arrival may still have been captured before it.
                    motion = getattr(self, 'person_motion', None)
                    measurement = getattr(motion, 'nav_measurement', None)
                    if not measurement or measurement[2] <= arrived_at:
                        self.error = '已抵达本段终点，等待到达后采集的人物数据'
                        return
                goal = follow_goal(distance, bearing, stand_off, *pose, *self.camera_pose, tolerance=0. if continuous else .1)
                if goal is None:
                    self.error = '目标已在跟随距离内，无需前进'
                    if continuous:
                        self.enabled=False;self.command_at=0.;self.command=(0.,0.)
                        self.goal_filter=StableGoalFilter()
                        if person is not None:
                            person.state='waiting';person.message='连续追踪：人物在 1 米内，等待离开后继续'
                    else:self.stop(self.error)
                    return
                # Filter the actual person in odom, not the robot-relative stopping point.
                px, py = person_position(distance, bearing, *pose, *self.camera_pose)
                target = self.goal_filter.update((px, py, 0.), time.monotonic())
                if target is None:
                    if self.handle is None and not self.pending:
                        self.error = '等待连续稳定的目标位置（3 帧）'
                    return
                # A click pins the world-space reference, not the measurement's
                # freshness. Original live-person gates still run before update.
                locked = getattr(self, 'person_locked_target', None)
                motion = getattr(self, 'person_motion', None)
                if locked and motion is not None:
                    if (locked['id'] == motion.settings.target_id
                            and time.monotonic()-locked['selected_at'] <= 30.
                            and math.hypot(target[0]-locked['position'][0],target[1]-locked['position'][1]) <= .5):
                        target = (*locked['position'],0.)
                    else:
                        self.person_locked_target = None
                now = time.monotonic()
                anchor = getattr(self, 'person_anchor', None)
                moved = anchor is not None and math.hypot(target[0]-anchor[0], target[1]-anchor[1]) > .5
                if self.pending or self.canceling:
                    person = getattr(self, 'person_navigation', None)
                    if (person is not None and person.busy and moved):
                        self.stop('规划期间人物终点已移动，重新规划')
                    return
                if self.handle is not None:
                    if moved and now-self.goal_at >= 2.0:
                        self.stop('稳定目标移动超过 0.5 米，更新导航目标')  # Serialize cancel -> terminal result -> replacement.
                    return
                if now - self.goal_at < 1.0:
                    return
                # Generate a new approach endpoint only when starting a new task.
                dx, dy = target[0]-pose[0], target[1]-pose[1]
                goal = follow_goal(math.hypot(dx, dy), math.atan2(dy, dx)-pose[2], stand_off, *pose, tolerance=0. if continuous else .1)
                if goal is None:
                    self.error = '目标已在跟随距离内，无需前进'
                    self.stop(self.error)
                    return
                self.person_anchor = target[:2]
                candidates = person_stopping_candidates(goal, target, pose)
                self.person_navigation.request(goal, alternatives=candidates[1:])
            except Exception as exc:
                self.error = str(exc)
                self.stop(self.error)

    # 【职责 / R11 R13 R16 R17 R18 R19 R22】hold_for_visual_update：暂时等待新视觉帧时清零输出并检查定位。
    def hold_for_visual_update(self):
        """Keep the action briefly, but invalidate output and check sensors."""
        with self.lock:
            try:
                self.healthy_pose()
            except Exception as exc:
                self.error = str(exc)
                self.stop(self.error)
            self.command_at = 0.0
            self.command = (0.0, 0.0)

    # 【职责 / R11 R13 R16 R17 R18 R19 R22】pause_for_vision：沿用短暂过期停车等待；持续超时取消，不拿旧帧放行。
    def pause_for_vision(self, *, source_at=None):
        """Zero output now; retain fixed-goal action only until source age reaches 2s."""
        self.command_at = 0.0
        self.command = (0.0, 0.0)
        self.vision_paused = True
        self.error = '视觉障碍数据超时，已停车等待（最长至数据年龄 2 秒）'
        try:
            self.healthy_pose(check_vision=False)
        except Exception as exc:
            self.error = str(exc)
            self.stop(self.error)
            return
        stamp = self.vision_at
        if source_at is not None:
            stamp = min(stamp, source_at)
        age = time.monotonic()-stamp
        if (not getattr(self, 'fixed_goal_active', False) or stamp <= 0
                or not math.isfinite(age) or not 0 <= age <= 2.0):
            self.stop('视觉障碍数据持续超时，取消导航')

    # 【职责 / R11 R13 R16 R17 R18 R19 R22】velocity：先检查传感器和指令时效，再限幅并保曲率加速；任何停车输出立即清零。
    def velocity(self):
        with self.lock:
            active=self.enabled
            def report(reason, output):
                if output==(0.,0.) and getattr(self,'velocity_ramp',None) is not None:self.velocity_ramp.reset()
                now=time.monotonic()
                if active and now-getattr(self,'_velocity_log_at',0.)>=2.:
                    self._velocity_log_at=now
                    logging.getLogger(__name__).warning(
                        '导航速度检查：%s；控制器 v=%.3f w=%.3f，输出 v=%.3f w=%.3f；'
                        '指令年龄 %.3f 秒，视觉年龄 %.3f 秒',
                        reason,*self.command,*output,now-self.command_at,
                        now-getattr(self,'vision_at',now))
                return output
            task = recovery(self)
            if task is not None and task.holding:
                # The timer checks authorization/health; never forward old cmd_vel
                # during cancellation, planning or lock-free geometry validation.
                return report('障碍绕行复核中，保持停车', (0., 0.))
            # R31: the original fixed-route loop skips person freshness, but
            # authorization/ID/stream changes must still stop this exact segment.
            person = getattr(self, 'person_navigation', None)
            segment = getattr(person, 'locked_segment', None)
            if segment is not None:
                if segment.get('finished'):
                    return report('历史人物本段已结束', (0., 0.))
                reason = person.locked_segment_error()
                if reason:
                    self.error = reason
                    self.stop(reason)
                    return report(reason, (0., 0.))
            try:
                pose = self.healthy_pose()
                if getattr(self, 'vision_paused', False):
                    # R34: a newer health stamp alone cannot recover an older
                    # full-cloud pause. Check the evidence before lifting it.
                    if task is not None and task.active:
                        from nav2.replan_support import visual_snapshot
                        visual_snapshot(self)
                    self.vision_paused = False
                    self.command_at = 0.0
                    self.command = (0.0, 0.0)
                    self.error = ''
                    return report('视觉恢复，等待新控制指令',(0.,0.))
            except NearObstacle as exc:
                if not task or not task.trigger(str(exc)):
                    self.stop(str(exc))
                return report(str(exc), (0., 0.))
            except VisionStale as exc:
                self.pause_for_vision(source_at=exc.source_at)
                return report('视觉数据过期',(0.,0.))
            except Exception as exc:
                self.error = str(exc)
                self.stop(self.error)
            if not self.enabled or time.monotonic() - self.command_at > 0.3:
                return report('任务停止或速度指令超过 0.3 秒',(0.,0.))
            # Bound travel while using the Nav2-specific visual latency budget.
            linear, yaw = self.command
            # Respect downstream user limits before the original controller clips them.
            motion = getattr(self, 'person_motion', None)
            settings = getattr(motion, 'settings', None)
            output=limit_twist(linear, yaw,
                min(.18, getattr(settings, 'max_forward_mps', .18)),
                min(.15, getattr(settings, 'max_reverse_mps', .15)),
                min(.4, getattr(settings, 'max_yaw_radps', .4)))

            ramp=getattr(self,'velocity_ramp',None)
            if ramp is not None:output=ramp.apply(*output,time.monotonic())
            try:
                if not guard_command(self, pose, output):
                    return report('新障碍阻挡整车扫掠，已停车', (0., 0.))
            except VisionStale as exc:
                # R34: the frame can age after healthy_pose(), while the full
                # swept-body check runs. Use the same zero/wait rule here too.
                self.pause_for_vision(source_at=exc.source_at)
                return report('视觉扫掠数据过期，停车等待', (0., 0.))
            except Exception as exc:
                self.stop(str(exc))
                return report(str(exc), (0., 0.))
            return report('速度放行（保曲率加速过渡）',output)

    # 【职责 / R11 R13 R16 R17 R18 R19 R22】status：向网页提供导航、预览和故障诊断快照。
    def status(self):
        with self.lock:
            return {'nav2_person_navigation': ({'state': self.person_navigation.state,
                        'message': self.person_navigation.message} if hasattr(self, 'person_navigation') else {}),
                    'nav2_fixed_goal_active': getattr(self, 'fixed_goal_active', False),
                    'nav2_vision_enabled': getattr(self, 'vision_enabled', False),
                    'nav2_vision_error': getattr(self, 'vision_error', ''),
                    'nav2_vision_age': round(time.monotonic()-getattr(self, 'vision_at', 0), 2),
                    'nav2_vision_points': getattr(self, 'vision_count', 0),
                    'nav2_enabled': True, 'nav2_obstacle_mode': self.obstacle_mode, 'nav2_error': self.error,
                    'nav2_last_stop_reason': getattr(self, 'last_stop_reason', ''),
                    'nav2_last_stop_age': (round(time.monotonic() - self.last_stop_at, 1)
                                          if hasattr(self, 'last_stop_at') else None),
                    'nav2_goal_pending': self.pending,
                    'nav2_canceling': self.canceling,
                    'nav2_command_fresh': time.monotonic() - self.command_at <= 0.3,
                    'nav2_preview': dict(self.preview, age=round(time.monotonic()-self.preview_at, 1)),
                    'nav2_goal_active': self.handle is not None and not self.canceling,
                    'nav2_radar_fresh': self.obstacle_mode == 'map' or time.monotonic() - self.radar_at <= 0.6}
