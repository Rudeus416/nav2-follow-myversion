"""Nav2 moving-goal adapter. Nav2 never publishes directly to the chassis."""
from __future__ import annotations

import math
import logging
import os
import threading
import time
from collections import deque


class VisionStale(RuntimeError):
    """No fresh visual obstacle measurement; distinct from a detected hazard."""


class StableGoalFilter:
    """Require a compact sequence of odom goals before using its mean."""

    def __init__(self):
        self.samples = deque(maxlen=3)

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


def follow_goal(distance, bearing, stand_off, robot_x, robot_y, robot_yaw,
                camera_x=0.0, camera_y=0.0, camera_yaw=0.0):
    """Return an odom goal on the robot-to-person ray, or None inside stand-off."""
    values = (distance, bearing, stand_off, robot_x, robot_y, robot_yaw,
              camera_x, camera_y, camera_yaw)
    if not all(math.isfinite(v) for v in values) or distance <= 0 or stand_off <= 0:
        raise ValueError("Invalid follow goal geometry")
    x = camera_x + distance * math.cos(camera_yaw + bearing)
    y = camera_y + distance * math.sin(camera_yaw + bearing)
    radius = math.hypot(x, y)
    if radius <= stand_off + 0.1:
        return None
    heading = robot_yaw + math.atan2(y, x)
    travel = min(radius - stand_off, 3.0)  # Stay inside the rolling planning map.
    return (robot_x + travel * math.cos(heading),
            robot_y + travel * math.sin(heading), heading)


class Nav2Follower:
    def __init__(self, node, radar_topic, camera_pose, radar_frame="ti_mmwave_0"):
        from geometry_msgs.msg import Twist
        from nav2_msgs.action import NavigateToPose, ComputePathToPose
        from rclpy.action import ActionClient
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2
        from tf2_ros import Buffer, TransformListener
        self.obstacle_mode = os.environ.get('NAV2_OBSTACLE_MODE', 'radar')
        if self.obstacle_mode not in ('radar', 'map', 'both'):
            raise ValueError('Invalid NAV2_OBSTACLE_MODE')
        self.vision_enabled = os.environ.get('NAV2_VISION_OBSTACLES', '0') == '1'
        self.vision_at = 0.0
        self.vision_error = '等待视觉障碍数据'
        self.vision_count = 0
        self.node = node
        self.action_type = NavigateToPose
        self.client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        self.preview_type = ComputePathToPose
        self.preview_client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        self.preview_sequence = 0
        self.preview_handle = None
        self.preview = {'state': 'idle', 'message': '保持强停，识别目标后可预览路径', 'points': []}
        self.preview_at = 0.0
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, node)
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
        self.goal_filter = StableGoalFilter()
        self.error = '等待雷达、TF 和 Nav2'
        self.sub = node.create_subscription(Twist, '/visual_car/nav2_cmd_vel', self._command, 1)
        self.radar_sub = node.create_subscription(PointCloud2, radar_topic, self._radar,
                                                  qos_profile_sensor_data)
        self.preview_timer = node.create_timer(0.1, self._preview_timeout)

    def clear_preview(self):
        with self.lock:
            self.preview_sequence += 1
            if self.preview_handle is not None:
                self.preview_handle.cancel_goal_async()
                self.preview_handle = None
            self.preview = {'state': 'idle', 'message': '预览已清除，请重新计算', 'points': []}

    def _preview_timeout(self):
        with self.lock:
            if self.preview['state'] == 'pending' and time.monotonic() - self.preview_at > 5:
                self.clear_preview()
                self.preview.update(state='error', message='规划超时：5 秒内未返回路径')

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
                self._request_preview(goal)
            except Exception as exc:
                self.preview.update(state='error', message=str(exc))

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

    def _request_preview(self, goal):
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
        self.preview_at = time.monotonic()
        sequence = self.preview_sequence
        self.preview_client.send_goal_async(request).add_done_callback(
            lambda f: self._preview_accepted(f, sequence))

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

    def _preview_result(self, future, sequence):
        with self.lock:
            if sequence != self.preview_sequence:
                return
            self.preview_handle = None
            try:
                response = future.result()
                if response.status != 4:
                    code = getattr(response.result, 'error_code', None)
                    raise RuntimeError(f'规划失败（状态 {response.status}，错误码 {code}）；请查看 Nav2 日志中的具体原因')
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
                self.preview_at = time.monotonic()
                self.preview.update(state='ready', message='预览完成，仅表示本次规划结果，未执行运动', points=points)
            except Exception as exc:
                self.preview.update(state='error', message=str(exc), points=[])

    def _command(self, msg):
        with self.lock:
            if self.enabled and self.handle is not None and not self.canceling:
                values = (msg.linear.x, msg.angular.z)
                if all(math.isfinite(v) for v in values):
                    self.command = values
                    self.command_at = time.monotonic()

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

    def check_vision(self):
        if getattr(self, 'vision_enabled', False):
            if self.vision_error:
                raise RuntimeError(self.vision_error)
            if time.monotonic() - self.vision_at > 1.2:
                raise VisionStale('视觉障碍数据超时，停车等待新数据')

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
            raise RuntimeError('里程计 TF 超时')
        if getattr(self, 'obstacle_mode', 'radar') != 'map':
            self.tf.lookup_transform('base_link', self.radar_frame, Time())
        t, q = transform.transform.translation, transform.transform.rotation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        return t.x, t.y, yaw

    def stop(self, reason='接入层请求停止'):
        with self.lock:
            self.fixed_goal_active = False
            if self.enabled or (self.pending and not getattr(self, 'stop_recorded_pending', False)):
                self.last_stop_reason = reason
                self.last_stop_at = time.monotonic()
                self.stop_recorded_pending = True
                logging.getLogger(__name__).warning('Nav2 停止/取消：%s', reason)
            self.goal_filter = StableGoalFilter()
            self.enabled = False
            self.command_at = 0.0
            self.command = (0.0, 0.0)
            self.goal = None
            self.generation += 1
            if self.handle is not None and not self.canceling:
                self.canceling = True
                self.handle.cancel_goal_async().add_done_callback(self._canceled)

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

    def _result(self, future, handle):
        with self.lock:
            if self.handle is handle:
                self.fixed_goal_active = False
                self.enabled = False
                self.handle = None
                self.canceling = False
                self.command_at = 0.0
                self.goal = None
            try:
                result = future.result()
                if result.status not in (4, 5):
                    self.error = f'Nav2 任务结束，状态 {result.status}'
            except Exception as exc:
                self.error = str(exc)

    def _accepted(self, future, generation):
        with self.lock:
            self.pending = False
            try:
                handle = future.result()
                if not handle.accepted:
                    self.fixed_goal_active = False
                    self.enabled = False
                    self.goal = None
                    self.error = 'Nav2 拒绝目标'
                    return
                self.handle = handle
                handle.get_result_async().add_done_callback(lambda f: self._result(f, handle))
                if generation != self.generation or not self.enabled:
                    self.canceling = True
                    handle.cancel_goal_async().add_done_callback(self._canceled)
            except Exception as exc:
                self.error = str(exc)
                self.fixed_goal_active = False
                self.enabled = False
                self.goal = None

    def update(self, distance, bearing, stand_off):
        """Called with a fresh, frame-matched visual measurement, never PID yaw."""
        with self.lock:
            try:
                pose = self.healthy_pose()
                goal = follow_goal(distance, bearing, stand_off, *pose, *self.camera_pose)
                if goal is None:
                    self.error = '目标已在跟随距离内，无需前进'
                    self.stop(self.error)
                    return
                goal = self.goal_filter.update(goal, time.monotonic())
                if goal is None:
                    if self.handle is None and not self.pending:
                        self.error = '等待连续稳定的目标位置（3 帧）'
                    return
                if not self.client.server_is_ready():
                    raise RuntimeError('Nav2 action server 尚未就绪')
                if self.pending or self.canceling:
                    return
                now = time.monotonic()
                if self.handle is not None:
                    if self.goal and math.hypot(goal[0]-self.goal[0], goal[1]-self.goal[1]) > 0.5 and now-self.goal_at >= 2.0:
                        self.stop('稳定目标移动超过 0.5 米，更新导航目标')  # Serialize cancel -> terminal result -> replacement.
                    return
                if now - self.goal_at < 1.0:
                    return
                self.enabled = True
                self.stop_recorded_pending = False
                self.command_at = 0.0
                request = self.action_type.Goal()
                request.pose.header.frame_id = 'odom'
                request.pose.header.stamp = self.node.get_clock().now().to_msg()
                request.pose.pose.position.x, request.pose.pose.position.y = goal[:2]
                request.pose.pose.orientation.z = math.sin(goal[2]/2)
                request.pose.pose.orientation.w = math.cos(goal[2]/2)
                self.goal, self.goal_at, self.pending = goal, now, True
                generation = self.generation
                self.client.send_goal_async(request).add_done_callback(
                    lambda f: self._accepted(f, generation))
                self.error = ''
            except Exception as exc:
                self.error = str(exc)
                self.stop(self.error)

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

    def pause_for_vision(self):
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
        if (not getattr(self, 'fixed_goal_active', False) or self.vision_at <= 0
                or time.monotonic() - self.vision_at > 2.0):
            self.stop('视觉障碍数据持续超时，取消导航')

    def velocity(self):
        with self.lock:
            try:
                self.healthy_pose()
                if getattr(self, 'vision_paused', False):
                    self.vision_paused = False
                    self.command_at = 0.0
                    self.command = (0.0, 0.0)
                    self.error = ''
                    return 0.0, 0.0  # Require a new controller command after recovery.
            except VisionStale:
                self.pause_for_vision()
                return 0.0, 0.0
            except Exception as exc:
                self.error = str(exc)
                self.stop(self.error)
            if not self.enabled or time.monotonic() - self.command_at > 0.3:
                return 0.0, 0.0
            # Bound travel while using the Nav2-specific visual latency budget.
            linear, yaw = self.command
            return max(-0.15, min(0.15, linear)), max(-0.4, min(0.4, yaw))

    def status(self):
        with self.lock:
            return {'nav2_fixed_goal_active': getattr(self, 'fixed_goal_active', False),
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
