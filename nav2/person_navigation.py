# 【内容标注】用途：人物路线规划、复核和执行状态机。
# 对应用户需求：R13 R16 R17 R18 R31 R32（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：有限候选串行规划，复核同一路径后 FollowPath；处理取消/超时；连续模式固定本段。
# 需求关联用于追溯用途，不代表精确创建/提交记录；后续自检修复见 nav2/AUDIT.md。
"""Selected-person navigation: plan, validate, then execute the exact full path.

Called by authorized automatic follow or an explicit historical-route start.
R31 locked segments finish their checked path independently of person visibility;
a single completed/failed segment revokes follow authorization under control locks.
This module never releases estop, changes manual mode or publishes chassis speed.
"""
import math
import threading
import time


# 【职责 / R13 R16 R17 R18】PersonNavigation：人物路线规划、复核和执行状态机的状态封装；各方法职责见下方标注。
class PersonNavigation:
    # 【职责 / R13 R16 R17 R18】__init__：初始化本类依赖与状态；副作用以原初始化语句为准。
    def __init__(self, nav):
        self.nav = nav
        from nav2.obstacle_replan import ObstacleReplanner
        self.obstacle_replan = ObstacleReplanner(self)
        self.sequence = 0
        self.busy = False
        self.handle = None
        self.started_at = 0.
        self.arrived_at = None
        self.route_points = []
        self.last_route = None  # Display-only; never read by planning/execution.
        self.candidates = []
        # R31: present only for an explicitly approved historical segment.
        # It never changes the legacy automatic-follow entry point.
        self.locked_segment = None
        self._locked_finish_worker = None
        self.state = 'idle'
        self.message = '等待开启自动跟随并检测选定人物'

    # 【职责 / R13 R16 R17 R18】cancel：丢弃旧规划并取消句柄；历史路线仅用于显示，到达时间门槛按连续模式保留。
    def cancel(self):
        # Called under nav.lock. Invalidates even an unaccepted planner request.
        self.finish_locked_segment()
        self.sequence += 1
        handle, self.handle = self.handle, None
        if self.busy:
            self.nav.pending = False
        self.busy = False
        if not getattr(self.nav,'continuous_follow',False):
            self.arrived_at = None
        if self.route_points and self.last_route is not None:
            self.last_route['at'] = time.monotonic()
        self.route_points = []
        self.candidates = []
        self.state = 'idle'
        self.message = '人物导航已停止'
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:
                # A failed cancel transport must never prevent local stop/invalidation.
                self.message = '人物导航已停止；规划取消通信失败，迟到结果将丢弃'

    # 【职责 / R31】locked_segment_error：只检查锁定段授权身份，不把人物短暂不可见当成路线失效。
    def locked_segment_error(self):
        segment = self.locked_segment
        if segment is None or segment.get('finished'):
            return ''
        n = self.nav
        motion = getattr(n, 'person_motion', None)
        if (motion is None or motion.estop or motion.mode != 'auto' or not motion.following
                or getattr(motion, 'radar_reconfiguring', False)
                or motion.settings.target_id != segment['target_id']
                or n.generation != segment['generation']):
            return '历史人物路线授权或人物 ID 已改变，停止当前路线'
        try:
            if tuple(n.person_stream_key()) != segment['stream_key']:
                return '人物相机配置或图像流已改变，停止历史路线'
        except Exception:
            return '人物相机流状态不可用，停止历史路线'
        return ''

    # 【职责 / R31】finish_locked_segment：单次终态撤销 following；ROS 回调持导航锁时不反向获取运动锁。
    def finish_locked_segment(self):
        segment = self.locked_segment
        if segment is None or segment.get('finished'):
            return
        if segment['continuous']:
            self.locked_segment = None
            return
        segment['finished'] = True  # Immediately blocks update/output before cleanup runs.
        n = self.nav
        motion = getattr(n, 'person_motion', None)
        if motion is None:
            return
        authorization = getattr(motion, '_nav2_follow_authorization', 0)
        # 【职责 / R31】finish：按正常锁顺序结束单次历史人物授权，迟到清理不能覆盖新会话。
        def finish():
            # Normal lock order, never publish or start another action. A later
            # explicit historical start replaces the token. The instance adapter
            # counts successful original set_following calls under motion.lock;
            # plain estop/release must not be mistaken for new follow authorization.
            with motion.lock, n.lock:
                if self.locked_segment is not segment:
                    return
                if getattr(motion, '_nav2_follow_authorization', 0) == authorization:
                    motion.following = False
                self.locked_segment = None
        self._locked_finish_worker = threading.Thread(
            target=finish, name='nav2-person-segment-finish', daemon=True)
        try:
            self._locked_finish_worker.start()
        except RuntimeError:
            # Thread creation must never abort stop()/zeroing. Keep the finished
            # token blocking motion until the user explicitly starts a new task.
            self._locked_finish_worker = None

    # 【职责 / R13 R16 R17 R18】tick：检查人物规划总时限。
    def tick(self):
        with self.nav.lock:
            self.obstacle_replan.tick()
            if self.busy and time.monotonic() - self.started_at > 5.:
                self.fail('人物路线规划或复核超时，请等待新鲜目标重新规划')

    # 【职责 / R13 R16 R17 R18】fail：记录原因并进入停车/错误状态。
    def fail(self, message):
        self.nav.error = message
        self.nav.stop(message)
        self.state = 'error'
        self.message = message

    # 【职责 / R13 R16 R17 R18】request：建立本次人物规划代际/期限和候选队列，规划期间禁止输出。
    def request(self, goal, alternatives=()):
        """Caller holds nav.lock and has confirmed three stable measurements."""
        n = self.nav
        motion = getattr(n, 'person_motion', None)
        if motion is None or not callable(getattr(n, 'validate_preview', None)):
            raise RuntimeError('人物导航路径复核接口尚未就绪')
        if not n.preview_client.server_is_ready() or not n.path_client.server_is_ready():
            raise RuntimeError('人物导航规划/执行服务尚未就绪')
        self.arrived_at = None
        if getattr(n,'continuous_follow',False) and getattr(n,'person_target_marker',None) and n.person_anchor:
            measurement=getattr(motion,'nav_measurement',None)
            n.person_target_marker=dict(n.person_target_marker,position=list(n.person_anchor),
                source_at=measurement[2] if measurement else time.monotonic(),selected_at=time.monotonic())
        n.clear_preview()
        self.sequence += 1
        sequence, generation = self.sequence, n.generation
        target_id = motion.settings.target_id
        n.stop_recorded_pending = False
        self.busy = n.pending = True
        self.started_at = n.goal_at = time.monotonic()
        n.goal = goal
        n.enabled = False  # No output during planning/validation.
        n.command_at = 0.
        n.command = (0., 0.)
        n.error = ''
        self.candidates = list(alternatives)
        self.send_candidate(goal, sequence, generation, target_id)

    # 【职责 / R13 R16 R17 R18】send_candidate：串行发送一个候选终点，重试不重置总预算。
    def send_candidate(self, goal, sequence, generation, target_id):
        # Serial attempts share one deadline and cancellation generation.
        n = self.nav
        request = n.preview_type.Goal()
        request.goal.header.frame_id = 'odom'
        request.goal.header.stamp = n.node.get_clock().now().to_msg()
        request.goal.pose.position.x, request.goal.pose.position.y = goal[:2]
        request.goal.pose.orientation.z = math.sin(goal[2] / 2)
        request.goal.pose.orientation.w = math.cos(goal[2] / 2)
        request.planner_id = 'GridBased'
        request.use_start = False
        n.goal = goal
        self.state = 'planning'
        self.message = '正在计算人物附近的整车安全停车路线'
        n.error = self.message
        n.preview_client.send_goal_async(request).add_done_callback(
            lambda f: self.accepted(f, sequence, generation, target_id, goal))

    # 【职责 / R13 R16 R17 R18】current：确认回调属于仍有效的规划任务。
    def current(self, sequence, generation):
        return self.busy and sequence == self.sequence and generation == self.nav.generation

    # 【职责 / R13 R16 R17 R18】accepted：接收规划句柄，失效请求不进入后续执行。
    def accepted(self, future, sequence, generation, target_id, goal):
        with self.nav.lock:
            try:
                handle = future.result()
                if not self.current(sequence, generation):
                    if handle.accepted:
                        handle.cancel_goal_async()
                    return
                if not handle.accepted:
                    raise RuntimeError('规划服务拒绝人物终点')
                self.handle = handle
                handle.get_result_async().add_done_callback(
                    lambda f: self.result(f, sequence, generation, target_id, goal))
            except Exception as exc:
                if self.current(sequence, generation):
                    self.fail(str(exc))

    # 【职责 / R13 R16 R17 R18】result：在后台线程运行路径复核。
    def result(self, future, sequence, generation, target_id, goal):
        # Full-footprint validation is expensive: never block the ROS executor.
        threading.Thread(target=self.finish, args=(future, sequence, generation, target_id, goal),
                         name='nav2-person-path-check', daemon=True).start()

    # 【职责 / R13 R16 R17 R18】finish：复核整车路径及当前授权、新鲜人物和起点，再提交同一路径。
    def finish(self, future, sequence, generation, target_id, goal):
        n = self.nav
        with n.lock:
            if not self.current(sequence, generation):
                return
            self.handle = None
            self.state = 'validating'
            self.message = '正在复核人物路线的整车通行空间'
            n.error = self.message
        try:
            response = future.result()
            if response.status == 6 and self.candidates:
                # Only a completed planner abort may try another endpoint. Loss,
                # stop, validation failures and cancellations never cause retries.
                motion = n.person_motion
                with motion.lock, n.lock:
                    if not self.current(sequence, generation):
                        return
                    now = time.monotonic()
                    fresh = (not motion.estop and motion.mode == 'auto' and motion.following
                             and motion.settings.target_id == target_id and motion.target_visible
                             and motion.target_distance is not None
                             and 0 <= now-motion.tracking_seen_at <= motion.target_timeout()
                             and 0 <= now-motion.distance_seen_at <= motion.target_timeout())
                    if fresh and now-self.started_at < 5.:
                        n.healthy_pose()
                        self.send_candidate(self.candidates.pop(0), sequence, generation, target_id)
                        return
            if response.status != 4:
                raise RuntimeError(f'无有效人物路线：规划状态 {response.status}，错误码 '
                                   f'{getattr(response.result, "error_code", None)}')
            path = response.result.path
            if path.header.frame_id != 'odom' or not path.poses:
                raise RuntimeError('人物规划返回空路径或错误坐标系')
            for item in path.poses:
                p, q = item.pose.position, item.pose.orientation
                if not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
                    raise RuntimeError('人物路径包含无效坐标')
            n.validate_preview(path)  # Same validator as manually selected endpoints.
            motion = n.person_motion
            # Same lock order as original controls: motion -> navigation.
            # A stop/ID change while planning must not be undone by a late result.
            with motion.lock, n.lock:
                if not self.current(sequence, generation):
                    return
                now = time.monotonic()
                if (motion.estop or motion.mode != 'auto' or not motion.following
                        or motion.settings.target_id != target_id or not motion.target_visible
                        or motion.target_distance is None
                        or now - motion.tracking_seen_at > motion.target_timeout()
                        or now - motion.distance_seen_at > motion.target_timeout()):
                    raise RuntimeError('人物目标或跟随授权已失效，取消路线执行')
                if now - self.started_at > 5.:
                    raise RuntimeError('人物路线复核超时')
                rx, ry, ra = n.healthy_pose()
                first = path.poses[0].pose
                q = first.orientation
                heading = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                if (math.hypot(rx-first.position.x, ry-first.position.y) > .05
                        or abs(math.atan2(math.sin(ra-heading), math.cos(ra-heading))) > .1):
                    raise RuntimeError('车位已偏离人物路线起点，等待重新规划')
                self.execute_path(path, target_id, generation)
        except Exception as exc:
            with n.lock:
                if sequence == self.sequence and generation == n.generation:
                    # On synchronous FollowPath send failure, no acceptance is pending.
                    if not self.busy:
                        n.pending = False
                    self.fail(str(exc))


    # 【职责 / R13 R16 R17 R18 R31】execute_path：提交复核路线；显式历史点/连续段保持终点，保留障碍和传感器门控。
    def execute_path(self, path, target_id, generation, locked=False):
        """Commit a checked path; caller holds motion/nav locks and authorization."""
        n = self.nav
        now = time.monotonic()
        request = n.path_action_type.Goal()
        request.path = path
        request.controller_id = 'FollowPath'
        request.goal_checker_id = 'goal_checker'
        if locked:
            marker = getattr(n, 'person_target_marker', None)
            stream = getattr(n, 'person_stream_key', None)
            if not marker or not callable(stream) or tuple(marker['stream_key']) != tuple(stream()):
                raise RuntimeError('人物相机配置或历史点已改变，请重新规划')
            self.locked_segment = {'target_id': target_id, 'generation': generation,
                                   'stream_key': tuple(marker['stream_key']),
                                   'continuous': bool(getattr(n, 'continuous_follow', False)),
                                   'finished': False}
        else:
            self.locked_segment = None
        self.obstacle_replan.arm(path, target_id, generation, locked=locked)
        self.busy = False
        n.goal_at = now  # Start the execution-acceptance budget after validation.
        # An explicit historical route or continuous mode executes a fixed,
        # already-checked segment even if the person leaves the image.
        # Sensors/collision/estop still gate output.
        # Arrival clears fixed_goal_active before acquiring the next person pose.
        n.fixed_goal_active = locked or bool(getattr(n,'continuous_follow',False))
        n.enabled = True
        n.stop_recorded_pending = False
        n.command_at = 0.
        n.command = (0., 0.)
        n.error = ''
        self.route_points = [(item.pose.position.x,item.pose.position.y) for item in path.poses]
        self.last_route = {'points':list(self.route_points), 'target_id':target_id, 'at':now}
        self.state = 'executing'
        self.message = ('正在执行已锁定的历史人物路线，本段终点不更新' if locked
                        else '正在沿整车绕障路线跟随选定人物')
        n.path_client.send_goal_async(request).add_done_callback(
            lambda f: n._accepted(f, generation))
