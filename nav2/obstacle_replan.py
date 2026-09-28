# 【内容标注】用途：人物固定路线遇新障碍后，停车并向原终点重新规划。
# 对应用户需求：R32（同一终点绕障、无法绕行则停、取消后不得恢复；见 CODE_GUIDE.md）。
# 添加逻辑：等待旧 FollowPath 终态，异步计算与整车复核，再提交同一终点的新路线。
"""Bounded obstacle recovery for an authorized, fixed person-follow segment.

This module never changes the selected person or releases a stop.  The original
motion controller remains the only chassis publisher.  ``enabled=False`` keeps
output at zero while ``fixed_goal_active=True`` prevents temporary person loss
from replacing the locked destination during cancellation/planning.

Integration contracts (all state transitions hold ``nav.lock``):
* PersonNavigation.execute_path calls arm() for each checked FollowPath request.
* Normal stop/cancel and ordinary action completion call invalidate().
* The FollowPath result handler calls execution_result() before clearing handle.
* replan_ready_pose() checks fresh vehicle/sensor state, allowing only the
  particular obstacle which caused recovery; it returns (x, y, yaw).
* validate_replan(path, since) runs outside control locks and checks the complete
  body against current static/global maps and fresh, full visual obstacle data.
  It raises on any unsafe/unknown result.  It must not start motion.
"""
import math
import logging
import threading
import time


class ObstacleReplanner:
    """One cancel → plan → validate → execute episode at a time, same endpoint."""

    CANCEL_TIMEOUT = 5.0
    PLAN_TIMEOUT = 5.0
    MAX_ATTEMPTS = 3

    def __init__(self, owner):
        self.owner = owner
        self.nav = owner.nav
        self.segment = None
        self.stage = 'idle'
        self.attempts = 0
        self.started_at = 0.0
        self.obstacle_at = 0.0
        self.reason = ''
        self._path = None
        self._episode = None
        self._planner_handle = None
        self._old_handle = None
        self._worker = None
        self._committing = False
        self._allow_near = False

    @property
    def active(self):
        return self.segment is not None

    @property
    def current_path(self):
        """Actual armed/executing path, also an identity fence for route monitors."""
        return self._path

    @property
    def holding(self):
        return self.segment is not None and self.stage in ('canceling', 'planning', 'validating')

    @property
    def permits_near(self):
        """A checked detour may use the directional swept-body output guard."""
        return self.segment is not None and self._allow_near and self.stage == 'executing'

    @staticmethod
    def _pose(item):
        p, q = item.pose.position, item.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values):
            raise RuntimeError('绕障路径包含无效坐标')
        norm = sum(v * v for v in (q.x, q.y, q.z, q.w))
        if abs(norm - 1.0) > .02:
            raise RuntimeError('绕障路径包含无效朝向')
        return p.x, p.y, math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))

    def arm(self, path, target_id, generation, locked=False):
        """Remember a checked segment; automatic ordinary follow stays unchanged.

        During our own commit, retain the original destination and attempt count;
        the new route does not become a new person segment or reset its budget.
        """
        n = self.nav
        if self._committing:
            if self.segment is None or generation != self.segment['generation']:
                raise RuntimeError('绕障路线提交时任务已失效')
            self._path = path
            self.stage = 'executing'
            self._allow_near = True
            self._episode = None
            return
        self.invalidate()
        if not locked and not getattr(n, 'continuous_follow', False):
            return
        # Recovery is specific to the optional visual obstacle integration.
        # Map-only and pre-existing automatic follow keep their original gates.
        if (not getattr(n, 'vision_enabled', False)
                or not callable(getattr(n, 'replan_ready_pose', None))
                or not callable(getattr(n, 'validate_replan', None))):
            return
        if path.header.frame_id != 'odom' or not path.poses:
            raise RuntimeError('固定人物路线缺少有效终点')
        motion = getattr(n, 'person_motion', None)
        stream = getattr(n, 'person_stream_key', None)
        if motion is None or not callable(stream):
            raise RuntimeError('固定人物路线缺少授权或相机流状态')
        self.segment = {
            'goal': self._pose(path.poses[-1]),
            'target_id': target_id,
            'generation': generation,
            'stream_key': tuple(stream()),
            'authorization': getattr(motion, '_nav2_follow_authorization', 0),
            'locked': bool(locked),
            'continuous': bool(getattr(n, 'continuous_follow', False)),
            'anchor': getattr(n, 'person_anchor', None),
        }
        self._path = path
        self.stage = 'executing'

    def invalidate(self):
        """Revoke all recovery callbacks; late planner acceptance is canceled."""
        self._episode = None
        self.segment = None
        self.stage = 'idle'
        self.attempts = 0
        self._path = None
        self._allow_near = False
        self._old_handle = None
        handle, self._planner_handle = self._planner_handle, None
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:
                # ComputePath cannot move the robot.  Its late result is fenced
                # by episode identity even if cancel transport is unavailable.
                pass

    def _authorization_error(self):
        s, n = self.segment, self.nav
        if s is None:
            return '固定人物路线已取消'
        motion = getattr(n, 'person_motion', None)
        if (motion is None or motion.estop or motion.mode != 'auto' or not motion.following
                or getattr(motion, 'radar_reconfiguring', False)
                or motion.settings.target_id != s['target_id']
                or n.generation != s['generation']
                or getattr(motion, '_nav2_follow_authorization', 0) != s['authorization']
                or bool(getattr(n, 'continuous_follow', False)) != s['continuous']):
            return '绕障任务的强停、模式、人物 ID 或跟随授权已改变'
        try:
            if tuple(n.person_stream_key()) != s['stream_key']:
                return '绕障期间人物相机配置或图像流已改变'
        except Exception:
            return '绕障期间人物相机流状态不可用'
        return ''

    def _current(self, token):
        return token is not None and token is self._episode and self.holding

    def _check(self, token):
        """Return False for stale callbacks; current but invalid work fails closed."""
        if not self._current(token):
            return False
        reason = self._authorization_error()
        if reason:
            raise RuntimeError(reason)
        timeout = self.CANCEL_TIMEOUT if self.stage == 'canceling' else self.PLAN_TIMEOUT
        if time.monotonic() - self.started_at > timeout:
            raise RuntimeError('障碍绕行等待旧路线取消超时' if self.stage == 'canceling'
                               else '障碍绕行规划或整车复核超时')
        return True

    def trigger(self, reason):
        """Latch an actual collision, stop locally, then wait for old action end.

        Never call normal stop here: that correctly revokes the user's segment.
        Increasing generation makes an outstanding old acceptance cancel itself.
        """
        n = self.nav
        with n.lock:
            if not self.active:
                return False
            if self.holding:
                return True
            error = self._authorization_error()
            if error:
                self.fail(error)
                return True
            if self.attempts >= self.MAX_ATTEMPTS:
                self.fail('本段连续遇障已重规划 3 次，保持停车，请检查障碍后重新开始')
                return True
            self.attempts += 1
            self.reason = str(reason)
            self.obstacle_at = self.started_at = time.monotonic()
            self._episode = object()
            self.stage = 'canceling'
            self._allow_near = False
            self._old_handle = n.handle
            n.generation += 1
            self.segment['generation'] = n.generation
            locked = getattr(self.owner, 'locked_segment', None)
            if locked is not None:
                locked['generation'] = n.generation
            n.enabled = False
            n.fixed_goal_active = True
            n.command_at = 0.0
            n.command = (0.0, 0.0)
            ramp = getattr(n, 'velocity_ramp', None)
            if ramp is not None:
                ramp.reset()
            self.owner.state = 'replanning'
            self.owner.message = '检测到新障碍，已停车；等待旧路线结束后绕行到原终点'
            n.error = self.owner.message
            logging.getLogger(__name__).warning(
                'Nav2 遇障重规划：本段第 %d 次；保留终点 %s；原因：%s',
                self.attempts, self.segment['goal'], self.reason)
            if n.handle is not None and not n.canceling:
                n.canceling = True
                try:
                    n.handle.cancel_goal_async().add_done_callback(n._canceled)
                except Exception as exc:
                    self.fail('旧路线取消通信失败，保持停车：' + str(exc))
            return True

    def execution_result(self, future, handle):
        """Consume only this recovery's old canceled/aborted execution result.

        A racing real arrival remains an arrival.  A cancellation reply by itself
        is never sufficient: only terminal status 5/6 releases the action slot.
        """
        n = self.nav
        with n.lock:
            if (self.stage != 'canceling' or not self.holding or n.handle is not handle
                    or (self._old_handle is not None and self._old_handle is not handle)):
                return False
            response = None
            try:
                token = self._episode
                response = future.result()
                if not self._check(token):
                    return False
                if response.status == 4:
                    # The old goal actually arrived before cancellation won.
                    self.invalidate()
                    self.owner.state = 'executing'
                    return False
                if response.status not in (5, 6):
                    raise RuntimeError('旧路线尚未返回可确认的终态')
                n.handle = None
                n.canceling = False
                n.pending = False
                n.enabled = False
                n.fixed_goal_active = True
                n.command_at = 0.0
                n.command = (0.0, 0.0)
                self._old_handle = None
                return True
            except Exception as exc:
                # A readable terminal result frees the action slot even if
                # authorization expired. An unreadable result stays quarantined.
                if response is not None and response.status in (4, 5, 6):
                    n.handle = None
                    n.canceling = False
                    n.pending = False
                self.fail('障碍停车时无法确认旧路线终态：' + str(exc))
                return True

    def tick(self):
        """Advance after confirmed cancellation; enforce one total plan budget."""
        n = self.nav
        with n.lock:
            if not self.holding:
                return
            token = self._episode
            try:
                if not self._check(token):
                    return
                # Output holds zero while recovering; still check sensor/TF
                # health throughout cancellation, planning and validation.
                self._ready_pose()
                if self.stage != 'canceling' or n.handle is not None or n.pending or n.canceling:
                    return
                self.stage = 'planning'
                self.started_at = time.monotonic()
                self.owner.message = '正在从停车位置计算通往原终点的绕障路线'
                n.error = self.owner.message
                self._worker = threading.Thread(target=self._plan, args=(token,),
                                                name='nav2-obstacle-plan', daemon=True)
                self._worker.start()
            except Exception as exc:
                self.fail(str(exc))

    def _ready_pose(self):
        provider = getattr(self.nav, 'replan_ready_pose', None)
        if not callable(provider):
            raise RuntimeError('障碍绕行的传感器复核接口尚未就绪')
        pose = tuple(provider())
        if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
            raise RuntimeError('障碍绕行缺少有效停车位置')
        return pose

    def _plan(self, token):
        n = self.nav
        try:
            motion = n.person_motion
            # The authorization and send are atomic with original stop/settings.
            with motion.lock, n.lock:
                if not self._check(token):
                    return
                if not callable(getattr(n, 'validate_replan', None)):
                    raise RuntimeError('障碍绕行的整车路径复核接口尚未就绪')
                self._ready_pose()
                if not n.preview_client.server_is_ready() or not n.path_client.server_is_ready():
                    raise RuntimeError('障碍绕行规划或执行服务尚未就绪')
                goal = self.segment['goal']
                request = n.preview_type.Goal()
                request.goal.header.frame_id = 'odom'
                request.goal.header.stamp = n.node.get_clock().now().to_msg()
                request.goal.pose.position.x, request.goal.pose.position.y = goal[:2]
                request.goal.pose.orientation.z = math.sin(goal[2] / 2)
                request.goal.pose.orientation.w = math.cos(goal[2] / 2)
                request.planner_id = 'GridBased'
                request.use_start = False
                n.goal = goal
                n.preview_client.send_goal_async(request).add_done_callback(
                    lambda f: self._accepted(f, token))
        except Exception as exc:
            with n.lock:
                if self._current(token):
                    self.fail(str(exc))

    def _accepted(self, future, token):
        n = self.nav
        with n.lock:
            handle = None
            try:
                handle = future.result()
                if not self._check(token):
                    if handle.accepted:
                        handle.cancel_goal_async()
                    return
                if not handle.accepted:
                    raise RuntimeError('规划服务拒绝绕障到原终点')
                self._planner_handle = handle
                handle.get_result_async().add_done_callback(lambda f: self._result(f, token))
            except Exception as exc:
                if self._current(token):
                    # Also cancel a handle which arrived after auth/deadline expired.
                    if handle is not None and getattr(handle, 'accepted', False):
                        self._planner_handle = handle
                    self.fail(str(exc))

    def _result(self, future, token):
        with self.nav.lock:
            if not self._current(token):
                return
            self._planner_handle = None  # Planner already terminal; no cancel needed.
            self.stage = 'validating'
            self.owner.message = '正在复核绕障路线的完整车身与最新障碍数据'
            self.nav.error = self.owner.message
            try:
                self._worker = threading.Thread(target=self._finish, args=(future, token),
                                                name='nav2-obstacle-path-check', daemon=True)
                self._worker.start()
            except Exception as exc:
                self.fail(str(exc))

    def _finish(self, future, token):
        """Expensive geometry runs without motion/nav locks; commit is rechecked."""
        n = self.nav
        segment = None
        try:
            with n.lock:
                if not self._check(token):
                    return
                segment = self.segment
                goal = self.segment['goal']
                since = self.obstacle_at
            response = future.result()
            if response.status != 4:
                raise RuntimeError('无法绕障到原终点：规划状态 ' + str(response.status)
                                   + '，错误码 ' + str(getattr(response.result, 'error_code', None)))
            path = response.result.path
            if path.header.frame_id != 'odom' or not path.poses:
                raise RuntimeError('绕障规划返回空路径或错误坐标系')
            poses = [self._pose(item) for item in path.poses]
            end = poses[-1]
            if (math.hypot(end[0]-goal[0], end[1]-goal[1]) > .05
                    or abs(math.atan2(math.sin(end[2]-goal[2]), math.cos(end[2]-goal[2]))) > .1):
                raise RuntimeError('绕障规划改变了本段锁定终点，拒绝执行')
            n.validate_replan(path, since)  # Full static/global/visual body check, outside locks.
            motion = n.person_motion
            with motion.lock, n.lock:
                if not self._check(token):
                    return
                rx, ry, ra = self._ready_pose()
                first = poses[0]
                if (math.hypot(rx-first[0], ry-first[1]) > .05
                        or abs(math.atan2(math.sin(ra-first[2]), math.cos(ra-first[2]))) > .1):
                    raise RuntimeError('车位已偏离绕障路线起点，保持停车')
                if n.handle is not None or n.pending or n.canceling:
                    raise RuntimeError('旧导航任务尚未结束，不能提交绕障路线')
                s = self.segment
                n.goal = goal
                n.pending = True
                self._committing = True
                try:
                    self.owner.execute_path(path, s['target_id'], s['generation'], locked=s['locked'])
                except Exception:
                    # send_goal_async raising synchronously leaves no future pending.
                    n.pending = False
                    raise
                finally:
                    self._committing = False
                # arm() must run in execute_path before sending; keeps path identity
                # and the attempt budget correct even for immediate ROS callbacks.
                if self.segment is segment and self.stage == 'executing' and n.enabled:
                    self.owner.message = '已通过整车复核，正在绕障前往本段原终点'
                    logging.getLogger(__name__).warning('Nav2 绕障路线已提交：原终点 %s；路径点 %d', goal, len(path.poses))
        except Exception as exc:
            with n.lock:
                # execute_path.arm can clear the episode during commit.  It is
                # still this segment unless stop/cancel has revoked it entirely.
                if self._current(token) or (segment is not None and self.segment is segment
                                           and self.stage == 'executing'
                                           and segment.get('generation') == n.generation):
                    self.fail(str(exc))

    def fail(self, reason):
        """A failed attempt is terminal, never an automatic retry loop."""
        message = '障碍绕行停止：' + str(reason)
        logging.getLogger(__name__).warning('Nav2 %s', message)
        with self.nav.lock:
            self.nav.error = message
            try:
                # stop() must see the active recovery before invalidation so a
                # failed continuous segment cannot silently acquire another goal.
                self.nav.stop(message)
            finally:
                self.invalidate()
            self.owner.state = 'error'
            self.owner.message = message
