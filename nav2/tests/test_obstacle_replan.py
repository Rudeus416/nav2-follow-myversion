# 【R32】锁定人物段遇新障碍：先停车、等待旧动作终态、同终点整车复核后绕行。
"""Offline regression: real navigation state machine, mocked ROS actions only.

Reuse the original MotionManager method bodies through the R31 fixture; do not
import control.py's camera/robot dependencies and never publish to real topics.
Threads and action futures are advanced explicitly so cancellation races are
repeatable rather than timing-dependent.
"""
import time
import unittest
from concurrent.futures import Future
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
from nav2.tests import test_locked_person_segment as locked_fixtures
from nav2.tests import test_person_navigation as fixtures


class HeldThread:
    """Expose queued worker execution without starting an OS thread."""
    queue = []

    def __init__(self, *, target, args=(), kwargs=None, **options):
        self.target = target; self.args = args; self.kwargs = kwargs or {}; self.ran = False

    def start(self): self.queue.append(self)

    def join(self, timeout=None):
        if not self.ran:
            self.ran = True; self.target(*self.args, **self.kwargs)

    def is_alive(self): return not self.ran


class ObstacleReplanTests(unittest.TestCase):
    def setUp(self, continuous=True):
        HeldThread.queue = []
        self.threads = patch('nav2.obstacle_replan.threading.Thread', HeldThread)
        self.threads.start(); self.addCleanup(self.threads.stop)
        self.fixture = locked_fixtures.LockedPersonSegmentTests(); self.fixture.setUp()
        self.n, self.m = self.fixture.n, self.fixture.m
        self.n.replan_ready_pose = Mock(return_value=(0., 0., 0.))
        self.n.validate_replan = Mock()
        self.n.vision_enabled = True; self.n.vision_error = ''; self.n.vision_at = time.monotonic()
        self.fixture.start(continuous=continuous)
        self.r = self.n.person_navigation.obstacle_replan
        self.old_handle = self.fixture.handle; self.old_result = self.fixture.result
        self.cancel_reply = Future(); self.old_handle.cancel_goal_async.return_value = self.cancel_reply
        self.path = NS(header=NS(frame_id='odom'), poses=[fixtures.pose(0., 0.), fixtures.pose(.8, .8), fixtures.pose(2., 0.)])
        self.plan_accept = Future(); self.plan_result = Future()
        self.n.preview_client.send_goal_async.return_value = self.plan_accept
        self.new_accept = Future(); self.new_result = Future()
        self.n.path_client.send_goal_async.return_value = self.new_accept

    def workers(self):
        # A callback can enqueue another worker; bounded draining catches an
        # accidental self-rescheduling loop rather than hanging the suite.
        for _ in range(20):
            if not HeldThread.queue: return
            HeldThread.queue.pop(0).join()
        self.fail('Replan workers rescheduled indefinitely')

    def hazard(self):
        self.r.trigger('近距障碍保护：离线新障碍')
        self.assertEqual(self.n.command, (0., 0.)); self.assertEqual(self.n.velocity(), (0., 0.))
        self.assertTrue(self.r.holding)

    def begin_plan(self):
        self.hazard()
        self.old_result.set_result(NS(status=5))  # Genuine terminal result, not just cancel acknowledgement.
        self.r.tick(); self.workers()
        self.assertEqual(self.n.preview_client.send_goal_async.call_count, 1)
        self.assertEqual(self.n.path_client.send_goal_async.call_count, 1)
        self.assertEqual(self.n.velocity(), (0., 0.))
        goal = self.n.preview_client.send_goal_async.call_args.args[0].goal.pose.position
        self.assertEqual((goal.x, goal.y), (2., 0.))
        return goal

    def planner_response(self, *, status=4, path=None, accepted=True):
        handle = Mock(accepted=accepted)
        handle.get_result_async.return_value = self.plan_result
        self.plan_accept.set_result(handle)
        if accepted:
            self.plan_result.set_result(NS(status=status, result=NS(path=path or self.path, error_code=0 if status==4 else 208)))
        return handle

    def execute_detour(self):
        self.begin_plan(); self.planner_response(); self.workers()
        self.assertEqual(self.n.path_client.send_goal_async.call_count, 2)
        sent = self.n.path_client.send_goal_async.call_args.args[0]
        self.assertIs(sent.path, self.path)
        self.assertEqual(self.n.validate_replan.call_count, 1)
        self.assertIs(self.n.validate_replan.call_args.args[0], self.path)
        self.assertFalse(self.r.holding); self.assertTrue(self.r.permits_near)
        handle = Mock(accepted=True); handle.get_result_async.return_value = self.new_result
        handle.cancel_goal_async.return_value = Future()
        self.new_accept.set_result(handle)
        return handle

    def assert_no_detour_execution(self):
        self.workers()
        self.assertEqual(self.n.path_client.send_goal_async.call_count, 1)
        self.assertEqual(self.n.velocity(), (0., 0.))

    def assert_no_live_restart_after_failure(self):
        # A stopped planner has no executing action flags. Fresh person frames
        # must not reinterpret that idle-looking state as permission to follow.
        count = self.n.preview_client.send_goal_async.call_count
        self.n.goal_at = 0.
        now = time.monotonic()
        for stamp in (now+.1, now+.4, now+.7):
            self.m.target_visible = True; self.m.target_distance = 3.
            self.m.tracking_seen_at = self.m.distance_seen_at = stamp
            self.m.nav_measurement = (3., 0., stamp)
            with patch('nav2_follow.time.monotonic', return_value=stamp): self.n.update(3., 0., 1.)
        self.assertEqual(self.n.preview_client.send_goal_async.call_count, count,
                         'Fresh person frames restarted a canceled/failed obstacle recovery')
        self.assertFalse(self.n.enabled)

    def test_hazard_zeros_and_cancel_ack_never_replaces_terminal_confirmation(self):
        self.hazard(); self.old_handle.cancel_goal_async.assert_called_once()
        self.cancel_reply.set_result(NS(goals_canceling=[object()]))
        for _ in range(3): self.r.tick(); self.workers()
        self.assertIs(self.n.handle, self.old_handle)
        self.n.preview_client.send_goal_async.assert_not_called()
        self.assertFalse(self.r.permits_near)
        self.old_result.set_result(NS(status=5)); self.r.tick(); self.workers()
        self.n.preview_client.send_goal_async.assert_called_once()
        self.assert_no_detour_execution()

    def test_repeated_obstacle_frames_do_not_duplicate_cancel_or_planning(self):
        self.hazard()
        for _ in range(5): self.r.trigger('近距障碍保护：同一危险'); self.r.tick()
        self.old_handle.cancel_goal_async.assert_called_once()
        self.n.preview_client.send_goal_async.assert_not_called()
        self.old_result.set_result(NS(status=5))
        for _ in range(5): self.r.tick(); self.workers()
        self.n.preview_client.send_goal_async.assert_called_once()

    def test_missing_or_moved_live_person_does_not_replace_historical_goal(self):
        self.m.target_visible = False; self.m.distance_seen_at = 0.; self.m.tracking_seen_at = 0.
        self.begin_plan()
        for _ in range(3):
            self.n.update(9., 1., 1.)
            self.m.handle_nav_measurement_pause(time.monotonic())
            self.assertEqual(self.n.velocity(), (0., 0.))
        self.n.preview_client.send_goal_async.assert_called_once()
        self.planner_response(); self.workers()
        self.assertEqual(self.n.path_client.send_goal_async.call_count, 2)
        sent = self.n.path_client.send_goal_async.call_args.args[0].path.poses[-1].pose.position
        self.assertEqual((sent.x, sent.y), (2., 0.))

    def test_original_pause_hook_preserves_holding_during_cancel_and_validation(self):
        self.m.target_visible = False; self.m.distance_seen_at = 0.
        self.hazard(); self.m.handle_nav_measurement_pause(time.monotonic())
        self.assertTrue(self.r.holding); self.assertTrue(self.m.following)
        self.old_result.set_result(NS(status=5)); self.r.tick(); self.workers()
        self.planner_response()  # Validation worker is queued, not run yet.
        self.m.handle_nav_measurement_pause(time.monotonic())
        self.assertTrue(self.r.holding); self.assertEqual(self.n.velocity(), (0., 0.))
        self.workers(); self.assertEqual(self.n.path_client.send_goal_async.call_count, 2)

    def test_safe_revalidated_path_is_exactly_the_executed_path(self):
        self.execute_detour()
        self.assertEqual(self.n.person_target_marker['position'], [3., 0.])
        self.assertTrue(self.n.fixed_goal_active)

    def test_cancel_rejection_no_response_and_transport_failure_never_race_new_path(self):
        for failure in ('rejected', 'silent', 'transport'):
            with self.subTest(failure=failure):
                if failure != 'rejected': self.tearDown(); self.doCleanups(); self.setUp()
                if failure == 'transport': self.old_handle.cancel_goal_async.side_effect = RuntimeError('cancel transport')
                self.r.trigger('近距障碍保护：离线新障碍')
                if failure == 'rejected': self.cancel_reply.set_result(NS(goals_canceling=[]))
                self.r.tick(); self.workers()
                self.assertIs(self.n.handle, self.old_handle)
                self.n.preview_client.send_goal_async.assert_not_called()
                self.assertEqual(self.n.velocity(), (0., 0.))
                with patch('nav2.obstacle_replan.time.monotonic', return_value=time.monotonic()+6.): self.r.tick()
                self.old_result.set_result(NS(status=5)); self.r.tick(); self.workers()
                self.n.preview_client.send_goal_async.assert_not_called()
                self.assertEqual(self.n.path_client.send_goal_async.call_count, 1)

    def test_user_stop_and_authority_changes_reject_late_planner_callbacks(self):
        for change in ('stop', 'id', 'stream', 'generation', 'epoch', 'estop', 'manual', 'ended', 'sensor_config'):
            with self.subTest(change=change):
                if change != 'stop': self.tearDown(); self.doCleanups(); self.setUp()
                self.begin_plan()
                if change == 'stop': self.n.stop('用户强停')
                elif change == 'id': self.m.settings.target_id = 11
                elif change == 'stream': self.fixture.stream = (2, 'camera')
                elif change == 'generation': self.n.generation += 1
                elif change == 'epoch': self.m._nav2_follow_authorization += 1
                elif change == 'estop': self.m.set_estop(True)
                elif change == 'manual': self.m.set_mode('manual')
                elif change == 'ended': self.m.set_following(False)
                else: self.m.radar_reconfiguring = True
                self.r.tick()
                planner = self.planner_response(); self.assert_no_detour_execution()
                self.assertFalse(self.r.permits_near)

    def test_explicit_stop_during_planning_cannot_restart_from_fresh_person_frames(self):
        self.begin_plan()
        self.n.stop('规划期间用户结束追踪')
        self.assert_no_live_restart_after_failure()
        self.planner_response(); self.assert_no_detour_execution()

    def test_stop_while_validation_is_running_cannot_resume(self):
        self.begin_plan()
        self.n.validate_replan.side_effect = lambda *_, **__: self.n.stop('验证期间用户结束追踪')
        self.planner_response(); self.assert_no_detour_execution()
        self.assertFalse(self.r.permits_near)
        self.assert_no_live_restart_after_failure()

    def test_no_path_rejection_collision_or_moved_start_stays_stopped(self):
        for failure in ('no_path', 'rejected', 'collision', 'moved_start'):
            with self.subTest(failure=failure):
                if failure != 'no_path': self.tearDown(); self.doCleanups(); self.setUp()
                self.begin_plan()
                if failure == 'collision': self.n.validate_replan.side_effect = ValueError('整车碰撞或紫色缓冲区')
                if failure == 'moved_start': self.n.replan_ready_pose.return_value = (1., 0., 0.)
                self.planner_response(status=6 if failure == 'no_path' else 4, accepted=failure != 'rejected')
                self.assert_no_detour_execution()
                self.assertFalse(self.r.permits_near)
                self.assert_no_live_restart_after_failure()

    def test_planning_deadline_rejects_late_result_and_late_acceptance(self):
        self.begin_plan()
        with patch('nav2.obstacle_replan.time.monotonic', return_value=time.monotonic()+6.): self.r.tick()
        planner = self.planner_response()
        planner.cancel_goal_async.assert_called_once()
        self.assert_no_detour_execution()
        self.assertFalse(self.r.permits_near)

    def test_one_locked_segment_has_at_most_three_automatic_detours(self):
        terminal = self.old_result
        for attempt in range(1, 4):
            planner_accept, planner_result, execution_accept, next_terminal = (Future() for _ in range(4))
            self.n.preview_client.send_goal_async.return_value = planner_accept
            self.n.path_client.send_goal_async.return_value = execution_accept
            self.n.handle.cancel_goal_async.return_value = Future()
            self.r.trigger('近距障碍保护：第 %d 次离线新障碍' % attempt)
            self.assertEqual(self.n.velocity(), (0., 0.))
            terminal.set_result(NS(status=5)); self.r.tick(); self.workers()
            self.assertEqual(self.n.preview_client.send_goal_async.call_count, attempt)
            planned = Mock(accepted=True); planned.get_result_async.return_value = planner_result
            planner_accept.set_result(planned)
            planner_result.set_result(NS(status=4, result=NS(path=self.path, error_code=0)))
            self.workers()
            handle = Mock(accepted=True); handle.get_result_async.return_value = next_terminal
            execution_accept.set_result(handle); terminal = next_terminal
            self.assertEqual(self.r.attempts, attempt)
        self.r.trigger('近距障碍保护：已达到单段重规划上限')
        terminal.set_result(NS(status=5)); self.r.tick(); self.workers()
        self.assertEqual(self.n.preview_client.send_goal_async.call_count, 3)
        self.assertEqual(self.n.path_client.send_goal_async.call_count, 4)
        self.assertEqual(self.n.velocity(), (0., 0.)); self.assertFalse(self.r.permits_near)

    def test_pending_old_acceptance_is_canceled_before_any_new_plan(self):
        # Establish another locked request but hold its execution acceptance.
        self.n.stop('离线夹具切换到待接受动作'); self.old_result.set_result(NS(status=5)); self.workers()
        pending_accept = Future(); pending_result = Future()
        self.n.path_client.reset_mock(); self.n.path_client.send_goal_async.return_value = pending_accept
        self.n.pending = True; self.n.continuous_blocked = False
        with self.m.lock, self.n.lock:
            self.n.person_navigation.execute_path(self.fixture.path, 10, self.n.generation, locked=True)
        self.r.trigger('近距障碍保护：旧动作尚未接受')
        self.r.tick(); self.workers(); self.n.preview_client.send_goal_async.assert_not_called()
        late = Mock(accepted=True); late.get_result_async.return_value = pending_result
        late.cancel_goal_async.return_value = Future()
        pending_accept.set_result(late)
        late.cancel_goal_async.assert_called_once()
        self.r.tick(); self.workers(); self.n.preview_client.send_goal_async.assert_not_called()
        pending_result.set_result(NS(status=5)); self.r.tick(); self.workers()
        self.n.preview_client.send_goal_async.assert_called_once()
        self.assertEqual(self.n.path_client.send_goal_async.call_count, 1)
        self.assertEqual(self.n.velocity(), (0., 0.))

    def test_late_old_validation_failure_cannot_stop_new_explicit_segment(self):
        self.begin_plan(); new_token = []
        def replace_then_fail(*args, **kwargs):
            self.n.stop('用户取消旧路线')
            self.m.set_following(True)
            self.n.continuous_blocked = False; self.n.pending = True
            with self.m.lock, self.n.lock:
                self.n.person_navigation.execute_path(self.fixture.path, 10, self.n.generation, locked=True)
            new_token.append(self.r.segment)
            raise ValueError('旧验证迟到失败，不属于新任务')
        self.n.validate_replan.side_effect = replace_then_fail
        self.planner_response(); self.workers()
        self.assertEqual(len(new_token), 1)
        self.assertIs(self.r.segment, new_token[0])
        self.assertTrue(self.n.enabled); self.assertTrue(self.n.pending)
        self.assertTrue(self.m.following); self.assertEqual(self.r.stage, 'executing')
        self.assertEqual(self.n.path_client.send_goal_async.call_count, 2)

    def test_sensor_fault_during_holding_stops_and_rejects_late_plan(self):
        from nav2_follow import VisionStale
        for reason in (RuntimeError('里程计 TF 超时'), VisionStale('视觉数据过期')):
            with self.subTest(reason=type(reason).__name__):
                if isinstance(reason, VisionStale): self.doCleanups(); self.setUp()
                self.begin_plan()
                self.n.replan_ready_pose.side_effect = reason
                self.r.tick(); self.planner_response(); self.assert_no_detour_execution()
                self.assertFalse(self.r.holding); self.assertFalse(self.r.permits_near)

    def test_single_locked_detour_finishes_without_new_follow_authorization(self):
        self.doCleanups(); self.setUp(continuous=False)
        self.execute_detour(); self.new_result.set_result(NS(status=4)); self.workers()
        self.assertFalse(self.m.following); self.assertFalse(self.n.enabled)
        self.assertIsNone(self.n.person_navigation.locked_segment)
        self.n.update(9., 1., 1.)
        self.assertEqual(self.n.preview_client.send_goal_async.call_count, 1)

    def test_hazard_resets_velocity_ramp_before_any_output_poll(self):
        from nav2.motion_geometry import CurvatureRamp
        ramp = self.n.velocity_ramp = CurvatureRamp()
        for i in range(40): ramp.apply(.18, .3, 10. + i * .02)
        self.assertIsNotNone(ramp.at)
        self.r.trigger('近距障碍保护：立即清理加速状态')
        self.assertIsNone(ramp.at)
        self.assertEqual(self.n.command, (0., 0.))

    def test_old_action_success_during_cancel_is_arrival_not_an_extra_detour(self):
        self.hazard()
        self.old_result.set_result(NS(status=4)); self.r.tick(); self.workers()
        self.n.preview_client.send_goal_async.assert_not_called()
        self.assertFalse(self.n.enabled)
        self.assertEqual(self.n.person_navigation.state, 'arrived')

    def test_continuous_arrival_reacquires_only_frames_captured_after_detour_arrival(self):
        self.execute_detour()
        self.n.goal_at = 5.  # Same synthetic clock domain as the post-arrival samples below.
        with patch('nav2_follow.time.monotonic', return_value=10.): self.new_result.set_result(NS(status=4))
        self.assertTrue(self.m.following)
        self.assertEqual(self.n.person_navigation.arrived_at, 10.)
        self.n.preview_client.reset_mock(); self.n.preview_client.send_goal_async.return_value = Future()
        self.m.nav_measurement = (3., 0., 9.9)
        with patch('nav2_follow.time.monotonic', return_value=10.1): self.n.update(3., 0., 1.)
        self.n.preview_client.send_goal_async.assert_not_called()
        for stamp in (10.2, 10.5, 10.8):
            self.m.nav_measurement = (3., 0., stamp)
            with patch('nav2_follow.time.monotonic', return_value=stamp): self.n.update(3., 0., 1.)
        self.n.preview_client.send_goal_async.assert_called_once()


if __name__ == '__main__': unittest.main()
