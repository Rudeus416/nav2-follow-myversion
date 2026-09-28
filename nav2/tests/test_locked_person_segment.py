# 【R31】显式历史人物路线：原控制入口、同段冻结、终态清理及重新授权竞态。
"""Offline only: original control method bodies, mocked actions, no robot/ROS IO."""
import ast
import threading
import time
import types
import unittest
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
from nav2.continuous_segment import attach
from nav2_follow import VisionStale
from nav2.tests import test_person_navigation as fixtures
from nav2.tests.test_person_navigation import done


def original_methods():
    module = ast.parse((Path(__file__).resolve().parents[2] / 'control.py').read_text())
    cls = next(n for n in module.body if isinstance(n, ast.ClassDef) and n.name == 'MotionManager')
    names = {'update_tracking', 'update_measurement', 'handle_nav_measurement_pause',
             'nav_stop_reason', 'stop_now', 'set_following', 'set_mode', 'set_estop'}
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    namespace = {}
    exec(compile(ast.Module(body=methods, type_ignores=[]), 'control.py', 'exec'), namespace)
    return namespace


class DeferredThread:
    """Hold cleanup until the test explicitly interleaves a user action."""
    def __init__(self, *, target, **kwargs): self.target = target
    def start(self): pass
    def join(self, timeout=None): self.target()
    def is_alive(self): return False


class LockedPersonSegmentTests(unittest.TestCase):
    def setUp(self):
        fixture = fixtures.PersonNavigationTests(); fixture.setUp()
        self.n = n = fixture.n; self.path = fixture.path; self.m = m = n.person_motion
        m.navigation = n; m.radar_reconfiguring = False
        m.auto_frame_id = 0; m.auto_seen_at = 0.; m._publish = Mock(); m.status = Mock(return_value={})
        n.person_target_marker = {'id': 10, 'stream_key': (1, 'camera'), 'position': [3., 0.]}
        self.stream = (1, 'camera'); n.person_stream_key = lambda: self.stream
        for name, method in original_methods().items():
            if not name.startswith('__'): setattr(m, name, types.MethodType(method, m))
        attach(m)
        self.result = Future(); self.handle = Mock(accepted=True)
        self.handle.get_result_async.return_value = self.result

    def start(self, continuous=False):
        n = self.n; n.continuous_follow = continuous; n.continuous_target_id = 10
        n.continuous_blocked = False; n.pending = True
        with self.m.lock, n.lock:
            n.person_navigation.execute_path(self.path, 10, n.generation, locked=True)
        n.path_client.send_goal_async.return_value.set_result(self.handle)
        n.command = (.1, .1); n.command_at = time.monotonic()

    def finish_cleanup(self):
        worker = self.n.person_navigation._locked_finish_worker
        if worker is not None:
            worker.join(timeout=2)
            self.assertFalse(worker.is_alive())

    def test_single_and_continuous_keep_exact_path_on_original_target_loss_hooks(self):
        for continuous in (False, True):
            with self.subTest(continuous=continuous):
                self.setUp(); self.start(continuous); n, m = self.n, self.m
                self.assertTrue(n.fixed_goal_active)
                self.assertIs(n.path_client.send_goal_async.call_args.args[0].path, self.path)
                stamp = time.monotonic() + .1
                self.assertTrue(m.update_tracking(False, 0., stamp, 1))
                self.assertTrue(m.update_measurement(False, None, 0., 0., stamp, 1))
                m.handle_nav_measurement_pause(stamp)
                n.update(8., 1., 1.)  # A new person location cannot replace this historical endpoint.
                self.assertTrue(n.enabled); self.assertTrue(n.fixed_goal_active)
                self.assertEqual(n.velocity(), (.1, .1))
                self.handle.cancel_goal_async.assert_not_called()
                n.preview_client.send_goal_async.assert_not_called()
                self.assertEqual(n.path_client.send_goal_async.call_count, 1)
                m._publish.assert_not_called()

    def test_velocity_stops_on_id_stream_or_authorization_change_even_without_new_frames(self):
        for change in ('id', 'stream', 'following', 'estop', 'mode', 'generation', 'sensor_config'):
            with self.subTest(change=change):
                self.setUp(); self.start()
                if change == 'id': self.m.settings.target_id = 11
                elif change == 'stream': self.stream = (2, 'camera')
                elif change == 'generation': self.n.generation += 1
                elif change == 'sensor_config': self.m.radar_reconfiguring = True
                elif change == 'mode': self.m.mode = 'manual'
                else: setattr(self.m, change, change == 'estop')
                self.assertEqual(self.n.velocity(), (0., 0.))
                self.finish_cleanup()
                self.assertFalse(self.n.enabled); self.assertFalse(self.n.fixed_goal_active)
                self.handle.cancel_goal_async.assert_called_once()
                self.assertFalse(self.m.following)

    def test_original_manual_stop_still_invalidates_exact_segment_and_zeros(self):
        self.start(); self.m.set_estop(True); self.finish_cleanup()
        self.assertFalse(self.n.enabled); self.assertFalse(self.n.fixed_goal_active)
        self.handle.cancel_goal_async.assert_called_once()
        self.m._publish.assert_called_with(0., 0.)
        self.assertIsNone(self.n.person_navigation.locked_segment)

    def test_real_tf_and_obstacle_guards_remain_active_after_person_disappears(self):
        for reason in ('里程计 TF 超时', '前方突发障碍'):
            with self.subTest(reason=reason):
                self.setUp(); self.start(); self.m.target_visible = False
                self.n.healthy_pose.side_effect = RuntimeError(reason)
                self.m.handle_nav_measurement_pause(time.monotonic()); self.finish_cleanup()
                self.assertFalse(self.n.enabled)
                self.assertEqual(self.n.last_stop_reason, reason)
                self.handle.cancel_goal_async.assert_called_once()

    def test_vision_staleness_keeps_zero_output_grace_and_then_cancels(self):
        self.start(); n = self.n
        n.vision_at = time.monotonic() - 1.3
        def health(check_vision=True):
            if check_vision: raise VisionStale('视觉超时')
            return (0., 0., 0.)
        n.healthy_pose.side_effect = health
        self.assertEqual(n.velocity(), (0., 0.)); self.assertTrue(n.enabled)
        n.vision_at = time.monotonic() - 2.1
        self.assertEqual(n.velocity(), (0., 0.)); self.finish_cleanup()
        self.assertFalse(n.enabled); self.handle.cancel_goal_async.assert_called_once()

    def test_single_terminal_success_cancel_abort_disable_following_without_new_plan(self):
        for status in (4, 5, 6):
            with self.subTest(status=status):
                self.setUp(); self.start()
                self.result.set_result(NS(status=status)); self.finish_cleanup()
                self.assertFalse(self.m.following); self.assertFalse(self.n.enabled)
                self.assertIsNone(self.n.person_navigation.locked_segment)
                self.assertFalse(self.n.fixed_goal_active)
                self.n.preview_client.send_goal_async.assert_not_called()

    def test_single_action_rejection_and_acceptance_timeout_end_session(self):
        self.start(); self.n._accepted(done(Mock(accepted=False)), self.n.generation)
        self.finish_cleanup(); self.assertFalse(self.m.following)
        self.setUp(); n = self.n; n.pending = True
        with self.m.lock, n.lock:
            n.person_navigation.execute_path(self.path, 10, n.generation, locked=True)
        n.goal_at -= 6; n._execution_timeout(); self.finish_cleanup()
        self.assertFalse(self.m.following); self.assertFalse(n.enabled)
        late = Mock(accepted=True); late.get_result_async.return_value = Future()
        n.path_client.send_goal_async.return_value.set_result(late)
        late.cancel_goal_async.assert_called_once()

    def test_continuous_success_reacquires_only_three_frames_captured_after_arrival(self):
        self.start(continuous=True); n, m = self.n, self.m
        n.goal_at = 5.
        with patch('nav2_follow.time.monotonic', return_value=10.):
            self.result.set_result(NS(status=4))
        self.assertTrue(m.following); self.assertIsNone(n.person_navigation.locked_segment)
        self.assertFalse(n.fixed_goal_active)
        m.nav_measurement = (3., 0., 9.9)
        with patch('nav2_follow.time.monotonic', return_value=10.1): n.update(3., 0., 1.)
        n.preview_client.send_goal_async.assert_not_called()
        for stamp in (10.2, 10.5, 10.8):
            m.nav_measurement = (3., 0., stamp)
            with patch('nav2_follow.time.monotonic', return_value=stamp): n.update(3., 0., 1.)
        n.preview_client.send_goal_async.assert_called_once()

    def test_continuous_failure_blocks_automatic_restart(self):
        self.start(continuous=True); self.result.set_result(NS(status=6))
        self.assertTrue(self.n.continuous_blocked)
        self.n.update(3., 0., 1.); self.n.preview_client.send_goal_async.assert_not_called()

    def test_finished_token_blocks_new_live_plan_until_cleanup_runs(self):
        self.start()
        with patch('nav2.person_navigation.threading.Thread', DeferredThread):
            self.result.set_result(NS(status=4))
        self.assertTrue(self.n.person_navigation.locked_segment['finished'])
        for _ in range(3): self.n.update(8., 0., 1.)
        self.n.preview_client.send_goal_async.assert_not_called()
        self.finish_cleanup(); self.assertFalse(self.m.following)

    def test_late_cleanup_cannot_revoke_original_explicit_follow_restart(self):
        self.start()
        with patch('nav2.person_navigation.threading.Thread', DeferredThread):
            self.result.set_result(NS(status=4))
        old_sequence = self.n.preview_sequence
        # Execute the real original stop/reset path, not an imitation of its contract.
        self.m.set_following(True)
        self.assertGreater(self.n.preview_sequence, old_sequence)
        self.finish_cleanup()
        self.assertTrue(self.m.following)
        self.assertIsNone(self.n.person_navigation.locked_segment)

    def test_late_cleanup_cannot_revoke_original_mode_reset_and_follow_restart(self):
        self.start()
        with patch('nav2.person_navigation.threading.Thread', DeferredThread):
            self.n.stop('突发障碍')
        self.m.set_mode('manual'); self.m.set_mode('auto'); self.m.set_following(True)
        self.finish_cleanup(); self.assertTrue(self.m.following)
        self.assertIsNone(self.n.person_navigation.locked_segment)

    def test_estop_then_release_before_late_cleanup_is_not_new_follow_authorization(self):
        self.start()
        with patch('nav2.person_navigation.threading.Thread', DeferredThread):
            self.m.set_estop(True)
        self.m.set_estop(False)  # Original reset clears preview twice, but does not start a session.
        self.assertFalse(self.m.estop)
        self.finish_cleanup()
        self.assertFalse(self.m.following)
        self.assertIsNone(self.n.person_navigation.locked_segment)
        self.assertFalse(self.n.enabled)
        self.n.preview_client.send_goal_async.assert_not_called()

    def test_explicit_follow_restart_after_estop_release_survives_late_cleanup(self):
        self.start()
        with patch('nav2.person_navigation.threading.Thread', DeferredThread):
            self.m.set_estop(True)
        self.m.set_estop(False); self.m.set_following(True)
        self.finish_cleanup()
        self.assertTrue(self.m.following)
        self.assertIsNone(self.n.person_navigation.locked_segment)

    def test_follow_adapter_is_idempotent_and_restores_original_method(self):
        wrapped = self.m.set_following
        close = attach(self.m)
        self.assertIs(self.m.set_following, wrapped)
        self.m.set_following(False)
        epoch = self.m._nav2_follow_authorization
        self.assertEqual(epoch, 1)
        close(); self.assertIsNot(self.m.set_following, wrapped)
        self.m.set_following(True)
        self.assertEqual(self.m._nav2_follow_authorization, epoch)
        self.assertTrue(self.m.following)
        close()

    def test_cleanup_thread_creation_failure_cannot_interrupt_stop(self):
        self.start()
        with patch('nav2.person_navigation.threading.Thread') as worker:
            worker.return_value.start.side_effect = RuntimeError('thread unavailable')
            self.n.stop('强停')
        self.assertFalse(self.n.enabled); self.assertFalse(self.n.fixed_goal_active)
        self.assertEqual(self.n.velocity(), (0., 0.))
        self.n.update(8., 0., 1.); self.n.preview_client.send_goal_async.assert_not_called()
        self.handle.cancel_goal_async.assert_called_once()

    def test_late_cleanup_cannot_revoke_new_explicit_locked_route(self):
        self.start()
        with patch('nav2.person_navigation.threading.Thread', DeferredThread):
            self.result.set_result(NS(status=4))
        old_worker = self.n.person_navigation._locked_finish_worker
        self.n.path_client.send_goal_async.return_value = Future()
        self.n.person_navigation.execute_path(self.path, 10, self.n.generation, locked=True)
        new_token = self.n.person_navigation.locked_segment
        old_worker.join()
        self.assertIs(self.n.person_navigation.locked_segment, new_token)
        self.assertTrue(self.m.following)


if __name__ == '__main__': unittest.main()
