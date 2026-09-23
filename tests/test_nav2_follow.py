"""Offline geometry and asynchronous stop-race regression tests."""
import math
import threading
import time
import unittest
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock, patch
from nav2_follow import Nav2Follower, StableGoalFilter, follow_goal


def done(value):
    f = Future()
    f.set_result(value)
    return f


class NavigationTests(unittest.TestCase):
    def test_visual_hold_invalidates_command_but_keeps_healthy_goal(self):
        n = self.follower()
        n.handle = Mock()
        n.healthy_pose = Mock(return_value=(0, 0, 0))
        n.hold_for_visual_update()
        self.assertTrue(n.enabled)
        n.handle.cancel_goal_async.assert_not_called()
        self.assertEqual(n.command_at, 0)
        self.assertEqual(n.command, (0, 0))
        n.healthy_pose.side_effect = RuntimeError('雷达超时')
        n.hold_for_visual_update()
        n.handle.cancel_goal_async.assert_called_once()

    def test_stop_preserves_first_cause_without_repeated_logs(self):
        n = self.follower()
        with self.assertLogs('nav2_follow', level='WARNING') as logs:
            n.stop('目标跟踪超时：1.3 秒')
            n.stop('目标丢失')
        self.assertEqual(len(logs.output), 1)
        self.assertEqual(n.last_stop_reason, '目标跟踪超时：1.3 秒')
        self.assertEqual(n.command, (0.0, 0.0))

    def test_goal_filter_rejects_jumps_and_confirms_stable_measurements(self):
        f = StableGoalFilter()
        self.assertIsNone(f.update((1, 0, 0), 1))
        self.assertIsNone(f.update((3, 0, 0), 1.3))
        self.assertIsNone(f.update((1, 0, 0), 1.6))
        self.assertIsNone(f.update((1.1, 0, 0), 1.9))
        self.assertAlmostEqual(f.update((1.2, 0, 0), 2.2)[0], 1.1)
        self.assertIsNone(f.update((1.2, 0, 0), 4))

    def test_active_goal_ignores_jitter_but_updates_after_confirmed_motion(self):
        n = self.follower()
        n.goal_filter = StableGoalFilter()
        n.healthy_pose = lambda: (0, 0, 0)
        n.camera_pose = (0, 0, 0)
        n.client = Mock()
        n.handle = Mock()
        n.goal = (2, 0, 0)
        n.goal_at = 0
        for now, distance in [(3, 3.1), (3.3, 3.2), (3.6, 3.1), (3.9, 6), (4.2, 3.1)]:
            with patch('nav2_follow.time.monotonic', return_value=now):
                n.update(distance, 0, 1)
        n.handle.cancel_goal_async.assert_not_called()
        for now in (5, 5.3, 5.6):
            with patch('nav2_follow.time.monotonic', return_value=now):
                n.update(4, 0, 1)
        n.handle.cancel_goal_async.assert_called_once()
        self.assertFalse(n.enabled)
        self.assertEqual(len(n.goal_filter.samples), 0)

    def test_velocity_is_limited_and_stale_commands_still_stop(self):
        n = self.follower()
        n.healthy_pose = lambda: (0, 0, 0)
        n.command = (0.8, -0.9)
        self.assertEqual(n.velocity(), (0.15, -0.4))
        n.command_at = time.monotonic() - 0.4
        self.assertEqual(n.velocity(), (0.0, 0.0))

    def follower(self):
        n = Nav2Follower.__new__(Nav2Follower)
        n.lock = threading.RLock()
        n.handle = n.goal = None
        n.pending = n.canceling = False
        n.enabled = True
        n.generation = 0
        n.command = (0.2, 0.1)
        n.command_at = time.monotonic()
        n.error = ''
        return n

    def test_standoff_and_rotated_odometry(self):
        x, y, heading = follow_goal(3, 0, 1, 10, 20, math.pi / 2)
        self.assertAlmostEqual(x, 10)
        self.assertAlmostEqual(y, 22)
        self.assertAlmostEqual(heading, math.pi / 2)

    def preview_follower(self):
        n = self.follower()
        n.preview_sequence = 0
        n.preview_handle = None
        n.preview_at = 0.0
        n.preview = {'state': 'idle', 'points': []}
        n.healthy_pose = lambda: (0, 0, 0)
        n.camera_pose = (0, 0, 0)
        n.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None)))
        def goal():
            return SimpleNamespace(goal=SimpleNamespace(header=SimpleNamespace(), pose=SimpleNamespace(
                position=SimpleNamespace(), orientation=SimpleNamespace())))
        n.preview_type = SimpleNamespace(Goal=goal)
        n.preview_client = Mock()
        n.preview_client.server_is_ready.return_value = True
        n.preview_client.send_goal_async.return_value = Future()
        n.client = Mock()
        return n

    def test_preview_uses_only_planner_and_preserves_motion_state(self):
        n = self.preview_follower()
        n.preview_path(3, 0, 1)
        request = n.preview_client.send_goal_async.call_args.args[0]
        self.assertEqual(request.goal.pose.position.x, 2)
        self.assertFalse(request.use_start)
        n.client.send_goal_async.assert_not_called()
        self.assertEqual(n.command, (0.2, 0.1))
        self.assertEqual(n.preview['state'], 'pending')

    def test_preview_timeout_cancels_late_acceptance(self):
        n = self.preview_follower()
        n.preview_path(3, 0, 1)
        n.preview_at -= 6
        n._preview_timeout()
        self.assertEqual(n.preview['state'], 'error')
        handle = Mock(accepted=True)
        n.preview_client.send_goal_async.return_value.set_result(handle)
        handle.cancel_goal_async.assert_called_once()
        handle.get_result_async.assert_not_called()

    def test_preview_failure_explains_failure_and_clears_path(self):
        n = self.preview_follower()
        n._preview_result(done(SimpleNamespace(status=6, result=SimpleNamespace(error_code=208))), 0)
        self.assertEqual(n.preview['state'], 'error')
        self.assertIn('208', n.preview['message'])
        self.assertEqual(n.preview['points'], [])

    def test_old_preview_cannot_overwrite_new_result(self):
        n = self.preview_follower()
        n.preview_sequence = 2
        n._preview_result(done(SimpleNamespace(status=6)), 1)
        self.assertEqual(n.preview['state'], 'idle')

    def test_camera_mount_offset_and_turn(self):
        x, y, _ = follow_goal(2, 0, 1, 0, 0, 0, 0, 0.5, math.pi/2)
        self.assertAlmostEqual(x, 0)
        self.assertAlmostEqual(y, 1.5)

    def test_near_target_stops_and_far_goal_is_bounded(self):
        self.assertIsNone(follow_goal(1.05, 0, 1, 0, 0, 0))
        self.assertEqual(follow_goal(20, 0, 1, 0, 0, 0), (3, 0, 0))
        with self.assertRaises(ValueError):
            follow_goal(float('nan'), 0, 1, 0, 0, 0)

    def test_late_goal_acceptance_after_stop_is_canceled(self):
        n = self.follower()
        result = Future()
        calls = []
        handle = SimpleNamespace(accepted=True, get_result_async=lambda: result,
            cancel_goal_async=lambda: (calls.append('cancel') or done(SimpleNamespace(goals_canceling=[1]))))
        n.pending = True
        n.stop()
        n._accepted(done(handle), 0)
        self.assertEqual(calls, ['cancel'])
        self.assertTrue(n.canceling)
        n._command(SimpleNamespace(linear=SimpleNamespace(x=1), angular=SimpleNamespace(z=1)))
        self.assertEqual(n.command, (0, 0))
        result.set_result(SimpleNamespace(status=5))
        self.assertIsNone(n.handle)
        self.assertFalse(n.canceling)

    def test_command_timeout_and_sensor_failure_stop(self):
        n = self.follower()
        n.healthy_pose = lambda: (0, 0, 0)
        self.assertEqual(n.velocity(), (0.15, 0.1))
        n.command_at -= 1
        self.assertEqual(n.velocity(), (0, 0))
        n.command_at = time.monotonic()
        def stale():
            raise RuntimeError('stale radar')
        n.healthy_pose = stale
        self.assertEqual(n.velocity(), (0, 0))
        self.assertFalse(n.enabled)
        self.assertEqual(n.error, 'stale radar')

    def test_missing_camera_mount_blocks_motion(self):
        n = self.follower()
        n.camera_pose = None
        self.assertEqual(n.velocity(), (0.0, 0.0))
        self.assertFalse(n.enabled)
        self.assertIn('未配置相机', n.error)

    def test_wrong_radar_frame_immediately_invalidates_sensor(self):
        n = self.follower()
        n.expected_radar_frame = 'ti_mmwave_0'
        n.radar_at = time.monotonic()
        n.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=10_000_000_000)))
        message = SimpleNamespace(header=SimpleNamespace(
            stamp=SimpleNamespace(sec=10, nanosec=0), frame_id='radar'))
        n._radar(message)
        self.assertEqual(n.radar_at, 0.0)
        self.assertFalse(n.enabled)
        self.assertIn('坐标系不匹配', n.error)

    def test_expected_radar_frame_accepts_only_fresh_stamps(self):
        n = self.follower()
        n.expected_radar_frame = 'ti_mmwave_0'
        n.radar_at = 0.0
        n.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=10_000_000_000)))
        message = SimpleNamespace(header=SimpleNamespace(
            stamp=SimpleNamespace(sec=8, nanosec=0), frame_id='ti_mmwave_0'))
        n._radar(message)
        self.assertEqual(n.radar_at, 0.0)
        message.header.stamp.sec = 10
        n._radar(message)
        self.assertGreater(n.radar_at, 0)
        self.assertEqual(n.radar_frame, 'ti_mmwave_0')

    def test_old_result_cannot_clear_new_handle(self):
        n = self.follower()
        handle = object()
        n.handle = handle
        n._result(done(SimpleNamespace(status=5)), object())
        self.assertIs(n.handle, handle)


if __name__ == '__main__':
    unittest.main()
