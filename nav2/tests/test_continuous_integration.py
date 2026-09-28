# 【内容标注】用途：原 MotionManager 暂停入口与自有连续段接入的离线集成回归。
# 对应用户需求：R18、R22。不启动 ROS 节点、不发送底盘指令。
"""Use original method bodies without importing camera/GPU/robot dependencies."""
import ast
import time
import types
import unittest
from pathlib import Path
from unittest.mock import Mock
from nav2.continuous_segment import attach
from nav2_follow import VisionStale
from nav2.tests import test_person_navigation as fixtures


def original_motion_methods():
    module = ast.parse((Path(__file__).resolve().parents[2] / 'control.py').read_text())
    cls = next(node for node in module.body if isinstance(node, ast.ClassDef)
               and node.name == 'MotionManager')
    methods = [node for node in cls.body if isinstance(node, ast.FunctionDef)
               and node.name in ('handle_nav_measurement_pause', 'nav_stop_reason')]
    namespace = {}
    exec(compile(ast.Module(body=methods, type_ignores=[]), 'control.py', 'exec'), namespace)
    return namespace


class ContinuousIntegrationTests(unittest.TestCase):
    def setUp(self):
        fixture = fixtures.PersonNavigationTests(); fixture.setUp()
        self.n = n = fixture.n
        self.m = m = n.person_motion; m.navigation = n
        for name, method in original_motion_methods().items():
            if not name.startswith('__'):
                setattr(m, name, types.MethodType(method, m))
        n.continuous_follow = True; n.continuous_blocked = False
        n.continuous_target_id = 10; n.enabled = True; n.fixed_goal_active = True
        n.handle = Mock(); n.person_navigation.state = 'executing'
        n.command = (.1, .1); n.command_at = time.monotonic()
        m.target_visible = False; m.target_distance = None; m.distance_seen_at = 0.

    def test_original_pause_would_cancel_but_adapter_keeps_approved_segment(self):
        # This was the real cross-module failure; the old unit tests called
        # Nav2Follower.update directly and never exercised this pause hook.
        self.m.handle_nav_measurement_pause(time.monotonic())
        self.assertTrue(self.n.continuous_blocked)
        self.assertFalse(self.n.enabled)
        self.setUp(); attach(self.m)
        self.m.handle_nav_measurement_pause(time.monotonic())
        self.assertTrue(self.n.enabled); self.assertTrue(self.n.fixed_goal_active)
        self.assertFalse(self.n.continuous_blocked)
        self.n.handle.cancel_goal_async.assert_not_called()
        self.assertEqual(self.n.command, (.1, .1))

    def test_estop_end_manual_id_change_and_ordinary_follow_keep_original_stop(self):
        for field, value in (('estop', True), ('following', False), ('mode', 'manual'),
                             ('id', 11), ('continuous', False)):
            self.setUp(); attach(self.m)
            if field == 'id': self.m.settings.target_id = value
            elif field == 'continuous':
                self.n.continuous_follow = value; self.n.fixed_goal_active = False
            else: setattr(self.m, field, value)
            self.m.handle_nav_measurement_pause(time.monotonic())
            self.assertFalse(self.n.enabled, field)
            self.n.handle.cancel_goal_async.assert_called_once()

    def test_planning_without_verified_fixed_segment_cannot_use_gap_exemption(self):
        attach(self.m); n = self.n
        n.fixed_goal_active = n.enabled = False; n.handle = None
        n.person_navigation.state = 'planning'; n.person_navigation.busy = True; n.pending = True
        self.m.handle_nav_measurement_pause(time.monotonic())
        self.assertFalse(n.pending); self.assertTrue(n.continuous_blocked)

    def test_tf_or_obstacle_fault_still_cancels_latched_segment(self):
        for error in ('里程计 TF 超时', '前方近距障碍'):
            self.setUp(); attach(self.m)
            self.n.healthy_pose.side_effect = RuntimeError(error)
            self.m.handle_nav_measurement_pause(time.monotonic())
            self.assertFalse(self.n.enabled); self.assertTrue(self.n.continuous_blocked)
            self.assertEqual(self.n.last_stop_reason, error)

    def test_visual_staleness_zeros_immediately_then_cancels_at_existing_deadline(self):
        attach(self.m); n = self.n
        n.vision_at = time.monotonic() - 1.3
        def health(check_vision=True):
            if check_vision: raise VisionStale('视觉帧过期')
            return (0., 0., 0.)
        n.healthy_pose.side_effect = health
        self.m.handle_nav_measurement_pause(time.monotonic())
        self.assertEqual(n.command, (0., 0.)); self.assertTrue(n.enabled)
        n.handle.cancel_goal_async.assert_not_called()
        n.vision_at = time.monotonic() - 2.1
        self.m.handle_nav_measurement_pause(time.monotonic())
        self.assertFalse(n.enabled); self.assertTrue(n.continuous_blocked)
        n.handle.cancel_goal_async.assert_called_once()

    def test_fresh_person_update_cannot_override_velocity_grace_policy(self):
        n = self.n
        n.healthy_pose.side_effect = VisionStale('视觉帧 1.3 秒旧')
        n.update(3., 0., 1.)
        n.healthy_pose.assert_not_called()
        self.assertTrue(n.enabled); self.assertFalse(n.continuous_blocked)
        n.handle.cancel_goal_async.assert_not_called()

    def test_after_arrival_person_loss_requires_new_measurements(self):
        attach(self.m); n = self.n
        n.enabled = n.fixed_goal_active = False; n.handle = None
        n.person_navigation.state = 'arrived'; n.person_navigation.arrived_at = 12.
        self.m.handle_nav_measurement_pause(time.monotonic())
        self.assertFalse(n.enabled); self.assertFalse(n.continuous_blocked)
        self.assertEqual(n.person_navigation.arrived_at, 12.)

    def test_adapter_installs_once_and_restores_original(self):
        original = self.m.handle_nav_measurement_pause
        close = attach(self.m)
        self.assertIs(attach(self.m), close)
        close()
        self.assertIs(self.m.handle_nav_measurement_pause, original)
        close()  # Cleanup is safe more than once.


if __name__ == '__main__':
    unittest.main()
