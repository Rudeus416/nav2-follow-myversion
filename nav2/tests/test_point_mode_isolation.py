# 【R33】模拟手动选点从提交到速度输出；不启动 ROS 节点或底盘。
# 用户要求：历史人物点不设 3 秒；排查原来能跑的手动选点途中停止。
import time
import types
import unittest
from concurrent.futures import Future
from types import SimpleNamespace as NS
from unittest.mock import Mock
from nav2_follow import Nav2Follower, VisionStale
from nav2 import continuous_segment, point_navigation, replan_support
from nav2.tests import test_person_navigation as fixtures
from nav2.tests.test_continuous_integration import original_motion_methods


class PointModeIsolationTests(unittest.TestCase):
    def setUp(self):
        fixture = fixtures.PersonNavigationTests(); fixture.setUp()
        self.nav = n = fixture.n; self.path = fixture.path
        self.motion = m = n.person_motion; m.navigation = n; m.following = False
        n.continuous_follow = False
        n.vision_enabled = True; n.vision_at = time.monotonic(); n.vision_error = ''
        n.vision_safety = None  # New person-only data cannot be required by a map goal.
        n.replan_ready_pose = Mock(side_effect=AssertionError('person recovery called for map goal'))
        n.validate_replan = Mock(side_effect=AssertionError('person replan called for map goal'))
        for name, method in original_motion_methods().items():
            if not name.startswith('__'): setattr(m, name, types.MethodType(method, m))
        continuous_segment.attach(m)
        self.handle = Mock(accepted=True)
        self.handle.get_result_async.return_value = Future()
        point_navigation.start(n, (2.,0.,0.), self.path)
        n.path_client.send_goal_async.return_value.set_result(self.handle)
        n._command(NS(linear=NS(x=.1), angular=NS(z=.05)))

    def test_manual_goal_runs_through_original_target_loss_hook_without_person_replan(self):
        n,m = self.nav,self.motion
        m.target_visible = False; m.target_distance = None
        m.tracking_seen_at = m.distance_seen_at = 0.
        m.handle_nav_measurement_pause(time.monotonic())
        replan_support.monitor_route(n)
        self.assertEqual(n.velocity(), (.1,.05))
        self.assertTrue(n.enabled)
        self.assertFalse(n.person_navigation.obstacle_replan.active)
        n.replan_ready_pose.assert_not_called(); n.validate_replan.assert_not_called()
        self.handle.cancel_goal_async.assert_not_called()
        self.assertIs(n.path_client.send_goal_async.call_args.args[0].path,self.path)
        self.assertEqual(n.path_client.send_goal_async.call_count,1)

    def test_brief_vision_delay_still_waits_then_requires_a_new_command(self):
        n=self.nav; n.vision_at=time.monotonic()-1.3
        def healthy(check_vision=True):
            if check_vision: raise VisionStale('视觉延迟')
            return (0.,0.,0.)
        n.healthy_pose.side_effect=healthy
        self.assertEqual(n.velocity(),(0.,0.)); self.assertTrue(n.enabled)
        self.handle.cancel_goal_async.assert_not_called()
        n.healthy_pose.side_effect=None
        n._command(NS(linear=NS(x=.1),angular=NS(z=.05)))
        self.assertEqual(n.velocity(),(0.,0.))  # Discard even an in-pause command.
        n._command(NS(linear=NS(x=.1),angular=NS(z=.05)))
        self.assertEqual(n.velocity(),(.1,.05))

    def test_real_tf_staleness_still_cancels_at_original_half_second(self):
        n=self.nav; now=[10.2]
        n.healthy_pose=types.MethodType(Nav2Follower.healthy_pose,n)
        n.obstacle_mode='map'
        n.node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(now[0]*1e9))))
        n.tf=NS(lookup_transform=Mock(return_value=NS(
            header=NS(stamp=NS(sec=10,nanosec=0)), transform=NS(
                translation=NS(x=0.,y=0.),rotation=NS(x=0.,y=0.,z=0.,w=1.)))))
        self.assertEqual(n.velocity(),(.1,.05))
        now[0]=10.501
        self.assertEqual(n.velocity(),(0.,0.)); self.assertFalse(n.enabled)
        self.handle.cancel_goal_async.assert_called_once()
        self.assertIn('0.501',n.last_stop_reason)

    def test_near_obstacle_keeps_existing_manual_stop_without_person_replan(self):
        n=self.nav
        n.healthy_pose.side_effect=replan_support.NearObstacle('近距障碍')
        self.assertEqual(n.velocity(),(0.,0.)); self.assertFalse(n.enabled)
        self.handle.cancel_goal_async.assert_called_once()
        n.replan_ready_pose.assert_not_called(); n.validate_replan.assert_not_called()


if __name__=='__main__': unittest.main()
