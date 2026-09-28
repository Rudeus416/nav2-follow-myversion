# 【R34】真实人物执行状态机 + 视觉监测/速度出口，模拟短暂过期；不发实车动作。
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import numpy as np
from nav2 import replan_support as support
from nav2.tests import test_locked_person_segment as fixtures

FOOT=np.array([[.297,.232],[.297,-.232],[-.297,-.232],[-.297,.232]])


class ReplanVisualPauseTests(unittest.TestCase):
    def setUp(self):
        self.fixture=fixtures.LockedPersonSegmentTests();self.fixture.setUp()
        self.n=n=self.fixture.n
        self.now=100.
        self.clock=patch('time.monotonic',side_effect=lambda:self.now);self.clock.start()
        self.addCleanup(self.clock.stop)
        self.addCleanup(self.fixture.finish_cleanup)
        n.vision_enabled=True;n.vision_error='';n.vision_near_blocked=False
        self.frame(100.)
        support.attach(n,NS(snapshot=Mock()))
        self.tf_error=None
        def health(check_vision=True):
            if self.tf_error:raise RuntimeError(self.tf_error)
            if check_vision:n.check_vision()
            return (0.,0.,0.)
        n.healthy_pose=Mock(side_effect=health)
        self.fixture.start(continuous=True)
        self.task=n.person_navigation.obstacle_replan
        self.handle=self.fixture.handle
        self.generation=n.generation

    def frame(self,stamp):
        self.n.vision_at=stamp
        self.n.vision_safety=dict(points=np.empty((0,2)),source_at=stamp,
                                 footprint=FOOT,footprint_at=stamp)

    def command(self):
        self.n._command(NS(linear=NS(x=.1),angular=NS(z=0.)))

    def assert_waiting(self):
        self.assertTrue(self.n.enabled)
        self.assertTrue(self.task.active)
        self.assertFalse(self.task.holding)
        self.assertEqual(self.n.command,(0.,0.))
        self.assertEqual(self.n.generation,self.generation)
        self.handle.cancel_goal_async.assert_not_called()
        self.n.preview_client.send_goal_async.assert_not_called()

    def test_monitor_short_stale_keeps_same_task_and_resumes_only_with_new_command(self):
        self.now=101.3
        support.monitor_route(self.n)
        self.assert_waiting()
        self.assertEqual(self.n.velocity(),(0.,0.))
        self.now=101.5;self.frame(101.5)
        self.command()  # A command arriving during the pause is discarded.
        self.assertEqual(self.n.velocity(),(0.,0.))
        self.assert_waiting()
        self.command();self.assertEqual(self.n.velocity(),(.1,0.))
        self.assertEqual(self.n.path_client.send_goal_async.call_count,1)

    def test_monitor_repeats_cannot_extend_source_deadline(self):
        for age in (1.21,1.5,1.9,2.0):
            self.now=100.+age;support.monitor_route(self.n);self.assert_waiting()
        self.now=102.01;support.monitor_route(self.n)
        self.assertFalse(self.n.enabled)
        self.handle.cancel_goal_async.assert_called_once()
        self.assertIn('持续超时',self.n.last_stop_reason)

    def test_stale_full_cloud_between_health_check_and_command_guard_waits(self):
        self.now=101.3
        # A newer shallow health stamp cannot renew the old full safety sample.
        self.n.vision_at=101.3;self.command()
        self.assertEqual(self.n.velocity(),(0.,0.))
        self.assert_waiting()
        self.now=102.01;self.n.vision_at=102.01;self.command()
        self.assertEqual(self.n.velocity(),(0.,0.))
        self.assertFalse(self.n.enabled)
        self.handle.cancel_goal_async.assert_called_once()

    def test_frame_aging_during_route_geometry_waits_instead_of_replanning(self):
        self.now=101.19
        def geometry(*args,**kwargs):self.now=101.21;return False
        with patch('nav2.replan_safety.path_clear',side_effect=geometry):
            support.monitor_route(self.n)
        self.assert_waiting()

    def test_frame_aging_during_command_geometry_never_releases_speed(self):
        self.now=101.19;self.command()
        def geometry(*args,**kwargs):self.now=101.21;return True
        with patch('nav2.replan_safety.command_clear',side_effect=geometry):
            self.assertEqual(self.n.velocity(),(0.,0.))
        self.assert_waiting()

    def test_missing_full_cloud_or_footprint_is_a_real_fault(self):
        self.n.vision_safety=None
        support.monitor_route(self.n)
        self.assertFalse(self.n.enabled)
        self.handle.cancel_goal_async.assert_called_once()

    def test_tf_fault_during_vision_wait_still_cancels(self):
        self.now=101.3;self.tf_error='里程计 TF 超时'
        support.monitor_route(self.n)
        self.assertFalse(self.n.enabled)
        self.handle.cancel_goal_async.assert_called_once()
        self.assertEqual(self.n.last_stop_reason,'里程计 TF 超时')

    def test_obstacle_after_fresh_recovery_still_stops_for_replan(self):
        self.now=101.3;support.monitor_route(self.n);self.assert_waiting()
        self.now=101.4;self.frame(101.4)
        self.n.vision_safety['points']=np.array([[1.,.5]])
        support.monitor_route(self.n)
        self.assertTrue(self.task.holding)
        self.assertEqual(self.n.command,(0.,0.))
        self.handle.cancel_goal_async.assert_called_once()

    def test_canceled_task_never_resumes_on_new_visual_frame(self):
        self.now=101.3;support.monitor_route(self.n);self.assert_waiting()
        self.n.stop('用户取消')
        self.now=101.4;self.frame(101.4)
        support.monitor_route(self.n);self.command()
        self.assertEqual(self.n.velocity(),(0.,0.))
        self.assertFalse(self.n.enabled)
        self.assertEqual(self.n.path_client.send_goal_async.call_count,1)


    def test_same_frame_skips_duplicate_route_geometry_but_not_freshness_check(self):
        with patch('nav2.replan_safety.path_clear',return_value=True) as geometry:
            support.monitor_route(self.n);support.monitor_route(self.n)
            geometry.assert_called_once()
            self.now=101.3;support.monitor_route(self.n)
            self.assert_waiting();geometry.assert_called_once()
            self.now=101.4;self.frame(101.4);support.monitor_route(self.n)
            self.assertEqual(geometry.call_count,2)

    def test_new_route_or_generation_is_checked_even_with_same_frame(self):
        with patch('nav2.replan_safety.path_clear',return_value=True) as geometry:
            support.monitor_route(self.n)
            old_path=self.task.current_path
            self.task._path=NS(header=old_path.header,poses=list(old_path.poses))
            support.monitor_route(self.n)
            self.n.generation+=1
            support.monitor_route(self.n)
            self.assertEqual(geometry.call_count,3)


if __name__=='__main__':unittest.main()
