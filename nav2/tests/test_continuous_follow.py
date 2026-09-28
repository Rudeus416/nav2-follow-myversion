# 【内容标注】用途：导航离线回归：continuous_follow。
# 对应用户需求：R18（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明。
# 本次仅加注释；需求关联不是精确创建/提交记录。
"""Continuous segments: no robot or ROS nodes."""
import time
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace as NS
from fastapi import HTTPException
from nav2.person_execution import execute
from nav2.tests import test_person_execution as setup
from nav2.tests import test_person_navigation as fixtures


class ContinuousFollowTests(unittest.TestCase):
    def setUp(self):
        c=fixtures.PersonNavigationTests();c.setUp();self.n=c.n
        self.n.continuous_follow=True;self.n.continuous_blocked=False;self.n.continuous_target_id=10

    def test_person_motion_and_distance_do_not_change_executing_segment(self):
        n=self.n;n.handle=Mock();n.enabled=True;n.goal=(2.,0.,0.);n.fixed_goal_active=True
        for stamp,distance in ((5.,4.),(5.3,.5),(5.6,6.)):
            with patch('nav2_follow.time.monotonic',return_value=stamp):n.update(distance,1.,1.)
        self.assertEqual(n.goal,(2.,0.,0.));self.assertEqual(len(n.goal_filter.samples),0)
        n.handle.cancel_goal_async.assert_not_called();n.preview_client.send_goal_async.assert_not_called()

    def test_planning_segment_is_not_replaced_by_new_target(self):
        n=self.n;n.pending=True;n.person_navigation.busy=True
        for stamp in (5.,5.3,5.6):
            with patch('nav2_follow.time.monotonic',return_value=stamp):n.update(5.,0.,1.)
        self.assertTrue(n.pending);self.assertFalse(n.continuous_blocked)
        n.preview_client.send_goal_async.assert_not_called()

    def test_one_meter_wait_then_resume_after_three_fresh_samples(self):
        n=self.n
        with patch('nav2_follow.time.monotonic',return_value=5.):n.update(1.,0.,2.)
        self.assertEqual(n.person_navigation.state,'waiting');self.assertFalse(n.enabled)
        self.assertTrue(n.person_motion.following)
        for stamp in (6.,6.3,6.6):
            with patch('nav2_follow.time.monotonic',return_value=stamp):n.update(1.2,0.,2.)
        self.assertAlmostEqual(n.preview_client.send_goal_async.call_args.args[0].goal.pose.position.x,.2)

    def test_arrival_requires_post_arrival_measurements(self):
        n=self.n;n.handle=Mock();n.enabled=True;n.fixed_goal_active=True;n.person_navigation.state='executing'
        with patch('nav2_follow.time.monotonic',return_value=10.):n._result(fixtures.done(NS(status=4)),n.handle)
        self.assertFalse(n.fixed_goal_active)
        n.person_motion.nav_measurement=(3.,0.,9.9)
        with patch('nav2_follow.time.monotonic',return_value=10.1):n.update(3.,0.,1.)
        n.preview_client.send_goal_async.assert_not_called()
        for stamp in (10.2,10.5,10.8):
            n.person_motion.nav_measurement=(3.,0.,stamp)
            with patch('nav2_follow.time.monotonic',return_value=stamp):n.update(3.,0.,1.)
        n.preview_client.send_goal_async.assert_called_once()

    def test_obstacle_stop_does_not_restart_on_next_person_frame(self):
        n=self.n;n.enabled=True;n.handle=Mock();n.fixed_goal_active=True
        n.stop('前方突发障碍')
        self.assertTrue(n.continuous_blocked);self.assertFalse(n.enabled)
        for stamp in (5.,5.3,5.6):
            with patch('nav2_follow.time.monotonic',return_value=stamp):n.update(3.,0.,1.)
        n.preview_client.send_goal_async.assert_not_called()

    def test_explicit_start_uses_fixed_segment_only_in_continuous_mode(self):
        c=setup.PersonExecutionTests();c.setUp();n=c.n;n.vision_enabled=True
        execute(c.motion,lambda:c.people,lambda:(1,'camera'),12,continuous=True)
        self.assertTrue(n.fixed_goal_active);self.assertTrue(n.continuous_follow)
        self.assertEqual(n.continuous_target_id,10)

    def test_continuous_requires_visual_obstacle_monitor(self):
        c=setup.PersonExecutionTests();c.setUp()
        with self.assertRaises(HTTPException):execute(c.motion,lambda:c.people,lambda:(1,'camera'),12,continuous=True)
        self.assertTrue(c.motion.estop);c.n.path_client.send_goal_async.assert_not_called()

    def test_missing_frame_after_arrival_does_not_erase_arrival_timestamp(self):
        n=self.n;n.person_navigation.arrived_at=10.
        n.stop('到达后目标暂不可见')
        self.assertEqual(n.person_navigation.arrived_at,10.)
        self.assertFalse(n.continuous_blocked)

    def test_ended_follow_does_not_acquire_another_segment(self):
        n=self.n;n.person_motion.following=False
        for stamp in (5.,5.3,5.6):
            with patch('nav2_follow.time.monotonic',return_value=stamp):n.update(3.,0.,1.)
        n.preview_client.send_goal_async.assert_not_called()
