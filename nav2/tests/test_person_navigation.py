# 【内容标注】用途：导航离线回归：person_navigation。
# 对应用户需求：R16 R17 R18（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明。
# 本次仅加注释；需求关联不是精确创建/提交记录。
"""Person plan/validate/execute tests: no ROS nodes or real motion."""
import math
import threading
import time
import unittest
from concurrent.futures import Future
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
from nav2_follow import Nav2Follower, StableGoalFilter
from nav2.person_navigation import PersonNavigation


def done(value):
    f=Future();f.set_result(value);return f


def pose(x,y):
    return NS(pose=NS(position=NS(x=x,y=y),orientation=NS(x=0.,y=0.,z=0.,w=1.)))


class PersonNavigationTests(unittest.TestCase):
    def setUp(self):
        self.n=n=Nav2Follower.__new__(Nav2Follower)
        n.lock=threading.RLock();n.pending=n.canceling=n.enabled=False
        n.person_anchor=(3.,0.);n.handle=n.goal=None;n.generation=0;n.command_at=n.goal_at=0.
        n.command=(0.,0.);n.error='';n.goal_filter=StableGoalFilter()
        n.preview_sequence=0;n.preview_handle=None;n.preview_at=0.;n.preview={}
        n.camera_pose=(0.,0.,0.);n.healthy_pose=Mock(return_value=(0.,0.,0.))
        n.node=NS(get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:None)))
        n.preview_type=NS(Goal=lambda:NS(goal=NS(header=NS(),pose=NS(position=NS(),orientation=NS()))))
        n.path_action_type=NS(Goal=lambda:NS())
        n.preview_client=Mock();n.preview_client.send_goal_async.return_value=Future()
        n.path_client=Mock();n.path_client.send_goal_async.return_value=Future()
        n.validate_preview=Mock()
        n.person_motion=NS(lock=threading.RLock(),estop=False,mode='auto',following=True,
            settings=NS(target_id=10),target_visible=True,target_distance=3.,
            tracking_seen_at=time.monotonic(),distance_seen_at=time.monotonic(),target_timeout=lambda:1.2)
        n.person_navigation=PersonNavigation(n)
        self.path=NS(header=NS(frame_id='odom'),poses=[pose(0.,0.),pose(1.,.5),pose(2.,0.)])
        self.response=done(NS(status=4,result=NS(path=self.path)))

    def request(self):
        self.n.person_navigation.request((2.,0.,0.))
        return self.n.person_navigation.sequence,self.n.generation

    def finish(self):
        seq,gen=self.request()
        self.n.person_navigation.finish(self.response,seq,gen,10,(2.,0.,0.))

    def test_exact_verified_path_executes_with_person_loss_gate_retained(self):
        self.finish();n=self.n
        n.validate_preview.assert_called_once_with(self.path)
        sent=n.path_client.send_goal_async.call_args.args[0]
        self.assertIs(sent.path,self.path)
        self.assertEqual(sent.controller_id,'FollowPath')
        self.assertFalse(n.fixed_goal_active)
        self.assertTrue(n.enabled)
        self.assertEqual(n.command,(0.,0.))

    def test_selected_person_uses_gridbased_after_three_stable_measurements(self):
        n=self.n
        for now in (5.,5.3,5.6):
            with patch('nav2_follow.time.monotonic',return_value=now):n.update(3.,0.,1.)
        sent=n.preview_client.send_goal_async.call_args.args[0]
        self.assertEqual(sent.planner_id,'GridBased')
        self.assertAlmostEqual(sent.goal.pose.position.x,2.)  # Keep 1m stand-off.
        n.path_client.send_goal_async.assert_not_called()
        self.assertFalse(n.enabled)

    def test_invalid_path_does_not_execute(self):
        self.n.validate_preview.side_effect=ValueError('车身与紫色区碰撞')
        self.finish();self.n.path_client.send_goal_async.assert_not_called()
        self.assertFalse(self.n.pending)
        self.assertIn('紫色',self.n.error)

    def test_target_lost_or_stop_or_mode_or_id_change_before_execution(self):
        for field,value in [('target_visible',False),('estop',True),('mode','manual'),('following',False)]:
            self.setUp();setattr(self.n.person_motion,field,value);self.finish()
            self.n.path_client.send_goal_async.assert_not_called()
        self.setUp();seq,gen=self.request();self.n.person_motion.settings.target_id=11
        self.n.person_navigation.finish(self.response,seq,gen,10,(2.,0.,0.))
        self.n.path_client.send_goal_async.assert_not_called()

    def test_stale_person_data_does_not_execute(self):
        self.n.person_motion.distance_seen_at-=3.
        self.finish();self.n.path_client.send_goal_async.assert_not_called()

    def test_stop_during_validation_cannot_restart_robot(self):
        self.n.validate_preview.side_effect=lambda _:self.n.stop('用户强停')
        self.finish();self.n.path_client.send_goal_async.assert_not_called()
        self.assertFalse(self.n.enabled)

    def test_planning_timeout_cancels_late_acceptance(self):
        self.request();n=self.n;n.person_navigation.started_at-=6.
        n.person_navigation.tick()
        handle=Mock(accepted=True)
        n.preview_client.send_goal_async.return_value.set_result(handle)
        handle.cancel_goal_async.assert_called_once()
        handle.get_result_async.assert_not_called()
        self.assertFalse(n.pending)

    def test_stop_after_send_cancels_late_execution_acceptance(self):
        self.finish();n=self.n;n.stop('目标丢失')
        handle=Mock(accepted=True);handle.get_result_async.return_value=Future()
        n.path_client.send_goal_async.return_value.set_result(handle)
        handle.cancel_goal_async.assert_called_once()
        self.assertFalse(n.fixed_goal_active);self.assertFalse(n.enabled)

    def test_bad_planner_response_or_moved_start_does_not_execute(self):
        self.n.healthy_pose.return_value=(.2,0.,0.)
        self.finish();self.n.path_client.send_goal_async.assert_not_called()
        self.setUp();self.response=done(NS(status=6,result=NS(error_code=208)))
        self.finish();self.n.path_client.send_goal_async.assert_not_called()
        self.assertIn('208',self.n.error)

    def test_stable_goal_moves_during_planning_old_plan_invalidated(self):
        self.request();n=self.n
        for now in (6.,6.3,6.6):
            with patch('nav2_follow.time.monotonic',return_value=now):n.update(4.,0.,1.)
        self.assertFalse(n.person_navigation.busy)
        self.assertFalse(n.pending)
        n.path_client.send_goal_async.assert_not_called()

    def test_followpath_send_exception_does_not_leave_pending(self):
        self.n.path_client.send_goal_async.side_effect=RuntimeError('send failed')
        self.finish();self.assertFalse(self.n.pending);self.assertFalse(self.n.enabled)

    def test_cancel_transport_failure_still_invalidates_and_stops(self):
        self.request();n=self.n
        n.person_navigation.handle=Mock()
        n.person_navigation.handle.cancel_goal_async.side_effect=RuntimeError('transport')
        n.stop('强停')
        self.assertFalse(n.pending);self.assertFalse(n.enabled)
        self.assertFalse(n.person_navigation.busy)

    def test_successful_terminal_result_updates_person_state(self):
        self.finish();n=self.n
        result=Future();handle=Mock(accepted=True);handle.get_result_async.return_value=result
        n.path_client.send_goal_async.return_value.set_result(handle)
        result.set_result(NS(status=4))
        self.assertEqual(n.person_navigation.state,'arrived')
        self.assertFalse(n.enabled)

    def test_arrival_discards_old_samples_and_waits_for_three_new_frames(self):
        n=self.n
        # Old traversal observations must not count toward the next target acquisition.
        for stamp in (8.,8.3,8.6):n.goal_filter.update((3.,0.,0.),stamp)
        n.handle=object();n.person_navigation.state='executing'
        with patch('nav2_follow.time.monotonic',return_value=10.):
            n._result(done(NS(status=4)),n.handle)
        self.assertEqual(len(n.goal_filter.samples),0)
        self.assertIsNone(n.person_anchor)
        n.person_motion.nav_measurement=(3.,0.,9.9)
        with patch('nav2_follow.time.monotonic',return_value=10.1):n.update(3.,0.,1.)
        self.assertEqual(len(n.goal_filter.samples),0)
        n.preview_client.send_goal_async.assert_not_called()
        for stamp in (10.2,10.5):
            n.person_motion.nav_measurement=(3.,0.,stamp)
            with patch('nav2_follow.time.monotonic',return_value=stamp):n.update(3.,0.,1.)
        n.preview_client.send_goal_async.assert_not_called()
        n.person_motion.nav_measurement=(3.,0.,10.8)
        with patch('nav2_follow.time.monotonic',return_value=10.8):n.update(3.,0.,1.)
        n.preview_client.send_goal_async.assert_called_once()
        self.assertIsNone(n.person_navigation.arrived_at)

    def test_small_target_motion_keeps_endpoint_and_path_until_arrival(self):
        n=self.n;n.enabled=True;n.handle=Mock();n.goal=(2.,0.,0.);n.goal_at=0.
        for stamp,distance in ((5.,3.1),(5.3,3.2),(5.6,3.15),(6.,3.2)):
            with patch('nav2_follow.time.monotonic',return_value=stamp):n.update(distance,0.,1.)
        self.assertEqual(n.goal,(2.,0.,0.))
        n.handle.cancel_goal_async.assert_not_called()
        n.preview_client.send_goal_async.assert_not_called()
        n.path_client.send_goal_async.assert_not_called()

    def test_missing_validator_fails_closed(self):
        self.n.validate_preview=None
        with self.assertRaises(RuntimeError):self.request()
        self.n.path_client.send_goal_async.assert_not_called()

    def test_blocked_person_goal_retries_then_executes_only_verified_alternative(self):
        n=self.n;person=n.person_navigation
        person.request((2.,0.,0.),alternatives=[(1.75,0.,0.)])
        seq,gen=person.sequence,n.generation;started=person.started_at
        person.finish(done(NS(status=6,result=NS(error_code=208))),seq,gen,10,(2.,0.,0.))
        self.assertEqual(n.preview_client.send_goal_async.call_count,2)
        self.assertEqual(person.started_at,started)
        self.assertEqual(n.goal,(1.75,0.,0.));self.assertFalse(n.enabled)
        n.path_client.send_goal_async.assert_not_called()
        self.path.poses[-1]=pose(1.75,0.)
        person.finish(self.response,seq,gen,10,(1.75,0.,0.))
        n.validate_preview.assert_called_once_with(self.path)
        self.assertIs(n.path_client.send_goal_async.call_args.args[0].path,self.path)

    def test_no_retry_after_stop_loss_id_change_or_deadline(self):
        for case in ('stop','loss','id','deadline','cancel'):
            self.setUp();n=self.n;p=n.person_navigation
            p.request((2.,0.,0.),alternatives=[(1.75,0.,0.)])
            seq,gen=p.sequence,n.generation
            if case=='stop':n.stop('强停')
            if case=='loss':n.person_motion.target_visible=False
            if case=='id':n.person_motion.settings.target_id=11
            if case=='deadline':p.started_at-=6
            status=5 if case=='cancel' else 6
            p.finish(done(NS(status=status,result=NS(error_code=208))),seq,gen,10,(2.,0.,0.))
            self.assertEqual(n.preview_client.send_goal_async.call_count,1,case)
            n.path_client.send_goal_async.assert_not_called()
            self.assertFalse(n.enabled)

    def test_stop_retains_only_display_history(self):
        self.finish();n=self.n
        n.stop('人物超时')
        self.assertEqual(n.person_navigation.route_points,[])
        self.assertEqual(len(n.person_navigation.last_route['points']),3)
        self.assertEqual(n.person_navigation.last_route['target_id'],10)
        self.assertFalse(n.enabled)

    def test_person_preview_retries_without_resetting_deadline(self):
        n=self.n;n.preview_path(3.,0.,1.)
        started=n.preview_at;sequence=n.preview_sequence
        n._validate_preview_result(done(NS(status=6,result=NS(error_code=208))),sequence)
        self.assertEqual(n.preview_client.send_goal_async.call_count,2)
        self.assertEqual(n.preview_at,started)
        self.assertAlmostEqual(n.preview_client.send_goal_async.call_args.args[0].goal.pose.position.x,1.75)
        n.path_client.send_goal_async.assert_not_called()
        n.preview_at-=6;n._preview_timeout()
        n._validate_preview_result(done(NS(status=6,result=NS(error_code=208))),sequence)
        self.assertEqual(n.preview_client.send_goal_async.call_count,2)

    def test_manual_endpoint_never_substituted_by_person_candidates(self):
        n=self.n;n.preview_path(3.,0.,1.);n.preview_goal(1.,1.)
        count=n.preview_client.send_goal_async.call_count
        n._validate_preview_result(done(NS(status=6,result=NS(error_code=208))),n.preview_sequence)
        self.assertEqual(n.preview_client.send_goal_async.call_count,count)
        self.assertEqual(n.preview['state'],'error')

    def test_click_world_point_planning_and_marker_survive_missing_live_target(self):
        n=self.n;n.healthy_pose.return_value=(1.,0.,0.)
        point={'id':10,'position':[3.,0.],'source_at':time.monotonic(),'stream_key':(1,'camera')}
        n.preview_person_point(point,1.)
        sent=n.preview_client.send_goal_async.call_args.args[0]
        self.assertAlmostEqual(sent.goal.pose.position.x,2.)
        self.assertEqual(n.person_target_marker['position'],[3.,0.])
        n.stop('人物短暂丢失')
        self.assertEqual(n.person_target_marker['position'],[3.,0.])
        n.path_client.send_goal_async.assert_not_called()

    def test_unreachable_click_keeps_marker_and_reports_failure(self):
        n=self.n;n.healthy_pose.side_effect=RuntimeError('车位过期')
        n.preview_person_point({'id':10,'position':[3.,0.],'source_at':time.monotonic()},1.)
        self.assertEqual(n.preview['state'],'error')
        self.assertEqual(n.person_target_marker['position'],[3.,0.])
        n.preview_client.send_goal_async.assert_not_called()

    def test_follow_plans_toward_clicked_reference_when_live_person_is_nearby(self):
        n=self.n
        n.person_locked_target={'id':10,'position':[3.,0.],'selected_at':5.}
        for stamp in (5.,5.3,5.6):
            with patch('nav2_follow.time.monotonic',return_value=stamp):n.update(3.2,0.,1.)
        sent=n.preview_client.send_goal_async.call_args.args[0]
        self.assertAlmostEqual(sent.goal.pose.position.x,2.)
        self.assertEqual(n.person_anchor,(3.,0.))
        n.path_client.send_goal_async.assert_not_called()

if __name__=='__main__':unittest.main()
