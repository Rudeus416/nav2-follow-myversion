# 【内容标注】用途：导航离线回归：person_execution。
# 对应用户需求：R17 R18 R30 R31（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明。
# 本次仅加注释；需求关联不是精确创建/提交记录。
"""Explicit preview-start tests, with fake ROS clients and no robot commands."""
import time
import unittest
from unittest.mock import Mock, patch
from fastapi import HTTPException
from nav2.person_execution import execute, readiness
from nav2.tests import test_person_navigation as fixtures


class PersonExecutionTests(unittest.TestCase):
    def setUp(self):
        case=fixtures.PersonNavigationTests();case.setUp()
        self.n=n=case.n;self.motion=m=n.person_motion;m.navigation=n
        m.estop=True;m.following=False;m.radar_reconfiguring=False
        n.fixed_goal_active=False;n.preview_sequence=n.person_marker_sequence=12
        n.preview_full_path=case.path;n.preview_at=time.monotonic()
        n.preview={'state':'ready','points':[(0,0),(2,0)]}
        n.person_target_marker={'id':10,'position':[3.,0.],'stream_key':(1,'camera')}
        n.person_stream_key=lambda:(1,'camera')
        self.people=[{'id':10,'position':[3.,0.],'source_at':time.monotonic()}]

    def start(self, sequence=12, continuous=False):
        return execute(self.motion,lambda:self.people,lambda:(1,'camera'),sequence,continuous)

    def test_click_executes_same_locked_path_without_replanning(self):
        n=self.n
        with patch.object(n.person_navigation,'execute_path',wraps=n.person_navigation.execute_path) as send:
            self.start()
        send.assert_called_once_with(n.preview_full_path,10,n.generation,locked=True)
        self.assertIs(n.path_client.send_goal_async.call_args.args[0].path,n.preview_full_path)
        self.assertTrue(n.fixed_goal_active)
        self.assertFalse(self.motion.estop);self.assertTrue(self.motion.following)
        self.assertEqual(self.motion.mode,'auto')
        self.assertEqual(n.person_navigation.state,'executing')
        n.preview_client.send_goal_async.assert_not_called()

    def test_old_click_and_expired_preview_do_not_move(self):
        with self.assertRaises(HTTPException):self.start(11)
        self.n.preview_at-=31
        with self.assertRaises(HTTPException):self.start()
        self.assertTrue(self.motion.estop)
        self.n.path_client.send_goal_async.assert_not_called()

    def test_stale_lost_or_moved_live_person_does_not_replace_locked_route(self):
        for continuous in (False,True):
            for case in ('stale','lost','moved','missing','tracking_stale','distance_stale','no_distance'):
                with self.subTest(continuous=continuous,case=case):
                    self.setUp();n=self.n;n.vision_enabled=True
                    if case=='stale':self.people[0]['source_at']-=3.
                    if case=='lost':self.motion.target_visible=False
                    if case=='moved':self.people[0]['position']=[4.,0.]
                    if case=='missing':self.people=[]
                    if case=='tracking_stale':self.motion.tracking_seen_at-=3.
                    if case=='distance_stale':self.motion.distance_seen_at-=3.
                    if case=='no_distance':self.motion.target_distance=None
                    # Explicit execute must not even request another live observation.
                    observe=Mock(side_effect=AssertionError('live observation is not part of locked start'))
                    path=n.preview_full_path;marker=n.person_target_marker
                    execute(self.motion,observe,lambda:(1,'camera'),12,continuous)
                    observe.assert_not_called()
                    self.assertIs(n.path_client.send_goal_async.call_args.args[0].path,path)
                    self.assertIs(n.person_target_marker,marker)
                    self.assertEqual(n.person_anchor,(3.,0.))
                    self.assertFalse(self.motion.estop)
                    n.preview_client.send_goal_async.assert_not_called()

    def test_changed_id_stream_or_invalid_marker_never_starts_old_route(self):
        cases=('id','stream','nan','infinite','missing_position','wrong_shape','text')
        for case in cases:
            with self.subTest(case=case):
                self.setUp();stream=(1,'camera');n=self.n
                if case=='id':self.motion.settings.target_id=11
                if case=='stream':stream=(1,'new')
                if case=='nan':n.person_target_marker['position']=[float('nan'),0.]
                if case=='infinite':n.person_target_marker['position']=[3.,float('inf')]
                if case=='missing_position':del n.person_target_marker['position']
                if case=='wrong_shape':n.person_target_marker['position']=[3.]
                if case=='text':n.person_target_marker['position']=['bad',0.]
                with self.assertRaises(HTTPException):
                    execute(self.motion,None,lambda:stream,12)
                self.assertTrue(self.motion.estop)
                n.path_client.send_goal_async.assert_not_called()

    def test_changed_preview_or_collision_during_validation_does_not_move(self):
        for case in ('changed','collision','pose','tf','server','stopped'):
            self.setUp()
            if case=='changed':self.n.validate_preview.side_effect=lambda _:self.n.clear_preview()
            if case=='collision':self.n.validate_preview.side_effect=ValueError('整车碰撞')
            if case=='pose':self.n.healthy_pose.return_value=(.2,0.,0.)
            if case=='tf':self.n.healthy_pose.side_effect=ValueError('里程计 TF 超时')
            if case=='server':self.n.path_client.server_is_ready.return_value=False
            if case=='stopped':
                # Original MotionController.stop_now performs both operations.
                def stop_during_validation(_):
                    self.n.stop('用户强停')
                    self.n.clear_preview()
                self.n.validate_preview.side_effect=stop_during_validation
            with self.assertRaises(HTTPException):self.start()
            self.assertTrue(self.motion.estop)
            self.n.path_client.send_goal_async.assert_not_called()

    def test_duplicate_click_does_not_send_second_path(self):
        self.start()
        with self.assertRaises(HTTPException):self.start()
        self.n.path_client.send_goal_async.assert_called_once()

    def test_send_failure_restores_stop(self):
        self.n.path_client.send_goal_async.side_effect=RuntimeError('transport')
        with self.assertRaises(HTTPException):self.start()
        self.assertTrue(self.motion.estop);self.assertFalse(self.motion.following)
        self.assertFalse(self.n.enabled);self.assertFalse(self.n.pending)


class PersonReadinessTests(unittest.TestCase):
    setUp = PersonExecutionTests.setUp
    start = PersonExecutionTests.start

    def ready(self, sequence=12, continuous=False, stream=(1,'camera')):
        with self.motion.lock,self.n.lock:
            return readiness(self.motion,self.people,lambda:stream,sequence,continuous)

    def test_readiness_reads_only_without_authorizing_motion(self):
        n=self.n;m=self.motion
        marker=n.person_target_marker;path=n.preview_full_path
        before=(m.estop,m.following,m.mode,n.pending,n.generation,n.person_navigation.state)
        self.assertEqual(self.ready(),{'ready':True,'reason':''})
        self.assertEqual((m.estop,m.following,m.mode,n.pending,n.generation,n.person_navigation.state),before)
        self.assertIs(n.person_target_marker,marker);self.assertIs(n.preview_full_path,path)
        n.healthy_pose.assert_not_called();n.validate_preview.assert_not_called()
        self.assertEqual(n.path_client.mock_calls,[])
        self.assertEqual(n.preview_client.mock_calls,[])

    def test_single_mode_remains_available_when_continuous_needs_vision(self):
        self.n.vision_enabled=False
        self.assertTrue(self.ready()['ready'])
        denied=self.ready(continuous=True)
        self.assertFalse(denied['ready']);self.assertIn('先启用视觉障碍检测',denied['reason'])
        self.n.vision_enabled=True
        self.assertTrue(self.ready(continuous=True)['ready'])

    def test_lightweight_reasons_match_locked_start_guards(self):
        cases=('id','sequence','preview_expired','stream','pending','bad_point','stopped','sensor','continuous_vision')
        for case in cases:
            with self.subTest(case=case):
                self.setUp();sequence=12;stream=(1,'camera');continuous=False
                if case=='id':self.motion.settings.target_id=11
                if case=='sequence':sequence=11
                if case=='preview_expired':self.n.preview_at-=31.
                if case=='stream':stream=(1,'new')
                if case=='pending':self.n.pending=True
                if case=='bad_point':self.n.person_target_marker['position']=[float('nan'),0.]
                if case=='stopped':self.motion.estop=False
                if case=='sensor':self.motion.radar_reconfiguring=True
                if case=='continuous_vision':continuous=True;self.n.vision_enabled=False
                result=self.ready(sequence=sequence,stream=stream,continuous=continuous)
                self.assertFalse(result['ready'])
                with self.assertRaises(HTTPException) as caught:
                    execute(self.motion,None,lambda:stream,sequence,continuous)
                self.assertEqual(result['reason'],caught.exception.detail)
                self.n.path_client.send_goal_async.assert_not_called()

    def test_readiness_uses_valid_locked_preview_after_live_observation_disappears(self):
        self.people=[];self.motion.target_visible=False;self.motion.target_distance=None
        self.motion.tracking_seen_at-=5.;self.motion.distance_seen_at-=5.
        self.assertEqual(self.ready(),{'ready':True,'reason':''})
        self.n.path_client.send_goal_async.assert_not_called()
        self.assertTrue(self.motion.estop)

    def test_readiness_does_not_replace_expensive_start_validation(self):
        self.assertTrue(self.ready()['ready'])
        self.n.validate_preview.side_effect=ValueError('整车碰撞')
        with self.assertRaises(HTTPException) as caught:self.start()
        self.assertIn('整车碰撞',caught.exception.detail)
        self.assertTrue(self.motion.estop)
        self.n.path_client.send_goal_async.assert_not_called()

    def test_execute_preserves_lock_release_and_rechecks_after_validation(self):
        events=[]
        def validate(path):
            self.assertFalse(self.motion.lock._is_owned())
            self.assertFalse(self.n.lock._is_owned())
            events.append('validate')
            self.n.generation+=1
        observe=Mock(side_effect=AssertionError('locked execution must not observe again'))
        self.n.validate_preview.side_effect=validate
        with self.assertRaises(HTTPException) as caught:
            execute(self.motion,observe,lambda:(1,'camera'),12)
        self.assertEqual(events,['validate'])
        observe.assert_not_called()
        self.assertIn('复核期间状态改变',caught.exception.detail)
        self.n.healthy_pose.assert_not_called()
        self.n.path_client.send_goal_async.assert_not_called()
        self.assertTrue(self.motion.estop)
