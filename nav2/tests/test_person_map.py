# 【内容标注】用途：导航离线回归：person_map。
# 对应用户需求：R15 R16 R30 R31（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明。
# 本次仅加注释；需求关联不是精确创建/提交记录。
import math
import threading
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
from fastapi import HTTPException
from nav2.person_map import observed_people, attach, DisplayPeopleCache, LatestPersonPoint


class App:
    def __init__(self):self.routes={}
    def get(self,name):return self.post(name)
    def post(self,name):
        def register(fn):self.routes[name]=fn;return fn
        return register


class PersonMapTests(unittest.TestCase):
    def setup_case(self):
        stamp=time.monotonic()
        person=NS(id=7,distance=2.,center=(50,50))
        track=NS(config_version=1,frame=NS(source_at=stamp,stream_epoch='camera',stamp_ns=12345),
                 people=(person,),calibration=NS(normalized=lambda x,y:([0.],[0.])))
        fused=NS(frame_id=1,config_version=1,people=(person,))
        engine=NS(lock=threading.RLock(),_config_version=1,_stream_epoch='camera',
                  _buffer_m1={1:track},_buffer_f={1:fused})
        transform=NS(translation=NS(x=1.,y=2.),rotation=NS(x=0.,y=0.,z=math.sin(math.pi/4),w=math.cos(math.pi/4)))
        nav=NS(lock=threading.RLock(),camera_pose=(0.,0.,0.),tf=NS(lookup_transform=Mock(return_value=NS(transform=transform))),
               person_navigation=NS(state='idle',message='等待',route_points=[]),enabled=False,
               fixed_goal_active=False,pending=False,canceling=False,handle=None,vision_enabled=False,
               goal=None,healthy_pose=Mock(return_value=(1.,2.,0.)))
        motion=NS(lock=threading.RLock(),navigation=nav,settings=NS(target_id=7,follow_distance_m=1.),
                  following=False,mode='auto',estop=True,radar_reconfiguring=False,set_following=Mock(),
                  target_visible=True,target_distance=2.,tracking_seen_at=stamp,distance_seen_at=stamp,
                  target_timeout=lambda:1.2)
        nav.preview={}
        def preview_point(point, stand_off):
            nav.person_target_marker=dict(point)
            nav.person_marker_sequence=nav.preview_sequence=1
            nav.preview_at=time.monotonic()
            nav.preview={'state':'pending','message':'正在规划','points':[]}
        nav.preview_person_point=Mock(side_effect=preview_point)
        app=App();attach(app,engine,motion)
        return engine,motion,track,person,app

    def test_marks_matching_person_at_capture_pose_without_starting(self):
        engine,motion,track,person,app=self.setup_case()
        state=app.routes['/api/nav2/people-map']()
        point=state['people'][0]['position']
        self.assertAlmostEqual(point[0],1.);self.assertAlmostEqual(point[1],4.)
        self.assertEqual(state['selected_id'],7)
        request=motion.navigation.tf.lookup_transform.call_args.args[2]
        self.assertEqual(request.nanoseconds,12345)
        motion.set_following.assert_not_called()

    def test_stale_or_lost_person_is_hidden(self):
        engine,motion,track,person,app=self.setup_case()
        track.frame.source_at-=2
        self.assertEqual(observed_people(engine,motion),[])
        track.frame.source_at=time.monotonic();track.people=()
        self.assertEqual(observed_people(engine,motion),[])

    def test_invalid_distance_or_old_stream_is_hidden(self):
        engine,motion,track,person,app=self.setup_case()
        person.distance=float('nan');self.assertEqual(observed_people(engine,motion),[])
        person.distance=2.;track.frame.stream_epoch='old'
        self.assertEqual(observed_people(engine,motion),[])

    def test_estop_cannot_be_released_by_start(self):
        engine,motion,track,person,app=self.setup_case()
        with self.assertRaises(HTTPException):app.routes['/api/nav2/person-start']()
        self.assertTrue(motion.estop);motion.set_following.assert_not_called()

    def test_start_delegates_to_existing_follow_authorization(self):
        engine,motion,track,person,app=self.setup_case();motion.estop=False
        response=app.routes['/api/nav2/person-start']()
        self.assertEqual(response['selected_id'],7)
        motion.set_following.assert_called_once_with(True)
        motion.navigation.healthy_pose.assert_called_once()

    def test_selected_id_change_does_not_follow_previous_marker(self):
        engine,motion,track,person,app=self.setup_case();motion.estop=False;motion.settings.target_id=8
        with self.assertRaises(HTTPException):app.routes['/api/nav2/person-start']()
        motion.set_following.assert_not_called()

    def test_point_navigation_is_not_replaced_by_person_button(self):
        engine,motion,track,person,app=self.setup_case();motion.estop=False;motion.navigation.fixed_goal_active=True
        with self.assertRaises(HTTPException):app.routes['/api/nav2/person-start']()
        motion.set_following.assert_not_called()

    def test_route_is_visible_only_during_authorized_person_execution(self):
        engine,motion,track,person,app=self.setup_case();nav=motion.navigation
        nav.enabled=True;nav.person_navigation.state='executing';nav.person_navigation.route_points=[(1.,2.),(1.,3.)];nav.goal=(1.,3.,0.)
        self.assertEqual(app.routes['/api/nav2/people-map']()['route'],[])
        motion.estop=False;motion.following=True
        self.assertEqual(len(app.routes['/api/nav2/people-map']()['route']),2)
        motion.following=False
        self.assertEqual(app.routes['/api/nav2/people-map']()['route'],[])


class DisplayCacheTests(unittest.TestCase):
    def test_missing_frame_retains_labelled_position_without_renewing_timestamp(self):
        c=DisplayPeopleCache();p={'id':7,'position':[1,2],'source_at':10.}
        self.assertFalse(c.update([p],(1,'camera',7),10.1)[0]['stale'])
        stale=c.update([],(1,'camera',7),10.5)[0]
        self.assertTrue(stale['stale']);self.assertEqual(stale['source_at'],10.)
        self.assertEqual(c.update([p],(1,'camera',7),13.1),[])

    def test_stream_config_and_selection_changes_clear_old_positions(self):
        for key in ((2,'camera',7),(1,'new',7),(1,'camera',8)):
            c=DisplayPeopleCache();c.update([{'id':7,'position':[1,2],'source_at':10.}],(1,'camera',7),10.)
            self.assertEqual(c.update([],key,10.5),[])

    def test_cached_marker_cannot_start_motion(self):
        engine,motion,track,person,app=PersonMapTests().setup_case()
        app.routes['/api/nav2/people-map']()
        track.people=();motion.estop=False
        state=app.routes['/api/nav2/people-map']()
        self.assertTrue(state['people'][0]['stale'])
        with self.assertRaises(HTTPException):app.routes['/api/nav2/person-start']()
        motion.set_following.assert_not_called()

    def test_history_is_separate_from_executable_route_and_selected_id(self):
        engine,motion,track,person,app=PersonMapTests().setup_case()
        motion.navigation.person_navigation.last_route={'points':[(0,0),(1,1)],'target_id':7,'at':time.monotonic()}
        state=app.routes['/api/nav2/people-map']()
        self.assertEqual(state['route'],[]);self.assertEqual(len(state['historical_route']),2)
        motion.settings.target_id=8
        self.assertEqual(app.routes['/api/nav2/people-map']()['historical_route'],[])


class ClickedPersonTests(unittest.TestCase):
    def test_missing_detection_after_display_still_latches_recent_point_for_preview(self):
        engine,motion,track,person,app=PersonMapTests().setup_case()
        old=app.routes['/api/nav2/people-map']()['people'][0]
        track.people=()
        response=app.routes['/api/nav2/person-preview']()
        self.assertEqual(response['target_marker']['position'],old['position'])
        self.assertEqual(response['target_marker']['source_at'],old['source_at'])
        self.assertEqual(response['preview']['state'],'pending')
        motion.set_following.assert_not_called()
        # It remains a historical observation, not a fresh motion authorization.
        motion.estop=False
        with self.assertRaises(HTTPException):app.routes['/api/nav2/person-start']()

    def test_expired_display_point_still_plans_but_wrong_id_never_does(self):
        from unittest.mock import patch
        engine,motion,track,person,app=PersonMapTests().setup_case()
        app.routes['/api/nav2/people-map']()
        track.people=()
        with patch('nav2.person_map.time.monotonic',return_value=track.frame.source_at+600.):
            self.assertEqual(app.routes['/api/nav2/people-map']()['people'], [])
            result=app.routes['/api/nav2/person-preview']()
            self.assertEqual(result['target_marker']['source_at'],track.frame.source_at)
            self.assertEqual(result['target_marker']['age'],600.)
        motion.navigation.preview_person_point.assert_called_once()
        motion.settings.target_id=8
        with self.assertRaises(HTTPException):app.routes['/api/nav2/person-preview']()

    def test_preview_requires_stop_and_start_latches_fresh_point(self):
        engine,motion,track,person,app=PersonMapTests().setup_case();motion.estop=False
        with self.assertRaises(HTTPException):app.routes['/api/nav2/person-preview']()
        response=app.routes['/api/nav2/person-start']()
        self.assertEqual(response['target_marker']['id'],7)
        motion.navigation.preview_person_point.assert_called_once()

    def test_latched_point_survives_display_cache_expiry_but_not_new_id(self):
        from unittest.mock import patch
        engine,motion,track,person,app=PersonMapTests().setup_case()
        app.routes['/api/nav2/person-preview']();track.people=()
        with patch('nav2.person_map.time.monotonic',return_value=track.frame.source_at+10.):
            state=app.routes['/api/nav2/people-map']()
        self.assertEqual(state['people'],[])
        self.assertEqual(state['target_marker']['id'],7)
        motion.settings.target_id=8
        self.assertIsNone(app.routes['/api/nav2/people-map']()['target_marker'])


class PersonFeedbackTests(unittest.TestCase):
    """Explicit readiness belongs to the locked point and its valid preview."""
    def setup_ready(self):
        engine,motion,track,person,app=PersonMapTests().setup_case()
        app.routes['/api/nav2/person-preview']()
        nav=motion.navigation
        nav.preview={'state':'ready','points':[(1.,2.),(1.,3.)]}
        nav.preview_full_path=NS(header=NS(frame_id='odom'),poses=[object()])
        nav.validate_preview=Mock()
        nav.path_client=Mock()
        return engine,motion,track,person,app

    def test_state_exposes_single_and_continuous_reasons_without_motion_or_extra_tf(self):
        engine,motion,track,person,app=self.setup_ready();nav=motion.navigation
        before_calls=nav.tf.lookup_transform.call_count
        with self.assertNoLogs('nav2.person_map',level='WARNING'):
            state=app.routes['/api/nav2/people-map']()
        self.assertEqual(state['start_readiness']['single'],{'ready':True,'reason':''})
        self.assertFalse(state['start_readiness']['continuous']['ready'])
        self.assertIn('先启用视觉障碍',state['start_readiness']['continuous']['reason'])
        # One existing projection request; lightweight readiness adds none.
        self.assertEqual(nav.tf.lookup_transform.call_count,before_calls+1)
        nav.healthy_pose.assert_not_called();nav.validate_preview.assert_not_called()
        self.assertEqual(nav.path_client.mock_calls,[])
        motion.set_following.assert_not_called();self.assertTrue(motion.estop)
        nav.vision_enabled=True
        self.assertTrue(app.routes['/api/nav2/people-map']()['start_readiness']['continuous']['ready'])

    def test_cached_person_does_not_invalidate_locked_start(self):
        engine,motion,track,person,app=self.setup_ready()
        app.routes['/api/nav2/people-map']();track.people=()
        state=app.routes['/api/nav2/people-map']()
        self.assertTrue(state['people'][0]['stale'])
        self.assertIsNotNone(state['target_marker'])
        self.assertTrue(state['start_readiness']['single']['ready'])
        motion.target_visible=False;motion.target_distance=None
        with patch('nav2.person_map.time.monotonic',return_value=track.frame.source_at+5.):
            state=app.routes['/api/nav2/people-map']()
        self.assertEqual(state['people'],[])
        self.assertIsNotNone(state['target_marker'])
        self.assertTrue(state['start_readiness']['single']['ready'])

    def test_id_or_stream_change_cannot_use_previous_start(self):
        for changed in ('id','stream'):
            with self.subTest(changed=changed):
                engine,motion,track,person,app=self.setup_ready()
                if changed=='id':motion.settings.target_id=8
                else:engine._stream_epoch='next'
                state=app.routes['/api/nav2/people-map']()
                self.assertIsNone(state['target_marker'])
                self.assertFalse(state['start_readiness']['single']['ready'])
                motion.set_following.assert_not_called()

    def test_preview_rejection_logs_operation_status_and_original_detail(self):
        engine,motion,track,person,app=PersonMapTests().setup_case();motion.estop=False
        with self.assertLogs('nav2.person_map',level='WARNING') as logs:
            with self.assertRaises(HTTPException) as caught:
                app.routes['/api/nav2/person-preview']()
        self.assertEqual(caught.exception.status_code,409)
        self.assertEqual(caught.exception.detail,'请先强停，再记录人物点并规划')
        self.assertIn('operation=person-preview',logs.output[0])
        self.assertIn('status=409',logs.output[0])
        self.assertIn(caught.exception.detail,logs.output[0])
        motion.navigation.preview_person_point.assert_not_called()

    def test_execute_rejection_is_logged_and_reraised_without_rewriting(self):
        engine,motion,track,person,app=PersonMapTests().setup_case()
        rejection=HTTPException(409,'连续追踪需要先启用视觉障碍检测，以监测途中突发障碍')
        with patch('nav2.person_execution.execute',side_effect=rejection) as execute:
            with self.assertLogs('nav2.person_map',level='WARNING') as logs:
                with self.assertRaises(HTTPException) as caught:
                    app.routes['/api/nav2/person-execute']({'sequence':12,'continuous':True})
        self.assertIs(caught.exception,rejection)
        self.assertIn('operation=person-execute',logs.output[0])
        self.assertIn('status=409',logs.output[0]);self.assertIn(rejection.detail,logs.output[0])
        self.assertEqual(execute.call_args.kwargs,{'continuous':True})
        self.assertIsNone(execute.call_args.args[1])
        self.assertTrue(motion.estop);motion.set_following.assert_not_called()

    def test_runtime_source_provider_reads_configuration_without_live_observations(self):
        engine,motion,track,person,app=self.setup_ready()
        self.assertEqual(motion.navigation.person_stream_key(),(1,'camera'))
        engine._config_version=2;engine._stream_epoch='next'
        self.assertEqual(motion.navigation.person_stream_key(),(2,'next'))


class LatestHistoryTests(unittest.TestCase):
    def test_history_has_no_age_deadline_but_input_must_have_been_fresh(self):
        cache=LatestPersonPoint();key=(1,'camera',7)
        point={'id':7,'position':[1.,2.],'source_at':10.}
        self.assertIsNone(cache.update([point],key,14.))
        cache.update([point],key,10.2)
        old=cache.update([],key,3610.)
        self.assertEqual(old['position'],[1.,2.]);self.assertEqual(old['source_at'],10.)
        self.assertEqual(old['age'],3600.)
        self.assertTrue(old['stale'])

    def test_older_or_invalid_measurement_cannot_replace_latest(self):
        cache=LatestPersonPoint();key=(1,'camera',7)
        point={'id':7,'position':[1.,2.],'source_at':10.5}
        cache.update([point],key,10.6)
        for candidate in (dict(point,source_at=10.4,position=[9,9]),
                          dict(point,position=[float('nan'),2],source_at=10.7),
                          dict(point,source_at=float('nan')),dict(point,position=[1]),
                          dict(point,id=8,source_at=10.7)):
            self.assertEqual(cache.update([candidate],key,10.8)['source_at'],10.5)
        saved=cache.update([],key,20.);saved['position'][0]=99
        self.assertEqual(cache.update([],key,20.)['position'],[1.,2.])

    def test_new_valid_frame_replaces_position_without_refreshing_previous_frame(self):
        cache=LatestPersonPoint();key=(1,'camera',7)
        p={'id':7,'position':[1.,2.],'source_at':10.}
        cache.update([p],key,10.)
        self.assertEqual(cache.update([p],key,50.)['source_at'],10.)
        new=dict(p,position=[2.,3.],source_at=51.)
        self.assertEqual(cache.update([new],key,51.1)['position'],[2.,3.])

    def test_id_stream_or_config_change_discards_history(self):
        for key in ((2,'camera',7),(1,'new',7),(1,'camera',8)):
            cache=LatestPersonPoint();initial=(1,'camera',7)
            cache.update([{'id':7,'position':[1,2],'source_at':10.}],initial,10.)
            self.assertIsNone(cache.update([],key,20.))
            self.assertIsNone(cache.update([],initial,21.))

    def test_api_scope_change_rejects_old_history_and_no_history_cannot_plan(self):
        for change in ('stream','config','id','never_seen'):
            engine,motion,track,person,app=PersonMapTests().setup_case()
            if change!='never_seen':app.routes['/api/nav2/people-map']()
            track.people=()
            if change=='stream':engine._stream_epoch='new'
            if change=='config':engine._config_version+=1
            if change=='id':motion.settings.target_id=8
            with self.assertRaises(HTTPException):app.routes['/api/nav2/person-preview']()
            motion.navigation.preview_person_point.assert_not_called()
