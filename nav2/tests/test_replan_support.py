# 【R32】全量视觉证据、最新地图与速度出口；离线模拟，不连接车辆。
import copy
import threading
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import numpy as np
from nav2 import replan_support as support
from nav2.tests.test_person_navigation import pose
from nav2_follow import VisionStale


FOOT = np.array([[.297,.232],[.297,-.232],[-.297,-.232],[-.297,.232]])


class ReplanSupportTests(unittest.TestCase):
    def setUp(self):
        self.now = time.monotonic()
        self.nav = n = NS(lock=threading.RLock(), vision_enabled=True, vision_at=self.now,
                          vision_error='', vision_near_blocked=False,
                          healthy_pose=Mock(return_value=(0.,0.,0.)), stop=Mock(),
                          enabled=True, generation=1)
        n.vision_safety = dict(points=np.array([[1.,0.]]), source_at=self.now,
                              footprint=FOOT, footprint_at=self.now)
        n.tf=NS(lookup_transform=Mock(return_value=NS(transform=NS(
            translation=NS(x=0.,y=0.),rotation=NS(x=0.,y=0.,z=0.,w=1.)))))
        self.path = NS(poses=[pose(0.,0.), pose(1.,0.), pose(2.,0.)])
        self.task = NS(active=True, holding=False, permits_near=False,
                       current_path=self.path, trigger=Mock(return_value=True))
        n.person_navigation = NS(obstacle_replan=self.task)
        self.snap = {'layers':{'global_map':{'received':self.now, 'stale':False},
            'robot':{'position':[0.,0.], 'yaw':0., 'stale':False, 'age':.1},
            'footprint':{'points':FOOT.tolist(), 'stale':False, 'age':.2, 'stamp':123.}},
            'editing':{'version':1}}
        self.view = NS(snapshot=Mock(side_effect=lambda **kw: copy.deepcopy(self.snap)))
        support.attach(n, self.view)

    def test_only_fresh_typed_near_error_can_be_ignored_for_planning(self):
        n = self.nav; n.vision_error = 'near'; n.vision_near_blocked = True
        with self.assertRaises(support.NearObstacle): support.check_vision(n)
        support.check_vision(n, allow_near=True)
        n.vision_near_blocked = False
        with self.assertRaisesRegex(RuntimeError, 'near'): support.check_vision(n, allow_near=True)
        n.vision_near_blocked = True; n.vision_at -= 2.
        with self.assertRaises(VisionStale): support.check_vision(n, allow_near=True)

    def test_ready_pose_never_bypasses_tf_or_missing_full_cloud(self):
        self.nav.healthy_pose.side_effect = RuntimeError('TF lost')
        with self.assertRaisesRegex(RuntimeError, 'TF'): self.nav.replan_ready_pose()
        self.nav.healthy_pose.side_effect = None; self.nav.vision_safety = None
        with self.assertRaisesRegex(RuntimeError, '全量'): self.nav.replan_ready_pose()

    def test_missing_stale_footprint_or_disabled_visual_fails_closed(self):
        for kind in ('missing', 'stale', 'disabled'):
            with self.subTest(kind=kind):
                self.setUp()
                if kind == 'missing': self.nav.vision_safety['footprint'] = None
                elif kind == 'stale': self.nav.vision_safety['footprint_at'] -= 4
                else: self.nav.vision_enabled = False
                with self.assertRaises(RuntimeError): self.nav.replan_ready_pose()

    def test_capture_preserves_source_age_and_outside_map_points(self):
        points = np.array([[-3.,5.],[.4,.8],[1.,0.]])
        saved = support.capture_visual_safety(self.nav, points, self.now-.5)
        points[:] = 99
        self.assertEqual(saved['source_at'], self.now-.5)
        self.assertEqual(saved['points'][0].tolist(), [-3.,5.])
        self.assertFalse(saved['points'].flags.writeable)
        self.assertFalse(saved['footprint'].flags.writeable)
        self.assertLess(saved['footprint_at'], time.monotonic()-.15)

    def test_footprint_uses_its_own_tf_not_newer_robot_marker(self):
        self.snap['layers']['robot']['position'] = [.2,0.]
        body, _ = self.nav.navigation_footprint()
        np.testing.assert_allclose(body, FOOT)
        query = self.nav.tf.lookup_transform.call_args.args[2]
        self.assertEqual(query.nanoseconds, 123000000000)

    def test_validation_uses_both_existing_map_validator_and_full_cloud(self):
        self.nav.vision_safety['points'] = np.array([[1.,1.]])
        with patch('nav2.route_preview.validate_route_snapshot') as original:
            self.nav.validate_replan(self.path, self.now-.1)
            original.assert_called_once()
            self.assertIs(original.call_args.args[0], self.path)

    def test_stale_global_snapshot_before_hazard_cannot_authorize(self):
        self.snap['layers']['global_map']['received'] -= 2
        with patch.object(support.time, 'monotonic', side_effect=[10.,11.]):
            with self.assertRaisesRegex(RuntimeError, '全局地图'):
                self.nav.validate_replan(self.path, self.now-.1)

    def test_new_cloud_obstacle_rejects_path_even_if_costmap_has_not_caught_up(self):
        with patch('nav2.route_preview.validate_route_snapshot'):
            with self.assertRaisesRegex(RuntimeError, '扫掠'):
                self.nav.validate_replan(self.path, self.now-.1)

    def test_static_purple_map_rejection_is_not_overridden(self):
        with patch('nav2.route_preview.validate_route_snapshot', side_effect=ValueError('紫色')):
            with self.assertRaisesRegex(ValueError, '紫色'):
                self.nav.validate_replan(self.path, self.now-.1)

    def test_edit_during_geometry_is_rejected(self):
        self.nav.vision_safety['points'] = np.empty((0,2))
        def edited(*args): self.snap['editing']['version'] = 2
        with patch('nav2.route_preview.validate_route_snapshot', side_effect=edited):
            with self.assertRaisesRegex(RuntimeError, '已改变'):
                self.nav.validate_replan(self.path, self.now-.1)

    def test_new_obstacle_on_remaining_path_triggers_fixed_goal_recovery(self):
        support.monitor_route(self.nav)
        self.task.trigger.assert_called_once()

    def test_obstacle_on_completed_path_does_not_trigger_replan(self):
        self.nav.healthy_pose.return_value = (1.,0.,0.)
        self.nav.vision_safety['points'] = np.array([[0.,0.]])
        support.monitor_route(self.nav)
        self.task.trigger.assert_not_called()

    def test_progress_never_jumps_to_a_nearby_return_leg_or_backwards(self):
        path = NS(poses=[pose(x,0.) for x in np.linspace(0,2,41)]+[
            pose(x,.1) for x in np.linspace(2,0,41)])
        first = support.remaining_index(self.nav,path,(.1,.09,0.))
        self.assertLess(first, 12)
        next_index = support.remaining_index(self.nav,path,(.4,0.,0.))
        self.assertGreaterEqual(next_index, first)
        back = support.remaining_index(self.nav,path,(0.,0.,0.))
        self.assertEqual(back, next_index)

    def test_late_monitor_result_cannot_affect_new_path(self):
        def stop_during_check(*args, **kwargs):
            self.nav.generation += 1
            return False
        with patch('nav2.replan_safety.path_clear', side_effect=stop_during_check):
            support.monitor_route(self.nav)
        self.task.trigger.assert_not_called()

    def test_command_toward_close_obstacle_is_never_released(self):
        self.nav.vision_safety['points'] = np.array([[.31,0.]])
        self.assertFalse(support.guard_command(self.nav, (0.,0.,0.), (.18,0.)))
        self.task.trigger.assert_called_once()

    def test_empty_but_fresh_full_cloud_allows_motion(self):
        self.nav.vision_safety['points'] = np.empty((0,2))
        self.assertTrue(support.guard_command(self.nav, (0.,0.,0.), (.1,.1)))
        self.task.trigger.assert_not_called()

    def test_visual_route_monitor_and_footprint_read_do_not_hold_control_lock(self):
        from nav2.vision_layer import VisionLayer
        layer=VisionLayer.__new__(VisionLayer)
        frame=NS(source_at=time.monotonic(), frame_id=1, stream_epoch=1)
        record=NS(config_version=1, frame=frame)
        layer.engine=NS(lock=threading.RLock(), _config_version=1, _stream_epoch=1,
                        _buffer_m2={1:record})
        layer.nav=NS(lock=threading.RLock(),vision_enabled=True,enabled=True,generation=1,
                     person_navigation=NS(obstacle_replan=NS(active=True,holding=False,
                         current_path=self.path)))
        layer.semantic=NS(set_enabled=Mock(),offer=Mock(),snapshot=Mock(return_value=None))
        layer._tick_locked=Mock()
        checks=[];checked=threading.Event()
        def check_unlocked(*args,**kwargs):
            acquired=[]
            def worker():
                locked=layer.nav.lock.acquire(timeout=.2)
                acquired.append(locked)
                if locked: layer.nav.lock.release()
            worker_thread=threading.Thread(target=worker)
            worker_thread.start(); worker_thread.join(1.)
            self.assertEqual(acquired,[True])
            checks.append(True)
            if len(checks)==2:checked.set()
            return FOOT, time.monotonic()
        layer.nav.navigation_footprint=check_unlocked
        with patch('nav2.replan_support.monitor_route',side_effect=check_unlocked):
            try:
                layer.tick()
                self.assertTrue(checked.wait(1.))
            finally:
                monitor=getattr(layer,'_route_monitor',None)
                if monitor is not None:monitor.close()
        self.assertEqual(len(checks),2)

    def test_inactive_legacy_navigation_does_not_acquire_new_requirements(self):
        self.task.active = False; self.nav.vision_safety = None
        self.assertTrue(support.guard_command(self.nav, (0.,0.,0.), (.1,0.)))
        support.monitor_route(self.nav)
        self.nav.stop.assert_not_called()

    def test_lightweight_footprint_never_reads_editor_or_whole_map(self):
        self.view.lock = threading.Lock()
        self.view.layers = {'footprint':dict(points=FOOT.tolist(), stamp=123.,
                                             received=self.now-.2)}
        self.view.node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=123100000000)))
        body, at = self.nav.navigation_footprint()
        np.testing.assert_allclose(body, FOOT)
        self.assertLessEqual(at, self.now-.2+.001)
        self.view.snapshot.assert_not_called()
        # Receiving an old stamped footprint now cannot make it fresh.
        self.view.layers['footprint'].update(stamp=119., received=time.monotonic())
        with self.assertRaisesRegex(RuntimeError, '轮廓'):
            self.nav.navigation_footprint()

    def test_duplicate_disabled_and_stale_frames_skip_added_footprint_work(self):
        from nav2.vision_layer import VisionLayer
        layer = VisionLayer.__new__(VisionLayer)
        frame = NS(source_at=time.monotonic(), frame_id=1, stream_epoch=1)
        record = NS(config_version=1, frame=frame)
        layer.engine = NS(lock=threading.RLock(), _config_version=1, _stream_epoch=1,
                          _buffer_m2={1:record})
        layer.nav = NS(lock=threading.RLock(), vision_enabled=True,
                       navigation_footprint=Mock(return_value=(FOOT,self.now)))
        layer.semantic = NS(set_enabled=Mock(),offer=Mock(),snapshot=Mock(return_value=None),close=Mock())
        layer._tick_locked = Mock()
        with patch('nav2.replan_support.monitor_route') as monitor:
            layer.tick(); layer.tick()
            layer.nav.navigation_footprint.assert_called_once()
            monitor.assert_not_called()  # Manual point route has no recovery task.
            frame.frame_id=2; frame.source_at=time.monotonic()-2.
            layer.tick()
            frame.source_at=time.monotonic(); layer.nav.vision_enabled=False
            layer.tick()
            layer.nav.navigation_footprint.assert_called_once()
        self.assertEqual(layer._tick_locked.call_count,4)  # Existing safety gates still run.


if __name__ == '__main__': unittest.main()
