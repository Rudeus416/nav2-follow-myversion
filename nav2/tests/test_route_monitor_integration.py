# 【R40】用户“优化”：视觉新帧与全路径复核解耦，不发送 ROS 或底盘动作。
import copy
import threading
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from nav2 import replan_support as support
from nav2.tests.simulate_vision_contention import fixture, fake_ros_time
from nav2.tests.test_person_navigation import pose


class RouteMonitorIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.ros=fake_ros_time();self.ros.__enter__()
        self.addCleanup(self.ros.__exit__,None,None,None)
        self.layer,self.nav,self.record,self.published=fixture()
        self.nav.vision_layer=self.layer
        self.task=NS(active=True,holding=False,permits_near=False,
                     current_path=NS(poses=[pose(0.,0.),pose(2.,0.)]),
                     trigger=Mock(return_value=True))
        self.nav.person_navigation=NS(obstacle_replan=self.task)
        self.nav.replan_ready_pose=lambda:(0.,0.,0.)
        self.addCleanup(self.cleanup)

    def cleanup(self):
        monitor=getattr(self.layer,'_route_monitor',None)
        if monitor is not None:monitor.close()
        self.layer._depth_mailbox.close()
        self.layer._epoch_gate.close()

    def new_frame(self):
        r=copy.copy(self.record);r.frame=copy.copy(r.frame)
        r.frame.source_at=time.monotonic()-.03
        r.frame.frame_id+=1;r.frame.stamp_ns=int(r.frame.source_at*1e9)
        self.layer._depth_mailbox._depth.on_result(r)
        return r

    def test_blocked_route_geometry_does_not_block_new_visual_commit(self):
        entered=threading.Event();release=threading.Event();done=threading.Event()
        def geometry(*args,**kwargs):
            entered.set()
            if not release.wait(2.):raise RuntimeError('test release timeout')
            return True
        errors=[]
        def tick():
            try:self.layer.tick()
            except BaseException as exc:errors.append(exc)
            finally:done.set()
        with patch('nav2.replan_safety.path_clear',side_effect=geometry):
            thread=threading.Thread(target=tick);thread.start()
            try:
                self.assertTrue(entered.wait(1.))
                self.assertTrue(done.wait(.3),'vision tick waited on remaining-route sweep')
                r=self.new_frame();self.layer.tick()
                self.assertEqual(self.nav.vision_at,r.frame.source_at)
                self.assertEqual(len(self.published),2)
                self.assertFalse(self.task.trigger.called)
            finally:
                release.set();thread.join(2.)
                self.layer._route_monitor.close()
        self.assertEqual(errors,[])

    def test_context_invalidation_discards_late_obstacle_result(self):
        entered=threading.Event();release=threading.Event()
        def geometry(*args,**kwargs):
            entered.set();release.wait(2.);return False
        with patch('nav2.replan_safety.path_clear',side_effect=geometry):
            try:
                self.layer.tick();self.assertTrue(entered.wait(1.))
                with self.nav.lock:self.layer.invalidate()
            finally:
                release.set();self.layer._route_monitor.close()
        self.task.trigger.assert_not_called()
        self.assertTrue(self.nav.enabled)
        self.assertEqual(self.nav.vision_at,0.)

    def test_closed_owner_skips_health_checks_before_stopping_a_new_task(self):
        self.nav.replan_ready_pose=Mock(side_effect=RuntimeError('old error'))
        support.monitor_route(self.nav,valid=lambda:False)
        self.nav.replan_ready_pose.assert_not_called()
        self.assertTrue(self.nav.enabled)

    def test_revoked_owner_cannot_commit_collision_result(self):
        self.layer.tick()
        self.layer._route_monitor.close()
        current=[True]
        def geometry(*args,**kwargs):current[0]=False;return False
        self.nav._visual_route_checked=None
        with patch('nav2.replan_safety.path_clear',side_effect=geometry):
            support.monitor_route(self.nav,valid=lambda:current[0])
        self.task.trigger.assert_not_called()

    def test_phase_metrics_separate_publication_without_changing_source_time(self):
        self.task.active=False
        self.layer.publisher=NS(publish=lambda _:time.sleep(.02))
        self.layer.tick()
        phases=self.layer.timing['phases']
        self.assertGreaterEqual(phases['publish'],.015)
        self.assertIn('full_cloud',phases)
        self.assertIn('candidates',phases)
        self.assertEqual(self.nav.vision_at,self.record.frame.source_at)


if __name__=='__main__':unittest.main()
