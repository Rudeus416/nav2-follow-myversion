# 【内容标注 / R37】离线检查锁外视觉计算的提交边界；不连接底盘/ROS节点。
"""Exercise configuration/cancel races in the real visual consumer."""
import threading
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
from nav2.tests.simulate_vision_contention import fixture, fake_ros_time
from nav2.vision_epoch import VisionEpoch
from nav2.vision_layer import depth_points


class VisionCommitTests(unittest.TestCase):
    def setUp(self):
        self.layer,self.nav,self.record,self.published=fixture()
        self.layer.cells={};self.layer.camera_cells={};self.layer.diagnostics={}
        self.layer.semantic_preview=None
        self.ros=fake_ros_time();self.ros.__enter__()

    def tearDown(self):
        self.layer._depth_mailbox.close()
        self.layer._epoch_gate.close()
        self.layer.semantic.close()
        self.ros.__exit__(None,None,None)

    def during_first_geometry(self, mutation):
        first=True
        def compute(*args,**kwargs):
            nonlocal first
            if first:
                first=False;mutation()
            return depth_points(*args,**kwargs)
        with patch('nav2.vision_layer.depth_points',side_effect=compute):self.layer.tick()

    def test_user_cancel_during_geometry_cannot_restore_evidence_or_task(self):
        old=self.nav.vision_at
        self.during_first_geometry(lambda:self.nav.stop('用户取消'))
        self.assertFalse(self.nav.enabled)
        self.assertEqual(self.nav.command,(0.,0.))
        self.assertEqual(self.nav.vision_at,old)
        self.assertIsNone(self.nav.vision_safety)
        self.assertEqual(self.published,[])

    def test_calibration_change_discards_computed_old_scale(self):
        def change():
            with self.nav.lock:
                self.layer.invalidate();self.layer.scale=2.
                self.nav.stop('应用校准')
        self.during_first_geometry(change)
        self.assertEqual(self.nav.vision_at,0.)
        self.assertIsNone(self.nav.vision_safety)
        self.assertEqual(self.published,[])

    def test_configuration_change_discards_old_frame_despite_completed_work(self):
        engine=self.layer.engine
        def configure():
            with engine.lock:engine._config_version+=1
        engine.configure=configure
        self.layer._epoch_gate.close()
        self.layer._epoch_gate=VisionEpoch(engine,self.layer._configuration_changed)
        self.during_first_geometry(engine.configure)
        self.assertFalse(self.nav.enabled)
        self.assertEqual(self.nav.vision_at,0.)
        self.assertEqual(self.published,[])
        self.assertEqual(self.layer._epoch_gate.snapshot().version,2)

    def test_change_during_publish_cannot_commit_late_display(self):
        def publish(_):
            with self.nav.lock:
                self.layer.invalidate();self.nav.stop('切换视觉')
                self.nav.vision_enabled=False
        self.layer.publisher=NS(publish=publish)
        self.layer.tick()
        self.assertEqual(self.nav.vision_at,0.)
        self.assertIsNone(self.nav.vision_safety)
        self.assertIsNone(self.layer.last)
        self.assertEqual(self.layer.cells,{})

    def test_slow_geometry_does_not_hold_velocity_lock(self):
        entered=threading.Event();release=threading.Event();done=threading.Event()
        failures=[]
        def compute(*args,**kwargs):
            entered.set()
            if not release.wait(2.):raise RuntimeError('test release timeout')
            return depth_points(*args,**kwargs)
        def velocity():
            try:self.nav.velocity()
            except BaseException as exc:failures.append(exc)
            finally:done.set()
        worker=None;consumer=None
        with patch('nav2.vision_layer.depth_points',side_effect=compute):
            try:
                worker=threading.Thread(target=self.layer.tick);worker.start()
                self.assertTrue(entered.wait(1.))
                consumer=threading.Thread(target=velocity);consumer.start()
                self.assertTrue(done.wait(.2),'velocity waited for depth geometry')
            finally:
                release.set()
                if worker:worker.join(2.)
                if consumer:consumer.join(2.)
        self.assertEqual(failures,[])
        self.assertFalse(worker.is_alive())

    def test_near_hazard_stops_before_map_publication(self):
        observed=[]
        def compute(*args,**kwargs):return np.array([[.3,0.]])
        def publish(_):observed.append((self.nav.enabled,self.nav.velocity()))
        self.layer.publisher=NS(publish=publish)
        with patch('nav2.vision_layer.depth_points',side_effect=compute):self.layer.tick()
        self.assertEqual(observed,[(False,(0.,0.))])
        self.assertTrue(self.nav.vision_near_blocked)
        self.assertEqual(self.nav.vision_at,self.record.frame.source_at)

    def test_republished_identical_map_does_not_invalidate_fresh_evidence(self):
        grid=NS(header=NS(frame_id='map'),data=[0,100,0,0],info=NS(width=2,height=2,
            resolution=.05,origin=NS(position=NS(x=0.,y=0.),rotation=None,
                orientation=NS(x=0.,y=0.,z=0.,w=1.))))
        self.layer.receive_static(grid)
        revision=self.layer._revision
        self.nav.vision_at=123.
        self.layer.receive_static(grid)
        self.assertEqual(self.layer._revision,revision)
        self.assertEqual(self.nav.vision_at,123.)
        grid.data=[100,100,0,0]
        self.layer.receive_static(grid)
        self.assertEqual(self.layer._revision,revision+1)
        self.assertEqual(self.nav.vision_at,0.)

    def test_new_clear_evidence_waits_for_current_map_after_invalidation(self):
        self.layer.invalidate()
        during=[]
        self.layer.publisher=NS(publish=lambda msg:during.append(self.nav.vision_at))
        self.layer.tick()
        self.assertEqual(during,[0.])
        self.assertEqual(self.nav.vision_at,self.record.frame.source_at)

    def test_configuration_does_not_reuse_old_clear_frame_count(self):
        from nav2.near_obstacle import NearObstacleGuard
        guard=self.layer.near_guard=NearObstacleGuard()
        guard.blocked=True;guard.clear_count=2;guard.last_stamp=time.monotonic()-.2
        self.layer.invalidate()
        blocked,_=guard.update(np.empty((0,2)),0.,0.,time.monotonic(),time.monotonic())
        self.assertTrue(blocked)
        self.assertEqual(guard.clear_count,1)

    def test_shutdown_during_publish_rejects_late_freshness(self):
        self.layer._worker_stop=threading.Event()
        self.layer.invalidate()
        self.layer.publisher=NS(publish=lambda msg:self.layer._worker_stop.set())
        self.layer.tick()
        self.assertEqual(self.nav.vision_at,0.)
        self.assertIsNone(self.nav.vision_safety)
        self.assertIsNone(self.layer.last)

    def test_real_sensor_exception_is_not_a_transient_pause(self):
        with patch('nav2.vision_layer.depth_points',side_effect=ValueError('bad depth')):
            self.layer.tick()
        self.assertFalse(self.nav.enabled)
        self.assertEqual(self.nav.vision_at,0.)
        self.assertIn('bad depth',self.nav.vision_error)
        self.assertEqual(self.nav.velocity(),(0.,0.))


if __name__=='__main__':unittest.main()
