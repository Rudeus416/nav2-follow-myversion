# 【内容标注】用途：导航离线回归：vision_scheduler。
# 对应用户需求：R22（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明。
# 本次仅加注释；需求关联不是精确创建/提交记录。
"""Independent vision updates: no ROS executor, camera or chassis is started."""
import threading
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock
from nav2.vision_layer import VisionLayer


class VisionSchedulerTests(unittest.TestCase):
    def layer(self, tick):
        layer=VisionLayer.__new__(VisionLayer)
        layer.engine=NS(_stop=threading.Event())
        layer.nav=NS(lock=threading.RLock(),vision_at=123.,vision_error='',stop=Mock())
        layer.semantic=NS(close=Mock())
        layer._worker_stop=threading.Event()
        layer.tick=tick
        layer._worker=threading.Thread(target=layer._run,daemon=True)
        return layer

    def test_updates_without_any_ros_executor_spinning(self):
        updates=[];ready=threading.Event()
        def tick():
            updates.append(time.monotonic())
            if len(updates)>=3:ready.set()
        layer=self.layer(tick)
        layer._worker.start()
        try:self.assertTrue(ready.wait(2.))
        finally:layer.close()
        self.assertFalse(layer._worker.is_alive())
        layer.semantic.close.assert_called_once()
        count=len(updates);time.sleep(.15)
        self.assertEqual(len(updates),count)

    def test_slow_tick_never_overlaps_or_queues_parallel_work(self):
        active=[];threads=[];done=threading.Event()
        def tick():
            active.append(True);self.assertEqual(len(active),1)
            threads.append(threading.get_ident());time.sleep(.15);active.pop()
            if len(threads)==2:done.set()
        layer=self.layer(tick);layer._worker.start()
        try:self.assertTrue(done.wait(2.))
        finally:layer.close()
        self.assertEqual(len(set(threads)),1)

    def test_unexpected_exception_invalidates_vision_and_stops(self):
        layer=self.layer(None)
        def fail():
            layer._worker_stop.set()
            raise RuntimeError('read failed')
        layer.tick=fail
        with self.assertLogs('nav2.vision_layer',level='ERROR'):
            layer._worker.start();layer._worker.join(2.)
        self.assertFalse(layer._worker.is_alive())
        self.assertEqual(layer.nav.vision_at,0.)
        self.assertIn('read failed',layer.nav.vision_error)
        layer.nav.stop.assert_called_once()
        layer.semantic.close.assert_called_once()

    def test_engine_shutdown_exits_without_reading_destroyed_engine(self):
        tick=Mock();layer=self.layer(tick);layer.engine._stop.set()
        layer._worker.start();layer._worker.join(2.)
        self.assertFalse(layer._worker.is_alive())
        tick.assert_not_called();layer.semantic.close.assert_called_once()

    def test_old_depth_frame_cannot_refresh_navigation_freshness(self):
        layer=self.layer(Mock())
        layer.nav.vision_enabled=True;layer.nav.vision_error=''
        old=time.monotonic()-2.;layer.nav.vision_at=old
        layer.nav.pause_for_vision=Mock()
        record=NS(config_version=1,frame=NS(source_at=old,stream_epoch=1,frame_id=10))
        layer._tick_locked([record],1)
        self.assertEqual(layer.nav.vision_at,old)
        layer.nav.pause_for_vision.assert_called_once()

    def test_repeated_frame_does_not_refresh_timestamp(self):
        layer=self.layer(Mock())
        layer.nav.vision_enabled=True
        stamp=time.monotonic();layer.nav.vision_at=stamp
        layer.last=(1,1,10,False)
        record=NS(config_version=1,frame=NS(source_at=stamp,stream_epoch=1,frame_id=10))
        layer._tick_locked([record],1)
        self.assertEqual(layer.nav.vision_at,stamp)
