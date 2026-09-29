# 【R35】视觉接入延迟回归：无模型、相机、底盘或 ROS 动作。
import queue
import threading
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import numpy as np
from nav2.depth_mailbox import DepthMailbox
from nav2.semantic_obstacles import SemanticWorker
from nav2.vision_layer import VisionLayer


def record(at=100., fid=10, version=1, epoch=1):
    return NS(config_version=version, started_at=at+.1, completed_at=at+.3,
              frame=NS(source_at=at, stream_epoch=epoch, frame_id=fid,
                       image=np.zeros((60,80,3),np.uint8)))


class SemanticSubmissionTests(unittest.TestCase):
    def setUp(self):
        self.now=100.5
        clock=patch('nav2.semantic_obstacles.time.monotonic',side_effect=lambda:self.now)
        clock.start();self.addCleanup(clock.stop)
        self.w=SemanticWorker();self.w.enabled=True;self.w.attempted=True
        self.w.process=Mock();self.w.process.is_alive.return_value=True
        self.w.requests=queue.Queue(maxsize=1);self.w.responses=queue.Queue(maxsize=2)

    def complete(self):
        token,_,_=self.w.requests.get_nowait()
        self.w.responses.put_nowait(('result',token,[]))

    def test_completed_frame_is_not_submitted_again_even_after_throttle(self):
        r=record();self.w.update(r);self.complete();self.now=101.05
        with patch('nav2.semantic_obstacles.cv2.resize') as resize:
            self.assertEqual(self.w.update(r),(r,[]))
            resize.assert_not_called()
        self.assertTrue(self.w.requests.empty());self.assertIsNone(self.w.pending)

    def test_expired_future_and_nonfinite_inputs_never_enter_inference(self):
        for stamp in (98.,102.,float('nan'),float('inf')):
            with self.subTest(stamp=stamp),patch('nav2.semantic_obstacles.cv2.resize') as resize:
                self.w.update(record(at=stamp));resize.assert_not_called()
                self.assertTrue(self.w.requests.empty());self.assertIsNone(self.w.last_submitted)

    def test_old_input_still_drains_completed_response(self):
        r=record();self.w.update(r);self.complete();self.now=103.
        self.assertIsNone(self.w.update(r));self.assertIsNone(self.w.pending)
        self.assertTrue(self.w.responses.empty());self.assertTrue(self.w.requests.empty())
        fresh=record(at=103.,fid=11);self.w.update(fresh)
        # R38: drain the old response, but wait for stable depth before adding load.
        self.assertIsNone(self.w.pending)
        self.assertTrue(self.w.resource_budget['paused'])

    def test_queue_full_does_not_consume_frame_or_throttle_retry(self):
        r=record();self.w.requests.put_nowait(('other',None));self.w.update(r)
        self.assertIsNone(self.w.last_submitted);self.assertEqual(self.w.last_offer,0.)
        self.w.requests.get_nowait();self.w.update(r)
        self.assertIs(self.w.pending[1],r)

    def test_new_frame_and_config_or_stream_restart_are_allowed(self):
        for r in (record(),record(100.6,11),record(101.2,1,version=2),
                  record(101.8,1,version=2,epoch=2)):
            self.now=r.frame.source_at+.5
            self.w.update(r);self.assertIs(self.w.pending[1],r)
            self.complete();self.w.update(r)
            self.assertTrue(self.w.requests.empty())

    def test_older_frame_cannot_replace_new_submission(self):
        self.w.update(record());self.complete();self.now=101.05
        self.w.update(record(100.1,9));self.assertTrue(self.w.requests.empty())

    def test_resize_that_exhausts_freshness_does_not_submit(self):
        def resize(*args):self.now=101.21;return np.zeros((1,1,3))
        with patch('nav2.semantic_obstacles.cv2.resize',side_effect=resize):
            self.w.update(record())
        self.assertTrue(self.w.requests.empty());self.assertIsNone(self.w.last_submitted)


class MailboxWakeTests(unittest.TestCase):
    def test_notice_precedes_original_callback_and_keeps_capture_time(self):
        seen=[]
        worker=NS(on_result=lambda r:seen.append(box.updated.is_set()))
        box=DepthMailbox(worker);self.addCleanup(box.close)
        r=record()
        with patch('nav2.depth_mailbox.time.monotonic',return_value=100.4):worker.on_result(r)
        self.assertEqual(seen,[True]);self.assertEqual(box.arrival(r),(100.4,None))
        self.assertEqual(r.frame.source_at,100.)
        box.updated.clear()
        worker.on_result(r);worker.on_result(record(99.,9))
        self.assertFalse(box.updated.is_set());self.assertIs(box.latest(1,1),r)
        fresh=record(100.5,11)
        with patch('nav2.depth_mailbox.time.monotonic',return_value=101.):worker.on_result(fresh)
        self.assertTrue(box.updated.is_set());self.assertAlmostEqual(box.arrival(fresh)[1],.6)
        self.assertIsNone(box.arrival(r))

    def test_arrival_during_processing_is_not_cleared_before_wait(self):
        layer=VisionLayer.__new__(VisionLayer)
        layer.engine=NS(_stop=threading.Event());layer._worker_stop=threading.Event()
        layer.semantic=NS(close=Mock());worker=NS(on_result=Mock())
        layer._depth_mailbox=box=DepthMailbox(worker)
        # No sleeps: the wait probe verifies the event was set DURING tick and
        # passed intact to the next wait, which should return immediately.
        wait=box.updated.wait
        checks=[]
        def wait_probe(timeout):
            checks.append(box.updated.is_set())
            self.assertTrue(wait(0));self.assertLessEqual(timeout,.1)
            layer._worker_stop.set()
        box.updated.wait=wait_probe
        layer.tick=lambda:worker.on_result(record())
        layer._run()
        self.assertEqual(checks,[True]);layer.semantic.close.assert_called_once()
        self.assertIs(worker.on_result,box._original)

    def test_missing_frames_still_use_bounded_health_poll(self):
        layer=VisionLayer.__new__(VisionLayer)
        layer.engine=NS(_stop=threading.Event());layer._worker_stop=threading.Event()
        layer.semantic=NS(close=Mock());box=DepthMailbox(NS(on_result=Mock()))
        layer._depth_mailbox=box;layer.tick=Mock();timeouts=[]
        def wait(timeout):
            self.assertFalse(box.updated.is_set());timeouts.append(timeout)
            if len(timeouts)==3:layer._worker_stop.set()
        box.updated.wait=wait;layer._run()
        self.assertEqual(layer.tick.call_count,3)
        self.assertTrue(all(0<=t<=.1 for t in timeouts))

    def test_first_read_metrics_are_not_repeated_old_frame_age(self):
        r=record();worker=NS(on_result=Mock());box=DepthMailbox(worker)
        self.addCleanup(box.close)
        with patch('time.monotonic',return_value=100.4):worker.on_result(r)
        layer=VisionLayer.__new__(VisionLayer);layer._depth_mailbox=box
        layer.engine=NS(lock=threading.RLock(),_buffer_m2={},_config_version=1,_stream_epoch=1)
        layer.nav=NS(lock=threading.RLock(),vision_enabled=True,navigation_footprint=Mock())
        layer.semantic=NS(set_enabled=Mock(),offer=Mock(),snapshot=Mock(return_value=None));layer._tick_locked=Mock()
        with patch('time.monotonic',return_value=100.45):layer.tick()
        self.assertTrue(layer.timing['new_frame'])
        self.assertEqual(layer.timing['first_read_after_inference'],.15)
        self.assertEqual(layer.timing['result_delivery'],.1)
        self.assertEqual(layer.timing['mailbox_wait'],.05)
        with patch('time.monotonic',return_value=101.5):layer.tick()
        self.assertFalse(layer.timing['new_frame'])
        self.assertNotIn('first_read_after_inference',layer.timing)
        self.assertEqual(layer.timing['source_age'],1.5)
        self.assertEqual(r.frame.source_at,100.)
