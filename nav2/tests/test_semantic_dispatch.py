# 【内容标注】用途：导航离线回归：语义分发线程隔离。
# 对应用户需求：R37（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：Mock 阻塞推理/状态/关闭，验证最新引用、代际和深度消费者可立即继续。
"""No model, ROS executor, camera, or chassis is started by these tests."""
import queue
import threading
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from nav2.semantic_dispatch import SemanticDispatch


def record(frame_id=1, version=1, epoch='camera-a'):
    return NS(config_version=version,
              frame=NS(frame_id=frame_id, stream_epoch=epoch,
                       source_at=time.monotonic(), image=object()))


def worker():
    return NS(enabled=True, message='ready', update=Mock(return_value=None),
              status=Mock(return_value={'enabled':True, 'message':'ready',
                                        'fresh':False, 'objects':[]}),
              close=Mock())


def wait_until(predicate, timeout=1.):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if predicate():
            return True
        time.sleep(.002)
    return False


class SemanticDispatchTests(unittest.TestCase):
    def shutdown(self, dispatch, *release):
        dispatch.close()
        for event in release:
            event.set()
        dispatch._thread.join(1.)
        self.assertFalse(dispatch._thread.is_alive())

    def test_blocked_update_does_not_block_offer_snapshot_status_or_close(self):
        raw=worker();entered=threading.Event();release=threading.Event()
        owner_threads=[]
        def update(source):
            owner_threads.append(threading.get_ident())
            entered.set();release.wait(2.)
            return source, []
        raw.update.side_effect=update
        raw.close.side_effect=lambda: owner_threads.append(threading.get_ident())
        dispatch=SemanticDispatch(raw, poll_interval=.005)
        source=record();dispatch.offer(source)
        try:
            self.assertTrue(entered.wait(1.))
            complete=threading.Event()
            def consumer():
                dispatch.offer(record(2));dispatch.snapshot();dispatch.status()
                dispatch.close();complete.set()
            consumer_thread=threading.Thread(target=consumer, daemon=True)
            consumer_thread.start()
            self.assertTrue(complete.wait(.25), 'semantic operation blocked the consumer')
            consumer_thread.join(.25)
            raw.close.assert_not_called()
            self.assertIsNone(dispatch.snapshot())
        finally:
            self.shutdown(dispatch, release)
        raw.close.assert_called_once()
        self.assertEqual(len(set(owner_threads)), 1)
        self.assertNotEqual(owner_threads[0], threading.get_ident())

    def test_only_latest_pending_reference_is_processed(self):
        raw=worker();first=threading.Event();release_first=threading.Event()
        second=threading.Event();release_second=threading.Event();seen=[]
        def update(source):
            seen.append(source)
            if len(seen)==1:
                first.set();release_first.wait(2.)
            else:
                second.set();release_second.wait(2.)
            return None
        raw.update.side_effect=update
        dispatch=SemanticDispatch(raw, poll_interval=.005)
        original=record();newest=None;dispatch.offer(original)
        try:
            self.assertTrue(first.wait(1.))
            for i in range(2, 202):
                newest=record(i);dispatch.offer(newest)
            release_first.set()
            self.assertTrue(second.wait(1.))
            self.assertEqual(len(seen), 2)
            self.assertIs(seen[0], original);self.assertIs(seen[1], newest)
            self.assertIs(seen[1].frame.image, newest.frame.image)
        finally:
            self.shutdown(dispatch, release_first, release_second)

    def test_same_frame_is_polled_until_async_result_arrives(self):
        raw=worker();source=record();objects=[{'label':'cart', 'confidence':.8}]
        raw.update.side_effect=lambda source: ((source, objects)
                                               if raw.update.call_count>=2 else None)
        raw.status.return_value={'enabled':True, 'message':'cart',
                                 'fresh':True, 'objects':objects}
        dispatch=SemanticDispatch(raw, poll_interval=.005)
        try:
            dispatch.offer(source)
            self.assertTrue(wait_until(lambda: dispatch.snapshot() is not None))
            packet=dispatch.snapshot()
            self.assertIs(packet[0], source);self.assertIs(packet[1], objects)
            self.assertGreaterEqual(raw.update.call_count, 2)
            self.assertTrue(all(call.args[0] is source for call in raw.update.call_args_list))
            self.assertTrue(dispatch.status()['fresh'])
        finally:
            self.shutdown(dispatch)

    def test_disable_reenable_discards_late_result_and_serializes_reset(self):
        raw=worker();entered=threading.Event();release=threading.Event()
        closing=threading.Event();release_close=threading.Event();new_started=threading.Event()
        old,new=record(),record(2);calls=[]
        def update(source):
            calls.append(('update', source, threading.get_ident()))
            if source is old:
                entered.set();release.wait(2.)
                return old, [{'label':'old', 'confidence':1.}]
            new_started.set()
            return new, []
        def close():
            calls.append(('close', None, threading.get_ident()))
            closing.set();release_close.wait(2.)
        raw.update.side_effect=update;raw.close.side_effect=close
        dispatch=SemanticDispatch(raw, poll_interval=.005)
        try:
            dispatch.offer(old);self.assertTrue(entered.wait(1.))
            dispatch.set_enabled(False)
            dispatch.offer(record(99))
            self.assertFalse(dispatch.status()['fresh'])
            dispatch.set_enabled(True);dispatch.offer(new)
            self.assertIsNone(dispatch.snapshot())
            release.set();self.assertTrue(closing.wait(1.))
            self.assertIsNone(dispatch.snapshot())
            self.assertFalse(new_started.is_set())
            # A slow process shutdown does not hold the public mailbox lock.
            dispatch.status();dispatch.offer(new)
            release_close.set();self.assertTrue(new_started.wait(1.))
            self.assertTrue(wait_until(lambda: dispatch.snapshot() is not None))
            self.assertIs(dispatch.snapshot()[0], new)
            self.assertEqual([kind for kind, _, _ in calls[:3]], ['update','close','update'])
            self.assertEqual(len({thread_id for _, _, thread_id in calls}), 1)
        finally:
            self.shutdown(dispatch, release, release_close)

    def test_wrong_configuration_or_stream_result_is_not_published(self):
        for newest in (record(2, version=2), record(2, epoch='camera-b')):
            with self.subTest(version=newest.config_version, epoch=newest.frame.stream_epoch):
                raw=worker();old=record();entered=threading.Event();release=threading.Event()
                second=threading.Event();release_second=threading.Event()
                def update(source):
                    if source is old:
                        entered.set();release.wait(2.)
                        return old, []
                    second.set();release_second.wait(2.)
                    return newest, []
                raw.update.side_effect=update
                dispatch=SemanticDispatch(raw, poll_interval=.005)
                try:
                    dispatch.offer(old);self.assertTrue(entered.wait(1.))
                    dispatch.offer(newest);release.set()
                    self.assertTrue(second.wait(1.))
                    self.assertIsNone(dispatch.snapshot())
                    self.assertFalse(dispatch.status()['fresh'])
                finally:
                    self.shutdown(dispatch, release, release_second)

    def test_worker_failure_is_cached_without_killing_owner(self):
        raw=worker();raw.update.side_effect=RuntimeError('synthetic resize failure')
        dispatch=SemanticDispatch(raw, poll_interval=.005)
        try:
            dispatch.offer(record())
            self.assertTrue(wait_until(lambda: 'synthetic resize failure' in dispatch.status()['message']))
            self.assertIsNone(dispatch.snapshot())
            self.assertTrue(dispatch._thread.is_alive())
        finally:
            self.shutdown(dispatch)

    def test_cached_status_never_calls_blocked_worker_status(self):
        raw=worker();entered=threading.Event();release=threading.Event()
        def status():
            entered.set();release.wait(2.)
            return {'enabled':True, 'message':'ready', 'fresh':False, 'objects':[]}
        raw.status.side_effect=status
        dispatch=SemanticDispatch(raw, poll_interval=.005)
        try:
            dispatch.offer(record());self.assertTrue(entered.wait(1.))
            read=threading.Event()
            reader=threading.Thread(target=lambda: (dispatch.status(), read.set()), daemon=True)
            reader.start();self.assertTrue(read.wait(.25));reader.join(.25)
            self.assertEqual(raw.status.call_count, 1)
        finally:
            self.shutdown(dispatch, release)

    def test_expired_and_future_packets_are_not_fresh(self):
        raw=worker();source=record()
        raw.update.side_effect=lambda source: (source, [])
        dispatch=SemanticDispatch(raw, poll_interval=.005)
        try:
            dispatch.offer(source)
            self.assertTrue(wait_until(lambda: dispatch.snapshot() is not None))
            for source_at in (time.monotonic()-2., time.monotonic()+10., float('nan')):
                source.frame.source_at=source_at
                self.assertIsNone(dispatch.snapshot())
                self.assertFalse(dispatch.status()['fresh'])
                self.assertEqual(dispatch.status()['objects'], [])
        finally:
            self.shutdown(dispatch)

    def test_idle_owner_does_not_start_model_and_disabled_offer_is_ignored(self):
        raw=worker();dispatch=SemanticDispatch(raw, poll_interval=.005)
        try:
            dispatch.set_enabled(False)
            dispatch.offer(record())
            self.assertTrue(wait_until(lambda: raw.close.call_count>=1))
            raw.update.assert_not_called()
            self.assertIsNone(dispatch.snapshot())
            self.assertFalse(dispatch.status()['enabled'])
        finally:
            self.shutdown(dispatch)

    def test_environment_disabled_model_stays_disabled_after_runtime_toggle(self):
        raw=worker();raw.enabled=False;raw.message='YOLOE 已关闭'
        dispatch=SemanticDispatch(raw, poll_interval=.005)
        try:
            dispatch.offer(record())
            dispatch.set_enabled(False);dispatch.set_enabled(True)
            dispatch.offer(record(2))
            self.assertTrue(wait_until(lambda: raw.close.call_count>=1))
            raw.update.assert_not_called();raw.status.assert_not_called()
            self.assertFalse(dispatch.status()['enabled'])
            self.assertEqual(dispatch.status()['message'], 'YOLOE 已关闭')
        finally:
            self.shutdown(dispatch)

    def test_status_failure_discards_result_but_keeps_owner_alive(self):
        raw=worker();raw.update.side_effect=lambda source: (source, [])
        raw.status.side_effect=RuntimeError('synthetic status failure')
        dispatch=SemanticDispatch(raw, poll_interval=.005)
        try:
            dispatch.offer(record())
            self.assertTrue(wait_until(lambda: 'synthetic status failure' in dispatch.status()['message']))
            self.assertIsNone(dispatch.snapshot())
            self.assertFalse(dispatch.status()['fresh'])
            self.assertTrue(dispatch._thread.is_alive())
        finally:
            self.shutdown(dispatch)

    def test_failed_reset_is_retried_before_processing_new_generation(self):
        raw=worker();closing=threading.Event();release=threading.Event()
        def close():
            if raw.close.call_count==1:
                raise RuntimeError('synthetic close failure')
            closing.set();release.wait(2.)
        raw.close.side_effect=close
        dispatch=SemanticDispatch(raw, poll_interval=.005)
        try:
            dispatch.set_enabled(False);dispatch.set_enabled(True)
            new=record(2);dispatch.offer(new)
            self.assertTrue(closing.wait(1.))
            raw.update.assert_not_called()
            self.assertIsNone(dispatch.snapshot())
            self.assertIn('synthetic close failure', dispatch.status()['message'])
            release.set()
            self.assertTrue(wait_until(lambda: raw.update.call_count>=1))
            self.assertIs(raw.update.call_args.args[0], new)
        finally:
            self.shutdown(dispatch, release)

    def test_real_worker_polls_same_frame_without_duplicate_inference_submission(self):
        from nav2.semantic_obstacles import SemanticWorker
        raw=SemanticWorker();raw.enabled=True;raw.attempted=True
        raw.process=Mock();raw.process.is_alive.return_value=True
        raw.requests=queue.Queue(maxsize=1);raw.responses=queue.Queue()
        raw.close=Mock();raw.update=Mock(wraps=raw.update)
        source=record();source.frame.image=NS(shape=(60, 80, 3))
        with patch('nav2.semantic_obstacles.cv2.resize', return_value=object()) as resize:
            dispatch=SemanticDispatch(raw, poll_interval=.005)
            try:
                dispatch.offer(source)
                self.assertTrue(wait_until(lambda: raw.requests.qsize()==1))
                token, _, _=raw.requests.get_nowait()
                raw.responses.put(('result', token, []))
                self.assertTrue(wait_until(lambda: dispatch.snapshot() is not None))
                self.assertTrue(wait_until(lambda: raw.update.call_count>=4))
                self.assertIs(dispatch.snapshot()[0], source)
                self.assertEqual(token, (source.config_version, source.frame.stream_epoch,
                                         source.frame.frame_id))
                self.assertTrue(raw.requests.empty())
                resize.assert_called_once()
            finally:
                self.shutdown(dispatch)
