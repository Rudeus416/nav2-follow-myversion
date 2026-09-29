# 【内容标注】用途：语义进程部分初始化与串行关闭的离线回归。
# 对应用户需求：R37（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：Mock 进程/队列失败与退出超时；不启动模型、ROS 或底盘。
import unittest
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from nav2.semantic_obstacles import SemanticWorker


class SemanticCleanupTests(unittest.TestCase):
    def worker(self, process=None):
        raw=SemanticWorker();raw.process=process;raw.attempted=True
        raw.requests=Mock();raw.responses=Mock()
        raw.pending=object();raw.latest=object();raw.last_submitted=object()
        raw.last_offer=10.
        return raw

    def assert_reset(self, raw):
        self.assertIsNone(raw.process);self.assertIsNone(raw.pending)
        self.assertIsNone(raw.latest);self.assertIsNone(raw.last_submitted)
        self.assertEqual(raw.last_offer, 0.);self.assertFalse(raw.attempted)

    def test_start_failure_does_not_join_unstarted_process(self):
        process=Mock(pid=None);process.is_alive.return_value=False
        process.start.side_effect=OSError('synthetic spawn failure')
        requests,responses=Mock(),Mock();ctx=Mock()
        ctx.Queue.side_effect=[requests,responses];ctx.Process.return_value=process
        raw=SemanticWorker();raw.enabled=True
        with patch('nav2.semantic_obstacles.Path.is_file', return_value=True), \
             patch('nav2.semantic_obstacles.mp.get_context', return_value=ctx):
            with self.assertRaisesRegex(OSError, 'synthetic spawn failure'):
                raw.update(NS(config_version=1,frame=NS(source_at=time.monotonic(),stream_epoch=1,frame_id=1)))
            raw.close()
        process.join.assert_not_called();process.terminate.assert_not_called()
        process.close.assert_called_once()
        for channel in (requests,responses):
            channel.cancel_join_thread.assert_called_once();channel.close.assert_called_once()
        self.assert_reset(raw)

    def test_partial_queue_construction_cleans_existing_queue(self):
        requests=Mock();ctx=Mock()
        ctx.Queue.side_effect=[requests,OSError('synthetic queue failure')]
        raw=SemanticWorker();raw.enabled=True
        with patch('nav2.semantic_obstacles.Path.is_file', return_value=True), \
             patch('nav2.semantic_obstacles.mp.get_context', return_value=ctx):
            with self.assertRaisesRegex(OSError, 'synthetic queue failure'):
                raw.update(NS(config_version=1,frame=NS(source_at=time.monotonic(),stream_epoch=1,frame_id=1)))
            raw.close()
        ctx.Process.assert_not_called()
        requests.cancel_join_thread.assert_called_once();requests.close.assert_called_once()
        self.assert_reset(raw)

    def test_running_process_is_terminated_and_joined_before_queue_cleanup(self):
        process=Mock(pid=123);process.is_alive.side_effect=[True,False]
        raw=self.worker(process);requests,responses=raw.requests,raw.responses
        calls=Mock();calls.attach_mock(process,'process')
        calls.attach_mock(requests,'requests');calls.attach_mock(responses,'responses')
        raw.close()
        process.terminate.assert_called_once();process.join.assert_called_once_with(timeout=2.)
        process.close.assert_called_once()
        names=[call[0] for call in calls.mock_calls]
        self.assertLess(names.index('process.join'), names.index('requests.close'))
        self.assertLess(names.index('process.join'), names.index('responses.close'))
        self.assert_reset(raw)

    def test_dead_started_process_is_reaped_without_terminate(self):
        process=Mock(pid=123);process.is_alive.return_value=False
        raw=self.worker(process);raw.close()
        process.terminate.assert_not_called();process.join.assert_called_once_with(timeout=2.)
        process.close.assert_called_once();self.assert_reset(raw)

    def test_still_live_process_and_queues_stay_owned_until_later_cleanup(self):
        process=Mock(pid=123);process.is_alive.return_value=True
        raw=self.worker(process);requests,responses=raw.requests,raw.responses
        with self.assertRaisesRegex(RuntimeError, '仍在退出'):
            raw.close()
        self.assertIs(raw.process, process);self.assertTrue(raw.attempted)
        self.assertIs(raw.requests, requests);self.assertIs(raw.responses, responses)
        process.close.assert_not_called();requests.close.assert_not_called()
        responses.close.assert_not_called()
        self.assertIsNone(raw.pending);self.assertIsNone(raw.latest)
        process.is_alive.return_value=False
        raw.close();self.assert_reset(raw)
        requests.close.assert_called_once();responses.close.assert_called_once()

    def test_termination_failure_preserves_process_and_queue_ownership(self):
        process=Mock(pid=123);process.is_alive.return_value=True
        process.terminate.side_effect=OSError('synthetic termination failure')
        raw=self.worker(process);requests=raw.requests
        with self.assertRaisesRegex(OSError, 'synthetic termination failure'):
            raw.close()
        self.assertIs(raw.process, process);self.assertTrue(raw.attempted)
        self.assertIs(raw.requests, requests);requests.close.assert_not_called()
        process.close.assert_not_called()

    def test_failed_queue_close_does_not_skip_other_queue_and_can_be_retried(self):
        raw=self.worker();requests,responses=raw.requests,raw.responses
        requests.close.side_effect=[OSError('synthetic queue close failure'),None]
        with self.assertRaisesRegex(RuntimeError, 'synthetic queue close failure'):
            raw.close()
        self.assertIs(raw.requests, requests);self.assertIsNone(raw.responses)
        self.assertTrue(raw.attempted);responses.close.assert_called_once()
        raw.close();self.assert_reset(raw)
        self.assertEqual(requests.close.call_count,2);responses.close.assert_called_once()

    def test_cancel_join_failure_still_closes_both_queues(self):
        raw=self.worker();requests,responses=raw.requests,raw.responses
        requests.cancel_join_thread.side_effect=OSError('synthetic cancel failure')
        with self.assertRaisesRegex(RuntimeError, 'synthetic cancel failure'):
            raw.close()
        requests.close.assert_called_once();responses.close.assert_called_once()
        raw.close();self.assert_reset(raw)

    def test_repeated_close_is_safe_after_successful_cleanup(self):
        process=Mock(pid=None);process.is_alive.return_value=False
        raw=self.worker(process);requests,responses=raw.requests,raw.responses
        raw.close();raw.close();self.assert_reset(raw)
        process.close.assert_called_once();requests.close.assert_called_once()
        responses.close.assert_called_once()
