# 【内容标注 / R38】预算只暂停附加提交；已加载模型保留、在途响应继续回收。
import queue
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
import numpy as np
from nav2.semantic_obstacles import SemanticWorker


def record(fid,now,age=.2):
    return NS(config_version=1,completed_at=now,
              frame=NS(source_at=now-age,stream_epoch=1,frame_id=fid,
                       image=np.zeros((60,80,3),np.uint8)))


class SemanticBudgetWorkerTests(unittest.TestCase):
    def setUp(self):
        self.now=100.
        clock=patch('nav2.semantic_obstacles.time.monotonic',side_effect=lambda:self.now)
        clock.start();self.addCleanup(clock.stop)
        self.w=SemanticWorker();self.w.enabled=True;self.w.attempted=True
        self.w.process=Mock();self.w.process.is_alive.return_value=True
        self.w.requests=queue.Queue(maxsize=1);self.w.responses=queue.Queue(maxsize=2)

    def test_pressure_does_not_unload_or_start_another_process(self):
        old_process=self.w.process
        self.w.update(record(1,self.now,1.))
        self.assertTrue(self.w.resource_budget['paused'])
        self.assertTrue(self.w.requests.empty())
        self.assertIs(self.w.process,old_process)
        self.w.process.terminate.assert_not_called()
        self.w.process.join.assert_not_called()
        self.assertIn('深度避障继续',self.w.status()['message'])

    def test_paused_budget_keeps_child_exit_diagnostic(self):
        self.w.process.is_alive.return_value=False
        self.w.update(record(1,self.now,1.))
        status=self.w.status()
        self.assertTrue(status['resource_budget']['paused'])
        self.assertIn('子进程退出',status['message'])
        self.assertIn('深度避障继续',status['message'])

    def test_model_does_not_start_for_already_expired_depth(self):
        self.w.process=None;self.w.attempted=False
        with patch('nav2.semantic_obstacles.Path.is_file',return_value=True),\
             patch('nav2.semantic_obstacles.mp.get_context') as context:
            self.w.update(record(1,self.now,2.769))
        context.assert_not_called()
        self.assertFalse(self.w.attempted)

    def test_inflight_response_drains_while_budget_paused(self):
        r=record(1,self.now);self.w.update(r)
        token,_,source=self.w.requests.get_nowait()
        self.assertEqual(source,r.frame.source_at)
        self.w.responses.put(('result',token,[]))
        self.now+=1.
        self.assertIsNone(self.w.update(r))
        self.assertTrue(self.w.resource_budget['paused'])
        self.assertIsNone(self.w.pending)
        self.assertTrue(self.w.responses.empty())
        self.assertTrue(self.w.requests.empty())

    def test_recovery_submits_only_latest_frame_without_reloading(self):
        process=self.w.process
        self.w.update(record(1,self.now,1.))
        for i in range(1,15):
            self.now=100+i*.2;self.w.update(record(i+1,self.now))
            self.assertTrue(self.w.requests.empty())
        self.now=103.;latest=record(16,self.now);self.w.update(latest)
        token,_,source=self.w.requests.get_nowait()
        self.assertEqual(token,(1,1,16));self.assertEqual(source,latest.frame.source_at)
        self.assertIs(self.w.process,process)
        self.w.process.terminate.assert_not_called()

    def test_expired_child_response_acknowledges_without_becoming_detection(self):
        r=record(1,self.now);self.w.update(r)
        token,_,_=self.w.requests.get_nowait()
        self.w.responses.put(('dropped',token,'语义请求源帧已过期'))
        self.assertIsNone(self.w.update(r))
        self.assertIsNone(self.w.pending);self.assertIsNone(self.w.latest)
        self.assertTrue(self.w.requests.empty())
        self.now+=.6;fresh=record(2,self.now);self.w.update(fresh)
        self.assertIs(self.w.pending[1],fresh)

    def test_unrelated_dropped_reply_cannot_release_current_pending(self):
        r=record(1,self.now);self.w.update(r)
        pending=self.w.pending
        self.w.responses.put(('dropped',(9,9,9),'old reply'))
        self.w.update(r)
        self.assertIs(self.w.pending,pending)


if __name__=='__main__':unittest.main()
