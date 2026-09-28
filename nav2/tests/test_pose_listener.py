# 【R33】TF 调度隔离/生命周期模拟；不创建 ROS 网络节点或运动发布者。
import queue
import threading
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
from nav2.pose_listener import PoseListener


class Executor:
    def __init__(self, context):
        self.context=context; self.nodes=[]; self.queue=queue.Queue(); self.shutdown_count=0
    def add_node(self,node): self.nodes.append(node)
    def remove_node(self,node): self.nodes.remove(node)
    def shutdown(self,timeout_sec): self.shutdown_count+=1
    def spin_once(self,timeout_sec):
        try: action=self.queue.get(timeout=min(timeout_sec,.02))
        except queue.Empty: return
        action()


class PoseListenerTests(unittest.TestCase):
    def setUp(self):
        self.source=Mock();self.source.get_namespace.return_value='/robot'
        self.source.get_parameter.return_value=NS(value=True)
        self.node=Mock();self.listener=Mock();self.buffer=object()
        self.executor=Executor(self.source.context)
        self.patches=[patch('rclpy.node.Node',return_value=self.node),
                      patch('rclpy.executors.SingleThreadedExecutor',return_value=self.executor),
                      patch('tf2_ros.TransformListener',return_value=self.listener)]
        self.mocks=[p.start() for p in self.patches]
        for p in self.patches:self.addCleanup(p.stop)
        self.helper=None
    def create(self):
        self.helper=PoseListener(self.source,self.buffer)
        self.addCleanup(self.helper.close)
        return self.helper

    def test_private_node_context_clock_and_executor_never_move_source_node(self):
        h=self.create();kwargs=self.mocks[0].call_args.kwargs
        self.assertIs(kwargs['context'],self.source.context)
        self.assertEqual(kwargs['namespace'],'/robot')
        self.assertTrue(kwargs['parameter_overrides'][0].value)
        self.assertFalse(kwargs['use_global_arguments'])
        self.assertEqual(self.executor.nodes,[self.node])
        self.mocks[2].assert_called_once_with(self.buffer,self.node,spin_thread=False)
        self.assertTrue(h.close());self.assertTrue(h.close())
        self.node.destroy_node.assert_called_once();self.listener.unregister.assert_called_once()
        self.source.destroy_node.assert_not_called();self.source.executor.add_node.assert_not_called()

    def test_control_lock_block_cannot_block_listener_callbacks(self):
        h=self.create();navigation_lock=threading.Lock();received=threading.Event()
        with navigation_lock:
            # The production helper neither takes this lock nor runs on the
            # blocked control executor; a pending TF callback must complete.
            self.executor.queue.put(received.set)
            self.assertTrue(received.wait(1.))
        self.assertTrue(h.close())

    def test_context_shutdown_callback_never_joins_under_context_lock(self):
        h=self.create();callback=self.source.context.on_shutdown.call_args.args[0]
        started=time.monotonic();callback()
        self.assertLess(time.monotonic()-started,.1)
        self.assertTrue(h._closed.wait(1.))
        self.assertTrue(h.close());self.node.destroy_node.assert_called_once()

    def test_close_from_listener_thread_defers_disposal_until_callback_returns(self):
        h=self.create();done=threading.Event();inside=[]
        def callback():
            inside.append(h.close());inside.append(self.node.destroy_node.call_count);done.set()
        self.executor.queue.put(callback)
        self.assertTrue(done.wait(1.));self.assertTrue(h._closed.wait(1.))
        self.assertEqual(inside,[False,0]);self.node.destroy_node.assert_called_once()

    def test_listener_construction_failure_releases_private_node(self):
        self.mocks[2].side_effect=RuntimeError('subscribe failed')
        with self.assertRaisesRegex(RuntimeError,'subscribe failed'):self.create()
        self.node.destroy_node.assert_called_once();self.source.destroy_node.assert_not_called()
        self.assertEqual(self.executor.shutdown_count,1)

    def test_thread_start_failure_cleans_resources(self):
        thread=Mock();thread.start.side_effect=RuntimeError('thread failed');thread.is_alive.return_value=False
        with patch('nav2.pose_listener.threading.Thread',return_value=thread):
            with self.assertRaisesRegex(RuntimeError,'thread failed'):self.create()
        self.node.destroy_node.assert_called_once();self.listener.unregister.assert_called_once()

    def test_spin_failure_is_reported_and_does_not_leave_stale_listener_running(self):
        h=self.create()
        def broken():raise RuntimeError('spin failed')
        with self.assertLogs('nav2.pose_listener',level='ERROR'):
            self.executor.queue.put(broken)
            self.assertTrue(h._closed.wait(1.))
        self.assertIn('spin failed',h.error)
        self.assertTrue(h.close());self.node.destroy_node.assert_called_once()


if __name__=='__main__':unittest.main()
