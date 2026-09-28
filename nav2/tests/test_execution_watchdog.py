# 【内容标注】用途：执行动作无响应、迟到接受、取消传输失败的离线回归。
# 对应用户需求：R18、R22。不创建 ROS 节点，不发布速度。
import time
import unittest
from concurrent.futures import Future
from types import SimpleNamespace as NS
from unittest.mock import Mock
from nav2.tests import test_person_navigation as fixtures


class ExecutionWatchdogTests(unittest.TestCase):
    def setUp(self):
        case = fixtures.PersonNavigationTests(); case.setUp(); case.finish()
        self.n = case.n
        self.n.continuous_follow = True; self.n.continuous_blocked = False
        self.n.continuous_target_id = 10

    def test_acceptance_timeout_stops_and_quarantines_then_cancels_late_acceptance(self):
        n = self.n; generation = n.generation
        n.goal_at = time.monotonic() - 6.
        n._execution_timeout()
        self.assertFalse(n.enabled); self.assertTrue(n.pending)
        self.assertTrue(n.continuous_blocked)
        self.assertGreater(n.generation, generation)
        self.assertIn('5 秒未确认', n.error)
        result = Future(); handle = Mock(accepted=True)
        handle.get_result_async.return_value = result
        n.path_client.send_goal_async.return_value.set_result(handle)
        self.assertFalse(n.pending); self.assertTrue(n.canceling)
        handle.cancel_goal_async.assert_called_once()
        n.update(3., 0., 1.)
        n.path_client.send_goal_async.assert_called_once()
        result.set_result(NS(status=5))
        self.assertIsNone(n.handle); self.assertFalse(n.canceling)
        self.assertTrue(n.continuous_blocked); self.assertFalse(n.enabled)

    def test_timeout_budget_starts_after_validation_not_initial_plan(self):
        n = self.n
        self.assertLess(time.monotonic()-n.goal_at, 1.)
        n.person_navigation.started_at = time.monotonic() - 6.
        n._execution_timeout()
        self.assertTrue(n.enabled); self.assertTrue(n.pending)

    def test_missing_acceptance_response_cannot_send_another_route(self):
        n = self.n
        n.path_client.send_goal_async.return_value.set_exception(RuntimeError('transport lost'))
        self.assertFalse(n.enabled); self.assertTrue(n.pending)
        self.assertTrue(n.continuous_blocked)
        n.update(3., 0., 1.)
        n.path_client.send_goal_async.assert_called_once()

    def test_cancel_transport_exception_keeps_local_stop_and_old_handle(self):
        n = self.n; handle = Mock(accepted=True)
        handle.get_result_async.return_value = Future()
        n.path_client.send_goal_async.return_value.set_result(handle)
        handle.cancel_goal_async.side_effect = RuntimeError('cancel transport lost')
        n.stop('用户强停')
        self.assertFalse(n.enabled); self.assertEqual(n.command, (0., 0.))
        self.assertIs(n.handle, handle); self.assertTrue(n.canceling)
        self.assertIn('取消通信失败', n.error)

    def test_immediately_completed_result_does_not_leave_canceling_stuck(self):
        n = self.n; handle = Mock(accepted=True)
        handle.get_result_async.return_value = fixtures.done(NS(status=4))
        n.path_client.send_goal_async.return_value.set_result(handle)
        self.assertIsNone(n.handle); self.assertFalse(n.canceling)
        self.assertEqual(n.person_navigation.state, 'arrived')
        handle.cancel_goal_async.assert_not_called()

    def test_result_transport_error_is_not_treated_as_successful_arrival(self):
        n = self.n; result = Future(); handle = Mock(accepted=True)
        handle.get_result_async.return_value = result
        n.path_client.send_goal_async.return_value.set_result(handle)
        result.set_exception(RuntimeError('result lost'))
        self.assertFalse(n.enabled); self.assertTrue(n.continuous_blocked)
        self.assertIs(n.handle, handle); self.assertTrue(n.canceling)
        self.assertEqual(n.person_navigation.state, 'error')
        self.assertIsNone(n.person_navigation.arrived_at)
        n.update(3., 0., 1.)
        n.path_client.send_goal_async.assert_called_once()


if __name__ == '__main__':
    unittest.main()
