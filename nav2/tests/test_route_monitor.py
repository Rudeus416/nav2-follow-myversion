# 【内容标注 / R40】验证后台路线复核：不阻塞视觉更新，不重入，不积压，失效后无副作用。
"""Offline scheduler contracts, with no ROS executor, sensor or motion output."""
import threading
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from nav2.route_monitor import RouteMonitor


class RouteMonitorTests(unittest.TestCase):
    def setUp(self):
        self.task = NS(active=True, holding=False, segment=object(),
                       current_path=NS(poses=[object()]))
        self.nav = NS(lock=threading.RLock(), enabled=True, vision_enabled=True,
                      generation=1, stop=Mock(),
                      person_navigation=NS(obstacle_replan=self.task))
        self.workers = []
        self.releases = []

    def tearDown(self):
        for release in self.releases:
            release.set()
        for worker in self.workers:
            self.assertTrue(worker.close(2.))

    def worker(self, monitor, **kwargs):
        result = RouteMonitor(self.nav, monitor=monitor, **kwargs)
        self.workers.append(result)
        return result

    def blocking_monitor(self):
        entered, release, ended = (threading.Event() for _ in range(3))
        self.releases.append(release)
        def monitor(nav, valid):
            entered.set()
            self.assertTrue(release.wait(2.))
            with nav.lock:
                if valid():
                    nav.stop('accepted result')
            ended.set()
        return monitor, entered, release, ended

    def test_lazy_no_thread_for_inactive_or_disabled_navigation(self):
        monitor = Mock()
        worker = self.worker(monitor)
        self.assertFalse(worker.snapshot()['started'])
        for attr in ('enabled', 'vision_enabled'):
            setattr(self.nav, attr, False)
            self.assertFalse(worker.offer())
            setattr(self.nav, attr, True)
        self.task.active = False
        self.assertFalse(worker.offer())
        self.assertFalse(worker.snapshot()['started'])
        monitor.assert_not_called()

    def test_offer_returns_while_geometry_waits_and_coalesces_without_overlap(self):
        entered, release, twice = (threading.Event() for _ in range(3))
        self.releases.append(release)
        calls, active = [], []
        def monitor(nav, valid):
            active.append(True)
            self.assertEqual(len(active), 1)
            calls.append(threading.get_ident())
            entered.set()
            self.assertTrue(release.wait(2.))
            active.pop()
            if len(calls) == 2:
                twice.set()
        worker = self.worker(monitor)
        self.assertTrue(worker.offer())
        self.assertTrue(entered.wait(1.))
        before = time.monotonic()
        for _ in range(200):
            self.assertTrue(worker.offer())
        self.assertLess(time.monotonic() - before, .2)
        self.assertEqual(len(calls), 1)
        self.assertEqual(worker.snapshot()['coalesced'], 199)
        release.set()
        self.assertTrue(twice.wait(1.))
        time.sleep(.03)
        self.assertEqual(len(calls), 2)
        self.assertEqual(len(set(calls)), 1)
        self.nav.stop.assert_not_called()

    def test_close_immediately_revokes_slow_geometry_and_discards_pending(self):
        monitor, entered, release, ended = self.blocking_monitor()
        worker = self.worker(monitor)
        worker.offer()
        self.assertTrue(entered.wait(1.))
        worker.offer()
        before = time.monotonic()
        self.assertFalse(worker.close(.02))
        self.assertLess(time.monotonic() - before, .2)
        self.assertFalse(worker.offer())
        self.assertFalse(worker.snapshot()['pending'])
        release.set()
        self.assertTrue(ended.wait(1.))
        self.nav.stop.assert_not_called()

    def test_generation_segment_path_and_configuration_fence_late_results(self):
        for change in ('generation', 'segment', 'path', 'context', 'disable'):
            with self.subTest(change=change):
                context = [1]
                monitor, entered, release, ended = self.blocking_monitor()
                worker = self.worker(monitor, context=lambda: context[0])
                self.nav.enabled = True
                self.nav.vision_enabled = True
                worker.offer()
                self.assertTrue(entered.wait(1.))
                with self.nav.lock:
                    if change == 'generation': self.nav.generation += 1
                    elif change == 'segment': self.task.segment = object()
                    elif change == 'path': self.task.current_path = NS(poses=[object()])
                    elif change == 'context': context[0] = None
                    else: self.nav.vision_enabled = False
                release.set()
                self.assertTrue(ended.wait(1.))
                self.assertTrue(worker.close(1.))
                self.nav.stop.assert_not_called()

    def test_current_unknown_failure_stops_but_late_failure_does_not(self):
        for invalidate in (False, True):
            with self.subTest(invalidate=invalidate):
                entered, release = threading.Event(), threading.Event()
                self.releases.append(release)
                def monitor(nav, valid):
                    entered.set()
                    self.assertTrue(release.wait(2.))
                    raise RuntimeError('geometry failed')
                worker = self.worker(monitor)
                worker.offer()
                self.assertTrue(entered.wait(1.))
                if invalidate:
                    with self.nav.lock: self.nav.generation += 1
                release.set()
                deadline = time.monotonic()+1.
                while worker.snapshot()['running'] and time.monotonic() < deadline:
                    time.sleep(.005)
                self.assertFalse(worker.snapshot()['running'])
                self.assertEqual(self.nav.stop.call_count, 0 if invalidate else 1)
                self.nav.stop.reset_mock()

    def test_unknown_owner_context_rejects_work(self):
        worker = self.worker(Mock(), context=lambda: None)
        self.assertFalse(worker.offer())
        self.assertFalse(worker.snapshot()['started'])

    def test_thread_start_failure_stops_current_task_without_holding_nav_lock(self):
        worker = self.worker(Mock())
        def fail_start():
            self.assertFalse(self.nav.lock._is_owned())
            raise RuntimeError('thread unavailable')
        with patch('nav2.route_monitor.threading.Thread') as constructor:
            constructor.return_value.start.side_effect = fail_start
            self.assertFalse(worker.offer())
        self.nav.stop.assert_called_once()
        self.assertFalse(worker.snapshot()['started'])


if __name__ == '__main__':
    unittest.main()
