# 【内容标注 / R40】用户要求“优化”：将整段路线复核移出视觉更新线程。
# 修改逻辑：一个后台线程、最多一个待处理通知；不发布速度，不延长传感器有效期。
# 原近距检查与每次速度输出的整车扫掠检查保持同步，旧任务复核不得影响新任务。
"""Coalesced full-route checks, independent of production of visual evidence.

``monitor(nav, valid=...)`` must test ``valid()`` under ``nav.lock`` before
pausing/stopping and after geometry, before committing any result.  A context
provider runs under that same lock and returns an immutable equality-comparable
owner/configuration token, or None while that owner cannot accept work.
"""
from dataclasses import dataclass
import logging
import threading
import time

from nav2.replan_support import recovery


# 【职责 / R40】_Ticket：冻结一次路线复核的任务、分段、路径、代次和视觉上下文身份。
@dataclass(frozen=True)
class _Ticket:
    task: object
    segment: object
    path: object
    generation: object
    context: object


# 【职责 / R40】RouteMonitor：单后台线程合并复核通知，慢几何不阻塞视觉提交。
class RouteMonitor:
    """One lazy worker with one replaceable pending notification, never a queue."""

    # 【职责 / R40】__init__：初始化可替换待处理通知、关闭代次和小型诊断计数。
    def __init__(self, nav, *, context=None, monitor=None):
        self.nav = nav
        self._context = context or (lambda: True)
        self._monitor = monitor
        self._closed = threading.Event()
        self._wake = threading.Event()
        self._lock = threading.Lock()
        self._thread = None
        self._pending = None
        self._running = False
        self._stats = dict(offers=0, coalesced=0, checks=0, discarded=0,
                           failures=0, last_seconds=0., max_seconds=0., last_error='')

    # 【职责 / R40】_ticket：在导航锁内捕获当前授权路线身份，本身不授权运动。
    def _ticket(self):
        """Caller holds nav.lock; a notification itself never authorizes motion."""
        nav = self.nav
        task = recovery(nav)
        if (self._closed.is_set() or task is None or not task.active or task.holding
                or not getattr(nav, 'enabled', False)
                or not getattr(nav, 'vision_enabled', False)):
            return None
        path = task.current_path
        if path is None or not path.poses:
            return None
        context = self._context()
        if context is None:
            return None
        return _Ticket(task, getattr(task, 'segment', None), path,
                       nav.generation, context)

    # 【职责 / R40】_current：复核任务、段、路径、代次和上下文仍完全一致。
    def _current(self, ticket):
        """Called under nav.lock: fence both geometry and late worker failures."""
        if self._closed.is_set():
            return False
        nav, task = self.nav, ticket.task
        if (task is not recovery(nav) or not task.active or task.holding
                or getattr(task, 'segment', None) is not ticket.segment
                or task.current_path is not ticket.path
                or nav.generation != ticket.generation
                or not getattr(nav, 'enabled', False)
                or not getattr(nav, 'vision_enabled', False)):
            return False
        context = self._context()
        return context is not None and context == ticket.context

    # 【职责 / R40】offer：仅保留最新复核请求并立即返回，空闲时才启动一个线程。
    def offer(self):
        """Schedule latest state and return immediately; start no idle threads."""
        with self.nav.lock:
            ticket = self._ticket()
        if ticket is None:
            return False
        start_error = None
        # Starting a thread is deliberately outside nav.lock: the speed output
        # must not wait for operating-system thread creation.
        with self._lock:
            if self._closed.is_set():
                return False
            self._stats['offers'] += 1
            if self._pending is not None:
                self._stats['coalesced'] += 1
            self._pending = ticket
            self._wake.set()
            if self._thread is None:
                self._thread = threading.Thread(target=self._run,
                    name='nav2-route-monitor', daemon=True)
                try:
                    self._thread.start()
                except Exception as exc:
                    self._thread = None
                    self._pending = None
                    self._wake.clear()
                    start_error = exc
        if start_error is not None:
            self._fail(ticket, start_error)
            return False
        return True

    # 【职责 / R40】_fail：未知复核异常只停止仍属于该票据的原任务。
    def _fail(self, ticket, exc):
        """Unknown check failure stops only the still-authorized original task."""
        reason = '整段路线复核异常，保持停车：' + str(exc)
        with self.nav.lock:
            if not self._current(ticket):
                return False
            self.nav.stop(reason)
        with self._lock:
            self._stats['failures'] += 1
            self._stats['last_error'] = reason
        logging.getLogger(__name__).error(reason)
        return True

    # 【职责 / R40】_run：串行执行最新路线复核并统计耗时，迟到结果由票据隔离。
    def _run(self):
        while not self._closed.is_set():
            self._wake.wait()
            with self._lock:
                self._wake.clear()
                ticket, self._pending = self._pending, None
                if self._closed.is_set():
                    return
                if ticket is None:
                    continue
                self._running = True
            started = time.monotonic()
            try:
                with self.nav.lock:
                    current = self._current(ticket)
                if not current:
                    with self._lock:
                        self._stats['discarded'] += 1
                    continue
                monitor = self._monitor
                if monitor is None:
                    from nav2.replan_support import monitor_route
                    monitor = monitor_route
                monitor(self.nav, valid=lambda: self._current(ticket))
                with self._lock:
                    self._stats['checks'] += 1
            except Exception as exc:
                # Configuration/ownership/generation checks precede any stop.
                # Work still unwinding after close cannot stop a later journey.
                try:
                    self._fail(ticket, exc)
                except Exception:
                    logging.getLogger(__name__).exception('路线复核失败处理异常')
            finally:
                elapsed = time.monotonic() - started
                with self._lock:
                    self._running = False
                    self._stats['last_seconds'] = elapsed
                    self._stats['max_seconds'] = max(self._stats['max_seconds'], elapsed)

    # 【职责 / R40】snapshot：只返回计数和耗时，不复制点云或路线。
    def snapshot(self):
        """Small diagnostics only; never expose or copy a cloud or route."""
        with self._lock:
            return dict(self._stats, running=self._running,
                        pending=self._pending is not None,
                        closed=self._closed.is_set(),
                        started=self._thread is not None)

    # 【职责 / R40】close：立即撤销结果效力，再有界等待自有线程退出。
    def close(self, timeout=.5):
        """Revoke effects immediately, then wait only a bounded time for geometry."""
        self._closed.set()
        with self._lock:
            self._pending = None
            thread = self._thread
            self._wake.set()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=max(0., float(timeout)))
        return thread is None or not thread.is_alive()
