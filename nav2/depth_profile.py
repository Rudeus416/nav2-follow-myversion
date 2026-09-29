# 【内容标注 / R39】用户确认“降到 512，优先实时性”：只降低 Nav2 视觉启用时的新深度任务输入上限。
# 本适配器不改旧代码文件、相机帧、人物模型、帧率、配置代次或安全时效；关闭后恢复原 offer。
"""Navigation-owned depth input profile, installed on one worker instance.

Only future offers are adapted. Jobs already queued or being inferred retain
what they captured; toggling/closing never drains the original worker's queue.
The input cap reduces network work, not the model's full-image output transfer.
Changing input resolution can change depth detail and requires scene validation.
"""
from dataclasses import replace
import functools
import logging
from numbers import Integral
import threading


class DepthProfile:
    """Cap immutable depth jobs without changing shared engine settings.

``enabled`` must be a cheap callback. It is called outside our private lock;
no navigation/engine lock is acquired here. The original offer is also always
called outside our lock, once, preserving its queue, return and error behavior.
``active`` in status describes the most recent offer, not a model readiness
signal. ``offered``/``adjusted`` count attempts, including a rejected offer.
    """

    @staticmethod
    def validate_limit(limit):
        """Validate before installing any navigation lifecycle hooks."""
        if isinstance(limit, bool) or not isinstance(limit, Integral):
            raise ValueError('Nav2 深度输入上限必须是 320 至 1280 之间的 32 倍数整数')
        limit = int(limit)
        if not 320 <= limit <= 1280 or limit % 32:
            raise ValueError('Nav2 深度输入上限必须是 320 至 1280 之间的 32 倍数整数')
        return limit

    def __init__(self, depth_worker, enabled, limit=512):
        limit = self.validate_limit(limit)
        if not callable(enabled):
            raise TypeError('enabled 必须是返回视觉启用状态的函数')
        self._lock = threading.Lock()
        self._depth = depth_worker
        self._enabled = enabled
        self.limit = limit
        self._closed = False
        self._original = getattr(depth_worker, 'offer', None)
        self._wrapper = None
        self._active = False
        self._requested = None
        self._effective = None
        self._offered = 0
        self._adjusted = 0
        self._error = ''
        self._last_notice = None
        if callable(self._original):
            @functools.wraps(self._original)
            def offer(job):
                return self._offer(job)
            self._wrapper = offer
            depth_worker.offer = offer

    def _offer(self, job):
        # A wrapper captured by an earlier caller stays safe after close.
        with self._lock:
            closed = self._closed
        if closed:
            return self._original(job)
        enabled = False
        error = ''
        try:
            enabled = bool(self._enabled())
        except Exception as exc:
            # This is only a workload optimization. Failure preserves the
            # original job and its existing freshness/stop protection.
            error = '读取 Nav2 深度档位启用状态失败：' + str(exc)
        with self._lock:
            closed = self._closed
        if closed:
            return self._original(job)
        requested = getattr(getattr(job, 'settings', None), 'depth_imgsz', None)
        effective = requested
        offered_job = job
        active = enabled
        try:
            if enabled and requested > self.limit:
                params = getattr(type(job), '__dataclass_params__', None)
                if params is None or not params.frozen:
                    raise TypeError('深度任务必须是不可变 dataclass')
                settings = job.settings.model_copy(update={'depth_imgsz': self.limit})
                offered_job = replace(job, settings=settings)
                effective = self.limit
        except Exception as exc:
            # Unsupported/legacy job types still get exactly their old offer;
            # do not claim the requested cap was applied.
            offered_job = job
            effective = requested
            active = False
            error = 'Nav2 深度档位未应用：' + str(exc)
        notice = None
        with self._lock:
            # If close won while copying settings, do not submit a new profile.
            if self._closed:
                offered_job = job
            else:
                self._active = active
                self._requested = requested
                self._effective = effective
                self._offered += 1
                self._adjusted += offered_job is not job
                self._error = error
                signature = (active, requested, effective, error)
                if signature != self._last_notice:
                    self._last_notice = signature
                    notice = (requested, effective, error or
                              ('Nav2 视觉已启用，仅修改新深度任务' if active else
                               'Nav2 视觉未启用，保留原设置'))
        if notice is not None:
            # Only the first offer or a changed profile/error is logged; no
            # per-frame I/O and no lock is held while the logger runs.
            logging.getLogger(__name__).warning('Nav2 深度推理尺寸：%s → %s；%s', *notice)
        # No private/nav/engine lock is held across queue admission. An offer
        # already captured here may complete during a concurrent close/toggle.
        return self._original(offered_job)

    def status(self):
        """Read small diagnostics only; never invoke callbacks or copy images."""
        with self._lock:
            installed = (self._wrapper is not None and not self._closed
                         and getattr(self._depth, 'offer', None) is self._wrapper)
            if self._closed:
                message = 'Nav2 深度输入档位已关闭'
            elif self._wrapper is None:
                message = '深度工作器未提供 offer，Nav2 深度输入档位未安装'
            elif not installed:
                message = '深度 offer 已由其他接入替换，当前档位不再直接持有入口'
            elif self._error:
                message = self._error
            elif self._offered == 0:
                message = 'Nav2 深度输入档位已安装，等待新任务'
            elif self._active:
                message = 'Nav2 深度输入档位已应用于最近新任务'
            else:
                message = 'Nav2 视觉未启用，最近新任务保持原深度设置'
            return dict(installed=installed, closed=self._closed, limit=self.limit,
                        active=installed and self._active,
                        requested_imgsz=self._requested, effective_imgsz=self._effective,
                        offered=self._offered, adjusted=self._adjusted,
                        error=self._error, message=message)

    def close(self):
        """Restore only our own entry point; never clear another owner's work."""
        with self._lock:
            self._closed = True
            self._active = False
        if self._wrapper is not None and getattr(self._depth, 'offer', None) is self._wrapper:
            self._depth.offer = self._original
