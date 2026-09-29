# 【内容标注 / R37】用户“修复这些模拟时出现的问题”：隔离配置代次读取与融合长锁。
# 仅包装当前引擎实例；不改原库文件；配置开始使旧结果失效，完成后发布一致快照。
"""Navigation's coherent configuration generation, independent of fusion's lock.

Install before serving configuration requests. ProcessingEngine changes its
configuration only through ``configure``; its stream epoch is fixed at creation.
A future stream-reset API must also run through this gate. Direct assignment to
engine generation fields is deliberately not a supported mutation interface.
"""
from dataclasses import dataclass
import functools
import threading


@dataclass(frozen=True)
class VisionSnapshot:
    token: int
    version: object
    epoch: object
    ready: bool = True


class VisionEpoch:
    """Mirror an engine generation and invalidate it before configuration waits.

``snapshot`` and ``current`` take only a short private lock. The optional
invalidation callback runs outside that lock, before the original configure
method, and can stop navigation or discard its previously committed evidence.
Only successful, completely finished configuration batches publish a snapshot.

The caller must recheck ``current(snapshot)`` while committing navigation state.
It must not hold the gate lock across external calls. The invalidation callback
must invalidate navigation under the same navigation lock used for that commit.
An engine without ``configure`` is accepted for read-only test fixtures.
    """

    def __init__(self, engine, invalidate_callback=None):
        self._engine = engine
        self._lock = threading.Lock()
        self._invalidate = invalidate_callback
        self._active = 0
        self._token = 0
        self._failed = False
        self._closed = False
        self._original = getattr(engine, 'configure', None)
        self._wrapper = None
        # Installation is part of startup, before any API can capture configure.
        with engine.lock:
            self._snapshot = self._read_key()
            if callable(self._original):
                @functools.wraps(self._original)
                def configure(*args, **kwargs):
                    return self._configure(*args, **kwargs)
                self._wrapper = configure
                engine.configure = configure

    def _read_key(self):
        """Called under engine.lock; no private gate lock is held on entry."""
        return VisionSnapshot(self._token, self._engine._config_version,
                              getattr(self._engine, '_stream_epoch', None))

    def snapshot(self):
        """Return a coherent ready generation, or None while unsafe/closed."""
        with self._lock:
            return self._snapshot

    def current(self, snapshot):
        """Check both generation identity and readiness without engine.lock."""
        with self._lock:
            return snapshot is not None and snapshot is self._snapshot

    def _configure(self, *args, **kwargs):
        with self._lock:
            if self._closed:
                active = False
            else:
                active = True
                if self._active == 0:
                    self._failed = False
                self._active += 1
                self._token += 1
                self._snapshot = None
        if not active:
            # A previously captured wrapper retains original call semantics.
            return self._original(*args, **kwargs)
        succeeded = False
        try:
            if self._invalidate is not None:
                self._invalidate()
            result = self._original(*args, **kwargs)
            succeeded = True
            return result
        finally:
            # The original method has released its own locks. Reacquire only
            # to copy a coherent key and publish the last completion atomically
            # with engine writers. Publishing a key copied before releasing
            # engine.lock could otherwise resurrect an older concurrent write.
            with self._engine.lock:
                version = self._engine._config_version
                epoch = getattr(self._engine, '_stream_epoch', None)
                with self._lock:
                    self._failed = self._failed or not succeeded
                    self._active -= 1
                    if not self._closed and self._active == 0 and not self._failed:
                        self._snapshot = VisionSnapshot(self._token, version, epoch)

    def close(self):
        """Disable reads and restore configure only while owning its wrapper."""
        with self._lock:
            self._closed = True
            self._snapshot = None
        if self._wrapper is not None and self._engine.configure is self._wrapper:
            self._engine.configure = self._original
