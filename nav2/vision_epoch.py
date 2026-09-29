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


# 【职责 / R37】VisionSnapshot：冻结一次视觉配置代次、流身份和可用状态。
@dataclass(frozen=True)
class VisionSnapshot:
    token: int
    version: object
    epoch: object
    ready: bool = True


# 【职责 / R37】VisionEpoch：配置开始前失效旧代次，成功结束后发布一致快照。
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

    # 【职责 / R37】__init__：只包装当前引擎实例的 configure，并建立短锁快照。
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
                # 【职责 / R37】configure：把该实例的配置调用转给代次门控。
                @functools.wraps(self._original)
                def configure(*args, **kwargs):
                    return self._configure(*args, **kwargs)
                self._wrapper = configure
                engine.configure = configure

    # 【职责 / R37】_read_key：在引擎锁内复制版本和流身份，不等待融合工作。
    def _read_key(self):
        """Called under engine.lock; no private gate lock is held on entry."""
        return VisionSnapshot(self._token, self._engine._config_version,
                              getattr(self._engine, '_stream_epoch', None))

    # 【职责 / R37】snapshot：短锁返回当前可提交代次；配置中或关闭时为空。
    def snapshot(self):
        """Return a coherent ready generation, or None while unsafe/closed."""
        with self._lock:
            return self._snapshot

    # 【职责 / R37】current：用对象身份确认结果仍属于当前代次。
    def current(self, snapshot):
        """Check both generation identity and readiness without engine.lock."""
        with self._lock:
            return snapshot is not None and snapshot is self._snapshot

    # 【职责 / R37】_configure：先使旧结果失效，再执行原配置；仅最后一个成功批次恢复快照。
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

    # 【职责 / R37】close：关闭读取并仅在仍拥有包装器时恢复原 configure。
    def close(self):
        """Disable reads and restore configure only while owning its wrapper."""
        with self._lock:
            self._closed = True
            self._snapshot = None
        if self._wrapper is not None and self._engine.configure is self._wrapper:
            self._engine.configure = self._original
