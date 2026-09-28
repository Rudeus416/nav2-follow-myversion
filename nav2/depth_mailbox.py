# 【内容标注】用途：最新深度结果的轻量读取桥。
# 对应用户需求：R22（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：保留原回调，额外保存最新结果引用；按流/配置过滤，关闭时恢复回调。
# 本次仅加注释；需求关联不是精确创建/提交记录。
"""A navigation-only latest-result mailbox alongside the original fusion callback.

No model/settings change, no extra inference, no queue consumption. The original
callback still receives each result exactly once with unchanged exception behavior.
"""
import threading


# 【职责 / R22】DepthMailbox：最新深度结果的轻量读取桥的状态封装；各方法职责见下方标注。
class DepthMailbox:
    # 【职责 / R22】__init__：初始化本类依赖与状态；副作用以原初始化语句为准。
    def __init__(self, depth_worker):
        self._lock=threading.Lock()
        self._latest=None
        self._closed=False
        self._depth=depth_worker
        self._original=depth_worker.on_result

        # 【职责 / R22】forward：先保存最新深度引用并保持原回调调用语义，不复制大图。
        def forward(result):
            # Constant-time reference copy before fusion; never hold this lock
            # while calling the original processing code.
            with self._lock:
                if not self._closed:
                    previous=self._latest
                    if previous is None or result.frame.source_at>=previous.frame.source_at:
                        self._latest=result
            self._original(result)
        self._forward=forward
        depth_worker.on_result=forward

    # 【职责 / R22】latest：按配置版本及相机流读取最新有效结果。
    def latest(self, version, epoch=None):
        with self._lock:
            record=self._latest
            if (record is None or record.config_version!=version or
                    (epoch is not None and record.frame.stream_epoch!=epoch)):
                return None
            return record

    # 【职责 / R22】close：只在仍持有接入回调时恢复原回调。
    def close(self):
        with self._lock:
            self._closed=True
            self._latest=None
        # Do not overwrite somebody else's later callback replacement.
        if self._depth.on_result is self._forward:
            self._depth.on_result=self._original
