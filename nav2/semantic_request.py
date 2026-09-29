# 【内容标注 / R38】丢弃过期的附加语义请求，不拿旧图片消耗新的推理时间。
"""Original capture timestamp is checked before and after lazy model setup."""
import math
import time


class SemanticRequestExpired(RuntimeError):
    pass


def require_fresh(source_at):
    age=time.monotonic()-source_at
    if not math.isfinite(age) or not 0<=age<=1.2:
        raise SemanticRequestExpired('语义请求源帧已过期，跳过本次识别')


class RequestFreshness:
    """Ultralytics on_predict_start callback; bound to this worker's current job."""
    def __init__(self):
        self.source_at=None

    def begin(self, source_at):
        self.source_at=source_at
        require_fresh(source_at)

    def __call__(self, _predictor):
        # Setup may include loading/fusing a model. An image valid before setup
        # is not necessarily valid when the first actual inference is to run.
        if self.source_at is None:
            raise SemanticRequestExpired('缺少语义源帧时间')
        require_fresh(self.source_at)
