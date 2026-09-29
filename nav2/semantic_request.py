# 【内容标注 / R38】丢弃过期的附加语义请求，不拿旧图片消耗新的推理时间。
"""Original capture timestamp is checked before and after lazy model setup."""
import math
import time


# 【职责 / R38】SemanticRequestExpired：标识任务在排队、初始化或推理前已经过期。
class SemanticRequestExpired(RuntimeError):
    pass


# 【职责 / R38】require_fresh：始终按原采集时间拒绝超过 1.2 秒的语义任务。
def require_fresh(source_at):
    age=time.monotonic()-source_at
    if not math.isfinite(age) or not 0<=age<=1.2:
        raise SemanticRequestExpired('语义请求源帧已过期，跳过本次识别')


# 【职责 / R38】RequestFreshness：把当前任务采集时间带到 Ultralytics 推理前回调。
class RequestFreshness:
    """Ultralytics on_predict_start callback; bound to this worker's current job."""
    # 【职责 / R38】__init__：初始化尚未绑定任务的时效守卫。
    def __init__(self):
        self.source_at=None

    # 【职责 / R38】begin：绑定新任务并在任何模型工作前先检查时效。
    def begin(self, source_at):
        self.source_at=source_at
        require_fresh(source_at)

    # 【职责 / R38】__call__：模型准备完成、真正 forward 前再次拒绝已过期任务。
    def __call__(self, _predictor):
        # Setup may include loading/fusing a model. An image valid before setup
        # is not necessarily valid when the first actual inference is to run.
        if self.source_at is None:
            raise SemanticRequestExpired('缺少语义源帧时间')
        require_fresh(self.source_at)
