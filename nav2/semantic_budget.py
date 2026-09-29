# 【内容标注 / R38】用户“能不能修复这个问题”：深度超时前暂停附加语义负载。
# 只控制自有 YOLOE 新任务；不改深度源、人物追踪、运动时效或障碍阈值。
"""Shed optional semantic work when the depth stream has little time remaining."""
import math
import time


class SemanticBudget:
    """Hysteresis on observed source age and completed-result cadence.

    A pause retains the loaded model and drains pending replies. Recovery needs
    five distinct healthy observations and a three-second quiet period, so a
    single fast frame cannot repeatedly restart optional work. This never turns
    stale depth into motion permission and does not interrupt an in-flight call.
    """
    def __init__(self):
        self.paused=False
        self.until=0.
        self.healthy_frames=0
        self._key=None
        self._completed=None
        self._gap=None
        self._source=None

    def observe(self, record, now=None):
        now=time.monotonic() if now is None else now
        if record is None:
            return self._hold(now,'等待深度帧',None,None)
        source=record.frame.source_at
        age=now-source
        key=(record.config_version,record.frame.stream_epoch,record.frame.frame_id)
        completed=getattr(record,'completed_at',None)
        valid_completed=(isinstance(completed,(float,int)) and math.isfinite(completed)
                         and source<=completed<=now)
        new=key!=self._key
        same_context=self._key is not None and key[:2]==self._key[:2]
        if new:
            if (same_context and self._source is not None and source<=self._source):
                return self._hold(now,'深度帧乱序，暂停额外语义计算',age,None)
            gap=(completed-self._completed if same_context and valid_completed
                 and self._completed is not None else None)
            self._gap=gap if gap is not None and gap>0 else None
            self._key=key;self._source=source
            self._completed=completed if valid_completed else None
            if not same_context:
                self.healthy_frames=0
        # Completion-to-capture delay plus result cadence estimates how old the
        # current evidence will be just before the next result. Repeated polling
        # cannot advance cadence or count as another recovery observation.
        ready_age=completed-source if valid_completed else age
        projected=ready_age+(self._gap or 0.)
        invalid=not math.isfinite(age) or age<0 or not math.isfinite(projected)
        if invalid or age>.85 or projected>1.05:
            return self._hold(now,'深度更新偏慢，暂停额外语义计算',age,projected)
        if self.paused:
            # A result that already caused a pause cannot count as a healthy one
            # on another poll, even if another diagnostic is now available.
            if new:
                self.healthy_frames=(min(5,self.healthy_frames+1)
                                     if age<=.65 and projected<=.85 else 0)
            if now>=self.until and self.healthy_frames>=5:
                self.paused=False
            else:
                return self._state(now,age,projected,'深度恢复观察中，暂缓额外语义计算')
        return self._state(now,age,projected,'深度时序正常')

    def _hold(self, now, reason, age, projected):
        self.paused=True;self.until=now+3.;self.healthy_frames=0
        return self._state(now,age,projected,reason)

    def _state(self, now, age, projected, reason):
        def finite(value):
            return round(value,3) if value is not None and math.isfinite(value) else None
        return dict(allowed=not self.paused,paused=self.paused,reason=reason,
                    source_age=finite(age),producer_interval=finite(self._gap),
                    projected_age=finite(projected),healthy_frames=self.healthy_frames,
                    hold_remaining=round(max(0.,self.until-now),3) if self.paused else 0.)
