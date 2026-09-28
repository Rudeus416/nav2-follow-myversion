# 【内容标注】用途：近距障碍停车与恢复判定。
# 对应用户需求：R19（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：保留 0.70m 最低停车距离并增加延迟/速度提前量；危险立即进入，恢复需余量和三帧。
# 本次仅加注释；需求关联不是精确创建/提交记录。
"""Conservative near-field guard; no temporal delay on hazard entry."""
import math
import numpy as np


# 【职责 / R19】NearObstacleGuard：近距障碍停车与恢复判定的状态封装；各方法职责见下方标注。
class NearObstacleGuard:
    # 【职责 / R19】__init__：初始化本类依赖与状态；副作用以原初始化语句为准。
    def __init__(self):
        self.blocked=False
        self.clear_count=0
        self.last_stamp=None
        self.release_at=.82
        self.blocked_width=.40

    # 【职责 / R19】update：结合速度、源帧年龄和转弯宽度判断危险；立即触发、带余量的三帧安全恢复。
    def update(self, points, speed, yaw_rate, source_at, now):
        if not all(math.isfinite(x) for x in (speed,yaw_rate,source_at,now)) or not 0<=now-source_at<=1.2:
            raise ValueError('近距障碍数据无效或超时')
        p=np.asarray(points,dtype=float).reshape(-1,2)
        if not np.isfinite(p).all():raise ValueError('近距障碍坐标无效')
        # Keep the old .70m minimum. Add capture latency and conservative
        # braking-distance allowance; this is a heuristic, not a calibrated model.
        v=max(0.,speed)
        threshold=.70+v*(now-source_at)+v*v/(2*.5)
        width=.40+min(.15,abs(yaw_rate)*.3)
        if self.blocked:width=max(width,self.blocked_width)
        near=p[(p[:,0]>0)&(np.abs(p[:,1])<=width)]
        distance=float(near[:,0].min()) if len(near) else None
        danger=distance is not None and distance<threshold
        new_frame=self.last_stamp is None or source_at>self.last_stamp
        if danger:
            self.blocked=True;self.clear_count=0
            self.release_at=max(self.release_at,threshold+.12)
            self.blocked_width=width
        elif new_frame:
            if self.last_stamp is not None and source_at-self.last_stamp>1.2:self.clear_count=0
            clear=distance is None or distance>=max(threshold+.12,self.release_at)
            self.clear_count=min(3,self.clear_count+1) if clear else 0
            if self.clear_count>=3:
                self.blocked=False;self.release_at=.82;self.blocked_width=.40
        if new_frame:self.last_stamp=source_at
        return self.blocked, {'near_distance_m':distance,'near_stop_m':threshold,
                             'near_half_width_m':width,'near_clear_frames':self.clear_count,
                             'near_blocked':self.blocked}
