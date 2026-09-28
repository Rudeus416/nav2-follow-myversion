# 【内容标注】用途：人物世界坐标、停车候选和速度几何。
# 对应用户需求：R13 R16 R18 R19（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：相机测量转 odom；生成保留跟随距离的停车点；同步缩放速度保持曲率，增加加速过渡。
# 需求关联用于追溯用途，不代表精确创建/提交记录。
"""Navigation-only geometry; never changes the original chassis controller."""
import math


# 【职责 / R13 R16 R18 R19】person_position：将相机距离/方位变换为不截断的 odom 人物坐标。
def person_position(distance, bearing, robot_x, robot_y, robot_yaw,
                    camera_x=0., camera_y=0., camera_yaw=0.):
    """Unclipped person position in odom, independent of stand-off and robot motion."""
    values=(distance,bearing,robot_x,robot_y,robot_yaw,camera_x,camera_y,camera_yaw)
    if not all(math.isfinite(v) for v in values) or distance <= 0:
        raise ValueError('人物位置测量无效')
    x=camera_x+distance*math.cos(camera_yaw+bearing)
    y=camera_y+distance*math.sin(camera_yaw+bearing)
    return (robot_x+math.cos(robot_yaw)*x-math.sin(robot_yaw)*y,
            robot_y+math.sin(robot_yaw)*x+math.cos(robot_yaw)*y)


# 【职责 / R13 R16 R18 R19】limit_twist：统一缩放线角速度，遵守上限且不改变曲率。
def limit_twist(linear, yaw, forward=.18, reverse=.15, angular=.4):
    """Scale both components together: preserve v/w and hence turning curvature."""
    if not all(math.isfinite(v) for v in (linear,yaw,forward,reverse,angular)):
        return 0.,0.
    cap=max(0., forward if linear >= 0 else reverse)
    factor=1.
    if abs(linear)>0:factor=min(factor,cap/abs(linear))
    if abs(yaw)>0:factor=min(factor,max(0.,angular)/abs(yaw))
    return linear*factor,yaw*factor


# 【职责 / R13 R16 R18 R19】person_stopping_candidates：提议更远及两侧停车点；候选不等于可通行，必须交给整车校验。
def person_stopping_candidates(goal, person, robot):
    """Bounded approach-side alternatives; these are proposals, NOT safe paths.

    Never shrink stand-off, exceed the existing 3m segment, or remove the person
    from the costmap. GridBased and the full-body validator must approve a path.
    """
    if not all(math.isfinite(v) for v in (*goal, *person[:2], *robot[:2])):
        raise ValueError('人物候选停车点坐标无效')
    px, py = person[:2]
    radius = math.hypot(goal[0]-px, goal[1]-py)
    angle = math.atan2(goal[1]-py, goal[0]-px)
    result = [goal]
    # Try a little farther from the person first, then both approach-side flanks.
    for extra, degrees in ((.25,0), (.5,0), (.25,30), (.25,-30),
                           (.5,30), (.5,-30), (.5,60), (.5,-60)):
        a = angle + math.radians(degrees)
        x, y = px+(radius+extra)*math.cos(a), py+(radius+extra)*math.sin(a)
        travel = math.hypot(x-robot[0], y-robot[1])
        if .1 < travel <= 3.+1e-6:
            result.append((x, y, math.atan2(py-y, px-x)))
    return result


# 【职责 / R13 R16 R18 R19】CurvatureRamp：人物世界坐标、停车候选和速度几何的状态封装；各方法职责见下方标注。
class CurvatureRamp:
    """Ramp increases only; output stays a scalar multiple of current command.

    Never blend old and new curvatures or retain speed after a zero command.
    Linear direction reversals cross zero. A steering correction through zero
    only restarts angular acceleration; it must not repeatedly stop forward
    travel. Safety stops reset the ramp immediately.
    """
    # 【职责 / R13 R16 R18 R19】__init__：初始化本类依赖与状态；副作用以原初始化语句为准。
    def __init__(self, linear_accel=.30, angular_accel=.70):
        self.linear_accel=linear_accel;self.angular_accel=angular_accel
        self.reset()

    # 【职责 / R13 R16 R18 R19】reset：清除平滑器历史，使下次重新缓起。
    def reset(self):
        self.previous=(0.,0.);self.at=None

    # 【职责 / R13 R16 R18 R19】apply：按当前曲率限制加速；前后换向整车过零，左右修正不强制清零线速度。
    def apply(self, linear, angular, now):
        if not all(math.isfinite(v) for v in (linear,angular,now)):
            self.reset();return (0.,0.)
        if linear==0 and angular==0:
            self.reset();return (0.,0.)
        dt=.02 if self.at is None else min(.05,max(0.,now-self.at))
        if self.at is not None and (now-self.at>.3 or now<self.at):
            self.reset();dt=.02
        old=self.previous
        if old[0]*linear<0:
            self.reset();self.at=now;return (0.,0.)
        # Braking is intentionally immediate. On a left/right reversal, treat
        # the old angular velocity as braked to zero, then ramp the NEW turn.
        # Applying the old whole-vehicle reset here made small DWB +/- steering
        # corrections restart forward acceleration forever, causing no-progress
        # failures. Keep one shared factor so we still follow the current
        # collision-checked curvature, never a blend of old and new arcs.
        previous_angular=0. if old[1]*angular<0 else old[1]
        factor=1.
        for wanted,previous,accel in zip((linear,angular),(old[0],previous_angular),
                                       (self.linear_accel,self.angular_accel)):
            if abs(wanted)>abs(previous):factor=min(factor,(abs(previous)+accel*dt)/abs(wanted))
        self.previous=(linear*factor,angular*factor);self.at=now
        return self.previous
