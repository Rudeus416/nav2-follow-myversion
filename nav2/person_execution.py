# 【内容标注】用途：开始追踪按钮的执行事务。
# 对应用户需求：R17 R18 R30 R31（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：显式锁定人物历史点后复核预览序号/时效/车位，执行同一路径；失败恢复停车。
# 需求关联不是精确创建/提交记录；R31 移除锁定路线对实时人物的重复依赖，保留整车/TF/障碍保护。
"""Explicit start of a previewed person route, through the existing speed gate."""
import math
import time
from fastapi import HTTPException


# 【职责 / R17 R18】共享启动条件：网页反馈和实际启动读取同一组条件，避免按钮看似可用却被拒绝。
def _checked_state(motion, stream_key, sequence, continuous):
    """Read-only shared guards; the caller holds motion.lock and nav.lock."""
    nav = motion.navigation
    if not motion.estop or motion.radar_reconfiguring:
        raise HTTPException(409, '请保持强停并完成传感器设置，再开始追踪')
    if nav.pending or nav.canceling or nav.handle is not None or nav.fixed_goal_active:
        raise HTTPException(409, '旧导航尚未结束，请稍后重试')
    if continuous and not getattr(nav,'vision_enabled',False):
        raise HTTPException(409,'连续追踪需要先启用视觉障碍检测，以监测途中突发障碍')
    marker = getattr(nav, 'person_target_marker', None)
    if (not marker or marker['id'] != motion.settings.target_id
            or tuple(marker['stream_key']) != stream_key()
            or sequence != nav.preview_sequence
            or sequence != getattr(nav, 'person_marker_sequence', None)):
        raise HTTPException(409, '人物或预览已改变，请重新规划人物路线')
    # R31: this is the explicitly recorded endpoint, not a live-person gate.
    # A malformed historical position must still never authorize movement.
    try:
        position = marker['position']
        valid_position = len(position) == 2 and all(math.isfinite(value) for value in position)
    except (KeyError, TypeError, ValueError, OverflowError):
        valid_position = False
    if not valid_position:
        raise HTTPException(409, '锁定人物点的位置无效，请重新记录并规划')
    path = getattr(nav, 'preview_full_path', None)
    if (nav.preview['state'] != 'ready' or not nav.preview.get('points')
            or path is None or not path.poses or path.header.frame_id != 'odom'
            or not 0 <= time.monotonic()-nav.preview_at <= 30.):
        raise HTTPException(409, '没有 30 秒内的有效人物路线，请重新规划')
    return marker, path


# 【职责 / R30 R31】readiness：只读报告开始追踪的共同前置条件，不授权或发送运动。
def readiness(motion, people, stream_key, sequence, continuous=False):
    """Report whether a start may be submitted, without authorizing movement.

    Caller holds the existing motion/navigation locks. Only the explicitly
    locked marker and preview identity/age are checked. ``people`` is retained
    for call compatibility and deliberately unused: a disappearing or moved
    live detection must not replace or prevent this already planned segment.
    No TF, collision checks, state changes, client calls or GET logging occur.
    execute() still performs costly geometry and atomic rechecks on every POST.
    """
    try:
        _checked_state(motion, stream_key, sequence, continuous)
    except HTTPException as exc:
        return {'ready': False, 'reason': str(exc.detail)}
    return {'ready': True, 'reason': ''}


# 【职责 / R17 R18】execute：停车状态下复核，随后原子授权并执行同一条预览路径；失败恢复强停。
def execute(motion, observe, stream_key, sequence, continuous=False):
    """Validate while stopped, then atomically authorize the same exact path.

    R31: ``observe`` remains for call compatibility but is never invoked. The
    user explicitly chose the recorded endpoint, so live-person freshness or
    movement cannot alter this segment. Full route, pose, TF and action-server
    checks still apply; the execution layer retains obstacle/stop protection.
    """
    nav = motion.navigation

    with motion.lock, nav.lock:
        marker, path = _checked_state(motion, stream_key, sequence, continuous)
        generation = nav.generation
    # Expensive geometry must not hold up perception, stop requests or ROS callbacks.
    try:
        nav.validate_preview(path)
    except Exception as exc:
        raise HTTPException(409, '人物路线启动复核失败：'+str(exc)) from exc
    with motion.lock, nav.lock:
        current_marker, current_path = _checked_state(motion, stream_key, sequence, continuous)
        if current_path is not path or current_marker is not marker or generation != nav.generation:
            raise HTTPException(409, '复核期间状态改变，请重新规划')
        now = time.monotonic()
        try:
            rx, ry, yaw = nav.healthy_pose()
        except Exception as exc:
            raise HTTPException(409, str(exc)) from exc
        first=path.poses[0].pose; q=first.orientation
        heading=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if (math.hypot(rx-first.position.x,ry-first.position.y)>.05
                or abs(math.atan2(math.sin(yaw-heading),math.cos(yaw-heading)))>.1):
            raise HTTPException(409,'车位偏离预览起点，请重新规划')
        if not nav.path_client.server_is_ready():
            raise HTTPException(409,'Nav2 路线执行服务未就绪')
        last=path.poses[-1].pose; q=last.orientation
        nav.goal=(last.position.x,last.position.y,
                  math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        nav.goal_at=now
        nav.person_anchor=tuple(marker['position'])
        nav.person_locked_target=dict(marker,selected_at=now)
        nav.person_navigation.arrived_at=None
        nav.continuous_follow=continuous
        nav.continuous_blocked=False
        nav.continuous_target_id=marker['id']
        nav.pending=True
        # Like explicit map-point start, this click authorizes leaving stopped
        # mode only after all checks. Do not call set_following: it clears preview.
        motion.mode='auto';motion.following=True;motion.estop=False
        try:
            nav.person_navigation.execute_path(path,marker['id'],generation,locked=True)
        except Exception as exc:
            motion.estop=True;motion.following=False
            nav.pending=False;nav.stop('人物路线发送失败')
            raise HTTPException(409,'启动失败，保持强停：'+str(exc)) from exc
        return {'message':('已开始连续追踪：本段终点保持固定，到达后重新定位；1 米内等待' if continuous else '已开始追踪，执行刚才预览的整车路线；强停可随时停止')}
