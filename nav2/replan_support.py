# 【内容标注 / R32 R33 R34】遇障绕行的视觉、地图与整车门控；只供自有导航使用。
# 用户要求：差什么补什么，但是只可以修改你写的 nav2 代码。
# 不改原识别人/底盘接口，不把任意传感器错误当成可恢复的障碍。
"""Providers for fixed-endpoint obstacle recovery. No chassis publications."""
import math
import time
import numpy as np


# 【职责 / R32 R34】NearObstacle：区分新鲜近障与传感器丢失，只有前者可进入同终点绕行。
class NearObstacle(RuntimeError):
    """Fresh near-field obstacle, distinct from loss of sensor/pose data."""


# 【职责 / R32】recovery：取得当前人物导航所拥有的绕障状态机。
def recovery(nav):
    return getattr(getattr(nav, 'person_navigation', None), 'obstacle_replan', None)


# 【职责 / R32 R34】check_vision：检查视觉源年龄和错误；近障仅在明确允许时交给绕行处理。
def check_vision(nav, allow_near=False):
    if not getattr(nav, 'vision_enabled', False):
        return
    # Stale danger is still stale data: it cannot authorize planning or motion.
    age = time.monotonic() - getattr(nav, 'vision_at', 0.)
    if not 0 <= age <= 1.2:
        from nav2_follow import VisionStale
        raise VisionStale('视觉障碍数据超时，停车等待新数据', source_at=getattr(nav, 'vision_at', 0.))
    error = getattr(nav, 'vision_error', '')
    if error:
        if getattr(nav, 'vision_near_blocked', False):
            if not allow_near:
                raise NearObstacle(error)
        else:
            raise RuntimeError(error)


# 【职责 / R32 R34 R40】visual_snapshot：读取新鲜、全量且不可变的视觉安全证据和车身轮廓。
def visual_snapshot(nav):
    """Return fresh immutable full-cloud evidence, never just the nearest cluster."""
    sample = getattr(nav, 'vision_safety', None)
    now = time.monotonic()
    if not getattr(nav, 'vision_enabled', False) or not sample:
        raise RuntimeError('绕行缺少新鲜的全量视觉障碍数据')
    if sample.get('footprint') is None or not 0 <= now-sample['footprint_at'] <= 3.:
        raise RuntimeError('绕行缺少新鲜的完整车身轮廓')
    age = now-sample['source_at']
    if not math.isfinite(age) or age < 0:
        raise RuntimeError('全量视觉障碍数据的源时间无效')
    if age > 1.2:
        from nav2_follow import VisionStale
        raise VisionStale('全量视觉障碍数据超时，停车等待新数据',
                          source_at=sample['source_at'])
    return sample


# 【职责 / R32 R33】attach：安装车身、停车位姿和新路线复核提供器，不新增运动通道。
def attach(nav, view):
    """Install read-only providers; all whole-route work remains outside locks."""
    # 【职责 / R32 R33】footprint：按轮廓自身时间戳恢复底盘坐标轮廓，读取不能续期证据。
    def footprint():
        # R33: read only the four body vertices. A whole browser snapshot also
        # takes the map-editor lock; doing that every visual tick couples point
        # driving to an unrelated recovery provider. Never renew source ages.
        if hasattr(view, 'layers') and hasattr(view, 'lock'):
            now = time.monotonic()
            ros_now = view.node.get_clock().now().nanoseconds * 1e-9
            with view.lock:
                value = view.layers.get('footprint')
                foot = dict(value) if value else None
            if foot:
                receipt_age = now-foot['received']
                stamp_age = ros_now-foot['stamp'] if foot['stamp'] > 0 else receipt_age
                foot['age'] = max(receipt_age, stamp_age)
                foot['stale'] = foot['age'] > 3. or stamp_age < -.1
        else:
            # Support snapshot-only adapters without changing their interface.
            foot = view.snapshot()['layers'].get('footprint')
        if not foot or foot['stale'] or foot.get('stamp', 0.) <= 0.:
            raise RuntimeError('等待带有效时间戳的完整车身轮廓')
        # World footprint and robot marker arrive separately. Reconstruct the
        # body using the footprint's OWN timestamp, never the latest robot pose.
        from rclpy.time import Time
        transform = nav.tf.lookup_transform('odom', 'base_link',
            Time(nanoseconds=int(foot['stamp']*1e9))).transform
        q, t = transform.rotation, transform.translation
        a = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        c, sine = math.cos(a), math.sin(a)
        body = (np.asarray(foot['points'], float)-[t.x,t.y]) @ np.array([[c,-sine],[sine,c]])
        if not np.isfinite(body).all():
            raise RuntimeError('车身轮廓变换无效')
        # Source ages are preserved; a view read must never renew sensor evidence.
        return body, time.monotonic()-float(foot.get('age', 0.))

    # 【职责 / R32 R34】ready_pose：允许近障存在时仍复核视觉新鲜度、全量证据和当前 TF。
    def ready_pose():
        if not nav.vision_enabled:
            raise RuntimeError('遇障自动绕行需要视觉障碍检测')
        check_vision(nav, allow_near=True)
        visual_snapshot(nav)
        return nav.healthy_pose(check_vision=False)

    # 【职责 / R32 R40】validate：等待障碍后的新全局图，并锁外复核地图、整车和全量视觉。
    def validate(path, since):
        from nav2.route_preview import validate_route_snapshot
        from nav2.replan_safety import path_clear
        # Wait for a global snapshot published after this obstacle stop. The
        # full-cloud check below also catches vision points not yet in costmap.
        deadline = time.monotonic()+.8
        while True:
            snap = view.snapshot(include_global=True)
            grid = snap['layers'].get('global_map')
            if grid and not grid.get('stale') and grid.get('received', 0.) >= since:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError('障碍出现后的全局地图尚未更新，保持停车')
            time.sleep(.04)
        editing = snap.get('editing', {})
        validate_route_snapshot(path, snap)  # Original static/purple/full-body checks.
        with nav.lock:
            ready_pose()
            sample = visual_snapshot(nav)
        if not path_clear(path, sample['points'], sample['footprint']):
            raise RuntimeError('新路线整车扫掠仍会碰到视觉障碍，保持停车')
        # A manual edit/configuration invalidates authorization separately; this
        # check additionally rejects edits made while geometry ran outside locks.
        if view.snapshot().get('editing', {}) != editing:
            raise RuntimeError('复核期间手绘地图已改变，保持停车')

    nav.navigation_footprint = footprint
    nav.replan_ready_pose = ready_pose
    nav.validate_replan = validate


# 【职责 / R32 R40】capture_visual_safety：复制并冻结同帧全点云/车身，预先准备可复用碰撞几何。
def capture_visual_safety(nav, world_points, source_at, footprint_snapshot=None):
    """Called outside nav.lock by the visual worker; retain all valid candidates."""
    if footprint_snapshot is None:
        try:
            body, body_at = nav.navigation_footprint()
        except Exception:
            body, body_at = None, 0.
    else:
        body, body_at = footprint_snapshot
    points = np.array(world_points, dtype=float, copy=True).reshape(-1, 2)
    points.setflags(write=False)
    if body is not None:
        body = np.array(body, dtype=float, copy=True)
        body.setflags(write=False)
    snapshot=dict(points=points, source_at=source_at, footprint=body, footprint_at=body_at)
    # R40: prepare cells/body once per frame outside nav.lock. Command checks
    # still evaluate the actual pose, output velocity and original age each time.
    if body is not None:
        from nav2.replan_safety import prepare_geometry
        import cv2
        try:
            snapshot['geometry']=prepare_geometry(points,body)
        except (ValueError,TypeError,OverflowError,cv2.error):
            # Retain invalid evidence for the existing fail-closed checks.
            # In particular a preparation error is never an empty safe cloud.
            pass
    return snapshot


# 【职责 / R32 R40】remaining_index：按有限前向窗口单调更新路线进度，防止回头段被误跳过。
def remaining_index(nav, path, pose):
    """Monotonic, locally bounded progress; a nearby return leg is not a shortcut."""
    old = getattr(nav, '_obstacle_route_progress', None)
    if old is None or old[0] is not path:
        index, last_pose = 0, pose
    else:
        _, index, last_pose = old
    budget = .5+math.hypot(pose[0]-last_pose[0], pose[1]-last_pose[1])
    end, length = index, 0.
    while end+1 < len(path.poses) and length < budget:
        a, b = path.poses[end].pose.position, path.poses[end+1].pose.position
        length += math.hypot(b.x-a.x, b.y-a.y)
        end += 1
    index = min(range(index, end+1), key=lambda i:
        (path.poses[i].pose.position.x-pose[0])**2+
        (path.poses[i].pose.position.y-pose[1])**2)
    nav._obstacle_route_progress = (path, index, pose)
    return index


# 【职责 / R32 R34 R40】monitor_route：锁外复核剩余路线，只有同任务同证据结果可触发停车或绕行。
def monitor_route(nav, valid=None):
    """R40: full-cloud check; optional worker ownership guard runs under nav.lock.

    A canceled worker/configuration may finish geometry, but cannot pause, stop
    or replan a later task. Direct synchronous callers keep the same behavior.
    """
    from nav2.replan_safety import path_clear
    from nav2_follow import VisionStale
    with nav.lock:
        if valid is not None and not valid():
            return
        task = recovery(nav)
        if not task or not task.active or task.holding or not nav.enabled:
            return
        path = task.current_path
        if path is None or not path.poses:
            return
        try:
            pose = nav.replan_ready_pose()
            sample = visual_snapshot(nav)
        except VisionStale as exc:
            # R34: this added monitor must not bypass the existing fixed-route
            # brief pause by treating every exception as a permanent stop.
            nav.pause_for_vision(source_at=exc.source_at)
            return
        except Exception as exc:
            nav.stop(str(exc))
            return
        generation = nav.generation
        # R34: tick() also runs when no new depth frame arrived. Rechecking an
        # unchanged full route/cloud adds work to the visual producer itself.
        # Health checks above still run on every tick; the speed gate separately
        # checks the current robot pose against this cloud before every output.
        previous = getattr(nav, '_visual_route_checked', None)
        if (previous is not None and previous[0] is sample
                and previous[1] is path and previous[2] == generation):
            return
        # Past waypoints must not make an obstacle behind the car cancel a route.
        index = remaining_index(nav, path, pose)
    geometry=sample.get('geometry')
    clear = path_clear(path, geometry if geometry is not None else sample['points'],
                       None if geometry is not None else sample['footprint'],
                       start_index=index, robot_pose=pose)
    with nav.lock:
        if valid is not None and not valid():
            return
        if (task is not recovery(nav) or not task.active or task.holding
                or task.current_path is not path or nav.generation != generation
                or getattr(nav, 'vision_safety', None) is not sample or not nav.enabled):
            return
        # Geometry ran without the lock. Even unchanged evidence may have aged
        # meanwhile; expiry is a pause, not proof of a newly appeared obstacle.
        try:
            nav.replan_ready_pose()
            visual_snapshot(nav)
        except VisionStale as exc:
            nav.pause_for_vision(source_at=exc.source_at)
            return
        except Exception as exc:
            nav.stop(str(exc))
            return
        nav._visual_route_checked = (sample, path, generation)
        if not clear and not task.trigger('锁定路线前方出现新障碍，停车后保持原终点绕行'):
            nav.stop('锁定路线障碍复核未通过')


# 【职责 / R32 R40】guard_command：每次速度输出前用当前车位和全点云检查整车扫掠。
def guard_command(nav, pose, output):
    """Check the actual post-limit/post-ramp twist against full-cloud swept body."""
    task = recovery(nav)
    if not task or not task.active:
        return True
    from nav2.replan_safety import command_clear
    sample = visual_snapshot(nav)
    geometry=sample.get('geometry')
    clear = command_clear(geometry if geometry is not None else sample['points'],
                          None if geometry is not None else sample['footprint'],
                          pose, *output, sample['source_at'], time.monotonic())
    # The caller holds nav.lock: identity cannot change, age can. Never release
    # a twist or count a recovery attempt using evidence expired during geometry.
    visual_snapshot(nav)
    if clear:
        return True
    if not task.trigger('近距整车扫掠会碰到障碍，停车并复核绕行路线'):
        nav.stop('近距整车扫掠仍不安全，保持停车')
    return False
