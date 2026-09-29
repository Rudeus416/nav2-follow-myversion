# 【内容标注】用途：锁定人物终点重新绕行时的全量视觉点几何安全检查。
# 对应用户需求：R32（遇到新障碍时绕行原终点，仅修改自有 Nav2 代码）。
# 添加逻辑：全部视觉候选按 5 cm 方格占用；检查完整车身和转角扫掠；
# R40（用户“优化”）：先校验完整采样预算，再保守筛除扫掠范围外的点；
# 空点云不再重复构建数百个车身凸包，近处点仍使用原整车扫掠检查。
# 命令检查包含采集延迟与制动行程，不代替地图/紫色禁区、TF、强停及视觉时效门控。
"""Pure geometry for replan decisions and directional near-obstacle gating.

Inputs are world/odom coordinates. No point is clipped to the map or reduced to
only the nearest object. The footprint must include the same padding used by
Nav2. This module creates no ROS clients, authorizations or motion commands.

The braking values are conservative model parameters, not a measured chassis
braking guarantee. Existing source freshness, TF, map and whole-body controller
checks remain mandatory at the call site.
"""
import math
from dataclasses import dataclass
from numbers import Integral

import cv2
import numpy as np


_RESOLUTION = .05
# Match the existing planner/validator: supplied footprint already has padding.
# Another geometric clearance radius here would reject paths the planner cannot
# be asked to move farther away from; retain only its numerical sweep allowance.
_POINT_MARGIN = 1e-6
_MAX_SAMPLES = 20000


@dataclass(frozen=True)
class _PreparedGeometry:
    """Frame-local validated geometry, with immutable detached array storage."""
    centers: np.ndarray
    corners: np.ndarray
    radius: float


def prepare_geometry(points_world, footprint):
    """Prepare one immutable visual snapshot for repeated geometric checks.

    Raises ValueError/TypeError/cv2.error for invalid input, just like the regular
    input validator. Pass its result as ``points_world`` and ``None`` as the
    footprint to the existing path_clear/command_clear entry points. No motion
    permission, source timestamp, robot pose or collision result is cached.
    Callers retain the original source-age and authorization checks each time.
    """
    centers, corners, radius = _inputs(points_world, footprint)
    # Bytes-backed arrays cannot have write permission restored, unlike a plain
    # ndarray with setflags(write=False). Mutating the source snapshot cannot
    # silently change a geometry object already used by another worker.
    centers = np.frombuffer(centers.tobytes(), dtype=float).reshape(-1, 2)
    corners = np.frombuffer(corners.tobytes(), dtype=float).reshape(-1, 2)
    return _PreparedGeometry(centers, corners, radius)


def _inputs(points_world, footprint):
    """Return occupied-cell centers and a convex, padded footprint, or fail."""
    if isinstance(points_world, _PreparedGeometry):
        if footprint is not None:
            raise ValueError('prepared geometry must use its original footprint')
        return points_world.centers, points_world.corners, points_world.radius
    points = np.asarray(points_world, dtype=float)
    if points.size == 0:
        points = np.empty((0, 2), dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        raise ValueError('invalid visual points')
    corners = np.asarray(footprint, dtype=float)
    if (corners.ndim != 2 or corners.shape[1] != 2 or len(corners) < 3
            or not np.isfinite(corners).all()):
        raise ValueError('invalid footprint')
    # Convexifying a supplied footprint only enlarges it; it cannot make an
    # unobserved notch or an incorrectly ordered vertex list look traversable.
    corners = cv2.convexHull(corners.astype(np.float32)).reshape(-1, 2).astype(float)
    if len(corners) < 3 or cv2.contourArea(corners.astype(np.float32)) <= 1e-10:
        raise ValueError('degenerate footprint')
    radius = float(np.max(np.linalg.norm(corners, axis=1)))
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError('invalid footprint radius')
    # A point represents its entire grid cell, not just a center/zero-width ray.
    # float64 grid coordinates avoid integer overflow for malformed world data.
    cells = np.unique(np.floor(points / _RESOLUTION), axis=0)
    centers = (cells + .5) * _RESOLUTION
    if not np.isfinite(centers).all():
        raise ValueError('invalid visual cell coordinates')
    return centers, corners, radius


def _pose(value):
    p = np.asarray(value, dtype=float)
    if p.shape != (3,) or not np.isfinite(p).all():
        raise ValueError('invalid robot pose')
    return p


def _polygon(corners, pose):
    x, y, angle = pose
    c, s = math.cos(angle), math.sin(angle)
    return corners @ np.array([[c, s], [-s, c]]) + [x, y]


def _clear_polygon(polygon, centers, margin):
    """Vectorized SAT against complete square cells; touching is collision."""
    if not len(centers):
        return True
    half = _RESOLUTION / 2
    low = polygon.min(axis=0) - margin - half
    high = polygon.max(axis=0) + margin + half
    nearby = centers[np.all((centers >= low) & (centers <= high), axis=1)]
    if not len(nearby):
        return True
    edges = np.roll(polygon, -1, axis=0) - polygon
    lengths = np.linalg.norm(edges, axis=1)
    good = lengths > 1e-10
    normals = np.column_stack((-edges[good, 1], edges[good, 0])) / lengths[good, None]
    axes = np.vstack((np.eye(2), normals))
    body = polygon @ axes.T
    projected = nearby @ axes.T
    extent = half * np.abs(axes).sum(axis=1)
    separated = ((body.max(axis=0) + margin < projected - extent)
                 | (body.min(axis=0) - margin > projected + extent))
    return bool(np.all(separated.any(axis=1)))


def _hull(first, second):
    # Recentering prevents float32 hull rounding from swallowing thin obstacles
    # when odom/world origins are large. All collision projections use float64.
    joined = np.vstack((first, second))
    origin = joined[0]
    return cv2.convexHull((joined-origin).astype(np.float32)).reshape(-1, 2).astype(float)+origin


def _trajectory_clear(poses, centers, corners, radius, center_sagitta=None):
    """Check full swept polygons; linear center interpolation for planned paths.

    Command rollouts supply exact circular sample poses plus the center-arc
    sagitta so neither the moving center nor a turning body corner is skipped.
    R40: all segment budgets are validated *before* any empty-cloud shortcut.
    A bounding box may discard cells only when the entire swept body, the full
    occupied cell and the same arc/numerical margins provably cannot intersect.
    """
    if not len(poses):
        return False
    segments = []
    checks = 0
    largest_margin = _POINT_MARGIN
    previous = poses[0]
    for index, pose in enumerate(poses[1:]):
        da = math.atan2(math.sin(pose[2]-previous[2]), math.cos(pose[2]-previous[2]))
        count = max(1, math.ceil((math.hypot(pose[0]-previous[0], pose[1]-previous[1])
                                 + abs(da)*radius) / (_RESOLUTION*.25)))
        checks += count
        if checks > _MAX_SAMPLES:
            return False
        margin = _POINT_MARGIN + radius*(1-math.cos(abs(da)/(2*count)))
        if center_sagitta is not None:
            margin += center_sagitta[index]
        if not math.isfinite(margin) or margin < _POINT_MARGIN:
            return False
        segments.append((da, count, margin))
        largest_margin = max(largest_margin, margin)
        previous = pose

    # This bound encloses every rotated corner, every interpolated center and
    # every conservatively expanded swept polygon. In particular it also keeps
    # thin cells grazed during a pure turn or reverse motion. The cell half-size
    # is included: testing just its center would wrongly discard edge contact.
    extent = radius + _RESOLUTION/2 + largest_margin + 1e-6
    low = poses[:, :2].min(axis=0) - extent
    high = poses[:, :2].max(axis=0) + extent
    if np.all(poses[:, 2] == poses[0, 2]):
        # Exact translation has no rotating-corner bulge. A fixed-orientation
        # footprint bound is tighter than its circumscribed circle, so a close
        # parallel wall can be rejected in one pass without repeated hull work.
        rotated = _polygon(corners, (0., 0., poses[0, 2]))
        extent = _RESOLUTION/2 + largest_margin + 1e-6
        low = poses[:, :2].min(axis=0) + rotated.min(axis=0) - extent
        high = poses[:, :2].max(axis=0) + rotated.max(axis=0) + extent
    centers = centers[np.all((centers >= low) & (centers <= high), axis=1)]
    if not len(centers):
        return True

    before = _polygon(corners, poses[0])
    if not _clear_polygon(before, centers, _POINT_MARGIN):
        return False
    previous = poses[0]
    for pose, (da, count, margin) in zip(poses[1:], segments):
        for step in range(1, count+1):
            fraction = step/count
            current = (previous[0]+fraction*(pose[0]-previous[0]),
                       previous[1]+fraction*(pose[1]-previous[1]),
                       previous[2]+fraction*da)
            after = _polygon(corners, current)
            if not _clear_polygon(_hull(before, after), centers, margin):
                return False
            before = after
        previous = pose
    return True


def _path_poses(path):
    if not hasattr(path, 'poses'):
        values = np.asarray(path, dtype=float)
        if values.ndim != 2 or values.shape[1] != 3 or not np.isfinite(values).all():
            raise ValueError('invalid path')
        return values
    values = []
    for entry in path.poses:
        p, q = entry.pose.position, entry.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
            raise ValueError('invalid path coordinates')
        norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
        if abs(norm-1.) > .01:
            raise ValueError('invalid path orientation')
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        values.append((p.x, p.y, yaw))
    return np.asarray(values, dtype=float).reshape(-1, 3)


def path_clear(path, points_world, footprint, start_index=0, robot_pose=None):
    """True only if the remaining path clears every supplied visual point cell.

    ``start_index`` is the caller's monotonic route progress index. Optional
    ``robot_pose`` checks the bridge from the actual car to that remaining path,
    so skipping passed poses cannot silently skip the current body's clearance.
    The caller separately checks point age/stream and static/global map safety.
    Invalid inputs, an empty route or exhausted work budget return False.
    """
    try:
        centers, corners, radius = _inputs(points_world, footprint)
        poses = _path_poses(path)
        if (not isinstance(start_index, Integral) or isinstance(start_index, bool)
                or start_index < 0 or start_index >= len(poses)):
            return False
        poses = poses[start_index:]
        if robot_pose is not None:
            poses = np.vstack((_pose(robot_pose), poses))
        return _trajectory_clear(poses, centers, corners, radius)
    except (ValueError, TypeError, AttributeError, IndexError, OverflowError, cv2.error):
        return False


def command_clear(points_world, footprint, robot_pose, linear, yaw, source_at, now):
    """Check the actual limited command, including source lag and braking travel.

    The source must still be 0..1.2 seconds old. Roll out the current v/w arc
    through age + 0.10 seconds reaction reserve, then add the conservative stop
    travel for a shared deceleration factor (0.5 m/s², 0.8 rad/s² bounds).
    A scalar brake keeps the same curvature as limit_twist/CurvatureRamp.
    Whole-body cell overlap at any time returns False, including a pure turn.
    This gate never bypasses TF, static/virtual walls or the original stale gate.
    """
    try:
        if not all(math.isfinite(v) for v in (linear, yaw, source_at, now)):
            return False
        age = now-source_at
        if not 0 <= age <= 1.2:
            return False
        origin = _pose(robot_pose)
        centers, corners, radius = _inputs(points_world, footprint)
        braking_time = max(abs(linear)/.5, abs(yaw)/.8)
        duration = age + .10 + braking_time/2
        count = max(1, math.ceil((abs(linear)+radius*abs(yaw))*duration/(_RESOLUTION*.25)))
        if count > _MAX_SAMPLES:
            return False
        times = np.linspace(0., duration, count+1)
        angle = yaw*times
        if abs(yaw) < 1e-8:
            dx, dy = linear*times, np.zeros_like(times)
            sagitta = 0.
        else:
            turn_radius = linear/yaw
            dx = turn_radius*np.sin(angle)
            dy = turn_radius*(1-np.cos(angle))
            sagitta = abs(turn_radius)*(1-math.cos(abs(yaw*duration)/(2*count)))
        c, s = math.cos(origin[2]), math.sin(origin[2])
        poses = np.column_stack((origin[0]+c*dx-s*dy, origin[1]+s*dx+c*dy, origin[2]+angle))
        # Only discard cells provably outside the entire motion's bounding
        # circle. No map-boundary filter is ever applied to safety observations.
        bound = radius + abs(linear)*duration + _RESOLUTION/2 + _POINT_MARGIN + 1e-6
        centers = centers[np.all(np.abs(centers-origin[:2]) <= bound, axis=1)]
        return _trajectory_clear(poses, centers, corners, radius, [sagitta]*count)
    except (ValueError, TypeError, AttributeError, IndexError, OverflowError, cv2.error):
        return False
