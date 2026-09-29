# 【内容标注 / R40】用户“优化”：同一深度帧的安全检查、地图候选和相机回退共用几何预处理。
# 本模块只处理一帧；不缓存跨帧数据，不延长源数据有效期，不改变候选筛选阈值。
"""Single-frame depth projection with independent safety and map selections."""
import math

import cv2
import numpy as np


# 【职责 / R40】DepthProjection：一帧共享深度投影和连通域，按需计算最近物体分组。
class DepthProjection:
    """Own projected arrays for one frame, with lazy nearest-object grouping.

    The full safety cloud does not need depth grouping or map clipping. Build it
    first, then reuse the projection when selecting display/navigation candidates.
    The caller owns this object for one serial visual tick only; it is not a
    cross-frame cache and does not carry or replace source timestamps.
    """

    # 【职责 / R40】__init__：投影并冻结同帧基础数组，后续筛选不修改源深度。
    def __init__(self, depth, shape, calibration, height, pitch, scale):
        values = cv2.resize(np.squeeze(depth).astype(np.float32), (80, 60),
                            interpolation=cv2.INTER_NEAREST) * scale
        valid = np.isfinite(values) & (values > 0)
        if np.mean(valid) < .5:
            raise ValueError('视觉深度有效像素不足')
        h, w = shape[:2]
        yy, xx = np.indices(values.shape)
        pixels = np.stack(((xx + .5) * w / 80, (yy + .5) * h / 60), axis=-1).astype(float)
        k = np.array(calibration['camera_matrix']['data'], float).reshape(3, 3)
        k[0, :] *= w / calibration['image_width']
        k[1, :] *= h / calibration['image_height']
        rays = cv2.undistortPoints(pixels.reshape(-1, 1, 2), k,
            np.array(calibration['distortion_coefficients']['data'], float)).reshape(60, 80, 2)
        angle = math.radians(pitch)
        forward = values * (math.cos(angle) - rays[:, :, 1] * math.sin(angle))
        lateral = -values * rays[:, :, 0]
        z = height - values * (math.sin(angle) + rays[:, :, 1] * math.cos(angle))
        ranged = valid & (forward > .1) & (forward < 4) & (np.abs(lateral) < 2)
        geometric = ranged & (z > .10) & (z < 1.8)
        # R40: preserve support before clipping. Safety retains all supported
        # image components; only nearest-object selection requires depth groups.
        count, labels, stats, _ = cv2.connectedComponentsWithStats(geometric.astype(np.uint8), 8)
        eligible = [i for i in range(1, count) if stats[i, cv2.CC_STAT_AREA] >= 6]
        components = np.isin(labels, eligible)
        self._forward, self._lateral = forward, lateral
        self._valid, self._ranged = valid, ranged
        self._geometric, self._components = geometric, components
        self._scale = scale
        self._nearest_labels = None
        self._nearest_mask = None
        # Each caller receives copies of masks/results, so clipping or output
        # edits cannot remove points needed by the independent safety selection.
        for array in (forward, lateral, valid, ranged, geometric, components):
            array.setflags(write=False)

    # 【职责 / R40】_nearest：最多一次计算深度连通域和最近碎片掩膜。
    def _nearest(self, split_components):
        if self._nearest_labels is None:
            labels = split_components(self._components, self._forward)
            support = np.bincount(labels.ravel(), minlength=self._components.size + 1)
            mask = self._components & (support[labels] >= 6)
            labels.setflags(write=False)
            mask.setflags(write=False)
            self._nearest_labels, self._nearest_mask = labels, mask
        return self._nearest_labels, self._nearest_mask

    # 【职责 / R40】points：用独立掩膜筛选安全、地图或相机候选，保持原像素顺序。
    def points(self, *, accept=None, nearest_only=True, diagnostics=None,
               instances=None, split_components=None, nearest_fragments=None):
        """Select points in original row order, without mutating shared masks.

        The two selection callbacks come from the existing navigation code. This
        keeps their established grouping/percentile behavior and test entrypoints.
        ``accept`` is evaluated independently for every request; an out-of-map
        selection cannot contaminate the unrestricted camera fallback or safety.
        """
        if nearest_only:
            if split_components is None or nearest_fragments is None:
                raise ValueError('最近候选筛选缺少深度分组方法')
            labels, supported = self._nearest(split_components)
        else:
            labels, supported = None, self._components
        mask = supported.copy()
        before_map = int(mask.sum())
        accepted = None
        if accept is not None:
            accepted = np.asarray(accept(np.column_stack((self._forward.ravel(),
                self._lateral.ravel()))), dtype=bool).reshape(mask.shape)
            mask &= accepted
        keep = (nearest_fragments(np.where(mask, labels, 0), self._forward, self._lateral)
                if nearest_only else mask)
        selected_label = None
        if nearest_only and instances:
            candidates = []
            for label, instance in instances:
                full = self._geometric & instance
                if np.count_nonzero(full) < 6:
                    continue
                clipped = full if accept is None else full & accepted
                if clipped.any():
                    candidates.append((float(np.percentile(self._forward[clipped], 10)),
                                       label, clipped))
            if candidates:
                distance, label, instance = min(candidates, key=lambda item: item[0])
                # A classified object may not hide a substantially nearer
                # unclassified one. The previous .30 m allowance is unchanged.
                if not keep.any() or distance <= float(np.percentile(self._forward[keep], 10)) + .30:
                    keep, selected_label = instance, label
        if diagnostics is not None:
            diagnostics.update(valid_pixels=int(self._valid.sum()),
                range_pixels=int(self._ranged.sum()), height_pixels=int(self._geometric.sum()),
                component_pixels=before_map, map_pixels=int(mask.sum()),
                selected_pixels=int(keep.sum()), depth_scale=float(self._scale))
            if selected_label is not None:
                reason = 'YOLOE 物体有效深度已用于候选：' + selected_label
            elif not before_map:
                reason = ('距离/横向范围内无候选' if not self._ranged.any() else
                          '高度筛选后无候选' if not self._geometric.any() else '候选面积不足')
            elif not mask.any():
                reason = '视觉候选位于地图外；请核对车位、相机安装参数和深度比例'
            else:
                reason = '地图内候选有效'
            diagnostics['reason'] = reason
            diagnostics['selected_object'] = selected_label
        return np.column_stack((self._forward[keep], self._lateral[keep]))
