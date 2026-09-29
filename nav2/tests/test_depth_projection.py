# 【内容标注 / R40】比较共享预处理与优化前实现，覆盖裁剪、语义选择、无效深度及输入隔离。
"""The reference is the pre-R40 selector, independent of the new projection."""
import math
import statistics
import time
import unittest
import warnings
from unittest import mock

import cv2
import numpy as np

from nav2.depth_projection import DepthProjection
from nav2.vision_layer import depth_components, nearest_fragments


def reference_depth_points(depth, shape, calibration, height, pitch, scale, accept=None, nearest_only=True, diagnostics=None, instances=None):
    values=cv2.resize(np.squeeze(depth).astype(np.float32),(80,60),interpolation=cv2.INTER_NEAREST)*scale
    if np.mean(np.isfinite(values)&(values>0))<.5:
        raise ValueError('视觉深度有效像素不足')
    h,w=shape[:2]; yy,xx=np.indices(values.shape)
    pixels=np.stack(((xx+.5)*w/80,(yy+.5)*h/60),axis=-1).astype(float)
    k=np.array(calibration['camera_matrix']['data'],float).reshape(3,3)
    k[0,:]*=w/calibration['image_width'];k[1,:]*=h/calibration['image_height']
    rays=cv2.undistortPoints(pixels.reshape(-1,1,2),k,np.array(calibration['distortion_coefficients']['data'],float)).reshape(60,80,2)
    a=math.radians(pitch)
    forward=values*(math.cos(a)-rays[:,:,1]*math.sin(a))
    lateral=-values*rays[:,:,0]
    z=height-values*(math.sin(a)+rays[:,:,1]*math.cos(a))
    valid=np.isfinite(values)&(values>0)
    ranged=valid&(forward>.1)&(forward<4)&(np.abs(lateral)<2)
    mask=ranged&(z>.10)&(z<1.8)
    # Reject image noise BEFORE map clipping. A real object must not become
    # noise merely because only a few of its pixels lie inside the map.
    count,labels,stats,_=cv2.connectedComponentsWithStats(mask.astype(np.uint8),8)
    eligible=[i for i in range(1,count) if stats[i,cv2.CC_STAT_AREA]>=6]
    mask=np.isin(labels,eligible)
    if nearest_only:
        # Apply the six-pixel support requirement BEFORE map clipping and AFTER
        # depth splitting: isolated near noise cannot borrow a far wall's area.
        labels=depth_components(mask,forward)
        support=np.bincount(labels.ravel(),minlength=mask.size+1)
        mask &= support[labels]>=6
    before_map=int(mask.sum())
    if accept is not None:
        accepted=accept(np.column_stack((forward.ravel(),lateral.ravel())))
        mask &= np.asarray(accepted,dtype=bool).reshape(mask.shape)
    # Keep pre-clipping identities and do not discard a valid object's surviving
    # pixels at the map border. Emergency nearest_only=False still uses ALL
    # original candidates, without the new depth grouping/selection.
    keep=nearest_fragments(np.where(mask,labels,0),forward,lateral) if nearest_only else mask
    selected_label=None
    if nearest_only and instances:
        candidates=[]
        geometric=ranged&(z>.10)&(z<1.8)
        for label,instance in instances:
            full=geometric&instance
            if np.count_nonzero(full)<6:continue
            clipped=full if accept is None else full&np.asarray(accepted,dtype=bool).reshape(full.shape)
            if clipped.any():candidates.append((float(np.percentile(forward[clipped],10)),label,clipped))
        if candidates:
            distance,label,instance=min(candidates,key=lambda item:item[0])
            # Never discard a substantially nearer unclassified obstacle.
            if not keep.any() or distance<=float(np.percentile(forward[keep],10))+.30:
                keep=instance;selected_label=label

    if diagnostics is not None:
        diagnostics.update(valid_pixels=int(valid.sum()), range_pixels=int(ranged.sum()),
                           height_pixels=int((ranged&(z>.10)&(z<1.8)).sum()),
                           component_pixels=before_map, map_pixels=int(mask.sum()),
                           selected_pixels=int(keep.sum()), depth_scale=float(scale))
        if selected_label is not None:
            reason='YOLOE 物体有效深度已用于候选：'+selected_label
        elif not before_map:
            reason=('距离/横向范围内无候选' if not ranged.any() else
                    '高度筛选后无候选' if not (ranged&(z>.10)&(z<1.8)).any() else '候选面积不足')
        elif not mask.any():reason='视觉候选位于地图外；请核对车位、相机安装参数和深度比例'
        else:reason='地图内候选有效'
        diagnostics['reason']=reason
        diagnostics['selected_object']=selected_label
    return np.column_stack((forward[keep],lateral[keep]))

CALIBRATION = {
    'image_width': 640, 'image_height': 480,
    'camera_matrix': {'data': [450., 0., 320., 0., 450., 240., 0., 0., 1.]},
    'distortion_coefficients': {'data': [-.06, .01, .0003, -.0005, 0.]},
}


def selected(projection, **kwargs):
    return projection.points(split_components=depth_components,
        nearest_fragments=nearest_fragments, **kwargs)


class DepthProjectionTests(unittest.TestCase):
    def arguments(self, depth, height=.5, pitch=0., scale=1.):
        return depth, (480, 640, 3), CALIBRATION, height, pitch, scale

    def assert_equivalent(self, args, **kwargs):
        expected_info, actual_info = {'preserved': 1}, {'preserved': 1}
        expected = reference_depth_points(*args, diagnostics=expected_info, **kwargs)
        actual = selected(DepthProjection(*args), diagnostics=actual_info, **kwargs)
        np.testing.assert_array_equal(actual, expected)
        self.assertEqual(actual_info, expected_info)
        return actual

    def test_equivalent_dense_depth_and_output_order(self):
        for height, pitch, scale in [(.5, 0., 1.), (.8, 20., 1.2), (.3, -10., .5)]:
            args = self.arguments(np.full((120, 160), 1.4, np.float32), height, pitch, scale)
            for nearest in [False, True]:
                self.assert_equivalent(args, nearest_only=nearest)

    def test_equivalent_fragmented_and_invalid_pixels(self):
        rng = np.random.default_rng(40)
        for index in range(8):
            depth = cv2.resize(rng.uniform(.06, 4.3, (12, 16)).astype(np.float32),
                               (80, 60), interpolation=cv2.INTER_NEAREST)
            depth.flat[rng.choice(depth.size, 250, replace=False)] = [np.nan, np.inf, 0., -1.][index % 4]
            args = self.arguments(depth)
            with warnings.catch_warnings():
                warnings.simplefilter('ignore', RuntimeWarning)
                for accept in [None, lambda points: points[:, 1] >= 0.,
                               lambda points: np.zeros(len(points), bool)]:
                    for nearest in [False, True]:
                        self.assert_equivalent(args, nearest_only=nearest, accept=accept)

    def test_shared_frame_map_clip_does_not_remove_safety_or_camera_pixels(self):
        args = self.arguments(np.ones((60, 80), np.float32))
        projection = DepthProjection(*args)
        safety_before = selected(projection, nearest_only=False)
        self.assertGreater(len(safety_before), 0)
        clipped = selected(projection, accept=lambda points: np.zeros(len(points), bool))
        self.assertEqual(len(clipped), 0)
        camera_after = selected(projection)
        safety_after = selected(projection, nearest_only=False)
        np.testing.assert_array_equal(camera_after, reference_depth_points(*args))
        np.testing.assert_array_equal(safety_after, safety_before)
        # Returned arrays are owned by the caller, never views into the cache.
        camera_after[:] = 42.
        safety_after[:] = -42.
        np.testing.assert_array_equal(selected(projection), reference_depth_points(*args))
        np.testing.assert_array_equal(selected(projection, nearest_only=False), safety_before)

    def test_map_accept_is_fresh_on_every_selection(self):
        args = self.arguments(np.ones((60, 80), np.float32))
        projection = DepthProjection(*args)
        for sign in [1., -1., 1.]:
            accept = lambda points, sign=sign: points[:, 1] * sign > .01
            actual = selected(projection, accept=accept)
            expected = reference_depth_points(*args, accept=accept)
            np.testing.assert_array_equal(actual, expected)
            self.assertTrue((actual[:, 1] * sign > .01).all())

    def test_depth_groups_are_lazy_and_reused_once(self):
        projection = DepthProjection(*self.arguments(np.ones((60, 80), np.float32)))
        grouping = mock.Mock(wraps=depth_components)
        def select(**kwargs):
            return projection.points(split_components=grouping,
                nearest_fragments=nearest_fragments, **kwargs)
        select(nearest_only=False)
        grouping.assert_not_called()
        select(accept=lambda points: np.zeros(len(points), bool))
        select()
        select(nearest_only=False)
        self.assertEqual(grouping.call_count, 1)

    def test_semantic_masks_preserve_previous_selection_and_map_clipping(self):
        depth = np.ones((60, 80), np.float32) * 1.6
        depth[15:35, 5:20] = .8
        near = np.zeros((60, 80), bool)
        near[15:35, 5:20] = True
        far = np.zeros((60, 80), bool)
        far[8:40, 45:65] = True
        args = self.arguments(depth)
        for instances in [[('cart', near)], [('chair', far)],
                          [('chair', far), ('cart', near)], [('tiny', np.eye(60, 80, dtype=bool))]]:
            for accept in [None, lambda points: points[:, 1] < 0.,
                           lambda points: np.zeros(len(points), bool)]:
                self.assert_equivalent(args, accept=accept, instances=instances)

    def test_semantic_instances_cannot_mutate_shared_or_original_masks(self):
        instance = np.ones((60, 80), bool)
        args = self.arguments(np.ones((60, 80), np.float32))
        projection = DepthProjection(*args)
        expected = instance.copy()
        selected(projection, instances=[('cart', instance)],
                 accept=lambda points: points[:, 1] > 0.)
        np.testing.assert_array_equal(instance, expected)
        np.testing.assert_array_equal(selected(projection, nearest_only=False),
                                      reference_depth_points(*args, nearest_only=False))

    def test_new_frame_has_no_cache_from_previous_frame(self):
        first = self.arguments(np.full((60, 80), .8, np.float32))
        second = self.arguments(np.full((60, 80), 2.5, np.float32))
        first_points = selected(DepthProjection(*first))
        second_points = selected(DepthProjection(*second))
        np.testing.assert_array_equal(second_points, reference_depth_points(*second))
        self.assertFalse(np.array_equal(first_points, second_points))

    def test_source_depth_and_calibration_are_not_retained_as_mutable_views(self):
        depth = np.ones((60, 80), np.float32)
        args = self.arguments(depth)
        expected = reference_depth_points(*args)
        projection = DepthProjection(*args)
        depth[:] = 3.
        np.testing.assert_array_equal(selected(projection), expected)
        self.assertEqual(CALIBRATION['camera_matrix']['data'][0], 450.)

    def test_invalid_depth_raises_the_original_error(self):
        for depth in [np.zeros((60, 80), np.float32), np.full((60, 80), np.nan),
                      np.full((60, 80), np.inf)]:
            args = self.arguments(depth)
            for function in [lambda: reference_depth_points(*args), lambda: DepthProjection(*args)]:
                with self.assertRaisesRegex(ValueError, '视觉深度有效像素不足'):
                    function()

    def test_empty_masks_diagnostics_match_all_three_reasons(self):
        # Far depth, valid depth projected below ground, and isolated area noise.
        for depth, height, pitch in [(np.full((60, 80), 5.), .5, 0.),
                                     (np.ones((60, 80)), .1, 60.)]:
            self.assert_equivalent(self.arguments(depth, height, pitch))
        depth = np.full((60, 80), 5., np.float32)
        depth[20, 30] = 1.
        self.assert_equivalent(self.arguments(depth))

    def test_report_shared_frame_performance_without_wall_clock_assertions(self):
        args = self.arguments(np.ones((60, 80), np.float32))
        timings = {}
        for name, use_shared in [('independent', False), ('shared', True)]:
            samples = []
            for _ in range(5):
                started = time.perf_counter()
                if use_shared:
                    frame = DepthProjection(*args)
                    selected(frame, nearest_only=False)
                    selected(frame, accept=lambda points: np.zeros(len(points), bool))
                    selected(frame)
                else:
                    reference_depth_points(*args, nearest_only=False)
                    reference_depth_points(*args, accept=lambda points: np.zeros(len(points), bool))
                    reference_depth_points(*args)
                samples.append((time.perf_counter() - started) * 1000)
            timings[name] = round(statistics.median(samples), 3)
        print('Depth single-frame safety/map/camera fallback median milliseconds:', timings)


if __name__ == '__main__':
    unittest.main()
