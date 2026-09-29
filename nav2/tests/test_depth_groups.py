# 【内容标注 / R38】验证最近深度碎片优化与旧选择结果等价；仅离线测试，不连接小车。
"""Fragment-selection equivalence, including interpolation and threshold edges."""
import statistics
import time
import unittest
import warnings

import numpy as np

from nav2.depth_groups import nearest_fragments


def original_nearest_fragments(labels, forward, lateral):
    """Independent reference copied before replacing the production selector."""
    ids = np.unique(labels)
    ids = ids[ids > 0]
    if not len(ids):
        return labels > 0
    distance = {i: float(np.percentile(forward[labels == i], 10)) for i in ids}
    seed = min(ids, key=lambda i: distance[i])
    selected = labels == seed
    seed_xy = np.column_stack((forward[selected], lateral[selected]))
    lo, hi = seed_xy.min(axis=0), seed_xy.max(axis=0)
    for label in ids:
        if label == seed or abs(distance[label] - distance[seed]) > .30:
            continue
        candidate = labels == label
        xy = np.column_stack((forward[candidate], lateral[candidate]))
        gap = np.maximum(0, np.maximum(lo - xy.max(axis=0), xy.min(axis=0) - hi))
        if np.linalg.norm(gap) <= .20:
            selected |= candidate
    return selected


class DepthGroupTests(unittest.TestCase):
    def assert_equivalent(self, labels, forward, lateral):
        actual = nearest_fragments(labels, forward, lateral)
        reference = original_nearest_fragments(labels, forward, lateral)
        np.testing.assert_array_equal(actual, reference)
        self.assertEqual(actual.dtype, np.dtype(bool))
        self.assertEqual(actual.shape, labels.shape)
        return actual

    def test_empty_and_nonpositive_labels(self):
        for labels in [np.zeros((4, 7), dtype=int), -np.ones((4, 7), dtype=int),
                       np.empty((0, 7), dtype=int)]:
            actual = self.assert_equivalent(labels, np.zeros(labels.shape),
                                            np.zeros(labels.shape))
            self.assertFalse(actual.any())

    def test_single_pixel_and_label_order_tie(self):
        labels = np.array([[8, 2, 6]])
        actual = self.assert_equivalent(labels, np.ones((1, 3)), np.array([[0., 2., 4.]]))
        np.testing.assert_array_equal(actual, [[False, True, False]])
        self.assertTrue(self.assert_equivalent(np.array([[7]]), np.array([[1.]]),
                                               np.array([[.1]])).item())

    def test_tenth_percentile_is_not_the_mean(self):
        labels = np.array([[1] * 10 + [2]])
        forward = np.array([[0.] * 9 + [10., .2]])
        lateral = np.array([[0.] * 10 + [2.]])
        actual = self.assert_equivalent(labels, forward, lateral)
        np.testing.assert_array_equal(actual, labels == 1)

    def test_depth_threshold_including_adjacent_floats(self):
        for value in [np.nextafter(.30, -np.inf), .30, np.nextafter(.30, np.inf)]:
            # The seed's 10th percentile is zero, but its box spans [0, 1],
            # so only the depth difference, not the spatial gap, selects label 2.
            labels = np.array([[1] * 11 + [2]])
            forward = np.array([[0.] * 10 + [1., value]])
            lateral = np.zeros_like(forward)
            actual = self.assert_equivalent(labels, forward, lateral)
            self.assertEqual(bool(actual[0, -1]), value <= .30)

    def test_spatial_threshold_including_adjacent_floats(self):
        labels = np.array([[1, 2]])
        for dtype in [np.float32, np.float64]:
            center = dtype(.20)
            for value in [np.nextafter(center, dtype(-np.inf)), center,
                          np.nextafter(center, dtype(np.inf))]:
                with self.subTest(dtype=dtype, value=value):
                    self.assert_equivalent(labels, np.zeros((1, 2), dtype=dtype),
                                           np.array([[0., value]], dtype=dtype))
            # A diagonal 3-4-5 triangle exercises scalar norm rounding.
            for value in [np.nextafter(dtype(.16), dtype(-np.inf)), dtype(.16),
                          np.nextafter(dtype(.16), dtype(np.inf))]:
                self.assert_equivalent(labels, np.array([[0., .12]], dtype=dtype),
                                       np.array([[0., value]], dtype=dtype))

    def test_interpolation_uses_numpy_rounding_for_all_small_group_sizes(self):
        rng = np.random.default_rng(705)
        for dtype in [np.float32, np.float64]:
            for count in range(1, 32):
                values = rng.uniform(.1, 2., count).astype(dtype)
                percentile = float(np.percentile(values, 10))
                # Candidate is at the depth cutoff and inside the seed's extent
                # when possible; the reference remains authoritative at edges.
                for candidate in [np.nextafter(percentile + .30, -np.inf),
                                  percentile + .30,
                                  np.nextafter(percentile + .30, np.inf)]:
                    labels = np.array([[1] * count + [2]])
                    forward = np.array([list(values) + [candidate]], dtype=dtype)
                    self.assert_equivalent(labels, forward, np.zeros_like(forward))

    def test_clipped_sparse_labels_preserve_holes(self):
        labels = np.array([[0, 1007, 1007, 0, 90123],
                           [0, 0, 1007, 0, 90123]])
        forward = np.array([[np.nan, 1., 1.1, np.inf, 1.2],
                            [np.nan, np.inf, 1.15, np.nan, 1.2]])
        lateral = np.array([[np.nan, 0., .01, np.nan, .10],
                            [np.inf, np.nan, .05, np.nan, .11]])
        actual = self.assert_equivalent(labels, forward, lateral)
        self.assertFalse(actual[labels == 0].any())
        self.assertEqual(int(actual.sum()), 5)

    def test_accepted_fragments_do_not_expand_seed_bounds(self):
        labels = np.array([[1, 2, 3]])
        forward = np.ones((1, 3))
        lateral = np.array([[0., .19, .38]])
        actual = self.assert_equivalent(labels, forward, lateral)
        np.testing.assert_array_equal(actual, [[True, True, False]])

    def test_random_clustered_inputs_and_dtypes(self):
        rng = np.random.default_rng(706)
        for dtype in [np.float32, np.float64]:
            for _ in range(50):
                labels = rng.integers(0, 24, size=(20, 30))
                centers = rng.uniform(.1, 3., 24)
                forward = (centers[labels] + rng.normal(0., .08, labels.shape)).astype(dtype)
                lateral = rng.uniform(-1., 1., labels.shape).astype(dtype)
                self.assert_equivalent(labels, forward, lateral)

    def test_nonfinite_standalone_inputs_preserve_reference_behavior(self):
        labels = np.array([[1, 1, 2, 2, 3]])
        for bad in [np.nan, np.inf, -np.inf]:
            with warnings.catch_warnings():
                warnings.simplefilter('ignore', RuntimeWarning)
                self.assert_equivalent(labels, np.array([[1., 1., bad, 1., 1.1]]),
                                       np.array([[0., .1, .1, .2, .1]]))
                self.assert_equivalent(labels, np.array([[1., 1., 1., 1., 1.1]]),
                                       np.array([[0., .1, bad, .2, .1]]))

    def test_extreme_finite_interpolation_keeps_original_nan_ordering(self):
        high = np.finfo(np.float64).max
        labels = np.array([[1] + [2] * 11])
        forward = np.array([[1.] + [-high] * 2 + [high] * 9])
        lateral = np.zeros_like(forward)
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            self.assert_equivalent(labels, forward, lateral)

    def test_inputs_are_unchanged(self):
        labels = np.array([[3, 1, 1, 0, 2]])
        forward = np.array([[1.1, 1., 1.2, 9., 1.15]], dtype=np.float32)
        lateral = np.array([[.1, 0., .1, 9., .05]], dtype=np.float64)
        originals = [array.copy() for array in (labels, forward, lateral)]
        self.assert_equivalent(labels, forward, lateral)
        for actual, expected in zip((labels, forward, lateral), originals):
            np.testing.assert_array_equal(actual, expected)

    def test_report_fragmented_performance_without_timing_assertion(self):
        yy, xx = np.indices((60, 80))
        labels = (yy // 4) * 20 + xx // 4 + 1
        labels[(yy % 4 == 0) | (xx % 4 == 0)] = 0
        forward = np.ones((60, 80))
        lateral = (xx - 40) * .015
        self.assert_equivalent(labels, forward, lateral)
        timings = {}
        for name, function in [('reference', original_nearest_fragments),
                               ('grouped', nearest_fragments)]:
            samples = []
            for _ in range(5):
                started = time.perf_counter()
                function(labels, forward, lateral)
                samples.append((time.perf_counter() - started) * 1000)
            timings[name] = round(statistics.median(samples), 3)
        print('Depth fragment synthetic 80x60 / 300 labels median milliseconds:', timings)


if __name__ == '__main__':
    unittest.main()
