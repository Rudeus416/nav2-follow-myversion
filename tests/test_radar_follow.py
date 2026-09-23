"""Offline checks for live radar matching, projection, and distance fallback."""

from __future__ import annotations

import threading
import unittest
from collections import OrderedDict
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

import control
from calibration_common import transform_matrix
from radar_follow import (RadarFrame, RadarManager, RadarSettings, ScaleKalman,
                          person_radar_range, project_points)


class FakeNode:
    def __init__(self):
        self.publishers = 1
        self.subscription = None

    def count_publishers(self, _topic):
        return self.publishers

    def create_subscription(self, _message_type, _topic, callback, _qos):
        self.subscription = SimpleNamespace(callback=callback)
        return self.subscription

    def create_publisher(self, _message_type, _topic, _qos):
        return SimpleNamespace(publish=lambda _message: None)

    def destroy_subscription(self, subscription):
        if self.subscription is subscription:
            self.subscription = None


class RadarFollowTests(unittest.TestCase):
    def test_nav_stale_measurement_holds_then_cancels_without_extending_deadline(self):
        from unittest.mock import Mock
        m = control.MotionManager.__new__(control.MotionManager)
        m.navigation = Mock()
        m.settings = control.FollowSettings()
        m.estop = False
        m.mode = 'auto'
        m.following = m.target_visible = True
        m.target_distance = 2.0
        m.tracking_seen_at = m.distance_seen_at = 10.0
        m.handle_nav_measurement_pause(11.3)
        m.navigation.hold_for_visual_update.assert_called_once()
        m.navigation.stop.assert_not_called()
        m.handle_nav_measurement_pause(12.01)
        m.navigation.stop.assert_called_once()
        m.navigation.reset_mock()
        m.target_visible = False
        m.handle_nav_measurement_pause(11.3)
        m.navigation.stop.assert_called_once()
        m.navigation.hold_for_visual_update.assert_not_called()

    def test_nav2_visual_timeout_is_independent_of_ordinary_follow(self):
        motion = control.MotionManager.__new__(control.MotionManager)
        motion.settings = control.FollowSettings()
        motion.navigation = None
        self.assertAlmostEqual(motion.target_timeout(), 0.6)
        motion.navigation = object()
        self.assertEqual(motion.target_timeout(), 1.2)
        self.assertLess(0.866, motion.target_timeout())
        self.assertGreater(1.201, motion.target_timeout())
        self.assertAlmostEqual(motion.command_timeout(), 0.6)

    def test_preview_requires_stop_and_fresh_target(self):
        import time
        from unittest.mock import Mock
        motion = control.MotionManager.__new__(control.MotionManager)
        motion.lock = threading.RLock()
        motion.navigation = Mock()
        motion.navigation.status.return_value = {'nav2_preview': {'state': 'pending'}}
        motion.settings = control.FollowSettings()
        motion.radar_reconfiguring = False
        motion.estop = False
        with self.assertRaisesRegex(Exception, '强制停止'):
            motion.preview_path()
        motion.navigation.preview_path.assert_not_called()
        motion.estop = True
        motion.target_visible = True
        motion.tracking_seen_at = time.monotonic()
        motion.nav_measurement = (3, 0, time.monotonic()-10)
        with self.assertRaisesRegex(Exception, '超时'):
            motion.preview_path()
        motion.navigation.preview_path.assert_not_called()
        motion.tracking_seen_at = time.monotonic() - 1.168
        motion.nav_measurement = (3, 0, time.monotonic() - 1.168)
        self.assertEqual(motion.preview_path()['state'], 'pending')
        motion.navigation.preview_path.assert_called_once_with(3, 0, motion.settings.follow_distance_m)
        self.assertTrue(motion.estop)

    def test_preview_diagnostics_distinguishes_visibility_missing_and_stale_data(self):
        motion = control.MotionManager.__new__(control.MotionManager)
        motion.settings = control.FollowSettings()
        motion.target_visible = True
        motion.tracking_seen_at = 9.9
        motion.nav_measurement = (1.89, 0, 7.9)
        d = motion.preview_target_diagnostics(10)
        self.assertEqual(len(d['problems']), 1)
        self.assertIn('距离/方位数据超时', d['problems'][0])
        self.assertEqual(d['last_distance_m'], 1.89)
        motion.nav_measurement = None
        self.assertIn('同帧', motion.preview_target_diagnostics(10)['problems'][0])
        motion.nav_measurement = (1.89, 0, 9.9)
        motion.target_visible = False
        self.assertIn('不可见', motion.preview_target_diagnostics(10)['problems'][0])
        motion.target_visible = True
        self.assertTrue(motion.preview_target_diagnostics(10)['ready'])
        motion.tracking_seen_at = 8.0
        motion.nav_measurement = (1.89, 0, 8.0)
        self.assertTrue(motion.preview_target_diagnostics(10)['ready'])
        self.assertEqual(motion.preview_target_diagnostics(10)['timeout'], 2.0)
        self.assertAlmostEqual(motion.command_timeout(), 0.6)
        d = motion.preview_target_diagnostics(10.001)
        self.assertFalse(d['ready'])
        self.assertEqual(len(d['problems']), 2)

    def test_projection_uses_optical_axes_horizontal_distance_and_distortion(self):
        k = np.array([[100., 0., 50.], [0., 100., 50.], [0., 0., 1.]])
        matrix = transform_matrix(0, 0, 0, 0, 0, 0)
        points = np.array([[2., 0., 0.], [2., -0.5, 0.], [-1., 0., 0.],
                           [2., 0., 10.]], dtype=np.float32)
        projected = project_points(points, matrix, k, np.zeros(5), 100, 100)
        np.testing.assert_allclose(projected, [[50, 50, 2], [75, 50, np.hypot(.5, 2)]])
        distorted = project_points(points[:2], matrix, k,
                                   np.array([.2, 0, 0, 0, 0]), 100, 100)
        self.assertGreater(distorted[1, 0], projected[1, 0])

    def test_box_mask_and_distance_cluster_reject_background(self):
        # The person mask is letterboxed to 100x100 from a 200x100 image.
        mask = np.zeros((100, 100), dtype=bool)
        mask[40:60, 45:55] = True
        points = np.array([[100., 50., 2.0], [101., 51., 2.1],
                           [100., 50., 2.2], [140., 50., 5.0]])
        distance, count = person_radar_range(points, np.array([60, 30, 150, 70]),
                                              mask, (200, 100))
        self.assertEqual(count, 3)
        self.assertAlmostEqual(distance, 2.1)
        self.assertEqual(person_radar_range(points, np.array([0, 0, 20, 20]),
                                            mask, (200, 100)), (None, 0))

    def test_nearest_stamp_has_no_matching_threshold(self):
        node = FakeNode()
        radar = RadarManager(node, RadarSettings(enabled=True, external_radar=True))
        radar.frames.append(RadarFrame(100, 0, np.empty((0, 8))))
        radar.frames.append(RadarFrame(300, 0, np.empty((0, 8))))
        self.assertEqual(radar.nearest(260).stamp_ns, 300)
        self.assertEqual(radar.nearest(10_000).stamp_ns, 300)
        node.publishers = 0
        self.assertIsNone(radar.nearest(260))
        radar.close()

    def test_extrinsic_adjustment_guard(self):
        node = FakeNode()
        radar = RadarManager(node, RadarSettings())
        with self.assertRaisesRegex(ValueError, "允许调整外参"):
            radar.configure(RadarSettings(yaw_deg=10))
        radar.configure(RadarSettings(yaw_deg=10, allow_extrinsic_adjustment=True))
        self.assertEqual(radar.settings.yaw_deg, 10)
        radar.close()

    def test_reapplying_settings_recovers_a_dead_process(self):
        node = FakeNode()
        node.publishers = 0
        settings = RadarSettings(enabled=True)
        radar = RadarManager(node, RadarSettings())
        radar.settings = settings
        radar.subscription = node.create_subscription(None, settings.radar_topic, None, None)
        old_subscription = radar.subscription
        radar.process = SimpleNamespace(poll=lambda: 1)
        radar.handle = object()
        radar.frames.append(RadarFrame(100, 0, np.empty((0, 8))))
        self.assertTrue(radar.needs_recovery())
        live_process = SimpleNamespace(poll=lambda: None)
        with patch("radar_follow.stop_launch") as stop, \
             patch("radar_follow.check_radar_network"), \
             patch.object(Path, "exists", return_value=True), \
             patch("radar_follow.prepare_radar_config", return_value=(Path("a"), Path("b"), 5)), \
             patch("radar_follow.subprocess.run", return_value=SimpleNamespace(returncode=0)), \
             patch("radar_follow.start_launch", return_value=(live_process, object())):
            radar.configure(settings)
            stop.assert_called_once()
            self.assertIsNot(radar.subscription, old_subscription)
            self.assertEqual(len(radar.frames), 0)
            self.assertFalse(radar.needs_recovery())
            radar.close()

    def test_radar_change_requires_force_stop_and_blocks_early_release(self):
        motion = control.MotionManager(FakeNode(), "/test_cmd_vel", control.FollowSettings())
        try:
            motion.set_estop(False)
            with self.assertRaisesRegex(Exception, "强制停止"):
                motion.begin_radar_configuration()
            motion.set_estop(True)
            motion.following = True
            motion.begin_radar_configuration()
            self.assertFalse(motion.following)
            with self.assertRaisesRegex(Exception, "尚未完成"):
                motion.set_estop(False)
            motion.end_radar_configuration()
            motion.set_estop(False)
        finally:
            motion.close()

    def test_scale_kalman_rejects_isolated_bad_ratio(self):
        scale = ScaleKalman()
        for _ in range(4):
            scale.update(2, 1, 3)
        before = scale.value
        scale.update(8, 1, 1)
        self.assertEqual(scale.value, before)
        self.assertAlmostEqual(scale.value, 2, delta=.1)

    def test_fusion_uses_radar_for_pid_then_scaled_visual_when_box_empty(self):
        calibration = SimpleNamespace(k=np.array([[100., 0., 50.], [0., 100., 50.], [0., 0., 1.]]),
                                      distortion=np.zeros(5), width=100, height=100)
        settings = control.FollowSettings(target_id=0)
        mask = np.ones((100, 100), dtype=bool)
        person = control.Person(0, .9, np.array([40., 40., 60., 60.]), mask,
                                (50., 50.), (50., 60.), None, "pending")
        frame1 = control.FrameRecord("epoch", 1, 1.0, np.zeros((100, 100, 3), dtype=np.uint8), 100)
        frame2 = control.FrameRecord("epoch", 2, 2.0, np.zeros((100, 100, 3), dtype=np.uint8), 200)
        radar_settings = RadarSettings(enabled=True, external_radar=True, tx_m=0)
        radar = SimpleNamespace(lock=threading.RLock(), settings=radar_settings,
                                nearest=lambda stamp: RadarFrame(
                                    100 if stamp == 100 else 200, 0,
                                    np.array([[2., 0., 0.]]) if stamp == 100
                                    else np.empty((0, 3))))
        used_by_pid = []
        motion = SimpleNamespace(lock=threading.RLock(), following=True, mode="auto", estop=False,
                                 navigation=None,
                                 update_measurement=lambda *_args: True)
        pid = SimpleNamespace(yaw=0., reset=lambda: None, reset_distance=lambda: None,
                              update_distance=lambda target, *_args: used_by_pid.append(target.distance) or .1)
        engine = control.ProcessingEngine.__new__(control.ProcessingEngine)
        engine.radar, engine.scale, engine.motion, engine.pid = radar, ScaleKalman(), motion, pid
        engine.lock = threading.RLock()
        engine._config_version = 1
        engine._latest_tracking_control_frame_id = 0
        engine._last_controlled_frame_id = 0
        engine._buffer_f = OrderedDict()
        engine.rejected_config_results = engine.rejected_stale_control_results = 0
        engine.fused_frames = 0
        engine._offer_draw = lambda _job: None
        for frame in (frame1, frame2):
            tracking = control.TrackingResult(frame, 1, settings, calibration, None,
                                              (person,), None, None, None, 0, True, 0, 0)
            depth = control.DepthResult(frame, 1, settings, np.ones((10, 10)), 0, 0)
            with patch.object(control, "measured_distance", return_value=(1.0, "mask-depth", (50, 50), (50, 60))):
                engine._fuse_and_control(tracking, depth)
        self.assertAlmostEqual(used_by_pid[0], 2.0)
        self.assertAlmostEqual(used_by_pid[1], engine.scale.value, places=5)
        self.assertEqual(engine._people[0].method, "visual-scaled")
        self.assertEqual(engine._radar_pair_delta_ms, 0)


if __name__ == "__main__":
    unittest.main()
