# 【内容标注 / R39】深度输入限额接入回归：不可变任务、原入口语义及关闭并发，不加载模型/ROS。
from dataclasses import dataclass, replace
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from nav2.depth_profile import DepthProfile


@dataclass(frozen=True)
class Settings:
    depth_imgsz: int = 768
    depth_fps: float = 5.
    segment_imgsz: int = 640
    process_fps: float = 15.

    def model_copy(self, *, update):
        return replace(self, **update)


@dataclass(frozen=True)
class Frame:
    image: object
    stream_epoch: str = 'camera-a'
    frame_id: int = 12
    source_at: float = 23.456
    stamp_ns: int = 987654321


@dataclass(frozen=True)
class Job:
    frame: Frame
    config_version: int
    settings: Settings


def job(size=768):
    return Job(Frame(object()), 7, Settings(depth_imgsz=size))


class DepthProfileTests(unittest.TestCase):
    def install(self, enabled=True, limit=512, result=True):
        original = Mock(return_value=result)
        worker = SimpleNamespace(offer=original)
        toggle = [enabled]
        profile = DepthProfile(worker, lambda: toggle[0], limit)
        self.addCleanup(profile.close)
        return worker, original, profile, toggle

    def test_enabled_cap_copies_only_settings_and_keeps_frame_identity(self):
        worker, original, profile, _ = self.install()
        source = job()
        self.assertTrue(worker.offer(source))
        submitted = original.call_args.args[0]
        self.assertIsNot(submitted, source)
        self.assertIs(submitted.frame, source.frame)
        self.assertIs(submitted.frame.image, source.frame.image)
        self.assertEqual(submitted.config_version, source.config_version)
        self.assertEqual(submitted.frame.source_at, 23.456)
        self.assertEqual(submitted.frame.stamp_ns, 987654321)
        self.assertEqual(submitted.frame.stream_epoch, 'camera-a')
        self.assertEqual(submitted.frame.frame_id, 12)
        self.assertIsNot(submitted.settings, source.settings)
        self.assertEqual(submitted.settings, replace(source.settings, depth_imgsz=512))
        self.assertEqual(source.settings.depth_imgsz, 768)
        self.assertEqual(profile.status()['effective_imgsz'], 512)
        original.assert_called_once()

    def test_disabled_passes_exact_original_job(self):
        worker, original, profile, _ = self.install(False)
        source = job()
        worker.offer(source)
        original.assert_called_once_with(source)
        self.assertIs(original.call_args.args[0], source)
        self.assertFalse(profile.status()['active'])
        self.assertEqual(profile.status()['effective_imgsz'], 768)

    def test_at_or_below_cap_does_not_raise_resolution_or_copy(self):
        for size in (320, 512):
            with self.subTest(size=size):
                worker, original, profile, _ = self.install()
                source = job(size)
                worker.offer(source)
                self.assertIs(original.call_args.args[0], source)
                self.assertEqual(profile.status()['adjusted'], 0)
                self.assertEqual(profile.status()['effective_imgsz'], size)

    def test_user_selected_640_cap_is_honoured(self):
        worker, original, profile, _ = self.install(limit=640)
        worker.offer(job())
        self.assertEqual(original.call_args.args[0].settings.depth_imgsz, 640)
        self.assertEqual(profile.status()['limit'], 640)

    def test_false_return_is_not_changed_or_counted_as_acceptance(self):
        worker, original, profile, _ = self.install(result=False)
        self.assertIs(worker.offer(job()), False)
        original.assert_called_once()
        self.assertEqual(profile.status()['offered'], 1)
        self.assertEqual(profile.status()['adjusted'], 1)
        self.assertNotIn('accepted', profile.status())

    def test_original_error_propagates_after_single_call(self):
        worker, original, profile, _ = self.install()
        failure = RuntimeError('original queue failure')
        original.side_effect = failure
        with self.assertRaises(RuntimeError) as caught:
            worker.offer(job())
        self.assertIs(caught.exception, failure)
        original.assert_called_once()
        self.assertEqual(profile.status()['offered'], 1)

    def test_toggle_changes_only_future_offers_and_keeps_queued_jobs(self):
        worker, original, profile, toggle = self.install()
        queued = []
        original.side_effect = lambda item: queued.append(item) or True
        source = job()
        worker.offer(source)
        toggle[0] = False
        worker.offer(source)
        toggle[0] = True
        worker.offer(source)
        self.assertEqual([item.settings.depth_imgsz for item in queued], [512, 768, 512])
        self.assertIs(queued[1], source)
        self.assertEqual(profile.status()['offered'], 3)
        self.assertEqual(profile.status()['adjusted'], 2)

    def test_close_restores_original_and_captured_wrapper_delegates(self):
        worker, original, profile, _ = self.install()
        captured = worker.offer
        profile.close()
        self.assertIs(worker.offer, original)
        source = job()
        self.assertTrue(captured(source))
        original.assert_called_once_with(source)
        self.assertIs(original.call_args.args[0], source)
        self.assertTrue(profile.status()['closed'])
        self.assertFalse(profile.status()['installed'])
        self.assertFalse(profile.status()['active'])
        profile.close()
        self.assertIs(worker.offer, original)

    def test_close_does_not_replace_later_owner(self):
        worker, original, profile, _ = self.install()
        captured = worker.offer
        later = Mock(side_effect=captured)
        worker.offer = later
        self.assertFalse(profile.status()['installed'])
        profile.close()
        self.assertIs(worker.offer, later)
        source = job()
        worker.offer(source)
        self.assertIs(original.call_args.args[0], source)
        later.assert_called_once()

    def test_missing_offer_reports_not_installed(self):
        worker = SimpleNamespace()
        profile = DepthProfile(worker, lambda: True)
        self.assertFalse(profile.status()['installed'])
        self.assertFalse(profile.status()['active'])
        self.assertIn('未提供 offer', profile.status()['message'])
        self.assertFalse(hasattr(worker, 'offer'))
        profile.close()
        self.assertFalse(hasattr(worker, 'offer'))

    def test_status_is_read_only_and_contains_no_frame_or_callback_work(self):
        enabled = Mock(return_value=True)
        worker = SimpleNamespace(offer=Mock(return_value=True))
        profile = DepthProfile(worker, enabled)
        self.addCleanup(profile.close)
        before = profile.status()
        self.assertTrue(before['installed'])
        self.assertEqual(before['offered'], 0)
        enabled.assert_not_called()
        worker.offer(job())
        enabled.assert_called_once()
        first = profile.status()
        self.assertEqual(first, profile.status())
        enabled.assert_called_once()
        self.assertEqual(first['requested_imgsz'], 768)
        self.assertEqual(first['effective_imgsz'], 512)
        first['limit'] = 320
        self.assertEqual(profile.status()['limit'], 512)
        for value in profile.status().values():
            self.assertIsInstance(value, (int, bool, str, type(None)))

    def test_callback_and_original_run_outside_private_lock(self):
        worker = SimpleNamespace(offer=Mock())
        profiles = []
        def enabled():
            self.assertTrue(profiles[0]._lock.acquire(blocking=False))
            profiles[0]._lock.release()
            profiles[0].status()
            return True
        profile = DepthProfile(worker, enabled)
        profiles.append(profile)
        self.addCleanup(profile.close)
        def original(item):
            self.assertTrue(profile._lock.acquire(blocking=False))
            profile._lock.release()
            return 'unchanged-return'
        profile._original.side_effect = original
        self.assertEqual(worker.offer(job()), 'unchanged-return')

    def test_close_while_enabled_callback_waits_preserves_original_job(self):
        entered, release = threading.Event(), threading.Event()
        original = Mock(return_value=True)
        worker = SimpleNamespace(offer=original)
        def enabled():
            entered.set()
            release.wait(2.)
            return True
        profile = DepthProfile(worker, enabled)
        self.addCleanup(profile.close)
        source = job()
        thread = threading.Thread(target=lambda: worker.offer(source), daemon=True)
        thread.start()
        try:
            self.assertTrue(entered.wait(1.))
            profile.close()
        finally:
            release.set()
            thread.join(2.)
        self.assertFalse(thread.is_alive())
        self.assertIs(original.call_args.args[0], source)
        original.assert_called_once()

    def test_callback_failure_preserves_original_path_and_reports_failure(self):
        original = Mock(return_value=True)
        worker = SimpleNamespace(offer=original)
        enabled = Mock(side_effect=RuntimeError('state unavailable'))
        profile = DepthProfile(worker, enabled)
        self.addCleanup(profile.close)
        source = job()
        self.assertTrue(worker.offer(source))
        self.assertIs(original.call_args.args[0], source)
        self.assertIn('state unavailable', profile.status()['error'])
        self.assertFalse(profile.status()['active'])
        original.assert_called_once()

    def test_unsupported_settings_preserve_job_and_report_unapplied(self):
        worker, original, profile, _ = self.install()
        source = SimpleNamespace(settings=SimpleNamespace(depth_imgsz=768))
        worker.offer(source)
        self.assertIs(original.call_args.args[0], source)
        self.assertFalse(profile.status()['active'])
        self.assertEqual(profile.status()['effective_imgsz'], 768)
        self.assertIn('未应用', profile.status()['message'])

    def test_mutable_job_type_is_not_silently_adapted(self):
        @dataclass
        class LegacyJob:
            frame: Frame
            config_version: int
            settings: Settings
        worker, original, profile, _ = self.install()
        source = LegacyJob(Frame(object()), 7, Settings())
        worker.offer(source)
        self.assertIs(original.call_args.args[0], source)
        self.assertFalse(profile.status()['active'])
        self.assertEqual(profile.status()['effective_imgsz'], 768)
        self.assertIn('不可变 dataclass', profile.status()['error'])

    def test_logs_only_first_offer_or_changed_effective_profile(self):
        worker, _, profile, toggle = self.install()
        with patch('nav2.depth_profile.logging.getLogger') as logger:
            worker.offer(job())
            worker.offer(job())
            logger.return_value.warning.assert_called_once()
            self.assertEqual(logger.return_value.warning.call_args.args[1:3], (768, 512))
            toggle[0] = False
            worker.offer(job())
            worker.offer(job())
            self.assertEqual(logger.return_value.warning.call_count, 2)
            self.assertEqual(logger.return_value.warning.call_args.args[1:3], (768, 768))
            toggle[0] = True
            worker.offer(job(640))
            self.assertEqual(logger.return_value.warning.call_count, 3)
            self.assertEqual(logger.return_value.warning.call_args.args[1:3], (640, 512))

    def test_same_fallback_error_logged_once_until_it_changes(self):
        worker = SimpleNamespace(offer=Mock(return_value=True))
        enabled = Mock(side_effect=RuntimeError('state unavailable'))
        profile = DepthProfile(worker, enabled)
        self.addCleanup(profile.close)
        with patch('nav2.depth_profile.logging.getLogger') as logger:
            worker.offer(job())
            worker.offer(job())
            logger.return_value.warning.assert_called_once()
            self.assertIn('state unavailable', logger.return_value.warning.call_args.args[-1])
            enabled.side_effect = RuntimeError('different failure')
            worker.offer(job())
            self.assertEqual(logger.return_value.warning.call_count, 2)

    def test_invalid_limits_rejected_before_installation(self):
        for limit in (True, False, '512', 512., None, 0, 319, 321, 513, 1281):
            with self.subTest(limit=limit):
                original = Mock()
                worker = SimpleNamespace(offer=original)
                with self.assertRaises(ValueError):
                    DepthProfile(worker, lambda: True, limit)
                self.assertIs(worker.offer, original)

    def test_boundary_limits_accepted(self):
        for limit in (320, 1280):
            with self.subTest(limit=limit):
                worker, _, profile, _ = self.install(limit=limit)
                self.assertEqual(profile.status()['limit'], limit)

    def test_noncallable_enabled_rejected_before_installation(self):
        original = Mock()
        worker = SimpleNamespace(offer=original)
        with self.assertRaises(TypeError):
            DepthProfile(worker, True)
        self.assertIs(worker.offer, original)


if __name__ == '__main__':
    unittest.main()
