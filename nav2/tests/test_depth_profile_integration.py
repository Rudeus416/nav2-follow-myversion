# 【内容标注 / R39】Nav2 深度 512 档位的安装、开关和退出集成；不启动模型或 ROS 节点。
from dataclasses import dataclass
import threading
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
from pydantic import BaseModel

from nav2.vision_layer import VisionLayer


class Settings(BaseModel):
    depth_imgsz: int = 768
    segment_imgsz: int = 640
    depth_fps: float = 5.


@dataclass(frozen=True)
class Job:
    frame: object
    config_version: int
    settings: Settings


class DepthProfileIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.offer = Mock(return_value=True)
        self.engine = NS(lock=threading.RLock(), _config_version=1,
                         _stream_epoch='camera', _stop=threading.Event(),
                         depth=NS(offer=self.offer, on_result=Mock()))
        self.nav = NS(vision_enabled=True, obstacle_mode='radar')
        self.node = Mock()
        with patch.dict('os.environ', {'NAV2_DEPTH_IMGSZ': '512'}), \
             patch('nav2.semantic_dispatch.SemanticDispatch'), \
             patch('nav2.vision_layer.threading.Thread') as thread:
            thread.return_value.is_alive.return_value = False
            self.layer = VisionLayer(self.node, self.engine, self.nav)
        self.addCleanup(self.layer.close)
        self.settings = Settings()
        self.frame = NS(source_at=123., stamp_ns=123000000000, frame_id=9,
                        stream_epoch='camera', image=object())
        self.job = Job(self.frame, 1, self.settings)

    def test_nav2_construction_installs_cap_without_touching_shared_settings(self):
        self.engine.depth.offer(self.job)
        submitted = self.offer.call_args.args[0]
        self.assertEqual(submitted.settings.depth_imgsz, 512)
        self.assertIs(submitted.frame, self.frame)
        self.assertEqual(submitted.config_version, 1)
        self.assertEqual(self.settings.depth_imgsz, 768)
        self.assertEqual(submitted.settings.segment_imgsz, 640)
        self.assertEqual(submitted.settings.depth_fps, 5.)
        self.assertTrue(self.layer.depth_profile.status()['installed'])

    def test_vision_toggle_changes_only_future_offers(self):
        self.engine.depth.offer(self.job)
        in_flight = self.offer.call_args.args[0]
        self.nav.vision_enabled = False
        self.engine.depth.offer(self.job)
        self.assertIs(self.offer.call_args.args[0], self.job)
        self.assertEqual(in_flight.settings.depth_imgsz, 512)
        self.nav.vision_enabled = True
        self.engine.depth.offer(self.job)
        self.assertEqual(self.offer.call_args.args[0].settings.depth_imgsz, 512)

    def test_invalid_environment_does_not_install_partial_hooks(self):
        offer = self.engine.depth.offer
        callback = self.engine.depth.on_result
        node = Mock()
        with patch.dict('os.environ', {'NAV2_DEPTH_IMGSZ': '313'}), \
             patch('nav2.semantic_dispatch.SemanticDispatch') as semantic:
            with self.assertRaises(ValueError):
                VisionLayer(node, self.engine, self.nav)
        semantic.assert_not_called()
        node.create_publisher.assert_not_called()
        self.assertIs(self.engine.depth.offer, offer)
        self.assertIs(self.engine.depth.on_result, callback)

    def failed_start(self, where):
        original = Mock(return_value=True)
        on_result = Mock()
        configure = Mock()
        engine = NS(lock=threading.RLock(), _config_version=1,
                    _stream_epoch='camera', depth=NS(offer=original, on_result=on_result),
                    configure=configure)
        nav = NS(vision_enabled=True, obstacle_mode='radar')
        node = Mock()
        if where == 'context':
            node.context.on_shutdown.side_effect = RuntimeError('startup failed')
        with patch.dict('os.environ', {'NAV2_DEPTH_IMGSZ': '512'}), \
             patch('nav2.semantic_dispatch.SemanticDispatch') as semantic, \
             patch('nav2.vision_layer.threading.Thread') as thread:
            if where == 'thread':
                thread.return_value.start.side_effect = RuntimeError('startup failed')
            with self.assertRaisesRegex(RuntimeError, 'startup failed'):
                VisionLayer(node, engine, nav)
            semantic.return_value.close.assert_called_once()
        self.assertIs(engine.depth.offer, original)
        self.assertIs(engine.depth.on_result, on_result)
        self.assertIs(engine.configure, configure)
        self.assertIsNone(nav.vision_layer)

    def test_thread_start_failure_restores_installed_entries(self):
        self.failed_start('thread')

    def test_shutdown_registration_failure_restores_installed_entries(self):
        self.failed_start('context')

    def test_layer_close_restores_original_offer(self):
        captured = self.engine.depth.offer
        self.layer.close()
        self.assertIs(self.engine.depth.offer, self.offer)
        captured(self.job)
        self.assertIs(self.offer.call_args.args[0], self.job)

    def test_engine_shutdown_also_restores_offer_without_explicit_layer_close(self):
        self.engine._stop.set()
        self.layer._run()
        self.assertIs(self.engine.depth.offer, self.offer)
        self.assertTrue(self.layer.depth_profile.status()['closed'])


if __name__ == '__main__':
    unittest.main()
