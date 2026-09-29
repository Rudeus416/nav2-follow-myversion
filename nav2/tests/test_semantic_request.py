# 【内容标注 / R38】语义模型加载前后均检查原始拍摄时间，过期不推理。
import unittest
from unittest.mock import patch
from nav2.semantic_request import RequestFreshness,SemanticRequestExpired


class SemanticRequestTests(unittest.TestCase):
    def test_expired_queued_request_is_rejected_before_model(self):
        guard=RequestFreshness()
        with patch('nav2.semantic_request.time.monotonic',return_value=103.):
            with self.assertRaises(SemanticRequestExpired):guard.begin(100.)

    def test_lazy_setup_cannot_spend_time_and_then_infer_expired_image(self):
        guard=RequestFreshness()
        with patch('nav2.semantic_request.time.monotonic',return_value=100.3):guard.begin(100.)
        with patch('nav2.semantic_request.time.monotonic',return_value=101.21):
            with self.assertRaises(SemanticRequestExpired):guard(None)

    def test_same_source_is_kept_through_callback(self):
        guard=RequestFreshness()
        with patch('nav2.semantic_request.time.monotonic',return_value=100.3):
            guard.begin(100.);guard(None)
        self.assertEqual(guard.source_at,100.)

    def test_next_fresh_job_recovers_without_recreating_model(self):
        guard=RequestFreshness()
        with patch('nav2.semantic_request.time.monotonic',return_value=103.):
            with self.assertRaises(SemanticRequestExpired):guard.begin(100.)
            guard.begin(102.8);guard(None)

    def test_invalid_or_missing_sources_do_not_infer(self):
        guard=RequestFreshness()
        with self.assertRaises(SemanticRequestExpired):guard(None)
        for at in (float('nan'),float('inf'),105.):
            with patch('nav2.semantic_request.time.monotonic',return_value=103.):
                with self.assertRaises(SemanticRequestExpired):guard.begin(at)


if __name__=='__main__':unittest.main()
