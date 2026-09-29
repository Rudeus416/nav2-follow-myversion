# 【内容标注 / R38】附加语义预算回归；不加载模型，不修改主深度源。
import unittest
from types import SimpleNamespace as NS
from nav2.semantic_budget import SemanticBudget


def frame(index,now,latency=.2,version=1):
    return NS(config_version=version,completed_at=now,
              frame=NS(frame_id=index,stream_epoch=1,source_at=now-latency))


class SemanticBudgetTests(unittest.TestCase):
    def test_regular_fresh_frames_allow_optional_work(self):
        b=SemanticBudget()
        for i in range(12):
            self.assertTrue(b.observe(frame(i,100+i*.2),100+i*.2)['allowed'])

    def test_log_cadence_pauses_before_motion_freshness_is_exhausted(self):
        b=SemanticBudget();b.observe(frame(1,100,.5),100)
        s=b.observe(frame(2,100.692,.574),100.692)
        self.assertTrue(s['paused'])
        self.assertEqual(s['projected_age'],1.266)
        self.assertLess(s['source_age'],1.2)

    def test_old_result_never_starts_optional_work(self):
        b=SemanticBudget();s=b.observe(frame(340,100,2.769),100)
        self.assertFalse(s['allowed'])
        self.assertEqual(s['source_age'],2.769)

    def test_silent_source_is_detected_by_polling_same_record(self):
        b=SemanticBudget();r=frame(1,100)
        self.assertTrue(b.observe(r,100)['allowed'])
        self.assertFalse(b.observe(r,100.7)['allowed'])

    def test_repeated_frame_does_not_satisfy_recovery(self):
        b=SemanticBudget();b.observe(frame(1,100,1.),100)
        good=frame(2,100.2,.2)
        for _ in range(20):b.observe(good,100.2)
        self.assertLessEqual(b.healthy_frames,1)
        self.assertTrue(b.paused)

    def test_recovery_requires_quiet_period_and_distinct_frames(self):
        b=SemanticBudget();b.observe(frame(1,100,1.),100)
        for i in range(1,15):
            t=100+i*.2
            self.assertTrue(b.observe(frame(i+1,t),t)['paused'])
        self.assertTrue(b.observe(frame(16,103),103)['allowed'])

    def test_new_pressure_restarts_recovery(self):
        b=SemanticBudget();b.observe(frame(1,100,1.),100)
        for i in range(1,5):b.observe(frame(i+1,100+i*.2),100+i*.2)
        self.assertGreater(b.healthy_frames,0)
        b.observe(frame(6,101,.9),101)
        self.assertEqual(b.healthy_frames,0)
        self.assertEqual(b.until,104.)

    def test_stream_change_cannot_inherit_recovery_count(self):
        b=SemanticBudget();b.observe(frame(1,100,1.),100)
        for i in range(1,5):b.observe(frame(i+1,100+i*.2),100+i*.2)
        b.observe(frame(1,101,version=2),101)
        self.assertLessEqual(b.healthy_frames,1)
        self.assertTrue(b.paused)

    def test_future_nan_and_out_of_order_are_not_permission(self):
        for latency in (-.1,float('nan')):
            self.assertFalse(SemanticBudget().observe(frame(1,100,latency),100)['allowed'])
        b=SemanticBudget();b.observe(frame(2,100),100)
        self.assertFalse(b.observe(frame(1,99.9),100)['allowed'])

    def test_fixture_without_producer_metadata_remains_supported(self):
        b=SemanticBudget();r=NS(config_version=1,frame=NS(frame_id=1,stream_epoch=1,source_at=100.))
        self.assertTrue(b.observe(r,100.2)['allowed'])
        self.assertFalse(b.observe(r,101.)['allowed'])


if __name__=='__main__':unittest.main()
