# 【内容标注】用途：导航相关已有回归：test_nav2_vision_pause。
# 对应用户需求：R02 R03 R08 R11 R13 R22（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：与导航功能相关的离线回归；关联这些需求不表示整份测试最初都由本轮创建。
# 本次仅加注释；需求关联不是精确创建/提交记录。
import threading
import unittest
from unittest.mock import Mock, patch
from nav2_follow import Nav2Follower, VisionStale

class VisionPauseTests(unittest.TestCase):
    def nav(self):
        n=Nav2Follower.__new__(Nav2Follower)
        n.lock=threading.RLock();n.fixed_goal_active=True;n.vision_at=10.
        n.command_at=11.4;n.command=(.1,.1);n.enabled=True
        n.stop=Mock();n.healthy_pose=Mock()
        return n

    def test_brief_stale_zeroes_and_keeps_action(self):
        n=self.nav()
        with patch('nav2_follow.time.monotonic',return_value=11.5):n.pause_for_vision()
        self.assertEqual(n.command,(0.,0.));n.stop.assert_not_called()
        n.healthy_pose.assert_called_once_with(check_vision=False)

    def test_deadline_does_not_slide(self):
        n=self.nav()
        for now in (11.3,11.9,12.1):
            with patch('nav2_follow.time.monotonic',return_value=now):n.pause_for_vision()
        n.stop.assert_called_once();self.assertEqual(n.vision_at,10.)

    def test_recovery_discards_cached_command(self):
        n=self.nav();n.vision_paused=True
        self.assertEqual(n.velocity(),(0.,0.));self.assertEqual(n.command_at,0.)
        self.assertFalse(n.vision_paused)

    def test_other_sensor_failure_cancels(self):
        n=self.nav();n.healthy_pose.side_effect=RuntimeError('TF失效')
        n.pause_for_vision();n.stop.assert_called_once_with('TF失效')
