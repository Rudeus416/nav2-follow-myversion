# 【内容标注】用途：固化连续跟随 1 米阈值与实际到达容差之间的已知限制。
# 对应用户需求：R18、R22。此测试记录仍未修复行为，不是实车通过证明。
"""Known limitation: a stationary 1.10m target can cause repeated short segments.

This deliberately documents the current conflict rather than blessing it as the
intended behavior. It reads the production tolerance and supplies fake successful
FollowPath results when the unchanged robot already satisfies that tolerance.
No ROS nodes or robot commands are used. Replace this characterization when the
follow-specific arrival policy is fixed; do not loosen the 1m user requirement.
"""
import math
import unittest
from types import SimpleNamespace as NS
from pathlib import Path
from unittest.mock import Mock, patch
import yaml
from nav2.tests import test_person_navigation as fixtures


class ContinuousGoalToleranceKnownLimitationTests(unittest.TestCase):
    def test_known_limitation_stationary_person_at_1_10m_repeats_already_reached_goal(self):
        params = yaml.safe_load((Path(__file__).resolve().parents[1] / 'params.yaml').read_text())
        tolerance = params['controller_server']['ros__parameters']['goal_checker']['xy_goal_tolerance']
        fixture = fixtures.PersonNavigationTests(); fixture.setUp(); n = fixture.n
        n.continuous_follow = True; n.continuous_blocked = False; n.continuous_target_id = 10
        goals = []
        for segment in range(4):
            for stamp in (5.+segment*2, 5.3+segment*2, 5.6+segment*2):
                n.person_motion.nav_measurement = (1.1, 0., stamp)
                with patch('nav2_follow.time.monotonic', return_value=stamp):
                    n.update(1.1, 0., 1.)
            goals.append(n.goal)
            self.assertAlmostEqual(n.goal[0], .1)
            self.assertAlmostEqual(n.goal[1], 0.)
            self.assertLessEqual(math.hypot(*n.goal[:2]), tolerance)
            # Assume the path is valid and accepted. The pose remains (0,0,0);
            # its orientation already matches the goal, and XY is within the
            # real configured tolerance, so emulate immediate action success.
            n.pending = False; n.person_navigation.busy = False
            n.handle = Mock(); n.enabled = n.fixed_goal_active = True
            n.person_navigation.state = 'executing'
            with patch('nav2_follow.time.monotonic', return_value=5.8+segment*2):
                n._result(fixtures.done(NS(status=4)), n.handle)
        self.assertEqual(n.preview_client.send_goal_async.call_count, 4)
        self.assertEqual(goals, [goals[0]]*4)
        self.assertFalse(n.continuous_blocked)
        self.assertGreater(1.1, 1.)  # It never enters the requested <=1m wait region.


if __name__ == '__main__':
    unittest.main()
