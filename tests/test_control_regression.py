"""Exercise the real control loop offline; no ROS nodes or hardware commands."""
import builtins
import unittest
from unittest.mock import Mock, patch
import control


class ControlRegressionTests(unittest.TestCase):
    def motion(self, navigation=None):
        node = Mock()
        with patch('control.threading.Thread'):
            return control.MotionManager(node, '/cmd_vel', control.FollowSettings(), navigation)

    def tick(self, motion, now=10):
        motion._stop_event = Mock()
        motion._stop_event.wait.side_effect = [False, True]
        with patch('control.time.monotonic', side_effect=[now-.02, now-.01, now, now]):
            motion._loop()
        return motion.publisher.publish.call_args.args[0]

    def test_ordinary_follow_does_not_import_map_dependencies(self):
        real_import = builtins.__import__
        def checked(name, *args, **kwargs):
            if name in ('obstacle_view', 'nav2_map_edit', 'map_msgs.msg'):
                raise AssertionError('Optional map dependency imported')
            return real_import(name, *args, **kwargs)
        with patch('builtins.__import__', side_effect=checked):
            self.assertIsNone(control.create_obstacle_view(None, Mock(), '/radar'))

    def test_original_follow_velocity_and_timeout_unchanged(self):
        m = self.motion()
        m.estop = False
        m.following = m.target_visible = True
        m.target_distance = 2
        m.tracking_seen_at = m.distance_seen_at = 9.8
        m.auto_linear, m.auto_yaw = .4, .2
        out = self.tick(m)
        self.assertEqual((out.linear.x, out.angular.z), (.4, .2))
        m.tracking_seen_at = m.distance_seen_at = 9.3
        out = self.tick(m)
        self.assertEqual((out.linear.x, out.angular.z), (0, 0))

    def test_manual_control_bypasses_nav2_and_retains_heartbeat_stop(self):
        for navigation in (None, Mock()):
            m = self.motion(navigation)
            m.estop = False
            m.mode = 'manual'
            m.manual_direction = 'up'
            m.manual_seen_at = 9.9
            out = self.tick(m)
            self.assertGreater(out.linear.x, 0)
            if navigation:
                navigation.velocity.assert_not_called()
            m.manual_seen_at = 9
            self.assertEqual(self.tick(m).linear.x, 0)

    def test_force_stop_wins_over_manual_and_auto(self):
        for mode in ('auto', 'manual'):
            m = self.motion(Mock())
            m.mode = mode
            m.following = m.target_visible = True
            m.target_distance = 2
            m.manual_direction = 'up'
            m.manual_seen_at = m.tracking_seen_at = m.distance_seen_at = 10
            out = self.tick(m)
            self.assertEqual((out.linear.x, out.angular.z), (0, 0))
            m.navigation.velocity.assert_not_called()

    def test_nav2_short_timeout_zeroes_velocity_without_cancelling(self):
        n = Mock()
        m = self.motion(n)
        m.estop = False
        m.following = m.target_visible = True
        m.target_distance = 2
        m.tracking_seen_at = m.distance_seen_at = 8.5
        out = self.tick(m)
        self.assertEqual((out.linear.x, out.angular.z), (0, 0))
        n.hold_for_visual_update.assert_called_once()
        n.stop.assert_not_called()
        m.tracking_seen_at = m.distance_seen_at = 7.9
        self.tick(m)
        n.stop.assert_called_once()

    def test_fixed_goal_moves_without_person_and_estop_wins(self):
        n=Mock();n.fixed_goal_active=True;n.velocity.return_value=(.1,.1)
        m=self.motion(n);m.estop=False;m.mode='auto';m.following=False
        m.update_tracking(False,0.,9.,1)
        m.update_measurement(False,None,0.,0.,9.,1)
        n.stop.assert_not_called()
        self.assertEqual(self.tick(m).linear.x,.1)
        m.estop=True
        self.assertEqual(self.tick(m).linear.x,0.)
