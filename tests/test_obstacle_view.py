import threading
import time
import unittest
from types import SimpleNamespace as NS
from obstacle_view import ObstacleView


class ObstacleViewTests(unittest.TestCase):
    def test_constructor_initializes_editor_and_ros_path_subscription(self):
        import os
        from pathlib import Path
        from unittest.mock import Mock, patch
        from nav_msgs.msg import Path as NavPath
        for mode in ('radar', 'map', 'both'):
            with self.subTest(mode=mode), patch.dict(os.environ, NAV2_OBSTACLE_MODE=mode), \
                    patch('tf2_ros.Buffer'), patch('tf2_ros.TransformListener'):
                node = Mock()
                view = ObstacleView(node, '/radar')
                if mode == 'radar':
                    self.assertIsNone(view.editor)
                else:
                    self.assertIsInstance(view.editor.directory, Path)
                subscriptions = node.create_subscription.call_args_list
                self.assertEqual(any(c.args[1] == '/map' for c in subscriptions), mode != 'radar')
                path_call = next(c for c in subscriptions if c.args[1] == '/plan')
                self.assertIs(path_call.args[0], NavPath)

    def test_static_map_and_robot_share_odom_coordinates(self):
        from unittest.mock import Mock
        self.msg.header = NS(frame_id='map', stamp=NS(sec=1, nanosec=0))
        transform = NS(header=self.header, transform=NS(
            translation=NS(x=2., y=3., z=0.), rotation=NS(x=0., y=0., z=0., w=1.)))
        self.view.tf = NS(lookup_transform=Mock(return_value=transform))
        self.view.static_grid(self.msg)
        layers = self.view.snapshot()['layers']
        self.assertEqual(layers['static_map']['origin'], [1., 5.])
        self.assertFalse(layers['static_map']['stale'])
        self.assertEqual(layers['robot']['position'], [2., 3.])
        self.assertFalse(layers['robot']['stale'])

    def setUp(self):
        self.view = ObstacleView.__new__(ObstacleView)
        self.view.lock = threading.Lock()
        self.view.layers = {}
        self.view.errors = {}
        self.view.pending = {}
        self.view.node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000)))
        self.header = NS(frame_id='odom', stamp=NS(sec=10, nanosec=0))
        self.msg = NS(header=self.header, data=[0, 0, 0, 100, -1, 0], info=NS(
            width=3, height=2, resolution=0.05,
            origin=NS(position=NS(x=-1.0, y=2.0), orientation=NS(x=0,y=0,z=0,w=1))))

    def test_partial_update_preserves_other_cells_and_prior_snapshot(self):
        self.view.grid(self.msg)
        before = self.view.snapshot()
        self.view.update_grid(NS(header=self.header, x=1, y=0, width=1, height=2, data=[50, 75]))
        self.assertEqual(self.view.snapshot()['layers']['map']['data'], [0,50,0,100,75,0])
        self.assertEqual(before['layers']['map']['data'], [0,0,0,100,-1,0])

    def test_incompatible_update_invalidates_grid(self):
        self.view.grid(self.msg)
        self.view.update_grid(NS(header=self.header, x=3, y=0, width=1, height=1, data=[50]))
        self.assertNotIn('map', self.view.snapshot()['layers'])

    def test_wrong_frame_is_not_drawn_as_odom(self):
        self.msg.header.frame_id = 'map'
        self.view.grid(self.msg)
        self.assertNotIn('map', self.view.snapshot()['layers'])
        self.assertIn('map', self.view.snapshot()['errors'])

    def test_old_sensor_stamp_remains_stale_on_receipt(self):
        self.msg.header.stamp.sec = 1
        self.view.grid(self.msg)
        self.assertTrue(self.view.snapshot()['layers']['map']['stale'])

    def test_missing_updates_become_stale(self):
        self.view.grid(self.msg)
        self.view.layers['map']['received'] = time.monotonic()-5
        self.assertTrue(self.view.snapshot()['layers']['map']['stale'])

    def test_zero_timestamp_uses_receipt_and_still_expires(self):
        self.header.stamp.sec = 0
        self.view.grid(self.msg)
        self.assertFalse(self.view.snapshot()['layers']['map']['stale'])
        self.view.layers['map']['received'] -= 4
        self.assertTrue(self.view.snapshot()['layers']['map']['stale'])

    def test_zero_timestamp_increment_refreshes_map(self):
        self.view.grid(self.msg)
        self.header.stamp.sec = 0
        self.view.update_grid(NS(header=self.header, x=0, y=0, width=1, height=1, data=[10]))
        self.assertFalse(self.view.snapshot()['layers']['map']['stale'])

    def test_tf_retry_uses_original_stamp_and_receipt(self):
        from builtin_interfaces.msg import Time
        from tf2_ros import TransformException
        from unittest.mock import Mock
        self.header.frame_id = 'radar'
        self.header.stamp = Time(sec=10)
        transform = NS(transform=NS(rotation=NS(x=0,y=0,z=0,w=1), translation=NS(x=1,y=2,z=0)))
        self.view.tf = NS(lookup_transform=Mock(side_effect=[TransformException('future'), transform]))
        self.view.points('radar', self.header, [NS(x=2,y=0,z=0)])
        received = self.view.pending['radar'][2]
        self.view.retry_points()
        self.assertEqual(self.view.layers['radar']['points'], [[3,2]])
        self.assertEqual(self.view.layers['radar']['received'], received)
        self.assertEqual(self.view.tf.lookup_transform.call_args_list[0], self.view.tf.lookup_transform.call_args_list[1])
        self.assertFalse(self.view.pending)

    def test_tf_timeout_discards_untransformable_points(self):
        from builtin_interfaces.msg import Time
        from tf2_ros import TransformException
        from unittest.mock import Mock
        self.header.frame_id = 'radar'
        self.header.stamp = Time(sec=10)
        self.view.tf = NS(lookup_transform=Mock(side_effect=TransformException('missing TF')))
        self.view.points('radar', self.header, [NS(x=2,y=0,z=0)], time.monotonic()-1)
        self.assertFalse(self.view.pending)
        self.assertNotIn('radar', self.view.layers)
        self.assertIn('missing TF', self.view.errors['radar'])
