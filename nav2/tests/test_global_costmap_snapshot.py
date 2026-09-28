# 【内容标注】用途：全局代价地图快照接入的离线回归。
# 对应用户需求：R13、R22；全路线后端检查与局部网页图层隔离。
# 不启动 ROS 节点，不读取真实传感器，不发布地图或运动命令。
"""Global/local full maps, incremental updates and opt-in snapshot payloads."""
import json
import os
import threading
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
from obstacle_view import ObstacleView


def header(stamp=10, frame='odom'):
    return NS(frame_id=frame, stamp=NS(sec=stamp, nanosec=0))


def grid(width=4, height=4, fill=0):
    return NS(header=header(), data=[fill]*(width*height), info=NS(
        width=width, height=height, resolution=.05,
        origin=NS(position=NS(x=-1., y=-.3), orientation=NS(x=0., y=0., z=0., w=1.))))


def update(x=1, y=1, width=1, height=1, data=None, stamp=10, frame='odom'):
    return NS(header=header(stamp, frame), x=x, y=y, width=width, height=height,
              data=[100]*(width*height) if data is None else data)


class GlobalCostmapSnapshotTests(unittest.TestCase):
    def setUp(self):
        self.view = v = ObstacleView.__new__(ObstacleView)
        v.lock = threading.Lock(); v.layers = {}; v.errors = {}; v.pending = {}
        v.node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000)))
        v.grid(grid(2, 2))
        v.global_grid(grid())

    def test_global_is_opt_in_and_default_browser_payload_is_unchanged(self):
        v = self.view
        v.global_grid(grid(250, 400))
        small = v.snapshot()
        self.assertEqual(set(small['layers']), {'map'})
        self.assertLess(len(json.dumps(small)), 1000)
        full = v.snapshot(include_global=True)
        self.assertEqual(set(full['layers']), {'map', 'global_map'})
        self.assertEqual(len(full['layers']['global_map']['data']), 100000)

    def test_global_extent_origin_and_resolution_are_preserved(self):
        msg = grid(44, 170); msg.info.origin.position.y = -.35
        self.view.global_grid(msg)
        g = self.view.snapshot(include_global=True)['layers']['global_map']
        self.assertEqual((g['width'], g['height']), (44, 170))
        self.assertEqual(g['origin'], [-1., -.35])
        self.assertEqual(g['resolution'], .05)
        self.assertEqual(g['yaw'], 0.)

    def test_global_patch_does_not_touch_local_or_previous_snapshot(self):
        before = self.view.snapshot(include_global=True)
        self.view.update_global_grid(update(y=1, width=2, height=2, data=[25, 50, 75, 100]))
        after = self.view.snapshot(include_global=True)
        self.assertEqual(after['layers']['map']['data'], [0]*4)
        self.assertEqual(before['layers']['global_map']['data'], [0]*16)
        self.assertEqual(after['layers']['global_map']['data'][5:7], [25, 50])
        self.assertEqual(after['layers']['global_map']['data'][9:11], [75, 100])

    def test_local_patch_does_not_touch_global(self):
        self.view.update_grid(update(x=0, y=0))
        layers = self.view.snapshot(include_global=True)['layers']
        self.assertEqual(layers['map']['data'], [100, 0, 0, 0])
        self.assertEqual(layers['global_map']['data'], [0]*16)

    def test_replacing_full_global_map_keeps_previous_snapshot_immutable(self):
        before = self.view.snapshot(include_global=True)
        self.view.global_grid(grid(3, 3, fill=100))
        self.assertEqual(before['layers']['global_map']['data'], [0]*16)
        after = self.view.snapshot(include_global=True)
        self.assertEqual(after['layers']['global_map']['data'], [100]*9)
        self.assertEqual(after['layers']['map']['data'], [0]*4)

    def test_bad_global_full_map_invalidates_only_global_layer(self):
        for kind in ('size', 'data_size', 'frame', 'resolution', 'origin', 'cell'):
            self.setUp(); msg = grid()
            if kind == 'size': msg.info.width = 0
            if kind == 'data_size': msg.data.pop()
            if kind == 'frame': msg.header.frame_id = 'map'
            if kind == 'resolution': msg.info.resolution = float('inf')
            if kind == 'origin': msg.info.origin.position.x = float('nan')
            if kind == 'cell': msg.data[0] = 101
            self.view.global_grid(msg)
            snapshot = self.view.snapshot(include_global=True)
            self.assertNotIn('global_map', snapshot['layers'], kind)
            self.assertIn('global_map', snapshot['errors'], kind)
            self.assertIn('map', snapshot['layers'], kind)
            self.assertNotIn('global_map', self.view.snapshot()['errors'], kind)

    def test_bad_global_patch_invalidates_only_global_layer_until_full_map(self):
        for kwargs in ({'x':4}, {'x':-1}, {'y':-1}, {'width':0}, {'height':0},
                       {'data':[]}, {'frame':'map'}, {'data':[-2]}):
            self.setUp(); self.view.update_global_grid(update(**kwargs))
            snapshot = self.view.snapshot(include_global=True)
            self.assertNotIn('global_map', snapshot['layers'], kwargs)
            self.assertIn('global_map', snapshot['errors'], kwargs)
            self.assertIn('map', snapshot['layers'], kwargs)
            self.view.update_global_grid(update())
            self.assertNotIn('global_map', self.view.snapshot(include_global=True)['layers'])
            self.view.global_grid(grid())
            self.assertNotIn('global_map', self.view.snapshot(include_global=True)['errors'])

    def test_increment_without_full_map_cannot_create_validation_layer(self):
        self.view.layers.pop('global_map')
        self.view.update_global_grid(update())
        self.assertNotIn('global_map', self.view.snapshot(include_global=True)['layers'])

    def test_old_and_future_full_map_stamps_are_stale(self):
        for stamp in (1, 11):
            msg = grid(); msg.header.stamp.sec = stamp
            self.view.global_grid(msg)
            self.assertTrue(self.view.snapshot(include_global=True)['layers']['global_map']['stale'])
            self.assertFalse(self.view.snapshot()['layers']['map']['stale'])

    def test_future_increment_cannot_be_used_as_fresh_global_map(self):
        self.view.update_global_grid(update(stamp=11))
        self.assertTrue(self.view.snapshot(include_global=True)['layers']['global_map']['stale'])

    def test_late_increment_cannot_overwrite_newer_map_or_refresh_age(self):
        previous = self.view.layers['global_map']
        self.view.update_global_grid(update(stamp=9))
        self.assertIs(self.view.layers['global_map'], previous)
        self.assertEqual(previous['data'], [0]*16)

    def test_missing_global_messages_expire_independently(self):
        self.view.layers['global_map']['received'] = time.monotonic()-5.
        snapshot = self.view.snapshot(include_global=True)
        self.assertTrue(snapshot['layers']['global_map']['stale'])
        self.assertFalse(snapshot['layers']['map']['stale'])

    def test_zero_stamp_uses_receipt_age_and_still_expires(self):
        msg = grid(); msg.header.stamp.sec = 0
        self.view.global_grid(msg)
        self.assertFalse(self.view.snapshot(include_global=True)['layers']['global_map']['stale'])
        self.view.layers['global_map']['received'] -= 4.
        self.assertTrue(self.view.snapshot(include_global=True)['layers']['global_map']['stale'])

    def test_constructor_subscribes_global_only_for_map_modes(self):
        from nav_msgs.msg import OccupancyGrid
        from map_msgs.msg import OccupancyGridUpdate
        for mode in ('map', 'both', 'radar'):
            with self.subTest(mode=mode), patch.dict(os.environ, NAV2_OBSTACLE_MODE=mode), \
                    patch('tf2_ros.Buffer'), patch('tf2_ros.TransformListener'):
                node = Mock(); view = ObstacleView(node, '/radar')
                calls = {c.args[1]: c.args for c in node.create_subscription.call_args_list}
                self.assertEqual('/global_costmap/costmap' in calls, mode != 'radar')
                self.assertEqual('/global_costmap/costmap_updates' in calls, mode != 'radar')
                self.assertIs(calls['/local_costmap/costmap'][2].__func__, ObstacleView.grid)
                if mode != 'radar':
                    self.assertIs(calls['/global_costmap/costmap'][0], OccupancyGrid)
                    self.assertIs(calls['/global_costmap/costmap_updates'][0], OccupancyGridUpdate)
                    self.assertIs(calls['/global_costmap/costmap'][2].__func__, ObstacleView.global_grid)
                    self.assertIs(calls['/global_costmap/costmap_updates'][2].__func__, ObstacleView.update_global_grid)


if __name__ == '__main__':
    unittest.main()
