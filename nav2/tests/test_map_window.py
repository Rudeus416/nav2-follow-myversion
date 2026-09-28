# 【内容标注】用途：全局规划窗口覆盖真实地图的离线回归。
# 对应用户需求：R01 R13 R20 R22。
# 添加逻辑：验证旋转地图、模式隔离、范围上限与坏输入；不启动 ROS 或发布地图。
"""Geometry-driven window sizing without changing map occupancy or local control."""
import copy
import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
import yaml
from PIL import Image

from nav2.map_window import configure_global_map_window

ROOT = Path(__file__).resolve().parents[1]


class MapWindowTests(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load((ROOT / 'params.yaml').read_text())
        self.original = copy.deepcopy(self.config)
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        Image.new('L', (44, 168), 255).save(self.directory / 'test.png')
        self.map_file = self.directory / 'test.yaml'
        self.metadata = {'image': 'test.png', 'resolution': .05, 'origin': [-.05, 0., 0.]}
        self.write_map()

    def write_map(self):
        self.map_file.write_text(yaml.safe_dump(self.metadata))

    def global_settings(self):
        return self.config['global_costmap']['global_costmap']['ros__parameters']

    def assert_only_global_size_and_publication_changed(self):
        expected = copy.deepcopy(self.original)
        target = expected['global_costmap']['global_costmap']['ros__parameters']
        for key in ('width', 'height', 'always_send_full_costmap'):
            target[key] = self.global_settings()[key]
        self.assertEqual(self.config, expected)

    def test_actual_corridor_map_and_both_expand_to_eighteen_by_ten(self):
        for mode in ('map', 'both'):
            with self.subTest(mode=mode):
                self.config = copy.deepcopy(self.original)
                result = configure_global_map_window(self.config, mode, ROOT / 'maps/corridor.yaml')
                self.assertEqual((result['width'], result['height']), (18, 10))
                self.assertAlmostEqual(result['span_x'], 8.4)
                self.assertAlmostEqual(result['span_y'], 2.2)
                self.assertIs(self.global_settings()['always_send_full_costmap'], True)
                self.assert_only_global_size_and_publication_changed()

    def test_rotated_map_corners_fit_from_any_corner_even_after_origin_rounding(self):
        # A corner center is more demanding than any physically valid car center
        # inside the map. Check all center/target corner pairs at several yaws.
        for angle in (0., math.pi / 4, math.pi / 2, -math.pi / 3):
            with self.subTest(map_yaw=angle):
                self.metadata['origin'] = [12., -7., angle]
                self.write_map()
                self.config = copy.deepcopy(self.original)
                result = configure_global_map_window(self.config, 'map', self.map_file)
                yaw = angle - math.pi / 2
                c, s = math.cos(yaw), math.sin(yaw)
                corners = np.array([[0., 0.], [2.2, 0.], [0., 8.4], [2.2, 8.4]])
                corners = corners @ np.array([[c, s], [-s, c]])
                half = np.array([result['width'], result['height']]) / 2
                for car in corners:
                    # One full global cell accounts for rolling origin rounding.
                    self.assertTrue(np.all(np.abs(corners - car) + .05 < half))
                self.assert_only_global_size_and_publication_changed()

    def test_radar_does_not_read_map_or_change_configuration(self):
        with patch('nav2.map_window.yaml.safe_load', side_effect=AssertionError('map read')):
            self.assertIsNone(configure_global_map_window(self.config, 'radar', '/missing/map.yaml'))
        self.assertEqual(self.config, self.original)

    def test_preserves_explicitly_larger_global_window(self):
        self.global_settings().update(width=24, height=14)
        configure_global_map_window(self.config, 'map', self.map_file)
        self.assertEqual((self.global_settings()['width'], self.global_settings()['height']), (24, 14))
        self.assertEqual(self.config['local_costmap'], self.original['local_costmap'])

    def test_invalid_map_metadata_fails_before_configuration_mutation(self):
        cases = [None, [], {'resolution': 0.}, {'resolution': float('nan')},
                 {'resolution': float('inf')}, {'resolution': True},
                 {'origin': [0., 0.]}, {'origin': [0., 0., float('nan')]},
                 {'image': None}, {'image': 'missing.png'}, {'resolution': 1e308}]
        for changes in cases:
            with self.subTest(metadata=changes):
                self.config = copy.deepcopy(self.original)
                metadata = dict(self.metadata, **changes) if isinstance(changes, dict) else changes
                self.map_file.write_text(yaml.safe_dump(metadata))
                with self.assertRaises(ValueError):
                    configure_global_map_window(self.config, 'map', self.map_file)
                self.assertEqual(self.config, self.original)

    def test_missing_or_unreadable_map_fails_clearly(self):
        for content in (None, 'origin: ['):
            with self.subTest(content=content):
                path = self.directory / 'invalid.yaml'
                if content is not None:
                    path.write_text(content)
                with self.assertRaisesRegex(ValueError, '无法读取地图配置'):
                    configure_global_map_window(self.config, 'map', path)

    def test_image_and_global_cell_limits_reject_without_partial_configuration(self):
        Image.new('L', (1001, 1000), 255).save(self.directory / 'test.png')
        with self.assertRaisesRegex(ValueError, '最多支持'):
            configure_global_map_window(self.config, 'map', self.map_file)
        self.assertEqual(self.config, self.original)
        Image.new('L', (44, 168), 255).save(self.directory / 'test.png')
        self.metadata['resolution'] = 1.
        self.write_map()
        with self.assertRaisesRegex(ValueError, '超过.*格上限'):
            configure_global_map_window(self.config, 'map', self.map_file)
        self.assertEqual(self.config, self.original)

    def test_unsupported_global_geometry_is_rejected(self):
        for override in ({'global_frame': 'map'}, {'rolling_window': False},
                         {'resolution': 0.}, {'width': 10.5}, {'height': -1}):
            with self.subTest(override=override):
                self.config = copy.deepcopy(self.original)
                self.global_settings().update(override)
                before = copy.deepcopy(self.config)
                with self.assertRaises(ValueError):
                    configure_global_map_window(self.config, 'map', self.map_file)
                self.assertEqual(self.config, before)


if __name__ == '__main__':
    unittest.main()
