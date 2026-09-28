# 【内容标注】用途：导航离线回归：safety_config。
# 对应用户需求：R13 R14（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明。
# 本次仅加注释；需求关联不是精确创建/提交记录。
"""Launch composition regression; never starts ROS processes."""
import importlib.util
import math
import os
import unittest
from pathlib import Path
from unittest.mock import patch
import yaml
from launch import LaunchContext

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('nav2_follow_launch', ROOT / 'follow.launch.py')
launch_file = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_file)

class SafetyConfigTests(unittest.TestCase):
    def test_every_mode_keeps_hard_buffer_last_and_whole_body_plugins(self):
        original = yaml.safe_dump
        for mode in ('map', 'radar', 'both'):
            for vision in ('0', '1'):
                with self.subTest(mode=mode, vision=vision):
                    captured = []
                    def save(config, stream):
                        captured.append(config)
                        return original(config, stream)
                    context = LaunchContext()
                    context.launch_configurations.update(params_file=str(ROOT / 'params.yaml'), publish_radar_tf='false')
                    with patch.dict(os.environ, NAV2_OBSTACLE_MODE=mode, NAV2_VISION_OBSTACLES=vision), patch.object(launch_file.yaml, 'safe_dump', side_effect=save):
                        launch_file.navigation_nodes(context)
                    for name in ('local_costmap', 'global_costmap'):
                        cfg = captured[0][name][name]['ros__parameters']
                        self.assertEqual(cfg['plugins'][-1], 'inflation')
                        self.assertNotIn('safety_inflation', cfg['plugins'])
                        self.assertEqual(cfg['inflation']['plugin'], 'visual_car_nav2::HardBuffer')
                        self.assertEqual(cfg['inflation']['inflation_radius'], 0.0)
                        self.assertEqual(captured[0]['planner_server']['ros__parameters']['GridBased']['plugin'], 'visual_car_nav2::Planner')
                        self.assertEqual(captured[0]['controller_server']['ros__parameters']['FollowPath']['critics'][0], 'WholeBody')
                        self.assertEqual(cfg['vision_layer']['enabled'], vision == '1')
                        self.assertEqual('map_boundary' in cfg['plugins'], mode == 'map')
                        expanded = name == 'global_costmap' and mode in ('map', 'both')
                        self.assertEqual((cfg['width'], cfg['height']), (18, 10) if expanded else (10, 10))
                        self.assertEqual(cfg.get('always_send_full_costmap', False), expanded)

if __name__ == '__main__':
    unittest.main()
