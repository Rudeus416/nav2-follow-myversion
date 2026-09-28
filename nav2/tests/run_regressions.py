#!/usr/bin/env python3
# 【内容标注 / R22 R25】集中运行自有导航回归；不启动底盘、相机或实车 ROS 动作。
# 需要先加载 ROS 消息环境；真实插件/图层测试另见 whole_body/build.sh。
"""Run navigation-only offline tests from any working directory."""
import sys
import unittest
from pathlib import Path


def main():
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root))
    suite = unittest.TestSuite()
    # 每次使用独立 loader，避免两个未声明为包的 tests 目录发生顶层目录冲突。
    suite.addTests(unittest.TestLoader().discover(str(root / 'nav2/tests'), pattern='test_*.py'))
    for name in (
        'test_nav2_follow.py', 'test_nav2_map_edit.py', 'test_nav2_wall_merge.py',
        'test_nav2_vision_pause.py', 'test_obstacle_view.py', 'test_route_preview.py',
        'test_vision_layer.py', 'test_virtual_wall.py',
    ):
        suite.addTests(unittest.TestLoader().discover(str(root / 'tests'), pattern=name))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == '__main__':
    raise SystemExit(main())
