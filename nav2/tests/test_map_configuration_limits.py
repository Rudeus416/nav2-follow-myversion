# 【内容标注】用途：复现实际走廊地图与当前导航配置的已知限制。
# 对应用户需求：R13 R14 R20 R22（见 nav2/CODE_GUIDE.md）。
# 添加逻辑：读取真实地图/轮廓/起点，不修改生产配置、不放宽地图边界。
"""Known configuration limits; passing these tests does NOT mean they are fixed.

No ROS nodes or motion commands are created. Tests intentionally assert today's
too-small start clearance. The expanded global window must cover the real map;
the local window remains small and must not be used to validate a distant goal.
"""
import ast
import math
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

import cv2
import numpy as np
import yaml

from nav2.path_clearance import validate_path
from nav2.map_window import configure_global_map_window


ROOT = Path(__file__).resolve().parents[1]


def route(*poses):
    return NS(poses=[NS(pose=NS(
        position=NS(x=x, y=y), orientation=NS(
            x=0., y=0., z=math.sin(yaw / 2), w=math.cos(yaw / 2))))
        for x, y, yaw in poses])


def initial_map_pose():
    """Read literal launch arguments without executing a launch description."""
    tree = ast.parse((ROOT / 'follow.launch.py').read_text())
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        keywords = {kw.arg: kw.value for kw in node.keywords}
        name = keywords.get('name')
        if not isinstance(name, ast.Constant) or name.value != 'visual_car_map_origin':
            continue
        arguments = keywords['arguments'].elts
        values = {}
        for index, value in enumerate(arguments[:-1]):
            if isinstance(value, ast.Constant) and value.value in ('--x', '--y', '--yaw'):
                values[value.value] = arguments[index + 1]
        # Current yaw is the documented str(math.pi/2); reject changed expressions
        # instead of running arbitrary launch code or silently using an old angle.
        expected_yaw = ast.parse('str(math.pi/2)', mode='eval').body
        if ast.dump(values['--yaw']) != ast.dump(expected_yaw):
            raise AssertionError('地图起点朝向表达式已改变，请更新已知限制回归')
        return (float(ast.literal_eval(values['--x'])),
                float(ast.literal_eval(values['--y'])), math.pi / 2)
    raise AssertionError('没有找到地图起点静态变换')


class MapConfigurationLimits(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        metadata = yaml.safe_load((ROOT / 'maps/corridor.yaml').read_text())
        image = cv2.imread(str(ROOT / 'maps' / metadata['image']), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise AssertionError('实际走廊地图读取失败')
        if metadata['mode'] != 'trinary':
            raise AssertionError('此回归按当前 trinary 地图解释像素')
        occupancy = np.flipud(image).astype(float) / 255.
        if not metadata['negate']:
            occupancy = 1. - occupancy
        cells = np.full(image.shape, -1, dtype=np.int8)
        cells[occupancy < metadata['free_thresh']] = 0
        cells[occupancy > metadata['occupied_thresh']] = 100
        cls.static = dict(width=image.shape[1], height=image.shape[0],
                          resolution=metadata['resolution'],
                          origin=metadata['origin'][:2], yaw=metadata['origin'][2],
                          data=cells.ravel(), stale=False)
        config = yaml.safe_load((ROOT / 'params.yaml').read_text())
        configure_global_map_window(config, 'map', ROOT / 'maps/corridor.yaml')
        cls.costmaps = {name: config[name][name]['ros__parameters']
                        for name in ('local_costmap', 'global_costmap')}
        local = cls.costmaps['local_costmap']
        corners = np.asarray(yaml.safe_load(local['footprint']), dtype=float)
        # Nav2 pads this rectangular footprint by adding signed padding per axis.
        cls.footprint = corners + np.sign(corners) * local['footprint_padding']
        cls.start = initial_map_pose()

    def test_standard_start_passes_but_has_only_three_mm_rear_clearance(self):
        self.assertEqual((self.static['width'], self.static['height']), (44, 168))
        self.assertAlmostEqual(self.static['height'] * self.static['resolution'], 8.4)
        validate_path(route(self.start), self.static, self.footprint)
        rear = self.start[1] + self.footprint[:, 0].min() - self.static['origin'][1]
        self.assertAlmostEqual(rear, .003)

    def test_known_limit_one_degree_start_yaw_error_hits_map_boundary(self):
        x, y, yaw = self.start
        with self.assertRaisesRegex(ValueError, '车身超出已知地图边界'):
            validate_path(route((x, y, yaw + math.radians(1))), self.static, self.footprint)

    def test_known_limit_one_cm_backward_drift_hits_map_boundary(self):
        x, y, yaw = self.start
        with self.assertRaisesRegex(ValueError, '车身超出已知地图边界'):
            validate_path(route((x, y - .01, yaw)), self.static, self.footprint)

    def test_clear_far_route_fits_full_global_but_not_local_window(self):
        sx, sy, heading = self.start
        target = (sx, 7., heading)
        # The real static map admits the complete straight route and footprint.
        validate_path(route(self.start, target), self.static, self.footprint)
        dx, dy = target[0] - sx, target[1] - sy
        odom_target = (math.cos(heading) * dx + math.sin(heading) * dy,
                       -math.sin(heading) * dx + math.cos(heading) * dy, 0.)
        for name, costmap in self.costmaps.items():
            with self.subTest(costmap=name):
                self.assertTrue(costmap['rolling_window'])
                r = costmap['resolution']
                width, height = round(costmap['width'] / r), round(costmap['height'] / r)
                # Empty synthetic windows isolate range from detected obstacles:
                # the expanded global window admits the route, the local one does not.
                # Neither is published or used to extend the real known domain.
                window = dict(width=width, height=height, resolution=r, yaw=0.,
                              origin=[-costmap['width'] / 2, -costmap['height'] / 2],
                              data=np.zeros(width * height, dtype=np.int8), stale=False)
                validate_path(route((0., 0., 0.)), window, self.footprint)
                if name == 'local_costmap':
                    with self.assertRaisesRegex(ValueError, '车身超出已知地图边界'):
                        validate_path(route(odom_target), window, self.footprint)
                else:
                    validate_path(route((0., 0., 0.), odom_target), window, self.footprint)


if __name__ == '__main__':
    unittest.main()
