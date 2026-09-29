# 【内容标注】R40：验证几何加速不会把空点云、远处点或缓存当成运动授权。
# 对应用户“优化”；仅离线运行，不启动 ROS、相机、底盘或导航动作。
import unittest
from unittest.mock import patch

import numpy as np

from nav2.replan_safety import command_clear, path_clear, prepare_geometry


FOOT = np.asarray([[.297,.232],[.297,-.232],[-.297,-.232],[-.297,.232]])


class ReplanGeometryOptimizationTests(unittest.TestCase):
    def test_empty_and_provably_far_cloud_skip_swept_hulls(self):
        route = np.column_stack((np.linspace(0.,3.,301), np.zeros(301), np.zeros(301)))
        with patch('nav2.replan_safety._hull', side_effect=AssertionError('unnecessary hull')):
            self.assertTrue(path_clear(route, [], FOOT))
            self.assertTrue(path_clear(route, [[1.5, 2.]], FOOT))
            self.assertTrue(command_clear([[3.,3.]], FOOT, (0.,0.,0.), .18,.4,10.,10.8))

    def test_empty_cloud_still_validates_budget_path_and_footprint(self):
        self.assertFalse(path_clear([(0.,0.,0.),(1000.,0.,0.)], [], FOOT))
        self.assertFalse(path_clear([(0.,0.,0.),(1000.,0.,0.)], [[5.,5.]], FOOT))
        self.assertFalse(path_clear([], [], FOOT))
        self.assertFalse(path_clear([(0.,0.,float('nan'))], [], FOOT))
        self.assertFalse(path_clear([(0.,0.,0.)], [], [[0.,0.],[1.,0.],[2.,0.]]))
        self.assertFalse(command_clear([], FOOT, (0.,0.,0.), 100.,0.,10.,10.8))

    def test_far_cells_do_not_hide_near_cell_or_intermediate_swept_contact(self):
        self.assertFalse(path_clear([(0.,0.,0.),(3.,0.,0.)], [[1.5,0.],[1.5,9.]], FOOT))
        self.assertFalse(path_clear([(0.,0.,0.),(0.,0.,np.pi/2)], [[-.36,.1],[9.,9.]], FOOT))
        self.assertFalse(command_clear([[-.42,0.],[9.,9.]], FOOT, (0.,0.,0.), -.18,0.,10.,10.8))

    def test_preparation_is_detached_immutable_and_reuses_deduplication(self):
        points = np.asarray([[.42,0.],[9.,9.]])
        footprint = FOOT.copy()
        geometry = prepare_geometry(points, footprint)
        points[:] = 99.
        footprint[:] = .01
        for array in (geometry.centers, geometry.corners):
            with self.assertRaises(ValueError):
                array.setflags(write=True)
        with patch('nav2.replan_safety.np.unique', side_effect=AssertionError('repeated deduplication')):
            self.assertFalse(path_clear([(0.,0.,0.),(1.,0.,0.)], geometry, None))
            self.assertFalse(command_clear(geometry, None, (0.,0.,0.), .18,0.,10.,10.8))

    def test_preparation_does_not_cache_pose_command_age_or_path_decisions(self):
        geometry = prepare_geometry([[.42,0.]], FOOT)
        self.assertTrue(command_clear(geometry, None, (0.,0.,0.), 0.,0.,10.,10.8))
        self.assertFalse(command_clear(geometry, None, (0.,0.,0.), .18,0.,10.,10.8))
        self.assertFalse(command_clear(geometry, None, (.42,0.,0.), 0.,0.,10.,10.8))
        self.assertFalse(command_clear(geometry, None, (0.,0.,0.), 0.,0.,10.,11.201))
        self.assertTrue(path_clear([(0.,0.,0.)], geometry, None))
        self.assertFalse(path_clear([(0.,0.,0.),(1.,0.,0.)], geometry, None))
        self.assertFalse(path_clear([(0.,0.,0.)], geometry, FOOT))

    def test_preparation_rejects_invalid_snapshot_and_retains_complete_cells(self):
        for points, foot in (([[float('nan'),0.]], FOOT), ([], [[0.,0.],[1.,0.],[2.,0.]])):
            with self.assertRaises(ValueError):
                prepare_geometry(points, foot)
        geometry = prepare_geometry([[.1,.249]], FOOT)
        self.assertFalse(path_clear([(0.,0.,0.)], geometry, None))


if __name__ == '__main__':
    unittest.main()
