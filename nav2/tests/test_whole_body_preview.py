# 【内容标注】用途：导航离线回归：whole_body_preview。
# 对应用户需求：R13 R14（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明。
# 本次仅加注释；需求关联不是精确创建/提交记录。
"""Preview collision checks: no ROS, robot or service startup."""
import math
import unittest
from types import SimpleNamespace as NS
import numpy as np
from nav2.path_clearance import validate_path

FOOT=np.array([[.297,.232],[.297,-.232],[-.297,-.232],[-.297,.232]])
def route(*points):
    return NS(poses=[NS(pose=NS(position=NS(x=x,y=y),orientation=NS(w=math.cos(a/2),x=0,y=0,z=math.sin(a/2)))) for x,y,a in points])
def grid(cells):
    return dict(width=cells.shape[1],height=cells.shape[0],resolution=.05,origin=[0.,0.],yaw=0.,data=cells.ravel(),stale=False)
class WholeBodyPreview(unittest.TestCase):
    def test_zero_buffer_clear_narrow_corridor(self):
        cells=np.zeros((80,30));cells[:,5]=100;cells[:,18]=100
        validate_path(route((.6,.5,math.pi/2),(.6,3.5,math.pi/2)),grid(cells),FOOT)
    def test_purple_inside_body_is_forbidden_even_with_clear_center(self):
        for value in (1,98,100,-1):
            cells=np.zeros((40,40));cells[24,22]=value
            with self.subTest(cost=value),self.assertRaises(ValueError):
                validate_path(route((1.,1.,math.pi/2)),grid(cells),FOOT)
    def test_translation_cannot_skip_single_cell(self):
        cells=np.zeros((40,40));cells[20,20]=100
        with self.assertRaises(ValueError):validate_path(route((.5,1.,0),(1.6,1.,0)),grid(cells),FOOT)
    def test_turning_corner_sweep(self):
        cells=np.zeros((40,40));cells[25,25]=100;g=grid(cells)
        validate_path(route((1.,1.,0)),g,FOOT)
        validate_path(route((1.,1.,math.pi/2)),g,FOOT)
        with self.assertRaises(ValueError):validate_path(route((1.,1.,0),(1.,1.,math.pi/2)),g,FOOT)
    def test_rotated_map_preserves_clearance(self):
        cells=np.zeros((40,40));cells[20,20]=100;g=grid(cells);g['yaw']=math.pi/2
        with self.assertRaises(ValueError):validate_path(route((-1.,1.,math.pi/2)),g,FOOT)
    def test_missing_or_old_grid_rejected(self):
        for g in (None,{'stale':True}):
            with self.assertRaises(ValueError):validate_path(route((1.,1.,0)),g,FOOT)
if __name__=='__main__':unittest.main()
