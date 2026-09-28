# 【内容标注】用途：导航相关已有回归：test_virtual_wall。
# 对应用户需求：R02 R03 R08 R11 R13 R22（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：与导航功能相关的离线回归；关联这些需求不表示整份测试最初都由本轮创建。
# 本次仅加注释；需求关联不是精确创建/提交记录。
import unittest
import numpy as np
from nav2.virtual_wall import boundary_edges, clip_near, render

class WallTests(unittest.TestCase):
 def test_adjacent_cells_have_no_internal_wall(self):
  self.assertEqual(len(boundary_edges([{'cells':[[0,0],[1,0]]}])),6)
  self.assertEqual(set(boundary_edges([[0,0,2,1]])),set(boundary_edges([{'cells':[[0,0],[1,0]]}])))
 def test_near_plane_clipping(self):
  pts=clip_near([np.array([0.,0.,-.2]),np.array([1.,0.,1.]),np.array([1.,1.,1.])])
  self.assertTrue(all(p[2]>=.0999999 for p in pts))
 def test_overlay_does_not_modify_source_or_draw_behind_camera(self):
  image=np.full((120,160,3),200,np.uint8)
  calibration={'camera_matrix':{'data':[100,0,80,0,100,60,0,0,1]},'image_width':160,'image_height':120,'distortion_coefficients':{'data':[0,0,0,0,0]}}
  grid={'resolution':.1,'origin':[0,0],'yaw':0};robot={'position':[0,0],'yaw':0}
  front=render(image,[[20,0,21,1]],grid,robot,(0,0,0),.5,0,.8,calibration)
  self.assertFalse(np.array_equal(front,image))
  self.assertTrue(np.all(image==200))
  grid['origin']=[-4,0]
  back=render(image,[[20,0,21,1]],grid,robot,(0,0,0),.5,0,.8,calibration)
  self.assertTrue(np.array_equal(back,image))
