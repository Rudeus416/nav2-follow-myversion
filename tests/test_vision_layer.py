# 【内容标注】用途：导航相关已有回归：test_vision_layer。
# 对应用户需求：R02 R03 R08 R11 R13 R22（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：与导航功能相关的离线回归；关联这些需求不表示整份测试最初都由本轮创建。
# 本次仅加注释；需求关联不是精确创建/提交记录。
import time
import unittest
import numpy as np
from nav2.vision_layer import depth_points
from nav2_follow import Nav2Follower

class VisionLayerTests(unittest.TestCase):
 def calibration(self):
  return {'camera_matrix':{'data':[100.,0,80,0,100,60,0,0,1]},'image_width':160,'image_height':120,'distortion_coefficients':{'data':[0.,0,0,0,0]}}
 def test_near_surface_and_invalid_depth(self):
  pts=depth_points(np.ones((120,160)),(120,160,3),self.calibration(),.5,0,1)
  self.assertGreater(len(pts),0)
  np.testing.assert_allclose(pts[:,0],1)
  with self.assertRaises(ValueError):depth_points(np.full((120,160),np.nan),(120,160,3),self.calibration(),.5,0,1)
 def test_far_surface_does_not_mark_near_map(self):
  pts=depth_points(np.full((120,160),8),(120,160,3),self.calibration(),.5,0,1)
  self.assertEqual(len(pts),0)
 def test_disabled_isolated_and_enabled_requires_fresh_valid_data(self):
  n=Nav2Follower.__new__(Nav2Follower)
  n.vision_enabled=False;n.check_vision()
  n.vision_enabled=True;n.vision_error='等待视觉障碍数据';n.vision_at=0
  with self.assertRaises(RuntimeError):n.check_vision()
  n.vision_error='';n.vision_at=time.monotonic();n.check_vision()
  n.vision_at=time.monotonic()-2
  with self.assertRaises(RuntimeError):n.check_vision()
  n.vision_at=time.monotonic();n.vision_error='近距障碍'
  with self.assertRaises(RuntimeError):n.check_vision()
