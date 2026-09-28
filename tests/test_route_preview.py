# 【内容标注】用途：导航相关已有回归：test_route_preview。
# 对应用户需求：R02 R03 R08 R11 R13 R22（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：与导航功能相关的离线回归；关联这些需求不表示整份测试最初都由本轮创建。
# 本次仅加注释；需求关联不是精确创建/提交记录。
import unittest
import numpy as np
from nav2.route_preview import candidates

class PreviewTests(unittest.TestCase):
 def test_candidate_depth_threshold_and_invalid_data(self):
  k=np.array([[100.,0,80],[0,100,60],[0,0,1]])
  for value,expected in [(1.,True),(8.,False),(float('nan'),False)]:
   mask,distance=candidates(np.full((120,160),value),k,np.zeros(5),(120,160,3),.5,0,1)
   self.assertEqual(bool(mask.any()),expected)
   self.assertEqual(distance is not None,expected)
 def test_scale_changes_estimated_range(self):
  k=np.array([[100.,0,80],[0,100,60],[0,0,1]])
  _,distance=candidates(np.ones((120,160)),k,np.zeros(5),(120,160,3),.5,0,2)
  self.assertAlmostEqual(distance,2.)
