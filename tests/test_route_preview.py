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
