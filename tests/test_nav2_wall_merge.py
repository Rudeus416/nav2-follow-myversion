import math
import unittest
from types import SimpleNamespace as NS
import numpy as np
from nav2.vision_layer import depth_points, merge_static_walls

class WallMergeTests(unittest.TestCase):
 def test_nearest_component_only(self):
  depth=np.full((60,80),8.,np.float32)
  depth[10:20,30:40]=.8
  depth[10:20,55:65]=2.2
  calibration={'camera_matrix':{'data':[100.,0,80,0,100,60,0,0,1]},'image_width':160,'image_height':120,'distortion_coefficients':{'data':[0.,0,0,0,0]}}
  pts=depth_points(depth,(120,160,3),calibration,.5,0,1)
  self.assertGreater(len(pts),0)
  np.testing.assert_allclose(pts[:,0],.8)
 def test_outside_nearest_does_not_hide_inside_candidate(self):
  depth=np.full((60,80),8.,np.float32)
  depth[10:20,30:40]=.8
  depth[10:20,55:65]=2.2
  calibration={'camera_matrix':{'data':[100.,0,80,0,100,60,0,0,1]},'image_width':160,'image_height':120,'distortion_coefficients':{'data':[0.,0,0,0,0]}}
  pts=depth_points(depth,(120,160,3),calibration,.5,0,1,accept=lambda xy:xy[:,0]>1)
  self.assertGreater(len(pts),0)
  np.testing.assert_allclose(pts[:,0],2.2)
  safety=depth_points(depth,(120,160,3),calibration,.5,0,1,nearest_only=False)
  self.assertTrue(np.any(safety[:,0]<1))
 def test_rotated_map_bounds(self):
  from nav2.vision_layer import inside_static_map
  grid=NS(info=NS(width=2,height=4,resolution=1.,origin=NS(position=NS(x=0,y=0),orientation=NS(x=0,y=0,z=0,w=1))))
  tf=NS(translation=NS(x=10,y=20),rotation=NS(x=0,y=0,z=math.sqrt(.5),w=math.sqrt(.5)))
  actual=inside_static_map(np.array([[9,21],[11,21],[9,23],[5,21]]),grid,tf)
  np.testing.assert_array_equal(actual,[True,False,False,False])
 def test_rotated_corridor_walls_preserved_even_when_vision_empty(self):
  grid=NS(info=NS(width=44,height=84,resolution=.05,origin=NS(position=NS(x=-.05,y=0),orientation=NS(x=0,y=0,z=0,w=1))),data=[])
  base=np.zeros((84,44),np.int8);base[:,0]=base[:,-1]=100;base[30:33,20:23]=100
  grid.data=base.ravel().tolist()
  tf=NS(translation=NS(x=-.3,y=1.2),rotation=NS(x=0,y=0,z=-math.sqrt(.5),w=math.sqrt(.5)))
  data=np.zeros((200,200),np.int8);data[5,5]=100
  merge_static_walls(data,(-5,-5),.05,grid,tf)
  for y,x in np.argwhere(base==100):
   wx=-.3+(y+.5)*.05;wy=1.2-(-.05+(x+.5)*.05)
   self.assertEqual(data[math.floor((wy+5)/.05),math.floor((wx+5)/.05)],100)
  self.assertEqual(data[5,5],100)
  self.assertEqual(data[100,100],0)
 def test_no_static_obstacles_does_not_erase_visual_cells(self):
  grid=NS(info=NS(width=2,height=2,resolution=.05,origin=NS(position=NS(x=0,y=0),orientation=NS(x=0,y=0,z=0,w=1))),data=[0]*4)
  tf=NS(translation=NS(x=0,y=0),rotation=NS(x=0,y=0,z=0,w=1))
  data=np.zeros((10,10),np.int8);data[2,2]=100
  np.testing.assert_array_equal(merge_static_walls(data.copy(),(0,0),.05,grid,tf),data)
