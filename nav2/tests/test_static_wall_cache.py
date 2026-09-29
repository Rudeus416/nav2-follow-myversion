# 【R35】墙体缓存必须与逐帧完整栅格化一致，不能残留视觉点或丢失编辑。
import copy
import math
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
from nav2.boundary_map import boundary_grid
from nav2.static_wall_cache import StaticWallCache
from nav2.vision_layer import merge_static_walls


class StaticWallCacheTests(unittest.TestCase):
    def setUp(self):
        data=np.zeros((168,44),np.int8);data[:,0]=data[:,-1]=100
        data[50:55,20:24]=100
        self.grid=NS(info=NS(width=44,height=168,resolution=.05,
            origin=NS(position=NS(x=-.05,y=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))),data=data.ravel().tolist())
        self.tf=NS(translation=NS(x=-.3,y=.6),rotation=NS(x=0.,y=0.,z=-math.sqrt(.5),w=math.sqrt(.5)))
        self.origin=(-5.,-5.);self.cache=StaticWallCache()

    def raster(self,boundary=True):
        return self.cache.raster((200,200),self.origin,.05,self.grid,self.tf,boundary)

    def expected(self,boundary=True):
        data=np.zeros((200,200),np.int8)
        merge_static_walls(data,self.origin,.05,self.grid,self.tf)
        if boundary:merge_static_walls(data,self.origin,.05,boundary_grid(self.grid),self.tf)
        return data

    def test_identical_republished_map_skips_raster_but_preserves_every_wall(self):
        with patch('nav2.vision_layer.merge_static_walls',wraps=merge_static_walls) as merge:
            a=self.raster();self.assertEqual(merge.call_count,2)
            self.grid=copy.deepcopy(self.grid);b=self.raster()
            self.assertEqual(merge.call_count,2)
        np.testing.assert_array_equal(a,self.expected());np.testing.assert_array_equal(a,b)

    def test_live_obstacles_do_not_contaminate_cached_walls(self):
        expected=self.expected();data=self.raster();data[:]=100
        np.testing.assert_array_equal(self.raster(),expected)

    def test_add_and_remove_keepout_invalidates_immediately(self):
        self.raster()
        self.grid.data[100*44+10]=100
        np.testing.assert_array_equal(self.raster(),self.expected())
        self.grid.data[100*44+10]=0
        np.testing.assert_array_equal(self.raster(),self.expected())

    def test_window_tf_map_origin_and_boundary_mode_changes_recompute(self):
        self.raster()
        self.origin=(-4.,-5.)
        np.testing.assert_array_equal(self.raster(),self.expected())
        self.tf.translation.x+=.2
        np.testing.assert_array_equal(self.raster(),self.expected())
        self.grid.info.origin.orientation.z=math.sin(.2)
        self.grid.info.origin.orientation.w=math.cos(.2)
        np.testing.assert_array_equal(self.raster(),self.expected())
        np.testing.assert_array_equal(self.raster(False),self.expected(False))

    def test_failed_rebuild_never_serves_previous_map(self):
        self.raster();self.grid.data[0]=0
        with patch('nav2.vision_layer.merge_static_walls',side_effect=RuntimeError('bad map')):
            with self.assertRaisesRegex(RuntimeError,'bad map'):self.raster()
        np.testing.assert_array_equal(self.raster(),self.expected())
