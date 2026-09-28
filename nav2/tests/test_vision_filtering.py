# 【内容标注】用途：导航离线回归：vision_filtering。
# 对应用户需求：R08 R10 R22（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明。
# 需求关联用于追溯用途，不代表精确创建/提交记录。
import unittest
import numpy as np
from nav2.vision_layer import depth_points

class VisionFilteringTests(unittest.TestCase):
    calibration={'camera_matrix':{'data':[100.,0,80,0,100,60,0,0,1]},
                 'image_width':160,'image_height':120,
                 'distortion_coefficients':{'data':[0.,0,0,0,0]}}
    def points(self, depth, **kwargs):
        return depth_points(depth,(120,160,3),self.calibration,.5,0,1,**kwargs)
    def test_map_clipping_does_not_turn_valid_object_into_noise(self):
        depth=np.full((60,80),8.,np.float32)
        depth[10:20,30:40]=.8
        accepted=np.zeros(4800,bool);accepted[10*80+30]=True
        diag={}
        p=self.points(depth,accept=lambda _:accepted,diagnostics=diag)
        self.assertEqual(len(p),1)
        self.assertEqual(diag['component_pixels'],100)
        self.assertEqual(diag['map_pixels'],1)
    def test_raw_single_pixel_noise_is_still_rejected(self):
        depth=np.full((60,80),8.,np.float32);depth[10,30]=.8
        diag={};p=self.points(depth,diagnostics=diag)
        self.assertEqual(len(p),0)
        self.assertEqual(diag['reason'],'候选面积不足')
    def test_off_map_candidate_can_be_previewed_but_not_added_to_map(self):
        depth=np.full((60,80),8.,np.float32);depth[10:20,30:40]=.8
        diag={}
        self.assertEqual(len(self.points(depth,accept=lambda p:np.zeros(len(p),bool),diagnostics=diag)),0)
        self.assertEqual(len(self.points(depth)),100)
        self.assertIn('地图外',diag['reason'])
    def test_range_rejection_is_reported_without_fabricating_points(self):
        diag={};self.assertEqual(len(self.points(np.full((60,80),8.),diagnostics=diag)),0)
        self.assertEqual(diag['reason'],'距离/横向范围内无候选')


    def test_touching_near_and_far_surfaces_are_separate_candidates(self):
        # They share a full image edge, but the wall is 1.9 m behind the cart.
        depth=np.full((60,80),8.,np.float32)
        depth[10:20,30:40]=.6
        depth[10:20,40:50]=2.5
        points=self.points(depth)
        self.assertEqual(len(points),100)
        np.testing.assert_allclose(points[:,0],.6)

    def test_irregular_object_with_continuous_depth_is_not_cut_to_nearest_slice(self):
        depth=np.full((60,80),8.,np.float32)
        observed=np.zeros(depth.shape,bool)
        # A sloping L-shaped surface spans .6 m in depth overall, with only
        # .1 m changes between adjacent samples. Preserve all observed surface.
        for column in range(30,37):
            rows=slice(10,20 if column<33 else 14)
            depth[rows,column]=.6+.1*(column-30)
            observed[rows,column]=True
        points=self.points(depth)
        self.assertEqual(len(points),int(observed.sum()))
        self.assertAlmostEqual(float(points[:,0].max()),1.2,places=5)
        np.testing.assert_allclose(np.sort(points[:,0]),np.sort(depth[observed]))

    def test_near_speckles_cannot_borrow_far_surface_support(self):
        depth=np.full((60,80),8.,np.float32)
        depth[10:20,40:50]=2.5
        # Three disconnected two-pixel speckles all touch the far wall.
        for row in (10,14,18):depth[row,38:40]=.6
        points=self.points(depth)
        self.assertEqual(len(points),100)
        np.testing.assert_allclose(points[:,0],2.5)

    def test_emergency_all_candidates_still_contains_near_and_far_surfaces(self):
        depth=np.full((60,80),8.,np.float32)
        depth[10:20,30:40]=.6
        depth[10:20,40:50]=2.5
        points=self.points(depth,nearest_only=False)
        self.assertEqual(len(points),200)
        np.testing.assert_allclose(np.sort(points[:,0]),[.6]*100+[2.5]*100)

    def test_diagonal_continuous_support_is_preserved(self):
        depth=np.full((60,80),8.,np.float32)
        for offset in range(8):depth[10+offset,30+offset]=.8
        points=self.points(depth)
        self.assertEqual(len(points),8)
        np.testing.assert_allclose(points[:,0],.8)

if __name__=='__main__':unittest.main()
