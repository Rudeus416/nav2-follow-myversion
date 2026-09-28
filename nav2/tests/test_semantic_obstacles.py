# 【内容标注】用途：导航离线回归：semantic_obstacles。
# 对应用户需求：R09（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明。
# 本次仅加注释；需求关联不是精确创建/提交记录。
import queue
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import numpy as np
from nav2.semantic_obstacles import SemanticWorker, instance_masks, detections
from nav2.vision_layer import depth_points
from nav2.virtual_wall import draw_objects

class SemanticTests(unittest.TestCase):
    calibration={'camera_matrix':{'data':[100.,0,80,0,100,60,0,0,1]},'image_width':160,'image_height':120,'distortion_coefficients':{'data':[0.,0,0,0,0]}}
    def test_instance_polygon_scaled_without_letterbox_padding(self):
        masks=instance_masks([dict(label='cart',polygon=[[.25,.25],[.5,.25],[.5,.5],[.25,.5]])])
        self.assertTrue(masks[0][1][20,30]);self.assertFalse(masks[0][1][0,0])
    def test_one_instance_keeps_disconnected_parts_and_no_background(self):
        d=np.full((60,80),8.,np.float32);d[10:20,10:20]=1;d[10:20,60:70]=1
        instance=d==1;diag={}
        pts=depth_points(d,(120,160,3),self.calibration,.5,0,1,instances=[('cart',instance)],diagnostics=diag)
        self.assertEqual(len(pts),200);self.assertEqual(diag['selected_object'],'cart')
    def test_nearer_unknown_obstacle_is_not_suppressed(self):
        d=np.full((60,80),8.,np.float32);d[10:20,10:20]=.5;d[10:20,55:65]=2
        p=depth_points(d,(120,160,3),self.calibration,.5,0,1,instances=[('chair',d==2)])
        np.testing.assert_allclose(p[:,0],.5)
    def test_semantic_mask_cannot_bypass_map_bounds(self):
        d=np.ones((60,80),np.float32)
        p=depth_points(d,(120,160,3),self.calibration,.5,0,1,instances=[('cart',d>0)],accept=lambda p:np.zeros(len(p),bool))
        self.assertEqual(len(p),0)
    def test_empty_model_result_is_valid_and_drawing_is_non_mutating(self):
        self.assertEqual(detections(NS(masks=None,boxes=None)),[])
        img=np.zeros((60,80,3),np.uint8)
        rendered=draw_objects(img,[dict(label='cart',confidence=.8,polygon=[[.1,.1],[.5,.1],[.5,.5]])])
        self.assertTrue(rendered.any());self.assertFalse(img.any())
    def test_worker_drops_stale_and_wrong_epoch_results(self):
        w=SemanticWorker();w.enabled=True;w.attempted=True
        w.process=Mock();w.process.is_alive.return_value=True
        w.responses=queue.Queue();w.requests=queue.Queue(maxsize=1)
        now=time.monotonic()
        r=NS(config_version=1,frame=NS(source_at=now,stream_epoch='new',frame_id=4,image=np.zeros((60,80,3),np.uint8)))
        old=NS(config_version=1,frame=NS(source_at=now,stream_epoch='old'))
        w.latest=(old,[]);w.last_offer=now
        self.assertIsNone(w.update(r))
        w.latest=(r,[]);self.assertIs(w.update(r)[0],r)
        r.frame.source_at=now-2
        self.assertIsNone(w.update(r))
    def test_missing_weights_does_not_download_or_spawn(self):
        w=SemanticWorker();w.enabled=True
        with patch('nav2.semantic_obstacles.Path.is_file',return_value=False),patch('nav2.semantic_obstacles.mp.get_context') as create:
            self.assertIsNone(w.update(None));create.assert_not_called()
            self.assertIn('权重缺失',w.message)
