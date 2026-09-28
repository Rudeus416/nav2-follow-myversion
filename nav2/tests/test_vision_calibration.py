# 【内容标注】用途：导航离线回归：vision_calibration。
# 对应用户需求：R10（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明。
# 本次仅加注释；需求关联不是精确创建/提交记录。
import unittest
import math
import numpy as np
from nav2.vision_calibration import range_scale
from nav2.vision_layer import nearest_fragments

class CalibrationTests(unittest.TestCase):
    calibration={'camera_matrix':{'data':[100.,0,80,0,100,60,0,0,1]},'image_width':160,'image_height':120,'distortion_coefficients':{'data':[0.,0,0,0,0]}}
    def test_selected_surface_calibrates_in_metres(self):
        scale,raw=range_scale(np.full((120,160),4.),(120,160,3),self.calibration,.5,.5,.8)
        self.assertAlmostEqual(raw,4.)
        self.assertAlmostEqual(scale,.2)
    def test_off_axis_measurement_is_range_not_optical_depth(self):
        scale,raw=range_scale(np.full((120,160),4.),(120,160,3),self.calibration,.75,.5,.8)
        self.assertAlmostEqual(raw,4*math.sqrt(1+.4**2))
        self.assertAlmostEqual(raw*scale,.8)
    def test_bad_measurements_and_edge_depth_rejected(self):
        d=np.ones((120,160))
        for distance in (0,-1,float('nan'),float('inf'),6):
            with self.assertRaises(ValueError):range_scale(d,(120,160,3),self.calibration,.5,.5,distance)
        d[:,80:]=4
        with self.assertRaises(ValueError):range_scale(d,(120,160,3),self.calibration,.5,.5,.8)
    def test_nearby_fragments_retained_without_filling_gaps(self):
        labels=np.array([[1,1,0,2,2,0,3,3]])
        forward=np.array([[1,1,8,1.1,1.1,8,2,2.]])
        lateral=np.array([[0,.1,.2,.25,.3,.4,.5,.6]])
        selected=nearest_fragments(labels,forward,lateral)
        np.testing.assert_array_equal(selected,[[True,True,False,True,True,False,False,False]])

class CalibrationEndpointTests(unittest.TestCase):
    def setUp(self):
        import threading,time
        from types import SimpleNamespace as NS
        from unittest.mock import Mock
        from fastapi import FastAPI
        from nav2.vision_calibration import attach
        self.nav=NS(lock=threading.RLock(),stop=Mock(),clear_preview=Mock(),vision_enabled=True)
        self.motion=NS(lock=threading.RLock(),estop=True,navigation=self.nav)
        record=NS(config_version=1,depth=np.full((120,160),4.),frame=NS(source_at=time.monotonic(),image=np.zeros((120,160,3),np.uint8)))
        self.engine=NS(lock=threading.RLock(),_buffer_m2={1:record},_config_version=1)
        self.layer=NS(calibration=CalibrationTests.calibration,scale=1.)
        app=FastAPI();attach(app,self.engine,self.motion,self.layer)
        endpoints=[r for r in app.routes if r.path=='/api/nav2/vision-calibration']
        self.capture=next(r.endpoint for r in endpoints if 'GET' in r.methods)
        self.apply=next(r.endpoint for r in endpoints if 'POST' in r.methods)
    def test_apply_invalidates_navigation_and_token_without_starting_motion(self):
        from fastapi import HTTPException
        sample=self.capture();payload=dict(token=sample['token'],u=.5,v=.5,distance=.8)
        self.apply(payload)
        self.assertAlmostEqual(self.layer.scale,.2)
        self.nav.stop.assert_called_once();self.nav.clear_preview.assert_called_once()
        self.assertEqual(self.nav.vision_at,0.)
        self.assertTrue(self.motion.estop)
        with self.assertRaises(HTTPException):self.apply(payload)
    def test_apply_requires_stop_and_rejects_expired_sample(self):
        from fastapi import HTTPException
        from unittest.mock import patch
        sample=self.capture();payload=dict(token=sample['token'],u=.5,v=.5,distance=.8)
        self.motion.estop=False
        with self.assertRaises(HTTPException):self.apply(payload)
        self.motion.estop=True
        with patch('nav2.vision_calibration.time.monotonic',return_value=1e12):
            with self.assertRaises(HTTPException):self.apply(payload)
        self.assertEqual(self.layer.scale,1.)
