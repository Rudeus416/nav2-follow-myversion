# 【内容标注】用途：视觉估距比例的停车校验接口。
# 对应用户需求：R10（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：选图像点与实测距离校验深度比例；仅导航视觉路径使用，不宣称单目距离绝对准确。
# 本次仅加注释；需求关联不是精确创建/提交记录。
"""Stationary, same-frame range calibration for the Nav2 depth consumer only."""
import base64
import math
import secrets
import time
import cv2
import numpy as np
from fastapi import HTTPException


# 【职责 / R10】range_scale：根据图像选点与实测距离计算有效深度比例。
def range_scale(depth, shape, calibration, u, v, distance):
    if not all(math.isfinite(x) for x in (u,v,distance)) or not (0<=u<=1 and 0<=v<=1 and .15<=distance<=5):
        raise ValueError('请选择画面内的物体表面，并输入 0.15～5 米实测距离')
    values=np.squeeze(np.asarray(depth))
    h,w=values.shape
    x=min(w-1,int(u*w));y=min(h-1,int(v*h))
    patch=values[max(0,y-2):min(h,y+3),max(0,x-2):min(w,x+3)]
    valid=patch[np.isfinite(patch)&(patch>0)]
    if len(valid)<patch.size*.8:raise ValueError('选点深度无效，请选择物体内部表面')
    median=float(np.median(valid))
    if float(np.percentile(valid,90)-np.percentile(valid,10))>median*.25:
        raise ValueError('选点深度不连续，请避开物体边缘和空隙')
    ih,iw=shape[:2]
    k=np.array(calibration['camera_matrix']['data'],float).reshape(3,3)
    k[0,:]*=iw/calibration['image_width'];k[1,:]*=ih/calibration['image_height']
    ray=cv2.undistortPoints(np.array([[[u*iw,v*ih]]],float),k,
        np.array(calibration['distortion_coefficients']['data'],float)).reshape(2)
    raw_range=median*math.sqrt(1+float(ray@ray))
    scale=distance/raw_range
    if not .05<=scale<=10:raise ValueError('校准比例超出 0.05～10，请核对测量和选点')
    return scale,raw_range


# 【职责 / R10】attach：注册本模块专用接口与依赖，复用既有应用，不修改原业务实现。
def attach(app, engine, motion, layer):
    sample={}
    # 【职责 / R10】capture：保存停车校验所需图像/深度快照。
    @app.get('/api/nav2/vision-calibration')
    def capture():
        with motion.lock:
            if not motion.estop:raise HTTPException(409,'请先强停')
        with engine.lock:
            records=[r for r in engine._buffer_m2.values() if r.config_version==engine._config_version]
        if not records:raise HTTPException(409,'等待深度帧')
        record=max(records,key=lambda r:r.frame.source_at)
        if time.monotonic()-record.frame.source_at>1.2:raise HTTPException(409,'深度帧已过期')
        image=record.frame.image
        small=cv2.resize(image,(800,round(image.shape[0]*800/image.shape[1])))
        ok,jpeg=cv2.imencode('.jpg',small)
        if not ok:raise HTTPException(503,'校准画面编码失败')
        token=secrets.token_urlsafe(18)
        with motion.lock:
            if not motion.estop:raise HTTPException(409,'请保持强停')
            sample.clear();sample.update(token=token,created=time.monotonic(),depth=record.depth.copy(),shape=image.shape)
        return {'token':token,'image':'data:image/jpeg;base64,'+base64.b64encode(jpeg).decode(), 'expires_in':60}

    # 【职责 / R10】apply：检查样本及实测数据后应用导航视觉比例。
    @app.post('/api/nav2/vision-calibration')
    def apply(payload:dict):
        with motion.lock:
            if not motion.estop:raise HTTPException(409,'请先强停')
            if not sample or payload.get('token')!=sample['token'] or time.monotonic()-sample['created']>60:
                raise HTTPException(409,'校准画面已失效，请重新获取')
            try:
                scale,raw=range_scale(sample['depth'],sample['shape'],layer.calibration,
                    float(payload['u']),float(payload['v']),float(payload['distance']))
            except (KeyError,TypeError,ValueError) as exc:raise HTTPException(422,str(exc)) from exc
            nav=motion.navigation
            with nav.lock:
                nav.stop('应用 Nav2 视觉测距校准');nav.clear_preview()
                layer.scale=scale;layer.last=None
                layer.cells={};layer.camera_cells={};layer.diagnostics={}
                nav.vision_at=0.;nav.vision_count=0
                nav.vision_error='测距比例已更新，等待新帧' if nav.vision_enabled else ''
            sample.clear()
            return {'scale':scale,'raw_range':raw,'distance':float(payload['distance']),
                    'message':'已应用到 Nav2 视觉障碍；保持强停，等待新帧。重启恢复环境变量比例。'}
