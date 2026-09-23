"""Stage-one map target and visual obstacle preview; never executes motion."""
import math
import time
from pathlib import Path
import cv2
import numpy as np
import yaml
from fastapi import HTTPException, Query
from fastapi.responses import Response


def candidates(depth, k, distortion, image_shape, height, pitch, scale):
    h,w=image_shape[:2]
    values=cv2.resize(np.squeeze(depth).astype(np.float32),(80,60),interpolation=cv2.INTER_NEAREST)*scale
    yy,xx=np.indices(values.shape)
    pixels=np.stack(((xx+.5)*w/80,(yy+.5)*h/60),axis=-1).astype(np.float64)
    rays=cv2.undistortPoints(pixels.reshape(-1,1,2),k,distortion).reshape(60,80,2)
    p=math.radians(pitch)
    forward=values*(math.cos(p)-rays[:,:,1]*math.sin(p))
    elevation=height-values*(math.sin(p)+rays[:,:,1]*math.cos(p))
    lateral=-values*rays[:,:,0]
    mask=(np.isfinite(values)&(values>0)&(forward>.15)&(forward<3)&(np.abs(lateral)<.6)&(elevation>.12)&(elevation<1.8)).astype(np.uint8)
    count,labels,stats,_=cv2.connectedComponentsWithStats(mask,8)
    kept=np.zeros_like(mask)
    for i in range(1,count):
        if stats[i,cv2.CC_STAT_AREA]>=6: kept[labels==i]=1
    nearest=float(np.percentile(forward[kept>0],10)) if kept.any() else None
    return kept, nearest


def attach(app, engine, motion, view):
    from nav2.vision_toggle import attach as attach_toggle
    attach_toggle(app, engine, motion)
    from nav2.inflation_control import attach as attach_inflation
    attach_inflation(app, motion)
    def validate_preview(path):
        if motion.navigation.obstacle_mode not in ('map','both'):
            return
        from nav2.path_clearance import validate_path
        snapshot=view.snapshot()
        robot=snapshot['layers'].get('robot');foot=snapshot['layers'].get('footprint')
        if not robot or robot['stale'] or not foot or foot['stale']:
            raise ValueError('等待新鲜车身轮廓和位姿，不能确认路径可通行')
        a=robot['yaw'];c,s=math.cos(a),math.sin(a)
        footprint=(np.asarray(foot['points'])-robot['position'])@np.array([[c,-s],[s,c]])
        validate_path(path,snapshot['layers'].get('static_map'),footprint)
    motion.navigation.validate_preview=validate_preview
    selection={}
    from nav2.point_navigation import attach as attach_point_navigation
    attach_point_navigation(app, motion, view, selection)
    def allowed():
        if not motion.estop:raise HTTPException(409,'先强停再进行选点及视觉校验')
        if motion.navigation is None or motion.navigation.obstacle_mode not in ('map','both'):
            raise HTTPException(409,'请启用地图或叠加模式')

    @app.post('/api/nav2/point-preview')
    def point_preview(payload: dict):
        with motion.lock:
            allowed()
            snapshot=view.snapshot();grid=snapshot['layers'].get('static_map');state=snapshot.get('editing',{})
            if not grid or grid['stale']:raise HTTPException(409,'静态地图未就绪')
            if payload.get('base_id')!=state.get('base_id') or payload.get('revision')!=state.get('revision'):
                raise HTTPException(409,'地图已更新，请重新选点')
            x,y=payload.get('x'),payload.get('y')
            if type(x) is not int or type(y) is not int or not(0<=x<grid['width'] and 0<=y<grid['height']):
                raise HTTPException(422,'目标超出地图')
            if grid['data'][y*grid['width']+x]!=0:raise HTTPException(422,'请选择地图空地，不能选择墙或禁入区域')
            dx,dy=(x+.5)*grid['resolution'],(y+.5)*grid['resolution'];a=grid['yaw']
            gx=grid['origin'][0]+math.cos(a)*dx-math.sin(a)*dy
            gy=grid['origin'][1]+math.sin(a)*dx+math.cos(a)*dy
            motion.navigation.preview_goal(gx,gy)
            robot=snapshot['layers'].get('robot')
            if robot and not robot['stale']:
                rx,ry=robot['position']
                selection['value']={'payload':dict(payload),'sequence':motion.navigation.preview_sequence,
                                    'goal':(gx,gy,math.atan2(gy-ry,gx-rx))}
            else:
                selection.clear()
            return motion.navigation.status()['nav2_preview']

    @app.get('/api/nav2/vision-obstacle-preview')
    def vision_preview(camera_height:float=Query(.5,ge=.05,le=3),
                       pitch_down:float=Query(0,ge=-60,le=60),
                       depth_scale:float=Query(1,ge=.05,le=10)):
        with motion.lock:allowed()
        with engine.lock:
            records=list(engine._buffer_m2.values())
            version=engine._config_version
        records=[r for r in records if r.config_version==version]
        if not records:raise HTTPException(503,'等待深度数据')
        record=max(records,key=lambda r:r.frame.source_at)
        age=time.monotonic()-record.frame.source_at
        if age>1.2:raise HTTPException(409,f'视觉数据超时：{age:.2f} 秒')
        calibration=yaml.safe_load((Path(__file__).resolve().parent.parent/'camera_calibration.yaml').read_text())
        image=record.frame.image
        k=np.array(calibration['camera_matrix']['data'],float).reshape(3,3)
        k[0,:]*=image.shape[1]/calibration['image_width'];k[1,:]*=image.shape[0]/calibration['image_height']
        mask,nearest=candidates(record.depth,k,np.array(calibration['distortion_coefficients']['data'],float),image.shape,camera_height,pitch_down,depth_scale)
        output=cv2.resize(image,(640,round(image.shape[0]*640/image.shape[1])))
        large=cv2.resize(mask,(output.shape[1],output.shape[0]),interpolation=cv2.INTER_NEAREST)>0
        overlay=output.copy();overlay[large]=(0,100,255)
        output=cv2.addWeighted(output,.65,overlay,.35,0)
        ok,jpeg=cv2.imencode('.jpg',output)
        if not ok:raise HTTPException(503,'预览编码失败')
        return Response(jpeg.tobytes(),media_type='image/jpeg',headers={'Cache-Control':'no-store',
            'X-Candidate-Distance': '' if nearest is None else f'{nearest:.2f}', 'X-Frame-Age':f'{age:.2f}'})
