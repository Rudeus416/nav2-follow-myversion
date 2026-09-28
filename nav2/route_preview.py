# 【内容标注 / R32：遇障后保持同一终点绕行】用途：选点规划与视觉预览接口集合。
# 对应用户需求：R02 R08 R11 R13 R26（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：接入共用整车校验器、人物接口和人工选点；生成停车视觉预览候选。
# 需求关联用于追溯用途，不代表精确创建/提交记录；后续自检修复见 nav2/AUDIT.md。
"""Stage-one map target and visual obstacle preview; never executes motion."""
import math
import time
from pathlib import Path
import cv2
import numpy as np
import yaml
from fastapi import HTTPException, Query
from fastapi.responses import Response


# 【职责 / R02 R08 R11 R13】candidates：生成停车视觉预览候选，不代表已批准运动。
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


# 【职责 / R13 R20 R22】validate_route_snapshot：静态已知域与全局代价图共同校验整条路线。
def validate_route_snapshot(path, snapshot):
    from nav2.clearance_worker import validate_isolated
    layers=snapshot['layers']
    robot=layers.get('robot');foot=layers.get('footprint')
    if not robot or robot['stale'] or not foot or foot['stale']:
        raise ValueError('等待新鲜车身轮廓和位姿，不能确认路径可通行')
    global_map=layers.get('global_map')
    if not global_map or global_map.get('stale'):
        # Do not fall back to the local window or an empty map. Missing/stale
        # global data means the remote part of the planned route is unverified.
        raise ValueError('全局代价地图未就绪或已过期，保持停车，请等待地图更新后重新预览')
    a=robot['yaw'];c,s=math.cos(a),math.sin(a)
    footprint=(np.asarray(foot['points'])-robot['position'])@np.array([[c,-s],[s,c]])
    # R20/R22: checking the WHOLE route against the 10 m local window rejected
    # valid far goals even after the planner's global window was enlarged.
    # The full global map includes static/hand-drawn, vision/radar and purple
    # layers. The static map separately constrains the observed map extent;
    # enlarging a costmap must never make space outside that extent pass here.
    # DWB keeps its unchanged local costmap and real-time whole-body veto.
    validate_isolated(path,[layers.get('static_map'),global_map],footprint)


# 【职责 / R20 R22】route_snapshot：给规划结束后的全局地图发布一次有限追赶机会。
def route_snapshot(view, wait_seconds=.8):
    # Whole-body planning holds the global costmap mutex. A long search may end
    # just before the next 2 Hz full-map publication. Wait outside motion/nav
    # locks for fresh data; never increase the map/vision age allowance or use
    # stale data. The caller rechecks task generation and motion authorization.
    deadline=time.monotonic()+wait_seconds
    while True:
        snapshot=view.snapshot(include_global=True)
        global_map=snapshot['layers'].get('global_map')
        if global_map and not global_map.get('stale'):
            return snapshot
        remaining=deadline-time.monotonic()
        if remaining<=0:
            return snapshot  # validate_route_snapshot will reject it explicitly.
        time.sleep(min(.05,remaining))


# 【职责 / R02 R08 R11 R13】attach：注册本模块专用接口与依赖，复用既有应用，不修改原业务实现。
def attach(app, engine, motion, view):
    from nav2.vision_toggle import attach as attach_toggle
    attach_toggle(app, engine, motion)
    from nav2.inflation_control import attach as attach_inflation
    attach_inflation(app, motion)
    # 【职责 / R13 R20 R22】整路线使用完整全局代价图；局部图继续供 DWB 实时碰撞检查。
    def validate_preview(path):
        if motion.navigation.obstacle_mode not in ('map','both'):
            return
        validate_route_snapshot(path, route_snapshot(view))
    motion.navigation.validate_preview=validate_preview
    motion.navigation.person_motion=motion
    # R32: read-only global-map and full-vision providers for fixed-endpoint recovery.
    from nav2.replan_support import attach as attach_replan_support
    attach_replan_support(motion.navigation, view)
    from nav2.continuous_segment import attach as attach_continuous_segment
    app.add_event_handler('shutdown', attach_continuous_segment(motion))
    from nav2.person_map import attach as attach_person_map
    attach_person_map(app,engine,motion)
    selection={}
    from nav2.point_navigation import attach as attach_point_navigation
    attach_point_navigation(app, motion, view, selection)
    # 【职责 / R02 R08 R11 R13】allowed：限制选点与视觉校验的模式和强停条件。
    def allowed():
        if not motion.estop:raise HTTPException(409,'先强停再进行选点及视觉校验')
        if motion.navigation is None or motion.navigation.obstacle_mode not in ('map','both'):
            raise HTTPException(409,'请启用地图或叠加模式')

    # 【职责 / R02 R08 R11 R13】point_preview：校验人工终点与地图版本，保存预览身份。
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

    # 【职责 / R02 R08 R11 R13】vision_preview：生成停车状态下的视觉候选预览图。
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
