"""Read-only, stationary camera overlay for user-painted map boundaries."""
import math
import cv2
import numpy as np
import yaml
from pathlib import Path
from fastapi import HTTPException, Query
from fastapi.responses import Response


def boundary_edges(shapes):
    cells = set()
    for shape in shapes:
        if isinstance(shape, dict):
            cells.update(map(tuple, shape['cells']))
        else:
            x0,y0,x1,y1 = shape
            cells.update((x,y) for x in range(x0,x1) for y in range(y0,y1))
        if len(cells)>20000:
            raise ValueError('标注区域过大，虚拟墙预览最多支持 20000 格')
    edges=[]
    for x,y in cells:
        for neighbor, a, b in [((x-1,y),(x,y),(x,y+1)),((x+1,y),(x+1,y),(x+1,y+1)),
                                ((x,y-1),(x,y),(x+1,y)),((x,y+1),(x,y+1),(x+1,y+1))]:
            if neighbor not in cells: edges.append((a,b))
    if len(edges)>4000: raise ValueError('边界过于复杂，请减少标注区域')
    return edges


def clip_near(points, near=.1):
    result=[]
    for a,b in zip(points, points[1:]+points[:1]):
        inside_a,inside_b=a[2]>=near,b[2]>=near
        if inside_a: result.append(a)
        if inside_a != inside_b:
            result.append(a+(b-a)*((near-a[2])/(b[2]-a[2])))
    return result


def render(image, shapes, grid, robot, mount, height, pitch, wall_height, calibration, fill_color=(80,80,255), edge_color=(50,50,230)):
    k=np.array(calibration['camera_matrix']['data'],float).reshape(3,3)
    k[0,:]*=image.shape[1]/calibration['image_width']
    k[1,:]*=image.shape[0]/calibration['image_height']
    distortion=np.array(calibration['distortion_coefficients']['data'],float)
    rx,ry=robot['position']; yaw=robot['yaw']
    mx,my,myaw=mount
    cx=rx+math.cos(yaw)*mx-math.sin(yaw)*my
    cy=ry+math.sin(yaw)*mx+math.cos(yaw)*my
    camera_yaw=yaw+myaw; p=math.radians(pitch)
    def camera(cell,z):
        gx,gy=cell[0]*grid['resolution'],cell[1]*grid['resolution']; g=grid['yaw']
        x=grid['origin'][0]+math.cos(g)*gx-math.sin(g)*gy-cx
        y=grid['origin'][1]+math.sin(g)*gx+math.cos(g)*gy-cy
        forward=math.cos(camera_yaw)*x+math.sin(camera_yaw)*y; left=-math.sin(camera_yaw)*x+math.cos(camera_yaw)*y; up=z-height
        return np.array([-left,-(math.sin(p)*forward+math.cos(p)*up),math.cos(p)*forward-math.sin(p)*up])
    polygons=[]
    for a,b in boundary_edges(shapes):
        points=clip_near([camera(a,0),camera(b,0),camera(b,wall_height),camera(a,wall_height)])
        if len(points)<3:continue
        pts,_=cv2.projectPoints(np.array(points),np.zeros(3),np.zeros(3),k,distortion)
        if not np.isfinite(pts).all():continue
        pts=np.clip(pts.reshape(-1,2),-100000,100000).astype(np.int32)
        polygons.append((sum(v[2] for v in points)/len(points),pts))
    layer=image.copy()
    for _,pts in polygons:
        cv2.fillPoly(layer,[pts],fill_color)
    result=cv2.addWeighted(layer,.22,image,.78,0)
    for _,pts in polygons:
        cv2.polylines(result,[pts],True,edge_color,1)
    return result


def attach(app, bridge, motion, view):
    @app.get('/api/nav2/virtual-wall')
    def preview(camera_height: float=Query(.5,ge=.05,le=3),
                pitch_down: float=Query(0,ge=-60,le=60),
                wall_height: float=Query(.8,ge=.1,le=3),
                vision: bool=False, red: bool=True):
        with motion.lock:
            if not motion.estop:
                raise HTTPException(409,'仅支持强停状态下的虚拟墙对齐预览')
            nav=motion.navigation
            if nav is None or nav.obstacle_mode not in ('map','both') or view is None or view.editor is None:
                raise HTTPException(409,'请启用地图或叠加模式')
            mount=nav.camera_pose
        if mount is None: raise HTTPException(409,'缺少相机平面安装参数')
        if not bridge.image_recent():raise HTTPException(503,'相机没有新鲜图像')
        jpeg=bridge.latest()
        if jpeg is None:raise HTTPException(503,'等待相机图像')
        snapshot=view.snapshot();grid=snapshot['layers'].get('static_map');robot=snapshot['layers'].get('robot')
        if not grid or grid['stale'] or not robot or robot['stale'] or robot['age']>.5:
            raise HTTPException(409,'等待新鲜地图及车身位姿')
        try:
            image=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
            if image is None:raise ValueError('相机图像解码失败')
            if image.shape[1]>960:
                image=cv2.resize(image,(960,round(image.shape[0]*960/image.shape[1])))
            calibration=yaml.safe_load((Path(__file__).resolve().parent.parent/'camera_calibration.yaml').read_text())
            result=image
            if red:
                result=render(result,snapshot['editing']['rectangles'],grid,robot,mount,camera_height,pitch_down,wall_height,calibration)
            if vision:
                import time
                with nav.lock:
                    layer=getattr(nav,'vision_layer',None)
                    if not nav.vision_enabled or layer is None:
                        raise HTTPException(409,'请先开启视觉障碍检测')
                    if nav.vision_at<=0 or time.monotonic()-nav.vision_at>1.2:
                        raise HTTPException(409,'视觉障碍数据过期，隐藏黄色墙')
                    cells=list(layer.cells)
                    camera_height,pitch_down=layer.height,layer.pitch
                # Cells are the actual selected visual candidate in odom, not static walls.
                visual_grid={'resolution':.05,'origin':[0.,0.],'yaw':0.}
                result=render(result,[{'cells':cells}],visual_grid,robot,mount,
                              camera_height,pitch_down,wall_height,calibration,
                              fill_color=(0,220,255),edge_color=(0,180,255))
            ok,encoded=cv2.imencode('.jpg',result,[cv2.IMWRITE_JPEG_QUALITY,80])
            if not ok:raise ValueError('预览编码失败')
            return Response(encoded.tobytes(),media_type='image/jpeg',headers={'Cache-Control':'no-store'})
        except (ValueError,KeyError,OSError,cv2.error) as exc:
            raise HTTPException(422,str(exc)) from exc
