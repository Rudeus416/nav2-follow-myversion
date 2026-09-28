# 【内容标注】用途：红黄虚拟墙相机叠加。
# 对应用户需求：R05 R06（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：投影手绘边界和视觉候选到同一相机预览；保持原相机视频功能独立。
# 本次仅加注释；需求关联不是精确创建/提交记录。
"""Read-only, stationary camera overlay for user-painted map boundaries."""
import math
import cv2
import numpy as np
import yaml
from pathlib import Path
from fastapi import HTTPException, Query
from fastapi.responses import Response


# 【职责 / R05 R06】boundary_edges：提取禁区栅格的外边缘，减少内部重复墙面。
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


# 【职责 / R05 R06】clip_near：裁剪相机近面后的线段，避免无效投影。
def clip_near(points, near=.1):
    result=[]
    for a,b in zip(points, points[1:]+points[:1]):
        inside_a,inside_b=a[2]>=near,b[2]>=near
        if inside_a: result.append(a)
        if inside_a != inside_b:
            result.append(a+(b-a)*((near-a[2])/(b[2]-a[2])))
    return result


# 【职责 / R05 R06】render：将三维虚拟墙投影并叠加到相机快照。
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
    # 【职责 / R05 R06】camera：取得用于停车叠加预览的相机数据。
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


# 【职责 / R05 R06】draw_objects：绘制视觉语义物体的辅助标注。
def draw_objects(image, objects):
    result=image.copy()
    h,w=result.shape[:2]
    for obj in objects:
        polygon=np.clip(np.asarray(obj['polygon'],float),0,1)
        pixels=np.minimum(polygon*[w,h],[w-1,h-1]).astype(np.int32)
        if len(pixels)<3:continue
        cv2.polylines(result,[pixels],True,(255,220,0),2)
        x,y=pixels.min(axis=0)
        cv2.putText(result,f"{obj['label']} {obj['confidence']:.2f}",(int(x),max(15,int(y))),
                    cv2.FONT_HERSHEY_SIMPLEX,.5,(255,220,0),1,cv2.LINE_AA)
    return result


# 【职责 / R05 R06】attach：注册本模块专用接口与依赖，复用既有应用，不修改原业务实现。
def attach(app, bridge, motion, view):
    # 【职责 / R05 R06】preview：在同一快照中叠加红色禁区和黄色视觉候选。
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
            objects=[]
            if vision:
                import time
                with nav.lock:
                    layer=getattr(nav,'vision_layer',None)
                    if not nav.vision_enabled or layer is None:
                        raise HTTPException(409,'请先开启视觉障碍检测')
                    if nav.vision_at<=0 or time.monotonic()-nav.vision_at>1.2:
                        raise HTTPException(409,'视觉障碍数据过期，隐藏黄色墙')
                    visual_snapshot=(list(layer.camera_cells),bool(layer.diagnostics.get('camera_only',False)),layer.height,layer.pitch)
                    packet=getattr(layer,'semantic_preview',None)
                    if packet and time.monotonic()-packet['source_at']<=1.2:
                        # Use segmentation's exact original frame and pose, never
                        # paint an old object mask onto a newer video frame.
                        image=packet['image'].copy();robot=packet['robot'];objects=packet['objects']
            if image.shape[1]>960:
                image=cv2.resize(image,(960,round(image.shape[0]*960/image.shape[1])))
            calibration=yaml.safe_load((Path(__file__).resolve().parent.parent/'camera_calibration.yaml').read_text())
            result=image
            yellow_count=0;camera_only=False
            if red:
                result=render(result,snapshot['editing']['rectangles'],grid,robot,mount,camera_height,pitch_down,wall_height,calibration)
            if vision:
                import time
                cells,camera_only,camera_height,pitch_down=visual_snapshot
                yellow_count=len(cells)
                # Map candidate when available; otherwise explicitly labelled camera-only candidate.
                visual_grid={'resolution':.05,'origin':[0.,0.],'yaw':0.}
                result=render(result,[{'cells':cells}],visual_grid,robot,mount,
                              camera_height,pitch_down,wall_height,calibration,
                              fill_color=(0,220,255),edge_color=(0,180,255))
            if objects:result=draw_objects(result,objects)
            ok,encoded=cv2.imencode('.jpg',result,[cv2.IMWRITE_JPEG_QUALITY,80])
            if not ok:raise ValueError('预览编码失败')
            return Response(encoded.tobytes(),media_type='image/jpeg',headers={'Cache-Control':'no-store',
                'X-Yellow-Wall-Count':str(yellow_count),
                'X-Semantic-Count':str(len(objects)),
                'X-Yellow-Camera-Only':'1' if camera_only else '0'})
        except (ValueError,KeyError,OSError,cv2.error) as exc:
            raise HTTPException(422,str(exc)) from exc
