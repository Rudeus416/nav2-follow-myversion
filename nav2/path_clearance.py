"""Reject paths whose swept footprint intersects the displayed edited map."""
import math
import numpy as np
import cv2


def validate_path(path, grid, footprint):
    if not grid or grid.get('stale'):
        raise ValueError('地图未就绪，无法复核车身通行空间')
    r=grid['resolution'];w=grid['width'];h=grid['height']
    occupied=np.asarray(grid['data']).reshape(h,w)
    corners=np.asarray(footprint,float)
    poses=[]
    for p in path.poses:
        t=p.pose.position;q=p.pose.orientation
        a=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        poses.append((t.x,t.y,a))
    if not poses or not np.isfinite(poses).all():raise ValueError('路径坐标无效')
    a=grid['yaw'];c,s=math.cos(a),math.sin(a)
    inv=np.array([[c,-s],[s,c]])
    radius=float(np.max(np.linalg.norm(corners,axis=1)))
    checks=0
    previous=poses[0]
    for pose in poses:
        da=math.atan2(math.sin(pose[2]-previous[2]),math.cos(pose[2]-previous[2]))
        count=max(1,math.ceil((math.hypot(pose[0]-previous[0],pose[1]-previous[1])+abs(da)*radius)/(r*.25)))
        for i in range(count+1):
            checks+=1
            if checks>20000:raise ValueError('路径过长，无法完成车身复核')
            f=i/count;x=previous[0]+f*(pose[0]-previous[0]);y=previous[1]+f*(pose[1]-previous[1]);a=previous[2]+f*da
            c,s=math.cos(a),math.sin(a)
            world=corners@np.array([[c,s],[-s,c]])+[x,y]
            local=(world-grid['origin'])@inv/r
            if np.any(local<0) or np.any(local[:,0]>=w) or np.any(local[:,1]>=h):
                raise ValueError('无有效路径：车身超出已知地图边界')
            pixels=np.floor(local).astype(np.int32)
            lo=pixels.min(axis=0);hi=pixels.max(axis=0)
            mask=np.zeros((hi[1]-lo[1]+1,hi[0]-lo[0]+1),np.uint8)
            cv2.fillConvexPoly(mask,pixels-lo,1)
            cells=occupied[lo[1]:hi[1]+1,lo[0]:hi[0]+1]
            if np.any((mask>0)&((cells>=65)|(cells<0))):
                raise ValueError('无有效路径：车身及安全边距会碰到墙体或手绘禁区')
        previous=pose
