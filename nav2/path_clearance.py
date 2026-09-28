# 【内容标注】用途：Python 完整车身碰撞校验。
# 对应用户需求：R13 R14（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：检查完整多边形及沿路径的扫掠区域，拒绝障碍、紫色区、未知区和越界。
# 本次仅加注释；需求关联不是精确创建/提交记录。
"""Conservative full-footprint preview check, matching the local Nav2 plugins."""
import math
import numpy as np
import cv2


# 【职责 / R13 R14】validate_path：对完整路线执行多边形与扫掠检查，不只检查中心点。
def validate_path(path, grid, footprint):
    if not grid or grid.get('stale'):
        raise ValueError('地图未就绪，无法复核车身通行空间')
    r=grid['resolution'];w=grid['width'];h=grid['height']
    occupied=np.asarray(grid['data']).reshape(h,w)!=0  # Purple AND unknown are hard blocked.
    corners=np.asarray(footprint,float)
    if corners.ndim!=2 or corners.shape[1]!=2 or len(corners)<3 or not np.isfinite(corners).all():
        raise ValueError('车身轮廓无效')
    poses=[]
    for p in path.poses:
        t=p.pose.position;q=p.pose.orientation
        a=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        poses.append((t.x,t.y,a))
    if not poses or not np.isfinite(poses).all():raise ValueError('路径坐标无效')
    a=grid['yaw'];c,s=math.cos(a),math.sin(a)
    inv=np.array([[c,-s],[s,c]])
    radius=float(np.max(np.linalg.norm(corners,axis=1)))

    # 【职责 / R13 R14】polygon：把车身局部顶点转换到地图平面。
    def polygon(p):
        x,y,a=p;c,s=math.cos(a),math.sin(a)
        return ((corners@np.array([[c,s],[-s,c]])+[x,y])-grid['origin'])@inv

    # 【职责 / R13 R14】check：检查边界及占用格与多边形的分离轴碰撞。
    def check(poly, margin):
        lower=poly.min(axis=0)-margin;upper=poly.max(axis=0)+margin
        if np.any(lower<=0) or upper[0]>=w*r or upper[1]>=h*r:
            raise ValueError('无有效路径：车身超出已知地图边界')
        lo=np.maximum(0,np.floor(lower/r).astype(int)-1)
        hi=np.minimum([w-1,h-1],np.floor(upper/r).astype(int))
        yy,xx=np.nonzero(occupied[lo[1]:hi[1]+1,lo[0]:hi[0]+1])
        if not len(xx):return
        centers=np.column_stack((xx+lo[0]+.5,yy+lo[1]+.5))*r
        edges=np.roll(poly,-1,axis=0)-poly
        lengths=np.linalg.norm(edges,axis=1)
        normals=np.column_stack((-edges[:,1],edges[:,0]))[lengths>1e-10]/lengths[lengths>1e-10,None]
        axes=np.vstack((np.eye(2),normals))
        projection=poly@axes.T
        center=centers@axes.T
        extent=r/2*np.abs(axes).sum(axis=1)
        separated=(projection.max(axis=0)+margin<center-extent)|(projection.min(axis=0)-margin>center+extent)
        if np.any(~separated.any(axis=1)):
            raise ValueError('无有效路径：整车及安全边距会碰到障碍、手绘禁区或紫色缓冲区')

    checks=0;previous=poses[0]
    check(polygon(previous),0.)
    for pose in poses[1:]:
        da=math.atan2(math.sin(pose[2]-previous[2]),math.cos(pose[2]-previous[2]))
        count=max(1,math.ceil((math.hypot(pose[0]-previous[0],pose[1]-previous[1])+abs(da)*radius)/(r*.25)))
        before=polygon(previous)
        for i in range(1,count+1):
            checks+=1
            if checks>20000:raise ValueError('路径过长，无法完成车身复核')
            f=i/count
            after=polygon((previous[0]+f*(pose[0]-previous[0]),previous[1]+f*(pose[1]-previous[1]),previous[2]+f*da))
            hull=cv2.convexHull(np.vstack((before,after)).astype(np.float32)).reshape(-1,2)
            check(hull,radius*(1-math.cos(abs(da)/(2*count)))+1e-6)
            before=after
        previous=pose
