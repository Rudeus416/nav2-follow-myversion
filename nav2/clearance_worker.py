# 【内容标注】用途：隔离进程中的整车路径复核。
# 对应用户需求：R13 R14 R22（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：限制并发与超时，避免 Python 几何计算阻塞视觉和控制线程；失败不放行。
# 本次仅加注释；需求关联不是精确创建/提交记录。
"""Run full-body collision checks outside the camera/ROS Python interpreter.

The child has no ROS node, camera or motion interface. A timeout, crash or invalid
reply rejects the route; it never falls back to skipping collision checks.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import threading

# Do not queue expensive checks from repeated clicks/person updates indefinitely.
_WORKER_LOCK = threading.Lock()


# 【职责 / R13 R14 R22】validate_isolated：在限时子进程中校验，避免复核抢占主线程；异常拒绝执行。
def validate_isolated(path, grids, footprint, timeout=3.0):
    if not _WORKER_LOCK.acquire(blocking=False):
        raise ValueError('路径复核正在进行，请稍后重新预览')
    try:
        poses=[]
        for entry in path.poses:
            p,q=entry.pose.position,entry.pose.orientation
            poses.append([p.x,p.y,q.x,q.y,q.z,q.w])
        snapshots=[]
        for grid in grids:
            if not grid or grid.get('stale'):
                raise ValueError('地图未就绪，无法复核车身通行空间')
            snap={k:grid[k] for k in ('resolution','width','height','origin','yaw','data')}
            snap['data']=list(snap['data']) if not hasattr(snap['data'],'tolist') else snap['data'].tolist()
            snapshots.append(snap)
        payload=json.dumps({'poses':poses,'grids':snapshots,'footprint':footprint.tolist()
                            if hasattr(footprint,'tolist') else footprint},allow_nan=False)
        # Limit numerical threads only in our child; never change the old app's
        # global BLAS/torch configuration. One child is allowed at a time.
        env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
        try:
            result=subprocess.run([sys.executable,'-B',str(Path(__file__).resolve())],
                input=payload,text=True,capture_output=True,timeout=timeout,env=env)
        except subprocess.TimeoutExpired as exc:
            raise ValueError('整车路径复核超时，保持停车') from exc
        if result.returncode:
            raise ValueError('整车路径复核进程异常，保持停车')
        try:reply=json.loads(result.stdout)
        except (ValueError,TypeError) as exc:
            raise ValueError('整车路径复核响应无效，保持停车') from exc
        if not isinstance(reply,dict) or reply.get('ok') is not True:
            raise ValueError(reply.get('error','整车路径复核未通过') if isinstance(reply,dict)
                             else '整车路径复核响应无效，保持停车')
    finally:
        _WORKER_LOCK.release()


# 【职责 / R13 R14 R22】main：子进程入口，读取校验输入并返回成功/错误结果。
def main():
    # Executed by file path: this directory contains path_clearance.py.
    from types import SimpleNamespace as NS
    from path_clearance import validate_path
    try:
        request=json.load(sys.stdin)
        path=NS(poses=[NS(pose=NS(position=NS(x=x,y=y),
                    orientation=NS(x=qx,y=qy,z=qz,w=qw)))
                    for x,y,qx,qy,qz,qw in request['poses']])
        if not request['grids']:raise ValueError('缺少待复核地图')
        for grid in request['grids']:
            validate_path(path,grid,request['footprint'])
        reply={'ok':True}
    except Exception as exc:
        reply={'ok':False,'error':str(exc)}
    print(json.dumps(reply,ensure_ascii=False))


if __name__=='__main__':
    main()
