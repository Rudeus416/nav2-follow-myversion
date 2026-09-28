# 【内容标注】用途：导航离线回归：check_paths。
# 对应用户需求：R13 R14 R22（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明。
# 本次仅加注释；需求关联不是精确创建/提交记录。
"""Check native A* output using the production swept-footprint validator.

Run from repository root after the three native clearance executable invocations.
"""
import math
import sys
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from nav2.path_clearance import validate_path

data = np.zeros((84, 44), dtype=np.int8)
data[0, :] = data[-1, :] = data[:, 0] = data[:, -1] = 100
data[40, 21:27] = 100
grid = dict(data=data.ravel().tolist(), width=44, height=84, resolution=.05,
            origin=[0., 0.], yaw=0., stale=False)
footprint = [[.297, .232], [.297, -.232], [-.297, -.232], [-.297, .232]]
for name in ('zero', 'safe', 'default'):
    points = np.loadtxt('/tmp/nav2-path-' + name + '.txt')
    path = NS(poses=[NS(pose=NS(position=NS(x=x, y=y), orientation=NS(
        x=0., y=0., z=math.sin(a/2), w=math.cos(a/2)))) for x, y, a in points])
    try:
        validate_path(path, grid, footprint)
    except ValueError:
        if name != 'zero':
            raise
        print('PASS: zero inflation reproduces a path intersecting the car body')
    else:
        if name == 'zero':
            raise AssertionError('zero-inflation regression was not reproduced')
        print('PASS:', name, 'native path clears the obstacle with the full swept footprint')
