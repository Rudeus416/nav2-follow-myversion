# 【内容标注】用途：导航离线回归：clearance_worker。
# 对应用户需求：R13 R22（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明。
# 本次仅加注释；需求关联不是精确创建/提交记录。
"""Isolated checker must preserve full-body collision rejection and fail closed."""
import subprocess
import unittest
from unittest.mock import patch
from types import SimpleNamespace as NS
import numpy as np
from nav2.clearance_worker import validate_isolated, _WORKER_LOCK
from test_whole_body_preview import route, grid, FOOT


class ClearanceWorkerTests(unittest.TestCase):
    def test_clear_path_passes_in_real_child(self):
        cells=np.zeros((60,60),dtype=np.int8)
        validate_isolated(route((1.,1.,0),(2.,1.,0)),[grid(cells),grid(cells)],FOOT)

    def test_obstacle_in_second_grid_is_rejected(self):
        free=np.zeros((60,60),dtype=np.int8);occupied=free.copy();occupied[20,30]=100
        with self.assertRaisesRegex(ValueError,'整车'):
            validate_isolated(route((1.,1.,0),(2.,1.,0)),[grid(free),grid(occupied)],FOOT)

    def test_purple_in_body_rejected_in_child(self):
        cells=np.zeros((60,60),dtype=np.int8);cells[23,20]=1
        with self.assertRaisesRegex(ValueError,'紫色'):
            validate_isolated(route((1.,1.,0)),[grid(cells)],FOOT)

    def test_timeout_crash_and_malformed_reply_reject(self):
        g=grid(np.zeros((60,60),dtype=np.int8));p=route((1.,1.,0))
        for result in (NS(returncode=1,stdout=''),NS(returncode=0,stdout='invalid'),
                       NS(returncode=0,stdout='{}'),NS(returncode=0,stdout='[]')):
            with self.subTest(result=result),patch('nav2.clearance_worker.subprocess.run',return_value=result):
                with self.assertRaises(ValueError):validate_isolated(p,[g],FOOT)
        with patch('nav2.clearance_worker.subprocess.run',side_effect=subprocess.TimeoutExpired('worker',3)):
            with self.assertRaisesRegex(ValueError,'超时'):validate_isolated(p,[g],FOOT)
        self.assertFalse(_WORKER_LOCK.locked())

    def test_busy_worker_rejects_without_queue(self):
        with _WORKER_LOCK,patch('nav2.clearance_worker.subprocess.run') as run:
            with self.assertRaisesRegex(ValueError,'正在进行'):
                validate_isolated(route((1.,1.,0)),[],FOOT)
            run.assert_not_called()

    def test_stale_grid_never_starts_child(self):
        with patch('nav2.clearance_worker.subprocess.run') as run:
            with self.assertRaises(ValueError):validate_isolated(route((1.,1.,0)),[{'stale':True}],FOOT)
            run.assert_not_called()
