# 【内容标注】用途：导航离线回归：vision_freshness。
# 对应用户需求：R22（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明。
# 本次仅加注释；需求关联不是精确创建/提交记录。
"""旧语义结果不能阻止最新深度进入安全检测。无需相机或 ROS 节点。"""
import threading
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock
from nav2.vision_layer import VisionLayer, stable_grid_origin


def record(frame_id, at, epoch=1, version=2):
    return NS(config_version=version, frame=NS(frame_id=frame_id,
        source_at=at, stream_epoch=epoch))


class VisionFreshnessTests(unittest.TestCase):
    def run_tick(self, records, packet, direct=None):
        layer=VisionLayer.__new__(VisionLayer)
        layer.engine=NS(lock=threading.RLock(), _config_version=2,
                        _buffer_m2=dict(enumerate(records)))
        layer.nav=NS(lock=threading.RLock(), vision_enabled=True)
        layer.semantic=Mock()
        layer.semantic.snapshot.return_value=packet
        if direct is not None:
            layer._depth_mailbox=NS(latest=lambda version,epoch:direct)
        layer._tick_locked=Mock()
        layer.tick()
        return layer._tick_locked.call_args.args

    def test_old_semantic_packet_does_not_replace_latest_depth(self):
        old,new=record(1,10),record(2,10.5)
        self.assertEqual(self.run_tick([old,new],(old,['old mask'])),([new],2,None))

    def test_matching_frame_keeps_semantics(self):
        new=record(2,10.5)
        self.assertEqual(self.run_tick([new],(new,['mask'])),([new],2,['mask']))

    def test_same_id_from_previous_stream_is_not_reused(self):
        new=record(2,10.5)
        self.assertEqual(self.run_tick([new],(record(2,10,epoch=0),['mask'])),([new],2,None))

    def test_previous_config_is_excluded(self):
        new=record(2,10.5)
        self.assertEqual(self.run_tick([record(3,11,version=1),new],None),([new],2,None))

    def test_empty_results_are_valid_same_frame_results(self):
        new=record(2,10.5)
        self.assertEqual(self.run_tick([new],(new,[])),([new],2,[]))

    def test_direct_result_is_used_before_fusion_catches_up(self):
        old,new=record(1,10),record(2,10.5)
        self.assertEqual(self.run_tick([old],None,new),([new],2,None))

    def test_mailbox_cannot_replace_newer_buffer_record(self):
        old,new=record(1,10),record(2,10.5)
        self.assertEqual(self.run_tick([new],None,old),([new],2,None))

class StableGridTests(unittest.TestCase):
    def test_driving_through_bend_does_not_resize_grid_each_cell(self):
        origin=stable_grid_origin(0.,0.)
        for x,y in ((.051,0.),(.5,.1),(1.,.5),(2.,1.),(2.,2.)):
            self.assertEqual(stable_grid_origin(x,y,origin),origin)

    def test_recenter_before_robot_nears_map_edge(self):
        origin=stable_grid_origin(0.,0.)
        for x,y in ((2.6,0.),(-2.6,0.),(0.,2.6),(0.,-2.6)):
            new=stable_grid_origin(x,y,origin)
            self.assertNotEqual(new,origin)
            self.assertEqual(new,(int(__import__('math').floor(x/.05))-100,
                                  int(__import__('math').floor(y/.05))-100))
