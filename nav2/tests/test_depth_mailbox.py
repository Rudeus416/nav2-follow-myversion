# 【内容标注】用途：导航离线回归：depth_mailbox。
# 对应用户需求：R22（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明。
# 本次仅加注释；需求关联不是精确创建/提交记录。
import threading
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock
from nav2.depth_mailbox import DepthMailbox


def result(at=10.,version=1,epoch=1):
    return NS(config_version=version,frame=NS(source_at=at,stream_epoch=epoch))


class DepthMailboxTests(unittest.TestCase):
    def test_original_callback_receives_each_result_once_and_unchanged(self):
        original=Mock();worker=NS(on_result=original);box=DepthMailbox(worker)
        a,b=result(),result(11.)
        worker.on_result(a);worker.on_result(b)
        self.assertEqual(original.call_args_list, [unittest.mock.call(a),unittest.mock.call(b)])
        self.assertIs(box.latest(1,1),b)
        box.close();self.assertIs(worker.on_result,original)

    def test_navigation_can_read_while_fusion_callback_is_blocked(self):
        entered=threading.Event();release=threading.Event()
        def fusion(record):entered.set();release.wait(2.)
        worker=NS(on_result=fusion);box=DepthMailbox(worker);r=result()
        thread=threading.Thread(target=worker.on_result,args=(r,));thread.start()
        try:
            self.assertTrue(entered.wait(1.))
            self.assertIs(box.latest(1,1),r)
        finally:release.set();thread.join(1.);box.close()

    def test_original_callback_exception_not_swallowed(self):
        original=Mock(side_effect=RuntimeError('original failure'))
        worker=NS(on_result=original);box=DepthMailbox(worker);r=result()
        with self.assertRaisesRegex(RuntimeError,'original failure'):worker.on_result(r)
        self.assertIs(box.latest(1,1),r);original.assert_called_once_with(r);box.close()

    def test_out_of_order_and_wrong_configuration_are_not_used(self):
        worker=NS(on_result=Mock());box=DepthMailbox(worker)
        new=result(12.);worker.on_result(new);worker.on_result(result(10.))
        self.assertIs(box.latest(1,1),new)
        self.assertIsNone(box.latest(2,1));self.assertIsNone(box.latest(1,2));box.close()

    def test_shutdown_does_not_overwrite_later_callback(self):
        worker=NS(on_result=Mock());box=DepthMailbox(worker);late=worker.on_result
        newer=Mock();worker.on_result=newer;box.close();late(result())
        self.assertIs(worker.on_result,newer);self.assertIsNone(box.latest(1))
