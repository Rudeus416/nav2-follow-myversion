# 【内容标注】用途：导航离线回归：point_start_idle_stop。
# 对应用户需求：R11 R22（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明。
# 本次仅加注释；需求关联不是精确创建/提交记录。
"""Start validation must tolerate idle stop repeats, but not explicit state changes."""
import threading,time,unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
from concurrent.futures import Future
from fastapi import HTTPException
from nav2_follow import Nav2Follower, VisionStale
from nav2.point_navigation import attach

class App:
    def __init__(self):self.routes={}
    def post(self,name):
        def register(fn):self.routes[name]=fn;return fn
        return register

class PointStartTests(unittest.TestCase):
    def setup_case(self,explicit=False):
        n=Nav2Follower.__new__(Nav2Follower)
        n.lock=threading.RLock();n.enabled=n.pending=n.canceling=False
        n.handle=None;n.generation=0;n.preview_sequence=1;n.preview_handle=None
        n.preview={'state':'ready','points':[[0,0],[1,0]]};n.preview_at=time.monotonic()
        n.preview_full_path=NS(poses=[NS(pose=NS(position=NS(x=0,y=0),orientation=NS(x=0,y=0,z=0,w=1)))])
        n.healthy_pose=lambda:(0.,0.,0.)
        n.path_client=Mock();n.path_client.send_goal_async.return_value=Future()
        n.path_action_type=NS(Goal=lambda:NS())
        def validate(path):
            for _ in range(4):n.stop('未处于自动跟随')
            if explicit:n.clear_preview()
        n.validate_preview=validate
        payload={'x':1,'y':1,'base_id':'map','revision':1}
        view=NS(snapshot=lambda:{'editing':dict(payload)})
        selection={'value':{'payload':dict(payload),'sequence':1,'goal':(1.,0.,0.)}}
        motion=NS(lock=threading.RLock(),navigation=n,estop=True,radar_reconfiguring=False,following=False,mode='auto')
        app=App();attach(app,motion,view,selection)
        return app.routes['/api/nav2/point-start'],payload,n,motion

    def test_idle_stop_during_validation_allows_valid_start(self):
        execute,payload,n,motion=self.setup_case()
        execute(payload)
        n.path_client.send_goal_async.assert_called_once()
        self.assertEqual(n.generation,0)
        self.assertTrue(n.fixed_goal_active)
        self.assertFalse(motion.estop)

    def test_explicit_stop_invalidates_preview_and_blocks_start(self):
        execute,payload,n,motion=self.setup_case(True)
        with self.assertRaises(HTTPException) as exc:execute(payload)
        self.assertEqual(exc.exception.status_code,409)
        n.path_client.send_goal_async.assert_not_called()
        self.assertTrue(motion.estop)

    def test_transient_stale_revalidates_before_start(self):
        execute,payload,n,motion=self.setup_case()
        n.vision_at=time.monotonic()-1.3
        n.healthy_pose=Mock(side_effect=[VisionStale('stale'),(0.,0.,0.),(0.,0.,0.)])
        validate=n.validate_preview
        n.validate_preview=Mock(side_effect=validate)
        with patch('nav2.point_navigation.time.sleep'):
            execute(payload)
        self.assertEqual(n.validate_preview.call_count,2)
        n.path_client.send_goal_async.assert_called_once()

    def test_persistent_stale_never_sends_motion(self):
        execute,payload,n,motion=self.setup_case()
        n.vision_at=time.monotonic()-2
        n.healthy_pose=Mock(side_effect=VisionStale('stale'))
        with patch('nav2.point_navigation.VISION_START_WAIT',0):
            with self.assertRaises(HTTPException):execute(payload)
        n.path_client.send_goal_async.assert_not_called()
        self.assertTrue(motion.estop)

    def test_explicit_stop_while_waiting_prevents_delayed_start(self):
        execute,payload,n,motion=self.setup_case()
        n.vision_at=time.monotonic()-2
        n.healthy_pose=Mock(side_effect=VisionStale('stale'))
        with patch('nav2.point_navigation.time.sleep',side_effect=lambda _:n.clear_preview()):
            with self.assertRaises(HTTPException):execute(payload)
        n.path_client.send_goal_async.assert_not_called()
        self.assertTrue(motion.estop)

    def test_waiting_stale_frames_does_not_repeat_path_work(self):
        execute,payload,n,motion=self.setup_case()
        n.vision_enabled=True;n.vision_error='';n.vision_at=time.monotonic()-2
        original=n.validate_preview;n.validate_preview=Mock(side_effect=original)
        waits=[]
        def wait(_):
            # 视觉生产线程必须能获得两把锁；过期时不做昂贵复核。
            self.assertEqual(n.validate_preview.call_count,0)
            acquired=[]
            def producer():
                with motion.lock,n.lock:acquired.append(True)
            thread=threading.Thread(target=producer);thread.start();thread.join(.5)
            self.assertEqual(acquired,[True])
            waits.append(True)
            if len(waits)==3:n.vision_at=time.monotonic()
        with patch('nav2.point_navigation.time.sleep',side_effect=wait):execute(payload)
        self.assertEqual(len(waits),3)
        n.validate_preview.assert_called_once()
        n.path_client.send_goal_async.assert_called_once()

    def test_validation_time_does_not_consume_first_visual_wait(self):
        execute,payload,n,motion=self.setup_case()
        now=[100.];n.preview_at=100.;n.vision_enabled=True;n.vision_error='';n.vision_at=100.
        original=n.validate_preview
        def validate(path):
            original(path)
            if now[0]==100.:now[0]+=3.  # 首次复核超过旧版的整个等待预算。
        n.validate_preview=Mock(side_effect=validate)
        def healthy():
            n.check_vision();return (0.,0.,0.)
        n.healthy_pose=healthy
        def wait(_):
            now[0]+=.05;n.vision_at=now[0]
        with patch('nav2.point_navigation.time.monotonic',side_effect=lambda:now[0]), patch(
                'nav2.point_navigation.time.sleep',side_effect=wait):execute(payload)
        self.assertEqual(n.validate_preview.call_count,2)
        n.path_client.send_goal_async.assert_called_once()

if __name__=='__main__':unittest.main()
