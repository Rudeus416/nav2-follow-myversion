# 【内容标注】用途：导航离线回归：motion_geometry。
# 对应用户需求：R13 R18 R19（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明。
# 本次仅加注释；需求关联不是精确创建/提交记录。
import math
import threading
import unittest
from unittest.mock import Mock, patch
from nav2.motion_geometry import limit_twist, person_position, person_stopping_candidates
from nav2_follow import Nav2Follower, StableGoalFilter

class MotionGeometryTests(unittest.TestCase):
    def test_curvature_preserved_at_all_limits(self):
        for v,w in ((.3,.5),(.8,-.9),(-.3,.5),(.1,.8),(.1,0.),(0.,.8)):
            out_v,out_w=limit_twist(v,w)
            self.assertLessEqual(abs(out_v),.18 if v>=0 else .15)
            self.assertLessEqual(abs(out_w),.4)
            self.assertAlmostEqual(out_v*w,out_w*v)
        self.assertEqual(limit_twist(.3,.5),(.18,.3))

    def test_lower_user_caps_also_preserve_curvature(self):
        v,w=limit_twist(.3,.5,forward=.06,angular=.1)
        self.assertAlmostEqual(v,.06);self.assertAlmostEqual(w,.1)
        self.assertEqual(limit_twist(float('nan'),.2),(0.,0.))
        self.assertEqual(limit_twist(.1,.2,forward=0),(0.,0.))

    def test_mount_and_rotated_robot(self):
        x,y=person_position(2.,0.,10.,20.,math.pi/2,0.,.5,math.pi/2)
        self.assertAlmostEqual(x,7.5);self.assertAlmostEqual(y,20.)

    def test_stationary_person_does_not_cancel_while_robot_detours(self):
        n=Nav2Follower.__new__(Nav2Follower)
        n.lock=threading.RLock();n.enabled=True;n.pending=n.canceling=False
        n.handle=Mock();n.goal=(2.,0.,0.);n.person_anchor=(3.,0.);n.goal_at=0.
        n.goal_filter=StableGoalFilter();n.camera_pose=(0.,0.,0.);n.generation=0
        rx,ry=1.7,1.;yaw=math.atan2(-ry,3.-rx)
        n.healthy_pose=lambda:(rx,ry,yaw)
        for stamp in (5.,5.3,5.6):
            with patch('nav2_follow.time.monotonic',return_value=stamp):
                n.update(math.hypot(3.-rx,ry),0.,1.)
        n.handle.cancel_goal_async.assert_not_called()
        self.assertTrue(n.enabled)
        self.assertEqual(n.goal,(2.,0.,0.))

    def test_actual_person_position_not_clipped_to_three_meters(self):
        self.assertEqual(person_position(20.,0.,0.,0.,0.),(20.,0.))

    def test_planner_and_output_forward_limits_agree(self):
        from pathlib import Path
        import yaml
        config=yaml.safe_load((Path(__file__).resolve().parents[1]/'params.yaml').read_text())
        controller=config['controller_server']['ros__parameters']['FollowPath']
        v,_=limit_twist(1.,0.)
        self.assertEqual(controller['max_vel_x'],v)
        self.assertEqual(controller['max_speed_xy'],v)

    def test_candidates_preserve_standoff_and_three_meter_segment(self):
        for distance in (1.2,3.,10.):
            goal=(min(distance-1.,3.),0.,0.)
            candidates=person_stopping_candidates(goal,(distance,0.),(0.,0.,0.))
            self.assertEqual(candidates[0],goal)
            self.assertLessEqual(len(candidates),9)
            for x,y,yaw in candidates:
                self.assertGreaterEqual(math.hypot(x-distance,y)+1e-6,distance-goal[0])
                self.assertLessEqual(math.hypot(x,y),3.+1e-6)
                self.assertAlmostEqual(math.atan2(-y,distance-x),yaw)
        candidates=person_stopping_candidates((2.,0.,0.),(3.,0.),(0.,0.,0.))
        self.assertTrue(any(g[1]>.1 for g in candidates))
        self.assertTrue(any(g[1]<-.1 for g in candidates))

if __name__=='__main__':unittest.main()
