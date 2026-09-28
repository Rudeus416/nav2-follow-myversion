# 【内容标注】用途：导航离线回归：near_and_ramp。
# 对应用户需求：R19（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：对应模块的输入边界、故障或几何场景验证；测试名描述具体断言，不作为实车通过证明。
# 需求关联用于追溯用途，不代表精确创建/提交记录。
import unittest
from nav2.near_obstacle import NearObstacleGuard
from nav2.motion_geometry import CurvatureRamp


class NearGuardTests(unittest.TestCase):
    def test_hazard_stops_on_first_frame_even_single_point(self):
        g=NearObstacleGuard()
        self.assertTrue(g.update([[.69,0]],0,0,10,10)[0])

    def test_latency_and_turning_expand_protection(self):
        g=NearObstacleGuard()
        blocked,d=g.update([[.8,.45]],.18,.4,10,10.8)
        self.assertTrue(blocked);self.assertGreater(d['near_stop_m'],.8)
        self.assertGreater(d['near_half_width_m'],.45)

    def test_release_needs_three_new_clear_frames(self):
        g=NearObstacleGuard();g.update([[.6,0]],0,0,10,10)
        for stamp in (10.1,10.2):self.assertTrue(g.update([],0,0,stamp,stamp)[0])
        self.assertTrue(g.update([],0,0,10.2,10.2)[0])
        self.assertFalse(g.update([],0,0,10.3,10.3)[0])

    def test_threshold_jitter_cannot_clear(self):
        g=NearObstacleGuard();g.update([[.69,0]],0,0,10,10)
        for stamp in (10.1,10.2,10.3):self.assertTrue(g.update([[.73,0]],0,0,stamp,stamp)[0])

    def test_stopping_does_not_shrink_release_region(self):
        g=NearObstacleGuard();g.update([[.8,.45]],.18,.4,10,10.8)
        for stamp in (11.,11.1,11.2):self.assertTrue(g.update([[.8,.45]],0,0,stamp,stamp)[0])

    def test_invalid_input_fails_closed(self):
        g=NearObstacleGuard()
        with self.assertRaises(ValueError):g.update([],0,0,10,12)
        with self.assertRaises(ValueError):g.update([[float('nan'),0]],0,0,10,10)


class RampTests(unittest.TestCase):
    def test_acceleration_and_curvature(self):
        r=CurvatureRamp();prev=(0.,0.)
        for i in range(40):
            v,w=r.apply(.18,.3,10+i*.02)
            self.assertAlmostEqual(v*.3,w*.18)
            self.assertLessEqual(v-prev[0],.0060001)
            self.assertLessEqual(w-prev[1],.0140001)
            prev=(v,w)
        self.assertAlmostEqual(prev[0],.18)

    def test_stop_immediate_and_restart_from_zero(self):
        r=CurvatureRamp()
        for i in range(40):r.apply(.18,.3,10+i*.02)
        self.assertEqual(r.apply(0,0,11),(0.,0.))
        self.assertLessEqual(r.apply(.18,.3,11.02)[0],.006)

    def test_deceleration_not_delayed_and_reverse_crosses_zero(self):
        r=CurvatureRamp()
        for i in range(40):r.apply(.18,.3,10+i*.02)
        self.assertEqual(r.apply(.03,.05,10.8),(.03,.05))
        self.assertEqual(r.apply(-.03,.05,10.82),(0.,0.))

    def test_current_curve_only_and_long_gap_resets(self):
        r=CurvatureRamp()
        for i in range(40):r.apply(.18,0.,10+i*.02)
        v,w=r.apply(.1,.4,10.8)
        self.assertAlmostEqual(v*.4,w*.1)
        self.assertLessEqual(r.apply(.18,0.,13)[0],.006)


    def test_repeated_small_steering_corrections_keep_forward_progress(self):
        # 50 Hz output, a +/- .02 rad/s DWB correction every .1 second.
        # Previously each sign change reset BOTH velocities: only .121 m / 10 s,
        # below the configured .15 m progress requirement.
        r=CurvatureRamp();distance=0.;zeros=0
        for i in range(500):
            wanted_yaw=.02 if (i//5)%2==0 else -.02
            v,w=r.apply(.18,wanted_yaw,10+i*.02)
            distance+=v*.02
            zeros+=v==0
            self.assertAlmostEqual(v*wanted_yaw,w*.18)
            self.assertGreaterEqual(v,0.)
            self.assertLessEqual(v,.18)
        self.assertEqual(zeros,0)
        self.assertGreater(distance,1.)

    def test_angular_reversal_ramps_new_turn_without_whole_vehicle_stop(self):
        r=CurvatureRamp()
        for i in range(40):r.apply(.18,.3,10+i*.02)
        v,w=r.apply(.18,-.3,10.8)
        self.assertGreater(v,0.)
        self.assertLess(w,0.)
        self.assertLessEqual(abs(w),.0140001)
        self.assertAlmostEqual(v*-.3,w*.18)
        # A true forward/reverse change still crosses zero immediately.
        self.assertEqual(r.apply(-.18,-.3,10.82),(0.,0.))

    def test_zero_command_preempts_steering_transition(self):
        r=CurvatureRamp();r.apply(.18,.02,10.)
        r.apply(.18,-.02,10.02)
        self.assertEqual(r.apply(0.,0.,10.04),(0.,0.))
        self.assertIsNone(r.at)


class RampIntegrationTests(unittest.TestCase):
    def test_navigation_fault_bypasses_ramp_and_resets_restart(self):
        import time
        from nav2.tests import test_person_navigation as fixtures
        c=fixtures.PersonNavigationTests();c.setUp();n=c.n
        n.velocity_ramp=CurvatureRamp();n.enabled=True;n.command=(.18,.3);n.command_at=time.monotonic()
        v,w=n.velocity();self.assertGreater(v,0);self.assertLess(v,.18)
        n.healthy_pose.side_effect=RuntimeError('前方障碍')
        self.assertEqual(n.velocity(),(0.,0.));self.assertIsNone(n.velocity_ramp.at)
        self.assertFalse(n.enabled)

    def test_expired_controller_command_clears_ramp(self):
        import time
        from nav2.tests import test_person_navigation as fixtures
        c=fixtures.PersonNavigationTests();c.setUp();n=c.n
        n.velocity_ramp=CurvatureRamp();n.enabled=True;n.command=(.18,.3);n.command_at=time.monotonic()
        n.velocity();n.command_at-=1.
        self.assertEqual(n.velocity(),(0.,0.));self.assertIsNone(n.velocity_ramp.at)
