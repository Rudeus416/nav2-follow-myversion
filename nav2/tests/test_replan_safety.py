# 【内容标注】用途：绕行时全量视觉候选与命令车身扫掠的离线回归。
# 对应用户需求：R32；只验证几何输入/输出，无 ROS、相机、底盘或运动请求。
import math
import unittest
from types import SimpleNamespace as NS

import numpy as np

from nav2.near_obstacle import NearObstacleGuard
from nav2.replan_safety import command_clear, path_clear


FOOT = [[.297,.232],[.297,-.232],[-.297,-.232],[-.297,.232]]


def ros_path(poses):
    return NS(poses=[NS(pose=NS(position=NS(x=x,y=y),
                              orientation=NS(x=0.,y=0.,z=math.sin(a/2),w=math.cos(a/2))))
                     for x,y,a in poses])


class ReplanPathSafetyTests(unittest.TestCase):
    def test_detour_clears_original_obstacle_while_straight_path_is_blocked(self):
        points=[[1.,0.]]
        self.assertFalse(path_clear(ros_path([(0,0,0),(2,0,0)]),points,FOOT))
        detour=[(0,0,math.pi/2),(0,1,math.pi/2),(1,1,0),(2,1,0)]
        self.assertTrue(path_clear(ros_path(detour),points,FOOT))

    def test_sparse_path_cannot_jump_over_a_single_thin_pole(self):
        self.assertFalse(path_clear([(0,0,0),(3,0,0)],[[1.37,.05]],FOOT))

    def test_turning_corner_is_checked_between_safe_endpoint_footprints(self):
        points=[[-.36,.10]]
        self.assertTrue(path_clear([(0,0,0)],points,FOOT))
        self.assertTrue(path_clear([(0,0,math.pi/2)],points,FOOT))
        self.assertFalse(path_clear([(0,0,0),(0,0,math.pi/2)],points,FOOT))

    def test_passed_obstacle_can_be_skipped_but_actual_pose_bridge_is_checked(self):
        route=[(-1,0,0),(0,0,0),(1,0,0)]
        self.assertFalse(path_clear(route,[[-1,0]],FOOT))
        self.assertTrue(path_clear(route,[[-1,0]],FOOT,start_index=1,robot_pose=(0,0,0)))
        self.assertFalse(path_clear(route,[[-.5,0]],FOOT,start_index=1,robot_pose=(-1,0,0)))

    def test_all_objects_are_checked_even_when_nearest_object_is_off_route(self):
        route=[(0,0,0),(3,0,0)]
        self.assertTrue(path_clear(route,[[0.,.8]],FOOT))
        self.assertFalse(path_clear(route,[[0.,.8],[2.,0.]],FOOT))

    def test_cell_area_and_contact_are_blocked_not_only_cell_center(self):
        # Point is outside the padded body's top, but its entire 5 cm cell
        # reaches the top edge. Treating it as a zero-width point would miss it.
        self.assertFalse(path_clear([(0,0,0)],[[.1,.249]],FOOT))
        self.assertTrue(path_clear([(0,0,0)],[[.1,.31]],FOOT))

    def test_invalid_path_footprint_points_and_progress_fail_closed(self):
        valid=[(0,0,0),(1,0,0)]
        for path in ([],[(0,0,float('nan'))],[[0,0]],None):
            with self.subTest(path=path):self.assertFalse(path_clear(path,[],FOOT))
        for points in ([[float('nan'),0]],[[0,float('inf')]],[[1,2,3]]):
            with self.subTest(points=points):self.assertFalse(path_clear(valid,points,FOOT))
        for foot in ([],[[0,0],[1,0],[2,0]],[[0,0],[1,0],[float('inf'),1]]):
            with self.subTest(foot=foot):self.assertFalse(path_clear(valid,[],foot))
        for index in (-1,2,True,.5):
            self.assertFalse(path_clear(valid,[],FOOT,start_index=index))
        bad=ros_path(valid);bad.poses[0].pose.orientation.w=0.
        self.assertFalse(path_clear(bad,[],FOOT))

    def test_existing_padded_body_clearance_is_not_inflated_a_second_time(self):
        from nav2.path_clearance import validate_path
        # Already-padded half-width .232 m clears cell edge y=.25 by 1.8 cm.
        # The prior extra .02 here rejected it even though the native planner and
        # normal whole-body validator use exactly this padded footprint.
        route=ros_path([(0.,0.,0.),(1.,0.,0.)])
        point=[.51,.26]
        data=np.zeros((100,100),dtype=int)
        x,y=np.floor((np.asarray(point)+2.)/.05).astype(int)
        data[y,x]=100
        grid=dict(resolution=.05,width=100,height=100,origin=[-2.,-2.],yaw=0.,data=data)
        validate_path(route,grid,FOOT)
        self.assertTrue(path_clear(route,[point],FOOT))

    def test_no_collision_missed_against_existing_full_body_validator(self):
        from nav2.path_clearance import validate_path
        rng=np.random.default_rng(32)
        blocked=0
        for _ in range(80):
            poses=[(1.75,1.75,float(rng.uniform(-math.pi,math.pi))),
                   (float(rng.uniform(1.25,2.25)),float(rng.uniform(1.25,2.25)),
                    float(rng.uniform(-math.pi,math.pi)))]
            points=rng.uniform(.8,2.7,size=(5,2))
            data=np.zeros((80,80),dtype=int)
            cells=np.floor(points/.05).astype(int)
            data[cells[:,1],cells[:,0]]=100
            grid=dict(resolution=.05,width=80,height=80,origin=[0.,0.],yaw=0.,data=data)
            path=ros_path(poses)
            try:validate_path(path,grid,FOOT)
            except ValueError:
                blocked+=1
                # Full occupied cells and rotation sweep must retain old collision rejection.
                self.assertFalse(path_clear(path,points,FOOT))
        self.assertGreater(blocked,10)

    def test_empty_fresh_observations_are_clear_not_an_obstacle(self):
        self.assertTrue(path_clear([(0,0,0),(1,0,0)],[],FOOT))


class ReplanCommandSafetyTests(unittest.TestCase):
    def check(self, points, linear=.18, yaw=0., age=.8, pose=(0.,0.,0.)):
        return command_clear(points,FOOT,pose,linear,yaw,10.,10.+age)

    def test_forward_collision_stops_but_clear_turn_is_allowed(self):
        points=[[.42,0.]]
        self.assertTrue(NearObstacleGuard().update(points,0.,0.,10.,10.)[0])
        self.assertFalse(self.check(points))
        self.assertTrue(self.check(points,linear=0.,yaw=.4))

    def test_pure_turn_cannot_sweep_a_rear_corner_through_an_obstacle(self):
        # The original forward rectangle does not cover rear/side points.
        points=[[-.36,.10]]
        self.assertFalse(NearObstacleGuard().update(points,0.,0.,10.,10.)[0])
        self.assertFalse(self.check(points,linear=0.,yaw=.4,age=1.))

    def test_source_delay_extends_required_stopping_space(self):
        points=[[.45,0.]]
        self.assertTrue(self.check(points,age=0.))
        self.assertFalse(self.check(points,age=1.))

    def test_braking_travel_is_checked_after_reaction_interval(self):
        # 0.18 m/s * 0.1 s alone is safe; added stopping distance reaches cell.
        foot=[[.27,.232],[.27,-.232],[-.27,-.232],[-.27,.232]]
        self.assertFalse(command_clear([[.34,0.]],foot,(0.,0.,0.),.18,0.,10.,10.))
        self.assertTrue(command_clear([[.34,0.]],foot,(0.,0.,0.),.03,0.,10.,10.))

    def test_clock_regression_stale_and_nonfinite_inputs_are_not_motion_permission(self):
        for age in (-.01,1.201,float('nan'),float('inf')):
            with self.subTest(age=age):self.assertFalse(self.check([],age=age))
        self.assertFalse(self.check([],linear=float('nan')))
        self.assertFalse(self.check([],yaw=float('inf')))
        self.assertFalse(self.check([],pose=(0.,0.,float('nan'))))
        self.assertFalse(self.check([[float('nan'),0.]]))

    def test_map_external_points_are_kept_and_world_pose_transform_is_used(self):
        # World x=12 is outside the project's 10 m local window centered at 0.
        # The helper cannot clip such points or treat them as free space.
        self.assertFalse(self.check([[12.42,0.]],pose=(12.,0.,0.)))
        self.assertFalse(self.check([[12.,.42]],pose=(12.,0.,math.pi/2)))
        self.assertTrue(self.check([[12.,.42]],pose=(12.,0.,-math.pi/2)))

    def test_reverse_motion_checks_behind_the_car(self):
        self.assertFalse(self.check([[-.42,0.]],linear=-.18))
        self.assertTrue(self.check([[-.42,0.]],linear=.18))

    def test_whole_arc_checks_collisions_that_a_centerline_test_misses(self):
        self.assertFalse(self.check([[.40,.20]],linear=.18,yaw=.4,age=1.))
        self.assertTrue(self.check([[.40,.75]],linear=.18,yaw=.4,age=1.))

    def test_inputs_are_not_mutated_or_replaced_by_nearest_only(self):
        points=np.asarray([[.1,.8],[.42,0.]],float)
        foot=np.asarray(FOOT,float)
        before=points.copy();body=foot.copy()
        self.assertFalse(self.check(points))
        self.assertTrue(np.array_equal(points,before))
        self.assertTrue(np.array_equal(foot,body))

    def test_zero_command_still_detects_an_existing_body_overlap(self):
        self.assertFalse(self.check([[0.,0.]],linear=0.,yaw=0.))
        self.assertTrue(self.check([],linear=0.,yaw=0.))


if __name__=='__main__':
    unittest.main()
