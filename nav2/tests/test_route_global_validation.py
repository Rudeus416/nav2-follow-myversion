# 【内容标注 / R13 R20 R22】长地图整路线复核回归：不启动 ROS 或实车。
# 重现局部窗越界、校验完整全局障碍，并验证缺图/过期/图外仍拒绝。
import copy
import math
import unittest
from contextlib import ExitStack
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import numpy as np
from nav2.path_clearance import validate_path
from nav2.route_preview import attach, validate_route_snapshot, route_snapshot


FOOT=np.array([[.297,.232],[.297,-.232],[-.297,-.232],[-.297,.232]])


def grid(width,height,origin):
    return dict(width=width,height=height,resolution=.05,origin=origin,yaw=0.,
                data=[0]*(width*height),stale=False)


def route(x=6.7,y=.22):
    return NS(poses=[NS(pose=NS(position=NS(x=px,y=py),orientation=NS(
        x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2))))
        for px,py,yaw in [(0.,0.,0.),(x,y,math.atan2(y,x))]])


def exact_validation(path,grids,footprint):
    for g in grids:validate_path(path,g,footprint)


def occupy(g,x,y,value):
    ix=math.floor((x-g['origin'][0])/g['resolution'])
    iy=math.floor((y-g['origin'][1])/g['resolution'])
    g['data'][iy*g['width']+ix]=value


class GlobalRouteValidationTests(unittest.TestCase):
    def setUp(self):
        # Same physical 8.4 x 2.2 m corridor, expressed here as an odom-aligned
        # synthetic grid. Static-map rotation is covered by the geometry suite.
        static=grid(168,44,[-.3,-.95])
        static['data'][:168]=[100]*168;static['data'][-168:]=[100]*168
        self.snapshot={'layers':{
            'static_map':static,
            'map':grid(200,200,[-4.95,-4.95]),
            'global_map':grid(360,200,[-8.95,-4.95]),
            'robot':dict(position=[0.,0.],yaw=0.,stale=False),
            'footprint':dict(points=FOOT.tolist(),stale=False),
        }}
        self.exact=patch('nav2.clearance_worker.validate_isolated',side_effect=exact_validation)
        self.exact.start();self.addCleanup(self.exact.stop)

    def test_reported_endpoint_and_far_route_pass_global_while_local_rejects(self):
        for x in (4.78,6.7):
            with self.subTest(goal=x):
                path=route(x)
                with self.assertRaisesRegex(ValueError,'地图边界'):
                    validate_path(path,self.snapshot['layers']['map'],FOOT)
                validate_route_snapshot(path,self.snapshot)

    def test_obstacle_purple_and_unknown_beyond_local_window_still_reject(self):
        for cost in (100,98,1,-1):
            with self.subTest(cost=cost):
                snapshot=copy.deepcopy(self.snapshot)
                occupy(snapshot['layers']['global_map'],6.,.20,cost)
                with self.assertRaisesRegex(ValueError,'障碍|缓冲'):
                    validate_route_snapshot(route(),snapshot)

    def test_static_edit_remains_blocking_even_if_global_has_not_received_it(self):
        occupy(self.snapshot['layers']['static_map'],5.7,.20,100)
        with self.assertRaisesRegex(ValueError,'障碍|禁区'):
            validate_route_snapshot(route(),self.snapshot)

    def test_larger_global_does_not_authorize_driving_outside_static_map(self):
        with self.assertRaisesRegex(ValueError,'地图边界'):
            validate_route_snapshot(route(8.),self.snapshot)

    def test_missing_stale_global_does_not_fall_back_to_local(self):
        for value in (None,{'stale':True}):
            with self.subTest(global_map=value):
                self.snapshot['layers']['global_map']=value
                with self.assertRaisesRegex(ValueError,'全局代价地图'):
                    validate_route_snapshot(route(),self.snapshot)

    def test_stale_pose_footprint_or_static_map_still_reject(self):
        for name in ('robot','footprint','static_map'):
            with self.subTest(layer=name):
                snapshot=copy.deepcopy(self.snapshot);snapshot['layers'][name]['stale']=True
                with self.assertRaises(ValueError):validate_route_snapshot(route(),snapshot)

    def test_isolated_worker_also_accepts_complete_long_route(self):
        # One real subprocess check verifies serialization of the larger global
        # snapshot and use of the same full-body checker across both maps.
        self.exact.stop()
        validate_route_snapshot(route(),self.snapshot)
        self.exact.start()

    def test_planner_release_can_wait_for_next_fresh_full_global_snapshot(self):
        stale=copy.deepcopy(self.snapshot);stale['layers']['global_map']['stale']=True
        view=Mock();view.snapshot.side_effect=[stale,self.snapshot]
        clock=[0.]
        with patch('nav2.route_preview.time.monotonic',side_effect=lambda:clock[0]), \
                patch('nav2.route_preview.time.sleep',side_effect=lambda dt:clock.__setitem__(0,clock[0]+dt)):
            snapshot=route_snapshot(view)
        self.assertIs(snapshot,self.snapshot)
        self.assertAlmostEqual(clock[0],.05)
        validate_route_snapshot(route(),snapshot)

    def test_global_wait_has_deadline_and_never_uses_stale_data(self):
        self.snapshot['layers']['global_map']['stale']=True
        view=Mock();view.snapshot.return_value=self.snapshot
        clock=[0.]
        with patch('nav2.route_preview.time.monotonic',side_effect=lambda:clock[0]), \
                patch('nav2.route_preview.time.sleep',side_effect=lambda dt:clock.__setitem__(0,clock[0]+dt)):
            snapshot=route_snapshot(view)
        self.assertAlmostEqual(clock[0],.8)
        with self.assertRaisesRegex(ValueError,'全局代价地图'):
            validate_route_snapshot(route(),snapshot)

    def test_attached_validator_requests_private_global_snapshot_in_map_modes(self):
        app=Mock();view=Mock();view.snapshot.return_value=self.snapshot
        nav=NS(obstacle_mode='map');motion=NS(navigation=nav)
        with ExitStack() as stack:
            for module in ('vision_toggle','inflation_control','continuous_segment','person_map','point_navigation'):
                stack.enter_context(patch('nav2.'+module+'.attach'))
            attach(app,Mock(),motion,view)
        with patch('nav2.route_preview.validate_route_snapshot') as validate:
            for mode in ('map','both'):
                nav.obstacle_mode=mode;view.snapshot.reset_mock()
                path=route();nav.validate_preview(path)
                view.snapshot.assert_called_once_with(include_global=True)
                validate.assert_called_with(path,self.snapshot)
            nav.obstacle_mode='radar';view.snapshot.reset_mock();validate.reset_mock()
            nav.validate_preview(route())
            view.snapshot.assert_not_called();validate.assert_not_called()


if __name__=='__main__':unittest.main()
