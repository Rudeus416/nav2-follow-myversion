#!/usr/bin/env python3
# 【R36/R37】保留离线时序模拟，并断言修复后的异步语义与暂停行为。
# 不加载推理模型，不创建ROS节点，不发布底盘消息，不改任何生产阈值。
"""Deterministic vision/command timing audit, NOT a robot physics simulation.

ROS message classes and fake action futures are used without any ROS transport.
Capture latency and arrival cadence are synthetic inputs, not field measurements.
The command-distance integral is not measured vehicle travel. Nav2's external
controller/progress checker, GPU inference and CAN hardware are not simulated.
"""
import json
import logging
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from nav_msgs.msg import OccupancyGrid
from builtin_interfaces.msg import Time
from nav2.depth_mailbox import DepthMailbox
from nav2.motion_geometry import CurvatureRamp
from nav2.semantic_dispatch import SemanticDispatch
from nav2.vision_epoch import VisionEpoch
from nav2.vision_layer import VisionLayer
from nav2.tests.test_point_mode_isolation import PointModeIsolationTests
from nav2.tests.test_replan_visual_pause import ReplanVisualPauseTests, FOOT


class Scenario:
    def __init__(self, mode):
        self.clock=100.
        self.patcher=patch('time.monotonic',side_effect=lambda:self.clock)
        self.patcher.start()
        if mode=='person':
            self.fixture=ReplanVisualPauseTests();self.fixture.setUp()
            # Its virtual clock remains linked to this scenario during all calls.
            self.fixture.now=self.clock
            self.nav=self.fixture.n
        else:
            self.fixture=PointModeIsolationTests();self.fixture.setUp()
            self.nav=self.fixture.nav
        n=self.nav;n.vision_enabled=True;n.vision_error='';n.vision_near_blocked=False
        n.obstacle_mode='map';n.velocity_ramp=CurvatureRamp()
        n.node=NS(get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:Time(
            sec=int(self.now()),nanosec=int((self.now()%1)*1e9)))))
        n.navigation_footprint=lambda:(FOOT,self.now())
        def healthy(check_vision=True):
            if check_vision:n.check_vision()
            return (0.,0.,0.)
        n.healthy_pose=healthy
        n.tf=NS(lookup_transform=lambda *args:NS(transform=NS(
            translation=NS(x=0.,y=0.),rotation=NS(x=0.,y=0.,z=0.,w=1.))))
        self.depth=NS(on_result=lambda record:None)
        self.box=DepthMailbox(self.depth)
        self.layer=layer=VisionLayer.__new__(VisionLayer)
        layer.node=n.node;layer.nav=n;layer.engine=NS(lock=threading.RLock(),
            _config_version=1,_stream_epoch=1,_buffer_m2={})
        layer._depth_mailbox=self.box
        layer._epoch_gate=VisionEpoch(layer.engine)
        # Deterministic empty semantic snapshot for the 14 virtual timelines.
        # The dedicated probe below exercises the real asynchronous dispatcher.
        layer.semantic=NS(set_enabled=lambda enabled:None,offer=lambda record:None,
                          snapshot=lambda:None,close=lambda:None)
        layer.kind=OccupancyGrid;layer.publisher=NS(publish=lambda msg:None)
        layer.height=.5;layer.pitch=0.;layer.scale=1.;layer.use_static=False
        layer.last=None;layer.cells={};layer.camera_cells={};layer.diagnostics={}
        layer.calibration={'camera_matrix':{'data':[100.,0,80,0,100,60,0,0,1]},
            'image_width':160,'image_height':120,'distortion_coefficients':{'data':[0.,0,0,0,0]}}
        self.fid=0;self.mode=mode

    def now(self):return self.clock

    def set_time(self,t):
        self.clock=t
        if self.mode=='person':self.fixture.now=t

    def deliver(self,latency):
        self.fid+=1;stamp=self.now()-latency
        record=NS(config_version=1,started_at=stamp+.05,completed_at=self.now(),
            depth=np.full((60,80),8.,np.float32),frame=NS(source_at=stamp,
                stamp_ns=int(stamp*1e9),stream_epoch=1,frame_id=self.fid,
                image=np.zeros((120,160,3),np.uint8)))
        self.depth.on_result(record)
        return record

    def tick(self):
        self.box.updated.clear();self.layer.tick()

    def close(self):
        self.layer.semantic.close()
        self.layer._epoch_gate.close()
        self.box.close()
        if self.mode=='person':self.fixture.doCleanups()
        self.patcher.stop()


def drive(mode, label, *, latency, period, seconds=12., dropout=None, canceled=False):
    sim=Scenario(mode);n=sim.nav
    samples=[];accepted=[];stopped_at=None;next_arrival=0.;next_command=0.
    try:
        sim.deliver(latency);sim.tick()
        assert not n.vision_error,n.vision_error
        start_stamp=n.vision_at
        next_arrival=period
        for i in range(int(seconds/.02)):
            t=i*.02;sim.set_time(100.+t)
            if t+1e-8>=next_arrival:
                if not dropout or not dropout[0]<=t<dropout[1]:sim.deliver(latency)
                next_arrival+=period
            if i%5==0 or sim.box.updated.is_set():
                sim.tick()
                if not accepted or n.vision_at!=accepted[-1]:accepted.append(n.vision_at)
            if canceled and i==100:n.stop('模拟用户取消')
            if t+1e-8>=next_command:
                # 20 Hz upstream commands; alternate turn direction every 3 s.
                yaw=.12 if int(t/3)%2==0 else -.12
                n._command(NS(linear=NS(x=.18),angular=NS(z=yaw)))
                next_command+=.05
            v,w=n.velocity()
            samples.append((t,v,w,n.enabled,t+100.-n.vision_at))
            if not n.enabled and stopped_at is None:stopped_at=round(t,3)
        zero=sum(abs(v)<1e-10 and abs(w)<1e-10 for _,v,w,_,_ in samples)
        moving=[r for r in samples if abs(r[1])+abs(r[2])>1e-10]
        # Every output is bounded and must use genuinely fresh evidence.
        assert all(age<=1.2+1e-9 and on for _,v,w,on,age in moving)
        assert all(abs(v)<=.18+1e-9 and abs(w)<=.4+1e-9 for _,v,w,_,_ in samples)
        if stopped_at is not None:
            assert all(abs(v)+abs(w)==0 for t,v,w,_,_ in samples if t>=stopped_at)
        result=dict(mode=mode,scenario=label,source_latency_s=latency,
            arrival_period_s=period,duration_s=seconds,
            zero_output_percent=round(100*zero/len(samples),2),
            command_distance_m=round(sum(v*.02 for _,v,_,_,_ in samples),4),
            safe_fresh_frames=len(accepted),canceled_at_s=stopped_at,
            stop_reason=getattr(n,'last_stop_reason',''),
            motion_after_cancel=False,stale_motion_outputs=0)
        if label=='fresh_5hz':assert stopped_at is None and zero==0
        if label=='brief_gap':assert stopped_at is None and zero>0
        if label in ('long_gap','user_cancel'):assert stopped_at is not None
        return result
    finally:sim.close()


def optional_semantic_delay():
    sim=Scenario('point')
    entered=threading.Event();release=threading.Event();completed=threading.Event()
    closed=threading.Event();worker_times={};dispatch=None;timer=None
    try:
        sim.deliver(.5);sim.tick()  # Establish a valid current safety sample.
        sim.set_time(100.5);r=sim.deliver(.95)
        observed_age=sim.now()-r.frame.source_at
        def delayed(record):
            worker_times.setdefault('started',time.perf_counter())
            entered.set()
            if not release.wait(2.):raise RuntimeError('Fake semantic worker release timed out')
            worker_times.setdefault('finished',time.perf_counter())
            completed.set()
            return None
        worker=NS(enabled=True,message='local synthetic worker',update=delayed,
                  status=lambda:dict(enabled=True,message='local synthetic worker',fresh=False,objects=[]),
                  close=closed.set)
        dispatch=SemanticDispatch(worker)
        sim.layer.semantic=dispatch
        dispatch.offer(r)
        assert entered.wait(1.),'Fake semantic update never started'
        timer=threading.Timer(.3,release.set);timer.start()
        old_stamp=sim.nav.vision_at
        started=time.perf_counter();sim.tick();tick_seconds=time.perf_counter()-started
        finished_while_semantic_blocked=not completed.is_set()
        # Keep the controller input fresh so an unrelated .3 s command timeout
        # cannot masquerade as a remaining semantic-stage stop.
        sim.nav._command(NS(linear=NS(x=.18),angular=NS(z=.12)))
        out=sim.nav.velocity()
        assert completed.wait(1.),'Fake semantic update did not finish after release'
        result=dict(scenario='optional_semantic_stage_300ms',
            age_before_semantic_s=round(observed_age,3),
            age_after_semantic_s=round(sim.now()-r.frame.source_at,3),
            injected_semantic_hold_s=.3,
            observed_semantic_hold_s=round(worker_times['finished']-worker_times['started'],6),
            tick_wall_seconds=round(tick_seconds,6),
            tick_finished_while_semantic_blocked=finished_while_semantic_blocked,
            fresh_result_accepted=sim.nav.vision_at==r.frame.source_at,
            prior_stamp_kept=sim.nav.vision_at==old_stamp,output=list(out))
        assert finished_while_semantic_blocked,'Optional semantic work blocked depth safety consumption'
        assert result['fresh_result_accepted'] and not result['prior_stamp_kept']
        assert not sim.nav.vision_error and sim.nav.enabled and out[0]>0
        return result
    finally:
        release.set()
        if timer is not None:timer.cancel();timer.join(timeout=1.)
        if dispatch is not None:
            dispatch.close(timeout=.2)
            dispatch._thread.join(timeout=1.)
            assert not dispatch._thread.is_alive(),'Semantic owner thread leaked from offline probe'
            assert closed.is_set(),'Semantic worker was not closed by its owner'
        sim.close()


def near_pause_conflict():
    entries=[]
    for first in ('velocity','visual'):
        sim=Scenario('person')
        try:
            sim.deliver(0.);sim.tick()
            n=sim.nav;task=n.person_navigation.obstacle_replan
            # Represents a detour that passed whole-body revalidation; the
            # remaining near guard is allowed only with directional checks.
            task._allow_near=True
            assert task.permits_near
            n.vision_near_blocked=True;n.vision_error='近距障碍保护：等待安全确认'
            def state():
                return dict(enabled=n.enabled,paused=getattr(n,'vision_paused',False),
                            fixed_goal=n.fixed_goal_active,detour_active=task.active,
                            stop_reason=getattr(n,'last_stop_reason',''))
            paths={'velocity':n.velocity,'visual':sim.tick}
            second='visual' if first=='velocity' else 'velocity'
            sim.set_time(101.3)
            states={}
            for entry in (first,second):
                output=paths[entry]()
                if entry=='velocity':assert output==(0.,0.)
                states[entry]=state()
                assert all(states[entry][key] for key in ('enabled','paused','fixed_goal','detour_active'))
            sim.set_time(102.01)
            paths[first]()
            expired=state()
            assert not expired['enabled'] and not expired['fixed_goal'] and not expired['detour_active']
            sim.deliver(0.);sim.tick()
            fresh_output=n.velocity()
            assert fresh_output==(0.,0.) and not n.enabled and not task.active
            entries.append(dict(first_entry=first,source_age_s=1.3,
                                paused_states=states,expired_source_age_s=2.01,
                                expired_state=expired,fresh_frame_after_cancel_output=list(fresh_output),
                                fresh_frame_did_not_revive=True))
        finally:sim.close()
    return dict(scenario='stale_near_hint_during_validated_detour',entry_orders=entries)


def main():
    logging.disable(logging.CRITICAL)
    rows=[]
    for mode in ('point','person'):
        for name,latency,period,drop in (
                ('fresh_5hz',.35,.2,None),
                ('slow_but_fresh',.6,.5,None),
                ('threshold_jitter',.9,.8,None),
                ('slow_1hz',1.05,1.,None),
                ('brief_gap',.35,.2,(3.,3.8)),
                ('long_gap',.35,.2,(3.,6.))):
            rows.append(drive(mode,name,latency=latency,period=period,dropout=drop))
        rows.append(drive(mode,'user_cancel',latency=.35,period=.2,canceled=True))
    report={'kind':'offline synthetic timing audit; no hardware IO',
            'fixed_behavior_assertions_passed':True,
            'scenarios':rows,'optional_stage_probe':optional_semantic_delay(),
            'near_pause_conflict':near_pause_conflict()}
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
