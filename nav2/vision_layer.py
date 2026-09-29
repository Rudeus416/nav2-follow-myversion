# 【内容标注】用途：视觉障碍候选与代价地图生成。
# 对应用户需求：R08 R09 R10 R19 R22 R32 R37 R38 R39 R40（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：串行读取深度；变换/筛选最近候选；合并静态墙；独立使用全部候选做近距停车。
# 需求关联用于追溯用途，不代表精确创建/提交记录。
"""Optional, estimated-depth obstacle map. Does not own chassis commands."""
import math
import logging
import threading
import os
import time
from pathlib import Path
import cv2
import numpy as np
import yaml


# 【职责 / R08 R09 R10 R19 R22】stable_grid_origin：固定局部视觉栅格原点直到车位离开中间区域，减少反复分配。
def stable_grid_origin(x, y, previous=None):
    """10 m visual grid: move its origin only outside the central 5 m window.

    Grid content still updates every fresh frame. Keeping its geometry stable
    avoids StaticLayer reallocating/copying the full map at every 5 cm step.
    The 2.5 m edge reserve exceeds the 2 s local trajectory at 0.15 m/s.
    """
    cx,cy=math.floor(x/.05),math.floor(y/.05)
    if previous is not None:
        ox,oy=previous
        if 50<=cx-ox<150 and 50<=cy-oy<150:
            return previous
    return cx-100,cy-100


# 【职责 / R08 R09 R10 R19 R22】nearest_fragments：保留最近且深度相近的连通碎片，不填补未观察到的间隙。
def nearest_fragments(labels, forward, lateral):
    """R38: exact grouped statistics; avoid one percentile scan per fragment."""
    from nav2.depth_groups import nearest_fragments as grouped_fragments
    return grouped_fragments(labels,forward,lateral)


# 【职责 / R10 R22】depth_components：按图像八邻接及相邻深度连续性分组；避免近物和背景因像素相连合为一体。
def depth_components(mask, forward, max_step=.30):
    """Label observed pixels connected in both image space and forward depth.

    R10/R22: a cart and a distant wall can touch in the image without touching
    in space. Split a depth jump larger than the existing .30 m fragment-depth
    allowance, while retaining slanted/irregular surfaces with continuous depth.
    Only existing pixels are grouped; no holes or unseen obstacle cells are filled.
    """
    height,width=mask.shape
    indices=np.arange(mask.size).reshape(mask.shape)
    parents=list(range(mask.size))

    def root(index):
        while parents[index]!=index:
            parents[index]=parents[parents[index]]
            index=parents[index]
        return index

    # Each undirected edge is visited once, including both diagonals. Vectorized
    # edge eligibility keeps work bounded on the existing 80 x 60 depth grid.
    for source,target in (
            ((slice(None),slice(None,-1)),(slice(None),slice(1,None))),
            ((slice(None,-1),slice(None)),(slice(1,None),slice(None))),
            ((slice(None,-1),slice(None,-1)),(slice(1,None),slice(1,None))),
            ((slice(None,-1),slice(1,None)),(slice(1,None),slice(None,-1)))):
        adjacent=mask[source]&mask[target]
        difference=np.full(adjacent.shape,np.inf)
        np.subtract(forward[source],forward[target],out=difference,where=adjacent)
        connected=adjacent&(np.abs(difference)<=max_step)
        for first,second in zip(indices[source][connected].tolist(),indices[target][connected].tolist()):
            first,second=root(first),root(second)
            if first!=second:parents[second]=first
    labels=np.zeros(height*width,np.int32)
    for index in np.flatnonzero(mask).tolist():labels[index]=root(index)+1
    return labels.reshape(mask.shape)


# 【职责 / R08 R09 R10 R19 R22】depth_points：从深度和相机参数生成候选，过滤范围并可用语义实例辅助选择。
def depth_points(depth, shape, calibration, height, pitch, scale, accept=None, nearest_only=True, diagnostics=None, instances=None, prepared=None):
    """R40: preserve the public selector; share preprocessing only within a tick."""
    from nav2.depth_projection import DepthProjection
    projection=(prepared if prepared is not None else
                DepthProjection(depth,shape,calibration,height,pitch,scale))
    return projection.points(accept=accept,nearest_only=nearest_only,
        diagnostics=diagnostics,instances=instances,
        split_components=depth_components,nearest_fragments=nearest_fragments)


# 【职责 / R08 R09 R10 R19 R22】inside_static_map：按旋转后的静态图范围筛选点，避免图外检测写进导航地图。
def inside_static_map(world, grid, transform):
    """Test odom points against the rotated source map extent."""
    def yaw(q):
        return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    a=yaw(transform.rotation)
    c,s=math.cos(a),math.sin(a)
    local=(world-np.array([transform.translation.x,transform.translation.y]))@np.array([[c,-s],[s,c]])
    local-=np.array([grid.info.origin.position.x,grid.info.origin.position.y])
    a=yaw(grid.info.origin.orientation);c,s=math.cos(a),math.sin(a)
    local=local@np.array([[c,-s],[s,c]])
    return ((local[:,0]>=0)&(local[:,1]>=0)&
            (local[:,0]<grid.info.width*grid.info.resolution)&
            (local[:,1]<grid.info.height*grid.info.resolution))


# 【职责 / R08 R09 R10 R19 R22】merge_static_walls：保守栅格化静态占用，避免视觉层覆盖掉原墙。
def merge_static_walls(data, origin, resolution, grid, transform):
    """Conservatively rasterize occupied static cells into the odom grid."""
    def yaw(q):
        return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    a=yaw(transform.rotation); b=yaw(grid.info.origin.orientation)
    c,s=math.cos(a),math.sin(a)
    px,py=grid.info.origin.position.x,grid.info.origin.position.y
    offset=np.array([transform.translation.x+c*px-s*py,transform.translation.y+s*px+c*py])
    c,s=math.cos(a+b),math.sin(a+b)
    rotation=np.array([[c,s],[-s,c]])
    indices=np.argwhere(np.asarray(grid.data).reshape(grid.info.height,grid.info.width)>=65)
    for y,x in indices:
        corners=np.array([[x,y],[x+1,y],[x+1,y+1],[x,y+1]],float)
        # Slightly inset corners avoid including an entire extra cell on exact edges.
        corners=(corners-(np.array([x,y])+.5))*.999+(np.array([x,y])+.5)
        world=corners*grid.info.resolution@rotation+offset
        pixels=np.floor((world-np.array(origin))/resolution).astype(np.int32)
        cv2.fillConvexPoly(data,pixels,100)
    return data


# 【职责 / R08 R09 R10 R19 R22】VisionLayer：视觉障碍候选与代价地图生成的状态封装；各方法职责见下方标注。
class VisionLayer:
    # 【职责 / R08 R09 R10 R19 R22】__init__：初始化本类依赖与状态；副作用以原初始化语句为准。
    def __init__(self,node,engine,navigation):
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from nav2.depth_profile import DepthProfile
        # Reject bad environment values before publishers, workers or wrappers exist.
        depth_limit=DepthProfile.validate_limit(int(os.environ.get('NAV2_DEPTH_IMGSZ','512')))
        self.node,self.engine,self.nav=node,engine,navigation
        self.height=float(os.environ.get('NAV2_VISION_HEIGHT','0.5'))
        self.pitch=float(os.environ.get('NAV2_VISION_PITCH','0'))
        self.scale=float(os.environ.get('NAV2_VISION_SCALE','1'))
        if not (.05<=self.height<=3 and -60<=self.pitch<=60 and .05<=self.scale<=10):
            raise ValueError('NAV2 视觉相机高度、俯角或深度比例无效')
        self.calibration=yaml.safe_load((Path(__file__).resolve().parent.parent/'camera_calibration.yaml').read_text())
        self.kind=OccupancyGrid
        self.publisher=node.create_publisher(OccupancyGrid,'/visual_car/vision_costmap',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.last=None;self.cells={};self.camera_cells={};self.diagnostics={};self.version=None
        from nav2.semantic_dispatch import SemanticDispatch
        self.semantic=SemanticDispatch()
        self.semantic_preview=None
        self.static_map=None
        self.use_static=navigation.obstacle_mode in ('map','both')
        if self.use_static:
            self.static_sub=node.create_subscription(OccupancyGrid,'/visual_car/edited_map',self.receive_static,
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        from nav2.depth_mailbox import DepthMailbox
        self._depth_mailbox=DepthMailbox(engine.depth)
        navigation.vision_layer=self
        self._revision=0
        from nav2.vision_epoch import VisionEpoch
        self._epoch_gate=VisionEpoch(engine,self._configuration_changed)
        # R39：只调整启用 Nav2 视觉避障时的新深度任务副本，保留原配置和帧。
        self.depth_profile=DepthProfile(engine.depth,
            enabled=lambda:bool(self.nav.vision_enabled),limit=depth_limit)
        # 不占用原相机的 SingleThreadedExecutor。只有一个串行消费者，
        # 每轮直接读取最新结果，不堆积定时器任务，也不更改原相机/深度代码。
        self._worker_stop=threading.Event()
        from nav2.route_monitor import RouteMonitor
        self._route_monitor=RouteMonitor(self.nav,context=self._route_context)
        try:
            node.context.on_shutdown(self._worker_stop.set)
            self._worker=threading.Thread(target=self._run,name='nav2-vision-update',daemon=True)
            self._worker.start()
        except BaseException:
            # R39：构造失败没有 _run.finally，仍须恢复自己安装的所有入口。
            self._worker_stop.set()
            self._route_monitor.close()
            self.depth_profile.close()
            self._depth_mailbox.close()
            self._epoch_gate.close()
            self.semantic.close()
            if getattr(navigation,'vision_layer',None) is self:
                navigation.vision_layer=None
            raise

    # 【职责 / R08 R09 R10 R19 R22】_run：单个串行工作线程消费最新数据，避免任务堆积。
    def _run(self):
        try:
            while not self._worker_stop.is_set():
                engine_stop=getattr(self.engine,'_stop',None)
                if engine_stop is not None and engine_stop.is_set():
                    break
                mailbox=getattr(self,'_depth_mailbox',None)
                wake=mailbox.updated if mailbox is not None else self._worker_stop
                # R35: clear BEFORE reading; arrivals during tick must trigger
                # another pass. A 100 ms timeout still checks health if frames stop.
                if mailbox is not None:wake.clear()
                started=time.monotonic()
                try:
                    self.tick()
                except Exception as exc:
                    # 工作线程不能静默退出并留下旧的可行驶状态。
                    logging.getLogger(__name__).exception('视觉更新线程异常')
                    with self.nav.lock:
                        self.nav.vision_at=0.
                        self.nav.vision_error='视觉更新异常：'+str(exc)
                        self.nav.stop(self.nav.vision_error)
                wake.wait(max(0.,.1-(time.monotonic()-started)))
        finally:
            self._worker_stop.set()
            monitor=getattr(self,'_route_monitor',None)
            if monitor is not None:monitor.close()
            profile=getattr(self,'depth_profile',None)
            if profile is not None:profile.close()
            # YOLO 子进程只由当前消费者关闭，防止 update/close 并发。
            mailbox=getattr(self,'_depth_mailbox',None)
            if mailbox is not None:
                mailbox.close()
            self.semantic.close()
            gate=getattr(self,"_epoch_gate",None)
            if gate is not None:gate.close()

    # 【职责 / R08 R09 R10 R19 R22】close：停止自有工作线程并释放接入资源。
    def close(self):
        # Stop applying the optional profile even if a visual tick is still exiting.
        profile=getattr(self,'depth_profile',None)
        if profile is not None:profile.close()
        self._worker_stop.set()
        monitor=getattr(self,'_route_monitor',None)
        if monitor is not None:monitor.close()
        mailbox=getattr(self,'_depth_mailbox',None)
        if mailbox is not None:mailbox.updated.set()
        if self._worker is not threading.current_thread():
            self._worker.join(timeout=3.)
        if self._worker.is_alive():
            logging.getLogger(__name__).warning('视觉线程仍在退出，保持停车')
            with self.nav.lock:
                self.nav.vision_at=0.
                self.nav.vision_error='视觉线程正在退出'
                self.nav.stop(self.nav.vision_error)

    # R37：仅本接入层保存失效代际；不改原相机/检测代码。
    def invalidate(self):
        """Call under nav.lock when a visual input/configuration changes."""
        self._revision=getattr(self,'_revision',0)+1
        self.last=None
        self._safety_frame_key=None
        self.nav.vision_at=0.
        self.nav.vision_safety=None
        guard=getattr(self,'near_guard',None)
        if guard is not None:
            # New configurations need three new clear frames, not a mixed history.
            guard.clear_count=0;guard.last_stamp=None

    def _configuration_changed(self):
        # Gate becomes not-ready BEFORE this callback; commits recheck it.
        with self.nav.lock:
            self.invalidate()
            self.semantic.set_enabled(False)
            if self.nav.vision_enabled:
                self.nav.vision_error='视觉配置已改变，等待新配置深度帧'
                self.nav.stop(self.nav.vision_error)
        self._depth_mailbox.updated.set()

    def receive_static(self, msg):
        # R37：相同地图的周期发布不打断处理；真实编辑使进行中的计算失效。
        info=msg.info;pos=info.origin.position;q=info.origin.orientation
        signature=(msg.header.frame_id,info.width,info.height,info.resolution,
                   pos.x,pos.y,q.x,q.y,q.z,q.w,bytes(np.asarray(msg.data,np.int8)))
        with self.nav.lock:
            if signature==getattr(self,'_map_signature',None):return
            self._map_signature=signature
            self.static_map=msg
            self.invalidate()

    # R37：邮箱读取不等融合锁；配置由独立的小型代际门控检查。
    def tick(self):
        tick_started=time.monotonic()
        tick_gap=tick_started-getattr(self,'_previous_tick',tick_started)
        self._previous_tick=tick_started
        gate=getattr(self,'_epoch_gate',None)
        snapshot=gate.snapshot() if gate is not None else None
        if gate is not None and snapshot is None:
            self.semantic.set_enabled(False)
            return  # 配置切换入口已经失效旧证据并停车。
        records=[]
        if snapshot is not None:
            version,epoch=snapshot.version,snapshot.epoch
        else:
            # Static test/legacy adapters without a gate must never block either.
            if not self.engine.lock.acquire(blocking=False):return
            try:
                version=self.engine._config_version
                epoch=getattr(self.engine,'_stream_epoch',None)
            finally:self.engine.lock.release()
        mailbox=getattr(self,'_depth_mailbox',None)
        direct=mailbox.latest(version,epoch) if mailbox is not None else None
        if direct is not None:records.append(direct)
        # Compatibility fallback: a busy fusion loop cannot hold up a ready mailbox.
        if self.engine.lock.acquire(blocking=False):
            try:
                if (self.engine._config_version==version
                        and getattr(self.engine,'_stream_epoch',None)==epoch):
                    records.extend(self.engine._buffer_m2.values())
            finally:self.engine.lock.release()
        records=[r for r in records if r.config_version==version
                 and (epoch is None or r.frame.stream_epoch==epoch)]
        self._processing_epoch=snapshot
        engine_done=time.monotonic()
        latest=max(records,key=lambda r:r.frame.source_at) if records else None
        first_read=time.monotonic()
        timing_key=((latest.config_version,latest.frame.stream_epoch,latest.frame.frame_id)
                    if latest is not None else None)
        new_frame=timing_key is not None and timing_key!=getattr(self,'_timing_frame_key',None)
        arrival=mailbox.arrival(latest) if mailbox is not None and hasattr(mailbox,'arrival') else None
        with self.nav.lock:
            enabled=self.nav.vision_enabled
        # R37：模型启动、图像缩放、收包和关闭都在另一个串行 owner。
        self.semantic.set_enabled(enabled)
        if enabled and latest is not None:self.semantic.offer(latest)
        packet=self.semantic.snapshot() if enabled else None
        # 安全检测始终使用最新深度帧。异步 YOLO 旧结果不能让时间戳倒退，
        # 也不能把旧掩膜贴到新图像；只有完全同帧时才参与候选分组。
        objects=None
        if packet and latest is not None:
            semantic_record=packet[0]
            if (semantic_record.config_version==latest.config_version
                    and semantic_record.frame.stream_epoch==latest.frame.stream_epoch
                    and semantic_record.frame.frame_id==latest.frame.frame_id):
                objects=packet[1]
        semantic_done=time.monotonic()
        # R33: duplicate/expired frames cannot produce new safety evidence, so
        # do not reread map/TF for them. The cached tuple retains its source age.
        # A fresh frame still prepares recovery before a person route is armed.
        safety_footprint = getattr(self, '_safety_footprint', (None, 0.))
        safety_key = ((latest.config_version, latest.frame.stream_epoch, latest.frame.frame_id)
                      if latest is not None else None)
        if (enabled and latest is not None
                and 0 <= time.monotonic()-latest.frame.source_at <= 1.2
                and safety_key != getattr(self, '_safety_frame_key', None)):
            try:
                safety_footprint = self.nav.navigation_footprint()
            except Exception:
                safety_footprint = (None, 0.)
            self._safety_frame_key = safety_key
        lock_done=time.monotonic()
        self._safety_footprint = safety_footprint
        # R40: current-tick timings only; no stale phase values after an early return.
        self._stage_timing={}
        # _tick_locked retains its historical name, but manages only short locks.
        self._tick_locked([latest] if latest is not None else [], version, objects)
        visual_done=time.monotonic()
        from nav2.replan_support import recovery
        task=recovery(self.nav)
        monitor=getattr(self,'_route_monitor',None)
        if task is not None and task.active:
            if monitor is None:
                # Snapshot-only/offline adapters also get a single owned worker.
                from nav2.route_monitor import RouteMonitor
                monitor=self._route_monitor=RouteMonitor(self.nav,context=self._route_context)
            monitor.offer()
        finished=time.monotonic()
        self._stage_timing['route_offer']=round(finished-visual_done,4)
        route_timing=monitor.snapshot() if monitor is not None else {}
        age=finished-latest.frame.source_at if latest is not None else float('inf')
        if latest is not None:
            inference_started=getattr(latest,'started_at',None)
            inference_done=getattr(latest,'completed_at',None)
            timing={
                'frame_id':latest.frame.frame_id,
                # Actual size requested for this result, including any in-flight old task.
                'depth_imgsz':getattr(getattr(latest,'settings',None),'depth_imgsz',None),
                'source_age':round(age,3),
                'input_wait':round(inference_started-latest.frame.source_at,3) if inference_started is not None else None,
                'inference':round(inference_done-inference_started,3) if inference_started is not None and inference_done is not None else None,
                'after_inference':round(finished-inference_done,3) if inference_done is not None else None,
                'direct_result':latest is direct,
                'new_frame':new_frame,
                'tick_seconds':round(finished-tick_started,3),
                'phases':dict(getattr(self,'_stage_timing',{})),
                'route_monitor':route_timing,
                'footprint_seconds':round(lock_done-semantic_done,4),
            }
            if new_frame:
                self._timing_frame_key=timing_key
                timing.update(first_read_age=round(first_read-latest.frame.source_at,3),
                    first_read_after_inference=(round(first_read-inference_done,3)
                                               if inference_done is not None else None),
                    result_delivery=(round(arrival[0]-inference_done,3)
                                     if arrival is not None and inference_done is not None else None),
                    mailbox_wait=(round(first_read-arrival[0],3) if arrival is not None else None),
                    producer_interval=(round(arrival[1],3)
                                       if arrival is not None and arrival[1] is not None else None))
                # Unlike the old stale polling log, this measures FIRST use.
                if finished-getattr(self,'_last_first_frame_log',0.)>5.:
                    self._last_first_frame_log=finished
                    logging.getLogger(__name__).warning('视觉新帧首次消费：%s',timing)
            with self.nav.lock:
                self.timing=timing
        else:
            timing={}
        if age>1.2 and finished-getattr(self,'_last_delay_log',0.)>5.:
            self._last_delay_log=finished
            logging.getLogger(__name__).warning(
                '视觉更新延迟：源帧年龄 %.3f 秒，读取深度缓冲 %.3f 秒，语义阶段 %.3f 秒，'
                '车身准备 %.3f 秒，视觉阶段 %.3f 秒，回调间隔 %.3f 秒，入口帧龄 %.3f 秒',
                age,engine_done-tick_started,semantic_done-engine_done,lock_done-semantic_done,
                finished-lock_done,tick_gap,
                engine_done-latest.frame.source_at if latest is not None else float('inf'))
            logging.getLogger(__name__).warning('视觉深度分段耗时：%s',timing)

    def _route_context(self):
        """R40: cheap ownership/configuration token, checked under nav.lock."""
        if not self._work_current(None):return None
        if getattr(self.nav,'vision_layer',self) is not self:return None
        engine_stop=getattr(self.engine,'_stop',None)
        if engine_stop is not None and engine_stop.is_set():return None
        gate=getattr(self,'_epoch_gate',None)
        epoch=gate.snapshot() if gate is not None else None
        if gate is not None and epoch is None:return None
        return (getattr(self,'_revision',0),epoch)

    def _work_current(self, context):
        """R37: called under nav.lock; never waits for the engine/semantic worker."""
        stopping=getattr(self,'_worker_stop',None)
        if stopping is not None and stopping.is_set():return False
        if not self.nav.vision_enabled or getattr(self,'switch_failed',False):return False
        if context is None:return True
        revision,generation,epoch=context
        gate=getattr(self,'_epoch_gate',None)
        return (revision==getattr(self,'_revision',0)
                and generation==getattr(self.nav,'generation',0)
                and (gate is None or gate.current(epoch)))

    @staticmethod
    def _fresh_record(record):
        age=time.monotonic()-record.frame.source_at
        if not math.isfinite(age) or age<0:
            raise ValueError('视觉障碍源帧时间无效')
        if age>1.2:
            from nav2_follow import VisionStale
            raise VisionStale('视觉障碍数据超时，停车等待新数据')

    # R37：分阶段更新。TF、深度几何、栅格和发布均不持速度控制锁。
    def _tick_locked(self, records, version, objects=None):
        context=None
        stages=self._stage_timing={}
        stage_at=time.monotonic()
        def mark(name):
            # R40: local phase clocks include lock wait; never modify source time.
            nonlocal stage_at
            now=time.monotonic()
            stages[name]=round(now-stage_at,4)
            stage_at=now
        try:
            with self.nav.lock:
                if not self._work_current(None):return
                context=(getattr(self,'_revision',0),getattr(self.nav,'generation',0),
                         getattr(self,'_processing_epoch',None))
                if not self._work_current(context):return
                records=[r for r in records if r.config_version==version]
                if not records:raise ValueError('等待视觉障碍深度数据')
                record=max(records,key=lambda r:r.frame.source_at)
                self._fresh_record(record)
                key=(version,record.frame.stream_epoch,record.frame.frame_id,objects is not None)
                if key==getattr(self,'last',None):return
                camera_pose=self.nav.camera_pose
                static_map=getattr(self,'static_map',None)
                args=(record.depth,record.frame.image.shape,self.calibration,
                      self.height,self.pitch,self.scale)
                footprint=getattr(self,'_safety_footprint',(None,0.))
                previous_origin=getattr(self,'_grid_origin',None)
                use_static=self.use_static
                boundary=self.nav.obstacle_mode=='map'
            from rclpy.time import Time
            tf=self.nav.tf.lookup_transform('odom','base_link',Time(nanoseconds=record.frame.stamp_ns)).transform
            q=tf.rotation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            if camera_pose is None:raise ValueError('缺少相机平面安装参数')
            mx,my,ma=camera_pose
            c,s=math.cos(ma),math.sin(ma)
            camera_rotation=np.array([[c,s],[-s,c]])
            c,s=math.cos(yaw),math.sin(yaw)
            base_rotation=np.array([[c,s],[-s,c]])
            def to_base(points):return points@camera_rotation+np.array([mx,my])
            def to_world(points):
                return to_base(points)@base_rotation+np.array([tf.translation.x,tf.translation.y])
            static_tf=None;accept=None
            if use_static:
                if static_map is None:raise ValueError('等待静态地图，不能发布缺失墙体的视觉地图')
                static_tf=self.nav.tf.lookup_transform('odom',static_map.header.frame_id,Time()).transform
                accept=lambda points:inside_static_map(to_world(points),static_map,static_tf)
            mark('snapshot_tf')
            from nav2.depth_projection import DepthProjection
            projection=DepthProjection(*args)
            mark('projection')
            # 先判断全点云近距危险：可选语义、候选分组或慢发布不能推迟停车。
            safety=to_base(depth_points(*args,nearest_only=False,prepared=projection))
            from nav2.replan_support import capture_visual_safety, recovery
            safety_world=safety@base_rotation+np.array([tf.translation.x,tf.translation.y])
            safety_snapshot=capture_visual_safety(self.nav,safety_world,record.frame.source_at,footprint)
            mark('full_cloud')
            from nav2.near_obstacle import NearObstacleGuard
            with self.nav.lock:
                if not self._work_current(context):return
                self._fresh_record(record)
                if not hasattr(self,'near_guard'):self.near_guard=NearObstacleGuard()
                speed,turn=self.nav.command if self.nav.enabled else (0.,0.)
                blocked,near_info=self.near_guard.update(safety,speed,turn,record.frame.source_at,time.monotonic())
                near_error=(f'近距障碍保护：停车阈值 {near_info["near_stop_m"]:.2f} 米；等待 3 帧安全确认' if blocked else '')
                if blocked:
                    # 危险立即生效，不等构图/发布；已验证绕行仍需真实全点云扫掠检查。
                    self.nav.vision_safety=safety_snapshot
                    self.nav.vision_near_blocked=True
                    self.nav.vision_at=record.frame.source_at
                    self.nav.vision_error=near_error
                    task=recovery(self.nav)
                    if not task or not task.permits_near:
                        if not task or not task.trigger(near_error):self.nav.stop(near_error)
                # 清空障碍的证据在本代地图发布完成后才提交。若发布期间切换配置，
                # 旧消息虽无法撤销，仍不能凭下一轮“先清除”而提前恢复行驶依据。
                # 本轮自己的停车/重规划可以改变 generation；外部取消仍使后续提交失效。
                context=(context[0],getattr(self.nav,'generation',0),context[2])
            mark('near_guard')
            from nav2.semantic_obstacles import instance_masks
            instances=instance_masks(objects or [])
            semantic_preview=None if objects is None else dict(image=record.frame.image,objects=objects,
                source_at=record.frame.source_at,robot=dict(position=[tf.translation.x,tf.translation.y],yaw=yaw),
                frame_id=record.frame.frame_id)
            diagnostics={}
            points=depth_points(*args,accept=accept,diagnostics=diagnostics,instances=instances,prepared=projection)
            camera_points=points if len(points) else depth_points(*args,instances=instances,prepared=projection)
            mark('candidates')
            camera_cells={(math.floor(x/.05),math.floor(y/.05)):True for x,y in to_world(camera_points)}
            diagnostics['camera_only']=bool(len(camera_points) and not len(points))
            diagnostics.update(near_info)
            world=to_world(points)
            cells={(math.floor(x/.05),math.floor(y/.05)):True for x,y in world}
            ox,oy=stable_grid_origin(tf.translation.x,tf.translation.y,previous_origin)
            msg=self.kind();msg.header.frame_id='odom';msg.header.stamp=self.node.get_clock().now().to_msg()
            msg.info.resolution=.05;msg.info.width=msg.info.height=200
            msg.info.origin.position.x=ox*.05;msg.info.origin.position.y=oy*.05;msg.info.origin.orientation.w=1.
            if use_static:
                from nav2.static_wall_cache import StaticWallCache
                if not hasattr(self,'_wall_cache'):self._wall_cache=StaticWallCache()
                data=self._wall_cache.raster((200,200),(ox*.05,oy*.05),.05,static_map,static_tf,boundary=boundary)
            else:data=np.zeros((200,200),np.int8)
            for x,y in cells:
                if 0<=x-ox<200 and 0<=y-oy<200:data[y-oy,x-ox]=100
            msg.data=data.ravel().tolist()
            mark('raster')
            with self.nav.lock:
                if not self._work_current(context):return
                self._fresh_record(record)
            mark('prepublish_check')
            self.publisher.publish(msg)  # 不持 nav.lock；控制线程能继续检查源帧年龄并输出零速。
            mark('publish')
            with self.nav.lock:
                if not self._work_current(context):return
                self._fresh_record(record)
                self.nav.vision_safety=safety_snapshot
                self.nav.vision_near_blocked=bool(blocked)
                self.nav.vision_at=record.frame.source_at  # 不以完成时间续期。
                self.nav.vision_error=near_error
                if not blocked and self.nav.error.startswith('视觉障碍数据超时，'):
                    self.nav.error=''
                self.camera_cells=camera_cells;self.cells=cells
                self.diagnostics=diagnostics;self.semantic_preview=semantic_preview
                self._grid_origin=(ox,oy);self.nav.vision_count=len(points);self.last=key
            mark('commit')
        except Exception as exc:
            from nav2_follow import VisionStale
            with self.nav.lock:
                if not self._work_current(context):return
                if isinstance(exc,VisionStale):
                    # R37：旧的近距提示不改变超时分类。与 velocity 共用同一个
                    # 零速暂停/源帧年龄 2 秒取消策略，仍保留其他故障检查。
                    self.nav.pause_for_vision()
                    return
                self.nav.vision_at=0.
                self.nav.vision_near_blocked=False
                self.nav.vision_safety=None
                self.nav.vision_error=str(exc)
                self.nav.stop('视觉障碍检测不可用：'+str(exc))
