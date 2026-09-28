# 【内容标注】用途：视觉障碍候选与代价地图生成。
# 对应用户需求：R08 R09 R10 R19 R22 R32（原话及追溯边界见 nav2/CODE_GUIDE.md）。
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
    """Keep nearby fragments at similar depth, without filling unobserved gaps."""
    ids=np.unique(labels);ids=ids[ids>0]
    if not len(ids):return labels>0
    distance={i:float(np.percentile(forward[labels==i],10)) for i in ids}
    seed=min(ids,key=lambda i:distance[i])
    selected=labels==seed
    seed_xy=np.column_stack((forward[selected],lateral[selected]))
    lo=seed_xy.min(axis=0);hi=seed_xy.max(axis=0)
    for i in ids:
        if i==seed or abs(distance[i]-distance[seed])>.30:continue
        candidate=labels==i
        xy=np.column_stack((forward[candidate],lateral[candidate]))
        gap=np.maximum(0,np.maximum(lo-xy.max(axis=0),xy.min(axis=0)-hi))
        if np.linalg.norm(gap)<=.20:selected |= candidate
    return selected


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
def depth_points(depth, shape, calibration, height, pitch, scale, accept=None, nearest_only=True, diagnostics=None, instances=None):
    values=cv2.resize(np.squeeze(depth).astype(np.float32),(80,60),interpolation=cv2.INTER_NEAREST)*scale
    if np.mean(np.isfinite(values)&(values>0))<.5:
        raise ValueError('视觉深度有效像素不足')
    h,w=shape[:2]; yy,xx=np.indices(values.shape)
    pixels=np.stack(((xx+.5)*w/80,(yy+.5)*h/60),axis=-1).astype(float)
    k=np.array(calibration['camera_matrix']['data'],float).reshape(3,3)
    k[0,:]*=w/calibration['image_width'];k[1,:]*=h/calibration['image_height']
    rays=cv2.undistortPoints(pixels.reshape(-1,1,2),k,np.array(calibration['distortion_coefficients']['data'],float)).reshape(60,80,2)
    a=math.radians(pitch)
    forward=values*(math.cos(a)-rays[:,:,1]*math.sin(a))
    lateral=-values*rays[:,:,0]
    z=height-values*(math.sin(a)+rays[:,:,1]*math.cos(a))
    valid=np.isfinite(values)&(values>0)
    ranged=valid&(forward>.1)&(forward<4)&(np.abs(lateral)<2)
    mask=ranged&(z>.10)&(z<1.8)
    # Reject image noise BEFORE map clipping. A real object must not become
    # noise merely because only a few of its pixels lie inside the map.
    count,labels,stats,_=cv2.connectedComponentsWithStats(mask.astype(np.uint8),8)
    eligible=[i for i in range(1,count) if stats[i,cv2.CC_STAT_AREA]>=6]
    mask=np.isin(labels,eligible)
    if nearest_only:
        # Apply the six-pixel support requirement BEFORE map clipping and AFTER
        # depth splitting: isolated near noise cannot borrow a far wall's area.
        labels=depth_components(mask,forward)
        support=np.bincount(labels.ravel(),minlength=mask.size+1)
        mask &= support[labels]>=6
    before_map=int(mask.sum())
    if accept is not None:
        accepted=accept(np.column_stack((forward.ravel(),lateral.ravel())))
        mask &= np.asarray(accepted,dtype=bool).reshape(mask.shape)
    # Keep pre-clipping identities and do not discard a valid object's surviving
    # pixels at the map border. Emergency nearest_only=False still uses ALL
    # original candidates, without the new depth grouping/selection.
    keep=nearest_fragments(np.where(mask,labels,0),forward,lateral) if nearest_only else mask
    selected_label=None
    if nearest_only and instances:
        candidates=[]
        geometric=ranged&(z>.10)&(z<1.8)
        for label,instance in instances:
            full=geometric&instance
            if np.count_nonzero(full)<6:continue
            clipped=full if accept is None else full&np.asarray(accepted,dtype=bool).reshape(full.shape)
            if clipped.any():candidates.append((float(np.percentile(forward[clipped],10)),label,clipped))
        if candidates:
            distance,label,instance=min(candidates,key=lambda item:item[0])
            # Never discard a substantially nearer unclassified obstacle.
            if not keep.any() or distance<=float(np.percentile(forward[keep],10))+.30:
                keep=instance;selected_label=label

    if diagnostics is not None:
        diagnostics.update(valid_pixels=int(valid.sum()), range_pixels=int(ranged.sum()),
                           height_pixels=int((ranged&(z>.10)&(z<1.8)).sum()),
                           component_pixels=before_map, map_pixels=int(mask.sum()),
                           selected_pixels=int(keep.sum()), depth_scale=float(scale))
        if selected_label is not None:
            reason='YOLOE 物体有效深度已用于候选：'+selected_label
        elif not before_map:
            reason=('距离/横向范围内无候选' if not ranged.any() else
                    '高度筛选后无候选' if not (ranged&(z>.10)&(z<1.8)).any() else '候选面积不足')
        elif not mask.any():reason='视觉候选位于地图外；请核对车位、相机安装参数和深度比例'
        else:reason='地图内候选有效'
        diagnostics['reason']=reason
        diagnostics['selected_object']=selected_label
    return np.column_stack((forward[keep],lateral[keep]))


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
        from nav2.semantic_obstacles import SemanticWorker
        self.semantic=SemanticWorker()
        self.semantic_preview=None
        self.static_map=None
        self.use_static=navigation.obstacle_mode in ('map','both')
        if self.use_static:
            self.static_sub=node.create_subscription(OccupancyGrid,'/visual_car/edited_map',self.receive_static,
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        from nav2.depth_mailbox import DepthMailbox
        self._depth_mailbox=DepthMailbox(engine.depth)
        navigation.vision_layer=self
        # 不占用原相机的 SingleThreadedExecutor。只有一个串行消费者，
        # 每轮直接读取最新结果，不堆积定时器任务，也不更改原相机/深度代码。
        self._worker_stop=threading.Event()
        node.context.on_shutdown(self._worker_stop.set)
        self._worker=threading.Thread(target=self._run,name='nav2-vision-update',daemon=True)
        self._worker.start()

    # 【职责 / R08 R09 R10 R19 R22】_run：单个串行工作线程消费最新数据，避免任务堆积。
    def _run(self):
        try:
            while not self._worker_stop.is_set():
                engine_stop=getattr(self.engine,'_stop',None)
                if engine_stop is not None and engine_stop.is_set():
                    break
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
                self._worker_stop.wait(max(0.,.1-(time.monotonic()-started)))
        finally:
            # YOLO 子进程只由当前消费者关闭，防止 update/close 并发。
            mailbox=getattr(self,'_depth_mailbox',None)
            if mailbox is not None:
                mailbox.close()
            self.semantic.close()

    # 【职责 / R08 R09 R10 R19 R22】close：停止自有工作线程并释放接入资源。
    def close(self):
        self._worker_stop.set()
        if self._worker is not threading.current_thread():
            self._worker.join(timeout=3.)
        if self._worker.is_alive():
            logging.getLogger(__name__).warning('视觉线程仍在退出，保持停车')
            with self.nav.lock:
                self.nav.vision_at=0.
                self.nav.vision_error='视觉线程正在退出'
                self.nav.stop(self.nav.vision_error)

    # 【职责 / R08 R09 R10 R19 R22】receive_static：记录新静态地图供裁剪和合成使用。
    def receive_static(self, msg):
        # 接收地图在 ROS 线程，构图在工作线程；使用已有导航锁保护更新。
        with self.nav.lock:
            self.static_map=msg
            self.last=None

    # 【职责 / R08 R09 R10 R19 R22】tick：读取最新深度/语义结果并检查时效，记录处理耗时。
    def tick(self):
        # Never wait for engine.lock while holding nav.lock: processing uses
        # engine -> motion -> nav. Copy records first and release engine.lock.
        tick_started=time.monotonic()
        tick_gap=tick_started-getattr(self,'_previous_tick',tick_started)
        self._previous_tick=tick_started
        with self.engine.lock:
            records=list(self.engine._buffer_m2.values())
            version=self.engine._config_version
            epoch=getattr(self.engine,'_stream_epoch',None)
        engine_done=time.monotonic()
        records=[r for r in records if r.config_version==version
                 and (epoch is None or r.frame.stream_epoch==epoch)]
        mailbox=getattr(self,'_depth_mailbox',None)
        direct=mailbox.latest(version,epoch) if mailbox is not None else None
        if direct is not None:
            records.append(direct)
        with self.nav.lock:
            enabled=self.nav.vision_enabled
        packet=None
        if enabled and records:
            try:packet=self.semantic.update(max(records,key=lambda r:r.frame.source_at))
            except Exception as exc:self.semantic.message='YOLOE 接入失败：'+str(exc)
        elif not enabled:
            self.semantic.close()
        # 安全检测始终使用最新深度帧。异步 YOLO 旧结果不能让时间戳倒退，
        # 也不能把旧掩膜贴到新图像；只有完全同帧时才参与候选分组。
        latest=max(records,key=lambda r:r.frame.source_at) if records else None
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
        safety_key = ((latest.frame.stream_epoch, latest.frame.frame_id)
                      if latest is not None else None)
        if (enabled and latest is not None
                and 0 <= time.monotonic()-latest.frame.source_at <= 1.2
                and safety_key != getattr(self, '_safety_frame_key', None)):
            try:
                safety_footprint = self.nav.navigation_footprint()
            except Exception:
                safety_footprint = (None, 0.)
            self._safety_frame_key = safety_key
        with self.nav.lock:
            lock_done=time.monotonic()
            self._safety_footprint = safety_footprint
            self._tick_locked([latest] if latest is not None else [], version, objects)
        from nav2.replan_support import monitor_route, recovery
        task = recovery(self.nav)
        if task is not None and task.active:
            monitor_route(self.nav)
        finished=time.monotonic()
        age=finished-latest.frame.source_at if latest is not None else float('inf')
        if latest is not None:
            inference_started=getattr(latest,'started_at',None)
            inference_done=getattr(latest,'completed_at',None)
            timing={
                'frame_id':latest.frame.frame_id,
                'source_age':round(age,3),
                'input_wait':round(inference_started-latest.frame.source_at,3) if inference_started is not None else None,
                'inference':round(inference_done-inference_started,3) if inference_started is not None and inference_done is not None else None,
                'after_inference':round(finished-inference_done,3) if inference_done is not None else None,
                'direct_result':latest is direct,
            }
            with self.nav.lock:
                self.timing=timing
        else:
            timing={}
        if age>1.2 and finished-getattr(self,'_last_delay_log',0.)>5.:
            self._last_delay_log=finished
            logging.getLogger(__name__).warning(
                '视觉更新延迟：源帧年龄 %.3f 秒，读取深度缓冲 %.3f 秒，语义阶段 %.3f 秒，'
                '导航锁等待 %.3f 秒，障碍计算 %.3f 秒，回调间隔 %.3f 秒，入口帧龄 %.3f 秒',
                age,engine_done-tick_started,semantic_done-engine_done,lock_done-semantic_done,
                finished-lock_done,tick_gap,
                engine_done-latest.frame.source_at if latest is not None else float('inf'))
            logging.getLogger(__name__).warning('视觉深度分段耗时：%s',timing)

    # 【职责 / R08 R09 R10 R19 R22】_tick_locked：转换坐标、近距保护、候选筛选及地图合成；新错误沿停车链路处理。
    def _tick_locked(self, records, version, objects=None):
        if not self.nav.vision_enabled or getattr(self, "switch_failed", False):
            return
        try:
            records=[r for r in records if r.config_version==version]
            if not records:raise ValueError('等待视觉障碍深度数据')
            record=max(records,key=lambda r:r.frame.source_at)
            if time.monotonic()-record.frame.source_at>1.2:
                from nav2_follow import VisionStale
                raise VisionStale('视觉障碍深度数据超过 1.2 秒')
            key=(record.frame.stream_epoch,record.frame.frame_id,objects is not None)
            if key==self.last:return
            from rclpy.time import Time
            tf=self.nav.tf.lookup_transform('odom','base_link',Time(nanoseconds=record.frame.stamp_ns)).transform
            q=tf.rotation; yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            if self.nav.camera_pose is None:raise ValueError('缺少相机平面安装参数')
            mx,my,ma=self.nav.camera_pose
            c,s=math.cos(ma),math.sin(ma)
            camera_rotation=np.array([[c,s],[-s,c]])
            c,s=math.cos(yaw),math.sin(yaw)
            base_rotation=np.array([[c,s],[-s,c]])
            # 【职责 / R08 R09 R10 R19 R22】to_base：相机候选转机器人平面坐标。
            def to_base(points):
                return points@camera_rotation+np.array([mx,my])
            # 【职责 / R08 R09 R10 R19 R22】to_world：机器人候选按拍摄时车位转 odom。
            def to_world(points):
                return to_base(points)@base_rotation+np.array([tf.translation.x,tf.translation.y])
            static_tf=None
            accept=None
            if self.use_static:
                if self.static_map is None:raise ValueError('等待静态地图，不能发布缺失墙体的视觉地图')
                static_tf=self.nav.tf.lookup_transform('odom',self.static_map.header.frame_id,Time()).transform
                accept=lambda points:inside_static_map(to_world(points),self.static_map,static_tf)
            from nav2.semantic_obstacles import instance_masks
            instances=instance_masks(objects or [])
            self.semantic_preview=None
            if objects is not None:
                self.semantic_preview={'image':record.frame.image, 'objects':objects,
                    'source_at':record.frame.source_at,
                    'robot':{'position':[tf.translation.x,tf.translation.y],'yaw':yaw},
                    'frame_id':record.frame.frame_id}
            args=(record.depth,record.frame.image.shape,self.calibration,self.height,self.pitch,self.scale)
            # Emergency near-field check uses all candidates, including outside the map.
            safety=to_base(depth_points(*args,nearest_only=False))
            from nav2.near_obstacle import NearObstacleGuard
            if not hasattr(self,'near_guard'):self.near_guard=NearObstacleGuard()
            with self.nav.lock:
                speed,turn=self.nav.command if self.nav.enabled else (0.,0.)
            near_now=time.monotonic()
            if near_now-record.frame.source_at>1.2:
                from nav2_follow import VisionStale
                raise VisionStale('视觉障碍数据超时，停车等待新数据')
            blocked,near_info=self.near_guard.update(safety,speed,turn,record.frame.source_at,near_now)
            # Clip BEFORE selecting the nearest component, so outside candidates cannot win.
            diagnostics={}
            points=depth_points(*args,accept=accept,diagnostics=diagnostics,instances=instances)
            # Prefer the SAME candidate as navigation; when no map candidate
            # survives, retain a camera-only preview without adding off-map cells.
            camera_points=points if len(points) else depth_points(*args,instances=instances)
            self.camera_cells={(math.floor(x/.05),math.floor(y/.05)):True
                               for x,y in to_world(camera_points)}
            diagnostics['camera_only']=bool(len(camera_points) and not len(points))
            diagnostics.update(near_info)
            self.diagnostics=diagnostics
            world=to_world(points)
            # Only the current nearest candidate is published; no old-candidate trails.
            self.cells={(math.floor(x/.05),math.floor(y/.05)):True for x,y in world}
            ox,oy=stable_grid_origin(tf.translation.x,tf.translation.y,
                                     getattr(self,'_grid_origin',None))
            self._grid_origin=(ox,oy)
            msg=self.kind();msg.header.frame_id='odom';msg.header.stamp=self.node.get_clock().now().to_msg()
            msg.info.resolution=.05;msg.info.width=msg.info.height=200
            msg.info.origin.position.x=ox*.05;msg.info.origin.position.y=oy*.05;msg.info.origin.orientation.w=1.
            data=np.zeros((200,200),np.int8)
            for x,y in self.cells:
                if 0<=x-ox<200 and 0<=y-oy<200:data[y-oy,x-ox]=100
            if self.use_static:
                if self.static_map is None:raise ValueError('等待静态地图，不能发布缺失墙体的视觉地图')
                merge_static_walls(data,(ox*.05,oy*.05),.05,self.static_map,static_tf)
                if self.nav.obstacle_mode == 'map':
                    # The final visual layer must carry the fence as well as static walls.
                    from nav2.boundary_map import boundary_grid
                    merge_static_walls(data,(ox*.05,oy*.05),.05,boundary_grid(self.static_map),static_tf)
            msg.data=data.ravel().tolist();self.publisher.publish(msg)
            # R32: keep all safety candidates in world coordinates, including
            # outside-map points. The displayed/published nearest cluster stays unchanged.
            from nav2.replan_support import capture_visual_safety, recovery
            safety_world = safety@base_rotation+np.array([tf.translation.x,tf.translation.y])
            safety_snapshot = capture_visual_safety(self.nav, safety_world, record.frame.source_at,
                getattr(self, '_safety_footprint', (None, 0.)))
            with self.nav.lock:
                self.nav.vision_safety = safety_snapshot
                self.nav.vision_near_blocked = bool(blocked)
                self.nav.vision_at=record.frame.source_at
                self.nav.vision_error=(f'近距障碍保护：停车阈值 {near_info["near_stop_m"]:.2f} 米；等待 3 帧安全确认' if blocked else '')
                self.nav.vision_count=len(points)
                # 仅清理已经恢复的视觉超时提示，不解除强停、不清除其他故障，
                # 保留 vision_paused，让速度链路恢复时仍先输出零速。
                if (not blocked and time.monotonic()-self.nav.vision_at<=1.2
                        and self.nav.error.startswith('视觉障碍数据超时，')):
                    self.nav.error=''
                if blocked:
                    task = recovery(self.nav)
                    if not task or not task.permits_near:
                        if not task or not task.trigger(self.nav.vision_error):
                            self.nav.stop(self.nav.vision_error)
            self.last=key
        except Exception as exc:
            from nav2_follow import VisionStale
            if isinstance(exc, VisionStale) and not self.nav.vision_error:
                self.nav.pause_for_vision()
                return
            with self.nav.lock:
                self.nav.vision_at=0.
                self.nav.vision_near_blocked=False
                self.nav.vision_safety=None
                self.nav.vision_error=str(exc)
                self.nav.stop('视觉障碍检测不可用：'+str(exc))
