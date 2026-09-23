"""Optional, estimated-depth obstacle map. Does not own chassis commands."""
import math
import os
import time
from pathlib import Path
import cv2
import numpy as np
import yaml


def depth_points(depth, shape, calibration, height, pitch, scale, accept=None, nearest_only=True):
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
    mask=(np.isfinite(values)&(values>0)&(forward>.1)&(forward<4)&(np.abs(lateral)<2)&(z>.10)&(z<1.8)).astype(np.uint8)
    if accept is not None:
        accepted=accept(np.column_stack((forward.ravel(),lateral.ravel())))
        mask &= np.asarray(accepted,dtype=np.uint8).reshape(mask.shape)
    count,labels,stats,_=cv2.connectedComponentsWithStats(mask,8)
    # Select one spatially connected candidate by robust forward distance.
    eligible=[i for i in range(1,count) if stats[i,cv2.CC_STAT_AREA]>=6]
    nearest=min(eligible,key=lambda i:float(np.percentile(forward[labels==i],10))) if eligible else None
    keep=(labels==nearest if nearest is not None else np.zeros_like(mask,dtype=bool)) if nearest_only else np.isin(labels,eligible)
    return np.column_stack((forward[keep],lateral[keep]))


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


class VisionLayer:
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
        self.last=None;self.cells={};self.version=None
        self.static_map=None
        self.use_static=navigation.obstacle_mode in ('map','both')
        if self.use_static:
            self.static_sub=node.create_subscription(OccupancyGrid,'/visual_car/edited_map',self.receive_static,
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        navigation.vision_layer=self
        self.timer=node.create_timer(.1,self.tick)

    def receive_static(self, msg):
        self.static_map=msg
        self.last=None

    def tick(self):
        # Never wait for engine.lock while holding nav.lock: processing uses
        # engine -> motion -> nav. Copy records first and release engine.lock.
        with self.engine.lock:
            records=list(self.engine._buffer_m2.values())
            version=self.engine._config_version
        with self.nav.lock:
            self._tick_locked(records, version)

    def _tick_locked(self, records, version):
        if not self.nav.vision_enabled or getattr(self, "switch_failed", False):
            return
        try:
            records=[r for r in records if r.config_version==version]
            if not records:raise ValueError('等待视觉障碍深度数据')
            record=max(records,key=lambda r:r.frame.source_at)
            if time.monotonic()-record.frame.source_at>1.2:
                from nav2_follow import VisionStale
                raise VisionStale('视觉障碍深度数据超过 1.2 秒')
            key=(record.frame.stream_epoch,record.frame.frame_id)
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
            def to_base(points):
                return points@camera_rotation+np.array([mx,my])
            def to_world(points):
                return to_base(points)@base_rotation+np.array([tf.translation.x,tf.translation.y])
            static_tf=None
            accept=None
            if self.use_static:
                if self.static_map is None:raise ValueError('等待静态地图，不能发布缺失墙体的视觉地图')
                static_tf=self.nav.tf.lookup_transform('odom',self.static_map.header.frame_id,Time()).transform
                accept=lambda points:inside_static_map(to_world(points),self.static_map,static_tf)
            args=(record.depth,record.frame.image.shape,self.calibration,self.height,self.pitch,self.scale)
            # Emergency near-field check uses all candidates, including outside the map.
            safety=to_base(depth_points(*args,nearest_only=False))
            near=safety[(safety[:,0]>0)&(np.abs(safety[:,1])<.40)]
            blocked=bool(len(near) and np.min(near[:,0])<.70)
            # Clip BEFORE selecting the nearest component, so outside candidates cannot win.
            points=depth_points(*args,accept=accept)
            world=to_world(points)
            # Only the current nearest candidate is published; no old-candidate trails.
            self.cells={(math.floor(x/.05),math.floor(y/.05)):True for x,y in world}
            ox=math.floor(tf.translation.x/.05)-100;oy=math.floor(tf.translation.y/.05)-100
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
            with self.nav.lock:
                self.nav.vision_at=record.frame.source_at
                self.nav.vision_error='前方 0.7 米内有疑似障碍，停止导航' if blocked else ''
                self.nav.vision_count=len(points)
                if blocked:self.nav.stop(self.nav.vision_error)
            self.last=key
        except Exception as exc:
            from nav2_follow import VisionStale
            if isinstance(exc, VisionStale) and not self.nav.vision_error:
                self.nav.pause_for_vision()
                return
            with self.nav.lock:
                self.nav.vision_at=0.
                self.nav.vision_error=str(exc)
                self.nav.stop('视觉障碍检测不可用：'+str(exc))
