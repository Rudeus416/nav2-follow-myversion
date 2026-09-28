# 【内容标注】用途：网页所需地图与机器人图层快照。
# 对应用户需求：R02 R03 R22 R26（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：订阅地图/轮廓/雷达/路径并转换坐标；局部供网页，全局按需供整路线校验。
# 需求关联用于功能追溯，不是精确创建/提交记录。
"""Read-only odom snapshots: local browser view and opt-in global validation."""
import math
import threading
import time
import os
from pathlib import Path


# 【职责 / R02 R03 R22】ObstacleView：网页所需地图与机器人图层快照的状态封装；各方法职责见下方标注。
class ObstacleView:
    # 【职责 / R02 R03 R22】__init__：初始化本类依赖与状态；副作用以原初始化语句为准。
    def __init__(self, node, radar_topic):
        from geometry_msgs.msg import PolygonStamped
        from nav_msgs.msg import OccupancyGrid, Path as NavPath
        from map_msgs.msg import OccupancyGridUpdate
        from sensor_msgs.msg import PointCloud2
        from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
        from tf2_ros import Buffer, TransformListener
        self.node = node
        self.lock = threading.Lock()
        self.layers = {}
        self.errors = {}
        self.pending = {}
        self.radar_at = 0.0
        self.static_message = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, node)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.editor = None
        if os.environ.get('NAV2_OBSTACLE_MODE', 'radar') in ('map', 'both'):
            from nav2_map_edit import MapEdits
            self.editor = MapEdits(Path(__file__).resolve().parent / 'data/nav2_map_edits')
            self.edited_publisher = node.create_publisher(OccupancyGrid, '/visual_car/map_edit_input', 10)
        self.subscriptions = [
            node.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.grid, map_qos),
            node.create_subscription(OccupancyGridUpdate, '/local_costmap/costmap_updates', self.update_grid, 10),
            node.create_subscription(PolygonStamped, '/local_costmap/published_footprint',
                                     lambda msg: self.points('footprint', msg.header, msg.polygon.points), 1),
            node.create_subscription(NavPath, '/plan',
                                     lambda msg: self.points('path', msg.header, [p.pose.position for p in msg.poses]), 1),
            node.create_subscription(PointCloud2, radar_topic, self.radar, qos_profile_sensor_data),
        ]
        if self.editor is not None:
            self.subscriptions.extend([
                node.create_subscription(OccupancyGrid, '/map', self.static_grid, map_qos),
                node.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.global_grid, map_qos),
                node.create_subscription(OccupancyGridUpdate, '/global_costmap/costmap_updates',
                                         self.update_global_grid, 10),
            ])
        self.retry_timer = node.create_timer(0.02, self.retry_points)
        self.pose_timer = node.create_timer(0.2, self.refresh_pose)

    # 【职责 / R02 R03 R22】static_grid：接收静态地图并关联编辑数据。
    def static_grid(self, msg):
        if getattr(self, 'editor', None) is not None:
            try:
                with self.editor.lock:
                    self.editor.set_base(msg)
                    self.publish_edited_map(self.editor.compose())
            except Exception as exc:
                with self.lock:
                    self.errors['map_edit'] = str(exc)
            return
        self.static_message = msg
        self.refresh_pose()

    # 【职责 / R02 R03 R22】publish_edited_map：发布手绘禁区合成后的图层。
    def publish_edited_map(self, msg):
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.edited_publisher.publish(msg)
        self.static_message = msg
        with self.lock:
            self.errors.pop('map_edit', None)
            self.layers.pop('path', None)
        self.refresh_pose()

    # 【职责 / R02 R03 R22】edit_map：处理编辑动作并更新发布结果。
    def edit_map(self, action, rect, base_id, revision):
        if self.editor is None:
            raise ValueError('仅静态地图或叠加模式支持编辑')
        with self.editor.lock:
            msg = self.editor.edit(action, rect, base_id, revision)
            self.publish_edited_map(msg)
            return self.editor.state()

    # 【职责 / R02 R03 R22】refresh_pose：读取机器人位姿和相关坐标变换。
    def refresh_pose(self):
        from rclpy.time import Time
        def yaw(q):
            return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        try:
            pose = self.tf.lookup_transform('odom', 'base_link', Time())
            t = pose.transform
            with self.lock:
                self.store('robot', pose.header, {'position': [t.translation.x, t.translation.y],
                                                'yaw': yaw(t.rotation)})
        except Exception as exc:
            with self.lock:
                self.layers.pop('robot', None)
                self.errors['robot'] = str(exc)
        msg = self.static_message
        if msg is None:
            return
        try:
            info = msg.info
            if not (0 < info.width * info.height <= 1000000 and info.resolution > 0
                    and len(msg.data) == info.width * info.height):
                raise ValueError('静态地图尺寸无效')
            x, y = info.origin.position.x, info.origin.position.y
            angle = yaw(info.origin.orientation)
            if msg.header.frame_id != 'odom':
                t = self.tf.lookup_transform('odom', msg.header.frame_id, Time()).transform
                a = yaw(t.rotation)
                x, y = (t.translation.x + math.cos(a)*x - math.sin(a)*y,
                        t.translation.y + math.sin(a)*x + math.cos(a)*y)
                angle += a
            with self.lock:
                self.store('static_map', msg.header, dict(width=info.width, height=info.height,
                    resolution=info.resolution, origin=[x, y], yaw=angle, data=list(msg.data)))
        except Exception as exc:
            with self.lock:
                self.layers.pop('static_map', None)
                self.errors['static_map'] = str(exc)

    # 【职责 / R02 R03 R22】store：以层名保存新快照及时间。
    def store(self, name, header, data):
        stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
        self.layers[name] = dict(data, stamp=stamp, received=time.monotonic())
        self.errors.pop(name, None)

    # 【职责 / R02 R03 R22】grid：接收网页局部代价地图；与全局层独立保存。
    def grid(self, msg):
        self._grid('map', msg)

    # 【职责 / R13 R22】global_grid：接收后端整条路线校验所需的全局代价地图。
    def global_grid(self, msg):
        self._grid('global_map', msg)

    def _grid(self, name, msg):
        """Shared validation, independent layer keys; never mutate old snapshots."""
        with self.lock:
            if msg.header.frame_id != 'odom':
                self.layers.pop(name, None)
                self.errors[name] = '地图坐标系不是 odom，无法叠加显示或校验'
                return
            info = msg.info
            q = info.origin.orientation
            geometry = (info.resolution, info.origin.position.x, info.origin.position.y,
                        q.x, q.y, q.z, q.w)
            if not (info.width > 0 and info.height > 0
                    and info.width * info.height <= 1000000 and info.resolution > 0
                    and len(msg.data) == info.width * info.height
                    and all(math.isfinite(v) for v in geometry)
                    and all(-1 <= value <= 100 for value in msg.data)):
                self.layers.pop(name, None)
                self.errors[name] = '地图尺寸或数据无效，或超过栅格上限'
                return
            self.store(name, msg.header, dict(width=info.width, height=info.height,
                resolution=info.resolution, origin=[info.origin.position.x, info.origin.position.y],
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)), data=list(msg.data)))

    # 【职责 / R02 R03 R22】update_grid：更新网页局部层，不修改全局图层。
    def update_grid(self, msg):
        self._update_grid('map', msg)

    # 【职责 / R13 R22】update_global_grid：仅更新后端全局层。
    def update_global_grid(self, msg):
        self._update_grid('global_map', msg)

    def _update_grid(self, name, msg):
        with self.lock:
            if msg.header.frame_id != 'odom':
                self.layers.pop(name, None)
                self.errors[name] = '地图增量坐标系不是 odom，等待完整地图'
                return
            grid = self.layers.get(name)
            if grid is None:
                return
            if (msg.x < 0 or msg.y < 0 or msg.width <= 0 or msg.height <= 0
                    or msg.x + msg.width > grid['width'] or msg.y + msg.height > grid['height']
                    or len(msg.data) != msg.width * msg.height
                    or not all(-1 <= value <= 100 for value in msg.data)):
                self.errors[name] = '地图增量尺寸或数据不匹配，等待完整地图'
                self.layers.pop(name, None)
                return
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if stamp > 0 and grid['stamp'] > 0 and stamp < grid['stamp']:
                # A late patch from before the current full map must not overwrite
                # newer geometry/data or refresh the current snapshot's age.
                return
            data = grid['data'].copy()
            for row in range(msg.height):
                start = (msg.y + row) * grid['width'] + msg.x
                data[start:start + msg.width] = msg.data[row*msg.width:(row+1)*msg.width]
            self.store(name, msg.header, dict(grid, data=data))

    # 【职责 / R02 R03 R22】points：转换路径/轮廓等点序列。
    def points(self, name, header, points, received=None):
        received = time.monotonic() if received is None else received
        try:
            from rclpy.time import Time
            import numpy as np
            xyz = np.asarray([(p.x, p.y, p.z) for p in points], dtype=float).reshape(-1, 3)
            xyz = xyz[np.isfinite(xyz).all(axis=1)]
            if header.frame_id != 'odom' and len(xyz):
                t = self.tf.lookup_transform('odom', header.frame_id, Time.from_msg(header.stamp)).transform
                q = t.rotation
                # Full quaternion rotation before dropping height for display.
                v = np.array([q.x, q.y, q.z])
                uv = 2 * np.cross(v, xyz)
                xyz = xyz + q.w * uv + np.cross(v, uv)
                xyz += [t.translation.x, t.translation.y, t.translation.z]
            with self.lock:
                self.store(name, header, {'points': xyz[:, :2].tolist()})
                self.layers[name]['received'] = received
                self.pending.pop(name, None)
        except Exception as exc:
            from tf2_ros import TransformException
            with self.lock:
                if isinstance(exc, TransformException) and time.monotonic() - received < 0.25:
                    # One pending sample per layer; timer retries let TF callbacks run.
                    self.pending[name] = (header, points, received)
                    self.errors[name] = '等待对应时间的 TF'
                    return
                self.pending.pop(name, None)
                self.layers.pop(name, None)
                self.errors[name] = str(exc)

    # 【职责 / R02 R03 R22】retry_points：坐标变换恢复后重试尚未转换的点。
    def retry_points(self):
        with self.lock:
            pending = list(self.pending.items())
        for name, (header, points, received) in pending:
            self.points(name, header, points, received)

    # 【职责 / R02 R03 R22】radar：将雷达数据变换为显示点。
    def radar(self, msg):
        now = time.monotonic()
        if now - self.radar_at < 0.5:
            return
        self.radar_at = now
        try:
            from sensor_msgs_py import point_cloud2
            from types import SimpleNamespace
            # Sample before materializing points; browser view never queues clouds.
            step = max(1, math.ceil(msg.width * msg.height / 1500))
            points = [SimpleNamespace(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                      for i, p in enumerate(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
                      if i % step == 0]
            self.points('radar', msg.header, points)
        except Exception as exc:
            with self.lock:
                self.layers.pop('radar', None)
                self.errors['radar'] = str(exc)

    # 【职责 / R02 R03 R22】snapshot：生成各图层年龄、过期标记和错误信息。
    def snapshot(self, include_global=False):
        now = time.monotonic()
        ros_now = self.node.get_clock().now().nanoseconds * 1e-9
        editor = getattr(self, 'editor', None)
        edit_state = editor.state() if editor is not None else {'ready': False}
        with self.lock:
            layers = {}
            for name, value in self.layers.items():
                # Browser calls keep their original local payload size. Only
                # the backend full-route validator opts into global occupancy.
                if name == 'global_map' and not include_global:
                    continue
                receipt_age = now - value['received']
                stamp_age = ros_now - value['stamp'] if value['stamp'] > 0 else receipt_age
                age = receipt_age if name == 'static_map' else max(receipt_age, stamp_age)
                layers[name] = {**value, 'age': round(age, 2), 'stale': age > 3 or stamp_age < -0.1}
            errors = {name: value for name, value in self.errors.items()
                      if include_global or name != 'global_map'}
            return {'frame': 'odom', 'layers': layers, 'errors': errors,
                    'editing': edit_state}
