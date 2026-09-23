"""Read-only ROS snapshots for the browser's odom-frame obstacle view."""
import math
import threading
import time
import os
from pathlib import Path


class ObstacleView:
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
            self.subscriptions.append(node.create_subscription(OccupancyGrid, '/map', self.static_grid, map_qos))
        self.retry_timer = node.create_timer(0.02, self.retry_points)
        self.pose_timer = node.create_timer(0.2, self.refresh_pose)

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

    def publish_edited_map(self, msg):
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.edited_publisher.publish(msg)
        self.static_message = msg
        with self.lock:
            self.errors.pop('map_edit', None)
            self.layers.pop('path', None)
        self.refresh_pose()

    def edit_map(self, action, rect, base_id, revision):
        if self.editor is None:
            raise ValueError('仅静态地图或叠加模式支持编辑')
        with self.editor.lock:
            msg = self.editor.edit(action, rect, base_id, revision)
            self.publish_edited_map(msg)
            return self.editor.state()

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

    def store(self, name, header, data):
        stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
        self.layers[name] = dict(data, stamp=stamp, received=time.monotonic())
        self.errors.pop(name, None)

    def grid(self, msg):
        with self.lock:
            if msg.header.frame_id != 'odom':
                self.layers.pop('map', None)
                self.errors['map'] = '地图坐标系不是 odom，无法叠加显示'
                return
            info = msg.info
            if not (0 < info.width * info.height <= 1000000 and info.resolution > 0
                    and len(msg.data) == info.width * info.height):
                self.layers.pop('map', None)
                self.errors['map'] = '地图尺寸无效或超过显示上限'
                return
            q = info.origin.orientation
            self.store('map', msg.header, dict(width=info.width, height=info.height,
                resolution=info.resolution, origin=[info.origin.position.x, info.origin.position.y],
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)), data=list(msg.data)))

    def update_grid(self, msg):
        with self.lock:
            grid = self.layers.get('map')
            if grid is None or msg.header.frame_id != 'odom':
                return
            if (msg.x + msg.width > grid['width'] or msg.y + msg.height > grid['height']
                    or len(msg.data) != msg.width * msg.height):
                self.errors['map'] = '地图增量尺寸不匹配，等待完整地图'
                self.layers.pop('map', None)
                return
            data = grid['data'].copy()
            for row in range(msg.height):
                start = (msg.y + row) * grid['width'] + msg.x
                data[start:start + msg.width] = msg.data[row*msg.width:(row+1)*msg.width]
            self.store('map', msg.header, dict(grid, data=data))

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

    def retry_points(self):
        with self.lock:
            pending = list(self.pending.items())
        for name, (header, points, received) in pending:
            self.points(name, header, points, received)

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

    def snapshot(self):
        now = time.monotonic()
        ros_now = self.node.get_clock().now().nanoseconds * 1e-9
        editor = getattr(self, 'editor', None)
        edit_state = editor.state() if editor is not None else {'ready': False}
        with self.lock:
            layers = {}
            for name, value in self.layers.items():
                receipt_age = now - value['received']
                stamp_age = ros_now - value['stamp'] if value['stamp'] > 0 else receipt_age
                age = receipt_age if name == 'static_map' else max(receipt_age, stamp_age)
                layers[name] = {**value, 'age': round(age, 2), 'stale': age > 3 or stamp_age < -0.1}
            return {'frame': 'odom', 'layers': layers, 'errors': dict(self.errors),
                    'editing': edit_state}
