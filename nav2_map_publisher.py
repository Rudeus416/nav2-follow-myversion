#!/usr/bin/env python3
"""Start before Nav2 lifecycle activation so static layers have an edited map."""
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from nav2_map_edit import MapEdits


class EditedMapPublisher(Node):
    def __init__(self):
        super().__init__('visual_car_edited_map')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.base = None
        self.publisher = self.create_publisher(OccupancyGrid, '/visual_car/edited_map', qos)
        self.base_sub = self.create_subscription(OccupancyGrid, '/map', self.base_received, qos)
        self.edit_sub = self.create_subscription(OccupancyGrid, '/visual_car/map_edit_input', self.reload, 10)

    def base_received(self, msg):
        self.base = msg
        self.reload()

    def reload(self, _notification=None):
        if self.base is None:
            return
        try:
            # The saved file is authoritative, including after either process restarts.
            editor = MapEdits(Path(__file__).resolve().parent / 'data/nav2_map_edits')
            editor.set_base(self.base)
            msg = editor.compose()
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(msg)
        except Exception as exc:
            self.get_logger().error(f'加载网页阻挡区失败，未发布新地图：{exc}')


if __name__ == '__main__':
    rclpy.init()
    node = EditedMapPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
