#!/usr/bin/env python3
# 【内容标注】用途：编辑地图的 ROS 发布桥。
# 对应用户需求：R03 R04（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：接收底图并重载手绘禁区，发布导航使用的编辑后地图。
# 需求关联用于追溯用途，不代表精确创建/提交记录；后续自检修复见 nav2/AUDIT.md。
"""Start before Nav2 lifecycle activation so static layers have an edited map."""
from pathlib import Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from nav2_map_edit import MapEdits


# 【职责 / R03 R04】EditedMapPublisher：编辑地图的 ROS 发布桥的状态封装；各方法职责见下方标注。
class EditedMapPublisher(Node):
    # 【职责 / R03 R04】__init__：初始化本类依赖与状态；副作用以原初始化语句为准。
    def __init__(self):
        super().__init__('visual_car_edited_map')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.base = None
        self.publisher = self.create_publisher(OccupancyGrid, '/visual_car/edited_map', qos)
        self.base_sub = self.create_subscription(OccupancyGrid, '/map', self.base_received, qos)
        self.edit_sub = self.create_subscription(OccupancyGrid, '/visual_car/map_edit_input', self.reload, 10)

    # 【职责 / R03 R04】base_received：收到原始底图后更新合成输出。
    def base_received(self, msg):
        self.base = msg
        self.reload()

    # 【职责 / R03 R04】reload：读取编辑内容并重发编辑后地图。
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
    except (KeyboardInterrupt, ExternalShutdownException):
        # 【自检修复 / R21 R22】launch 的正常停服会先关闭 ROS context；
        # 将它作为正常退出，避免把停车重启误报为地图进程崩溃。
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
