#!/usr/bin/env python3
# 【内容标注】用途：单地图模式的外侧禁区边框。
# 对应用户需求：R04（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：在原图四周外侧增加一格占用，并按地图朝向平移原点，保留内部净空间。
# 本次仅加注释；需求关联不是精确创建/提交记录。
"""Map-only navigation boundary, independent of editable and vision layers."""
import copy
import math


# 【职责 / R04】boundary_grid：仅向原图外侧添加一格禁区，原内部占用保持不变。
def boundary_grid(source):
    """One occupied cell outside each original edge; preserve all interior space."""
    result = copy.deepcopy(source)
    width, height = source.info.width, source.info.height
    result.info.width = width + 2
    result.info.height = height + 2
    q = source.info.origin.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    r = source.info.resolution
    result.info.origin.position.x -= r*(math.cos(yaw)-math.sin(yaw))
    result.info.origin.position.y -= r*(math.sin(yaw)+math.cos(yaw))
    data = [100] * ((width+2)*(height+2))
    for y in range(height):
        start = (y+1)*(width+2)+1
        data[start:start+width] = list(source.data[y*width:(y+1)*width])
    result.data = data
    return result


# 【职责 / R04】main：创建外边界 ROS 发布节点。
def main():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    rclpy.init()
    node = Node('visual_car_map_boundary')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(OccupancyGrid, '/visual_car/map_boundary', qos)
    # 【职责 / R04】receive：接到编辑图后生成并发布对应外边框。
    def receive(msg):
        result = boundary_grid(msg)
        result.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(result)
    subscription = node.create_subscription(OccupancyGrid, '/visual_car/edited_map', receive, qos)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
