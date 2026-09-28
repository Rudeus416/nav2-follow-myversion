#!/usr/bin/env python3
# 【内容标注】用途：静态地图启动就绪检查。
# 对应用户需求：R01 R21（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：等待地图相关服务和节点就绪，失败退出以阻止不完整导航启动。
# 本次仅加注释；需求关联不是精确创建/提交记录。
"""Bounded map lifecycle startup; reconcile state after a lost response."""
import os
import time


# 【职责 / R21】main：在总时限内读取并推进 map_server 生命周期，等待 map/edited_map/边界图就绪后退出。
def main():
    import rclpy
    from lifecycle_msgs.srv import GetState, ChangeState
    from lifecycle_msgs.msg import State, Transition
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import QoSProfile, DurabilityPolicy
    rclpy.init()
    node = rclpy.create_node('visual_car_map_startup')
    state = node.create_client(GetState, '/map_server/get_state')
    change = node.create_client(ChangeState, '/map_server/change_state')
    topics = ['/map', '/visual_car/edited_map']
    if os.environ.get('NAV2_OBSTACLE_MODE') == 'map':
        topics.append('/visual_car/map_boundary')
    received = set()
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [node.create_subscription(OccupancyGrid, topic,
        lambda msg, topic=topic: received.add(topic) if msg.info.width and msg.info.height else None,
        qos) for topic in topics]
    deadline = time.monotonic() + 45
    attempted = set()
    # 【职责 / R21】call：有限等待生命周期服务；响应丢失返回空值，由外层重新查询真实状态。
    def call(client, request):
        if not client.wait_for_service(timeout_sec=1.):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=min(3., max(0., deadline-time.monotonic())))
        if not future.done():
            future.cancel()
            return None
        return future.result()
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            response = call(state, GetState.Request())
            if response is None:
                continue
            current = response.current_state.id
            if current == State.PRIMARY_STATE_ACTIVE:
                if received.issuperset(topics):
                    node.get_logger().info('地图已激活，静态地图及边界图层就绪')
                    return 0
                rclpy.spin_once(node, timeout_sec=.1)
                continue
            transitions = {State.PRIMARY_STATE_UNCONFIGURED: Transition.TRANSITION_CONFIGURE,
                           State.PRIMARY_STATE_INACTIVE: Transition.TRANSITION_ACTIVATE}
            if current not in transitions:
                raise RuntimeError(f'地图处于异常生命周期状态 {current}')
            # Never blindly resend a transition after a lost response.
            if current in attempted:
                rclpy.spin_once(node, timeout_sec=.1)
                continue
            attempted.add(current)
            request = ChangeState.Request()
            request.transition.id = transitions[current]
            result = call(change, request)
            if result is not None and not result.success:
                raise RuntimeError(f'地图状态切换失败：{current}')
            if result is None:
                node.get_logger().warning('地图切换响应超时；查询实际状态，不重复发送切换')
        raise RuntimeError(f'地图启动超时；尚未收到图层：{sorted(set(topics)-received)}')
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
