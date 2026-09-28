#!/usr/bin/env python3
# 【内容标注】用途：Nav2 启动进度监测。
# 对应用户需求：R21（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：观察启动进度并限制无进展等待，输出失败诊断；具体首次新增轮次无法从对话确认。
# 本次仅加注释；需求关联不是精确创建/提交记录。
"""Bounded, read-only startup monitor; retain Nav2's lifecycle/bond manager."""
import time


# 【职责 / R21】Progress：Nav2 启动进度监测的状态封装；各方法职责见下方标注。
class Progress:
    # 【职责 / R21】__init__：初始化本类依赖与状态；副作用以原初始化语句为准。
    def __init__(self, now, timeout=25.):
        self.signature=None
        self.changed=now
        self.timeout=timeout

    # 【职责 / R21】observe：比较本次节点状态签名；有变化重置计时，否则判断是否超过无进展期限。
    def observe(self, states, now):
        signature=tuple(states)
        if signature!=self.signature:
            self.signature=signature
            self.changed=now
        return now-self.changed>=self.timeout


# 【职责 / R21】main：只读轮询导航节点生命周期状态，保留原生命周期管理器并诊断无进展。
def main():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from lifecycle_msgs.srv import GetState
    from lifecycle_msgs.msg import State
    rclpy.init()
    node=rclpy.create_node('visual_car_nav2_startup_watchdog')
    names=('controller_server','planner_server','bt_navigator')
    clients=[node.create_client(GetState,'/'+name+'/get_state') for name in names]
    started=time.monotonic();progress=Progress(started)
    states=[None]*len(names)
    try:
        while rclpy.ok() and time.monotonic()-started<65:
            for i,client in enumerate(clients):
                if not client.wait_for_service(timeout_sec=.3):continue
                future=client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(node,future,timeout_sec=1.)
                if future.done() and not future.cancelled():
                    result=future.result()
                    if result is not None:states[i]=result.current_state.id
                else:future.cancel()
            description=', '.join(f'{name}={state}' for name,state in zip(names,states))
            if all(state==State.PRIMARY_STATE_ACTIVE for state in states):
                node.get_logger().info('Nav2 三个导航节点均已激活')
                return 0
            if progress.observe(states,time.monotonic()):
                raise RuntimeError('Nav2 启动状态 25 秒未推进：'+description+
                    '；可能为 change_state 响应丢失。停止本次启动，不重复发送状态切换。')
            rclpy.spin_once(node,timeout_sec=.2)
        raise RuntimeError('Nav2 启动超过 65 秒：'+str(dict(zip(names,states))))
    except (KeyboardInterrupt,ExternalShutdownException):
        return 0
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':
    raise SystemExit(main())
