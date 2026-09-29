# 【内容标注 / R33】独立导航 TF 接收，避免原单线程回调等导航锁时阻塞定位更新。
# 只订阅 TF，不迁移原节点、不发布底盘指令、不修改定位新鲜度门槛。
"""A private TF node/executor, with bounded and idempotent cleanup."""
import logging
import threading


# 【职责 / R33】PoseListener：用独立 ROS 节点和执行线程接收导航 TF，避免控制锁饿死定位。
class PoseListener:
    # 【职责 / R33】__init__：在同一 ROS context 创建专用 TF listener 和执行器，并处理半初始化清理。
    def __init__(self, source_node, buffer):
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from tf2_ros import TransformListener

        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._closed = threading.Event()
        self._disposed = False
        self._node = self._executor = self._listener = self._thread = None
        self.error = ''
        try:
            self._node = Node(
                'nav2_pose_listener_' + format(id(self), 'x'),
                namespace=source_node.get_namespace(), context=source_node.context,
                use_global_arguments=False, start_parameter_services=False,
                parameter_overrides=[Parameter('use_sim_time',
                    value=bool(source_node.get_parameter('use_sim_time').value))])
            self._executor = SingleThreadedExecutor(context=source_node.context)
            self._listener = TransformListener(buffer, self._node, spin_thread=False)
            # NEVER add source_node: rclpy would remove it from its original executor.
            self._executor.add_node(self._node)
            source_node.context.on_shutdown(self._on_shutdown)
            if self._stop.is_set():
                raise RuntimeError('TF listener context already shutting down')
            self._thread = threading.Thread(target=self._spin,
                name='nav2-pose-listener', daemon=True)
            self._thread.start()
        except BaseException:
            self.close()
            raise

    # 【职责 / R33】_on_shutdown：context 关闭回调只置停止事件，避免在 context 锁内销毁 ROS 对象。
    def _on_shutdown(self):
        # Humble invokes context callbacks under its context lock. Do not join
        # or destroy ROS objects there: spin may need that same context lock.
        self._stop.set()

    # 【职责 / R33】_spin：独立运行 TF 执行器；正常关闭和异常都进入幂等释放。
    def _spin(self):
        from rclpy.executors import ExternalShutdownException, ShutdownException
        try:
            while not self._stop.is_set():
                self._executor.spin_once(timeout_sec=.1)
        except (ExternalShutdownException, ShutdownException):
            pass
        except Exception as exc:
            self.error = '独立 TF 监听线程异常：' + str(exc)
            logging.getLogger(__name__).exception(self.error)
        finally:
            self._dispose()

    # 【职责 / R33】close：停止并有界等待自有监听线程，不等待当前线程自身。
    def close(self):
        """Stop only our listener; never join self or destroy an active callback."""
        self._stop.set()
        thread = self._thread
        if thread is threading.current_thread():
            return False  # The spin finally block performs cleanup.
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.)
            if thread.is_alive():
                self.error = '独立 TF 监听线程尚未退出，等待后台完成清理'
                return False
        self._dispose()
        return self._closed.is_set()

    # 【职责 / R33】_dispose：只销毁本监听器创建的执行器、节点和关闭回调。
    def _dispose(self):
        with self._lock:
            if self._disposed:
                return
            self._disposed = True
            self._stop.set()
            # Each cleanup is independent so partial construction also unwinds.
            tasks = []
            if self._listener is not None:
                tasks.append(self._listener.unregister)
            if self._executor is not None:
                if self._node is not None:
                    tasks.append(lambda: self._executor.remove_node(self._node))
                tasks.append(lambda: self._executor.shutdown(timeout_sec=0.))
            if self._node is not None:
                tasks.append(self._node.destroy_node)
            for action in tasks:
                try:
                    action()
                except Exception:
                    logging.getLogger(__name__).debug('TF listener cleanup failed', exc_info=True)
            self._closed.set()
