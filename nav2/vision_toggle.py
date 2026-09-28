# 【内容标注】用途：视觉障碍层网页开关与状态。
# 对应用户需求：R07 R22（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：独立开关视觉参与导航；管理视觉工作线程、数据时效和诊断快照。
# 本次仅加注释；需求关联不是精确创建/提交记录。
"""Runtime switch for the optional Nav2 vision costmap layer."""
import threading
from fastapi import HTTPException


# 【职责 / R07 R22】attach：注册本模块专用接口与依赖，复用既有应用，不修改原业务实现。
def attach(app, engine, motion):
    nav = motion.navigation
    from nav2.vision_layer import VisionLayer
    layer = getattr(nav, 'vision_layer', None) or VisionLayer(nav.node, engine, nav)
    from nav2.vision_calibration import attach as attach_calibration
    attach_calibration(app, engine, motion, layer)
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
    parameter_node=Node('visual_car_vision_parameter_client',context=nav.node.context)
    executor=SingleThreadedExecutor(context=nav.node.context)
    executor.add_node(parameter_node)
    def spin():
        try:executor.spin()
        except ExternalShutdownException:pass
    worker=threading.Thread(target=spin,daemon=True)
    worker.start()
    def close():
        executor.shutdown(timeout_sec=3.)
        worker.join(timeout=3.)
        parameter_node.destroy_node()
    app.add_event_handler('shutdown',close)
    app.add_event_handler('shutdown',layer.close)
    clients = [parameter_node.create_client(SetParameters, name+'/set_parameters')
               for name in ('/local_costmap/local_costmap','/global_costmap/global_costmap')]
    # 【职责 / R07 R22】set_layer：启停导航视觉层并同步内部状态。
    def set_layer(client, enabled):
        if not client.wait_for_service(timeout_sec=1.):
            raise RuntimeError('Nav2 代价地图参数服务未就绪')
        request=SetParameters.Request()
        request.parameters=[Parameter(name='vision_layer.enabled',value=ParameterValue(
            type=ParameterType.PARAMETER_BOOL,bool_value=enabled))]
        done=threading.Event();future=client.call_async(request)
        future.add_done_callback(lambda _:done.set())
        if not done.wait(3.):
            future.cancel()
            raise RuntimeError('视觉层切换响应超时，请保持强停并重启服务')
        result=future.result()
        if not result.results or not all(r.successful for r in result.results):
            raise RuntimeError('视觉层切换失败：'+str([r.reason for r in result.results]))
    # 【职责 / R07 R22】state：汇总视觉源帧年龄、候选和诊断。
    @app.get('/api/nav2/vision-enabled')
    def state():
        import time
        with nav.lock:
            fresh=nav.vision_enabled and nav.vision_at>0 and time.monotonic()-nav.vision_at<=1.2
            return {'enabled':nav.vision_enabled,'stopped':motion.estop,
                    'wall_revision':str(hash(tuple(sorted(layer.camera_cells)))) if fresh else None,
                    'wall_count':len(layer.camera_cells) if fresh else 0,
                    'map_wall_count':len(layer.cells) if fresh else 0,
                    'diagnostics':dict(layer.diagnostics) if fresh else {},
                    'timing':dict(getattr(layer,'timing',{})),
                    'semantic':layer.semantic.status(),
                    'semantic_frame':layer.semantic_preview['frame_id'] if layer.semantic_preview and fresh else None,
                    'reason':layer.diagnostics.get('reason','等待视觉候选') if fresh else '视觉数据未就绪或已过期'}
    # 【职责 / R07 R22】toggle：在允许的停车状态变更视觉障碍选项。
    @app.post('/api/nav2/vision-enabled')
    def toggle(payload:dict):
        enabled=payload.get('enabled')
        if type(enabled) is not bool:raise HTTPException(422,'enabled 必须为布尔值')
        with motion.lock:
            if not motion.estop:raise HTTPException(409,'请先强制停止，再切换视觉障碍检测')
            with nav.lock:
                nav.stop('切换视觉障碍检测')
                nav.clear_preview()
                try:
                    for client in clients:set_layer(client,enabled)
                except Exception as exc:
                    # Fail closed if either layer state is uncertain.
                    nav.vision_enabled=True
                    nav.vision_error=str(exc)
                    nav.vision_at=0.
                    layer.switch_failed=True
                    raise HTTPException(503,str(exc)) from exc
                layer.switch_failed=False
                layer.last=None;layer.cells={};layer.camera_cells={};layer.diagnostics={}
                nav.vision_count=0;nav.vision_at=0.
                nav.vision_error='等待新的视觉障碍数据' if enabled else ''
                nav.vision_enabled=enabled
                return {'enabled':enabled,'stopped':True}
