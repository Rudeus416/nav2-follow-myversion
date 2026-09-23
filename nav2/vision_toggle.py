"""Runtime switch for the optional Nav2 vision costmap layer."""
import threading
from fastapi import HTTPException


def attach(app, engine, motion):
    nav = motion.navigation
    from nav2.vision_layer import VisionLayer
    layer = getattr(nav, 'vision_layer', None) or VisionLayer(nav.node, engine, nav)
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    clients = [nav.node.create_client(SetParameters, name+'/set_parameters')
               for name in ('/local_costmap/local_costmap','/global_costmap/global_costmap')]
    def set_layer(client, enabled):
        if not client.wait_for_service(timeout_sec=1.):
            raise RuntimeError('Nav2 代价地图参数服务未就绪')
        request=SetParameters.Request()
        request.parameters=[Parameter(name='vision_layer.enabled',value=ParameterValue(
            type=ParameterType.PARAMETER_BOOL,bool_value=enabled))]
        done=threading.Event();future=client.call_async(request)
        future.add_done_callback(lambda _:done.set())
        if not done.wait(3.):
            raise RuntimeError('视觉层切换响应超时，请保持强停并重启服务')
        result=future.result()
        if not result.results or not all(r.successful for r in result.results):
            raise RuntimeError('视觉层切换失败：'+str([r.reason for r in result.results]))
    @app.get('/api/nav2/vision-enabled')
    def state():
        import time
        with nav.lock:
            fresh=nav.vision_enabled and nav.vision_at>0 and time.monotonic()-nav.vision_at<=1.2
            return {'enabled':nav.vision_enabled,'stopped':motion.estop,
                    'wall_revision':str(hash(tuple(sorted(layer.cells)))) if fresh else None,
                    'wall_count':len(layer.cells) if fresh else 0}
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
                layer.last=None;layer.cells={}
                nav.vision_count=0;nav.vision_at=0.
                nav.vision_error='等待新的视觉障碍数据' if enabled else ''
                nav.vision_enabled=enabled
                return {'enabled':enabled,'stopped':True}
