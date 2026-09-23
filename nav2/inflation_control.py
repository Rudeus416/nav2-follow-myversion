"""Runtime inflation settings for both Nav2 costmaps, only while stopped."""
import math
import threading
from fastapi import HTTPException


def validate(payload):
    enabled=payload.get('enabled');radius=payload.get('radius')
    if type(enabled) is not bool or type(radius) not in (int,float) or not math.isfinite(radius) or not 0<=radius<=1.5:
        raise ValueError('启用状态必须为布尔值，半径须为 0～1.5 米')
    return enabled,float(radius)


def attach(app, motion):
    from rcl_interfaces.srv import GetParameters, SetParametersAtomically
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
    nav=motion.navigation
    # Keep service responses independent of control callbacks waiting on motion.lock.
    parameter_node=Node('visual_car_inflation_client',context=nav.node.context)
    executor=SingleThreadedExecutor(context=nav.node.context)
    executor.add_node(parameter_node)
    def spin():
        try:executor.spin()
        except ExternalShutdownException:pass
    worker=threading.Thread(target=spin,name='nav2-inflation-client',daemon=True)
    worker.start()
    def close():
        executor.shutdown(timeout_sec=3.)
        worker.join(timeout=3.)
        parameter_node.destroy_node()
    app.add_event_handler('shutdown',close)
    group=ReentrantCallbackGroup()
    names=['inflation.enabled','inflation.inflation_radius']
    clients=[(parameter_node.create_client(GetParameters,p+'/get_parameters',callback_group=group),
              parameter_node.create_client(SetParametersAtomically,p+'/set_parameters_atomically',callback_group=group))
             for p in ('/local_costmap/local_costmap','/global_costmap/global_costmap')]
    serial=threading.Lock()
    def call(client,request):
        if not client.wait_for_service(timeout_sec=.5):raise RuntimeError('Nav2 参数服务未就绪')
        event=threading.Event();future=client.call_async(request)
        future.add_done_callback(lambda _:event.set())
        if not event.wait(2.):
            future.cancel()
            raise TimeoutError('Nav2 参数响应超时，状态不确定，请保持强停并重新应用')
        return future.result()
    def read():
        result=[]
        for getter,_ in clients:
            req=GetParameters.Request();req.names=names
            values=call(getter,req).values
            if len(values)!=2 or values[0].type!=ParameterType.PARAMETER_BOOL or values[1].type!=ParameterType.PARAMETER_DOUBLE:
                raise RuntimeError('膨胀层参数类型无效，请重启 Nav2')
            result.append({'enabled':values[0].bool_value,'radius':values[1].double_value})
        return {'local':result[0],'global':result[1],'consistent':result[0]==result[1], 'stopped':motion.estop}
    @app.get('/api/nav2/inflation')
    def status():
        with serial:
            try:return read()
            except Exception as exc:raise HTTPException(503,str(exc)) from exc
    @app.post('/api/nav2/inflation')
    def apply(payload:dict):
        try:enabled,radius=validate(payload)
        except ValueError as exc:raise HTTPException(422,str(exc)) from exc
        with serial, motion.lock:
            if not motion.estop:raise HTTPException(409,'请先强停再调整缓冲区')
            nav.stop('调整膨胀缓冲区');nav.clear_preview()
            with nav.lock:nav.inflation_config_error='正在更新膨胀层参数'
            try:
                for _,setter in clients:
                    req=SetParametersAtomically.Request()
                    req.parameters=[Parameter(name=names[0],value=ParameterValue(type=ParameterType.PARAMETER_BOOL,bool_value=enabled)),
                                    Parameter(name=names[1],value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=radius))]
                    try:
                        answer=call(setter,req).result
                    except TimeoutError:
                        # Do not resend: both costmaps are read back below before success.
                        continue
                    if not answer.successful:raise RuntimeError(answer.reason)
                result=read()
                if not result['consistent'] or result['local']!={'enabled':enabled,'radius':radius}:
                    raise RuntimeError('两张代价地图参数未一致，请保持强停并重新应用')
                with nav.lock:nav.inflation_config_error=''
                return result
            except Exception as exc:
                with nav.lock:nav.inflation_config_error='膨胀层设置未确认：'+str(exc)
                raise HTTPException(503,str(exc)) from exc
