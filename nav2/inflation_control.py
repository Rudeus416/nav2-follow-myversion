# 【内容标注】用途：紫色硬缓冲参数网页控制。
# 对应用户需求：R12 R14 R21（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：强停下写两张代价地图的参数并读回；不确定状态阻止执行，成功使旧预览失效。
# 本次仅加注释；需求关联不是精确创建/提交记录。
"""Runtime inflation settings for both Nav2 costmaps, only while stopped."""
import math
import threading
from fastapi import HTTPException


# 【职责 / R12 R14 R21】validate：检查开关与半径的合法范围。
def validate(payload):
    enabled=payload.get('enabled');radius=payload.get('radius')
    if type(enabled) is not bool or type(radius) not in (int,float) or not math.isfinite(radius) or not 0<=radius<=1.5:
        raise ValueError('启用状态必须为布尔值，半径须为 0～1.5 米')
    return enabled,float(radius)


# 【职责 / R12 R14 R21】attach：注册本模块专用接口与依赖，复用既有应用，不修改原业务实现。
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
    # 【职责 / R12 R14 R21】spin：在自有线程运行参数客户端执行器，不占用主控制回调。
    def spin():
        try:executor.spin()
        except ExternalShutdownException:pass
    worker=threading.Thread(target=spin,name='nav2-inflation-client',daemon=True)
    worker.start()
    # 【职责 / R12 R14 R21】close：停止自有参数执行器并有界等待线程退出。
    def close():
        executor.shutdown(timeout_sec=3.)
        worker.join(timeout=3.)
        parameter_node.destroy_node()
    app.add_event_handler('shutdown',close)
    group=ReentrantCallbackGroup()
    names=['inflation.enabled','inflation.inflation_radius']
    plugin_names=['inflation.plugin']
    clients=[(parameter_node.create_client(GetParameters,p+'/get_parameters',callback_group=group),
              parameter_node.create_client(SetParametersAtomically,p+'/set_parameters_atomically',callback_group=group))
             for p in ('/local_costmap/local_costmap','/global_costmap/global_costmap')]
    serial=threading.Lock()
    # 【职责 / R12 R14 R21】call：封装 ROS 参数服务请求及超时。
    def call(client,request):
        if not client.wait_for_service(timeout_sec=.5):raise RuntimeError('Nav2 参数服务未就绪')
        event=threading.Event();future=client.call_async(request)
        future.add_done_callback(lambda _:event.set())
        if not event.wait(2.):
            future.cancel()
            raise TimeoutError('Nav2 参数响应超时，状态不确定，请保持强停并重新应用')
        return future.result()
    # 【职责 / R12 R14 R21】read：读回代价地图实际参数，避免仅相信写入请求。
    def read():
        result=[]
        for getter,_ in clients:
            req=GetParameters.Request();req.names=names+plugin_names
            values=call(getter,req).values
            if len(values)!=3 or values[0].type!=ParameterType.PARAMETER_BOOL or values[1].type!=ParameterType.PARAMETER_DOUBLE:
                raise RuntimeError('膨胀层参数类型无效，请重启 Nav2')
            if values[2].type!=ParameterType.PARAMETER_STRING or values[2].string_value!='visual_car_nav2::HardBuffer':
                raise RuntimeError('尚未加载整车禁入缓冲插件，请停车重启 Nav2')
            result.append({'enabled':values[0].bool_value,'radius':values[1].double_value})
        return {'local':result[0],'global':result[1],'consistent':result[0]==result[1], 'stopped':motion.estop}
    # 【职责 / R12 R14 R21】status：报告已确认的缓冲参数和错误状态。
    @app.get('/api/nav2/inflation')
    def status():
        with serial:
            try:return read()
            except Exception as exc:raise HTTPException(503,str(exc)) from exc
    # 【职责 / R12 R14 R21】apply：强停下修改两张图并确认一致；失败保持配置错误拦截。
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
                if not result['consistent'] or any(result['local'][k]!=v for k,v in {'enabled':enabled,'radius':radius}.items()):
                    raise RuntimeError('两张代价地图参数未一致，请保持强停并重新应用')
                with nav.lock:nav.inflation_config_error=''
                return result
            except Exception as exc:
                with nav.lock:nav.inflation_config_error='膨胀层设置未确认：'+str(exc)
                raise HTTPException(503,str(exc)) from exc
