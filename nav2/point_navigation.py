# 【内容标注】用途：人工地图选点的执行接口。
# 对应用户需求：R11 R13 R22（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：复核地图版本、起点、路线及视觉时效，再执行既有预览；不依赖人物跟踪。
# 本次仅加注释；需求关联不是精确创建/提交记录。
"""Explicit execution of a recently previewed map goal."""
import math
import logging
import time
from fastapi import HTTPException
from nav2_follow import VisionStale

VISION_START_WAIT = 2.0


# 【职责 / R11 R13 R22】start：提交已核对的固定终点路径，复用原速度通道。
def start(nav, goal, path):
    with nav.lock:
        if nav.pending or nav.canceling or nav.handle is not None:
            raise RuntimeError('旧导航尚未结束，请稍后重试')
        nav.healthy_pose()
        if not nav.path_client.server_is_ready():
            raise RuntimeError('Nav2 导航服务未就绪')
        request=nav.path_action_type.Goal()
        request.path=path
        request.controller_id='FollowPath'
        request.goal_checker_id='goal_checker'
        nav.fixed_goal_active=True
        nav.enabled=True;nav.command_at=0.;nav.command=(0.,0.)
        nav.goal=goal;nav.goal_at=time.monotonic();nav.pending=True
        nav.error='';nav.stop_recorded_pending=False
        generation=nav.generation
        try:
            nav.path_client.send_goal_async(request).add_done_callback(lambda f:nav._accepted(f,generation))
        except Exception:
            nav.pending=False;nav.stop('固定终点发送失败')
            raise


# 【职责 / R11 R13 R22】attach：注册本模块专用接口与依赖，复用既有应用，不修改原业务实现。
def attach(app, motion, view, selection):
    from nav2_msgs.action import FollowPath
    from rclpy.action import ActionClient
    nav=motion.navigation
    if not hasattr(nav, 'path_client'):
        nav.path_action_type=FollowPath
        nav.path_client=ActionClient(nav.node, FollowPath, '/follow_path')
    # 【职责 / R11 R13 R22】execute_checked：锁外整车复核，锁内重查地图/起点/序号并授权。
    def execute_checked(payload:dict):
        nav=motion.navigation
        with motion.lock, nav.lock:
            if not motion.estop:
                raise HTTPException(409,'请先强停')
            path=getattr(nav,'preview_full_path',None)
            sequence=nav.preview_sequence
            generation=nav.generation
            if path is None or nav.preview['state']!='ready':
                raise HTTPException(409,'请先完成有效路径预览')
        validation_at=time.monotonic()
        try:
            nav.validate_preview(path)
        except Exception as exc:
            raise HTTPException(409,str(exc)) from exc
        finally:
            logging.getLogger(__name__).warning('选点路径复核耗时 %.3f 秒', time.monotonic()-validation_at)
        lock_at=time.monotonic()
        with motion.lock:
            nav=motion.navigation
            if not motion.estop or motion.radar_reconfiguring:
                raise HTTPException(409,'请保持强停并完成传感器设置')
            with nav.lock:
                if nav.preview_sequence!=sequence or nav.generation!=generation or nav.preview_full_path is not path:
                    raise HTTPException(409,'状态已改变，请重新预览')
                logging.getLogger(__name__).warning(
                    '选点启动锁等待 %.3f 秒，视觉年龄 %.3f 秒',
                    time.monotonic()-lock_at,
                    time.monotonic()-getattr(nav,'vision_at',time.monotonic()))
                rx,ry,ra=nav.healthy_pose()
                first=path.poses[0].pose
                q=first.orientation
                pa=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                if math.hypot(rx-first.position.x,ry-first.position.y)>.05 or abs(math.atan2(math.sin(ra-pa),math.cos(ra-pa)))>.1:
                    raise HTTPException(409,'车位已偏离预览起点，请重新预览')
                saved=selection.get('value')
                state=view.snapshot().get('editing',{})
                if not saved or any(payload.get(k)!=saved['payload'].get(k) for k in ('x','y','base_id','revision')):
                    raise HTTPException(409,'请先预览当前选点')
                if any(state.get(k)!=saved['payload'].get(k) for k in ('base_id','revision')):
                    raise HTTPException(409,'地图已更改，请重新预览')
                if (nav.preview_sequence!=saved['sequence'] or nav.preview['state']!='ready'
                        or not nav.preview.get('points') or time.monotonic()-nav.preview_at>30):
                    raise HTTPException(409,'预览无效或超过 30 秒，请重新预览')
                try:start(nav,saved['goal'],path)
                except VisionStale:raise
                except Exception as exc:raise HTTPException(409,str(exc)) from exc
                motion.following=False;motion.mode='auto';motion.estop=False
                selection.clear()
                return {'message':'已开始导航到所选终点，强停按钮可随时停车'}
    # 【职责 / R11 R13 R22】execute：处理显式选点启动；有限等待新鲜视觉数据，失败保持停车。
    @app.post('/api/nav2/point-start')
    def execute(payload:dict):
        # 等待时只读状态，不反复做整条路径复核，不持锁休眠。
        # 超时预算从首次发现旧数据开始；当前请求结束后不遗留启动任务。
        deadline=None
        retry_after=None
        with motion.lock, nav.lock:
            identity=(nav.preview_sequence,nav.generation)
        try:
            while True:
                stale=None
                with motion.lock, nav.lock:
                    if (not motion.estop or motion.radar_reconfiguring
                            or identity!=(nav.preview_sequence,nav.generation)):
                        raise HTTPException(409,'等待期间状态已改变，请重新预览')
                    try:
                        nav.check_vision()
                        if (retry_after is not None and getattr(nav,'vision_enabled',False)
                                and nav.vision_at<=retry_after):
                            raise VisionStale('等待比上次复核更新的视觉帧')
                    except VisionStale as exc:
                        stale=exc
                if stale is None:
                    try:
                        return execute_checked(payload)
                    except VisionStale as exc:
                        # 复核期间过期：等更新帧后再从头复核，不能直接放行旧路径。
                        stale=exc
                        with nav.lock:
                            retry_after=getattr(nav,'vision_at',0.)
                now=time.monotonic()
                if deadline is None:
                    deadline=now+VISION_START_WAIT
                    logging.getLogger(__name__).warning(
                        '选点等待视觉新帧（停车）：年龄 %.3f 秒，等待上限 %.1f 秒',
                        now-getattr(nav,'vision_at',now),VISION_START_WAIT)
                if now>=deadline:
                    raise HTTPException(409,'视觉数据持续超时，保持停车；请等待检测恢复后重新启动') from stale
                time.sleep(min(.05,deadline-now))
        except HTTPException as exc:
            logging.getLogger(__name__).warning('选点行驶被拒绝 [%s]：%s', exc.status_code, exc.detail)
            raise
        except RuntimeError as exc:
            logging.getLogger(__name__).warning('选点行驶未就绪：%s', exc)
            raise HTTPException(409,str(exc)) from exc

    # 【职责 / R11 R13 R22】stop：显式停止选点任务并锁存强停。
    @app.post('/api/nav2/point-stop')
    def stop():
        return motion.set_estop(True)
