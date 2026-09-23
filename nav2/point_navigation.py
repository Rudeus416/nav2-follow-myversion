"""Explicit execution of a recently previewed map goal."""
import math
import time
from fastapi import HTTPException


def start(nav, goal):
    with nav.lock:
        if nav.pending or nav.canceling or nav.handle is not None:
            raise RuntimeError('旧导航尚未结束，请稍后重试')
        nav.healthy_pose()
        if not nav.client.server_is_ready():
            raise RuntimeError('Nav2 导航服务未就绪')
        request=nav.action_type.Goal()
        request.pose.header.frame_id='odom'
        request.pose.header.stamp=nav.node.get_clock().now().to_msg()
        request.pose.pose.position.x,request.pose.pose.position.y=goal[:2]
        request.pose.pose.orientation.z=math.sin(goal[2]/2)
        request.pose.pose.orientation.w=math.cos(goal[2]/2)
        nav.fixed_goal_active=True
        nav.enabled=True;nav.command_at=0.;nav.command=(0.,0.)
        nav.goal=goal;nav.goal_at=time.monotonic();nav.pending=True
        nav.error='';nav.stop_recorded_pending=False
        generation=nav.generation
        try:
            nav.client.send_goal_async(request).add_done_callback(lambda f:nav._accepted(f,generation))
        except Exception:
            nav.pending=False;nav.stop('固定终点发送失败')
            raise


def attach(app, motion, view, selection):
    @app.post('/api/nav2/point-start')
    def execute(payload:dict):
        with motion.lock:
            nav=motion.navigation
            if not motion.estop or motion.radar_reconfiguring:
                raise HTTPException(409,'请保持强停并完成传感器设置')
            with nav.lock:
                saved=selection.get('value')
                state=view.snapshot().get('editing',{})
                if not saved or any(payload.get(k)!=saved['payload'].get(k) for k in ('x','y','base_id','revision')):
                    raise HTTPException(409,'请先预览当前选点')
                if any(state.get(k)!=saved['payload'].get(k) for k in ('base_id','revision')):
                    raise HTTPException(409,'地图已更改，请重新预览')
                if (nav.preview_sequence!=saved['sequence'] or nav.preview['state']!='ready'
                        or not nav.preview.get('points') or time.monotonic()-nav.preview_at>30):
                    raise HTTPException(409,'预览无效或超过 30 秒，请重新预览')
                try:start(nav,saved['goal'])
                except Exception as exc:raise HTTPException(409,str(exc)) from exc
                motion.following=False;motion.mode='auto';motion.estop=False
                selection.clear()
                return {'message':'已开始导航到所选终点，强停按钮可随时停车'}
    @app.post('/api/nav2/point-stop')
    def stop():
        return motion.set_estop(True)
