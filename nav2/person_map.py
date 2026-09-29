# 【内容标注】用途：人物图标、历史点和人物网页接口。
# 对应用户需求：R15 R16 R17 R18 R30 R31 R33（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：同帧人物与历史 TF 定位；短时缓存和点击锁定点分离；接入规划、启动、结束按钮。
# 需求关联不是精确创建/提交记录；按钮预检反馈与显式请求拒绝日志仅修改自有接口。
"""Read-only person markers plus an explicit start using the existing follow flow."""
import logging
import math
import time
import threading
import numpy as np
from fastapi import HTTPException
from nav2.motion_geometry import person_position
from nav2.person_execution import readiness


logger = logging.getLogger(__name__)


# 【职责 / R15 R16 R17 R18】observed_people：匹配检测与深度同帧记录，使用拍摄时 TF 投影人物位置。
def observed_people(engine, motion, now=None):
    """Copy matched fusion/track records before taking motion/nav locks."""
    now=time.monotonic() if now is None else now
    with engine.lock:
        version=engine._config_version
        epoch=engine._stream_epoch
        tracks=dict(engine._buffer_m1)
        fusion=list(engine._buffer_f.values())
        current=[t for t in tracks.values() if t.config_version==version and t.frame.stream_epoch==epoch]
        latest=max(current,key=lambda t:t.frame.source_at) if current else None
        visible={p.id for p in latest.people} if latest and 0<=now-latest.frame.source_at<=1.2 else set()
        matched=[(f,tracks.get(f.frame_id)) for f in fusion if f.config_version==version]
        matched=[(f,t) for f,t in matched if t is not None and t.config_version==version
                 and t.frame.stream_epoch==epoch and 0<=now-t.frame.source_at<=1.2]
        if not matched:return []
        fused,tracking=max(matched,key=lambda pair:pair[1].frame.source_at)
        values=[(p.id,p.distance,tuple(p.center)) for p in fused.people if p.id in visible]
        calibration=tracking.calibration
        frame=tracking.frame
    nav=motion.navigation
    from rclpy.time import Time
    with nav.lock:
        mount=nav.camera_pose
        if mount is None:return []
        # Same frame's pose prevents a moving robot projecting an old person
        # observation using its current position.
        transform=nav.tf.lookup_transform('odom','base_link',Time(nanoseconds=frame.stamp_ns)).transform
    q=transform.rotation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    people=[]
    for ident,distance,center in values:
        if distance is None or not math.isfinite(distance) or distance<=0:continue
        ray,_=calibration.normalized(np.array([center[0]]),np.array([center[1]]))
        bearing=-math.atan(float(ray[0]))
        x,y=person_position(distance,bearing,transform.translation.x,transform.translation.y,yaw,*mount)
        people.append({'id':int(ident),'position':[x,y],'age':max(0.,time.monotonic()-frame.source_at),'source_at':frame.source_at})
    return people


# 【职责 / R15 R16 R17 R18】DisplayPeopleCache：人物图标、历史点和人物网页接口的状态封装；各方法职责见下方标注。
class DisplayPeopleCache:
    """Three-second last-seen observations for display only."""
    # 【职责 / R15 R16 R17 R18】__init__：初始化本类依赖与状态；副作用以原初始化语句为准。
    def __init__(self):
        self.lock = threading.Lock()
        self.key = None
        self.records = {}

    # 【职责 / R15 R16 R17 R18】update：维护最多 3 秒显示缓存；不将重复轮询变成新鲜测量。
    def update(self, people, key, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            if key != self.key:
                self.records.clear()
                self.key = key
            fresh = set()
            for person in people:
                age = now-person['source_at']
                if 0 <= age <= 1.2:
                    self.records[person['id']] = dict(person)
                    fresh.add(person['id'])
            self.records = {ident:p for ident,p in self.records.items()
                            if 0 <= now-p['source_at'] <= 3.}
            return [dict(p, age=now-p['source_at'], stale=ident not in fresh)
                    for ident,p in self.records.items()]


# 【R33】历史点无年龄上限，仅显式规划可读取；不能冒充连续模式的新观测。
class LatestPersonPoint:
    """Latest valid world point for one selected ID and camera scope.

    Accept only fresh finite measurements, but do not expire a point that was
    valid when observed. Polling and explicit planning never renew source_at.
    Selection/config/stream changes discard the old identity's point.
    """
    # 【职责 / R33】LatestPersonPoint.__init__：初始化所选 ID/相机作用域的不限时最近世界点。
    def __init__(self):
        self.key = None
        self.point = None

    # 【职责 / R33】LatestPersonPoint.update：只由真实新观测更新历史点；身份改变即丢弃旧点。
    def update(self, people, key, now=None):
        # The owning routes serialize this cache under motion/nav locks.
        now = time.monotonic() if now is None else now
        if self.key != key:
            self.key, self.point = key, None
        for person in people:
            if person.get('id') != key[-1]:
                continue
            try:
                stamp = float(person['source_at'])
                position = tuple(float(v) for v in person['position'])
                valid = (len(position) == 2 and all(math.isfinite(v) for v in position)
                         and 0 <= now-stamp <= 1.2)
            except (KeyError, TypeError, ValueError, OverflowError):
                continue
            if valid and (self.point is None or stamp > self.point['source_at']):
                self.point = dict(person, position=position, source_at=stamp)
        if self.point is None:
            return None
        return dict(self.point, position=list(self.point['position']),
                    age=max(0., now-self.point['source_at']), stale=True)


# 【职责 / R15 R16 R17 R18】attach：注册本模块专用接口与依赖，复用既有应用，不修改原业务实现。
def attach(app, engine, motion):
    nav=motion.navigation
    # R31: running locked segments can detect source changes without querying
    # live people or taking the engine lock under navigation/motion locks.
    nav.person_stream_key=lambda:(engine._config_version,engine._stream_epoch)
    display_cache=DisplayPeopleCache()
    history_cache=LatestPersonPoint()

    # 【职责 / R15 R16 R17 R18】state：区分实时人物、点击锁定点、执行路线和不可执行历史路线。
    @app.get('/api/nav2/people-map')
    def state():
        error=''
        with engine.lock:
            source_key=(engine._config_version,engine._stream_epoch)
        try:people=observed_people(engine,motion)
        except Exception as exc:people=[];error='人物地图暂不可用：'+str(exc)
        with engine.lock:
            current_key=(engine._config_version,engine._stream_epoch)
        if current_key != source_key:people=[]
        with motion.lock,nav.lock:
            selected=motion.settings.target_id
            # R31: display/history refresh does not change the explicitly locked
            # endpoint. Start readiness checks that saved point and route only.
            history_cache.update(people,(*current_key,selected))
            people=display_cache.update(people,(*current_key,selected))
            person=nav.person_navigation
            active=motion.following and motion.mode=='auto' and not motion.estop
            executing=active and nav.enabled and person.state=='executing'
            marker=getattr(nav,'person_target_marker',None)
            if marker and (marker['id']!=selected or tuple(marker['stream_key'])!=current_key):
                nav.person_target_marker=None
                nav.person_locked_target=None
                marker=None
            preview_state=dict(getattr(nav,'preview',{})) if marker and getattr(nav,'person_marker_sequence',None)==getattr(nav,'preview_sequence',None) else None
            if preview_state is not None:
                preview_state['age']=max(0.,time.monotonic()-nav.preview_at)
            history=getattr(person,'last_route',None)
            history_age=time.monotonic()-history['at'] if history else 999.
            history_ok=history and history['target_id']==selected and 0<=history_age<=15.
            sequence=getattr(nav,'preview_sequence',None)
            start_readiness={
                'single':readiness(motion,(),lambda:current_key,sequence,False),
                'continuous':readiness(motion,(),lambda:current_key,sequence,True),
            }
            return {'start_readiness':start_readiness,
                    'continuous_follow':getattr(nav,'continuous_follow',False) and active,
                    'continuous_blocked':getattr(nav,'continuous_blocked',False),
                    'target_marker':marker, 'person_preview':preview_state,
                    'preview_sequence':getattr(nav,'preview_sequence',None),
                    'historical_route':history['points'] if history_ok and not executing else [],
                    'history_age':history_age,
                    'stop_reason':getattr(nav,'last_stop_reason','') if not executing else '',
                    'people':people,'selected_id':selected,'following':active,
                    'state':person.state,'message':error or (getattr(nav,'error','') if active else '') or person.message,
                    'route':list(getattr(person,'route_points',[])) if executing else [],
                    'goal':list(nav.goal[:2]) if executing and nav.goal is not None else None}

    # 【职责 / R15 R16 R17 R18】point_snapshot：取同一 ID/相机流最后有效位置，不设历史点年龄期限。
    def point_snapshot(people, selected, key):
        display_cache.update(people,(*key,selected))
        point=history_cache.update(people,(*key,selected))
        if point is None:
            raise HTTPException(409,'当前所选人物尚无有效历史位置，请先让相机识别并定位该人物')
        return dict(point,stream_key=key)

    # 【职责 / R15 R16 R17 R18】preview_person：强停下记录世界坐标历史点并请求停车规划。
    @app.post('/api/nav2/person-preview')
    def preview_person():
        try:
            # R33: use the last valid position, with no three-second deadline.
            # Preserve its source time. Only this explicit request may plan from
            # history; ordinary follow and continuous next-leg inputs stay fresh.
            with engine.lock:key=(engine._config_version,engine._stream_epoch)
            try:people=observed_people(engine,motion)
            except Exception:people=[]
            with engine.lock:
                if key!=(engine._config_version,engine._stream_epoch):
                    raise HTTPException(409,'相机配置已改变，请重新检测人物')
            with motion.lock,nav.lock:
                if not motion.estop:raise HTTPException(409,'请先强停，再记录人物点并规划')
                if motion.radar_reconfiguring:raise HTTPException(409,'请先完成传感器设置')
                point=point_snapshot(people,motion.settings.target_id,key)
                nav.preview_person_point(point,motion.settings.follow_distance_m)
                return {'target_marker':nav.person_target_marker, 'preview':dict(nav.preview)}
        except HTTPException as exc:
            logger.warning('[Nav2人物请求拒绝] operation=%s status=%s detail=%s',
                           'person-preview',exc.status_code,exc.detail)
            raise

    # 【职责 / R15 R16 R17 R18】start：保留早期开始跟随接口；不以显示缓存代替新鲜目标授权。
    @app.post('/api/nav2/person-start')
    def start():
        # A marker/read request never starts the car; only this explicit POST does.
        with engine.lock:key=(engine._config_version,engine._stream_epoch)
        try:people=observed_people(engine,motion)
        except Exception as exc:raise HTTPException(409,'人物定位不可用：'+str(exc)) from exc
        with engine.lock:
            if key!=(engine._config_version,engine._stream_epoch):
                raise HTTPException(409,'相机配置已改变，请重新检测人物')
        with motion.lock,nav.lock:
            if motion.estop:raise HTTPException(409,'请先解除强制停止，再开始避障跟随')
            if motion.mode!='auto':raise HTTPException(409,'请先切换到自动模式')
            if motion.radar_reconfiguring:raise HTTPException(409,'请先完成传感器设置')
            if nav.fixed_goal_active:raise HTTPException(409,'请先停止当前选点导航')
            if not any(p['id']==motion.settings.target_id and 0<=time.monotonic()-p['source_at']<=1.2 for p in people):
                raise HTTPException(409,'当前所选人物没有新鲜且有效的位置，请核对 ID')
            try:nav.healthy_pose()
            except Exception as exc:raise HTTPException(409,str(exc)) from exc
            if not motion.following:
                motion.set_following(True)
            point=next(p for p in people if p['id']==motion.settings.target_id)
            nav.preview_person_point(dict(point,stream_key=key),motion.settings.follow_distance_m)
            return {'target_marker':nav.person_target_marker, 'preview':dict(nav.preview),
                    'message':'已记录人物历史点并开始规划；跟随执行仍需新鲜人物数据和整车复核',
                    'selected_id':motion.settings.target_id}


    # 【职责 / R15 R16 R17 R18】execute_person：接入同预览路线执行事务，传递连续模式选项。
    @app.post('/api/nav2/person-execute')
    def execute_person(payload: dict):
        try:
            from nav2.person_execution import execute
            # R31: explicit start consumes the locked point, never another live
            # observation. Immutable stream scalars avoid nesting the engine lock.
            return execute(motion,None,
                           lambda:(engine._config_version,engine._stream_epoch),
                           payload.get('sequence'),continuous=payload.get('continuous') is True)
        except HTTPException as exc:
            logger.warning('[Nav2人物请求拒绝] operation=%s status=%s detail=%s',
                           'person-execute',exc.status_code,exc.detail)
            raise

    # 【职责 / R15 R16 R17 R18】end_person：清除连续模式和跟随授权，并执行强停。
    @app.post('/api/nav2/person-end')
    def end_person():
        with motion.lock,nav.lock:
            nav.continuous_follow=False
            nav.continuous_blocked=False
            motion.following=False
            motion.set_estop(True)
            return {'message':'已结束追踪并强停，不再自动生成下一段路线'}
