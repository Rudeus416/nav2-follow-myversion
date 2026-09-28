# 【内容标注 / R32：遇障后保持同一终点绕行】用途：连续人物追踪与原控制暂停入口之间的接入保护。
# 对应用户需求：R18（途中固定本段）、R22（跨模块故障检查）、R31（显式历史人物段）。
# 添加逻辑 / R31：已授权、已整车复核的连续段及显式历史人物段允许人物遮挡；
# 视觉障碍、TF、强停、ID 变更仍按原安全门控停车，不改原 MotionManager。
"""Keep approved continuous/historical segments independent of target visibility.

The original pause hook exempts fixed *point* navigation only while following
is false. Fixed person segments keep following true until their terminal state,
so they need this narrowly scoped instance adapter. No alternate velocity publisher.
"""


def attach(motion):
    """Install once and return a cleanup callback restoring the original hook."""
    existing = getattr(motion, '_nav2_continuous_pause_close', None)
    if existing is not None:
        return existing
    original = motion.handle_nav_measurement_pause
    original_follow = getattr(motion, 'set_following', None)
    if not hasattr(motion, '_nav2_follow_authorization'):
        motion._nav2_follow_authorization = 0

    # R31: record only a successful explicit follow operation, atomically with
    # its original stop/reset. Estop/release also clears preview but is not a
    # new follow session. No control state or original return value is replaced.
    def set_following(enabled):
        with motion.lock:
            result = original_follow(enabled)
            motion._nav2_follow_authorization += 1
            return result

    def pause(now):
        nav = motion.navigation
        # Preserve motion -> nav lock order used by the original control loop.
        with motion.lock, nav.lock:
            person = getattr(nav, 'person_navigation', None)
            task = getattr(person, 'obstacle_replan', None)
            if task is not None and task.holding:
                task.tick()  # Checks original authorization/health; output stays zero.
                return
            locked = getattr(person, 'locked_segment', None)
            if locked and locked.get('finished'):
                return  # Single-route completion is already stopped; cleanup revokes following.
            continuous_segment = ((getattr(nav, 'continuous_follow', False) or locked is not None)
                                  and nav.fixed_goal_active
                                  and person is not None and person.state == 'executing')
            reason = person.locked_segment_error() if locked is not None else ''
            target_id = locked['target_id'] if locked is not None else getattr(nav, 'continuous_target_id', None)
            authorized = (not reason and not motion.estop and motion.mode == 'auto' and motion.following
                          and motion.settings.target_id == target_id)
            if continuous_segment and not authorized:
                # The original hook interprets not-following fixed routes as
                # manually selected point navigation. A revoked person session
                # must not accidentally inherit that exemption.
                nav.stop(reason or '人物路线授权或人物 ID 已改变，停止当前路线')
                return
            approved_segment = (continuous_segment and authorized and nav.enabled
                                and not nav.canceling
                                and not getattr(nav, 'continuous_blocked', False))
            if approved_segment:
                from nav2_follow import VisionStale
                from nav2.replan_support import NearObstacle
                try:
                    nav.healthy_pose()
                except NearObstacle as exc:
                    if task is None or not task.trigger(str(exc)):
                        nav.stop(str(exc))
                except VisionStale:
                    # Match velocity(): stop immediately, retain the action only
                    # within the existing bounded visual freshness grace period.
                    nav.pause_for_vision()
                except Exception as exc:
                    nav.error = str(exc)
                    nav.stop(nav.error)
                return
            return original(now)

    def close():
        with motion.lock:
            if motion.handle_nav_measurement_pause is pause:
                motion.handle_nav_measurement_pause = original
            if callable(original_follow) and motion.set_following is set_following:
                motion.set_following = original_follow
            if getattr(motion, '_nav2_continuous_pause_close', None) is close:
                del motion._nav2_continuous_pause_close

    motion.handle_nav_measurement_pause = pause
    if callable(original_follow):
        motion.set_following = set_following
    motion._nav2_continuous_pause_close = close
    return close
