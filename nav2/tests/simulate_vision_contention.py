"""R37 fixed-behavior contention assertions; no ROS node or chassis is started.

Run from the repository root:
    python3 -m nav2.tests.simulate_vision_contention

Only local fake message/TF/clock/publication objects are supplied. The actual
VisionLayer.tick/_tick_locked and Nav2Follower.velocity/healthy_pose run with
real threading locks and monotonic time. Injected delays are synthetic, not
measurements of ROS publication or this machine's normal workload.
"""
import argparse
import json
import logging
import sys
import threading
import time
from contextlib import contextmanager
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import patch

import numpy as np

from nav2.depth_mailbox import DepthMailbox
from nav2.vision_layer import VisionLayer
from nav2.vision_epoch import VisionEpoch
from nav2_follow import Nav2Follower


def ros_stamp():
    now_ns = time.monotonic_ns()
    return NS(sec=now_ns // 1_000_000_000, nanosec=now_ns % 1_000_000_000)


def fake_grid():
    return NS(header=NS(), info=NS(origin=NS(position=NS(), orientation=NS())))


@contextmanager
def fake_ros_time():
    # Imports are patched only during the probes; no rclpy init or node creation.
    module = ModuleType('rclpy.time')
    module.Time = lambda **kwargs: NS(**kwargs)
    with patch.dict(sys.modules, {'rclpy.time': module}):
        yield


def fixture(publish=None):
    n = Nav2Follower.__new__(Nav2Follower)
    n.lock = threading.RLock()
    n.enabled = n.fixed_goal_active = n.vision_enabled = True
    n.pending = n.canceling = False
    n.handle = n.goal = None
    n.generation = 0
    n.command = (.12, .10)
    n.command_at = n.vision_at = time.monotonic()
    n.vision_error = n.error = ''
    n.camera_pose = (0., 0., 0.)
    n.obstacle_mode = 'map'  # Healthy pose does not require a fake radar feed.
    n.vision_near_blocked = False
    n.vision_safety = None
    clock = NS(now=lambda: NS(nanoseconds=time.monotonic_ns(), to_msg=ros_stamp))
    n.node = NS(get_clock=lambda: clock)
    transform = NS(translation=NS(x=0., y=0., z=0.),
                   rotation=NS(x=0., y=0., z=0., w=1.))
    n.tf = NS(lookup_transform=lambda *args:
              NS(header=NS(stamp=ros_stamp()), transform=transform))
    body = np.array([[-.2, -.2], [.2, -.2], [.2, .2], [-.2, .2]])
    n.navigation_footprint = lambda: (body, time.monotonic())

    layer = VisionLayer.__new__(VisionLayer)
    layer.nav, layer.node = n, n.node
    layer.engine = NS(lock=threading.RLock(), _buffer_m2={},
                      _config_version=1, _stream_epoch=1)
    layer._epoch_gate = VisionEpoch(layer.engine)
    layer.semantic = NS(set_enabled=lambda enabled: None, offer=lambda record: None,
                        snapshot=lambda: None, close=lambda: None)
    layer.last = None
    layer.use_static = False
    layer.height, layer.pitch, layer.scale = .5, 0., 1.
    layer.calibration = {
        'camera_matrix': {'data': [100., 0., 80., 0., 100., 60., 0., 0., 1.]},
        'image_width': 160, 'image_height': 120,
        'distortion_coefficients': {'data': [0., 0., 0., 0., 0.]},
    }
    layer.kind = fake_grid
    published = []

    def publish_message(message):
        if publish is not None:
            publish(message)
        published.append(message)

    layer.publisher = NS(publish=publish_message)
    worker = NS(on_result=lambda record: None)
    layer._depth_mailbox = DepthMailbox(worker)
    source_at = time.monotonic() - .1
    record = NS(config_version=1, started_at=source_at + .02,
                completed_at=source_at + .08,
                depth=np.full((120, 160), 8., dtype=np.float32),
                frame=NS(source_at=source_at, stream_epoch=1, frame_id=1,
                         stamp_ns=int(source_at * 1e9),
                         image=np.zeros((120, 160, 3), dtype=np.uint8)))
    worker.on_result(record)
    return layer, n, record, published


def run_thread(operation):
    started, done = threading.Event(), threading.Event()
    result = {}

    def run():
        result['started_at'] = time.monotonic()
        started.set()
        try:
            result['value'] = operation()
        except BaseException as exc:
            result['error'] = repr(exc)
        finally:
            result['finished_at'] = time.monotonic()
            done.set()

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    if not started.wait(2.):
        raise RuntimeError('Local probe thread failed to start')
    return thread, done, result


def finish(task):
    thread, done, result = task
    if not done.wait(3.):
        raise RuntimeError('Local probe thread did not finish')
    thread.join(timeout=1.)
    if 'error' in result:
        raise RuntimeError(result['error'])
    return result


def seconds(value):
    return round(value, 6)


def baseline():
    layer, n, record, published = fixture()
    try:
        started = time.monotonic()
        layer.tick()
        tick_seconds = time.monotonic() - started
        n.command_at = time.monotonic()
        started = time.monotonic()
        output = n.velocity()
        assert output == (.12, .10) and len(published) == 1
        assert n.vision_at == record.frame.source_at
        return dict(case='baseline', tick_seconds=seconds(tick_seconds),
                    velocity_seconds=seconds(time.monotonic() - started),
                    output=output, published=len(published),
                    source_stamp_preserved=n.vision_at == record.frame.source_at)
    finally:
        layer._depth_mailbox.close()
        layer._epoch_gate.close()


def engine_contention(delay):
    layer, n, record, published = fixture()
    try:
        layer.engine.lock.acquire()
        try:
            tick_task = run_thread(layer.tick)
            already_arrived = layer._depth_mailbox.latest(1, 1) is record
            # R37: the generation gate permits mailbox consumption while the
            # original fusion/configuration lock is held by unrelated work.
            velocity_task = run_thread(n.velocity)
            time.sleep(delay)
            held_until = time.monotonic()
            tick_waited = not tick_task[1].is_set()
            velocity_finished = velocity_task[1].is_set()
        finally:
            layer.engine.lock.release()
        tick_result, velocity_result = finish(tick_task), finish(velocity_task)
        assert already_arrived and not tick_waited, 'Ready mailbox consumption waited for engine.lock'
        assert velocity_finished, 'Engine contention prevented the velocity call from completing'
        assert len(published) == 1 and n.vision_at == record.frame.source_at
        return dict(
            case='engine_lock_contention', injected_hold_seconds=delay,
            observed_hold_after_tick_entry_seconds=seconds(
                held_until - tick_result['started_at']),
            mailbox_ready_before_release=already_arrived,
            tick_blocked_until_engine_release=tick_waited,
            tick_finished_while_engine_locked=not tick_waited,
            tick_seconds=seconds(tick_result['finished_at'] - tick_result['started_at']),
            first_read_after_inference_seconds=layer.timing['first_read_after_inference'],
            mailbox_wait_seconds=layer.timing['mailbox_wait'],
            velocity_finished_while_engine_locked=velocity_finished,
            velocity_seconds=seconds(velocity_result['finished_at'] - velocity_result['started_at']),
            output=velocity_result['value'], published=len(published),
            source_stamp_preserved=n.vision_at == record.frame.source_at)
    finally:
        layer._depth_mailbox.close()
        layer._epoch_gate.close()


def publication_contention(delay):
    entered, release = threading.Event(), threading.Event()
    publish_times = {}

    def slow_publish(message):
        publish_times['started'] = time.monotonic()
        entered.set()
        if not release.wait(3.):
            raise RuntimeError('Local slow-publish release timed out')
        publish_times['finished'] = time.monotonic()

    layer, n, record, published = fixture(slow_publish)
    tick_task = velocity_task = None
    try:
        tick_task = run_thread(layer.tick)
        if not entered.wait(2.):
            raise RuntimeError('Actual visual tick did not reach fake publication')
        velocity_task = run_thread(n.velocity)
        time.sleep(delay)
        was_blocked = not velocity_task[1].is_set()
        release.set()
        tick_result, velocity_result = finish(tick_task), finish(velocity_task)
        assert not was_blocked, 'Slow publication retained the navigation control lock'
        assert velocity_result['value'] == (.12, .10), 'Fresh velocity output was lost during publication'
        assert n.enabled and not n.vision_error and len(published) == 1
        assert n.vision_at == record.frame.source_at
        return dict(
            case='slow_publish_outside_nav_lock', injected_hold_seconds=delay,
            observed_publish_seconds=seconds(publish_times['finished'] - publish_times['started']),
            tick_seconds=seconds(tick_result['finished_at'] - tick_result['started_at']),
            velocity_blocked_until_publish_release=was_blocked,
            velocity_finished_while_publish_blocked=not was_blocked,
            velocity_seconds=seconds(velocity_result['finished_at'] - velocity_result['started_at']),
            nominal_control_period_seconds=.02,
            output=velocity_result['value'],
            command_age_at_velocity_return_seconds=seconds(velocity_result['finished_at'] - n.command_at),
            vision_age_at_velocity_return_seconds=seconds(velocity_result['finished_at'] - n.vision_at),
            enabled_after_release=n.enabled, vision_error=n.vision_error,
            published=len(published),
            source_stamp_preserved=n.vision_at == record.frame.source_at)
    finally:
        release.set()
        for task in (tick_task, velocity_task):
            if task is not None:
                task[0].join(timeout=3.)
        layer._depth_mailbox.close()
        layer._epoch_gate.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--delay', type=float, default=.35,
                        help='Synthetic contention duration in seconds (default: .35)')
    args = parser.parse_args()
    if not .1 <= args.delay <= .8:
        parser.error('--delay must be between .1 and .8 seconds for the overlap assertions')
    # Keep stdout machine-readable. Production diagnostics, if enabled by a
    # caller, use logging/stderr rather than contaminating this JSON document.
    logging.disable(logging.CRITICAL)
    with fake_ros_time():
        result = dict(
            simulation_only=True, robot_or_ros_nodes_started=False,
            fixed_behavior_assertions_passed=True,
            actual_methods=['VisionLayer.tick', 'VisionLayer._tick_locked',
                            'Nav2Follower.velocity', 'Nav2Follower.healthy_pose'],
            synthetic='Engine-lock hold and local fake publisher wait; not hardware timings',
            cases=[baseline(), engine_contention(args.delay), publication_contention(args.delay)],
        )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
