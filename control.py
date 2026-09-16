#!/usr/bin/env python3
"""Saved-image video stream -> person measurements -> /cmd_vel and web controls.

Run with the follow_demo Python after sourcing ROS Humble and the driver workspace.
Coordinates for measurements: camera x right, y down, z forward.  Only the
camera-relative horizontal (x,z) distance is used for the default depth mode.
"""

from __future__ import annotations

import argparse
import math
import os
import queue
import tempfile
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable

import cv2
import numpy as np
import rclpy
import torch
import uvicorn
import yaml
from fastapi import HTTPException
from geometry_msgs.msg import Twist
from pydantic import BaseModel, Field
from ranger_msgs.msg import SystemState
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Image
from ultralytics import YOLO
from ultralytics.utils import YAML

from camera_server import (ROOT, CameraBridge, CameraController, CameraSettings,
                           CaptureSettings, FrameStore, make_app)


CALIBRATION_FILE = ROOT / "camera_calibration.yaml"
CONFIG_DIRECTORY = ROOT / "data" / "param_configs"
SEGMENT_MODEL = ROOT / "yolo26n-seg.pt"
DEPTH_MODEL = ROOT / "yolo26n-depth.pt"
REID_MODEL = ROOT / "yolo26n-reid.onnx"
TRACK_BUFFER = 45  # BoT-SORT lost-track retention, in processed frames.
# Annotation sizes are expressed in OpenCV's drawing units.  The camera image
# is 3072x2048 and is scaled down substantially in the web preview, so labels
# need to be larger than OpenCV's usual defaults to remain readable.
DETECTION_LABEL_FONT_SCALE = 5.0
DETECTION_LABEL_TEXT_THICKNESS = 6
DETECTION_BOX_THICKNESS = 4
DETECTION_LABEL_PADDING_X = 10
DETECTION_LABEL_PADDING_Y = 8


class FollowSettings(BaseModel):
    process_fps: float = Field(default=3.0, gt=0, le=30)
    camera_height_m: float = Field(default=0.50, gt=0, le=2)
    follow_distance_m: float = Field(default=1.0, ge=0.3, le=10)
    target_id: int = Field(default=0, ge=0)
    distance_mode: str = Field(default="depth", pattern="^(depth|footpoint|fused)$")
    detection_confidence: float = Field(default=0.25, gt=0, lt=1)
    detection_iou: float = Field(default=0.70, gt=0, lt=1)
    segment_imgsz: int = Field(default=640, ge=320, le=1280)
    depth_imgsz: int = Field(default=768, ge=320, le=1280)
    kp_distance: float = Field(default=0.85, ge=0, le=10)
    ki_distance: float = Field(default=0.08, ge=0, le=10)
    kd_distance: float = Field(default=0.12, ge=0, le=10)
    kp_bearing: float = Field(default=1.8, ge=0, le=10)
    kd_bearing: float = Field(default=0.12, ge=0, le=10)
    max_forward_mps: float = Field(default=0.8, gt=0, le=2)
    max_reverse_mps: float = Field(default=0.25, ge=0, le=1)
    max_yaw_radps: float = Field(default=1.2, gt=0, le=3)
    max_linear_accel_mps2: float = Field(default=1.4, gt=0, le=5)
    max_yaw_accel_radps2: float = Field(default=2.5, gt=0, le=10)
    distance_deadband_m: float = Field(default=0.08, ge=0, le=1)
    bearing_deadband_deg: float = Field(default=2.0, ge=0, le=30)
    manual_linear_accel_mps2: float = Field(default=0.5, gt=0, le=5)
    manual_yaw_accel_radps2: float = Field(default=1.5, gt=0, le=10)


class Configuration(BaseModel):
    capture: CaptureSettings
    camera: CameraSettings
    follow: FollowSettings


class ManualInput(BaseModel):
    direction: str = Field(pattern="^(up|down|left|right|stop)$")


class Calibration:
    """Calibrated K and plumb_bob coefficients scaled to the actual raw image."""

    def __init__(self, width: int, height: int):
        data = yaml.safe_load(CALIBRATION_FILE.read_text())
        if data["distortion_model"] != "plumb_bob":
            raise ValueError("Only plumb_bob calibration is supported")
        sx = width / data["image_width"]
        sy = height / data["image_height"]
        k = np.array(data["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
        k[0, :] *= sx
        k[1, :] *= sy
        self.k = k
        self.distortion = np.array(data["distortion_coefficients"]["data"], dtype=np.float64)
        self.width, self.height = width, height

    def normalized(self, u: np.ndarray, v: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        pixels = np.stack((u, v), axis=-1).reshape(-1, 1, 2).astype(np.float64)
        rays = cv2.undistortPoints(pixels, self.k, self.distortion).reshape(-1, 2)
        return rays[:, 0], rays[:, 1]


def mask_pixels_in_image(mask: np.ndarray, calibration: Calibration
                         ) -> tuple[np.ndarray, np.ndarray]:
    """Map a letterboxed inference mask to original calibrated image pixels.

    With ``retina_masks=False`` Ultralytics returns masks in the network's
    letterboxed input shape (for example 448x640), not in the 2048x3072 camera
    shape.  Keeping the mask small avoids a multi-megabyte GPU/CPU transfer and
    repeated scans of a full-resolution boolean array.  Only its foreground
    coordinates are mapped back to the calibrated camera image.
    """
    ys, xs = np.nonzero(mask)
    if not xs.size:
        return xs.astype(np.float64), ys.astype(np.float64)
    mask_h, mask_w = mask.shape
    image_w, image_h = calibration.width, calibration.height
    if (mask_w, mask_h) == (image_w, image_h):
        return xs.astype(np.float64), ys.astype(np.float64)

    gain = min(mask_w / image_w, mask_h / image_h)
    pad_x = (mask_w - image_w * gain) / 2.0
    pad_y = (mask_h - image_h * gain) / 2.0
    image_x = (xs.astype(np.float64) - pad_x) / gain
    image_y = (ys.astype(np.float64) - pad_y) / gain
    inside = ((image_x >= 0.0) & (image_x <= image_w - 1.0)
              & (image_y >= 0.0) & (image_y <= image_h - 1.0))
    return image_x[inside], image_y[inside]


def measured_distance(mask: np.ndarray, depth: np.ndarray, calibration: Calibration,
                      height_m: float, mode: str) -> tuple[float | None, str, tuple[float, float], tuple[float, float]]:
    xs, ys = mask_pixels_in_image(mask, calibration)
    if not xs.size:
        raise ValueError("Empty person mask")
    center = float(np.median(xs)), float(np.median(ys))
    bottom = ys >= np.quantile(ys, 0.98)
    foot = float(np.median(xs[bottom])), float(np.max(ys))
    depth_range = None
    if ys.size >= 30:
        low, high = np.quantile(ys, (0.55, 0.985))
        keep = (ys >= low) & (ys <= high)
        ys, xs = ys[keep], xs[keep]
        depth_h, depth_w = depth.shape
        depth_x = np.rint(xs * (depth_w - 1) / max(calibration.width - 1, 1)).astype(np.intp)
        depth_y = np.rint(ys * (depth_h - 1) / max(calibration.height - 1, 1)).astype(np.intp)
        z = depth[depth_y, depth_x].astype(np.float64)
        valid = np.isfinite(z) & (z >= 0.10) & (z <= 30.0)
        if valid.sum() >= 20:
            xray, _ = calibration.normalized(xs[valid], ys[valid])
            z = z[valid]
            # Camera-relative ground-horizontal range; camera height is not
            # added to a learned axial depth map.
            ranges = z * np.hypot(xray, 1.0)
            median = float(np.median(ranges))
            mad = float(np.median(np.abs(ranges - median)))
            if mad > 1e-6:
                ranges = ranges[np.abs(ranges - median) <= 3.5 * 1.4826 * mad]
            depth_range = float(np.median(ranges)) if ranges.size else median
    fxray, fyray = calibration.normalized(np.array([foot[0]]), np.array([foot[1]]))
    foot_range = None
    # A level camera at height h intersects the floor at z=h/y_normalized.
    # The foot ray must point below the horizon and the visible mask must
    # actually extend toward the image floor for this to be useful.
    if fyray[0] > 1e-3 and foot[1] < calibration.height - 2:
        zfloor = height_m / fyray[0]
        candidate = float(zfloor * math.hypot(fxray[0], 1.0))
        if 0.1 <= candidate <= 30.0:
            foot_range = candidate
    if mode == "footpoint":
        return foot_range, "foot-ray", center, foot
    if mode == "fused" and depth_range is not None and foot_range is not None:
        if abs(depth_range - foot_range) / max(depth_range, foot_range) <= 0.30:
            return 0.7 * depth_range + 0.3 * foot_range, "fused", center, foot
    if mode == "fused" and depth_range is None:
        return foot_range, "foot-ray", center, foot
    return depth_range, "mask-depth", center, foot


class Person:
    def __init__(self, public_id: int, confidence: float, box: np.ndarray,
                 mask: np.ndarray, center: tuple[float, float], foot: tuple[float, float],
                 distance: float | None, method: str):
        self.id, self.confidence, self.box, self.mask = public_id, confidence, box, mask
        self.center, self.foot, self.distance, self.method = center, foot, distance, method


@dataclass(frozen=True)
class DrawJob:
    track: object
    depth: np.ndarray
    people: list[Person]
    target_id: int
    linear: float
    yaw: float
    following: bool
    source_at: float


class PublicIds:
    """Never recycle display IDs, even if the tracker resets an internal ID."""

    def __init__(self):
        self.next_id = 0
        self.internal: dict[int, tuple[int, int]] = {}

    def assign(self, internal_id: int, frame_number: int) -> int:
        previous = self.internal.get(internal_id)
        if previous is None or frame_number - previous[1] > TRACK_BUFFER:
            public_id = self.next_id
            self.next_id += 1
        else:
            public_id = previous[0]
        self.internal[internal_id] = public_id, frame_number
        return public_id


class FollowPID:
    """Distance PID and image-bearing PD with output rate limits."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.last_time = None
        self.last_distance_error = 0.0
        self.last_bearing_error = 0.0
        self.integral = 0.0
        self.linear = 0.0
        self.yaw = 0.0

    def update(self, person: Person, calibration: Calibration, settings: FollowSettings,
               now: float) -> tuple[float, float]:
        dt = 1 / 30 if self.last_time is None else max(1 / 120, min(now - self.last_time, 0.25))
        self.last_time = now
        distance_error = person.distance - settings.follow_distance_m
        xray, _ = calibration.normalized(np.array([person.center[0]]), np.array([person.center[1]]))
        bearing_error = math.atan(float(xray[0]))
        if abs(distance_error) < settings.distance_deadband_m:
            distance_error = 0.0
        if abs(bearing_error) < math.radians(settings.bearing_deadband_deg):
            bearing_error = 0.0
        drate = (distance_error - self.last_distance_error) / dt
        brate = (bearing_error - self.last_bearing_error) / dt
        self.integral = max(-0.8, min(0.8, self.integral + distance_error * dt))
        wanted_linear = (settings.kp_distance * distance_error + settings.ki_distance * self.integral
                         + settings.kd_distance * drate)
        wanted_yaw = -(settings.kp_bearing * bearing_error + settings.kd_bearing * brate)
        wanted_linear = max(-settings.max_reverse_mps, min(settings.max_forward_mps, wanted_linear))
        wanted_yaw = max(-settings.max_yaw_radps, min(settings.max_yaw_radps, wanted_yaw))
        self.linear += max(-settings.max_linear_accel_mps2 * dt,
                           min(settings.max_linear_accel_mps2 * dt, wanted_linear - self.linear))
        self.yaw += max(-settings.max_yaw_accel_radps2 * dt,
                        min(settings.max_yaw_accel_radps2 * dt, wanted_yaw - self.yaw))
        self.last_distance_error, self.last_bearing_error = distance_error, bearing_error
        return self.linear, self.yaw


def tracker_yaml() -> Path:
    import ultralytics
    original = Path(ultralytics.__file__).parent / "cfg" / "trackers" / "botsort.yaml"
    config = YAML.load(original)
    config.update({"tracker_type": "botsort", "track_high_thresh": 0.25,
                   "track_low_thresh": 0.10, "new_track_thresh": 0.25,
                   "track_buffer": TRACK_BUFFER, "match_thresh": 0.80,
                   "fuse_score": True, "gmc_method": "sparseOptFlow",
                   "proximity_thresh": 0.50, "appearance_thresh": 0.80,
                   "with_reid": True, "model": str(REID_MODEL)})
    handle = tempfile.NamedTemporaryFile(prefix="visual_car_botsort_", suffix=".yaml", delete=False)
    handle.close()
    path = Path(handle.name)
    YAML.save(path, config)
    return path


def depth_color(depth: np.ndarray, size: tuple[int, int]) -> np.ndarray:
    if depth.shape[::-1] != size:
        depth = cv2.resize(depth, size, interpolation=cv2.INTER_LINEAR)
    valid = np.isfinite(depth) & (depth > 0)
    gray = np.zeros(depth.shape, dtype=np.uint8)
    if valid.any():
        near, far = np.percentile(depth[valid], (2, 98))
        gray[valid] = (255 * (far - np.clip(depth[valid], near, far)) / max(far - near, 1e-6)).astype(np.uint8)
    colored = cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)
    cv2.putText(colored, "YOLO26 metric depth (visualized)", (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
    return colored


def annotate(result, depth: np.ndarray, people: list[Person], target_id: int,
             linear: float, yaw: float, following: bool) -> np.ndarray:
    # Matches test.py: instance-mask image, custom ID/box/foot/range labels,
    # compact control banner, and a dense-depth color panel on the right.
    view = result.plot(boxes=False, masks=True, labels=False)
    for person in people:
        color = (0, 255, 0) if person.id == target_id else (0, 180, 255)
        x1, y1, x2, y2 = (int(v) for v in person.box)
        cv2.circle(view, tuple(round(v) for v in person.foot), 8, color, -1)
        cv2.rectangle(view, (x1, y1), (x2, y2), color, DETECTION_BOX_THICKNESS)
        value = "? m" if person.distance is None else f"{person.distance:.2f} m"
        label = f"ID {person.id} | {person.confidence:.2f} | {value}"
        (text_width, text_height), baseline = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX,
            DETECTION_LABEL_FONT_SCALE, DETECTION_LABEL_TEXT_THICKNESS,
        )
        panel_width = text_width + 2 * DETECTION_LABEL_PADDING_X
        panel_height = text_height + baseline + 2 * DETECTION_LABEL_PADDING_Y
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        panel_left = max(0, min(
            center_x - panel_width // 2,
            max(0, view.shape[1] - panel_width),
        ))
        panel_top = max(0, min(
            center_y - panel_height // 2,
            max(0, view.shape[0] - panel_height),
        ))
        panel_right = min(view.shape[1] - 1, panel_left + panel_width)
        panel_bottom = min(view.shape[0] - 1, panel_top + panel_height)
        cv2.rectangle(
            view, (panel_left, panel_top), (panel_right, panel_bottom),
            (20, 20, 20), -1,
        )
        cv2.putText(
            view, label,
            (panel_left + DETECTION_LABEL_PADDING_X,
             panel_top + DETECTION_LABEL_PADDING_Y + text_height),
            cv2.FONT_HERSHEY_SIMPLEX,
            DETECTION_LABEL_FONT_SCALE, color,
            DETECTION_LABEL_TEXT_THICKNESS, cv2.LINE_AA,
        )
    cv2.rectangle(view, (0, 0), (view.shape[1], 62), (20, 20, 20), -1)
    cv2.putText(view, f"target={target_id} follow={following} v={linear:+.2f}m/s",
                (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2)
    cv2.putText(view, f"yaw={yaw:+.2f}rad/s", (10, 51), cv2.FONT_HERSHEY_SIMPLEX, 0.58,
                (255, 255, 255), 2)
    return np.hstack((view, depth_color(depth, (view.shape[1], view.shape[0]))))


class MotionManager:
    """The sole /cmd_vel publisher; emergency stop wins over all modes."""

    def __init__(self, node, topic: str, settings: FollowSettings):
        self.topic = topic
        self.settings = settings
        self.lock = threading.RLock()
        self.publisher = node.create_publisher(Twist, topic, 10)
        self._battery_subscription = node.create_subscription(
            BatteryState, "/battery_state", self._on_battery_state,
            qos_profile_sensor_data,
        )
        self._system_subscription = node.create_subscription(
            SystemState, "/system_state", self._on_system_state,
            qos_profile_sensor_data,
        )
        self.mode = "auto"
        self.following = False
        self.estop = False
        self.target_visible = False
        self.target_distance = None
        self.auto_linear = 0.0
        self.auto_yaw = 0.0
        self.auto_seen_at = 0.0
        self.manual_direction = "stop"
        self.manual_seen_at = 0.0
        self.manual_linear = 0.0
        self.manual_yaw = 0.0
        self.published_linear = 0.0
        self.published_yaw = 0.0
        self.battery_percentage = None
        self.battery_voltage = None
        self.battery_seen_at = 0.0
        self.system_state_seen_at = 0.0
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _on_battery_state(self, msg: BatteryState):
        """Retain Ranger's battery percentage for the web status endpoint."""
        percentage = float(msg.percentage)
        if not math.isfinite(percentage) or percentage < 0:
            return
        # Ranger copies its uint8 battery_soc directly into this field, so the
        # driver value is already in percentage points (0..100).
        if percentage > 100.0:
            return
        with self.lock:
            self.battery_percentage = percentage
            self.battery_seen_at = time.monotonic()

    def _on_system_state(self, msg: SystemState):
        """Use the Ranger system-state voltage, whose unit is volts."""
        voltage = float(msg.battery_voltage)
        if not math.isfinite(voltage) or voltage <= 0:
            return
        with self.lock:
            self.battery_voltage = voltage
            self.system_state_seen_at = time.monotonic()

    def _publish(self, linear: float, yaw: float):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(yaw)
        self.publisher.publish(msg)
        self.published_linear = linear
        self.published_yaw = yaw

    def stop_now(self):
        with self.lock:
            self.auto_linear = self.auto_yaw = 0.0
            self.manual_linear = self.manual_yaw = 0.0
            self.manual_direction = "stop"
            self.target_visible = False
            self._publish(0.0, 0.0)

    def set_following(self, enabled: bool):
        with self.lock:
            if enabled and self.estop:
                raise HTTPException(status_code=409, detail="解除强制停止后才能开始跟随")
            self.following = enabled
            self.stop_now()
            return self.status()

    def set_mode(self, mode: str):
        if mode not in ("auto", "manual"):
            raise HTTPException(status_code=422, detail="Invalid mode")
        with self.lock:
            self.mode = mode
            self.stop_now()
            return self.status()

    def set_estop(self, enabled: bool):
        with self.lock:
            self.estop = enabled
            self.stop_now()
            return self.status()

    def manual(self, direction: str):
        with self.lock:
            if self.mode != "manual" or self.estop:
                raise HTTPException(status_code=409, detail="手动运行且解除强制停止后才可使用方向键")
            self.manual_direction = direction
            self.manual_seen_at = time.monotonic()
            if direction == "stop":
                self.manual_linear = self.manual_yaw = 0.0
                self._publish(0.0, 0.0)
            return self.status()

    def update_measurement(self, visible: bool, distance: float | None,
                           linear: float, yaw: float, source_at: float):
        with self.lock:
            self.target_visible = visible
            self.target_distance = distance
            self.auto_linear = linear if visible and distance is not None else 0.0
            self.auto_yaw = yaw if visible and distance is not None else 0.0
            self.auto_seen_at = source_at
            if self.mode == "auto" and (not visible or distance is None):
                # A lost or unmeasurable target stops immediately, even while
                # BoT-SORT still holds its latent track for possible recovery.
                self._publish(0.0, 0.0)

    def command_timeout(self) -> float:
        """Allow at most three sample periods, capped for stale-image safety."""
        return min(2.0, max(0.25, 300.0 / self.settings.process_fps))

    def status(self):
        with self.lock:
            now = time.monotonic()
            timeout = self.command_timeout()
            fresh = now - self.auto_seen_at <= timeout
            battery_fresh = now - self.battery_seen_at <= 2.0
            system_state_fresh = now - self.system_state_seen_at <= 2.0
            return {"motion_mode": self.mode, "following": self.following,
                    "force_stopped": self.estop, "target_visible": self.target_visible and fresh,
                    "target_distance_m": self.target_distance if fresh else None,
                    "cmd_vel_topic": self.topic,
                    "command_timeout_seconds": timeout,
                    "battery_percentage": round(self.battery_percentage, 1)
                    if battery_fresh and self.battery_percentage is not None else None,
                    "battery_voltage_v": round(self.battery_voltage, 1)
                    if system_state_fresh and self.battery_voltage is not None else None,
                    "linear_x_mps": round(self.published_linear, 3),
                    "angular_z_radps": round(self.published_yaw, 3),
                    "manual_direction": self.manual_direction}

    def _loop(self):
        last = time.monotonic()
        period = 1.0 / 50.0
        next_tick = last
        # Ranger's reference ROS interface consumes /cmd_vel at 50 Hz.  Keep
        # publishing at that rate, including zero commands while stopped.
        while True:
            next_tick += period
            if self._stop_event.wait(max(0.0, next_tick - time.monotonic())):
                break
            now = time.monotonic()
            if now - next_tick > period:
                # Resynchronize after a scheduler stall instead of trying to
                # emit a burst of stale commands.
                next_tick = now
            dt = min(now - last, 0.25)
            last = now
            with self.lock:
                s = self.settings
                if self.estop:
                    linear = yaw = 0.0
                elif self.mode == "manual":
                    if now - self.manual_seen_at > 0.35:
                        self.manual_direction = "stop"
                        self.manual_linear = self.manual_yaw = 0.0
                    else:
                        d = self.manual_direction
                        if d == "up":
                            self.manual_linear = min(s.max_forward_mps, self.manual_linear + s.manual_linear_accel_mps2 * dt)
                            self.manual_yaw = 0.0
                        elif d == "down":
                            self.manual_linear = max(-s.max_reverse_mps, self.manual_linear - s.manual_linear_accel_mps2 * dt)
                            self.manual_yaw = 0.0
                        elif d == "left":
                            self.manual_yaw = min(s.max_yaw_radps, self.manual_yaw + s.manual_yaw_accel_radps2 * dt)
                            self.manual_linear = 0.0
                        elif d == "right":
                            self.manual_yaw = max(-s.max_yaw_radps, self.manual_yaw - s.manual_yaw_accel_radps2 * dt)
                            self.manual_linear = 0.0
                        else:
                            self.manual_linear = self.manual_yaw = 0.0
                    linear, yaw = self.manual_linear, self.manual_yaw
                elif (self.following and self.target_visible and self.target_distance is not None
                      and now - self.auto_seen_at <= self.command_timeout()):
                    linear, yaw = self.auto_linear, self.auto_yaw
                else:
                    linear = yaw = 0.0
                self._publish(linear, yaw)

    def close(self):
        self._stop_event.set()
        self._thread.join(timeout=2)
        self.stop_now()


class ProcessingEngine:
    """Run control and visualization on independent latest-item pipelines."""

    def __init__(self, settings: FollowSettings, motion: MotionManager,
                 processed_store: FrameStore, persist_images: Callable[[], bool]):
        self.settings = settings
        self.motion = motion
        self.processed_store = processed_store
        self.ids = PublicIds()
        self.pid = FollowPID()
        self.lock = threading.RLock()
        self._pending: queue.Queue[tuple[np.ndarray, float] | None] = queue.Queue(maxsize=1)
        self._draw_pending: queue.Queue[DrawJob | None] = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._last_offered = 0.0
        self._latest: bytes | None = None
        self._last_processed = 0.0
        self._control_latency = None
        self._display_latency = None
        self._frame_number = 0
        self._people: list[Person] = []
        self._persist_images = persist_images
        self.error = ""
        self.drawing_error = ""
        self.dropped_frames = 0
        self.dropped_draw_frames = 0
        self.processed_frames = 0
        self.drawn_frames = 0
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._draw_thread = threading.Thread(target=self._draw_loop, daemon=True)
        self._thread.start()
        self._draw_thread.start()

    def offer(self, image: np.ndarray, source_at: float | None = None):
        """Offer an owned BGR frame; retain at most the newest eligible one."""
        now = time.monotonic() if source_at is None else source_at
        with self.lock:
            interval = 1.0 / self.settings.process_fps
            if now - self._last_offered < interval:
                self.dropped_frames += 1
                return
            self._last_offered = now
        try:
            self._pending.put_nowait((image, now))
        except queue.Full:
            try:
                self._pending.get_nowait()
                self._pending.task_done()
            except queue.Empty:
                pass
            self._pending.put_nowait((image, now))
            with self.lock:
                self.dropped_frames += 1

    def _offer_draw(self, job: DrawJob):
        if self._stop.is_set():
            return
        try:
            self._draw_pending.put_nowait(job)
        except queue.Full:
            try:
                self._draw_pending.get_nowait()
                self._draw_pending.task_done()
            except queue.Empty:
                pass
            self._draw_pending.put_nowait(job)
            with self.lock:
                self.dropped_draw_frames += 1

    def latest(self):
        with self.lock:
            return self._latest if time.monotonic() - self._last_processed <= max(15, 3 / self.settings.process_fps) else None

    def configure(self, settings: FollowSettings):
        with self.lock:
            previous = self.settings
            self.settings = settings
            self.motion.settings = settings
            if settings != previous:
                self.pid.reset()
                self.motion.stop_now()
            return settings

    def status(self):
        with self.lock:
            result = {"follow_settings": self.settings.model_dump(),
                      "processing_error": self.error or self.drawing_error,
                      "processed_total": self.processed_frames,
                      "drawn_total": self.drawn_frames,
                      "skipped_processing_frames": self.dropped_frames,
                      "skipped_drawing_frames": self.dropped_draw_frames,
                      "processed_image_age_seconds": round(time.monotonic() - self._last_processed, 1)
                      if self._last_processed else None,
                      "processing_latency_seconds": round(self._control_latency, 3)
                      if self._control_latency is not None else None,
                      "display_latency_seconds": round(self._display_latency, 3)
                      if self._display_latency is not None else None,
                      "visible_people": [{"id": p.id, "distance_m": round(p.distance, 2) if p.distance is not None else None}
                                         for p in self._people]}
        result.update(self.motion.status())
        return result

    def _loop(self):
        tracker_file = None
        try:
            tracker_file = tracker_yaml()
            device = 0 if torch.cuda.is_available() else "cpu"
            quantize = 16 if torch.cuda.is_available() else None
            segment = YOLO(str(SEGMENT_MODEL))
            depth_model = YOLO(str(DEPTH_MODEL))
            calibration = None
            while not self._stop.is_set():
                try:
                    item = self._pending.get(timeout=0.2)
                except queue.Empty:
                    continue
                if item is None:
                    self._pending.task_done()
                    break
                image, source_at = item
                try:
                    if calibration is None or (calibration.width, calibration.height) != (image.shape[1], image.shape[0]):
                        calibration = Calibration(image.shape[1], image.shape[0])
                    with self.lock:
                        settings = self.settings
                        self._frame_number += 1
                        frame_number = self._frame_number
                    track = segment.track(source=image, tracker=str(tracker_file), persist=True,
                                          classes=[0], conf=settings.detection_confidence,
                                          iou=settings.detection_iou, imgsz=settings.segment_imgsz,
                                          retina_masks=False, device=device, quantize=quantize,
                                          verbose=False)[0]
                    depth_result = depth_model.predict(source=image, imgsz=settings.depth_imgsz,
                                                       device=device, quantize=quantize, verbose=False)[0]
                    if depth_result.depth is None:
                        raise RuntimeError("Depth model returned no depth map")
                    depth = depth_result.depth.data.detach().float().cpu().numpy()
                    people: list[Person] = []
                    if track.boxes is not None and track.masks is not None and track.boxes.id is not None:
                        boxes = track.boxes.xyxy.detach().cpu().numpy()
                        ids = track.boxes.id.detach().cpu().numpy().astype(int)
                        confidences = track.boxes.conf.detach().cpu().numpy()
                        masks = track.masks.data.detach().cpu().numpy() > 0.5
                        for box, internal_id, confidence, mask in zip(boxes, ids, confidences, masks):
                            if not mask.any():
                                continue
                            public_id = self.ids.assign(int(internal_id), frame_number)
                            distance, method, center, foot = measured_distance(
                                mask, depth, calibration, settings.camera_height_m, settings.distance_mode)
                            people.append(Person(public_id, float(confidence), box, mask,
                                                 center, foot, distance, method))
                    target = next((p for p in people if p.id == settings.target_id), None)
                    with self.motion.lock:
                        active = self.motion.following and self.motion.mode == "auto" and not self.motion.estop
                    if target is None or target.distance is None or not active:
                        self.pid.reset()
                        linear = yaw = 0.0
                    else:
                        linear, yaw = self.pid.update(target, calibration, settings, time.monotonic())
                    self.motion.update_measurement(target is not None,
                                                   target.distance if target else None,
                                                   linear, yaw, source_at)
                    control_done = time.monotonic()
                    with self.lock:
                        self._people = people
                        self._control_latency = control_done - source_at
                        self.processed_frames += 1
                        self.error = ""
                    try:
                        # Move the small mask/box tensors off CUDA before the
                        # inference thread starts using the models again.
                        draw_track = track.cpu()
                        self._offer_draw(DrawJob(
                            draw_track, depth, people, settings.target_id,
                            linear if active else 0.0, yaw if active else 0.0,
                            active, source_at,
                        ))
                    except Exception as exc:
                        with self.lock:
                            self.drawing_error = str(exc)
                except Exception as exc:
                    self.motion.stop_now()
                    self.pid.reset()
                    with self.lock:
                        self.error = str(exc)
                finally:
                    self._pending.task_done()
        except Exception as exc:
            self.motion.stop_now()
            with self.lock:
                self.error = str(exc)
        finally:
            if tracker_file is not None:
                tracker_file.unlink(missing_ok=True)

    def _draw_loop(self):
        while not self._stop.is_set():
            try:
                job = self._draw_pending.get(timeout=0.2)
            except queue.Empty:
                continue
            if job is None:
                self._draw_pending.task_done()
                break
            try:
                output = annotate(job.track, job.depth, job.people, job.target_id,
                                  job.linear, job.yaw, job.following)
                ok, buffer = cv2.imencode(".jpg", output, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if not ok:
                    raise ValueError("Processed JPEG encoding failed")
                jpeg = buffer.tobytes()
                self.processed_store.submit(jpeg, self._persist_images())
                drawn_at = time.monotonic()
                with self.lock:
                    self._latest = jpeg
                    self._last_processed = drawn_at
                    self._display_latency = drawn_at - job.source_at
                    self.drawn_frames += 1
                    self.drawing_error = ""
            except Exception as exc:
                # Visualization failure must not stop an otherwise healthy
                # control pipeline.
                with self.lock:
                    self.drawing_error = str(exc)
            finally:
                self._draw_pending.task_done()

    @staticmethod
    def _wake(queue_: queue.Queue):
        try:
            queue_.put_nowait(None)
        except queue.Full:
            try:
                queue_.get_nowait()
                queue_.task_done()
            except queue.Empty:
                pass
            queue_.put_nowait(None)

    def close(self):
        self.motion.set_estop(True)
        self._stop.set()
        self._wake(self._pending)
        self._thread.join(timeout=30)
        self._wake(self._draw_pending)
        self._draw_thread.join(timeout=30)
        self.motion.stop_now()


class ConfigurationStore:
    def __init__(self, bridge: CameraBridge, camera: CameraController, engine: ProcessingEngine):
        self.bridge, self.camera, self.engine = bridge, camera, engine
        CONFIG_DIRECTORY.mkdir(parents=True, exist_ok=True)

    def current(self) -> Configuration:
        return Configuration(capture=self.bridge.settings, camera=self.camera.settings,
                             follow=self.engine.settings)

    def names(self) -> list[str]:
        return sorted((p.name for p in CONFIG_DIRECTORY.glob("*.json") if p.is_file()), reverse=True)

    def save(self, config: Configuration) -> dict:
        name = datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".json"
        path = CONFIG_DIRECTORY / name
        temporary = path.with_suffix(".tmp")
        temporary.write_text(config.model_dump_json(indent=2), encoding="utf-8")
        os.replace(temporary, path)
        return {"name": name, "config": config.model_dump()}

    def load(self, name: str) -> dict:
        if Path(name).name != name or not name.endswith(".json"):
            raise HTTPException(status_code=404, detail="配置文件不存在")
        path = CONFIG_DIRECTORY / name
        if not path.is_file():
            raise HTTPException(status_code=404, detail="配置文件不存在")
        config = Configuration.model_validate_json(path.read_text(encoding="utf-8"))
        # Validate the full file before changing any running parameters.
        # Stop before a camera parameter service call, which may take seconds.
        self.engine.pid.reset()
        self.engine.motion.stop_now()
        if self.camera.state()["camera_process_running"]:
            self.camera.apply_settings(config.camera)
        else:
            self.camera.settings = config.camera
        if self.camera.state()["camera_process_running"]:
            self.bridge.start_capture(config.capture)
        else:
            self.bridge.settings = config.capture
        self.engine.configure(config.follow)
        return {"name": name, "config": self.current().model_dump()}


def build_app(bridge: CameraBridge, raw_store: FrameStore, camera: CameraController,
              engine: ProcessingEngine, processed_store: FrameStore,
              motion: MotionManager):
    app = make_app(bridge, raw_store, camera, engine, processed_store)
    configs = ConfigurationStore(bridge, camera, engine)

    @app.get("/api/config/current")
    def current_config():
        return configs.current().model_dump()

    @app.get("/api/config/files")
    def config_files():
        return configs.names()

    @app.post("/api/config/save")
    def save_config(config: Configuration):
        return configs.save(config)

    @app.post("/api/config/load/{filename}")
    def load_config(filename: str):
        return configs.load(filename)

    @app.post("/api/follow/settings")
    def follow_settings(settings: FollowSettings):
        return engine.configure(settings).model_dump()

    @app.post("/api/follow/start")
    def follow_start():
        engine.pid.reset()
        return motion.set_following(True)

    @app.post("/api/follow/stop")
    def follow_stop():
        engine.pid.reset()
        return motion.set_following(False)

    @app.post("/api/mode/{mode}")
    def set_mode(mode: str):
        engine.pid.reset()
        return motion.set_mode(mode)

    @app.post("/api/manual")
    def manual_input(value: ManualInput):
        return motion.manual(value.direction)

    @app.post("/api/force-stop")
    def force_stop():
        engine.pid.reset()
        return motion.set_estop(True)

    @app.post("/api/force-stop/release")
    def release_force_stop():
        return motion.set_estop(False)

    return app


def migrate_raw_frames():
    raw = ROOT / "data" / "img_raw"
    raw.mkdir(parents=True, exist_ok=True)
    for path in (ROOT / "data").glob("frame_*.jpg"):
        destination = raw / path.name
        if not destination.exists():
            os.replace(path, destination)


def main():
    parser = argparse.ArgumentParser(description="Image-stream person following and web control")
    parser.add_argument("--fps", type=float, default=5.0, help="Camera photo capture frequency")
    parser.add_argument("--process-fps", type=float, default=3.0, help="Maximum inference sampling frequency")
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--preview-fps", type=float, default=10.0)
    parser.add_argument("--max-frames", type=int, default=20)
    parser.add_argument("--image-topic", default="/hk_camera/image_raw")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8890)
    parser.add_argument("--no-launch", action="store_true")
    parser.add_argument("--replay-existing", action="store_true",
                        help="Replay img_raw JPEGs in timestamp order as a simulated photo stream")
    args = parser.parse_args()
    capture = CaptureSettings(fps=args.fps, duration=args.duration)
    follow = FollowSettings(process_fps=args.process_fps)
    if not 0 < args.preview_fps <= 60 or not 1 <= args.max_frames <= 10000 or not 1 <= args.port <= 65535:
        parser.error("Invalid preview-fps, max-frames, or port")

    migrate_raw_frames()
    processed_store = FrameStore(ROOT / "data" / "img_processed", 10)
    raw_store = FrameStore(ROOT / "data" / "img_raw", args.max_frames)
    bridge = CameraBridge(capture, args.preview_fps, raw_store)
    node = camera = motion = engine = executor = spin_thread = replay_thread = None
    try:
        rclpy.init()
        node = rclpy.create_node("visual_car_control")
        node.create_subscription(Image, args.image_topic, bridge.on_image, qos_profile_sensor_data)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        camera = CameraController(node, bridge, args.no_launch)
        motion = MotionManager(node, args.cmd_vel_topic, follow)
        engine = ProcessingEngine(
            follow, motion, processed_store,
            persist_images=lambda: bridge.settings.persist_images,
        )
        bridge.on_frame = engine.offer
        if args.replay_existing:
            paths = sorted(raw_store.directory.glob("frame_*.jpg"))
            def replay():
                for path in paths:
                    if engine._stop.is_set():
                        break
                    image = cv2.imread(str(path))
                    if image is not None:
                        engine.offer(image)
                    time.sleep(1.0 / capture.fps)
            replay_thread = threading.Thread(target=replay, daemon=True)
            replay_thread.start()
        if not args.replay_existing:
            time.sleep(1)  # Allow discovery of a camera launched elsewhere.
            camera.start(initial=True)
        print(f"Control page: http://127.0.0.1:{args.port}; image={args.image_topic}; cmd_vel={args.cmd_vel_topic}", flush=True)
        uvicorn.run(build_app(bridge, raw_store, camera, engine, processed_store, motion),
                    host=args.host, port=args.port, workers=1)
    finally:
        bridge.on_frame = None
        if motion is not None:
            motion.set_estop(True)
        if camera is not None:
            camera.shutdown()
        if engine is not None:
            engine.close()
        if replay_thread is not None:
            replay_thread.join(timeout=2)
        if motion is not None:
            motion.close()
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=5)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        raw_store.close()
        processed_store.close()


if __name__ == "__main__":
    main()
