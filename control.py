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
import uuid
from collections import OrderedDict, deque
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
from pydantic import AliasChoices, BaseModel, Field, model_validator
from ranger_msgs.msg import SystemState
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Image
from ultralytics import YOLO
from ultralytics.trackers.bot_sort import BOTrack
from ultralytics.trackers.utils.kalman_filter import KalmanFilterXYWH
from ultralytics.utils import YAML

from camera_server import (ROOT, CameraBridge, CameraController, CameraSettings,
                           CaptureSettings, FrameStore, make_app)


CALIBRATION_FILE = ROOT / "camera_calibration.yaml"
CONFIG_DIRECTORY = ROOT / "data" / "param_configs"
SEGMENT_MODEL = ROOT / "yolo26n-seg.pt"
DEPTH_MODEL = ROOT / "yolo26n-depth.pt"
REID_MODEL = ROOT / "yolo26n-reid.onnx"
TRACK_BUFFER = 45  # Initial nominal value; runtime retention is normalized to real seconds.
TRACK_RETENTION_SECONDS = 3.0
IDENTITY_REACQUIRE_WINDOW_SECONDS = 3.0
IDENTITY_CONFIRM_FRAMES = 3
IDENTITY_MAX_COSINE_DISTANCE = 0.32
IDENTITY_MIN_SCORE_MARGIN = 0.06
IDENTITY_GALLERY_SIZE = 12
IDENTITY_GALLERY_MIN_CONFIDENCE = 0.35
IDENTITY_NEW_TRACK_GRACE_SECONDS = 0.35
DISPLAY_PANEL_MAX_WIDTH = 1280
# Annotation sizes target the bounded web-preview image, not the 3072x2048
# control frame.
DETECTION_LABEL_FONT_SCALE = 0.7
DETECTION_LABEL_TEXT_THICKNESS = 2
DETECTION_BOX_THICKNESS = 2
DETECTION_LABEL_PADDING_X = 6
DETECTION_LABEL_PADDING_Y = 4


class FollowSettings(BaseModel):
    process_fps: float = Field(default=15.0, gt=0, le=30)
    depth_fps: float = Field(default=5.0, gt=0, le=10)
    camera_height_m: float = Field(default=0.50, gt=0, le=2)
    follow_distance_m: float = Field(default=1.0, ge=0.3, le=10)
    target_id: int = Field(default=0, ge=0)
    persistent_identity_enabled: bool = False
    distance_mode: str = Field(default="depth", pattern="^(depth|footpoint|fused)$")
    # Keep accepting the old saved-config key while exposing the clearer name
    # everywhere in current API responses and newly saved configuration files.
    detector_min_conf: float = Field(
        default=0.10, gt=0, lt=1,
        validation_alias=AliasChoices("detector_min_conf", "detection_confidence"),
    )
    track_low_thresh: float = Field(default=0.10, gt=0, lt=1)
    track_high_thresh: float = Field(default=0.25, gt=0, lt=1)
    new_track_thresh: float = Field(default=0.25, gt=0, lt=1)
    proximity_thresh: float = Field(default=0.50, gt=0, lt=1)
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

    @model_validator(mode="after")
    def validate_tracker_thresholds(self):
        if self.track_low_thresh >= self.track_high_thresh:
            raise ValueError("track_low_thresh must be lower than track_high_thresh")
        return self


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


def mask_geometry(mask: np.ndarray, calibration: Calibration
                  ) -> tuple[np.ndarray, np.ndarray, tuple[float, float], tuple[float, float]]:
    xs, ys = mask_pixels_in_image(mask, calibration)
    if not xs.size:
        raise ValueError("Empty person mask")
    center = float(np.median(xs)), float(np.median(ys))
    bottom = ys >= np.quantile(ys, 0.98)
    foot = float(np.median(xs[bottom])), float(np.max(ys))
    return xs, ys, center, foot


def measured_distance(mask: np.ndarray, depth: np.ndarray, calibration: Calibration,
                      height_m: float, mode: str) -> tuple[float | None, str, tuple[float, float], tuple[float, float]]:
    xs, ys, center, foot = mask_geometry(mask, calibration)
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
                 distance: float | None, method: str, internal_id: int | None = None):
        self.id, self.confidence, self.box, self.mask = public_id, confidence, box, mask
        self.center, self.foot, self.distance, self.method = center, foot, distance, method
        self.internal_id = internal_id


@dataclass(frozen=True)
class IdentityObservation:
    internal_id: int
    confidence: float
    box: np.ndarray
    embedding: np.ndarray | None


@dataclass(frozen=True)
class DrawJob:
    frame_id: int
    track: object
    depth: np.ndarray | None
    people: list[Person]
    target_id: int
    linear: float
    yaw: float
    following: bool
    source_at: float


@dataclass(frozen=True)
class FrameRecord:
    """An immutable camera-buffer entry shared by the inference threads."""

    stream_epoch: str
    frame_id: int
    source_at: float
    image: np.ndarray


@dataclass(frozen=True)
class DepthJob:
    frame: FrameRecord
    config_version: int
    settings: FollowSettings


@dataclass(frozen=True)
class DepthResult:
    frame: FrameRecord
    config_version: int
    settings: FollowSettings
    depth: np.ndarray
    started_at: float
    completed_at: float


@dataclass(frozen=True)
class TrackingResult:
    """Combined detection/tracking output with real camera timing metadata."""

    frame: FrameRecord
    config_version: int
    settings: FollowSettings
    calibration: Calibration
    track: object
    people: tuple[Person, ...]
    previous_frame_id: int | None
    frame_dt: float | None
    kalman_time_scale: float | None
    skipped_camera_frames: int
    depth_requested: bool
    started_at: float
    completed_at: float


@dataclass(frozen=True)
class FusionResult:
    frame_id: int
    config_version: int
    people: tuple[Person, ...]
    linear: float
    yaw: float
    completed_at: float


def retain(buffer: OrderedDict[int, object], key: int, value: object, capacity: int):
    """Insert into a bounded, ordered history without mutating stored records."""
    buffer.pop(key, None)
    buffer[key] = value
    while len(buffer) > capacity:
        buffer.popitem(last=False)


class TimeAwareKalmanFilterXYWH(KalmanFilterXYWH):
    """BoT-SORT XYWH filter whose transition follows the real frame gap.

    Ultralytics stores velocity in units of one nominal tracker update.  The
    transition therefore uses ``actual_dt / nominal_dt`` rather than raw
    seconds.  Process uncertainty grows with the elapsed interval as well.
    """

    def __init__(self):
        super().__init__()
        self.time_scale = 1.0

    def set_time_scale(self, scale: float):
        self.time_scale = max(0.25, min(float(scale), 8.0))
        self._motion_mat[:4, 4:] = np.eye(4) * self.time_scale

    def predict(self, mean: np.ndarray, covariance: np.ndarray):
        uncertainty = math.sqrt(self.time_scale)
        std_pos = np.array([
            self._std_weight_position * mean[2],
            self._std_weight_position * mean[3],
            self._std_weight_position * mean[2],
            self._std_weight_position * mean[3],
        ]) * uncertainty
        std_vel = np.array([
            self._std_weight_velocity * mean[2],
            self._std_weight_velocity * mean[3],
            self._std_weight_velocity * mean[2],
            self._std_weight_velocity * mean[3],
        ]) * uncertainty
        motion_cov = np.diag(np.square(np.r_[std_pos, std_vel]))
        mean = np.dot(mean, self._motion_mat.T)
        covariance = np.linalg.multi_dot(
            (self._motion_mat, covariance, self._motion_mat.T)
        ) + motion_cov
        return mean, covariance

    def multi_predict(self, mean: np.ndarray, covariance: np.ndarray):
        uncertainty = math.sqrt(self.time_scale)
        std_pos = np.stack((
            self._std_weight_position * mean[:, 2],
            self._std_weight_position * mean[:, 3],
            self._std_weight_position * mean[:, 2],
            self._std_weight_position * mean[:, 3],
        ), axis=1) * uncertainty
        std_vel = np.stack((
            self._std_weight_velocity * mean[:, 2],
            self._std_weight_velocity * mean[:, 3],
            self._std_weight_velocity * mean[:, 2],
            self._std_weight_velocity * mean[:, 3],
        ), axis=1) * uncertainty
        sqr = np.square(np.concatenate((std_pos, std_vel), axis=1))
        motion_cov = np.zeros((sqr.shape[0], 8, 8))
        motion_cov[:, range(8), range(8)] = sqr
        mean = np.dot(mean, self._motion_mat.T)
        left = np.dot(self._motion_mat, covariance).transpose((1, 0, 2))
        covariance = np.dot(left, self._motion_mat.T) + motion_cov
        return mean, covariance


def configure_tracker_timing(model: YOLO, frame_dt: float | None,
                             nominal_fps: float) -> float | None:
    """Apply the current camera interval to an already-created BoT-SORT."""
    predictor = getattr(model, "predictor", None)
    trackers = getattr(predictor, "trackers", ())
    if not trackers:
        return None  # The first model.track() call creates the tracker.
    valid_dt = frame_dt is not None and frame_dt > 0
    scale = frame_dt * nominal_fps if valid_dt else 1.0
    for tracker in trackers:
        time_filter = getattr(tracker, "_visual_car_time_filter", None)
        if time_filter is None:
            time_filter = TimeAwareKalmanFilterXYWH()
            tracker._visual_car_time_filter = time_filter
        time_filter.set_time_scale(scale)
        tracker.kalman_filter = time_filter
        # BOTSORT.multi_predict uses BOTrack's shared vectorized filter, while
        # individual tracks retain the activation-time filter reference.
        BOTrack.shared_kalman = time_filter
        for collection in (
            tracker.tracked_stracks, tracker.lost_stracks, tracker.removed_stracks,
        ):
            for track in collection:
                track.kalman_filter = time_filter
        tracker.max_frames_lost = (
            max(1, int(round(TRACK_RETENTION_SECONDS / max(frame_dt, 1e-3))))
            if valid_dt else TRACK_BUFFER
        )
    return max(0.25, min(scale, 8.0))


def configure_tracker_thresholds(model: YOLO, settings: FollowSettings) -> bool:
    """Apply adjustable BoT-SORT association thresholds to live trackers."""
    predictor = getattr(model, "predictor", None)
    trackers = getattr(predictor, "trackers", ())
    if not trackers:
        return False  # The first model.track() call reads the generated YAML.
    for tracker in trackers:
        tracker.args.track_low_thresh = settings.track_low_thresh
        tracker.args.track_high_thresh = settings.track_high_thresh
        tracker.args.new_track_thresh = settings.new_track_thresh
        tracker.args.proximity_thresh = settings.proximity_thresh
        # BOTSORT copies this value out of args during construction, so both
        # locations must be updated for a running persistent tracker.
        if hasattr(tracker, "proximity_thresh"):
            tracker.proximity_thresh = settings.proximity_thresh
    return True


def active_track_embeddings(model: YOLO) -> dict[int, np.ndarray]:
    """Copy normalized BoT-SORT appearance features before its next update."""
    predictor = getattr(model, "predictor", None)
    trackers = getattr(predictor, "trackers", ())
    if not trackers:
        return {}
    features: dict[int, np.ndarray] = {}
    for track in trackers[0].tracked_stracks:
        feature = getattr(track, "smooth_feat", None)
        if track.is_activated and feature is not None:
            features[int(track.track_id)] = np.asarray(feature, dtype=np.float32).copy()
    return features


class PublicIds:
    """Never recycle display IDs, even if the tracker resets an internal ID."""

    def __init__(self):
        self.next_id = 0
        self.internal: dict[int, tuple[int, float]] = {}

    def assign(self, internal_id: int, source_at: float) -> int:
        previous = self.internal.get(internal_id)
        if previous is None or source_at - previous[1] > TRACK_RETENTION_SECONDS:
            public_id = self.next_id
            self.next_id += 1
        else:
            public_id = previous[0]
        self.internal[internal_id] = public_id, source_at
        return public_id


class IdentityRecord:
    """Session-local logical identity and its trusted appearance history."""

    def __init__(self, public_id: int, source_at: float):
        self.public_id = public_id
        self.first_seen_at = source_at
        self.last_seen_at = 0.0
        self.current_internal_id: int | None = None
        self.last_box: np.ndarray | None = None
        self.last_center: np.ndarray | None = None
        self.velocity = np.zeros(2, dtype=np.float32)
        self.gallery: deque[np.ndarray] = deque(maxlen=IDENTITY_GALLERY_SIZE)


class PersistentIdentityManager:
    """Map ephemeral BoT-SORT IDs to logical IDs and safely reacquire the selected target.

    Ordinary people keep the original monotonically allocated display-ID
    behavior. Only the operator-selected target is eligible for cross-ID
    reacquisition, and it must pass appearance/geometric gates for several
    consecutive tracker updates before its logical ID is restored.
    """

    def __init__(self, settings: FollowSettings):
        self.lock = threading.RLock()
        self.ids = PublicIds()
        self.records: dict[int, IdentityRecord] = {}
        self.internal_first_seen: dict[int, float] = {}
        self.enabled = settings.persistent_identity_enabled
        self.target_id = settings.target_id
        self.state = "waiting_target" if self.enabled else "disabled"
        self.lost_at: float | None = None
        self.candidate_internal_id: int | None = None
        self.candidate_confirmations = 0
        self.candidate_distance: float | None = None
        self.reacquisitions = 0

    def _reset_candidate(self):
        self.candidate_internal_id = None
        self.candidate_confirmations = 0
        self.candidate_distance = None

    def configure(self, enabled: bool, target_id: int):
        with self.lock:
            if enabled == self.enabled and target_id == self.target_id:
                return
            self.enabled = enabled
            self.target_id = target_id
            self.lost_at = None
            self._reset_candidate()
            self.state = "waiting_target" if enabled else "disabled"

    def _ensure_record(self, public_id: int, source_at: float) -> IdentityRecord:
        record = self.records.get(public_id)
        if record is None:
            record = IdentityRecord(public_id, source_at)
            self.records[public_id] = record
        return record

    def _baseline_assignment(self, internal_id: int, source_at: float) -> int:
        previous = self.ids.internal.get(internal_id)
        if previous is None or source_at - previous[1] > TRACK_RETENTION_SECONDS:
            self.internal_first_seen[internal_id] = source_at
        public_id = self.ids.assign(internal_id, source_at)
        self._ensure_record(public_id, source_at)
        return public_id

    def _force_new_public_id(self, internal_id: int, source_at: float) -> int:
        public_id = self.ids.next_id
        self.ids.next_id += 1
        self.ids.internal[internal_id] = public_id, source_at
        self.internal_first_seen[internal_id] = source_at
        self._ensure_record(public_id, source_at)
        return public_id

    @staticmethod
    def _normalized_embedding(embedding: np.ndarray | None) -> np.ndarray | None:
        if embedding is None:
            return None
        feature = np.asarray(embedding, dtype=np.float32).reshape(-1)
        norm = float(np.linalg.norm(feature))
        return feature / norm if norm > 1e-12 else None

    @staticmethod
    def _box_center_area(box: np.ndarray) -> tuple[np.ndarray, float]:
        x1, y1, x2, y2 = (float(value) for value in box[:4])
        center = np.array(((x1 + x2) * 0.5, (y1 + y2) * 0.5), dtype=np.float32)
        return center, max(1.0, (x2 - x1) * (y2 - y1))

    @staticmethod
    def _appearance_distance(record: IdentityRecord, embedding: np.ndarray) -> float | None:
        if not record.gallery:
            return None
        distances = [
            max(0.0, 1.0 - float(np.dot(feature, embedding)))
            for feature in record.gallery if feature.shape == embedding.shape
        ]
        return min(distances) if distances else None

    def _candidate_score(self, target: IdentityRecord, observation: IdentityObservation,
                         source_at: float, image_size: tuple[int, int]
                         ) -> tuple[float, float] | None:
        embedding = self._normalized_embedding(observation.embedding)
        if embedding is None:
            return None
        appearance = self._appearance_distance(target, embedding)
        if appearance is None or appearance > IDENTITY_MAX_COSINE_DISTANCE:
            return None

        center, area = self._box_center_area(observation.box)
        elapsed = max(0.0, source_at - target.last_seen_at)
        center_penalty = 0.0
        if target.last_center is not None:
            predicted = target.last_center + target.velocity * elapsed
            diagonal = max(1.0, math.hypot(*image_size))
            center_penalty = float(np.linalg.norm(center - predicted) / diagonal)
            allowed = min(0.75, 0.35 + 0.15 * elapsed)
            if center_penalty > allowed:
                return None

        scale_penalty = 0.0
        if target.last_box is not None:
            _, previous_area = self._box_center_area(target.last_box)
            ratio = area / previous_area
            if not 0.20 <= ratio <= 5.0:
                return None
            scale_penalty = abs(math.log(ratio))
        score = appearance + 0.18 * center_penalty + 0.04 * scale_penalty
        return score, appearance

    def _bind_target(self, internal_id: int, target_id: int, source_at: float,
                     assignments: dict[int, int]):
        temporary_id = assignments[internal_id]
        # A logical identity may have only one active underlying tracker ID.
        for previous_internal, (public_id, _) in list(self.ids.internal.items()):
            if public_id == target_id:
                self.ids.internal.pop(previous_internal, None)
        self.ids.internal[internal_id] = target_id, source_at
        assignments[internal_id] = target_id
        temporary = self.records.get(temporary_id)
        if temporary is not None and temporary_id != target_id:
            temporary.current_internal_id = None

    def _update_record(self, record: IdentityRecord, observation: IdentityObservation,
                       source_at: float, image_size: tuple[int, int]):
        center, _ = self._box_center_area(observation.box)
        if record.last_center is not None and record.last_seen_at > 0:
            dt = source_at - record.last_seen_at
            if dt > 1e-3:
                instantaneous = (center - record.last_center) / dt
                record.velocity = 0.7 * record.velocity + 0.3 * instantaneous
        record.current_internal_id = observation.internal_id
        record.last_seen_at = source_at
        record.last_center = center
        record.last_box = np.asarray(observation.box, dtype=np.float32).copy()

        embedding = self._normalized_embedding(observation.embedding)
        if embedding is None or observation.confidence < IDENTITY_GALLERY_MIN_CONFIDENCE:
            return
        width, height = image_size
        x1, y1, x2, y2 = (float(value) for value in observation.box[:4])
        if y2 - y1 < 64 or x1 <= 2 or y1 <= 2 or x2 >= width - 2 or y2 >= height - 2:
            return
        comparable = [feature for feature in record.gallery
                      if feature.shape == embedding.shape]
        if not comparable:
            record.gallery.append(embedding.copy())
            return
        nearest = min(
            max(0.0, 1.0 - float(np.dot(feature, embedding)))
            for feature in comparable
        )
        # Near-duplicates add no information; large jumps are more likely an
        # underlying tracker identity switch and must never poison the gallery.
        if 0.02 <= nearest <= 0.25:
            record.gallery.append(embedding.copy())

    def resolve(self, observations: tuple[IdentityObservation, ...], settings: FollowSettings,
                source_at: float, image_size: tuple[int, int]) -> dict[int, int]:
        with self.lock:
            self.configure(settings.persistent_identity_enabled, settings.target_id)
            assignments = {
                observation.internal_id: self._baseline_assignment(
                    observation.internal_id, source_at,
                )
                for observation in observations
            }
            by_internal = {observation.internal_id: observation for observation in observations}
            direct = [internal_id for internal_id, public_id in assignments.items()
                      if public_id == self.target_id]

            # Defensive cleanup for any stale duplicate mapping to the target.
            if len(direct) > 1:
                record = self.records.get(self.target_id)
                preferred = (record.current_internal_id
                             if record is not None and record.current_internal_id in direct
                             else max(direct, key=lambda value: by_internal[value].confidence))
                for internal_id in direct:
                    if internal_id != preferred:
                        assignments[internal_id] = self._force_new_public_id(internal_id, source_at)
                direct = [preferred]

            if not self.enabled:
                self.state = "disabled"
                self.lost_at = None
                self._reset_candidate()
            elif direct:
                self.state = "tracked"
                self.lost_at = None
                self._reset_candidate()
            else:
                target = self.records.get(self.target_id)
                if target is None:
                    self.state = "waiting_target"
                    self.lost_at = None
                    self._reset_candidate()
                elif not target.gallery:
                    self.state = "lost_no_gallery"
                    self.lost_at = self.lost_at or source_at
                    self._reset_candidate()
                else:
                    if self.lost_at is None:
                        self.lost_at = source_at
                    lost_for = source_at - self.lost_at
                    if lost_for > IDENTITY_REACQUIRE_WINDOW_SECONDS:
                        self.state = "expired"
                        self._reset_candidate()
                    else:
                        ranked: list[tuple[float, float, IdentityObservation]] = []
                        for observation in observations:
                            first_seen = self.internal_first_seen.get(
                                observation.internal_id, source_at,
                            )
                            # Never steal a bystander identity that was already
                            # established before the selected target disappeared.
                            if first_seen < self.lost_at - IDENTITY_NEW_TRACK_GRACE_SECONDS:
                                continue
                            candidate = self._candidate_score(
                                target, observation, source_at, image_size,
                            )
                            if candidate is not None:
                                ranked.append((candidate[0], candidate[1], observation))
                        ranked.sort(key=lambda value: value[0])
                        if (not ranked or (len(ranked) > 1
                                           and ranked[1][0] - ranked[0][0]
                                           < IDENTITY_MIN_SCORE_MARGIN)):
                            self.state = "ambiguous" if ranked else "lost"
                            self._reset_candidate()
                        else:
                            _, appearance, best = ranked[0]
                            if best.internal_id == self.candidate_internal_id:
                                self.candidate_confirmations += 1
                            else:
                                self.candidate_internal_id = best.internal_id
                                self.candidate_confirmations = 1
                            self.candidate_distance = appearance
                            self.state = "verifying"
                            if self.candidate_confirmations >= IDENTITY_CONFIRM_FRAMES:
                                self._bind_target(
                                    best.internal_id, self.target_id, source_at, assignments,
                                )
                                self.reacquisitions += 1
                                self.state = "reacquired"
                                self.lost_at = None
                                self._reset_candidate()

            for observation in observations:
                public_id = assignments[observation.internal_id]
                self._update_record(
                    self._ensure_record(public_id, source_at), observation,
                    source_at, image_size,
                )
            return assignments

    def status(self) -> dict:
        with self.lock:
            target = self.records.get(self.target_id)
            return {
                "persistent_identity_enabled": self.enabled,
                "identity_state": self.state,
                "identity_target_internal_id": target.current_internal_id if target else None,
                "identity_gallery_size": len(target.gallery) if target else 0,
                "identity_candidate_internal_id": self.candidate_internal_id,
                "identity_candidate_confirmations": self.candidate_confirmations,
                "identity_candidate_distance": (
                    round(self.candidate_distance, 4)
                    if self.candidate_distance is not None else None
                ),
                "identity_reacquisitions_total": self.reacquisitions,
            }


class FollowPID:
    """Distance PID and image-bearing PD with output rate limits."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.last_distance_time = None
        self.last_bearing_time = None
        self.last_distance_error = 0.0
        self.last_bearing_error = 0.0
        self.integral = 0.0
        self.linear = 0.0
        self.yaw = 0.0

    def reset_distance(self):
        self.last_distance_time = None
        self.last_distance_error = 0.0
        self.integral = 0.0
        self.linear = 0.0

    def reset_bearing(self):
        self.last_bearing_time = None
        self.last_bearing_error = 0.0
        self.yaw = 0.0

    def update_distance(self, person: Person, settings: FollowSettings, now: float) -> float:
        dt = (1 / 30 if self.last_distance_time is None
              else max(1 / 120, now - self.last_distance_time))
        self.last_distance_time = now
        distance_error = person.distance - settings.follow_distance_m
        if abs(distance_error) < settings.distance_deadband_m:
            distance_error = 0.0
        drate = (distance_error - self.last_distance_error) / dt
        # The derivative uses real elapsed time. Integral and slew increments
        # are capped independently so a long inference stall cannot command a
        # sudden jump when processing resumes.
        safe_dt = min(dt, 0.25)
        self.integral = max(-0.8, min(
            0.8, self.integral + distance_error * safe_dt,
        ))
        wanted = (settings.kp_distance * distance_error
                  + settings.ki_distance * self.integral
                  + settings.kd_distance * drate)
        wanted = max(-settings.max_reverse_mps, min(settings.max_forward_mps, wanted))
        self.linear += max(
            -settings.max_linear_accel_mps2 * safe_dt,
            min(settings.max_linear_accel_mps2 * safe_dt, wanted - self.linear),
        )
        self.last_distance_error = distance_error
        return self.linear

    def update_bearing(self, person: Person, calibration: Calibration,
                       settings: FollowSettings, now: float) -> float:
        dt = (1 / 30 if self.last_bearing_time is None
              else max(1 / 120, now - self.last_bearing_time))
        self.last_bearing_time = now
        xray, _ = calibration.normalized(
            np.array([person.center[0]]), np.array([person.center[1]])
        )
        bearing_error = math.atan(float(xray[0]))
        if abs(bearing_error) < math.radians(settings.bearing_deadband_deg):
            bearing_error = 0.0
        brate = (bearing_error - self.last_bearing_error) / dt
        wanted = -(settings.kp_bearing * bearing_error + settings.kd_bearing * brate)
        wanted = max(-settings.max_yaw_radps, min(settings.max_yaw_radps, wanted))
        safe_dt = min(dt, 0.25)
        self.yaw += max(
            -settings.max_yaw_accel_radps2 * safe_dt,
            min(settings.max_yaw_accel_radps2 * safe_dt, wanted - self.yaw),
        )
        self.last_bearing_error = bearing_error
        return self.yaw

    def update(self, person: Person, calibration: Calibration, settings: FollowSettings,
               now: float) -> tuple[float, float]:
        return (self.update_distance(person, settings, now),
                self.update_bearing(person, calibration, settings, now))


def tracker_yaml(settings: FollowSettings) -> Path:
    import ultralytics
    original = Path(ultralytics.__file__).parent / "cfg" / "trackers" / "botsort.yaml"
    config = YAML.load(original)
    config.update({"tracker_type": "botsort",
                   "track_high_thresh": settings.track_high_thresh,
                   "track_low_thresh": settings.track_low_thresh,
                   "new_track_thresh": settings.new_track_thresh,
                   "track_buffer": TRACK_BUFFER, "match_thresh": 0.80,
                   "fuse_score": True, "gmc_method": "sparseOptFlow",
                   "proximity_thresh": settings.proximity_thresh,
                   "appearance_thresh": 0.80,
                   "with_reid": True, "model": str(REID_MODEL)})
    handle = tempfile.NamedTemporaryFile(prefix="visual_car_botsort_", suffix=".yaml", delete=False)
    handle.close()
    path = Path(handle.name)
    YAML.save(path, config)
    return path


class DepthWorker:
    """Pipeline 3: infer depth for the newest tracking-selected frame."""

    def __init__(self, device: int | str, quantize: int | None,
                 on_result: Callable[[DepthResult], None]):
        self.device = device
        self.quantize = quantize
        self.on_result = on_result
        self.lock = threading.RLock()
        # The mailbox may retain a few requests, but the worker drains it to
        # the newest request before each inference. Old depth is not useful to
        # a real-time controller.
        self._pending: queue.Queue[DepthJob | None] = queue.Queue(maxsize=4)
        self._stop = threading.Event()
        self._last_offered_source_at = 0.0
        self._last_completed_at = 0.0
        self._latest_source_at = 0.0
        self._latest_frame_id: int | None = None
        self.submitted = 0
        self.processed = 0
        self.dropped = 0
        self.effective_fps = 0.0
        self.latency: float | None = None
        self.error = ""
        self._thread = threading.Thread(target=self._loop, daemon=True, name="depth-worker")
        self._thread.start()

    def offer(self, job: DepthJob) -> bool:
        """Publish buffer_n before tracking inference, without blocking it."""
        with self.lock:
            if job.frame.source_at - self._last_offered_source_at < 1.0 / job.settings.depth_fps:
                return False
            self._last_offered_source_at = job.frame.source_at
            self.submitted += 1
        try:
            self._pending.put_nowait(job)
        except queue.Full:
            try:
                self._pending.get_nowait()
                self._pending.task_done()
            except queue.Empty:
                pass
            self._pending.put_nowait(job)
            with self.lock:
                self.dropped += 1
        return True

    def reconfigure(self):
        """Discard queued work from older configuration generations."""
        with self.lock:
            self._last_offered_source_at = 0.0
        while True:
            try:
                item = self._pending.get_nowait()
            except queue.Empty:
                break
            self._pending.task_done()
            if item is not None:
                with self.lock:
                    self.dropped += 1

    def status(self) -> dict:
        with self.lock:
            age = time.monotonic() - self._latest_source_at if self._latest_source_at else None
            return {
                "depth_submitted_total": self.submitted,
                "depth_processed_total": self.processed,
                "depth_dropped_total": self.dropped,
                "depth_effective_fps": round(self.effective_fps, 1),
                "depth_latency_seconds": round(self.latency, 3) if self.latency is not None else None,
                "depth_age_seconds": round(age, 3) if age is not None else None,
                "latest_depth_frame_id": self._latest_frame_id,
                "depth_queue_size": self._pending.qsize(),
                "depth_error": self.error,
            }

    def _newest_job(self) -> DepthJob | None:
        try:
            job = self._pending.get(timeout=0.2)
        except queue.Empty:
            return None
        if job is None:
            self._pending.task_done()
            return None
        while True:
            try:
                newer = self._pending.get_nowait()
            except queue.Empty:
                break
            self._pending.task_done()
            if newer is None:
                self._pending.task_done()
                return None
            job = newer
            with self.lock:
                self.dropped += 1
        return job

    def _loop(self):
        try:
            model = YOLO(str(DEPTH_MODEL))
            while not self._stop.is_set():
                job = self._newest_job()
                if job is None:
                    continue
                started_at = time.monotonic()
                try:
                    result = model.predict(source=job.frame.image, imgsz=job.settings.depth_imgsz,
                                           device=self.device, quantize=self.quantize,
                                           verbose=False)[0]
                    if result.depth is None:
                        raise RuntimeError("Depth model returned no depth map")
                    depth = result.depth.data.detach().float().cpu().numpy()
                    completed_at = time.monotonic()
                    output = DepthResult(job.frame, job.config_version, job.settings,
                                         depth, started_at, completed_at)
                    with self.lock:
                        if self._last_completed_at > 0:
                            instant_fps = 1.0 / max(completed_at - self._last_completed_at, 1e-6)
                            self.effective_fps = (instant_fps if self.effective_fps == 0
                                                  else 0.8 * self.effective_fps + 0.2 * instant_fps)
                        self._last_completed_at = completed_at
                        self._latest_source_at = job.frame.source_at
                        self._latest_frame_id = job.frame.frame_id
                        self.latency = completed_at - job.frame.source_at
                        self.processed += 1
                        self.error = ""
                    self.on_result(output)
                except Exception as exc:
                    with self.lock:
                        self.error = str(exc)
                finally:
                    self._pending.task_done()
        except Exception as exc:
            with self.lock:
                self.error = str(exc)

    def close(self):
        self._stop.set()
        try:
            self._pending.put_nowait(None)
        except queue.Full:
            try:
                self._pending.get_nowait()
                self._pending.task_done()
            except queue.Empty:
                pass
            self._pending.put_nowait(None)
        self._thread.join(timeout=30)


def depth_color(depth: np.ndarray | None, size: tuple[int, int]) -> np.ndarray:
    if depth is None:
        colored = np.zeros((size[1], size[0], 3), dtype=np.uint8)
        cv2.putText(colored, "Waiting for depth...", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
        return colored
    valid = np.isfinite(depth) & (depth > 0)
    gray = np.zeros(depth.shape, dtype=np.uint8)
    if valid.any():
        near, far = np.percentile(depth[valid], (2, 98))
        gray[valid] = (255 * (far - np.clip(depth[valid], near, far)) / max(far - near, 1e-6)).astype(np.uint8)
    colored = cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)
    if colored.shape[1::-1] != size:
        colored = cv2.resize(colored, size, interpolation=cv2.INTER_LINEAR)
    cv2.putText(colored, "YOLO26 metric depth (visualized)", (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
    return colored


def preview_mask(mask: np.ndarray, image_size: tuple[int, int],
                 preview_size: tuple[int, int]) -> np.ndarray:
    """Remove inference letterbox padding and resize a mask for display only."""
    image_w, image_h = image_size
    mask_h, mask_w = mask.shape
    if (mask_w, mask_h) == (image_w, image_h):
        content = mask
    else:
        gain = min(mask_w / image_w, mask_h / image_h)
        content_w = image_w * gain
        content_h = image_h * gain
        left = max(0, int(round((mask_w - content_w) / 2.0)))
        top = max(0, int(round((mask_h - content_h) / 2.0)))
        right = min(mask_w, max(left + 1, int(round(left + content_w))))
        bottom = min(mask_h, max(top + 1, int(round(top + content_h))))
        content = mask[top:bottom, left:right]
    return cv2.resize(
        content.astype(np.uint8), preview_size, interpolation=cv2.INTER_NEAREST,
    ).astype(bool)


def annotate(result, depth: np.ndarray | None, people: list[Person], target_id: int,
             linear: float, yaw: float, following: bool, frame_id: int | None = None) -> np.ndarray:
    # Rendering is intentionally performed at web-preview resolution. The
    # controller continues to use original calibration coordinates and the
    # native depth/mask tensors; only pipeline 6 is downscaled.
    original = result.orig_img
    image_h, image_w = original.shape[:2]
    scale = min(1.0, DISPLAY_PANEL_MAX_WIDTH / image_w)
    preview_size = max(1, round(image_w * scale)), max(1, round(image_h * scale))
    view = cv2.resize(original, preview_size, interpolation=cv2.INTER_AREA)
    for person in people:
        color = (0, 255, 0) if person.id == target_id else (0, 180, 255)
        foreground = preview_mask(person.mask, (image_w, image_h), preview_size)
        if foreground.any():
            pixels = view[foreground].astype(np.float32)
            view[foreground] = (0.58 * pixels + 0.42 * np.asarray(color)).astype(np.uint8)
        x1, y1, x2, y2 = (round(float(v) * scale) for v in person.box)
        foot = tuple(round(float(v) * scale) for v in person.foot)
        cv2.circle(view, foot, 5, color, -1)
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
    cv2.rectangle(view, (0, 0), (view.shape[1], 58), (20, 20, 20), -1)
    frame_text = "" if frame_id is None else f" frame={frame_id}"
    cv2.putText(view, f"target={target_id} follow={following} v={linear:+.2f}m/s{frame_text}",
                (10, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2)
    cv2.putText(view, f"yaw={yaw:+.2f}rad/s", (10, 49), cv2.FONT_HERSHEY_SIMPLEX, 0.58,
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
        # Fail-safe default: no non-zero command is possible until an operator
        # explicitly releases the latch in the web UI.
        self.estop = True
        self.target_visible = False
        self.target_distance = None
        self.auto_linear = 0.0
        self.auto_yaw = 0.0
        self.auto_seen_at = 0.0
        self.tracking_seen_at = 0.0
        self.distance_seen_at = 0.0
        self.auto_frame_id = 0
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
            self.target_distance = None
            self.auto_seen_at = 0.0
            self.tracking_seen_at = 0.0
            self.distance_seen_at = 0.0
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

    def update_tracking(self, visible: bool, yaw: float, source_at: float,
                        frame_id: int) -> bool:
        """Accept only the newest tracking observation and update yaw early."""
        with self.lock:
            if frame_id < self.auto_frame_id or source_at < self.auto_seen_at:
                return False
            self.auto_frame_id = frame_id
            self.auto_seen_at = source_at
            self.tracking_seen_at = source_at
            self.target_visible = visible
            self.auto_yaw = yaw if visible else 0.0
            if not visible:
                self.target_distance = None
                self.auto_linear = 0.0
                self.distance_seen_at = 0.0
                if self.mode == "auto":
                    self._publish(0.0, 0.0)
            return True

    def update_measurement(self, visible: bool, distance: float | None,
                           linear: float, yaw: float, source_at: float,
                           frame_id: int) -> bool:
        with self.lock:
            if frame_id < self.auto_frame_id or source_at < self.auto_seen_at:
                return False
            self.auto_frame_id = frame_id
            self.auto_seen_at = source_at
            self.tracking_seen_at = max(self.tracking_seen_at, source_at)
            self.target_visible = visible
            self.target_distance = distance
            self.auto_linear = linear if visible and distance is not None else 0.0
            self.auto_yaw = yaw if visible and distance is not None else 0.0
            self.distance_seen_at = source_at if visible and distance is not None else 0.0
            if self.mode == "auto" and (not visible or distance is None):
                # A lost or unmeasurable target stops immediately, even while
                # BoT-SORT still holds its latent track for possible recovery.
                self._publish(0.0, 0.0)
            return True

    def command_timeout(self) -> float:
        """Allow at most three depth/control periods, capped for safety."""
        return min(1.0, max(0.25, 3.0 / self.settings.depth_fps))

    def status(self):
        with self.lock:
            now = time.monotonic()
            timeout = self.command_timeout()
            tracking_fresh = now - self.tracking_seen_at <= timeout
            distance_fresh = now - self.distance_seen_at <= timeout
            battery_fresh = now - self.battery_seen_at <= 2.0
            system_state_fresh = now - self.system_state_seen_at <= 2.0
            return {"motion_mode": self.mode, "following": self.following,
                    "force_stopped": self.estop,
                    "target_visible": self.target_visible and tracking_fresh,
                    "target_distance_m": self.target_distance if distance_fresh else None,
                    "latest_control_frame_id": self.auto_frame_id or None,
                    "tracking_measurement_age_seconds": round(now - self.tracking_seen_at, 3)
                    if self.tracking_seen_at else None,
                    "distance_measurement_age_seconds": round(now - self.distance_seen_at, 3)
                    if self.distance_seen_at else None,
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
                      and now - self.tracking_seen_at <= self.command_timeout()
                      and now - self.distance_seen_at <= self.command_timeout()):
                    linear, yaw = self.auto_linear, self.auto_yaw
                else:
                    linear = yaw = 0.0
                self._publish(linear, yaw)

    def close(self):
        self._stop_event.set()
        self._thread.join(timeout=2)
        self.stop_now()


class ProcessingEngine:
    """Six logical pipelines with combined detection/tracking execution.

    Pipeline 1 stores camera frames in ``buffer_c``. Pipelines 2 and 4 share
    one worker because Ultralytics wraps detection and BoT-SORT in
    ``model.track``. Before that call begins it publishes ``buffer_n`` so
    pipeline 3 can infer depth concurrently. Pipeline 5 joins exact frame IDs
    without blocking either producer. Pipeline 6 draws only matched frames;
    FastAPI configuration handlers remain independent of the drawing worker.
    """

    CAMERA_BUFFER_CAPACITY = 8
    RESULT_BUFFER_CAPACITY = 4
    JOIN_QUEUE_CAPACITY = 128

    def __init__(self, settings: FollowSettings, motion: MotionManager,
                 processed_store: FrameStore, persist_images: Callable[[], bool]):
        self.settings = settings
        self.motion = motion
        self.processed_store = processed_store
        self.identity = PersistentIdentityManager(settings)
        self.pid = FollowPID()
        self.lock = threading.RLock()
        self._camera_condition = threading.Condition(self.lock)
        self._stop = threading.Event()
        self._stream_epoch = uuid.uuid4().hex
        self._config_version = 1

        # Multi-record buffers. Records are immutable and may safely be
        # referenced after their history entry is evicted.
        self._buffer_c: OrderedDict[int, FrameRecord] = OrderedDict()
        self._buffer_m1: OrderedDict[int, TrackingResult] = OrderedDict()
        self._buffer_m2: OrderedDict[int, DepthResult] = OrderedDict()
        self._buffer_t: OrderedDict[int, TrackingResult] = OrderedDict()
        self._buffer_f: OrderedDict[int, FusionResult] = OrderedDict()
        self._pending_tracks: OrderedDict[int, TrackingResult] = OrderedDict()
        self._pending_depths: OrderedDict[int, DepthResult] = OrderedDict()
        self._join_events: queue.Queue[tuple[str, object] | None] = queue.Queue(
            maxsize=self.JOIN_QUEUE_CAPACITY
        )
        self._draw_pending: queue.Queue[DrawJob | None] = queue.Queue(maxsize=1)

        self._synthetic_camera_frame = 0
        self._last_received_frame_id = 0
        self._last_selected_frame_id = 0
        self._last_detection_started_at = 0.0
        self._last_tracker_source_at = 0.0
        self._last_tracker_dt: float | None = None
        self._last_skipped_camera_frames = 0
        self._skipped_camera_frames = 0
        self._kalman_time_scale: float | None = None
        self._tracker_effective_fps = 0.0
        self._tracker_latency: float | None = None
        self._bearing_latency: float | None = None
        self._latest_tracking_control_frame_id = 0
        self._last_controlled_frame_id = 0
        self._latest: bytes | None = None
        self._last_processed = 0.0
        self._control_latency: float | None = None
        self._display_latency: float | None = None
        self._people: list[Person] = []
        self._persist_images = persist_images

        self.error = ""
        self.join_error = ""
        self.drawing_error = ""
        self.camera_buffer_evicted = 0
        self.dropped_frames = 0
        self.dropped_join_events = 0
        self.expired_join_results = 0
        self.rejected_config_results = 0
        self.rejected_stale_control_results = 0
        self.dropped_draw_frames = 0
        self.processed_frames = 0
        self.fused_frames = 0
        self.drawn_frames = 0

        self._join_thread = threading.Thread(
            target=self._join_loop, daemon=True, name="fusion-control-worker"
        )
        self._tracking_thread = threading.Thread(
            target=self._tracking_loop, daemon=True, name="tracking-worker"
        )
        self._draw_thread = threading.Thread(
            target=self._draw_loop, daemon=True, name="display-worker"
        )
        self._join_thread.start()
        device = 0 if torch.cuda.is_available() else "cpu"
        quantize = 16 if torch.cuda.is_available() else None
        self.depth = DepthWorker(device, quantize, self._on_depth_result)
        self._tracking_thread.start()
        self._draw_thread.start()

    def offer(self, image: np.ndarray, source_at: float | None = None,
              camera_frame_number: int | None = None):
        """Pipeline 1: append a non-wrapping, immutable BGR frame to buffer_c."""
        captured_at = time.monotonic() if source_at is None else source_at
        with self._camera_condition:
            if camera_frame_number is None:
                self._synthetic_camera_frame += 1
                camera_frame_number = self._synthetic_camera_frame
            else:
                self._synthetic_camera_frame = max(self._synthetic_camera_frame, camera_frame_number)
            if camera_frame_number <= self._last_received_frame_id:
                self.dropped_frames += 1
                return
            self._last_received_frame_id = camera_frame_number
            record = FrameRecord(
                self._stream_epoch, int(camera_frame_number), captured_at, image,
            )
            if len(self._buffer_c) >= self.CAMERA_BUFFER_CAPACITY:
                self.camera_buffer_evicted += 1
            retain(self._buffer_c, record.frame_id, record, self.CAMERA_BUFFER_CAPACITY)
            self._camera_condition.notify()

    def _next_camera_frame(self) -> FrameRecord | None:
        """Select the newest camera frame at the configured tracking rate."""
        with self._camera_condition:
            while not self._stop.is_set():
                candidate = next(reversed(self._buffer_c.values()), None)
                unseen = candidate is not None and candidate.frame_id > self._last_selected_frame_id
                wait_for_rate = max(
                    0.0,
                    self._last_detection_started_at + 1.0 / self.settings.process_fps
                    - time.monotonic(),
                )
                if unseen and wait_for_rate <= 0:
                    self._last_selected_frame_id = candidate.frame_id
                    self._last_detection_started_at = time.monotonic()
                    return candidate
                self._camera_condition.wait(timeout=min(0.2, wait_for_rate) if unseen else 0.2)
        return None

    def _offer_join(self, kind: str, result: object):
        if self._stop.is_set():
            return
        try:
            self._join_events.put_nowait((kind, result))
        except queue.Full:
            try:
                self._join_events.get_nowait()
                self._join_events.task_done()
            except queue.Empty:
                pass
            self._join_events.put_nowait((kind, result))
            with self.lock:
                self.dropped_join_events += 1

    def _on_depth_result(self, result: DepthResult):
        self._offer_join("depth", result)

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
            maximum_age = max(15.0, 3.0 / self.settings.depth_fps)
            return self._latest if time.monotonic() - self._last_processed <= maximum_age else None

    def configure(self, settings: FollowSettings):
        """Atomically publish a new config generation; never mix generations."""
        with self._camera_condition:
            previous = self.settings
            self.settings = settings
            if settings != previous:
                self.identity.configure(
                    settings.persistent_identity_enabled, settings.target_id,
                )
                self._config_version += 1
                self._pending_tracks.clear()
                self._pending_depths.clear()
                self._buffer_m1.clear()
                self._buffer_m2.clear()
                self._buffer_t.clear()
                self._buffer_f.clear()
                self._last_tracker_source_at = 0.0
                self._last_tracker_dt = None
                self._kalman_time_scale = None
                self._latest_tracking_control_frame_id = 0
                self.pid.reset()
                self.depth.reconfigure()
                self._drain_queue(self._draw_pending)
                with self.motion.lock:
                    self.motion.settings = settings
                    self.motion.stop_now()
                self._camera_condition.notify_all()
        return settings

    def status(self):
        depth_status = self.depth.status()
        identity_status = self.identity.status()
        with self.lock:
            result = {
                "follow_settings": self.settings.model_dump(),
                "config_version": self._config_version,
                "stream_epoch": self._stream_epoch,
                "processing_error": (self.error or self.join_error or self.drawing_error
                                     or depth_status["depth_error"]),
                "processed_total": self.processed_frames,
                "fused_total": self.fused_frames,
                "drawn_total": self.drawn_frames,
                "skipped_processing_frames": self.dropped_frames,
                "skipped_drawing_frames": self.dropped_draw_frames,
                "camera_buffer_evicted_total": self.camera_buffer_evicted,
                "join_expired_total": self.expired_join_results,
                "join_dropped_total": self.dropped_join_events,
                "config_rejected_total": self.rejected_config_results,
                "stale_control_rejected_total": self.rejected_stale_control_results,
                "camera_buffer_size": len(self._buffer_c),
                "model_buffer_size": len(self._buffer_m1),
                "depth_buffer_size": len(self._buffer_m2),
                "tracking_buffer_size": len(self._buffer_t),
                "fusion_buffer_size": len(self._buffer_f),
                "join_pending_tracking": len(self._pending_tracks),
                "join_pending_depth": len(self._pending_depths),
                "processed_image_age_seconds": round(time.monotonic() - self._last_processed, 1)
                if self._last_processed else None,
                "processing_latency_seconds": round(self._control_latency, 3)
                if self._control_latency is not None else None,
                "tracker_latency_seconds": round(self._tracker_latency, 3)
                if self._tracker_latency is not None else None,
                "bearing_control_latency_seconds": round(self._bearing_latency, 3)
                if self._bearing_latency is not None else None,
                "display_latency_seconds": round(self._display_latency, 3)
                if self._display_latency is not None else None,
                "tracker_effective_fps": round(self._tracker_effective_fps, 1),
                "tracker_frame_interval_seconds": round(self._last_tracker_dt, 3)
                if self._last_tracker_dt is not None else None,
                "kalman_time_scale": round(self._kalman_time_scale, 3)
                if self._kalman_time_scale is not None else None,
                "latest_skipped_camera_frames": self._last_skipped_camera_frames,
                "skipped_camera_frames_total": self._skipped_camera_frames,
                "last_camera_frame_number": self._last_selected_frame_id or None,
                "latest_fused_frame_id": self._last_controlled_frame_id or None,
                "visible_people": [
                    {"id": p.id, "tracker_id": p.internal_id,
                     "distance_m": round(p.distance, 2)
                     if p.distance is not None else None}
                    for p in self._people
                ],
            }
        result.update(depth_status)
        result.update(identity_status)
        result.update(self.motion.status())
        return result

    def _tracking_loop(self):
        """Pipelines 2+4: start depth, then run combined YOLO/BoT-SORT."""
        tracker_file = None
        try:
            device = 0 if torch.cuda.is_available() else "cpu"
            quantize = 16 if torch.cuda.is_available() else None
            segment = YOLO(str(SEGMENT_MODEL))
            calibration = None
            while not self._stop.is_set():
                frame = self._next_camera_frame()
                if frame is None:
                    continue
                started_at = time.monotonic()
                try:
                    image = frame.image
                    if calibration is None or (calibration.width, calibration.height) != (image.shape[1], image.shape[0]):
                        calibration = Calibration(image.shape[1], image.shape[0])
                    with self.lock:
                        settings = self.settings
                        config_version = self._config_version
                        previous_frame_id = self._last_selected_tracking_frame_id()
                        previous_source_at = self._last_tracker_source_at
                    if tracker_file is None:
                        # The first tracker is constructed from the currently
                        # active web/API settings rather than hard-coded values.
                        tracker_file = tracker_yaml(settings)
                    frame_dt = frame.source_at - previous_source_at if previous_source_at > 0 else None
                    skipped = (max(0, frame.frame_id - previous_frame_id - 1)
                               if previous_frame_id is not None else 0)
                    kalman_time_scale = configure_tracker_timing(
                        segment, frame_dt, settings.process_fps,
                    )
                    configure_tracker_thresholds(segment, settings)

                    # buffer_n is published before model.track starts. Depth
                    # and detection/tracking therefore operate on the exact
                    # same immutable image concurrently.
                    depth_requested = self.depth.offer(DepthJob(frame, config_version, settings))
                    track = segment.track(
                        source=image, tracker=str(tracker_file), persist=True,
                        classes=[0], conf=settings.detector_min_conf,
                        iou=settings.detection_iou, imgsz=settings.segment_imgsz,
                        retina_masks=False, device=device, quantize=quantize,
                        verbose=False,
                    )[0].cpu()
                    with self.lock:
                        if config_version != self._config_version:
                            self.rejected_config_results += 1
                            continue

                    people: list[Person] = []
                    boxes = np.empty((0, 4), dtype=np.float32)
                    ids = np.empty(0, dtype=int)
                    confidences = np.empty(0, dtype=np.float32)
                    if track.boxes is not None and track.boxes.id is not None:
                        boxes = track.boxes.xyxy.numpy()
                        ids = track.boxes.id.numpy().astype(int)
                        confidences = track.boxes.conf.numpy()
                    embeddings = active_track_embeddings(segment)
                    observations = tuple(
                        IdentityObservation(
                            int(internal_id), float(confidence), box,
                            embeddings.get(int(internal_id)),
                        )
                        for box, internal_id, confidence in zip(boxes, ids, confidences)
                    )
                    # Keep the configuration generation stable while identity
                    # state is mutated; an in-flight old frame must not switch
                    # the manager back to an earlier target/toggle setting.
                    with self.lock:
                        if config_version != self._config_version:
                            self.rejected_config_results += 1
                            continue
                        public_ids = self.identity.resolve(
                            observations, settings, frame.source_at,
                            (image.shape[1], image.shape[0]),
                        )
                    if track.masks is not None:
                        masks = track.masks.data.numpy() > 0.5
                        for box, internal_id, confidence, mask in zip(boxes, ids, confidences, masks):
                            if not mask.any():
                                continue
                            public_id = public_ids[int(internal_id)]
                            _, _, center, foot = mask_geometry(mask, calibration)
                            people.append(Person(
                                public_id, float(confidence), box, mask, center, foot,
                                None, "depth-pending", int(internal_id),
                            ))
                    completed_at = time.monotonic()
                    output = TrackingResult(
                        frame, config_version, settings, calibration, track, tuple(people),
                        previous_frame_id, frame_dt, kalman_time_scale, skipped, depth_requested,
                        started_at, completed_at,
                    )
                    with self.lock:
                        if config_version != self._config_version:
                            self.rejected_config_results += 1
                            continue
                        if frame_dt is not None and frame_dt > 0:
                            instant_fps = 1.0 / frame_dt
                            self._tracker_effective_fps = (
                                instant_fps if self._tracker_effective_fps == 0
                                else 0.8 * self._tracker_effective_fps + 0.2 * instant_fps
                            )
                        self._last_tracker_source_at = frame.source_at
                        self._last_skipped_camera_frames = skipped
                        self._skipped_camera_frames += skipped
                        self._last_tracker_dt = frame_dt
                        self._kalman_time_scale = kalman_time_scale
                        self._tracker_latency = completed_at - frame.source_at
                        retain(self._buffer_m1, frame.frame_id, output, self.RESULT_BUFFER_CAPACITY)
                        retain(self._buffer_t, frame.frame_id, output, self.RESULT_BUFFER_CAPACITY)
                        self.processed_frames += 1
                        self.error = ""
                    # Every tracking result advances the safety watermark and
                    # can refresh steering without waiting for low-rate depth.
                    self._update_tracking_control(output)
                    if depth_requested:
                        self._offer_join("track", output)
                except Exception as exc:
                    # Treat a failed tracking frame as a newer target-loss
                    # observation so an older depth result cannot revive it.
                    with self.lock:
                        self._latest_tracking_control_frame_id = max(
                            self._latest_tracking_control_frame_id, frame.frame_id,
                        )
                        self.pid.reset()
                    self.motion.update_tracking(
                        False, 0.0, frame.source_at, frame.frame_id,
                    )
                    with self.lock:
                        self.error = str(exc)
        except Exception as exc:
            self.motion.stop_now()
            with self.lock:
                self.error = str(exc)
        finally:
            if tracker_file is not None:
                tracker_file.unlink(missing_ok=True)

    def _update_tracking_control(self, tracking: TrackingResult):
        """Advance the control watermark and update bearing at tracker rate."""
        target = next(
            (person for person in tracking.people
             if person.id == tracking.settings.target_id),
            None,
        )
        with self.lock:
            if tracking.config_version != self._config_version:
                self.rejected_config_results += 1
                return
            if tracking.frame.frame_id < self._latest_tracking_control_frame_id:
                self.rejected_stale_control_results += 1
                return
            self._latest_tracking_control_frame_id = tracking.frame.frame_id
            with self.motion.lock:
                active = (self.motion.following and self.motion.mode == "auto"
                          and not self.motion.estop)
            if target is None:
                self.pid.reset()
                yaw = 0.0
            elif active:
                yaw = self.pid.update_bearing(
                    target, tracking.calibration, tracking.settings,
                    tracking.frame.source_at,
                )
            else:
                self.pid.reset_bearing()
                yaw = 0.0
            accepted = self.motion.update_tracking(
                target is not None, yaw, tracking.frame.source_at,
                tracking.frame.frame_id,
            )
            if accepted:
                self._bearing_latency = time.monotonic() - tracking.frame.source_at
            else:
                self.rejected_stale_control_results += 1

    def _last_selected_tracking_frame_id(self) -> int | None:
        if not self._buffer_m1:
            return None
        return next(reversed(self._buffer_m1))

    def _join_timeout(self, settings: FollowSettings) -> float:
        return min(2.0, max(1.0, 4.0 / settings.depth_fps))

    def _expire_join_records(self, now: float):
        expired = 0
        for pending in (self._pending_tracks, self._pending_depths):
            for frame_id, result in list(pending.items()):
                if now - result.frame.source_at > self._join_timeout(result.settings):
                    pending.pop(frame_id, None)
                    expired += 1
        self.expired_join_results += expired

    def _join_loop(self):
        """Pipeline 5 input: non-blocking exact-frame join of T and M2."""
        while not self._stop.is_set():
            try:
                event = self._join_events.get(timeout=0.2)
            except queue.Empty:
                with self.lock:
                    self._expire_join_records(time.monotonic())
                continue
            if event is None:
                self._join_events.task_done()
                break
            kind, result = event
            pair = None
            try:
                with self.lock:
                    if result.config_version != self._config_version:
                        self.rejected_config_results += 1
                        continue
                    frame_id = result.frame.frame_id
                    if kind == "track":
                        self._pending_tracks[frame_id] = result
                    else:
                        retain(self._buffer_m2, frame_id, result, self.RESULT_BUFFER_CAPACITY)
                        self._pending_depths[frame_id] = result
                    track = self._pending_tracks.get(frame_id)
                    depth = self._pending_depths.get(frame_id)
                    if track is not None and depth is not None:
                        self._pending_tracks.pop(frame_id, None)
                        self._pending_depths.pop(frame_id, None)
                        pair = track, depth
                    self._expire_join_records(time.monotonic())
                if pair is not None:
                    self._fuse_and_control(*pair)
            except Exception as exc:
                if pair is not None:
                    failed_tracking = pair[0]
                    with self.lock:
                        self._latest_tracking_control_frame_id = max(
                            self._latest_tracking_control_frame_id,
                            failed_tracking.frame.frame_id,
                        )
                        self.pid.reset()
                    self.motion.update_tracking(
                        False, 0.0, failed_tracking.frame.source_at,
                        failed_tracking.frame.frame_id,
                    )
                else:
                    self.motion.stop_now()
                    self.pid.reset()
                with self.lock:
                    self.join_error = str(exc)
            finally:
                self._join_events.task_done()

    def _fuse_and_control(self, tracking: TrackingResult, depth: DepthResult):
        """Estimate exact-frame distances and update the 50 Hz command source."""
        if (tracking.frame.stream_epoch != depth.frame.stream_epoch
                or tracking.frame.frame_id != depth.frame.frame_id):
            raise ValueError("Tracking/depth frame mismatch")
        if tracking.config_version != depth.config_version:
            raise ValueError("Tracking/depth configuration mismatch")
        people: list[Person] = []
        for person in tracking.people:
            try:
                distance, method, center, foot = measured_distance(
                    person.mask, depth.depth, tracking.calibration,
                    tracking.settings.camera_height_m, tracking.settings.distance_mode,
                )
            except (ValueError, IndexError):
                distance, method = None, "invalid-depth"
                center, foot = person.center, person.foot
            people.append(Person(
                person.id, person.confidence, person.box, person.mask,
                center, foot, distance, method, person.internal_id,
            ))

        target = next((p for p in people if p.id == tracking.settings.target_id), None)
        with self.lock:
            if tracking.config_version != self._config_version:
                self.rejected_config_results += 1
                return
            if tracking.frame.frame_id < self._latest_tracking_control_frame_id:
                # A newer tracker observation (especially target loss) has
                # already advanced the safety state. Older depth must never
                # restore a command from the past.
                self.rejected_stale_control_results += 1
                return
            if tracking.frame.frame_id <= self._last_controlled_frame_id:
                return
            with self.motion.lock:
                active = (self.motion.following and self.motion.mode == "auto"
                          and not self.motion.estop)
            if target is None:
                self.pid.reset()
                linear = yaw = 0.0
            elif target.distance is None or not active:
                self.pid.reset_distance()
                linear = 0.0
                yaw = self.pid.yaw if active and target.distance is not None else 0.0
            else:
                linear = self.pid.update_distance(
                    target, tracking.settings, tracking.frame.source_at,
                )
                yaw = self.pid.yaw
            accepted = self.motion.update_measurement(
                target is not None, target.distance if target else None,
                linear, yaw, tracking.frame.source_at,
                tracking.frame.frame_id,
            )
            if not accepted:
                self.rejected_stale_control_results += 1
                return
            completed_at = time.monotonic()
            self._last_controlled_frame_id = tracking.frame.frame_id
            self._people = people
            self._control_latency = completed_at - tracking.frame.source_at
            self.fused_frames += 1
            self.join_error = ""
            retain(
                self._buffer_f, tracking.frame.frame_id,
                FusionResult(
                    tracking.frame.frame_id, tracking.config_version, tuple(people),
                    linear if active else 0.0, yaw if active else 0.0, completed_at,
                ),
                self.RESULT_BUFFER_CAPACITY,
            )
        self._offer_draw(DrawJob(
            tracking.frame.frame_id, tracking.track, depth.depth, people,
            tracking.settings.target_id, linear if active else 0.0,
            yaw if active else 0.0, active, tracking.frame.source_at,
        ))

    def _draw_loop(self):
        """Pipeline 6 display worker; HTTP/configuration remains independent."""
        while not self._stop.is_set():
            try:
                job = self._draw_pending.get(timeout=0.2)
            except queue.Empty:
                continue
            if job is None:
                self._draw_pending.task_done()
                break
            try:
                # A newer fused frame may have arrived after get() woke this
                # worker. Drain once more so expensive rendering always starts
                # from the freshest available result.
                while True:
                    try:
                        newer = self._draw_pending.get_nowait()
                    except queue.Empty:
                        break
                    self._draw_pending.task_done()
                    if newer is None:
                        return
                    job = newer
                    with self.lock:
                        self.dropped_draw_frames += 1
                output = annotate(
                    job.track, job.depth, job.people, job.target_id,
                    job.linear, job.yaw, job.following, job.frame_id,
                )
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
                with self.lock:
                    self.drawing_error = str(exc)
            finally:
                self._draw_pending.task_done()

    @staticmethod
    def _drain_queue(queue_: queue.Queue):
        while True:
            try:
                queue_.get_nowait()
            except queue.Empty:
                return
            queue_.task_done()

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
        # Safety is established before workers are asked to exit.
        self.motion.set_estop(True)
        self._stop.set()
        with self._camera_condition:
            self._camera_condition.notify_all()
        self._tracking_thread.join(timeout=30)
        self.depth.close()
        self._wake(self._join_events)
        self._join_thread.join(timeout=30)
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
    parser.add_argument("--fps", type=float, default=5.0, help="Raw image retention frequency")
    parser.add_argument("--camera-fps", type=float, default=30.0,
                        help="Maximum camera input frequency delivered to tracking")
    parser.add_argument("--process-fps", type=float, default=15.0,
                        help="Maximum segmentation/tracking frequency")
    parser.add_argument("--depth-fps", type=float, default=5.0,
                        help="Maximum independent metric-depth frequency")
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
    capture = CaptureSettings(fps=args.fps, tracking_fps=args.camera_fps,
                              duration=args.duration)
    follow = FollowSettings(process_fps=args.process_fps, depth_fps=args.depth_fps)
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
                for sequence, path in enumerate(paths, start=1):
                    if engine._stop.is_set():
                        break
                    image = cv2.imread(str(path))
                    if image is not None:
                        engine.offer(image, camera_frame_number=sequence)
                    time.sleep(1.0 / capture.tracking_fps)
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
        bridge.close()
        raw_store.close()
        processed_store.close()


if __name__ == "__main__":
    main()
