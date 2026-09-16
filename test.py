"""Monocular person-following reference pipeline.

Pipeline
--------
YOLO26 instance segmentation -> BoT-SORT + ReID -> YOLO26 metric depth
-> person range estimation -> acceleration-limited follow controller.

This is deliberately a single-file prototype.  All user-tunable settings are
constants below; there is no command-line parser.

Coordinate conventions
----------------------
OpenCV camera frame: x right, y down, z forward.
Vehicle/base frame:   x forward, y left, z up.

Important calibration note
--------------------------
The supplied intrinsics/extrinsics are placeholders.  Metric monocular depth
can produce values in metres, but camera-specific scale calibration is still
recommended before the values are used for closed-loop motion.  The foot-ray
method is disabled until camera height and pitch have been measured.
"""

from __future__ import annotations

import math
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Literal

import cv2
import numpy as np
import torch
from ultralytics import YOLO
from ultralytics.utils import ASSETS, YAML


# =============================================================================
# User settings (edit here; no terminal arguments are required)
# =============================================================================

# Ultralytics built-in test image.  Replace with 0 for a camera, or a video path. SOURCE: int | str | Path = ASSETS / "bus.jpg"
SOURCE: int | str | Path = ASSETS / "bus.jpg"
# SOURCE = Path("solutions_ci_demo.mp4")
TEST_IMAGE_FRAMES = 24  # Repeat bus.jpg so tracker IDs and controller can be tested.
MAX_FRAMES: int | None = None  # None means no limit for a camera/video.

SEGMENT_MODEL = "yolo26n-seg.pt"
DEPTH_MODEL = "yolo26n-depth.pt"

# Official tracker-only ReID encoder.  It is downloaded automatically.
# If ONNX Runtime is unavailable, use "auto" to reuse YOLO backbone features.
REID_MODEL = "yolo26n-reid.onnx"

SEGMENT_DEVICE: int | str = 0 if torch.cuda.is_available() else "cpu"
DEPTH_DEVICE: int | str = 0 if torch.cuda.is_available() else "cpu"
INFERENCE_QUANTIZE: int | None = 16 if torch.cuda.is_available() else None
SEGMENT_IMGSZ = 640
DEPTH_IMGSZ = 768
PERSON_CLASS_ID = 0  # COCO person class
DETECTION_CONFIDENCE = 0.25
DETECTION_IOU = 0.70

# Run depth every frame for control.  Increasing this saves compute but makes
# the range stale while the camera or target moves.
DEPTH_EVERY_N_FRAMES = 1

# "depth": robust depth from the lower part of the instance mask.
# "footpoint": calibrated foot-pixel/ground-plane intersection.
# "fused": combine both when their estimates agree.
DISTANCE_MODE: Literal["depth", "footpoint", "fused"] = "depth"
TARGET_DISTANCE_M = 1.0
MIN_VALID_DEPTH_M = 0.10
MAX_VALID_DEPTH_M = 30.0

# --------------------------- CAMERA INTRINSICS TODO ---------------------------
# TODO: Replace these with values from cv2.calibrateCamera() for the actual
#       capture resolution.  Leaving them as None uses APPROX_HORIZONTAL_FOV_DEG
#       so the demo runs, but this is not accurate enough for final control.
CAMERA_FX_PX: float | None = None
CAMERA_FY_PX: float | None = None
CAMERA_CX_PX: float | None = None
CAMERA_CY_PX: float | None = None
APPROX_HORIZONTAL_FOV_DEG = 70.0

# --------------------------- CAMERA EXTRINSICS TODO ---------------------------
# TODO: Measure camera pose relative to the vehicle/base frame.
# Pitch is positive when the optical axis points downward.
# CAMERA_HEIGHT_M and CAMERA_PITCH_DOWN_DEG must both be filled before the
# footpoint method is enabled.
CAMERA_HEIGHT_M: float | None = None
CAMERA_PITCH_DOWN_DEG: float | None = None
CAMERA_ROLL_DEG = 0.0
CAMERA_YAW_LEFT_DEG = 0.0
CAMERA_FORWARD_OFFSET_M = 0.0
CAMERA_LEFT_OFFSET_M = 0.0

# Controller convention: positive linear speed is forward; positive yaw rate
# turns left.  Set ALLOW_REVERSE=False if the chassis must never reverse.
ALLOW_REVERSE = True
MAX_FORWARD_SPEED_MPS = 0.8
MAX_REVERSE_SPEED_MPS = 0.25
MAX_LINEAR_ACCEL_MPS2 = 0.7
MAX_LINEAR_DECEL_MPS2 = 1.2
MAX_YAW_RATE_RADPS = 1.2
MAX_YAW_ACCEL_RADPS2 = 2.5
DISTANCE_DEADBAND_M = 0.08
BEARING_DEADBAND_DEG = 2.0

SAVE_OUTPUT = True
OUTPUT_PATH = Path("follow_demo.mp4")
SHOW_WINDOW = False  # Keep False o n a headless server.


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole intrinsics for the current image resolution."""

    fx: float
    fy: float
    cx: float
    cy: float
    approximate: bool

    @property
    def matrix(self) -> np.ndarray:
        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )


@dataclass(frozen=True)
class CameraExtrinsics:
    """Rigid transform p_base = R_base_from_camera @ p_camera + t."""

    rotation_base_from_camera: np.ndarray
    translation_base_from_camera_m: np.ndarray
    calibrated_for_ground_plane: bool


@dataclass
class PersonMeasurement:
    track_id: int
    confidence: float
    box_xyxy: np.ndarray
    mask: np.ndarray
    center_uv: tuple[float, float]
    foot_uv: tuple[float, float]
    depth_distance_m: float | None
    footpoint_distance_m: float | None
    distance_m: float | None
    distance_method: str
    mask_area_px: int


@dataclass(frozen=True)
class ControlCommand:
    """Generic differential-drive command and its time derivatives."""

    linear_velocity_mps: float
    linear_acceleration_mps2: float
    yaw_rate_radps: float
    yaw_acceleration_radps2: float
    distance_error_m: float | None
    bearing_error_rad: float | None
    target_track_id: int | None
    valid_target: bool


def _rotation_x(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)


def _rotation_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)


def _rotation_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


def resolve_intrinsics(width: int, height: int) -> CameraIntrinsics:
    """Return calibrated intrinsics, or an explicitly marked FOV approximation."""
    supplied = (CAMERA_FX_PX, CAMERA_FY_PX, CAMERA_CX_PX, CAMERA_CY_PX)
    if all(value is not None for value in supplied):
        return CameraIntrinsics(
            fx=float(CAMERA_FX_PX),
            fy=float(CAMERA_FY_PX),
            cx=float(CAMERA_CX_PX),
            cy=float(CAMERA_CY_PX),
            approximate=False,
        )

    # fx = W / (2 tan(FOV_x / 2)); square pixels are assumed for fy.
    fx = width / (2.0 * math.tan(math.radians(APPROX_HORIZONTAL_FOV_DEG) / 2.0))
    return CameraIntrinsics(
        fx=fx,
        fy=fx,
        cx=(width - 1.0) / 2.0,
        cy=(height - 1.0) / 2.0,
        approximate=True,
    )


def resolve_extrinsics() -> CameraExtrinsics:
    """Build the camera-to-base transform from the TODO mount parameters."""
    # A level OpenCV camera maps (right, down, forward) to
    # vehicle (forward, left, up).
    nominal = np.array(
        [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
        dtype=np.float64,
    )
    pitch_down = math.radians(CAMERA_PITCH_DOWN_DEG or 0.0)
    roll = math.radians(CAMERA_ROLL_DEG)
    yaw_left = math.radians(CAMERA_YAW_LEFT_DEG)
    rotation = _rotation_z(yaw_left) @ _rotation_y(pitch_down) @ _rotation_x(roll) @ nominal
    translation = np.array(
        [CAMERA_FORWARD_OFFSET_M, CAMERA_LEFT_OFFSET_M, CAMERA_HEIGHT_M or 0.0],
        dtype=np.float64,
    )
    calibrated = CAMERA_HEIGHT_M is not None and CAMERA_PITCH_DOWN_DEG is not None
    return CameraExtrinsics(rotation, translation, calibrated)


def make_botsort_reid_yaml() -> Path:
    """Create a temporary Ultralytics BoT-SORT configuration with ReID enabled."""
    import ultralytics

    default_path = Path(ultralytics.__file__).parent / "cfg" / "trackers" / "botsort.yaml"
    cfg = YAML.load(default_path)
    cfg.update(
        {
            "tracker_type": "botsort",
            "track_high_thresh": 0.25,
            "track_low_thresh": 0.10,
            "new_track_thresh": 0.25,
            "track_buffer": 45,
            "match_thresh": 0.80,
            "fuse_score": True,
            # The camera is mounted on a moving car, so GMC should remain on.
            "gmc_method": "sparseOptFlow",
            "proximity_thresh": 0.50,
            "appearance_thresh": 0.80,
            "with_reid": True,
            "model": REID_MODEL,
        }
    )
    handle = tempfile.NamedTemporaryFile(prefix="botsort_reid_", suffix=".yaml", delete=False)
    handle.close()
    path = Path(handle.name)
    YAML.save(path, cfg)
    return path


class FrameStream:
    """Yield BGR frames and deterministic timestamps from image/video/camera input."""

    IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}

    def __init__(self, source: int | str | Path):
        self.source = source
        self.fps = 30.0
        self.frame_size: tuple[int, int] | None = None

    def __iter__(self) -> Iterator[tuple[int, float, np.ndarray]]:
        path = Path(self.source) if isinstance(self.source, (str, Path)) else None
        if path is not None and path.suffix.lower() in self.IMAGE_SUFFIXES:
            frame = cv2.imread(str(path))
            if frame is None:
                raise FileNotFoundError(f"Cannot read image: {path}")
            self.frame_size = (frame.shape[1], frame.shape[0])
            count = TEST_IMAGE_FRAMES if MAX_FRAMES is None else min(TEST_IMAGE_FRAMES, MAX_FRAMES)
            for index in range(count):
                yield index, index / self.fps, frame.copy()
            return

        capture = cv2.VideoCapture(self.source)
        if not capture.isOpened():
            raise RuntimeError(f"Cannot open source: {self.source!r}")
        fps = float(capture.get(cv2.CAP_PROP_FPS))
        self.fps = fps if math.isfinite(fps) and fps > 1e-3 else 30.0
        index = 0
        wall_start = time.monotonic()
        try:
            while MAX_FRAMES is None or index < MAX_FRAMES:
                ok, frame = capture.read()
                if not ok:
                    break
                self.frame_size = (frame.shape[1], frame.shape[0])
                if isinstance(self.source, int):
                    timestamp = time.monotonic() - wall_start
                else:
                    timestamp = index / self.fps
                yield index, timestamp, frame
                index += 1
        finally:
            capture.release()


def foot_pixel(mask: np.ndarray) -> tuple[float, float]:
    """Robust foot/ground-contact pixel from the bottom 2% of a person mask."""
    ys, xs = np.nonzero(mask)
    if ys.size == 0:
        raise ValueError("Empty person mask")
    bottom_threshold = np.quantile(ys, 0.98)
    bottom_x = xs[ys >= bottom_threshold]
    return float(np.median(bottom_x)), float(np.max(ys))


def mask_center(mask: np.ndarray) -> tuple[float, float]:
    ys, xs = np.nonzero(mask)
    if ys.size == 0:
        raise ValueError("Empty person mask")
    return float(np.median(xs)), float(np.median(ys))


def robust_mask_depth_distance(
    mask: np.ndarray,
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    extrinsics: CameraExtrinsics,
) -> float | None:
    """Estimate horizontal camera-to-person range from lower-mask depth pixels.

    For a pinhole camera and axial depth Z:

        X_c = (u-cx) Z / fx
        Y_c = (v-cy) Z / fy
        p_b = R_bc [X_c, Y_c, Z]^T + t_bc
        range_horizontal = sqrt(p_bx^2 + p_by^2)

    A median/MAD filter suppresses mask edges, floor leakage and depth outliers.
    """
    if depth_m.shape != mask.shape:
        depth_m = cv2.resize(depth_m, (mask.shape[1], mask.shape[0]), interpolation=cv2.INTER_LINEAR)

    ys_all, _ = np.nonzero(mask)
    if ys_all.size < 30:
        return None

    # Legs/lower torso are less likely to mix with background above the head.
    low = np.quantile(ys_all, 0.55)
    high = np.quantile(ys_all, 0.985)
    yy, xx = np.indices(mask.shape)
    sample_mask = mask & (yy >= low) & (yy <= high)

    z = depth_m[sample_mask].astype(np.float64)
    u = xx[sample_mask].astype(np.float64)
    v = yy[sample_mask].astype(np.float64)
    valid = np.isfinite(z) & (z >= MIN_VALID_DEPTH_M) & (z <= MAX_VALID_DEPTH_M)
    if np.count_nonzero(valid) < 20:
        return None
    z, u, v = z[valid], u[valid], v[valid]

    x_c = (u - intrinsics.cx) * z / intrinsics.fx
    y_c = (v - intrinsics.cy) * z / intrinsics.fy
    points_camera = np.stack((x_c, y_c, z), axis=1)
    points_base = points_camera @ extrinsics.rotation_base_from_camera.T
    points_base += extrinsics.translation_base_from_camera_m
    ranges = np.hypot(points_base[:, 0], points_base[:, 1])

    median = float(np.median(ranges))
    mad = float(np.median(np.abs(ranges - median)))
    if mad > 1e-6:
        ranges = ranges[np.abs(ranges - median) <= 3.5 * 1.4826 * mad]
    return float(np.median(ranges)) if ranges.size else median


def footpoint_ground_distance(
    foot_uv: tuple[float, float],
    intrinsics: CameraIntrinsics,
    extrinsics: CameraExtrinsics,
) -> float | None:
    """Intersect the foot-pixel ray with vehicle ground plane z_base=0.

    ray_c = K^-1 [u,v,1]^T
    ray_b = R_bc ray_c
    p_b(lambda) = t_bc + lambda ray_b
    lambda = -t_bc,z / ray_b,z

    This requires camera height and orientation; it intentionally returns None
    while the extrinsics TODO values are missing.
    """
    if not extrinsics.calibrated_for_ground_plane or intrinsics.approximate:
        return None
    u, v = foot_uv
    ray_camera = np.linalg.inv(intrinsics.matrix) @ np.array([u, v, 1.0], dtype=np.float64)
    ray_base = extrinsics.rotation_base_from_camera @ ray_camera
    origin = extrinsics.translation_base_from_camera_m
    if ray_base[2] >= -1e-8:
        return None  # Ray does not point toward the ground.
    scale = -origin[2] / ray_base[2]
    if scale <= 0:
        return None
    point = origin + scale * ray_base
    return float(np.hypot(point[0], point[1]))


def choose_distance(depth_value: float | None, foot_value: float | None) -> tuple[float | None, str]:
    if DISTANCE_MODE == "depth":
        return depth_value, "mask-depth"
    if DISTANCE_MODE == "footpoint":
        return foot_value, "foot-ray"
    if depth_value is None:
        return foot_value, "foot-ray"
    if foot_value is None:
        return depth_value, "mask-depth"
    relative_difference = abs(depth_value - foot_value) / max(depth_value, foot_value, 1e-6)
    if relative_difference <= 0.30:
        # Depth gets more weight because a one-pixel foot error can move the
        # ground intersection considerably near the horizon.
        return 0.70 * depth_value + 0.30 * foot_value, "fused"
    return depth_value, "mask-depth(disagreement)"


class MeasurementExtractor:
    """Join track boxes, instance masks and one dense depth map by result index."""

    def __init__(self, intrinsics: CameraIntrinsics, extrinsics: CameraExtrinsics):
        self.intrinsics = intrinsics
        self.extrinsics = extrinsics
        self.smoothed_depth_by_id: dict[int, float] = {}

    def __call__(self, track_result, depth_m: np.ndarray) -> list[PersonMeasurement]:
        if track_result.boxes is None or track_result.masks is None or track_result.boxes.id is None:
            return []

        boxes = track_result.boxes.xyxy.detach().cpu().numpy()
        ids = track_result.boxes.id.detach().cpu().numpy().astype(int)
        confs = track_result.boxes.conf.detach().cpu().numpy()
        masks = track_result.masks.data.detach().cpu().numpy() > 0.5
        measurements: list[PersonMeasurement] = []

        for box, track_id, confidence, mask in zip(boxes, ids, confs, masks):
            if not np.any(mask):
                continue
            center = mask_center(mask)
            foot = foot_pixel(mask)
            depth_distance = robust_mask_depth_distance(
                mask, depth_m, self.intrinsics, self.extrinsics
            )
            if depth_distance is not None:
                old = self.smoothed_depth_by_id.get(track_id)
                depth_distance = depth_distance if old is None else 0.65 * old + 0.35 * depth_distance
                self.smoothed_depth_by_id[track_id] = depth_distance
            foot_distance = footpoint_ground_distance(foot, self.intrinsics, self.extrinsics)
            distance, method = choose_distance(depth_distance, foot_distance)
            measurements.append(
                PersonMeasurement(
                    track_id=track_id,
                    confidence=float(confidence),
                    box_xyxy=box,
                    mask=mask,
                    center_uv=center,
                    foot_uv=foot,
                    depth_distance_m=depth_distance,
                    footpoint_distance_m=foot_distance,
                    distance_m=distance,
                    distance_method=method,
                    mask_area_px=int(mask.sum()),
                )
            )
        return measurements


class TargetSelector:
    """Lock one track ID; reacquire a large central person after a timeout."""

    def __init__(self, image_width: int, lost_tolerance_frames: int = 30):
        self.image_width = image_width
        self.lost_tolerance_frames = lost_tolerance_frames
        self.target_id: int | None = None
        self.lost_frames = 0

    def select(self, people: list[PersonMeasurement]) -> PersonMeasurement | None:
        if self.target_id is not None:
            locked = next((person for person in people if person.track_id == self.target_id), None)
            if locked is not None:
                self.lost_frames = 0
                return locked
            self.lost_frames += 1
            if self.lost_frames <= self.lost_tolerance_frames:
                return None
            self.target_id = None

        if not people:
            return None

        # Prefer a visible, large and central instance.  A real robot should
        # replace this with click-to-select, face enrollment or operator ID.
        def score(person: PersonMeasurement) -> float:
            center_penalty = abs(person.center_uv[0] - self.image_width / 2) / self.image_width
            valid_range_bonus = 1.0 if person.distance_m is not None else 0.0
            return math.log1p(person.mask_area_px) - 2.0 * center_penalty + valid_range_bonus

        selected = max(people, key=score)
        self.target_id = selected.track_id
        self.lost_frames = 0
        return selected


class FollowController:
    """Distance PID + bearing PD with velocity/acceleration saturation."""

    def __init__(self):
        self.last_time: float | None = None
        self.last_distance_error = 0.0
        self.last_bearing_error = 0.0
        self.distance_integral = 0.0
        self.linear_velocity = 0.0
        self.yaw_rate = 0.0

        self.kp_distance = 0.85
        self.ki_distance = 0.08
        self.kd_distance = 0.12
        self.kp_bearing = 1.8
        self.kd_bearing = 0.12

    @staticmethod
    def _clip(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def update(
        self,
        target: PersonMeasurement | None,
        intrinsics: CameraIntrinsics,
        timestamp: float,
    ) -> ControlCommand:
        dt = 1.0 / 30.0 if self.last_time is None else timestamp - self.last_time
        dt = self._clip(dt, 1.0 / 120.0, 0.25)
        self.last_time = timestamp

        valid = target is not None and target.distance_m is not None
        if valid:
            distance_error = float(target.distance_m - TARGET_DISTANCE_M)
            # atan((u-cx)/fx): positive means the person is to image-right.
            bearing_error = math.atan((target.center_uv[0] - intrinsics.cx) / intrinsics.fx)

            if abs(distance_error) < DISTANCE_DEADBAND_M:
                distance_error = 0.0
            if abs(bearing_error) < math.radians(BEARING_DEADBAND_DEG):
                bearing_error = 0.0

            distance_rate = (distance_error - self.last_distance_error) / dt
            bearing_rate = (bearing_error - self.last_bearing_error) / dt
            self.distance_integral = self._clip(
                self.distance_integral + distance_error * dt, -0.8, 0.8
            )
            desired_linear = (
                self.kp_distance * distance_error
                + self.ki_distance * self.distance_integral
                + self.kd_distance * distance_rate
            )
            min_speed = -MAX_REVERSE_SPEED_MPS if ALLOW_REVERSE else 0.0
            desired_linear = self._clip(desired_linear, min_speed, MAX_FORWARD_SPEED_MPS)

            # Positive image-right error requires a right turn, i.e. negative
            # yaw under the standard positive-left convention.
            desired_yaw = -(self.kp_bearing * bearing_error + self.kd_bearing * bearing_rate)
            desired_yaw = self._clip(desired_yaw, -MAX_YAW_RATE_RADPS, MAX_YAW_RATE_RADPS)

            self.last_distance_error = distance_error
            self.last_bearing_error = bearing_error
        else:
            # Fail-safe: never continue blind.  Smoothly brake and stop yawing.
            distance_error = None
            bearing_error = None
            desired_linear = 0.0
            desired_yaw = 0.0
            self.distance_integral = 0.0

        requested_acceleration = (desired_linear - self.linear_velocity) / dt
        linear_acceleration = self._clip(
            requested_acceleration, -MAX_LINEAR_DECEL_MPS2, MAX_LINEAR_ACCEL_MPS2
        )
        self.linear_velocity += linear_acceleration * dt

        requested_yaw_acceleration = (desired_yaw - self.yaw_rate) / dt
        yaw_acceleration = self._clip(
            requested_yaw_acceleration, -MAX_YAW_ACCEL_RADPS2, MAX_YAW_ACCEL_RADPS2
        )
        self.yaw_rate += yaw_acceleration * dt

        return ControlCommand(
            linear_velocity_mps=self.linear_velocity,
            linear_acceleration_mps2=linear_acceleration,
            yaw_rate_radps=self.yaw_rate,
            yaw_acceleration_radps2=yaw_acceleration,
            distance_error_m=distance_error,
            bearing_error_rad=bearing_error,
            target_track_id=target.track_id if target is not None else None,
            valid_target=valid,
        )


def depth_to_colormap(depth_m: np.ndarray) -> np.ndarray:
    valid = np.isfinite(depth_m) & (depth_m > 0)
    normalized = np.zeros(depth_m.shape, dtype=np.uint8)
    if np.any(valid):
        near, far = np.percentile(depth_m[valid], (2, 98))
        clipped = np.clip(depth_m, near, max(far, near + 1e-6))
        # Near is warm, far is cool.
        normalized = (255.0 * (far - clipped) / max(far - near, 1e-6)).astype(np.uint8)
    return cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)


def annotate(
    track_result,
    depth_m: np.ndarray,
    people: list[PersonMeasurement],
    target: PersonMeasurement | None,
    command: ControlCommand,
) -> np.ndarray:
    # Draw masks through Ultralytics, but render compact custom labels so the
    # tracker label and our distance label do not overlap.
    view = track_result.plot(boxes=False, masks=True, labels=False)
    for person in people:
        foot = tuple(int(round(value)) for value in person.foot_uv)
        color = (0, 255, 0) if target is not None and person.track_id == target.track_id else (0, 180, 255)
        cv2.circle(view, foot, 5, color, -1)
        x1, y1, x2, y2 = (int(value) for value in person.box_xyxy)
        cv2.rectangle(view, (x1, y1), (x2, y2), color, 2)
        distance_text = "? m" if person.distance_m is None else f"{person.distance_m:.2f} m"
        label = f"ID {person.track_id} | {person.confidence:.2f} | {distance_text}"
        label_y = max(20, y1 - 8)
        text_width, text_height = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.52, 2
        )[0]
        cv2.rectangle(
            view,
            (x1, label_y - text_height - 5),
            (min(view.shape[1] - 1, x1 + text_width + 4), label_y + 4),
            (20, 20, 20),
            -1,
        )
        cv2.putText(
            view,
            label,
            (x1 + 2, label_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            color,
            2,
            cv2.LINE_AA,
        )

    line_1 = (
        f"target={command.target_track_id} valid={command.valid_target} "
        f"v={command.linear_velocity_mps:+.2f}m/s a={command.linear_acceleration_mps2:+.2f}m/s2"
    )
    line_2 = (
        f"yaw={command.yaw_rate_radps:+.2f}rad/s "
        f"yaw_acc={command.yaw_acceleration_radps2:+.2f}rad/s2"
    )
    cv2.rectangle(view, (0, 0), (view.shape[1], 62), (20, 20, 20), -1)
    cv2.putText(view, line_1, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2)
    cv2.putText(view, line_2, (10, 51), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2)

    depth_view = depth_to_colormap(depth_m)
    cv2.putText(
        depth_view,
        "YOLO26 metric depth (visualized)",
        (10, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return np.hstack((view, depth_view))


def print_runtime_devices(segment_model: YOLO, depth_model: YOLO) -> None:
    """Report where the three learned components actually execute."""
    print(f"Segmentation predictor device: {segment_model.predictor.device}")
    print(f"Depth predictor device: {depth_model.predictor.device}")
    tracker = segment_model.predictor.trackers[0]
    encoder = getattr(tracker, "encoder", None)
    if REID_MODEL == "auto":
        print("ReID device: native segmentation backbone features on segmentation GPU")
    elif encoder is not None:
        backend = getattr(encoder, "model", None)
        session = getattr(backend, "session", None)
        providers = session.get_providers() if session is not None else None
        print(f"ReID requested device: {getattr(encoder, 'device', 'unknown')}; providers={providers}")
    print("BoT-SORT Kalman/association: CPU; LAP/OpenCV operations use compiled CPU kernels")


def main() -> None:
    print(f"Ultralytics segmentation: {SEGMENT_MODEL} on {SEGMENT_DEVICE}")
    print(f"Ultralytics metric depth: {DEPTH_MODEL} on {DEPTH_DEVICE}")
    print(f"Ultralytics BoT-SORT ReID: {REID_MODEL}")

    tracker_yaml = make_botsort_reid_yaml()
    segment_model = YOLO(SEGMENT_MODEL)
    depth_model = YOLO(DEPTH_MODEL)
    frame_stream = FrameStream(SOURCE)
    writer: cv2.VideoWriter | None = None
    last_depth: np.ndarray | None = None
    device_reported = False
    extractor: MeasurementExtractor | None = None
    selector: TargetSelector | None = None
    controller = FollowController()

    try:
        for frame_index, timestamp, frame in frame_stream:
            height, width = frame.shape[:2]
            if extractor is None:
                intrinsics = resolve_intrinsics(width, height)
                extrinsics = resolve_extrinsics()
                extractor = MeasurementExtractor(intrinsics, extrinsics)
                selector = TargetSelector(width)
                print(f"Intrinsics: {intrinsics}")
                if intrinsics.approximate:
                    print("WARNING: using approximate FOV intrinsics; complete CAMERA INTRINSICS TODO")
                if not extrinsics.calibrated_for_ground_plane:
                    print("WARNING: footpoint range disabled; complete CAMERA EXTRINSICS TODO")

            track_result = segment_model.track(
                source=frame,
                tracker=str(tracker_yaml),
                persist=True,
                classes=[PERSON_CLASS_ID],
                conf=DETECTION_CONFIDENCE,
                iou=DETECTION_IOU,
                imgsz=SEGMENT_IMGSZ,
                retina_masks=True,
                device=SEGMENT_DEVICE,
                quantize=INFERENCE_QUANTIZE,
                verbose=False,
            )[0]

            if frame_index % DEPTH_EVERY_N_FRAMES == 0 or last_depth is None:
                depth_result = depth_model.predict(
                    source=frame,
                    imgsz=DEPTH_IMGSZ,
                    device=DEPTH_DEVICE,
                    quantize=INFERENCE_QUANTIZE,
                    verbose=False,
                )[0]
                if depth_result.depth is None:
                    raise RuntimeError("Depth model returned no depth map")
                last_depth = depth_result.depth.data.detach().float().cpu().numpy()

            if not device_reported:
                print_runtime_devices(segment_model, depth_model)
                device_reported = True

            people = extractor(track_result, last_depth)
            target = selector.select(people)
            command = controller.update(target, extractor.intrinsics, timestamp)

            distance = "?" if target is None or target.distance_m is None else f"{target.distance_m:.2f}m"
            print(
                f"frame={frame_index:04d} people={len(people)} target={selector.target_id} "
                f"distance={distance} v={command.linear_velocity_mps:+.2f} "
                f"a={command.linear_acceleration_mps2:+.2f} "
                f"yaw={command.yaw_rate_radps:+.2f}"
            )

            output = annotate(track_result, last_depth, people, target, command)
            if SAVE_OUTPUT:
                if writer is None:
                    writer = cv2.VideoWriter(
                        str(OUTPUT_PATH),
                        cv2.VideoWriter_fourcc(*"mp4v"),
                        frame_stream.fps,
                        (output.shape[1], output.shape[0]),
                    )
                    if not writer.isOpened():
                        raise RuntimeError(f"Cannot create output video: {OUTPUT_PATH}")
                writer.write(output)

            if SHOW_WINDOW:
                cv2.imshow("person-follow demo", output)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        if writer is not None:
            writer.release()
        if SHOW_WINDOW:
            cv2.destroyAllWindows()
        tracker_yaml.unlink(missing_ok=True)

    if SAVE_OUTPUT:
        print(f"Saved visualization to: {OUTPUT_PATH.resolve()}")


if __name__ == "__main__":
    main()
