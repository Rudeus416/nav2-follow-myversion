"""Shared paths, timestamps, pairing, and radar-to-camera transform math."""

from __future__ import annotations

import bisect
import math
import re
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parent
CALIB_ROOT = ROOT / "data" / "data_calib"
TRANSFER_ROOT = ROOT / "data" / "data_transfer"
CAMERA_CALIBRATION = ROOT / "camera_calibration.yaml"
TOOLKIT_ROOT = Path("/home/metaiot/Workspace/dataCollectToolkit")
RADAR_PARAMS = TOOLKIT_ROOT / "config" / "radar" / "radar_params.yaml"

# Radar ROS frame: X forward, Y left, Z up. Camera optical frame: X right,
# Y down, Z forward. This known axis convention is separate from the unknown
# physical mounting transform adjusted by the operator.
RADAR_TO_OPTICAL_AXES = np.array(
    [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]], dtype=np.float64
)
NAME_RE = re.compile(r"^\w[\w-]{0,79}$", re.UNICODE)


def safe_name(value: str) -> str:
    if not NAME_RE.fullmatch(value):
        raise ValueError("名称只能包含中英文字符、数字、下划线和连字符，且不超过 80 字符")
    return value


def timestamp_name() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")


def ros_stamp_ns(stamp, received_ns: int) -> tuple[int, str]:
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return (value, "header") if value > 0 else (received_ns, "receive")


def pair_nearest(camera_frames: list[dict], radar_frames: list[dict],
                 threshold_ns: int) -> list[dict]:
    """Pair every radar frame with its nearest image; images may be reused."""
    if threshold_ns < 0:
        raise ValueError("配对阈值必须非负")
    cameras = sorted(camera_frames, key=lambda item: item["stamp_ns"])
    camera_times = [item["stamp_ns"] for item in cameras]
    pairs = []
    if not cameras:
        return pairs
    for radar in sorted(radar_frames, key=lambda item: item["stamp_ns"]):
        index = bisect.bisect_left(camera_times, radar["stamp_ns"])
        candidates = cameras[max(index - 1, 0):min(index + 1, len(cameras))]
        camera = min(candidates, key=lambda item: (
            abs(item["stamp_ns"] - radar["stamp_ns"]), item["stamp_ns"]
        ))
        delta_ns = int(radar["stamp_ns"]) - int(camera["stamp_ns"])
        if abs(delta_ns) <= threshold_ns:
            pairs.append({
                "camera": camera["file"],
                "radar": radar["file"],
                "camera_stamp_ns": int(camera["stamp_ns"]),
                "radar_stamp_ns": int(radar["stamp_ns"]),
                "delta_ns": delta_ns,
                "abs_delta_ms": round(abs(delta_ns) / 1_000_000, 3),
            })
    return pairs


def camera_intrinsics() -> dict:
    with CAMERA_CALIBRATION.open(encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    matrix = data["camera_matrix"]["data"]
    return {
        "width": int(data["image_width"]),
        "height": int(data["image_height"]),
        "fx": float(matrix[0]),
        "fy": float(matrix[4]),
        "cx": float(matrix[2]),
        "cy": float(matrix[5]),
        "distortion": [float(value) for value in data["distortion_coefficients"]["data"][:5]],
        "distortion_model": data["distortion_model"],
    }


def transform_matrix(yaw_deg: float, pitch_deg: float, roll_deg: float,
                     tx_m: float, ty_m: float, tz_m: float) -> np.ndarray:
    """P_optical = axes @ (Rz(yaw) @ Ry(pitch) @ Rx(roll) @ P_radar + t_radar)."""
    values = (yaw_deg, pitch_deg, roll_deg, tx_m, ty_m, tz_m)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("外参必须是有限数字")
    yaw, pitch, roll = np.deg2rad([yaw_deg, pitch_deg, roll_deg])
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = RADAR_TO_OPTICAL_AXES @ rz @ ry @ rx
    matrix[:3, 3] = RADAR_TO_OPTICAL_AXES @ [tx_m, ty_m, tz_m]
    return matrix
