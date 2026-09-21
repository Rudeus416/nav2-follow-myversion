"""Live mmWave acquisition, camera projection, and distance scale estimation."""

from __future__ import annotations

import math
import os
import site
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from pydantic import BaseModel, ConfigDict, Field, model_validator

from calibration_common import ROOT, TOOLKIT_ROOT, TRANSFER_ROOT, ros_stamp_ns, transform_matrix
from capture import (RADAR_TOPIC, check_radar_network, decode_radar_points,
                     prepare_radar_config, start_launch, stop_launch)


RADAR_PROFILE_3TX = TOOLKIT_ROOT / "config/radar/xWR1843_profile_3tx4rx_20hz.cfg"
RADAR_RUNTIME = ROOT / "data/runtime/radar"


class RadarSettings(BaseModel):
    model_config = ConfigDict(allow_inf_nan=False)

    enabled: bool = False
    external_radar: bool = False
    radar_topic: str = RADAR_TOPIC
    cli_port: str = "/dev/ttyACM0"
    data_port: str = "/dev/ttyACM1"
    max_range_m: float = Field(default=5.0, gt=0, le=30)
    min_range_m: float = Field(default=0.0, ge=0, lt=30)
    frame_period_ms: float = Field(default=200.0, ge=50, le=1000)
    cfar_range_db: float = Field(default=10.0, gt=0, le=30)
    cfar_doppler_db: float = Field(default=10.0, gt=0, le=30)
    cfar_peak_grouping: str = Field(default="none", pattern="^(both|range|doppler|none)$")
    # These are the six radar-ROS-frame values saved by the calibration page.
    transform_file: str | None = None
    allow_extrinsic_adjustment: bool = False
    yaw_deg: float = Field(default=0.0, ge=-180, le=180)
    pitch_deg: float = Field(default=0.0, ge=-180, le=180)
    roll_deg: float = Field(default=0.0, ge=-180, le=180)
    tx_m: float = Field(default=-0.065, ge=-5, le=5)
    ty_m: float = Field(default=0.0, ge=-5, le=5)
    tz_m: float = Field(default=0.0, ge=-5, le=5)

    @model_validator(mode="after")
    def validate_range(self):
        if self.min_range_m >= self.max_range_m:
            raise ValueError("雷达最小距离必须小于最大距离")
        if not self.radar_topic.startswith("/"):
            raise ValueError("雷达话题必须是绝对 ROS 话题名")
        return self

    def extrinsics(self) -> tuple[float, ...]:
        return (self.yaw_deg, self.pitch_deg, self.roll_deg,
                self.tx_m, self.ty_m, self.tz_m)

    def hardware(self) -> tuple[object, ...]:
        return (self.external_radar, self.radar_topic, self.cli_port, self.data_port,
                self.max_range_m, self.min_range_m, self.frame_period_ms,
                self.cfar_range_db, self.cfar_doppler_db, self.cfar_peak_grouping)


def transform_names() -> list[str]:
    return sorted((p.name for p in TRANSFER_ROOT.glob("*.json") if p.is_file()), reverse=True)


def load_transform(name: str) -> dict[str, float | str]:
    import json

    if Path(name).name != name or not name.endswith(".json"):
        raise ValueError("外参文件名无效")
    path = TRANSFER_ROOT / name
    if not path.is_file():
        raise ValueError("外参文件不存在")
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema_version") not in (1, 2):
        raise ValueError("无法识别外参文件版本")
    angles, translation = data["adjustment_euler_deg"], data["translation_m"]
    if data["schema_version"] == 2 and data.get("parameter_frame") == "radar_ros":
        values = dict(transform_file=name, yaw_deg=angles["yaw"], pitch_deg=angles["pitch"],
                      roll_deg=angles["roll"], tx_m=translation["x"],
                      ty_m=translation["y"], tz_m=translation["z"])
    elif data["schema_version"] == 1:
        # Same V1 camera-optical -> V2 radar-axis conversion as calibration UI.
        values = dict(transform_file=name, yaw_deg=-angles["yaw"], pitch_deg=-angles["pitch"],
                      roll_deg=angles["roll"], tx_m=translation["z"],
                      ty_m=-translation["x"], tz_m=-translation["y"])
    else:
        raise ValueError("外参文件的坐标系无效")
    RadarSettings.model_validate(values)
    return values


@dataclass(frozen=True)
class RadarFrame:
    stamp_ns: int
    received_at: float
    points: np.ndarray


def project_points(points: np.ndarray, matrix: np.ndarray, k: np.ndarray,
                   distortion: np.ndarray, width: int, height: int) -> np.ndarray:
    """Return [u, v, horizontal camera range] for visible radar detections."""
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    xyz = np.asarray(points[:, :3], dtype=np.float64)
    optical = xyz @ matrix[:3, :3].T + matrix[:3, 3]
    valid = np.isfinite(optical).all(axis=1) & (optical[:, 2] > 0.05)
    optical = optical[valid]
    if not len(optical):
        return np.empty((0, 3), dtype=np.float64)
    pixels, _ = cv2.projectPoints(optical, np.zeros(3), np.zeros(3), k, distortion)
    pixels = pixels.reshape(-1, 2)
    ranges = np.hypot(optical[:, 0], optical[:, 2])
    inside = (np.isfinite(pixels).all(axis=1) & (pixels[:, 0] >= 0)
              & (pixels[:, 0] < width) & (pixels[:, 1] >= 0)
              & (pixels[:, 1] < height) & np.isfinite(ranges))
    return np.column_stack((pixels[inside], ranges[inside]))


def person_radar_range(projected: np.ndarray, box: np.ndarray,
                       mask: np.ndarray | None, image_size: tuple[int, int]
                       ) -> tuple[float | None, int]:
    """Use box points, preferring segmentation pixels and a tight range cluster.

    The output is the mean of the selected inliers. A lone detection is usable,
    but multiple detections must agree within a range cluster to reject clutter.
    """
    if not len(projected):
        return None, 0
    x1, y1, x2, y2 = box
    in_box = ((projected[:, 0] >= x1) & (projected[:, 0] <= x2)
              & (projected[:, 1] >= y1) & (projected[:, 1] <= y2))
    candidates = projected[in_box]
    if not len(candidates):
        return None, 0
    if mask is not None and mask.size:
        image_w, image_h = image_size
        gain = min(mask.shape[1] / image_w, mask.shape[0] / image_h)
        pad_x = (mask.shape[1] - image_w * gain) / 2.0
        pad_y = (mask.shape[0] - image_h * gain) / 2.0
        mx = np.clip(np.rint(candidates[:, 0] * gain + pad_x).astype(int), 0, mask.shape[1] - 1)
        my = np.clip(np.rint(candidates[:, 1] * gain + pad_y).astype(int), 0, mask.shape[0] - 1)
        # A small dilation tolerates residual calibration and mask quantization error.
        expanded = cv2.dilate(mask.astype(np.uint8), np.ones((5, 5), np.uint8))
        on_person = expanded[my, mx].astype(bool)
        if on_person.any():
            candidates = candidates[on_person]
    distances = np.sort(candidates[:, 2])
    if len(distances) == 2 and distances[1] - distances[0] > 0.6:
        # Conflicting isolated returns: the nearer one avoids commanding a
        # dangerous forward motion from a possible background reflection.
        distances = distances[:1]
    if len(distances) >= 3:
        # Find the densest 0.6 m window; on ties prefer the nearer cluster.
        width = 0.6
        counts = np.searchsorted(distances, distances + width, side="right") - np.arange(len(distances))
        start = int(np.flatnonzero(counts == counts.max())[0])
        distances = distances[start:start + counts[start]]
    return float(np.mean(distances)), int(len(distances))


class ScaleKalman:
    """One-dimensional Kalman estimate of radar range / visual range."""

    def __init__(self):
        self.value = 1.0
        self.variance = 0.25
        self.samples = 0

    def reset(self):
        self.__init__()

    def update(self, radar_range: float, visual_range: float, point_count: int) -> float:
        if not (math.isfinite(radar_range) and math.isfinite(visual_range)
                and 0.1 <= radar_range <= 30 and 0.1 <= visual_range <= 30):
            return self.value
        observation = radar_range / visual_range
        if not 0.1 <= observation <= 10:
            return self.value
        if self.samples >= 3 and abs(observation - self.value) > max(0.75, 0.5 * self.value):
            return self.value
        self.variance = min(2.0, self.variance + 0.002)
        # Sparse single-point returns are less trustworthy than a coherent
        # cluster, so let them move the scale more slowly.
        measurement_variance = 0.16 / max(1, min(point_count, 8)) + 0.04
        gain = self.variance / (self.variance + measurement_variance)
        self.value = min(10.0, max(0.1, self.value + gain * (observation - self.value)))
        self.variance *= 1 - gain
        self.samples += 1
        return self.value


class RadarManager:
    def __init__(self, node, settings: RadarSettings):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs_py import point_cloud2

        self.node = node
        self.point_cloud2 = point_cloud2
        self.message_type = PointCloud2
        self.qos = qos_profile_sensor_data
        self.lock = threading.RLock()
        self.settings = settings
        self.frames: deque[RadarFrame] = deque(maxlen=128)
        self.subscription = None
        self.process = None
        self.handle = None
        self.error = ""
        self.received = 0
        self.last_received_at = 0.0
        if settings.enabled:
            self.configure(settings)

    def _on_points(self, msg):
        try:
            points = decode_radar_points(msg, self.point_cloud2)
            stamp_ns, _ = ros_stamp_ns(msg.header.stamp, time.time_ns())
            with self.lock:
                if not self.settings.enabled:
                    return
                self.frames.append(RadarFrame(stamp_ns, time.monotonic(), points))
                self.received += 1
                self.last_received_at = time.monotonic()
                self.error = ""
        except Exception as exc:
            with self.lock:
                self.error = str(exc)

    def _stop(self):
        if self.subscription is not None:
            self.node.destroy_subscription(self.subscription)
            self.subscription = None
        if self.process is not None:
            stop_launch(self.process, self.handle)
            self.process = self.handle = None
        self.frames.clear()

    def needs_recovery(self) -> bool:
        with self.lock:
            return (self.settings.enabled and
                    (self.subscription is None or
                     (not self.settings.external_radar and
                      (self.process is None or self.process.poll() is not None))))

    def configure(self, settings: RadarSettings, from_saved_config: bool = False):
        with self.lock:
            previous = self.settings
            if (settings.extrinsics() != previous.extrinsics()
                    and not settings.allow_extrinsic_adjustment and not from_saved_config):
                file_values = (load_transform(settings.transform_file)
                               if settings.transform_file else None)
                file_extrinsics = (tuple(file_values[key] for key in
                                   ("yaw_deg", "pitch_deg", "roll_deg", "tx_m", "ty_m", "tz_m"))
                                   if file_values else None)
                if settings.extrinsics() != file_extrinsics:
                    raise ValueError("请先勾选“允许调整外参”，再修改六个外参值")
            driver_dead = (settings.enabled and not settings.external_radar and
                           (self.process is None or self.process.poll() is not None))
            if (settings == previous and (not settings.enabled or
                    (self.subscription is not None and not driver_dead))):
                return settings
            restart = (settings.enabled != previous.enabled
                       or settings.hardware() != previous.hardware()
                       or (settings.enabled and (self.subscription is None or driver_dead)))
            if restart:
                self._stop()
            if settings.enabled and restart:
                try:
                    if not settings.external_radar:
                        if self.node.count_publishers(settings.radar_topic):
                            raise ValueError("雷达话题已有发布者；选择“仅订阅外部雷达”")
                        if not Path(settings.cli_port).exists() or not Path(settings.data_port).exists():
                            raise ValueError("雷达命令口或数据口不存在")
                        check_radar_network()
                        RADAR_RUNTIME.mkdir(parents=True, exist_ok=True)
                        _, params, _ = prepare_radar_config(
                            RADAR_RUNTIME, settings.max_range_m, settings.cli_port,
                            settings.data_port, settings.frame_period_ms,
                            RADAR_PROFILE_3TX, settings.cfar_range_db,
                            settings.cfar_doppler_db, settings.cfar_peak_grouping,
                            settings.min_range_m,
                        )
                        radar_env = os.environ.copy()
                        numpy_site = str(Path(np.__file__).resolve().parent.parent)
                        radar_env["PYTHONPATH"] = os.pathsep.join(filter(None, (
                            numpy_site, site.getusersitepackages(), radar_env.get("PYTHONPATH", ""))))
                        check = subprocess.run(
                            ["/usr/bin/python3", "-c",
                             "from ti_mmwave_driver.mmwave.dataloader import DCA1000"],
                            cwd=TOOLKIT_ROOT, env=radar_env, capture_output=True, text=True,
                        )
                        if check.returncode:
                            detail = check.stderr.strip().splitlines()[-1] if check.stderr.strip() else "未知导入错误"
                            raise RuntimeError(f"雷达 Python 依赖检查失败：{detail}")
                        self.process, self.handle = start_launch(
                            ["ros2", "launch", "ti_mmwave_driver",
                             "ti_mmwave_driver_all.launch.py", f"params_file:={params}"],
                            RADAR_RUNTIME / "radar.log", radar_env,
                        )
                    self.subscription = self.node.create_subscription(
                        self.message_type, settings.radar_topic, self._on_points, self.qos,
                    )
                except Exception:
                    self._stop()
                    self.settings = previous.model_copy(update={"enabled": False})
                    raise
            self.settings = settings
            self.error = ""
            return settings

    def nearest(self, stamp_ns: int) -> RadarFrame | None:
        with self.lock:
            if not self.settings.enabled or not self.frames:
                return None
            if self.process is not None and self.process.poll() is not None:
                return None
            if not self.node.count_publishers(self.settings.radar_topic):
                return None
            return min(self.frames, key=lambda frame: (abs(frame.stamp_ns - stamp_ns), frame.stamp_ns))

    def status(self) -> dict:
        with self.lock:
            running = self.process is not None and self.process.poll() is None
            if self.process is not None and not running:
                self.error = f"雷达驱动已退出，退出码 {self.process.returncode}；请查看 data/runtime/radar/radar.log"
            return {
                "radar_settings": self.settings.model_dump(),
                "radar_driver_running": running,
                "radar_received_total": self.received,
                "radar_buffer_size": len(self.frames),
                "radar_age_seconds": round(time.monotonic() - self.last_received_at, 3)
                if self.last_received_at else None,
                "radar_error": self.error,
                "radar_to_camera_optical_4x4": transform_matrix(
                    *self.settings.extrinsics()).tolist(),
            }

    def close(self):
        with self.lock:
            self._stop()
