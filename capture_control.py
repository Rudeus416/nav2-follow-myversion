"""Run one calibration capture at a time from the local web server."""

from __future__ import annotations

import io
import os
import signal
import subprocess
import sys
import threading
from collections import deque
from contextlib import redirect_stderr
from datetime import datetime, timezone
from pathlib import Path

from pydantic import BaseModel, ConfigDict

from calibration_common import CALIB_ROOT, ROOT, timestamp_name
from capture import parse_args


_DEFAULTS = parse_args([])
_ROS_SETUP = "/opt/ros/humble/setup.bash"
_DRIVER_SETUP = "/home/metaiot/Workspace/dataCollectToolkit/ros2_driver_ws/install/setup.bash"
_SENSOR_PYTHON = Path("/home/metaiot/miniconda3/envs/follow_demo/bin/python")


def capture_python() -> str:
    """Use the sensor environment even when the web server uses system Python."""
    configured = os.environ.get("VISUAL_CAR_CAPTURE_PYTHON")
    if configured:
        return configured
    return str(_SENSOR_PYTHON) if _SENSOR_PYTHON.is_file() else sys.executable


class CaptureInput(BaseModel):
    model_config = ConfigDict(extra="forbid", str_strip_whitespace=True)

    name: str | None = _DEFAULTS.name
    duration: float = _DEFAULTS.duration
    camera_fps: float = _DEFAULTS.camera_fps
    pair_threshold_ms: float = _DEFAULTS.pair_threshold_ms
    max_range_m: float = _DEFAULTS.max_range_m
    radar_frame_period_ms: float = _DEFAULTS.radar_frame_period_ms
    radar_profile: str = str(_DEFAULTS.radar_profile)
    min_range_m: float = _DEFAULTS.min_range_m
    cfar_range_db: float = _DEFAULTS.cfar_range_db
    cfar_doppler_db: float = _DEFAULTS.cfar_doppler_db
    cfar_peak_grouping: str = _DEFAULTS.cfar_peak_grouping
    external_camera: bool = _DEFAULTS.external_camera
    external_radar: bool = _DEFAULTS.external_radar
    camera_topic: str = _DEFAULTS.camera_topic
    radar_topic: str = _DEFAULTS.radar_topic
    cli_port: str = _DEFAULTS.cli_port
    data_port: str = _DEFAULTS.data_port
    startup_timeout: float = _DEFAULTS.startup_timeout
    jpeg_quality: int = _DEFAULTS.jpeg_quality
    queue_size: int = _DEFAULTS.queue_size


def capture_argv(data: CaptureInput) -> list[str]:
    """Build argv safely and validate it with capture.py's own parser."""
    argv: list[str] = []
    if data.name:
        argv.extend(("--name", data.name))
    for name in ("duration", "camera_fps", "pair_threshold_ms", "camera_topic",
                 "radar_topic", "startup_timeout", "jpeg_quality", "queue_size"):
        argv.extend((f"--{name.replace('_', '-')}", str(getattr(data, name))))
    if data.external_camera:
        argv.append("--external-camera")
    if data.external_radar:
        argv.append("--external-radar")
    else:
        for name in ("max_range_m", "radar_frame_period_ms", "radar_profile",
                     "min_range_m", "cfar_range_db", "cfar_doppler_db",
                     "cfar_peak_grouping", "cli_port", "data_port"):
            value = getattr(data, name)
            if name == "radar_profile" and not value:
                continue
            argv.extend((f"--{name.replace('_', '-')}", str(value)))
    for name in ("camera_topic", "radar_topic"):
        if not getattr(data, name).startswith("/"):
            raise ValueError(f"{name.replace('_', '-')} 必须是以 / 开头的 ROS 话题")
    for name in ("cli_port", "data_port"):
        if not data.external_radar and not getattr(data, name):
            raise ValueError(f"{name.replace('_', '-')} 不能为空")
    error_output = io.StringIO()
    try:
        with redirect_stderr(error_output):
            parse_args(argv)
    except SystemExit as exc:
        detail = error_output.getvalue().strip().splitlines()
        raise ValueError(detail[-1] if detail else f"采集参数无效：{exc.code}") from exc
    if data.name and (CALIB_ROOT / data.name).exists():
        raise FileExistsError(f"采集目录已存在：{data.name}")
    return argv


class CaptureManager:
    def __init__(self):
        self._lock = threading.Lock()
        self._job: dict | None = None

    @staticmethod
    def _launch(argv: list[str]) -> subprocess.Popen:
        script = 'set -e; source "$1"; source "$2"; shift 2; exec "$@"'
        command = ["/bin/bash", "-c", script, "capture-web", _ROS_SETUP,
                   _DRIVER_SETUP, capture_python(),
                   "-B", str(ROOT / "capture.py"), *argv]
        environment = os.environ.copy()
        environment["PYTHONUNBUFFERED"] = "1"
        return subprocess.Popen(command, cwd=ROOT, env=environment,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, errors="replace", bufsize=1,
                                start_new_session=True)

    def _snapshot(self) -> dict:
        job = self._job
        if job is None:
            return {"run_id": None, "state": "idle", "running": False,
                    "session_name": None, "exit_code": None, "output": []}
        return {
            "run_id": job["run_id"],
            "state": job["state"],
            "running": job["process"].poll() is None,
            "session_name": job["session_name"],
            "started_at_utc": job["started_at_utc"],
            "finished_at_utc": job["finished_at_utc"],
            "exit_code": job["exit_code"],
            "output": list(job["output"]),
        }

    def status(self) -> dict:
        with self._lock:
            return self._snapshot()

    def _read_output(self, job: dict) -> None:
        process = job["process"]
        try:
            for line in process.stdout:
                line = line.rstrip("\r\n")[:4000]
                with self._lock:
                    job["output"].append(line)
                    if line.startswith("开始采集 ") and "：" in line:
                        job["session_name"] = Path(line.rsplit("：", 1)[1].strip()).name
        finally:
            code = process.wait()
            with self._lock:
                job["exit_code"] = code
                job["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
                job["state"] = ("failed" if code else
                                "stopped" if job["stop_requested"] else "completed")
            process.stdout.close()

    def start(self, data: CaptureInput) -> dict:
        argv = capture_argv(data)
        with self._lock:
            if self._job and self._job["process"].poll() is None:
                raise RuntimeError("已有采集任务正在运行")
            process = self._launch(argv)
            job = {
                "run_id": timestamp_name(), "process": process,
                "session_name": data.name or None,
                "started_at_utc": datetime.now(timezone.utc).isoformat(),
                "finished_at_utc": None, "exit_code": None,
                "state": "running", "stop_requested": False,
                "output": deque(maxlen=200),
            }
            self._job = job
            snapshot = self._snapshot()
        threading.Thread(target=self._read_output, args=(job,), daemon=True).start()
        return snapshot

    def stop(self) -> dict:
        with self._lock:
            if not self._job or self._job["process"].poll() is not None:
                raise RuntimeError("当前没有运行中的采集任务")
            job = self._job
            job["stop_requested"] = True
            job["state"] = "stopping"
            try:
                os.killpg(job["process"].pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            return self._snapshot()

    def shutdown(self) -> None:
        with self._lock:
            process = self._job["process"] if self._job else None
        if process and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass


CAPTURE_MANAGER = CaptureManager()
