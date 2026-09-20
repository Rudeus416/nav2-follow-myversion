#!/usr/bin/env python3
"""Local web API and frontend for manual camera/mmWave extrinsic calibration."""

from __future__ import annotations

import argparse
import json
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Literal
from urllib.parse import quote

import matplotlib
import numpy as np
import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, HTMLResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from calibration_common import (CALIB_ROOT, CAMERA_CALIBRATION, RADAR_TO_OPTICAL_AXES,
                                TRANSFER_ROOT, camera_intrinsics, safe_name,
                                timestamp_name, transform_matrix)
from capture_control import CAPTURE_MANAGER, CaptureInput


FRONTEND = Path(__file__).resolve().parent / "calibration_frontend"
app = FastAPI(title="毫米波雷达—相机人工外参标定")
app.mount("/static", StaticFiles(directory=FRONTEND), name="static")
app.add_event_handler("shutdown", CAPTURE_MANAGER.shutdown)
_save_lock = threading.Lock()


def _safe(value: str) -> str:
    try:
        return safe_name(value)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


def _session(name: str) -> Path:
    path = CALIB_ROOT / _safe(name)
    if not path.is_dir() or not (path / "manifest.json").is_file():
        raise HTTPException(status_code=404, detail="采集批次不存在")
    return path


def _pair(session: Path, pair_id: str) -> dict:
    if not pair_id.isdigit():
        raise HTTPException(status_code=400, detail="配对编号无效")
    path = session / "pairs" / f"{pair_id}.json"
    if not path.is_file():
        raise HTTPException(status_code=404, detail="配对不存在")
    return json.loads(path.read_text(encoding="utf-8"))


def _transform_file(name: str) -> Path:
    if not name.endswith(".json"):
        raise HTTPException(status_code=400, detail="参数文件名必须以 .json 结尾")
    return TRANSFER_ROOT / f"{_safe(name[:-5])}.json"


class TransformInput(BaseModel):
    parameter_frame: Literal["radar_ros"]
    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    tx_m: float = 0.0
    ty_m: float = 0.0
    tz_m: float = 0.0
    session: str | None = None
    pair_id: str | None = None


def _record(data: TransformInput, name: str, created_at: str | None = None) -> dict:
    try:
        matrix = transform_matrix(
            data.yaw_deg, data.pitch_deg, data.roll_deg,
            data.tx_m, data.ty_m, data.tz_m,
        )
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc
    if data.session:
        _safe(data.session)
    if data.pair_id and not data.pair_id.isdigit():
        raise HTTPException(status_code=422, detail="配对编号无效")
    now = datetime.now(timezone.utc).isoformat()
    return {
        "schema_version": 2,
        "name": name,
        "created_at_utc": created_at or now,
        "updated_at_utc": now,
        "source_session": data.session,
        "source_pair_id": data.pair_id,
        "camera_calibration_file": str(CAMERA_CALIBRATION),
        "camera_intrinsics": camera_intrinsics(),
        "parameter_frame": "radar_ros",
        "rotation_order": "Rz(yaw_deg) @ Ry(pitch_deg) @ Rx(roll_deg)",
        "convention": (
            "P_camera_optical = radar_to_optical_axes @ "
            "(Rz(yaw_deg) @ Ry(pitch_deg) @ Rx(roll_deg) "
            "@ P_radar_ros + [tx_m,ty_m,tz_m]_radar_ros)"
        ),
        "adjustment_euler_deg": {
            "yaw": data.yaw_deg, "pitch": data.pitch_deg, "roll": data.roll_deg,
        },
        "translation_m": {"x": data.tx_m, "y": data.ty_m, "z": data.tz_m},
        "translation_camera_optical_m": {
            "x": float(matrix[0, 3]), "y": float(matrix[1, 3]),
            "z": float(matrix[2, 3]),
        },
        "radar_to_optical_axes": RADAR_TO_OPTICAL_AXES.tolist(),
        "radar_to_camera_optical_4x4": matrix.tolist(),
    }


def _atomic_json(path: Path, payload: dict) -> None:
    temporary = path.with_name(f".{path.name}.tmp")
    try:
        temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                             encoding="utf-8")
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


@app.get("/")
def index():
    revision = max((FRONTEND / name).stat().st_mtime_ns
                   for name in ("index.html", "style.css", "app.js"))
    html = (FRONTEND / "index.html").read_text(encoding="utf-8")
    return HTMLResponse(html.replace("radar-axes-1", str(revision)),
                        headers={"Cache-Control": "no-store"})


@app.get("/api/calibration")
def calibration():
    return {
        "intrinsics": camera_intrinsics(),
        "radar_to_optical_axes": RADAR_TO_OPTICAL_AXES.tolist(),
        "parameter_frame": "radar_ros",
    }


@app.get("/api/colormap")
def colormap():
    cmap = matplotlib.colormaps["magma"]
    colors = []
    for value in np.linspace(0, 1, 256):
        rgb = cmap(float(value))[:3]
        colors.append("#" + "".join(f"{round(channel * 255):02x}" for channel in rgb))
    return {"name": "matplotlib.magma", "colors": colors}


@app.get("/api/capture/defaults")
def capture_defaults():
    return CaptureInput().model_dump()


@app.get("/api/capture")
def capture_status():
    return CAPTURE_MANAGER.status()


@app.post("/api/capture")
def start_capture(data: CaptureInput):
    try:
        return CAPTURE_MANAGER.start(data)
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc
    except (FileExistsError, RuntimeError) as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    except OSError as exc:
        raise HTTPException(status_code=500, detail=f"无法启动采集进程：{exc}") from exc


@app.post("/api/capture/stop")
def stop_capture():
    try:
        return CAPTURE_MANAGER.stop()
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.get("/api/sessions")
def sessions():
    if not CALIB_ROOT.exists():
        return []
    output = []
    for path in sorted(CALIB_ROOT.iterdir(), reverse=True):
        manifest = path / "manifest.json"
        if not path.is_dir() or not manifest.is_file():
            continue
        try:
            data = json.loads(manifest.read_text(encoding="utf-8"))
            output.append({
                "name": path.name,
                "pair_count": int(data.get("pair_count", 0)),
                "camera_count": len(data.get("camera_frames", [])),
                "radar_count": len(data.get("radar_frames", [])),
                "started_wall_ns": data.get("started_wall_ns", 0),
            })
        except (OSError, ValueError, TypeError):
            continue
    return output


@app.get("/api/sessions/{session_name}/pairs")
def pairs(session_name: str):
    session = _session(session_name)
    output = []
    for path in sorted((session / "pairs").glob("*.json")):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        output.append({"id": path.stem, **data})
    return output


@app.get("/api/sessions/{session_name}/pairs/{pair_id}")
def pair_detail(session_name: str, pair_id: str):
    session = _session(session_name)
    pair = _pair(session, pair_id)
    image_name = Path(pair["camera"]).name
    radar_name = Path(pair["radar"]).name
    image = session / "camera" / image_name
    radar = session / "radar" / radar_name
    if not image.is_file() or not radar.is_file():
        raise HTTPException(status_code=404, detail="配对引用的数据文件缺失")
    with np.load(radar, allow_pickle=False) as archive:
        points = np.asarray(archive["points"], dtype=np.float32)
        fields = [str(item) for item in archive["fields"]]
    return {
        **pair,
        "id": pair_id,
        "session": session_name,
        "image_url": f"/api/sessions/{quote(session_name)}/camera/{quote(image_name)}",
        "points": points.tolist(),
        "fields": fields,
    }


@app.get("/api/sessions/{session_name}/camera/{image_name}")
def camera_image(session_name: str, image_name: str):
    session = _session(session_name)
    if Path(image_name).name != image_name or not image_name.endswith(".jpg"):
        raise HTTPException(status_code=400, detail="图像文件名无效")
    image = session / "camera" / image_name
    if not image.is_file():
        raise HTTPException(status_code=404, detail="图像不存在")
    return FileResponse(image, media_type="image/jpeg")


@app.get("/api/transforms")
def transforms():
    if not TRANSFER_ROOT.exists():
        return []
    return [path.name for path in sorted(TRANSFER_ROOT.glob("*.json"), reverse=True)]


@app.get("/api/transforms/{name}")
def get_transform(name: str):
    path = _transform_file(name)
    if not path.is_file():
        raise HTTPException(status_code=404, detail="参数文件不存在")
    return json.loads(path.read_text(encoding="utf-8"))


@app.post("/api/transforms")
def create_transform(data: TransformInput):
    TRANSFER_ROOT.mkdir(parents=True, exist_ok=True)
    with _save_lock:
        name = timestamp_name() + ".json"
        while (TRANSFER_ROOT / name).exists():
            name = timestamp_name() + ".json"
        record = _record(data, name)
        _atomic_json(TRANSFER_ROOT / name, record)
    return record


@app.put("/api/transforms/{name}")
def update_transform(name: str, data: TransformInput):
    path = _transform_file(name)
    with _save_lock:
        if not path.is_file():
            raise HTTPException(status_code=404, detail="参数文件不存在")
        old = json.loads(path.read_text(encoding="utf-8"))
        record = _record(data, path.name, old.get("created_at_utc"))
        _atomic_json(path, record)
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8891)
    args = parser.parse_args()
    uvicorn.run(app, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
