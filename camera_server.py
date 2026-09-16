#!/usr/bin/env python3
"""Hikvision ROS camera capture and local live-view server.

Run after sourcing ROS Humble and the dataCollectToolkit driver workspace.
"""

import argparse
import os
import queue
import re
import shutil
import signal
import subprocess
import tempfile
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field
from rcl_interfaces.srv import SetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


ROOT = Path(__file__).resolve().parent
FRAME_NAME = re.compile(r"frame_[0-9]{19}\.jpg")
MIN_FREE_BYTES = 50 * 1024 * 1024


class FrameStore:
    """Retain recent JPEGs in RAM and optionally persist the same bytes."""

    def __init__(self, directory: Path, max_frames: int, on_saved=None):
        self.directory = directory
        self.max_frames = max_frames
        self.on_saved = on_saved
        self.directory.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._memory: dict[str, bytes] = {}
        self._names = sorted(
            path.name for path in self.directory.iterdir()
            if path.is_file() and FRAME_NAME.fullmatch(path.name)
        )
        self._prune()
        self._queue: queue.Queue[tuple[bytes, bool] | None] = queue.Queue(maxsize=2)
        self.saved_total = 0
        self.persisted_total = 0
        self.dropped_total = 0
        self.write_error = ""
        self._thread = threading.Thread(target=self._write_loop, daemon=True)
        self._thread.start()

    def _prune(self, remove_disk: bool = True):
        while len(self._names) > self.max_frames:
            old_name = self._names.pop(0)
            self._memory.pop(old_name, None)
            if remove_disk:
                (self.directory / old_name).unlink(missing_ok=True)

    def submit(self, jpeg: bytes, persist: bool = True):
        if persist:
            try:
                free_bytes = shutil.disk_usage(self.directory).free
            except OSError as exc:
                free_bytes = 0
                warning = str(exc)
            else:
                warning = f"Disk space below 50 MB; keeping images in RAM only ({free_bytes // (1024 * 1024)} MB free)"
            if free_bytes < MIN_FREE_BYTES:
                persist = False
                with self._lock:
                    self.dropped_total += 1
                    self.write_error = warning
        try:
            self._queue.put_nowait((jpeg, persist))
        except queue.Full:
            with self._lock:
                self.dropped_total += 1

    def _write_loop(self):
        while True:
            item = self._queue.get()
            if item is None:
                self._queue.task_done()
                return
            jpeg, persist = item
            filename = f"frame_{time.time_ns():019d}.jpg"
            target = self.directory / filename
            temporary = self.directory / f".{filename}.tmp"
            disk_error = ""
            try:
                if persist:
                    temporary.write_bytes(jpeg)
                    os.replace(temporary, target)
                with self._lock:
                    self._memory[filename] = jpeg
                    self._names.append(filename)
                    self._names.sort()
                    # RAM-only operation must not delete files from an older
                    # persisted session merely because they leave this
                    # process's recent-image index.
                    self._prune(remove_disk=persist)
                    self.saved_total += 1
                    if persist:
                        self.persisted_total += 1
                        self.write_error = ""
                if persist and self.on_saved is not None:
                    self.on_saved(target)
            except OSError as exc:
                disk_error = str(exc)
                with self._lock:
                    # The encoded image remains useful to the web page even if
                    # optional disk persistence fails.
                    self._memory[filename] = jpeg
                    self._names.append(filename)
                    self._names.sort()
                    self._prune(remove_disk=False)
                    self.saved_total += 1
                    self.dropped_total += 1
                    self.write_error = disk_error
            finally:
                temporary.unlink(missing_ok=True)
                self._queue.task_done()

    def names(self):
        with self._lock:
            return list(reversed(self._names))

    def count(self):
        with self._lock:
            return len(self._names)

    def get(self, filename: str) -> bytes | None:
        with self._lock:
            jpeg = self._memory.get(filename)
        if jpeg is not None:
            return jpeg
        path = self.directory / filename
        try:
            return path.read_bytes()
        except (OSError, FileNotFoundError):
            return None

    def close(self):
        self._queue.join()
        self._queue.put(None)
        self._thread.join(timeout=5)


class CaptureSettings(BaseModel):
    fps: float = Field(default=5.0, gt=0, le=60)
    duration: float = Field(default=0.0, ge=0, description="Seconds; 0 means unlimited")
    persist_images: bool = False


class CameraSettings(BaseModel):
    exposure_us: int = Field(default=20000, ge=32, le=999812)
    gain: float = Field(default=2.0, ge=0, le=23)


class CameraBridge:
    def __init__(self, settings: CaptureSettings, preview_fps: float, store: FrameStore):
        self.settings = settings
        self.preview_fps = preview_fps
        self.store = store
        self._condition = threading.Condition()
        self._latest: bytes | None = None
        self._sequence = 0
        self._last_encode = 0.0
        self._last_save = 0.0
        self._last_image = 0.0
        self._capture_started: float | None = None
        self._saving = True
        self.received_total = 0
        self.encoded_total = 0
        self.error = ""
        # Optional real-time consumer.  The callback receives an owned BGR
        # array before JPEG encoding/writing, so inference is not coupled to
        # the archival path.
        self.on_frame = None

    def on_image(self, image: Image):
        now = time.monotonic()
        with self._condition:
            self.received_total += 1
            self._last_image = now
            if self._saving and self._capture_started is None:
                self._capture_started = now
            if self._saving and self.settings.duration > 0 and self._capture_started is not None:
                if now - self._capture_started >= self.settings.duration:
                    self._saving = False
            preview_due = now - self._last_encode >= 1.0 / self.preview_fps
            save_due = self._saving and now - self._last_save >= 1.0 / self.settings.fps
            if not (preview_due or save_due):
                return
            if preview_due:
                self._last_encode = now
            if save_due:
                self._last_save = now

        try:
            if image.encoding != "rgb8":
                raise ValueError(f"Expected rgb8, got {image.encoding}")
            if image.step < image.width * 3:
                raise ValueError("Invalid image row stride")
            rgb_rows = np.frombuffer(image.data, dtype=np.uint8).reshape(
                image.height, image.step
            )
            rgb = rgb_rows[:, : image.width * 3].reshape(image.height, image.width, 3)
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            if save_due and self.on_frame is not None:
                self.on_frame(bgr, now)
            ok, buffer = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                raise ValueError("JPEG encoding failed")
            jpeg = buffer.tobytes()
            with self._condition:
                self._latest = jpeg
                self._sequence += 1
                self.encoded_total += 1
                self.error = ""
                self._condition.notify_all()
            if save_due:
                self.store.submit(jpeg, self.settings.persist_images)
        except Exception as exc:
            with self._condition:
                self.error = str(exc)

    def start_capture(self, settings: CaptureSettings):
        with self._condition:
            self.settings = settings
            self._capture_started = None
            self._last_save = 0.0
            self._saving = True

    def set_persistence(self, enabled: bool):
        with self._condition:
            self.settings = self.settings.model_copy(update={"persist_images": enabled})
            return self.settings

    def stop_capture(self):
        with self._condition:
            self._saving = False

    def clear_live(self):
        with self._condition:
            self._latest = None
            self._last_image = 0.0
            self._condition.notify_all()

    def status(self):
        with self._condition:
            if self._saving and self.settings.duration > 0 and self._capture_started is not None:
                if time.monotonic() - self._capture_started >= self.settings.duration:
                    self._saving = False
            result = {
                "saving": self._saving,
                "save_fps": self.settings.fps,
                "duration_seconds": self.settings.duration,
                "persist_images": self.settings.persist_images,
                "preview_fps_limit": self.preview_fps,
                "received_total": self.received_total,
                "encoded_total": self.encoded_total,
                "last_image_age_seconds": round(time.monotonic() - self._last_image, 1)
                if self._last_image else None,
                "error": self.error,
            }
        with self.store._lock:
            result.update({
                "saved_total": self.store.saved_total,
                "persisted_total": self.store.persisted_total,
                "dropped_saves": self.store.dropped_total,
                "retained_frames": len(self.store._names),
                "write_error": self.store.write_error,
                "disk_free_mb": shutil.disk_usage(self.store.directory).free // (1024 * 1024),
            })
        return result

    def latest(self):
        with self._condition:
            return self._latest

    def image_recent(self):
        with self._condition:
            return self._last_image > 0 and time.monotonic() - self._last_image < 5

    def stream(self):
        sequence = -1
        while True:
            with self._condition:
                self._condition.wait_for(
                    lambda: self._latest is not None and self._sequence != sequence,
                    timeout=20,
                )
                jpeg = self._latest
                sequence = self._sequence
            if jpeg is not None:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                       + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")


class CameraController:
    """Control only a ROS launch process started by this service."""

    def __init__(self, node, bridge: CameraBridge, no_launch: bool):
        self.node = node
        self.bridge = bridge
        self.no_launch = no_launch
        self.settings = CameraSettings(exposure_us=40000, gain=4.0)
        self.process = None
        self._params_file = None
        self._lock = threading.RLock()
        self._parameter_client = node.create_client(
            SetParameters, "/hk_camera/hik_camera/set_parameters"
        )

    def _camera_node_visible(self):
        try:
            return any(
                name == "hik_camera" and namespace == "/hk_camera"
                for name, namespace in self.node.get_node_names_and_namespaces()
            )
        except Exception:
            return False

    def state(self):
        with self._lock:
            managed = self.process is not None and self.process.poll() is None
        external = not managed and (
            self._camera_node_visible() or self.bridge.image_recent()
        )
        mode = "managed" if managed else "external" if external else "stopped"
        return {
            "device_mode": mode,
            "camera_process_running": managed or external,
            "image_stream_active": self.bridge.image_recent(),
            "device_control_available": not self.no_launch and not external,
            "external_launch_requested": self.no_launch,
        }

    def start(self, initial=False):
        if self.no_launch:
            if initial:
                return self.state()
            raise HTTPException(status_code=409, detail="Camera was launched outside this service; restart without --no-launch to control it here")
        with self._lock:
            if self.process is not None and self.process.poll() is None:
                return self.state()
            if self._camera_node_visible() or self.bridge.image_recent():
                if initial:
                    return self.state()
                raise HTTPException(status_code=409, detail="A camera node is already running outside this service; stop that launch first")
            if self._params_file is not None:
                self._params_file.unlink(missing_ok=True)
                self._params_file = None
            handle = tempfile.NamedTemporaryFile(prefix="visual_car_camera_", suffix=".yaml", delete=False)
            handle.close()
            params_file = Path(handle.name)
            params_file.write_text(
                "/hk_camera/hik_camera:\n  ros__parameters:\n"
                "    camera_name: narrow_stereo\n"
                f"    exposure_time: {self.settings.exposure_us}\n"
                f"    gain: {self.settings.gain}\n"
            )
            try:
                self.process = subprocess.Popen(
                    ["ros2", "launch", "hik_camera", "hik_camera.launch.py",
                     f"params_file:={params_file}"], start_new_session=True,
                )
            except Exception:
                params_file.unlink(missing_ok=True)
                raise
            self._params_file = params_file
        self.bridge.start_capture(self.bridge.settings)
        return self.state()

    def stop(self):
        if self.no_launch:
            raise HTTPException(status_code=409, detail="Camera was launched outside this service; stop it in its own terminal")
        with self._lock:
            process = self.process
            if process is None or process.poll() is not None:
                if self._camera_node_visible() or self.bridge.image_recent():
                    raise HTTPException(status_code=409, detail="The running camera belongs to another ROS launch; stop it in its own terminal")
                return self.state()
            self.process = None
        self.bridge.stop_capture()
        stop_camera(process)
        if self._params_file is not None:
            self._params_file.unlink(missing_ok=True)
            self._params_file = None
        self.bridge.clear_live()
        return self.state()

    def shutdown(self):
        with self._lock:
            process = self.process
            self.process = None
        stop_camera(process)
        if self._params_file is not None:
            self._params_file.unlink(missing_ok=True)
            self._params_file = None

    def apply_settings(self, settings: CameraSettings):
        if not self.state()["camera_process_running"]:
            raise HTTPException(status_code=409, detail="Start the camera before changing exposure or gain")
        if not self._parameter_client.wait_for_service(timeout_sec=4):
            raise HTTPException(status_code=503, detail="Camera parameter service is not available")
        request = SetParameters.Request()
        request.parameters = [
            Parameter("exposure_time", value=settings.exposure_us).to_parameter_msg(),
            Parameter("gain", value=settings.gain).to_parameter_msg(),
        ]
        response_ready = threading.Event()
        future = self._parameter_client.call_async(request)
        future.add_done_callback(lambda _: response_ready.set())
        if not response_ready.wait(timeout=8):
            raise HTTPException(status_code=504, detail="Camera setting request timed out")
        try:
            response = future.result()
        except Exception as exc:
            raise HTTPException(status_code=502, detail=f"Camera rejected settings: {exc}") from exc
        failures = [item.reason for item in response.results if not item.successful]
        if failures:
            raise HTTPException(status_code=502, detail="; ".join(failures))
        self.settings = settings
        return settings


def make_app(bridge: CameraBridge, store: FrameStore, controller: CameraController,
             processing=None, processed_store: FrameStore | None = None):
    app = FastAPI(title="visual_car camera")
    frontend = ROOT / "frontend"
    app.mount("/assets", StaticFiles(directory=frontend), name="assets")

    @app.get("/")
    def index():
        return FileResponse(frontend / "index.html")

    @app.get("/api/status")
    def status():
        result = bridge.status()
        result.update(controller.state())
        result["camera_settings"] = controller.settings.model_dump()
        if processing is not None:
            result.update(processing.status())
        return result

    @app.post("/api/capture/start")
    def start_capture(settings: CaptureSettings):
        if not controller.state()["camera_process_running"]:
            raise HTTPException(status_code=409, detail="Start the camera before saving photos")
        bridge.start_capture(settings)
        return bridge.status()

    @app.post("/api/capture/stop")
    def stop_capture():
        bridge.stop_capture()
        if processing is not None:
            processing.motion.set_following(False)
        return bridge.status()

    @app.post("/api/storage/{enabled}")
    def set_storage(enabled: bool):
        bridge.set_persistence(enabled)
        return bridge.status()

    @app.post("/api/device/start")
    def start_device():
        return controller.start()

    @app.post("/api/device/stop")
    def stop_device():
        if processing is not None:
            processing.motion.set_following(False)
        return controller.stop()

    @app.post("/api/camera/settings")
    def set_camera_settings(settings: CameraSettings):
        return controller.apply_settings(settings)

    @app.get("/api/stream")
    def stream():
        return StreamingResponse(
            bridge.stream(), media_type="multipart/x-mixed-replace; boundary=frame",
            headers={"Cache-Control": "no-store"},
        )

    @app.get("/api/snapshot")
    def snapshot():
        jpeg = processing.latest() if processing is not None else bridge.latest()
        if jpeg is None or (processing is None and not bridge.image_recent()):
            raise HTTPException(status_code=503, detail="No recent processed image")
        return Response(jpeg, media_type="image/jpeg", headers={"Cache-Control": "no-store"})

    @app.get("/api/frames")
    def frames():
        return [{"name": name, "url": f"/api/frames/{name}"} for name in store.names()]

    @app.get("/api/frames/{filename}")
    def saved_frame(filename: str):
        if not FRAME_NAME.fullmatch(filename):
            raise HTTPException(status_code=404, detail="Frame not found")
        jpeg = store.get(filename)
        if jpeg is None:
            raise HTTPException(status_code=404, detail="Frame not found")
        return Response(jpeg, media_type="image/jpeg", headers={"Cache-Control": "no-store"})

    if processed_store is not None:
        @app.get("/api/processed-frames")
        def processed_frames():
            return [{"name": name, "url": f"/api/processed-frames/{name}"}
                    for name in processed_store.names()]

        @app.get("/api/processed-frames/{filename}")
        def processed_frame(filename: str):
            if not FRAME_NAME.fullmatch(filename):
                raise HTTPException(status_code=404, detail="Frame not found")
            jpeg = processed_store.get(filename)
            if jpeg is None:
                raise HTTPException(status_code=404, detail="Frame not found")
            return Response(jpeg, media_type="image/jpeg", headers={"Cache-Control": "no-store"})

    return app


def stop_camera(process):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description="Hikvision camera capture and live preview")
    parser.add_argument("--fps", type=float, default=5.0, help="Saved photos per second")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Saving duration in seconds; 0 means unlimited")
    parser.add_argument("--preview-fps", type=float, default=10.0)
    parser.add_argument("--max-frames", type=int, default=100)
    parser.add_argument("--topic", default="/hk_camera/image_raw")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8890)
    parser.add_argument("--no-launch", action="store_true",
                        help="Use an already running Hikvision ROS camera node")
    args = parser.parse_args()
    settings = CaptureSettings(fps=args.fps, duration=args.duration)
    if not 0 < args.preview_fps <= 60:
        parser.error("--preview-fps must be between 0 and 60")
    if not 1 <= args.max_frames <= 10000:
        parser.error("--max-frames must be between 1 and 10000")
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")

    store = FrameStore(ROOT / "data" / "img_raw", args.max_frames)
    bridge = CameraBridge(settings, args.preview_fps, store)
    controller = None
    executor = None
    node = None
    spin_thread = None
    try:
        rclpy.init()
        node = rclpy.create_node("visual_car_camera_bridge")
        node.create_subscription(Image, args.topic, bridge.on_image, qos_profile_sensor_data)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        controller = CameraController(node, bridge, args.no_launch)
        # Let ROS discovery find an already running manual launch before opening the device.
        time.sleep(1)
        controller.start(initial=True)
        print(f"Preview: http://127.0.0.1:{args.port}  topic: {args.topic}", flush=True)
        uvicorn.run(make_app(bridge, store, controller), host=args.host,
                    port=args.port, workers=1)
    finally:
        if controller is not None:
            controller.shutdown()
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=5)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        store.close()


if __name__ == "__main__":
    main()
