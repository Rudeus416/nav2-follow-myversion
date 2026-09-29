#!/usr/bin/env python3
"""Capture timestamped Hikvision images and TI mmWave detected point clouds.

Source ROS Humble and dataCollectToolkit/ros2_driver_ws/install/setup.bash first.
The default mode starts both sensor launch files. Use --external-camera and/or
--external-radar when another process already owns a sensor.
"""

from __future__ import annotations

import argparse
import json
import os
import queue
import signal
import site
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

from calibration_common import (CALIB_ROOT, RADAR_PARAMS, ROOT, TOOLKIT_ROOT,
                                pair_nearest, ros_stamp_ns, safe_name,
                                timestamp_name)


RADAR_TOPIC = "/ti_mmwave/radar_scan_pcl"
CAMERA_TOPIC = "/hk_camera/image_raw"
DEFAULT_RADAR_PROFILE = TOOLKIT_ROOT / "config/radar/xWR1843_profile_1280.cfg"
POINT_FIELDS = ("x", "y", "z", "range", "velocity", "intensity", "snr", "noise")


def prepare_radar_config(session: Path, max_range_m: float | None,
                         cli_port: str = "/dev/ttyACM0",
                         data_port: str = "/dev/ttyACM1",
                         frame_period_ms: float = 100.0,
                         source_profile_override: Path | None = None,
                         cfar_range_db: float | None = None,
                         cfar_doppler_db: float | None = None,
                         peak_grouping: str | None = None,
                         min_range_m: float | None = None) -> tuple[Path, Path, float | None]:
    """Snapshot the active profile and point both radar nodes to that snapshot."""
    params = yaml.safe_load(RADAR_PARAMS.read_text(encoding="utf-8"))
    source_profile = source_profile_override or Path(
        params["mmwave_raw_node"]["ros__parameters"]["radar_config_file"]
    )
    if not source_profile.is_absolute():
        source_profile = (TOOLKIT_ROOT / source_profile).resolve()
    profile = source_profile.read_text(encoding="utf-8")
    configured_limit = None
    lines = []
    found = False
    lvds_enabled = False
    for line in profile.splitlines():
        parts = line.split()
        if len(parts) >= 8 and parts[0] == "frameCfg":
            parts[5] = f"{frame_period_ms:g}"
            line = " ".join(parts)
        elif len(parts) >= 8 and parts[0] == "guiMonitor":
            # Calibration only needs detected objects. The source profile
            # enables large heatmaps that saturate the 921600-baud data UART.
            line = "guiMonitor -1 1 0 0 0 0 0"
        elif len(parts) >= 10 and parts[0] == "cfarCfg" and parts[2] in ("0", "1"):
            threshold = cfar_range_db if parts[2] == "0" else cfar_doppler_db
            if threshold is not None:
                parts[8] = f"{threshold:g}"
            if peak_grouping is not None:
                group_range = parts[2] == "0" and peak_grouping in ("both", "range")
                group_doppler = parts[2] == "1" and peak_grouping in ("both", "doppler")
                parts[9] = "1" if group_range or group_doppler else "0"
            line = " ".join(parts)
        if len(parts) >= 5 and parts[0] == "lvdsStreamCfg":
            lvds_enabled = parts[3] != "0"
        if len(parts) >= 5 and parts[0] == "cfarFovCfg" and parts[2] == "0":
            found = True
            if min_range_m is not None and min_range_m != float(parts[3]):
                parts[3] = f"{min_range_m:.3f}"
            if max_range_m is not None:
                minimum = float(parts[3])
                if max_range_m <= minimum:
                    raise ValueError(f"距离上限必须大于 profile 的下限 {minimum} m")
                parts[4] = f"{max_range_m:.3f}"
            if float(parts[3]) >= float(parts[4]):
                raise ValueError("距离下限必须小于 profile 的距离上限")
            line = " ".join(parts)
            configured_limit = float(parts[4])
        lines.append(line)
    if not found:
        raise ValueError(f"雷达 profile 缺少距离维度的 cfarFovCfg: {source_profile}")
    if not lvds_enabled:
        raise ValueError(
            f"雷达 profile 未启用 LVDS 原始数据输出，不能用于当前联合采集: {source_profile}"
        )
    profile_snapshot = session / "radar_profile.cfg"
    profile_snapshot.write_text("\n".join(lines) + "\n", encoding="utf-8")
    for node_name in ("mmwave_raw_node", "ti_mmwave_point_node"):
        params[node_name]["ros__parameters"]["radar_config_file"] = str(profile_snapshot)
        params[node_name]["ros__parameters"]["data_port"] = data_port
    params["mmwave_raw_node"]["ros__parameters"]["cli_port"] = cli_port
    params["mmwave_raw_node"]["ros__parameters"]["frame_rate"] = max(
        1, round(1000 / frame_period_ms)
    )
    params["ti_mmwave_point_node"]["ros__parameters"]["command_port"] = cli_port
    params_snapshot = session / "radar_params.yaml"
    params_snapshot.write_text(yaml.safe_dump(params, sort_keys=False), encoding="utf-8")
    return profile_snapshot, params_snapshot, configured_limit


def decode_camera_image(msg) -> np.ndarray:
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}
    if msg.encoding not in channels:
        raise ValueError(f"不支持相机编码: {msg.encoding}")
    width, height = int(msg.width), int(msg.height)
    needed = width * channels[msg.encoding]
    if width <= 0 or height <= 0 or int(msg.step) < needed:
        raise ValueError("相机图像尺寸或行步长无效")
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, int(msg.step))
    pixels = rows[:, :needed].reshape(height, width, channels[msg.encoding])
    conversions = {
        "rgb8": cv2.COLOR_RGB2BGR,
        "rgba8": cv2.COLOR_RGBA2BGR,
        "bgra8": cv2.COLOR_BGRA2BGR,
        "mono8": cv2.COLOR_GRAY2BGR,
    }
    if msg.encoding == "bgr8":
        return pixels.copy()
    return cv2.cvtColor(pixels, conversions[msg.encoding])


def decode_radar_points(msg, point_cloud2) -> np.ndarray:
    available = {field.name for field in msg.fields}
    if not {"x", "y", "z"}.issubset(available):
        raise ValueError("雷达点云缺少 x/y/z 字段")
    selected = tuple(field for field in POINT_FIELDS if field in available)
    rows = point_cloud2.read_points(msg, field_names=selected, skip_nans=True)
    values = np.asarray([tuple(row) for row in rows], dtype=np.float32)
    if values.size == 0:
        return np.empty((0, len(POINT_FIELDS)), dtype=np.float32)
    values = values.reshape(-1, len(selected))
    points = np.zeros((len(values), len(POINT_FIELDS)), dtype=np.float32)
    for index, field in enumerate(selected):
        points[:, POINT_FIELDS.index(field)] = values[:, index]
    if "range" not in available:
        points[:, 3] = np.linalg.norm(points[:, :3], axis=1)
    return points


class SessionWriter:
    def __init__(self, session: Path, point_cloud2, jpeg_quality: int, queue_size: int):
        self.session = session
        self.point_cloud2 = point_cloud2
        self.jpeg_quality = jpeg_quality
        # Camera JPEG encoding cannot block the much smaller radar frames.
        self.pending: dict[str, queue.Queue[tuple | None]] = {
            "camera": queue.Queue(maxsize=queue_size),
            "radar": queue.Queue(maxsize=max(64, queue_size * 4)),
        }
        self.camera_frames: list[dict] = []
        self.radar_frames: list[dict] = []
        self.dropped = {"camera": 0, "radar": 0}
        self.errors: list[str] = []
        self._threads = [
            threading.Thread(target=self._loop, args=(kind,), daemon=True,
                             name=f"calibration-{kind}-writer")
            for kind in ("camera", "radar")
        ]
        for thread in self._threads:
            thread.start()

    def offer(self, kind: str, msg, filename: str, stamp_ns: int,
              received_ns: int, stamp_source: str) -> None:
        try:
            self.pending[kind].put_nowait(
                (kind, msg, filename, stamp_ns, received_ns, stamp_source)
            )
        except queue.Full:
            self.dropped[kind] += 1

    def _loop(self, kind: str):
        pending = self.pending[kind]
        while True:
            item = pending.get()
            try:
                if item is None:
                    return
                kind, msg, filename, stamp_ns, received_ns, stamp_source = item
                path = self.session / kind / filename
                metadata = {
                    "file": f"{kind}/{filename}",
                    "stamp_ns": stamp_ns,
                    "received_ns": received_ns,
                    "stamp_source": stamp_source,
                    "frame_id": msg.header.frame_id,
                }
                if kind == "camera":
                    image = decode_camera_image(msg)
                    ok, encoded = cv2.imencode(
                        ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
                    )
                    if not ok:
                        raise RuntimeError("JPEG 编码失败")
                    path.write_bytes(encoded.tobytes())
                    metadata.update(width=int(msg.width), height=int(msg.height))
                    self.camera_frames.append(metadata)
                else:
                    points = decode_radar_points(msg, self.point_cloud2)
                    np.savez_compressed(
                        path, points=points, fields=np.asarray(POINT_FIELDS),
                        stamp_ns=np.int64(stamp_ns), frame_id=np.str_(msg.header.frame_id),
                    )
                    metadata["point_count"] = len(points)
                    self.radar_frames.append(metadata)
            except Exception as exc:
                if len(self.errors) < 30:
                    self.errors.append(f"{item[0] if item else 'writer'}: {exc}")
            finally:
                pending.task_done()

    def close(self):
        for pending in self.pending.values():
            pending.join()
        for pending in self.pending.values():
            pending.put(None)
        for thread in self._threads:
            thread.join()


def start_launch(command: list[str], logfile: Path,
                 env: dict[str, str] | None = None) -> tuple[subprocess.Popen, object]:
    handle = logfile.open("w", encoding="utf-8")
    try:
        process = subprocess.Popen(
            command, cwd=TOOLKIT_ROOT, stdout=handle, stderr=subprocess.STDOUT,
            start_new_session=True, env=env,
        )
    except Exception:
        handle.close()
        raise
    return process, handle


def stop_launch(process: subprocess.Popen, handle) -> None:
    try:
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait(timeout=3)
    finally:
        handle.close()


def check_radar_network() -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        try:
            probe.bind(("192.168.33.30", 0))
        except OSError as exc:
            raise RuntimeError(
                "本机没有 DCA1000 驱动默认地址 192.168.33.30；"
                "先给连接 DCA1000 的网卡配置该地址"
            ) from exc


def save_session(session: Path, writer: SessionWriter, args,
                 started_wall_ns: int, ended_wall_ns: int,
                 profile_file: Path | None, configured_limit: float | None,
                 received_counts: dict[str, int]) -> dict:
    camera = sorted(writer.camera_frames, key=lambda frame: frame["stamp_ns"])
    radar = sorted(writer.radar_frames, key=lambda frame: frame["stamp_ns"])
    pairs = pair_nearest(camera, radar, round(args.pair_threshold_ms * 1_000_000))
    for index, pair in enumerate(pairs, start=1):
        pair["id"] = f"{index:06d}"
        (session / "pairs" / f"{index:06d}.json").write_text(
            json.dumps(pair, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
    manifest = {
        "session": session.name,
        "started_wall_ns": started_wall_ns,
        "ended_wall_ns": ended_wall_ns,
        "duration_s": args.duration,
        "actual_duration_s": ((ended_wall_ns - started_wall_ns) / 1e9
                              if started_wall_ns and ended_wall_ns else 0),
        "camera_fps_limit": args.camera_fps,
        "received_counts": received_counts,
        "camera_topic": args.camera_topic,
        "radar_topic": args.radar_topic,
        "pair_threshold_ms": args.pair_threshold_ms,
        "max_range_m": configured_limit if not args.external_radar else None,
        "min_range_m": args.min_range_m if not args.external_radar else None,
        "cfar_range_db": args.cfar_range_db if not args.external_radar else None,
        "cfar_doppler_db": args.cfar_doppler_db if not args.external_radar else None,
        "cfar_peak_grouping": args.cfar_peak_grouping if not args.external_radar else None,
        "radar_frame_period_ms": (args.radar_frame_period_ms
                                  if not args.external_radar else None),
        "radar_profile": profile_file.name if profile_file else None,
        "camera_frames": camera,
        "radar_frames": radar,
        "pair_count": len(pairs),
        "dropped_queue": writer.dropped,
        "write_errors": writer.errors,
        "warnings": (["本次未收到雷达点云；请检查雷达目标检测、UART 数据输出及距离阈值"]
                     if not radar else []),
    }
    (session / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return manifest


def run_capture(args) -> dict:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, PointCloud2
    from sensor_msgs_py import point_cloud2

    name = safe_name(args.name or timestamp_name())
    CALIB_ROOT.mkdir(parents=True, exist_ok=True)
    session = CALIB_ROOT / name
    session.mkdir(exist_ok=False)
    for subfolder in ("camera", "radar", "pairs"):
        (session / subfolder).mkdir()

    profile_file = None
    params_file = None
    configured_limit = None
    if not args.external_radar:
        if not Path(args.cli_port).exists() or not Path(args.data_port).exists():
            raise RuntimeError(f"未发现雷达串口 {args.cli_port} / {args.data_port}")
        check_radar_network()
        profile_file, params_file, configured_limit = prepare_radar_config(
            session, args.max_range_m, args.cli_port, args.data_port,
            args.radar_frame_period_ms, args.radar_profile,
            args.cfar_range_db, args.cfar_doppler_db,
            args.cfar_peak_grouping, args.min_range_m,
        )

    processes: list[tuple[subprocess.Popen, object]] = []
    writer = SessionWriter(session, point_cloud2, args.jpeg_quality, args.queue_size)
    rclpy.init()
    node = rclpy.create_node("visual_car_calibration_capture")
    recording = False
    camera_seen = False
    camera_seq = 0
    radar_seq = 0
    received_counts = {"camera": 0, "radar": 0}
    last_camera_offer_at = 0.0

    def on_camera(msg):
        nonlocal camera_seq, camera_seen, last_camera_offer_at
        camera_seen = True
        if not recording:
            return
        received_counts["camera"] += 1
        now = time.monotonic()
        if now - last_camera_offer_at < 1.0 / args.camera_fps:
            return
        last_camera_offer_at = now
        camera_seq += 1
        received_ns = time.time_ns()
        stamp_ns, source = ros_stamp_ns(msg.header.stamp, received_ns)
        writer.offer("camera", msg, f"{stamp_ns}_{camera_seq:06d}.jpg",
                     stamp_ns, received_ns, source)

    def on_radar(msg):
        nonlocal radar_seq
        if not recording:
            return
        received_counts["radar"] += 1
        radar_seq += 1
        received_ns = time.time_ns()
        stamp_ns, source = ros_stamp_ns(msg.header.stamp, received_ns)
        writer.offer("radar", msg, f"{stamp_ns}_{radar_seq:06d}.npz",
                     stamp_ns, received_ns, source)

    node.create_subscription(Image, args.camera_topic, on_camera, qos_profile_sensor_data)
    node.create_subscription(PointCloud2, args.radar_topic, on_radar, qos_profile_sensor_data)
    started_wall_ns = ended_wall_ns = 0
    try:
        if not (args.external_camera and args.external_radar):
            discovery_deadline = time.monotonic() + 1.5
            while time.monotonic() < discovery_deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if not args.external_camera and node.count_publishers(args.camera_topic):
                raise RuntimeError("相机话题已有发布者；请使用 --external-camera")
            if not args.external_radar and node.count_publishers(args.radar_topic):
                raise RuntimeError("雷达话题已有发布者；请使用 --external-radar")
        if not args.external_radar:
            # ROS's installed radar entry point uses /usr/bin/python3. Keep its
            # NumPy compatible with the interpreter running capture.py: the
            # machine's user-site SciPy may otherwise import an older system NumPy.
            # fpga_udp and pyserial are installed in the user site on this Jetson;
            # include it explicitly if the launching shell disables user sites.
            radar_env = os.environ.copy()
            numpy_site = str(Path(np.__file__).resolve().parent.parent)
            radar_env["PYTHONPATH"] = os.pathsep.join(
                filter(None, (numpy_site, site.getusersitepackages(),
                              radar_env.get("PYTHONPATH", "")))
            )
            check = subprocess.run(
                ["/usr/bin/python3", "-c",
                 "from ti_mmwave_driver.mmwave.dataloader import DCA1000"],
                cwd=TOOLKIT_ROOT, env=radar_env, capture_output=True, text=True,
            )
            if check.returncode:
                detail = check.stderr.strip().splitlines()[-1] if check.stderr.strip() else "未知导入错误"
                raise RuntimeError(f"雷达 Python 依赖检查失败：{detail}")
            processes.append(start_launch([
                "ros2", "launch", "ti_mmwave_driver", "ti_mmwave_driver_all.launch.py",
                f"params_file:={params_file}",
            ], session / "radar.log", radar_env))
        if not args.external_camera:
            processes.append(start_launch([
                "ros2", "launch", "hik_camera", "hik_camera.launch.py",
                f"params_file:={ROOT / 'camera_params.yaml'}",
            ], session / "camera.log"))

        print("等待相机图像首帧及雷达话题发布者……", flush=True)
        deadline = time.monotonic() + args.startup_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(process.poll() is None for process, _ in processes) and (
                node.count_publishers(args.camera_topic) > 0
                and node.count_publishers(args.radar_topic) > 0
                and (args.external_radar or node.count_publishers("/radar/adc_frame") > 0)
                and camera_seen
            ):
                break
            if any(process.poll() is not None for process, _ in processes):
                raise RuntimeError("传感器启动进程提前退出；查看本次采集目录中的日志")
        else:
            raise TimeoutError("等待相机首帧/雷达原始节点/点云节点超时；查看本次采集目录中的日志")

        started_wall_ns = time.time_ns()
        end_monotonic = time.monotonic() + args.duration
        recording = True
        print(f"开始采集 {args.duration:g} 秒：{session}", flush=True)
        while rclpy.ok() and time.monotonic() < end_monotonic:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, end_monotonic - time.monotonic())))
            if any(process.poll() is not None for process, _ in processes):
                raise RuntimeError("传感器启动进程在采集期间退出；查看日志")
        recording = False
        ended_wall_ns = time.time_ns()
    finally:
        recording = False
        node.destroy_node()
        rclpy.shutdown()
        writer.close()
        for process, handle in reversed(processes):
            stop_launch(process, handle)
        manifest = save_session(
            session, writer, args, started_wall_ns, ended_wall_ns,
            profile_file, configured_limit, received_counts,
        )
        print(
            f"已保存：相机 {len(manifest['camera_frames'])} 帧，雷达 "
            f"{len(manifest['radar_frames'])} 帧，配对 {manifest['pair_count']} 组；"
            f"队列丢弃 {writer.dropped}", flush=True,
        )
        for warning in manifest["warnings"]:
            print(f"警告：{warning}", flush=True)
    return manifest


def parse_args(argv: list[str] | None = None):
    parser = argparse.ArgumentParser(
        description=__doc__,
        
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        allow_abbrev=False,
    )
    parser.add_argument("--name", default=None,
                        help="本次采集目录名；不指定时使用启动时间")
    parser.add_argument("--duration", type=float, default=3.0,
                        help="采集时长，秒")
    parser.add_argument("--camera-fps", type=float, default=5.0,
                        help="图像存储频率上限，FPS")
    parser.add_argument("--pair-threshold-ms", type=float, default=120.0,
                        help="雷达帧匹配最近图像的最大时间差，毫秒")
    parser.add_argument("--max-range-m", type=float, default=6.0,
                        help="雷达输出的最大距离，米")
    parser.add_argument("--radar-frame-period-ms", type=float, default=200.0,
                        help="雷达帧周期，毫秒；200 ms 约为 5 FPS")
    parser.add_argument("--radar-profile", type=Path, default=DEFAULT_RADAR_PROFILE,
                        help="雷达配置源文件；默认使用实测过的 2TX profile")
    parser.add_argument("--min-range-m", type=float, default=0.0,
                        help="雷达输出的最小距离，米；设为 0.3 可排除近场串扰点")
    parser.add_argument("--cfar-range-db", type=float, default=10.0,
                        help="距离维 CFAR 门限，dB；源 profile 为 15 dB")
    parser.add_argument("--cfar-doppler-db", type=float, default=10.0,
                        help="速度维 CFAR 门限，dB；源 profile 为 15 dB")
    parser.add_argument("--cfar-peak-grouping", choices=("both", "range", "doppler", "none"),
                        default="none", help="CFAR 峰值合并方向")
    parser.add_argument("--external-camera", action="store_true", default=False,
                        help="相机已由其他进程启动，本脚本仅订阅")
    parser.add_argument("--external-radar", action="store_true", default=False,
                        help="雷达已由其他进程启动，本脚本仅订阅")
    parser.add_argument("--camera-topic", default=CAMERA_TOPIC,
                        help="订阅相机图像的 ROS 话题")
    parser.add_argument("--radar-topic", default=RADAR_TOPIC,
                        help="订阅雷达点云的 ROS 话题")
    parser.add_argument("--cli-port", default="/dev/ttyACM0",
                        help="雷达命令串口")
    parser.add_argument("--data-port", default="/dev/ttyACM1",
                        help="雷达点云数据串口")
    parser.add_argument("--startup-timeout", type=float, default=45.0,
                        help="等待传感器启动的最长秒数")
    parser.add_argument("--jpeg-quality", type=int, default=95,
                        help="JPEG 图像质量，范围 1–100")
    parser.add_argument("--queue-size", type=int, default=16,
                        help="每种数据的写盘队列容量基数")
    raw_argv = sys.argv[1:] if argv is None else argv
    args = parser.parse_args(raw_argv)
    if (not all(np.isfinite(value) for value in (
            args.duration, args.camera_fps, args.pair_threshold_ms,
            args.startup_timeout, args.radar_frame_period_ms))
            or args.duration <= 0 or args.camera_fps <= 0
            or args.pair_threshold_ms < 0 or args.startup_timeout <= 0
            or args.radar_frame_period_ms <= 0):
        parser.error("duration/startup-timeout/radar-frame-period-ms 必须大于 0，配对阈值必须非负")
    if args.max_range_m is not None and (
        not np.isfinite(args.max_range_m) or args.max_range_m <= 0
    ):
        parser.error("max-range-m 必须是正的有限数字")
    if args.min_range_m is not None and (
        not np.isfinite(args.min_range_m) or args.min_range_m < 0
    ):
        parser.error("min-range-m 必须是非负的有限数字")
    if (args.min_range_m is not None and args.max_range_m is not None
            and args.min_range_m >= args.max_range_m):
        parser.error("min-range-m 必须小于 max-range-m")
    for label, value in (("cfar-range-db", args.cfar_range_db),
                         ("cfar-doppler-db", args.cfar_doppler_db)):
        if value is not None and (not np.isfinite(value) or not 0 < value <= 30):
            parser.error(f"{label} 必须在 (0, 30] dB 范围内")
    if not args.external_radar and args.radar_profile is not None:
        args.radar_profile = args.radar_profile.expanduser().resolve()
        if not args.radar_profile.is_file():
            parser.error(f"雷达配置文件不存在: {args.radar_profile}")
    radar_options = (
        "--max-range-m", "--min-range-m", "--radar-profile",
        "--cfar-range-db", "--cfar-doppler-db", "--cfar-peak-grouping",
        "--radar-frame-period-ms", "--cli-port", "--data-port",
    )
    if args.external_radar and any(
        token == option or token.startswith(option + "=")
        for token in raw_argv for option in radar_options
    ):
        parser.error("使用 --external-radar 时，本脚本不能修改已有雷达的配置")
    if not 1 <= args.jpeg_quality <= 100 or args.queue_size < 1:
        parser.error("jpeg-quality 必须为 1~100，queue-size 必须至少为 1")
    if args.name:
        try:
            safe_name(args.name)
        except ValueError as exc:
            parser.error(str(exc))
    return args


if __name__ == "__main__":
    try:
        run_capture(parse_args())
    except KeyboardInterrupt:
        print("已中断采集，已保存收到的数据。", flush=True)
    except Exception as exc:
        raise SystemExit(f"采集失败：{exc}") from exc
