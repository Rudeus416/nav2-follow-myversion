"""Offline checks for calibration data and API behavior; no radar required."""

from __future__ import annotations

import json
import io
import tempfile
import unittest
from contextlib import redirect_stderr
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np
import yaml
from fastapi.testclient import TestClient

import calibration_server
from calibration_common import (RADAR_PARAMS, RADAR_TO_OPTICAL_AXES,
                                TOOLKIT_ROOT, pair_nearest, transform_matrix)
from capture import (POINT_FIELDS, SessionWriter, decode_camera_image,
                     parse_args, prepare_radar_config)
from capture_control import CaptureInput, capture_argv


class CalibrationTests(unittest.TestCase):
    def test_nearest_pairing_reuses_image_and_honors_threshold(self):
        cameras = [{"file": "camera/a.jpg", "stamp_ns": 1_000_000_000},
                   {"file": "camera/b.jpg", "stamp_ns": 1_100_000_000}]
        radar = [{"file": "radar/1.npz", "stamp_ns": 1_000_010_000},
                 {"file": "radar/2.npz", "stamp_ns": 1_000_020_000},
                 {"file": "radar/3.npz", "stamp_ns": 1_200_000_000}]
        pairs = pair_nearest(cameras, radar, 20_000)
        self.assertEqual([pair["radar"] for pair in pairs],
                         ["radar/1.npz", "radar/2.npz"])
        self.assertEqual([pair["camera"] for pair in pairs],
                         ["camera/a.jpg", "camera/a.jpg"])
        self.assertEqual(pairs[1]["delta_ns"], 20_000)

    def test_zero_adjustment_converts_ros_axes_to_optical_axes(self):
        matrix = transform_matrix(0, 0, 0, 0, 0, 0)
        np.testing.assert_allclose(matrix @ [2, 0, 0, 1], [0, 0, 2, 1])
        np.testing.assert_allclose(matrix @ [0, 1, 0, 1], [-1, 0, 0, 1])
        np.testing.assert_allclose(matrix @ [0, 0, 1, 1], [0, -1, 0, 1])
        self.assertAlmostEqual(np.linalg.det(matrix[:3, :3]), 1)
        moved = transform_matrix(10, -5, 3, .1, -.2, .3)
        np.testing.assert_allclose(moved[:3, :3].T @ moved[:3, :3], np.eye(3), atol=1e-12)
        np.testing.assert_allclose(moved[:3, 3], [.2, -.3, .1])

    def test_rotation_and_translation_follow_radar_xyz_axes(self):
        yaw = transform_matrix(90, 0, 0, 0, 0, 0)
        pitch = transform_matrix(0, 90, 0, 0, 0, 0)
        roll = transform_matrix(0, 0, 90, 0, 0, 0)
        np.testing.assert_allclose(yaw[:3, :3] @ [1, 0, 0],
                                   RADAR_TO_OPTICAL_AXES @ [0, 1, 0], atol=1e-12)
        np.testing.assert_allclose(pitch[:3, :3] @ [1, 0, 0],
                                   RADAR_TO_OPTICAL_AXES @ [0, 0, -1], atol=1e-12)
        np.testing.assert_allclose(roll[:3, :3] @ [0, 1, 0],
                                   RADAR_TO_OPTICAL_AXES @ [0, 0, 1], atol=1e-12)
        np.testing.assert_allclose(transform_matrix(0, 0, 0, 1, 2, 3)[:3, 3],
                                   [-2, -3, 1])

    def test_old_parameter_conversion_preserves_full_matrix(self):
        old_yaw, old_pitch, old_roll = 13, -7, 4
        old_tx, old_ty, old_tz = .25, -.13, 1.2
        y, p, r = np.deg2rad([old_yaw, old_pitch, old_roll])
        old_ry = np.array([[np.cos(y), 0, np.sin(y)], [0, 1, 0],
                           [-np.sin(y), 0, np.cos(y)]])
        old_rx = np.array([[1, 0, 0], [0, np.cos(p), -np.sin(p)],
                           [0, np.sin(p), np.cos(p)]])
        old_rz = np.array([[np.cos(r), -np.sin(r), 0],
                           [np.sin(r), np.cos(r), 0], [0, 0, 1]])
        old_matrix = np.eye(4)
        old_matrix[:3, :3] = old_ry @ old_rx @ old_rz @ RADAR_TO_OPTICAL_AXES
        old_matrix[:3, 3] = [old_tx, old_ty, old_tz]
        new_matrix = transform_matrix(-old_yaw, -old_pitch, old_roll,
                                      old_tz, -old_tx, -old_ty)
        np.testing.assert_allclose(new_matrix, old_matrix, atol=1e-12)

    def test_radar_profile_snapshot_changes_both_nodes_only(self):
        original = RADAR_PARAMS.read_bytes()
        with tempfile.TemporaryDirectory() as temporary:
            profile, params, limit = prepare_radar_config(Path(temporary), 5.0)
            self.assertEqual(limit, 5)
            self.assertIn("cfarFovCfg -1 0 0 5.000", profile.read_text())
            self.assertIn("frameCfg 0 1 64 0 100 1 0", profile.read_text())
            self.assertIn("guiMonitor -1 1 0 0 0 0 0", profile.read_text())
            snapshot = yaml.safe_load(params.read_text())
            self.assertEqual(snapshot["mmwave_raw_node"]["ros__parameters"]["frame_rate"], 10)
            self.assertEqual(
                snapshot["mmwave_raw_node"]["ros__parameters"]["radar_config_file"],
                str(profile),
            )
            self.assertEqual(
                snapshot["ti_mmwave_point_node"]["ros__parameters"]["radar_config_file"],
                str(profile),
            )
        self.assertEqual(RADAR_PARAMS.read_bytes(), original)

    def test_radar_profile_tuning_controls_cfar_and_near_field(self):
        source = TOOLKIT_ROOT / "config/radar/xWR1843_profile_2tx4rx_20hz.cfg"
        with tempfile.TemporaryDirectory() as temporary:
            profile, params, limit = prepare_radar_config(
                Path(temporary), 6.0, frame_period_ms=200,
                source_profile_override=source, cfar_range_db=12,
                cfar_doppler_db=12, peak_grouping="doppler",
                min_range_m=0.3,
            )
            contents = profile.read_text()
            self.assertEqual(limit, 6)
            self.assertIn("channelCfg 15 5 0", contents)
            self.assertIn("frameCfg 0 1 16 0 200 1 0", contents)
            self.assertIn("cfarCfg -1 0 2 8 4 3 0 12 0", contents)
            self.assertIn("cfarCfg -1 1 0 4 2 3 1 12 1", contents)
            self.assertIn("cfarFovCfg -1 0 0.300 6.000", contents)
            snapshot = yaml.safe_load(params.read_text())
            self.assertEqual(snapshot["mmwave_raw_node"]["ros__parameters"]["frame_rate"], 5)
            with self.assertRaisesRegex(ValueError, "未启用 LVDS"):
                prepare_radar_config(
                    Path(temporary), 6.0,
                    source_profile_override=TOOLKIT_ROOT / "config/radar/AWR1843_slam.cfg",
                )

    def test_capture_defaults_and_external_radar_mode(self):
        args = parse_args([])
        self.assertEqual(args.camera_fps, 5)
        self.assertEqual(args.pair_threshold_ms, 120)
        self.assertEqual(args.radar_frame_period_ms, 200)
        self.assertEqual((args.min_range_m, args.max_range_m), (0, 6))
        self.assertEqual((args.cfar_range_db, args.cfar_doppler_db), (10, 10))
        self.assertEqual(args.cfar_peak_grouping, "none")
        self.assertEqual(args.radar_profile.name, "xWR1843_profile_1280.cfg")
        with tempfile.TemporaryDirectory() as temporary:
            profile, _, _ = prepare_radar_config(
                Path(temporary), args.max_range_m, args.cli_port,
                args.data_port, args.radar_frame_period_ms,
                args.radar_profile, args.cfar_range_db,
                args.cfar_doppler_db, args.cfar_peak_grouping,
                args.min_range_m,
            )
            contents = profile.read_text()
            self.assertIn("frameCfg 0 1 64 0 200 1 0", contents)
            self.assertIn("cfarCfg -1 0 2 8 4 3 0 10 0", contents)
            self.assertIn("cfarCfg -1 1 0 8 4 4 1 10 0", contents)
            self.assertIn("cfarFovCfg -1 0 0 6.000", contents)
        self.assertTrue(parse_args(["--external-radar"]).external_radar)
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(["--external-radar", "--cfar-range-db", "10"])

    def test_web_capture_respects_external_radar_and_rejects_invalid_name(self):
        argv = capture_argv(CaptureInput(external_radar=True, external_camera=True))
        self.assertIn("--external-radar", argv)
        self.assertIn("--external-camera", argv)
        for option in ("--radar-profile", "--max-range-m", "--cfar-range-db",
                       "--radar-frame-period-ms", "--cli-port"):
            self.assertNotIn(option, argv)
        self.assertTrue(parse_args(argv).external_radar)
        with self.assertRaises(ValueError):
            capture_argv(CaptureInput(name="../outside"))

    def test_writer_saves_image_and_cloud(self):
        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary)
            (session / "camera").mkdir()
            (session / "radar").mkdir()
            header = SimpleNamespace(frame_id="camera_optical_frame")
            rgb = np.zeros((4, 5, 3), dtype=np.uint8)
            rgb[:, :, 0] = 255
            image = SimpleNamespace(
                header=header, encoding="rgb8", width=5, height=4, step=15,
                data=rgb.tobytes(),
            )
            np.testing.assert_array_equal(decode_camera_image(image)[0, 0], [0, 0, 255])
            cloud = SimpleNamespace(
                header=SimpleNamespace(frame_id="ti_mmwave_0"),
                fields=[SimpleNamespace(name=field) for field in POINT_FIELDS],
            )
            parser = SimpleNamespace(read_points=lambda *_args, **_kwargs: [
                (2.0, 0.0, 0.0, 2.0, .2, 10.0, 100.0, 5.0)
            ])
            writer = SessionWriter(session, parser, 95, 4)
            writer.offer("camera", image, "1.jpg", 1, 2, "header")
            writer.offer("radar", cloud, "2.npz", 2, 3, "header")
            writer.close()
            self.assertEqual(len(writer.camera_frames), 1)
            self.assertEqual(len(writer.radar_frames), 1)
            self.assertIsNotNone(cv2.imread(str(session / "camera" / "1.jpg")))
            with np.load(session / "radar" / "2.npz", allow_pickle=False) as saved:
                np.testing.assert_allclose(saved["points"][0, :4], [2, 0, 0, 2])

    def test_api_pair_and_save_load_update_same_file(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            calib = root / "data_calib"
            transfers = root / "data_transfer"
            session = calib / "demo"
            for subfolder in ("camera", "radar", "pairs"):
                (session / subfolder).mkdir(parents=True, exist_ok=True)
            (session / "manifest.json").write_text(json.dumps({
                "pair_count": 1, "camera_frames": [{}], "radar_frames": [{}],
            }))
            image = np.zeros((4, 5, 3), dtype=np.uint8)
            cv2.imwrite(str(session / "camera" / "1.jpg"), image)
            np.savez_compressed(session / "radar" / "1.npz",
                                points=np.asarray([[2, 0, 0, 2, 0, 10, 0, 0]], dtype=np.float32),
                                fields=np.asarray(POINT_FIELDS))
            (session / "pairs" / "000001.json").write_text(json.dumps({
                "id": "000001", "camera": "camera/1.jpg", "radar": "radar/1.npz",
                "camera_stamp_ns": 1, "radar_stamp_ns": 2,
                "delta_ns": 1, "abs_delta_ms": .001,
            }))
            with patch.object(calibration_server, "CALIB_ROOT", calib), \
                 patch.object(calibration_server, "TRANSFER_ROOT", transfers):
                client = TestClient(calibration_server.app)
                self.assertEqual(client.get("/api/sessions").json()[0]["pair_count"], 1)
                detail = client.get("/api/sessions/demo/pairs/000001").json()
                self.assertEqual(detail["points"][0][:4], [2, 0, 0, 2])
                self.assertEqual(client.get(detail["image_url"]).status_code, 200)
                self.assertEqual(len(client.get("/api/colormap").json()["colors"]), 256)
                payload = {"parameter_frame": "radar_ros", "yaw_deg": 10, "tx_m": .1,
                           "session": "demo", "pair_id": "000001"}
                self.assertEqual(client.post("/api/transforms", json={
                    "yaw_deg": 10, "tx_m": .1,
                }).status_code, 422)
                created = client.post("/api/transforms", json=payload).json()
                name = created["name"]
                self.assertTrue((transfers / name).is_file())
                self.assertEqual(created["schema_version"], 2)
                self.assertEqual(created["parameter_frame"], "radar_ros")
                self.assertEqual(created["translation_m"], {"x": .1, "y": 0, "z": 0})
                self.assertEqual(created["translation_camera_optical_m"],
                                 {"x": 0, "y": 0, "z": .1})
                np.testing.assert_allclose(created["radar_to_camera_optical_4x4"],
                                           transform_matrix(10, 0, 0, .1, 0, 0))
                self.assertEqual(client.get(f"/api/transforms/{name}").json()["adjustment_euler_deg"]["yaw"], 10)
                payload["yaw_deg"] = 12
                updated = client.put(f"/api/transforms/{name}", json=payload).json()
                self.assertEqual(updated["name"], name)
                self.assertEqual(updated["created_at_utc"], created["created_at_utc"])
                self.assertEqual(updated["adjustment_euler_deg"]["yaw"], 12)
                self.assertEqual(len(list(transfers.glob("*.json"))), 1)


if __name__ == "__main__":
    unittest.main()
