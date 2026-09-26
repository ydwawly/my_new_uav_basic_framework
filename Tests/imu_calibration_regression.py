#!/usr/bin/env python3
"""Host-side regression tests for the BMI088 calibration pipeline."""

from __future__ import annotations

import importlib.util
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "Tools" / "imu_calibration" / "imu_calibration.py"
sys.path.insert(0, str(MODULE_PATH.parent))
import bmi088_temperature_compensation as temperature_compensation  # noqa: E402

SPEC = importlib.util.spec_from_file_location("imu_calibration", MODULE_PATH)
imu = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(imu)


def fibonacci_sphere(count: int) -> np.ndarray:
    index = np.arange(count, dtype=np.float64)
    z = 1.0 - 2.0 * (index + 0.5) / count
    angle = index * math.pi * (3.0 - math.sqrt(5.0))
    radius = np.sqrt(1.0 - z * z)
    return np.column_stack((radius * np.cos(angle), radius * np.sin(angle), z))


class ImuCalibrationRegression(unittest.TestCase):
    def test_scalar_gravity_integrator_matches_reference_rodrigues_steps(self) -> None:
        rng = np.random.default_rng(19)
        time_s = np.cumsum(np.r_[0.0, rng.uniform(0.005, 0.015, 200)])
        gyro = rng.normal(0.0, 0.4, (len(time_s), 3))
        start = np.array([0.2, -0.3, 0.9327379053])
        start /= np.linalg.norm(start)
        transition = imu.RotationTransition(
            time_s=time_s,
            gyro_rps=gyro,
            gravity_start=start,
            gravity_end=start,
        )
        matrix = np.array(
            [[1.01, 0.004, -0.002], [0.001, 0.99, 0.003], [0.0, -0.002, 1.02]]
        )
        bias = np.array([0.01, -0.02, 0.005])
        corrected = (matrix @ (gyro - bias).T).T
        reference = start.copy()
        for index, step in enumerate(np.diff(time_s)):
            omega = 0.5 * (corrected[index] + corrected[index + 1])
            reference = imu.rotate_vector(reference, -omega * step)
        reference /= np.linalg.norm(reference)
        np.testing.assert_allclose(
            imu.integrate_gravity(transition, matrix, bias), reference, atol=1e-12
        )

    def test_guided_report_recovers_all_accepted_pose_windows(self) -> None:
        sample_count = 120
        timestamps = (np.arange(sample_count) * 1_000).astype(np.uint64)
        data = {
            "timestamp_us": timestamps,
            "accel": np.tile([0.0, 0.0, imu.GRAVITY_MPS2], (sample_count, 1)),
        }
        poses = []
        for index in range(12):
            end = 10 * (index + 1)
            poses.append(
                {
                    "code": f"P{index}",
                    "accepted_timestamp_us": int(timestamps[end - 1]),
                    "samples": 10,
                    "target_direction": [0.0, 0.0, 1.0],
                }
            )
        with tempfile.TemporaryDirectory() as directory:
            report = Path(directory) / "guided_report.json"
            report.write_text(json.dumps({"poses": poses}), encoding="utf-8")
            segments, codes = imu.guided_static_segments(data, report, 0.005)
        self.assertEqual(segments, [(index, index + 10) for index in range(0, 120, 10)])
        self.assertEqual(codes, [f"P{index}" for index in range(12)])

    def test_gyroscope_lm_accepts_a_deliberate_transition_over_30_seconds(
        self,
    ) -> None:
        sample_count = 390
        gyro = np.zeros((sample_count, 3), dtype=np.float64)
        gyro[20:370, 0] = 0.05
        data = {
            "timestamp_us": (np.arange(sample_count) * 100_000).astype(np.uint64),
            "gyro": gyro,
            "accel": np.tile([0.0, 0.0, imu.GRAVITY_MPS2], (sample_count, 1)),
        }
        _, _, metrics = imu.fit_gyroscope(
            data,
            [(0, 20), (370, 390)],
            np.zeros(3),
            np.eye(3),
        )
        self.assertEqual(imu.MAX_GYRO_TRANSITION_SECONDS, 90.0)
        self.assertEqual(metrics["transition_count"], 1)

        _, _, excluded_metrics = imu.fit_gyroscope(
            data,
            [(0, 20), (370, 390)],
            np.zeros(3),
            np.eye(3),
            pose_codes=["A", "B"],
            excluded_transition_pairs={("A", "B")},
        )
        self.assertEqual(excluded_metrics["transition_count"], 0)
        self.assertEqual(excluded_metrics["excluded_transition_pairs"], [["A", "B"]])

    def test_offline_temperature_compensation_and_validity_check(self) -> None:
        temperatures = np.array(
            [
                temperature_compensation.GYRO_TEMP_MINIMUM_C,
                temperature_compensation.GYRO_TEMP_REFERENCE_C,
                temperature_compensation.GYRO_TEMP_MAXIMUM_C,
            ]
        )
        raw_gyro = temperature_compensation.gyro_temperature_correction_rps(
            temperatures
        )
        data = {
            "gyro": raw_gyro,
            "temperature": temperatures,
            "validity": np.full(3, 7, dtype=np.uint16),
        }
        compensated = imu.apply_gyro_temperature_compensation(data)
        np.testing.assert_allclose(compensated["gyro"], 0.0, atol=1e-15)
        np.testing.assert_array_equal(data["gyro"], raw_gyro)

        invalid = dict(data)
        invalid["validity"] = np.array([7, 3, 7], dtype=np.uint16)
        with self.assertRaisesRegex(ValueError, "1 个样本缺少有效温度"):
            imu.apply_gyro_temperature_compensation(invalid)

    def test_crc_reference_vector(self) -> None:
        self.assertEqual(imu.crc16_ccitt(b"123456789"), 0x29B1)

    def test_accelerometer_lm_recovers_norm_model(self) -> None:
        rng = np.random.default_rng(7)
        true_bias = np.array([0.12, -0.08, 0.05])
        true_matrix = np.array(
            [[1.012, 0.008, -0.005], [0.0, 0.991, 0.006], [0.0, 0.0, 1.018]]
        )
        truth = imu.GRAVITY_MPS2 * fibonacci_sphere(30)
        measured = (np.linalg.inv(true_matrix) @ truth.T).T + true_bias
        measured += rng.normal(0.0, 0.002, measured.shape)
        bias, matrix, metrics = imu.fit_accelerometer(measured)
        corrected = (matrix @ (measured - bias).T).T
        self.assertLess(metrics["norm_rmse_after_mps2"], 0.005)
        self.assertLess(
            np.max(np.abs(np.linalg.norm(corrected, axis=1) - imu.GRAVITY_MPS2)), 0.012
        )
        np.testing.assert_allclose(bias, true_bias, atol=0.01)

    def test_allan_white_noise_density(self) -> None:
        rng = np.random.default_rng(11)
        sample_interval = 0.002
        sample_std = 0.02
        values = rng.normal(0.0, sample_std, (120_000, 3))
        tau, deviation = imu.allan_deviation(values, sample_interval, points=45)
        coefficients = imu.allan_coefficients(tau, deviation)
        expected = sample_std * math.sqrt(sample_interval)
        for axis in coefficients:
            self.assertTrue(math.isfinite(axis["white_noise_density"]))
            self.assertAlmostEqual(
                axis["white_noise_density"], expected, delta=0.25 * expected
            )

    def test_blackbox_extracts_imu_payload(self) -> None:
        payload = imu.IMU_PAYLOAD_V1.pack(
            9,
            123,
            1,
            2,
            3,
            4,
            5,
            6,
            0,
            3,
            0.1,
            0.2,
            9.7,
            0.01,
            0.02,
            0.03,
            0.0,
        )
        header = imu.FRAME_HEADER.pack(
            imu.FRAME_MAGIC, imu.IMU_MESSAGE_ID, 1, len(payload), 3, 10, 123456
        )
        frame = header + payload
        frame += imu.struct.pack("<H", imu.crc16_ccitt(frame))
        file_header_prefix = imu.struct.pack(
            "<IHHQI", imu.FILE_MAGIC, 1, imu.FILE_HEADER.size, 0, 3
        )
        file_header = file_header_prefix + imu.struct.pack(
            "<HH", imu.crc16_ccitt(file_header_prefix), 0
        )
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "log.bin"
            output = Path(directory) / "imu.csv"
            source.write_bytes(file_header + frame)
            stats = imu.extract_blackbox(source, output)
            data = imu.load_csv(output)
        self.assertEqual(stats["imu_samples"], 1)
        self.assertEqual(int(data["sequence"][0]), 9)
        self.assertEqual(int(data["accel_sensor_time"][0]), 123)
        np.testing.assert_allclose(data["accel"][0], [0.1, 0.2, 9.7], rtol=1e-6)

    def test_generated_header_uses_valid_c_float_literals(self) -> None:
        calibration = {
            "accelerometer": {"bias": [0.0, 1.0, -2.0], "matrix": np.eye(3).tolist()},
            "gyroscope": {"bias": [0.0, 0.0, 0.0], "matrix": np.eye(3).tolist()},
        }
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "calibration.h"
            imu.write_calibration_header(calibration, output)
            content = output.read_text(encoding="utf-8")
        self.assertIn("1.00000000e+00f", content)
        self.assertNotIn(" 1f", content)


if __name__ == "__main__":
    unittest.main()
