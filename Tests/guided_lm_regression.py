#!/usr/bin/env python3
"""Regression tests for the guided BMI088 LM capture core."""

from __future__ import annotations

import importlib.util
import json
import math
import argparse
import queue
import re
import struct
import sys
import tempfile
import unittest
from unittest import mock
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "Tools" / "imu_calibration" / "guided_lm_core.py"
sys.path.insert(0, str(MODULE_PATH.parent))
SPEC = importlib.util.spec_from_file_location("guided_lm_core", MODULE_PATH)
guided = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = guided
SPEC.loader.exec_module(guided)
import bmi088_temperature_compensation as temperature_compensation  # noqa: E402
import guided_lm_gui as gui  # noqa: E402


def make_frame(frame_sequence: int, sample_sequence: int, timestamp_us: int) -> bytes:
    payload = guided.IMU_PAYLOAD_V1.pack(
        sample_sequence,
        sample_sequence * 10,
        1,
        2,
        3,
        4,
        5,
        6,
        0,
        7,
        0.1,
        0.2,
        -9.7,
        0.01,
        0.02,
        0.03,
        32.5,
    )
    header = guided.FRAME_HEADER.pack(
        guided.FRAME_MAGIC,
        guided.IMU_MESSAGE_ID,
        1,
        len(payload),
        3,
        frame_sequence,
        timestamp_us,
    )
    frame = header + payload
    return frame + struct.pack("<H", guided.crc16_ccitt(frame))


def sample(
    timestamp_us: int,
    sequence: int,
    accel: np.ndarray,
    gyro: np.ndarray,
    temperature_c: float = 32.5,
    validity_flags: int = 7,
) -> guided.ImuSample:
    return guided.ImuSample(
        timestamp_us=timestamp_us,
        frame_sequence=sequence,
        sample_sequence=sequence,
        accel_sensor_time=sequence,
        validity_flags=validity_flags,
        accel_mps2=tuple(accel.tolist()),
        gyro_rps=tuple(gyro.tolist()),
        temperature_c=temperature_c,
    )


class StreamParserRegression(unittest.TestCase):
    def test_fragmented_frame_is_decoded(self) -> None:
        frame = make_frame(5, 9, 123456)
        parser = guided.FlogStreamParser()
        output = []
        for chunk in (frame[:3], frame[3:17], frame[17:61], frame[61:]):
            output.extend(parser.feed(chunk))
        self.assertEqual(len(output), 1)
        self.assertEqual(output[0].raw, frame)
        self.assertEqual(output[0].sample.sample_sequence, 9)
        self.assertEqual(output[0].sample.validity_flags, 7)
        self.assertAlmostEqual(output[0].sample.temperature_c, 32.5)
        self.assertEqual(parser.stats.crc_errors, 0)

    def test_corruption_resync_and_sequence_gap_are_reported(self) -> None:
        first = make_frame(10, 10, 10_000)
        corrupt = bytearray(make_frame(11, 11, 11_000))
        corrupt[-1] ^= 0x80
        last = make_frame(13, 13, 13_000)
        parser = guided.FlogStreamParser()
        output = parser.feed(b"noise" + first + bytes(corrupt) + last)
        self.assertEqual([item.sample.sample_sequence for item in output], [10, 13])
        self.assertEqual(parser.stats.crc_errors, 1)
        # The first corrupt-frame byte is accounted as a CRC error; the remainder is resync waste.
        self.assertGreaterEqual(
            parser.stats.resync_bytes, len(b"noise") + len(corrupt) - 1
        )
        self.assertEqual(parser.stats.sequence_gaps, 2)
        self.assertEqual(parser.stats.sample_sequence_gaps, 2)
        self.assertEqual(parser.stats.timestamp_interval_outliers, 1)


class GuidedSessionRegression(unittest.TestCase):
    def test_temperature_model_matches_firmware_header(self) -> None:
        header = (
            ROOT
            / "Modules"
            / "modules_BMI088"
            / "bmi088_temperature_calibration_generated.h"
        ).read_text(encoding="utf-8")

        def scalar(name: str) -> float:
            match = re.search(rf"{name}\s*=\s*([^;]+)f;", header)
            self.assertIsNotNone(match)
            return float(match.group(1))

        def vector(name: str) -> np.ndarray:
            match = re.search(rf"{name}\[3\]\s*=\s*\{{(.*?)\}};", header, re.S)
            self.assertIsNotNone(match)
            return np.asarray(
                [
                    float(value)
                    for value in re.findall(r"[-+0-9.eE]+(?=f)", match.group(1))
                ]
            )

        self.assertEqual(
            scalar("bmi088_gyro_temp_reference_c"),
            temperature_compensation.GYRO_TEMP_REFERENCE_C,
        )
        self.assertEqual(
            scalar("bmi088_gyro_temp_minimum_c"),
            temperature_compensation.GYRO_TEMP_MINIMUM_C,
        )
        self.assertEqual(
            scalar("bmi088_gyro_temp_maximum_c"),
            temperature_compensation.GYRO_TEMP_MAXIMUM_C,
        )
        np.testing.assert_array_equal(
            vector("bmi088_gyro_temp_c1_rps_per_c"),
            temperature_compensation.GYRO_TEMP_C1_RPS_PER_C,
        )
        np.testing.assert_array_equal(
            vector("bmi088_gyro_temp_c2_rps_per_c2"),
            temperature_compensation.GYRO_TEMP_C2_RPS_PER_C2,
        )

    def test_temperature_compensation_is_clamped_and_vectorized(self) -> None:
        temperatures = np.array(
            [
                temperature_compensation.GYRO_TEMP_MINIMUM_C - 10.0,
                temperature_compensation.GYRO_TEMP_REFERENCE_C,
                temperature_compensation.GYRO_TEMP_MAXIMUM_C + 10.0,
            ]
        )
        corrections = temperature_compensation.gyro_temperature_correction_rps(
            temperatures
        )
        self.assertEqual(corrections.shape, (3, 3))
        np.testing.assert_array_equal(corrections[1], np.zeros(3))
        np.testing.assert_array_equal(
            corrections[0],
            temperature_compensation.gyro_temperature_correction_rps(
                temperature_compensation.GYRO_TEMP_MINIMUM_C
            ),
        )
        np.testing.assert_array_equal(
            corrections[2],
            temperature_compensation.gyro_temperature_correction_rps(
                temperature_compensation.GYRO_TEMP_MAXIMUM_C
            ),
        )

    def test_live_session_uses_temperature_compensated_gyro(self) -> None:
        criteria = guided.GuidedCriteria(
            hold_seconds=0.1, evaluation_window_seconds=0.02
        )
        target = guided.TARGET_POSES[0]
        session = guided.GuidedLmSession(criteria=criteria, targets=(target,))
        temperature_c = 32.5
        raw_gyro = temperature_compensation.gyro_temperature_correction_rps(
            temperature_c
        )
        for sequence in range(40):
            session.process(
                sample(
                    (sequence + 1) * 5_000,
                    sequence,
                    guided.GRAVITY_MPS2 * np.asarray(target.direction),
                    raw_gyro,
                    temperature_c=temperature_c,
                )
            )

        self.assertEqual(session.state, "complete")
        np.testing.assert_allclose(
            session.pose_results[0].mean_gyro_rps, 0.0, atol=1e-15
        )

    def test_live_session_rejects_missing_temperature(self) -> None:
        session = guided.GuidedLmSession(
            criteria=guided.GuidedCriteria(
                hold_seconds=0.1, evaluation_window_seconds=0.02
            ),
            targets=(guided.TARGET_POSES[0],),
        )
        target = guided.GRAVITY_MPS2 * np.asarray(session.targets[0].direction)
        session.process(sample(5_000, 1, target, np.zeros(3), validity_flags=3))
        self.assertEqual(session.target_index, 0)
        self.assertIn("温度无效", session.live_status.reason)

    def test_temperature_monitor_requires_history_and_detects_stability(self) -> None:
        monitor = guided.TemperatureMonitor(
            window_seconds=120.0,
            minimum_window_seconds=90.0,
            maximum_range_c=0.5,
            maximum_abs_slope_c_per_min=0.1,
        )
        for second in range(121):
            monitor.process(
                sample(
                    (second + 1) * 1_000_000,
                    second,
                    np.array([0.0, 0.0, -guided.GRAVITY_MPS2]),
                    np.zeros(3),
                    temperature_c=35.0 + 0.0005 * second,
                )
            )
        self.assertTrue(monitor.status.valid)
        self.assertTrue(monitor.status.stable, monitor.status)
        self.assertLess(monitor.status.range_c, 0.1)

    def test_temperature_monitor_hysteresis_ignores_small_excursion(self) -> None:
        monitor = guided.TemperatureMonitor(
            window_seconds=8.0,
            minimum_window_seconds=3.0,
            maximum_range_c=0.5,
            maximum_abs_slope_c_per_min=10.0,
            entry_confirmation_seconds=2.0,
            exit_maximum_range_c=0.75,
            exit_maximum_abs_slope_c_per_min=10.0,
            exit_confirmation_seconds=3.0,
        )
        for second in range(7):
            status = monitor.process(
                sample(
                    (second + 1) * 1_000_000,
                    second,
                    np.array([0.0, 0.0, -guided.GRAVITY_MPS2]),
                    np.zeros(3),
                    temperature_c=35.0,
                )
            )
        self.assertTrue(status.stable)

        status = monitor.process(
            sample(
                8_000_000,
                7,
                np.array([0.0, 0.0, -guided.GRAVITY_MPS2]),
                np.zeros(3),
                temperature_c=36.0,
            )
        )
        self.assertTrue(status.stable)
        self.assertIn("迟滞保护", status.reason)

    def test_temperature_monitor_exits_after_sustained_violation_and_recovers(
        self,
    ) -> None:
        monitor = guided.TemperatureMonitor(
            window_seconds=8.0,
            minimum_window_seconds=3.0,
            maximum_range_c=0.5,
            maximum_abs_slope_c_per_min=10.0,
            entry_confirmation_seconds=2.0,
            exit_maximum_range_c=0.75,
            exit_maximum_abs_slope_c_per_min=10.0,
            exit_confirmation_seconds=3.0,
        )
        accel = np.array([0.0, 0.0, -guided.GRAVITY_MPS2])
        for second in range(7):
            status = monitor.process(
                sample(
                    (second + 1) * 1_000_000,
                    second,
                    accel,
                    np.zeros(3),
                    temperature_c=35.0,
                )
            )
        self.assertTrue(status.stable)

        for second in range(7, 12):
            status = monitor.process(
                sample(
                    (second + 1) * 1_000_000,
                    second,
                    accel,
                    np.zeros(3),
                    temperature_c=36.0,
                )
            )
        self.assertFalse(status.stable)
        self.assertIn("持续超限", status.reason)

        for second in range(12, 25):
            status = monitor.process(
                sample(
                    (second + 1) * 1_000_000,
                    second,
                    accel,
                    np.zeros(3),
                    temperature_c=36.0,
                )
            )
        self.assertTrue(status.stable, status)

    def test_temperature_suspend_resets_partial_pose_hold(self) -> None:
        session = guided.GuidedLmSession(
            criteria=guided.GuidedCriteria(
                hold_seconds=0.3, evaluation_window_seconds=0.05
            ),
            targets=(guided.TARGET_POSES[0],),
        )
        target = guided.GRAVITY_MPS2 * np.asarray(session.targets[0].direction)
        timestamp_us = 0
        sequence = 0
        for _ in range(25):
            timestamp_us += 10_000
            sequence += 1
            session.process(sample(timestamp_us, sequence, target, np.zeros(3)))

        session.suspend_for_temperature("温度持续超限，已暂停姿态计时")
        self.assertEqual(session.live_status.hold_seconds, 0.0)
        self.assertIn("暂停", session.live_status.reason)
        for _ in range(25):
            timestamp_us += 10_000
            sequence += 1
            session.process(sample(timestamp_us, sequence, target, np.zeros(3)))
        self.assertEqual(session.target_index, 0)

        for _ in range(20):
            timestamp_us += 10_000
            sequence += 1
            session.process(sample(timestamp_us, sequence, target, np.zeros(3)))
        self.assertEqual(session.state, "complete")

    def test_temperature_monitor_rejects_missing_validity_flag(self) -> None:
        monitor = guided.TemperatureMonitor()
        status = monitor.process(
            sample(
                1_000_000,
                1,
                np.array([0.0, 0.0, -guided.GRAVITY_MPS2]),
                np.zeros(3),
                validity_flags=3,
            )
        )
        self.assertFalse(status.valid)
        self.assertIn("未提供有效", status.reason)

    def test_wrong_pose_and_motion_do_not_advance_hold(self) -> None:
        session = guided.GuidedLmSession(
            criteria=guided.GuidedCriteria(
                hold_seconds=0.3, evaluation_window_seconds=0.05
            )
        )
        timestamp_us = 0
        wrong = guided.GRAVITY_MPS2 * np.array([1.0, 0.0, 0.0])
        target = guided.GRAVITY_MPS2 * np.asarray(session.targets[0].direction)
        for sequence in range(40):
            timestamp_us += 10_000
            session.process(sample(timestamp_us, sequence, wrong, np.zeros(3)))
        self.assertEqual(session.target_index, 0)
        for sequence in range(40, 80):
            timestamp_us += 10_000
            session.process(
                sample(timestamp_us, sequence, target, np.array([0.1, 0.0, 0.0]))
            )
        self.assertEqual(session.target_index, 0)

    def test_acceleration_shake_does_not_advance_hold(self) -> None:
        session = guided.GuidedLmSession(
            criteria=guided.GuidedCriteria(
                hold_seconds=0.3, evaluation_window_seconds=0.05
            )
        )
        target = guided.GRAVITY_MPS2 * np.asarray(session.targets[0].direction)
        for sequence in range(80):
            shake = np.array([0.25 if sequence % 2 else -0.25, 0.0, 0.0])
            session.process(
                sample((sequence + 1) * 10_000, sequence, target + shake, np.zeros(3))
            )
        self.assertEqual(session.target_index, 0)
        self.assertIn("抖动", session.live_status.reason)

    def test_brief_hand_wobble_pauses_without_losing_hold_progress(self) -> None:
        criteria = guided.GuidedCriteria(
            hold_seconds=0.3,
            hold_dropout_grace_seconds=0.15,
            evaluation_window_seconds=0.02,
        )
        target_pose = guided.TARGET_POSES[0]
        session = guided.GuidedLmSession(criteria=criteria, targets=(target_pose,))
        target = guided.GRAVITY_MPS2 * np.asarray(target_pose.direction)
        wrong = guided.GRAVITY_MPS2 * np.array([1.0, 0.0, 0.0])
        timestamp_us = 0
        sequence = 0
        while session.live_status.hold_seconds < 0.12:
            timestamp_us += 5_000
            sequence += 1
            session.process(sample(timestamp_us, sequence, target, np.zeros(3)))
        progress_before_wobble = session.live_status.hold_seconds

        for _ in range(12):
            timestamp_us += 5_000
            sequence += 1
            session.process(sample(timestamp_us, sequence, wrong, np.zeros(3)))
        self.assertGreaterEqual(
            session.live_status.hold_seconds, progress_before_wobble
        )
        self.assertIn("进度已暂停", session.live_status.reason)

        for _ in range(100):
            timestamp_us += 5_000
            sequence += 1
            session.process(sample(timestamp_us, sequence, target, np.zeros(3)))
            if session.state == "complete":
                break
        self.assertEqual(session.state, "complete")

    def test_prolonged_hand_wobble_resets_hold_progress(self) -> None:
        criteria = guided.GuidedCriteria(
            hold_seconds=0.3,
            hold_dropout_grace_seconds=0.05,
            evaluation_window_seconds=0.02,
        )
        target_pose = guided.TARGET_POSES[0]
        session = guided.GuidedLmSession(criteria=criteria, targets=(target_pose,))
        target = guided.GRAVITY_MPS2 * np.asarray(target_pose.direction)
        wrong = guided.GRAVITY_MPS2 * np.array([1.0, 0.0, 0.0])
        timestamp_us = 0
        sequence = 0
        while session.live_status.hold_seconds < 0.12:
            timestamp_us += 5_000
            sequence += 1
            session.process(sample(timestamp_us, sequence, target, np.zeros(3)))
        for _ in range(30):
            timestamp_us += 5_000
            sequence += 1
            session.process(sample(timestamp_us, sequence, wrong, np.zeros(3)))
        self.assertEqual(session.live_status.hold_seconds, 0.0)

    def test_bias_offset_is_reported_but_does_not_block_pose(self) -> None:
        criteria = guided.GuidedCriteria(
            hold_seconds=0.08,
            evaluation_window_seconds=0.02,
            hold_dropout_grace_seconds=0.02,
        )
        session = guided.GuidedLmSession(
            criteria=criteria, targets=guided.TARGET_POSES[:2]
        )
        timestamp_us = 0
        sequence = 0
        temperature_c = 32.5
        correction = temperature_compensation.gyro_temperature_correction_rps(
            temperature_c
        )
        first = guided.GRAVITY_MPS2 * np.asarray(session.targets[0].direction)
        second = guided.GRAVITY_MPS2 * np.asarray(session.targets[1].direction)
        while session.target_index == 0:
            timestamp_us += 5_000
            sequence += 1
            session.process(
                sample(timestamp_us, sequence, first, correction, temperature_c)
            )

        slow_rotation = correction + np.radians(np.array([0.20, 0.0, 0.0]))
        for _ in range(10):
            timestamp_us += 5_000
            sequence += 1
            session.process(
                sample(
                    timestamp_us,
                    sequence,
                    second,
                    slow_rotation,
                    temperature_c,
                )
            )
        self.assertEqual(session.target_index, 1)
        self.assertTrue(session.live_status.qualified)
        self.assertGreater(
            session.live_status.gyro_bias_residual_rps,
            math.radians(0.12),
        )

        for _ in range(80):
            timestamp_us += 5_000
            sequence += 1
            session.process(
                sample(
                    timestamp_us,
                    sequence,
                    second,
                    slow_rotation,
                    temperature_c,
                )
            )
            if session.state == "complete":
                break
        self.assertEqual(session.state, "complete")

    def test_group_temperature_budget_blocks_next_pose(self) -> None:
        criteria = guided.GuidedCriteria(
            hold_seconds=0.08, evaluation_window_seconds=0.02
        )
        session = guided.GuidedLmSession(
            criteria=criteria, targets=guided.TARGET_POSES[:2]
        )
        timestamp_us = 0
        sequence = 0
        first_temperature_c = 32.5
        first_correction = temperature_compensation.gyro_temperature_correction_rps(
            first_temperature_c
        )
        first = guided.GRAVITY_MPS2 * np.asarray(session.targets[0].direction)
        second = guided.GRAVITY_MPS2 * np.asarray(session.targets[1].direction)
        while session.target_index == 0:
            timestamp_us += 5_000
            sequence += 1
            session.process(
                sample(
                    timestamp_us,
                    sequence,
                    first,
                    first_correction,
                    first_temperature_c,
                )
            )

        second_temperature_c = first_temperature_c + 1.05
        second_correction = temperature_compensation.gyro_temperature_correction_rps(
            second_temperature_c
        )
        for _ in range(80):
            timestamp_us += 5_000
            sequence += 1
            session.process(
                sample(
                    timestamp_us,
                    sequence,
                    second,
                    second_correction,
                    second_temperature_c,
                )
            )
        self.assertEqual(session.target_index, 1)
        self.assertFalse(session.live_status.qualified)
        self.assertTrue(session.live_status.temperature_budget_warning)
        self.assertEqual(session.live_status.temperature_budget_remaining_c, 0.0)
        self.assertGreater(
            session.live_status.pose_temperature_span_c,
            criteria.maximum_pose_temperature_span_c,
        )

    def test_transition_over_30_seconds_remains_usable_below_safety_limit(
        self,
    ) -> None:
        criteria = guided.GuidedCriteria(
            hold_seconds=0.05,
            evaluation_window_seconds=0.02,
            recommended_transition_seconds=30.0,
            maximum_transition_seconds=90.0,
        )
        session = guided.GuidedLmSession(
            criteria=criteria, targets=guided.TARGET_POSES[:2]
        )
        timestamp_us = 0
        sequence = 0
        first = guided.GRAVITY_MPS2 * np.asarray(session.targets[0].direction)
        second = guided.GRAVITY_MPS2 * np.asarray(session.targets[1].direction)
        correction = temperature_compensation.gyro_temperature_correction_rps(32.5)
        while session.target_index == 0:
            timestamp_us += 10_000
            sequence += 1
            session.process(sample(timestamp_us, sequence, first, correction))

        for _ in range(700):
            timestamp_us += 50_000
            sequence += 1
            session.process(sample(timestamp_us, sequence, first, correction))
        for fraction in np.linspace(0.0, 1.0, 101):
            timestamp_us += 10_000
            sequence += 1
            direction = (1.0 - fraction) * first + fraction * second
            direction *= guided.GRAVITY_MPS2 / np.linalg.norm(direction)
            session.process(
                sample(
                    timestamp_us,
                    sequence,
                    direction,
                    correction + np.array([1.0, 0.9, 0.8]),
                )
            )
        while session.state != "complete":
            timestamp_us += 10_000
            sequence += 1
            session.process(sample(timestamp_us, sequence, second, correction))

        transition = session.transition_results[0]
        self.assertGreater(transition.duration_s, 30.0)
        self.assertLess(transition.duration_s, 90.0)
        self.assertTrue(transition.usable)

    def test_full_balanced_route_passes_all_quality_checks(self) -> None:
        criteria = guided.GuidedCriteria(
            hold_seconds=0.25, evaluation_window_seconds=0.05
        )
        session = guided.GuidedLmSession(criteria=criteria)
        timestamp_us = 0
        sequence = 0
        bias = np.array([0.001, -0.002, 0.0005])

        for pose_index, target in enumerate(session.targets):
            if pose_index:
                for fraction in np.linspace(0.0, 1.0, 101):
                    timestamp_us += 10_000
                    sequence += 1
                    direction = (1.0 - fraction) * np.asarray(
                        session.targets[pose_index - 1].direction
                    )
                    direction += fraction * np.asarray(target.direction)
                    direction /= np.linalg.norm(direction)
                    gyro = bias + np.array([1.0, 0.9, 0.8])
                    session.process(
                        sample(
                            timestamp_us,
                            sequence,
                            guided.GRAVITY_MPS2 * direction,
                            gyro,
                        )
                    )
            for _ in range(40):
                timestamp_us += 10_000
                sequence += 1
                session.process(
                    sample(
                        timestamp_us,
                        sequence,
                        guided.GRAVITY_MPS2 * np.asarray(target.direction),
                        bias,
                    )
                )

        self.assertEqual(session.state, "complete")
        report = session.build_report(
            guided.ParserStats(frames=sequence, imu_samples=sequence)
        )
        self.assertTrue(report["overall_pass"], report["checks"])
        self.assertAlmostEqual(
            report["metrics"]["coverage_min_eigenvalue"], 1.0 / 3.0, places=6
        )
        self.assertEqual(len(report["transitions"]), len(session.targets) - 1)
        temperature_check = next(
            item for item in report["checks"] if item["name"] == "temperature_valid"
        )
        self.assertTrue(temperature_check["pass"])

    def test_bias_drift_report_uses_six_axial_poses(self) -> None:
        session = guided.GuidedLmSession()
        stable_bias = np.radians(np.array([-0.06, -0.31, -0.055]))
        axial_codes = {"-X", "+X", "-Y", "+Y", "-Z", "+Z"}
        for index, target in enumerate(session.targets):
            gyro = stable_bias.copy()
            if target.code not in axial_codes:
                gyro += np.radians(np.array([0.30, -0.25, 0.20]))
            session.pose_results.append(
                guided.PoseResult(
                    code=target.code,
                    label=target.label,
                    target_direction=list(target.direction),
                    mean_accel_mps2=(
                        guided.GRAVITY_MPS2 * np.asarray(target.direction)
                    ).tolist(),
                    mean_gyro_rps=gyro.tolist(),
                    mean_temperature_c=40.0 + 0.01 * index,
                    accepted_timestamp_us=(index + 1) * 1_000_000,
                    samples=3_000,
                    angle_error_deg=0.0,
                )
            )
        report = session.build_report(guided.ParserStats())
        checks = {item["name"]: item for item in report["checks"]}
        self.assertTrue(checks["gyro_bias_drift_dps"]["pass"])
        self.assertEqual(
            set(report["metrics"]["gyro_bias_drift_pose_codes"]), axial_codes
        )

    def test_link_error_and_bias_drift_fail_report(self) -> None:
        criteria = guided.GuidedCriteria(
            hold_seconds=0.1,
            evaluation_window_seconds=0.02,
        )
        session = guided.GuidedLmSession(
            criteria=criteria, targets=guided.TARGET_POSES[:2]
        )
        timestamp_us = 0
        sequence = 0
        for pose_index, target in enumerate(session.targets):
            for _ in range(40):
                timestamp_us += 5_000
                sequence += 1
                gyro = np.array([math.radians(0.2 * pose_index), 0.0, 0.0])
                session.process(
                    sample(
                        timestamp_us,
                        sequence,
                        guided.GRAVITY_MPS2 * np.asarray(target.direction),
                        gyro,
                    )
                )
        report = session.build_report(
            guided.ParserStats(crc_errors=1, sequence_gaps=2, sample_sequence_gaps=2)
        )
        checks = {item["name"]: item for item in report["checks"]}
        self.assertFalse(checks["gyro_bias_drift_dps"]["pass"])
        self.assertFalse(checks["usb_link_errors"]["pass"])
        self.assertEqual(checks["usb_link_errors"]["value"]["sample_sequence_gaps"], 2)
        self.assertEqual(checks["usb_link_errors"]["value"]["crc_errors"], 1)

    def test_transition_timer_tracks_elapsed_time(self) -> None:
        criteria = guided.GuidedCriteria(
            hold_seconds=0.1, evaluation_window_seconds=0.02
        )
        session = guided.GuidedLmSession(
            criteria=criteria, targets=guided.TARGET_POSES[:2]
        )
        timestamp_us = 0
        sequence = 0
        first = guided.GRAVITY_MPS2 * np.asarray(session.targets[0].direction)
        while session.target_index == 0:
            timestamp_us += 5_000
            sequence += 1
            session.process(sample(timestamp_us, sequence, first, np.zeros(3)))
        self.assertEqual(session.target_index, 1)
        initial_transition_seconds = session.live_status.transition_seconds

        for _ in range(200):
            timestamp_us += 5_000
            sequence += 1
            session.process(sample(timestamp_us, sequence, first, np.zeros(3)))
        self.assertAlmostEqual(
            session.live_status.transition_seconds - initial_transition_seconds,
            1.0,
            places=2,
        )


class GuidedWorkerRegression(unittest.TestCase):
    def test_lm_pipeline_enables_temperature_compensation(self) -> None:
        events = queue.Queue()
        with tempfile.TemporaryDirectory() as directory, mock.patch.object(
            gui.subprocess, "run"
        ) as run:
            capture = Path(directory) / "guided.BIN"
            gui.run_lm_pipeline(capture, events)

        self.assertEqual(run.call_count, 2)
        extract_command = run.call_args_list[0].args[0]
        lm_command = run.call_args_list[1].args[0]
        self.assertNotIn("--gyro-temperature-compensation", extract_command)
        self.assertIn("--gyro-temperature-compensation", lm_command)
        self.assertIn("--guided-report", lm_command)
        self.assertIn(str(capture.with_suffix(".guided_report.json")), lm_command)
        self.assertEqual(events.get_nowait()["type"], "lm_complete")

    def test_completed_session_closes_file_and_emits_report(self) -> None:
        criteria = guided.GuidedCriteria(
            hold_seconds=0.25, evaluation_window_seconds=0.05
        )
        session = guided.GuidedLmSession(criteria=criteria)
        parser = guided.FlogStreamParser()
        events = queue.Queue()
        args = argparse.Namespace(
            port=None,
            serial_number=None,
            baudrate=115200,
            reconnect_delay=1.0,
            silence_timeout=3.0,
            sync_period=5.0,
        )
        worker = gui.UsbGuidedWorker(args, events)
        worker.thermal_monitor = guided.TemperatureMonitor(
            window_seconds=1.0,
            minimum_window_seconds=0.0,
            entry_confirmation_seconds=0.0,
        )
        frames = []
        timestamp_us = 0
        sequence = 0
        bias = np.array([0.001, -0.002, 0.0005])
        for pose_index, target in enumerate(session.targets):
            if pose_index:
                for fraction in np.linspace(0.0, 1.0, 101):
                    timestamp_us += 10_000
                    sequence += 1
                    direction = (1.0 - fraction) * np.asarray(
                        session.targets[pose_index - 1].direction
                    )
                    direction += fraction * np.asarray(target.direction)
                    direction /= np.linalg.norm(direction)
                    frames.append(
                        guided.ParsedFrame(
                            raw=b"frame",
                            sample=sample(
                                timestamp_us,
                                sequence,
                                guided.GRAVITY_MPS2 * direction,
                                bias + np.array([1.0, 0.9, 0.8]),
                            ),
                        )
                    )
            for _ in range(40):
                timestamp_us += 10_000
                sequence += 1
                frames.append(
                    guided.ParsedFrame(
                        raw=b"frame",
                        sample=sample(
                            timestamp_us,
                            sequence,
                            guided.GRAVITY_MPS2 * np.asarray(target.direction),
                            bias,
                        ),
                    )
                )
        parser.stats.frames = len(frames)
        parser.stats.imu_samples = len(frames)

        with tempfile.TemporaryDirectory() as directory:
            capture_path = Path(directory) / "guided.BIN"
            output = capture_path.open("wb")
            worker.record_requested = True
            output_after, session_after, _ = worker._consume_frames(
                frames, parser, output, capture_path, session, 24
            )
            report_path = capture_path.with_suffix(".guided_report.json")
            report = json.loads(report_path.read_text(encoding="utf-8"))

        self.assertIsNone(output_after)
        self.assertIsNone(session_after)
        self.assertFalse(worker.record_requested)
        self.assertTrue(report["overall_pass"])
        complete_events = [
            event for event in list(events.queue) if event["type"] == "complete"
        ]
        self.assertEqual(len(complete_events), 1)


if __name__ == "__main__":
    unittest.main()
