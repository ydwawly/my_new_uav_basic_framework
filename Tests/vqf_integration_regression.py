#!/usr/bin/env python3
"""Regression checks for the VQF firmware integration and frame conversion."""

from __future__ import annotations

import hashlib
import math
import struct
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
VQF_DIR = ROOT / "Modules" / "modules_Algorithm" / "VQF"
ATTITUDE_DIR = ROOT / "Application" / "App_attitude"

UPSTREAM_HASHES = {
    "vqf.cpp": "53874145a2d8a3b7670bbf844f5c9c96644bd6d7e5f9fc8c0d7865bfdac34215",
    "vqf.hpp": "7d9e43427fdd7bccaed40cc81c450a4ee3995409c5dac4a37a143e5e6a123e1c",
}


def quat_multiply(lhs: tuple[float, ...], rhs: tuple[float, ...]) -> tuple[float, ...]:
    lw, lx, ly, lz = lhs
    rw, rx, ry, rz = rhs
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def enu_to_zero_heading_ned(quat_enu: tuple[float, ...]) -> tuple[float, ...]:
    inv_sqrt_two = math.sqrt(0.5)
    raw_ned = quat_multiply((0.0, inv_sqrt_two, inv_sqrt_two, 0.0), quat_enu)
    yaw = math.atan2(
        2.0 * (raw_ned[0] * raw_ned[3] + raw_ned[1] * raw_ned[2]),
        1.0 - 2.0 * (raw_ned[2] ** 2 + raw_ned[3] ** 2),
    )
    correction = (math.cos(-0.5 * yaw), 0.0, 0.0, math.sin(-0.5 * yaw))
    result = quat_multiply(correction, raw_ned)
    norm = math.sqrt(sum(value * value for value in result))
    return tuple(value / norm for value in result)


def quaternion_to_euler_deg(quaternion: tuple[float, ...]) -> tuple[float, ...]:
    w, x, y, z = quaternion
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def assert_euler(actual: tuple[float, ...], expected: tuple[float, ...], tolerance_deg: float = 0.02) -> None:
    for axis, (measured, target) in enumerate(zip(actual, expected, strict=True)):
        assert abs(measured - target) <= tolerance_deg, (axis, measured, target)


def test_upstream_sources_are_unmodified() -> None:
    for filename, expected_hash in UPSTREAM_HASHES.items():
        digest = hashlib.sha256((VQF_DIR / filename).read_bytes()).hexdigest()
        assert digest == expected_hash, f"{filename} differs from pinned upstream commit"


def test_vqf_preserves_nan_sentinel_semantics() -> None:
    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    assert cmake.count("-fno-finite-math-only") == 2
    assert 'COMPILE_OPTIONS "-fno-exceptions;-fno-rtti;-fno-fast-math"' in cmake


def test_real_path_keeps_vqf_as_control_source() -> None:
    attitude_source = (ATTITUDE_DIR / "App_attitude.c").read_text(encoding="utf-8")
    observation_source = (ATTITUDE_DIR / "App_attitude_observations.c").read_text(encoding="utf-8")
    attitude_header = (ATTITUDE_DIR / "App_attitude.h").read_text(encoding="utf-8")
    config = (ATTITUDE_DIR / "App_attitude_config.h").read_text(encoding="utf-8")

    assert "AttitudeControl_" not in attitude_source
    assert "Control_Attitude.h" not in attitude_header
    assert not (ATTITUDE_DIR / "Control_Attitude.c").exists()
    assert not (ATTITUDE_DIR / "Control_Attitude.h").exists()
    assert "Real sensor VQF 6D task started; ESKF diagnostic only." in attitude_source
    assert "VqfC_Update(sample->gyro_rps" in attitude_source
    assert "NAV_ESKF_Predict(&nav_eskf_instance, &prediction)" in attitude_source
    assert "NAV_ESKF_UpdateGravity(&nav_eskf_instance, &prediction)" in attitude_source
    assert "attitude_vqf_debug_snapshot = attitude_runtime.vqf_output;" in attitude_source
    assert "output->control_q_nb = output->state.q_nb;" in attitude_source
    assert "Attitude_UpdateEstimatorComparisonSnapshot" in attitude_source
    assert "ATTITUDE_ESTIMATE_TOPIC_NAME" not in attitude_header
    assert "estimate_publisher" not in attitude_source
    assert "App_Attitude_Observations_Update" in attitude_source
    assert "NAV_ESKF_UpdateFlow" in observation_source
    assert "NAV_ESKF_UpdateRange" in observation_source
    assert "#define ATTITUDE_PARALLEL_ESKF_ENABLE 1U" in config
    assert "#define ATTITUDE_ENABLE_FLOW 1U" in config
    assert "#define ATTITUDE_ENABLE_RANGE 1U" in config
    assert "ATTITUDE_MTF02_ENABLE_RANGE_FUSION" not in config
    assert "ATTITUDE_ENABLE_MAG" not in config
    assert "ATTITUDE_ENABLE_BARO" not in config


def test_estimator_comparison_log_layout_is_versioned() -> None:
    sd_header = (ROOT / "Modules/modules_SD_Card/modules_SD_Card.h").read_text(encoding="utf-8")
    log_service = (ROOT / "Application/App_Data_Comm/log_service.c").read_text(encoding="utf-8")
    analyzer = (
        ROOT.parents[1]
        / "uav_control_upper_computer/artifacts/sd-downloads/analyze_flog.py"
    ).read_text(encoding="utf-8")

    assert "SD_CARD_MSG_ESTIMATOR_COMPARISON = 6U" in sd_header
    assert "sizeof(SDCard_EstimatorComparisonPayloadV1_t) == 96U" in sd_header
    assert "sizeof(SDCard_EstimatorComparisonPayloadV2_t) == 200U" in sd_header
    assert "SDCard_EnqueueFrame(SD_CARD_MSG_ESTIMATOR_COMPARISON, 2U" in log_service
    assert "ATTITUDE_ESTIMATOR_COMPARISON_LOG_INTERVAL_US 20000ULL" in (
        ROOT / "Application/App_attitude/App_attitude_config.h"
    ).read_text(encoding="utf-8")
    assert struct.calcsize("<8I15f4B") == 96
    assert struct.calcsize("<8I15f4B16f8IH6B") == 200
    assert 'ESTIMATOR_PAYLOAD_V1 = struct.Struct("<8I15f4B")' in analyzer
    assert 'ESTIMATOR_PAYLOAD_V2 = struct.Struct("<8I15f4B16f8IH6B")' in analyzer
    assert "def decode_estimator_comparison" in analyzer


def test_frd_ned_tilt_signs_and_heading_zero() -> None:
    # Representative steady-state getQuat6D() outputs from official PyVQF at 1 kHz.
    level = enu_to_zero_heading_ned((0.0, 1.0, 0.0, 0.0))
    roll_positive = enu_to_zero_heading_ned((0.087155767653, -0.996194695913, 0.0, 0.0))
    pitch_positive = enu_to_zero_heading_ned((0.087155767653, 0.0, -0.996194695913, 0.0))

    assert_euler(quaternion_to_euler_deg(level), (0.0, 0.0, 0.0))
    assert_euler(quaternion_to_euler_deg(roll_positive), (10.0, 0.0, 0.0))
    assert_euler(quaternion_to_euler_deg(pitch_positive), (0.0, 10.0, 0.0))


if __name__ == "__main__":
    test_upstream_sources_are_unmodified()
    test_vqf_preserves_nan_sentinel_semantics()
    test_real_path_keeps_vqf_as_control_source()
    test_estimator_comparison_log_layout_is_versioned()
    test_frd_ned_tilt_signs_and_heading_zero()
    print("VQF integration regression checks passed.")
