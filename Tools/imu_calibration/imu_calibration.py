#!/usr/bin/env python3
"""BMI088 black-box extraction, LM fixed calibration and Allan analysis."""

from __future__ import annotations

import argparse
import csv
import json
import math
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares

from bmi088_temperature_compensation import (
    compensate_gyro_rps,
    temperature_compensation_metadata,
)


FILE_MAGIC = 0x474F4C46
FRAME_MAGIC = 0xA55A
IMU_MESSAGE_ID = 3
FILE_HEADER = struct.Struct("<IHHQIHH")
FRAME_HEADER = struct.Struct("<HBBHHIQ")
IMU_PAYLOAD_V1 = struct.Struct("<II3h3hhH3f3ff")
GRAVITY_MPS2 = 9.80665
MAX_GYRO_TRANSITION_SECONDS = 90.0
MAX_GYRO_TRANSITION_POINTS = 5_000
GYRO_LM_TARGET_RATE_HZ = 250.0
CSV_COLUMNS = (
    "timestamp_us",
    "sample_sequence",
    "accel_sensor_time",
    "validity_flags",
    "accel_raw_x",
    "accel_raw_y",
    "accel_raw_z",
    "gyro_raw_x",
    "gyro_raw_y",
    "gyro_raw_z",
    "temperature_raw",
    "accel_x_mps2",
    "accel_y_mps2",
    "accel_z_mps2",
    "gyro_x_rps",
    "gyro_y_rps",
    "gyro_z_rps",
    "temperature_c",
)


def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    crc = initial
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (
                ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
            )
    return crc


def extract_blackbox(source: Path, destination: Path) -> dict[str, int]:
    blob = source.read_bytes()
    if len(blob) < FILE_HEADER.size:
        raise ValueError("日志短于 24 字节文件头")
    magic, version, header_size, *_ = FILE_HEADER.unpack_from(blob)
    if magic != FILE_MAGIC or version != 1 or header_size < FILE_HEADER.size:
        raise ValueError("不是受支持的 FLOG v1 文件")
    header_crc = FILE_HEADER.unpack_from(blob)[-2]
    if crc16_ccitt(blob[:20]) != header_crc:
        raise ValueError("FLOG 文件头 CRC 错误")

    stats = {"frames": 0, "imu_samples": 0, "crc_errors": 0, "resync_bytes": 0}
    destination.parent.mkdir(parents=True, exist_ok=True)
    offset = header_size
    magic_bytes = struct.pack("<H", FRAME_MAGIC)
    with destination.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(CSV_COLUMNS)
        while offset + FRAME_HEADER.size + 2 <= len(blob):
            if blob[offset : offset + 2] != magic_bytes:
                next_offset = blob.find(magic_bytes, offset + 1)
                if next_offset < 0:
                    stats["resync_bytes"] += len(blob) - offset
                    break
                stats["resync_bytes"] += next_offset - offset
                offset = next_offset
            header = FRAME_HEADER.unpack_from(blob, offset)
            _, message_id, message_version, payload_length, flags, _, timestamp_us = (
                header
            )
            frame_size = FRAME_HEADER.size + payload_length + 2
            if payload_length > 300 or offset + frame_size > len(blob):
                offset += 1
                continue
            frame_without_crc = blob[offset : offset + frame_size - 2]
            stored_crc = struct.unpack_from("<H", blob, offset + frame_size - 2)[0]
            if crc16_ccitt(frame_without_crc) != stored_crc:
                stats["crc_errors"] += 1
                offset += 1
                continue
            stats["frames"] += 1
            payload = blob[offset + FRAME_HEADER.size : offset + frame_size - 2]
            if (
                message_id == IMU_MESSAGE_ID
                and message_version == 1
                and len(payload) == IMU_PAYLOAD_V1.size
            ):
                fields = IMU_PAYLOAD_V1.unpack(payload)
                writer.writerow(
                    (
                        timestamp_us,
                        fields[0],
                        fields[1],
                        fields[9],
                        *fields[2:9],
                        *fields[10:],
                    )
                )
                stats["imu_samples"] += 1
            offset += frame_size
    return stats


def load_csv(path: Path) -> dict[str, np.ndarray]:
    table = np.genfromtxt(
        path, delimiter=",", names=True, dtype=np.float64, encoding="utf-8"
    )
    if table.size == 0:
        raise ValueError("CSV 中没有 BMI088 样本")
    table = np.atleast_1d(table)
    missing = set(CSV_COLUMNS) - set(table.dtype.names or ())
    if missing:
        raise ValueError(f"CSV 缺少列: {sorted(missing)}")
    result = {
        "timestamp_us": table["timestamp_us"].astype(np.uint64),
        "sequence": table["sample_sequence"].astype(np.uint32),
        "accel_sensor_time": table["accel_sensor_time"].astype(np.uint32),
        "accel_raw": np.column_stack(
            [table[f"accel_raw_{axis}"] for axis in "xyz"]
        ).astype(np.int16),
        "gyro_raw": np.column_stack(
            [table[f"gyro_raw_{axis}"] for axis in "xyz"]
        ).astype(np.int16),
        "accel": np.column_stack([table[f"accel_{axis}_mps2"] for axis in "xyz"]),
        "gyro": np.column_stack([table[f"gyro_{axis}_rps"] for axis in "xyz"]),
        "temperature": table["temperature_c"],
        "validity": table["validity_flags"].astype(np.uint16),
    }
    if np.any(np.diff(result["timestamp_us"].astype(np.int64)) <= 0):
        raise ValueError("时间戳必须严格递增")
    for name in ("accel", "gyro"):
        if not np.all(np.isfinite(result[name])):
            raise ValueError(f"{name} 数据包含 NaN 或无穷值")
    return result


def apply_gyro_temperature_compensation(
    data: dict[str, np.ndarray],
) -> dict[str, np.ndarray]:
    """Return calibration input with the validated gyro temperature model applied."""
    temperature = np.asarray(data["temperature"], dtype=np.float64)
    validity = np.asarray(data["validity"], dtype=np.uint16)
    invalid = ((validity & 0x4) == 0) | ~np.isfinite(temperature)
    if np.any(invalid):
        raise ValueError(
            f"有 {int(np.count_nonzero(invalid))} 个样本缺少有效温度，无法进行陀螺仪温漂补偿"
        )

    compensated = dict(data)
    compensated["gyro"] = compensate_gyro_rps(data["gyro"], temperature)
    return compensated


def contiguous_true_runs(mask: np.ndarray) -> list[tuple[int, int]]:
    changes = np.diff(np.r_[False, mask, False].astype(np.int8))
    return list(zip(np.flatnonzero(changes == 1), np.flatnonzero(changes == -1)))


def detect_static_segments(
    data: dict[str, np.ndarray], gyro_threshold: float, min_seconds: float
) -> list[tuple[int, int]]:
    timestamps = data["timestamp_us"].astype(np.float64) * 1e-6
    dt = float(np.median(np.diff(timestamps)))
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError("时间戳不是严格递增的有效序列")
    gyro_norm = np.linalg.norm(data["gyro"], axis=1)
    accel_norm = np.linalg.norm(data["accel"], axis=1)
    raw_static = (
        (gyro_norm < gyro_threshold)
        & (accel_norm > 0.65 * GRAVITY_MPS2)
        & (accel_norm < 1.35 * GRAVITY_MPS2)
    )
    window = max(3, int(round(0.20 / dt)))
    kernel = np.ones(window, dtype=np.float64) / window
    static = np.convolve(raw_static.astype(np.float64), kernel, mode="same") > 0.97
    min_samples = max(2, int(round(min_seconds / dt)))
    trim = int(round(0.20 / dt))
    return [
        (start + trim, end - trim)
        for start, end in contiguous_true_runs(static)
        if end - start >= min_samples + 2 * trim
    ]


def guided_static_segments(
    data: dict[str, np.ndarray], report_path: Path, min_seconds: float
) -> tuple[list[tuple[int, int]], list[str]]:
    """Recover the exact accepted pose windows from a guided-capture report."""
    report = json.loads(report_path.read_text(encoding="utf-8"))
    poses = report.get("poses")
    if not isinstance(poses, list) or len(poses) < 12:
        raise ValueError("引导报告至少需要 12 个已验收姿态")

    timestamps = data["timestamp_us"].astype(np.int64)
    median_dt_us = float(np.median(np.diff(timestamps)))
    segments: list[tuple[int, int]] = []
    pose_codes: list[str] = []
    previous_end = 0
    for pose in poses:
        accepted_timestamp_us = int(pose["accepted_timestamp_us"])
        sample_count = int(pose["samples"])
        end = int(np.searchsorted(timestamps, accepted_timestamp_us, side="right"))
        if end <= 0 or abs(int(timestamps[end - 1]) - accepted_timestamp_us) > max(
            2_000.0, 2.0 * median_dt_us
        ):
            raise ValueError(
                f"引导姿态 {pose['code']} 的验收时间戳不在标定 CSV 中"
            )
        start = end - sample_count
        if start < previous_end or start < 0:
            raise ValueError(f"引导姿态 {pose['code']} 的样本窗口重叠或越界")
        duration_s = (int(timestamps[end - 1]) - int(timestamps[start])) * 1e-6
        if duration_s < min_seconds:
            raise ValueError(
                f"引导姿态 {pose['code']} 只有 {duration_s:.2f} 秒，"
                f"少于要求的 {min_seconds:.2f} 秒"
            )

        mean_accel = np.mean(data["accel"][start:end], axis=0)
        direction = mean_accel / np.linalg.norm(mean_accel)
        target = np.asarray(pose["target_direction"], dtype=np.float64)
        target /= np.linalg.norm(target)
        angle_error_deg = math.degrees(
            math.acos(float(np.clip(np.dot(direction, target), -1.0, 1.0)))
        )
        if angle_error_deg > 8.0:
            raise ValueError(
                f"引导姿态 {pose['code']} 的 CSV 均值偏离目标 "
                f"{angle_error_deg:.2f}°，超过 8°"
            )
        segments.append((start, end))
        pose_codes.append(str(pose["code"]))
        previous_end = end
    return segments, pose_codes


def upper_matrix(params: np.ndarray) -> np.ndarray:
    return np.array(
        [
            [math.exp(params[0]), params[3], params[4]],
            [0.0, math.exp(params[1]), params[5]],
            [0.0, 0.0, math.exp(params[2])],
        ]
    )


def fit_accelerometer(
    pose_means: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, dict[str, float]]:
    if len(pose_means) < 12:
        raise ValueError(
            f"加速度计 LM 至少需要 12 个有效静止姿态，当前只有 {len(pose_means)} 个"
        )
    directions = pose_means / np.linalg.norm(pose_means, axis=1, keepdims=True)
    coverage_eigenvalues = np.linalg.eigvalsh(
        directions.T @ directions / len(directions)
    )
    if coverage_eigenvalues[0] < 0.04:
        raise ValueError("静止姿态方向覆盖不足；需要覆盖球面的正负 X/Y/Z 及斜向姿态")

    bias0 = 0.5 * (np.max(pose_means, axis=0) + np.min(pose_means, axis=0))
    half_span = 0.5 * (np.max(pose_means, axis=0) - np.min(pose_means, axis=0))
    diag0 = np.where(half_span > 0.3 * GRAVITY_MPS2, GRAVITY_MPS2 / half_span, 1.0)
    x0 = np.r_[bias0, np.log(diag0), np.zeros(3)]

    def residual(params: np.ndarray) -> np.ndarray:
        corrected = (upper_matrix(params[3:]) @ (pose_means - params[:3]).T).T
        return np.linalg.norm(corrected, axis=1) - GRAVITY_MPS2

    lower = np.r_[np.full(3, -5.0), np.log(np.full(3, 0.5)), np.full(3, -0.3)]
    upper = np.r_[np.full(3, 5.0), np.log(np.full(3, 1.5)), np.full(3, 0.3)]
    result = least_squares(
        residual,
        x0,
        bounds=(lower, upper),
        method="trf",
        loss="soft_l1",
        f_scale=0.03,
        max_nfev=4000,
    )
    if not result.success:
        raise RuntimeError(f"加速度计 LM 未收敛: {result.message}")
    matrix = upper_matrix(result.x[3:])
    before = np.linalg.norm(pose_means, axis=1) - GRAVITY_MPS2
    after = residual(result.x)
    metrics = {
        "pose_count": int(len(pose_means)),
        "coverage_min_eigenvalue": float(coverage_eigenvalues[0]),
        "norm_rmse_before_mps2": float(np.sqrt(np.mean(before**2))),
        "norm_rmse_after_mps2": float(np.sqrt(np.mean(after**2))),
        "max_abs_residual_mps2": float(np.max(np.abs(after))),
    }
    return result.x[:3], matrix, metrics


def rotate_vector(vector: np.ndarray, rotation_vector: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rotation_vector))
    if angle < 1e-12:
        return vector + np.cross(rotation_vector, vector)
    axis = rotation_vector / angle
    return (
        vector * math.cos(angle)
        + np.cross(axis, vector) * math.sin(angle)
        + axis * np.dot(axis, vector) * (1.0 - math.cos(angle))
    )


@dataclass
class RotationTransition:
    time_s: np.ndarray
    gyro_rps: np.ndarray
    gravity_start: np.ndarray
    gravity_end: np.ndarray


def integrate_gravity(
    transition: RotationTransition, matrix: np.ndarray, bias: np.ndarray
) -> np.ndarray:
    corrected = (matrix @ (transition.gyro_rps - bias).T).T
    dt = np.diff(transition.time_s)
    increments = -0.5 * (corrected[:-1] + corrected[1:]) * dt[:, None]
    gx, gy, gz = (float(value) for value in transition.gravity_start)
    for rx, ry, rz in increments:
        angle = math.sqrt(rx * rx + ry * ry + rz * rz)
        if angle < 1e-12:
            cross_x = ry * gz - rz * gy
            cross_y = rz * gx - rx * gz
            cross_z = rx * gy - ry * gx
            gx += cross_x
            gy += cross_y
            gz += cross_z
            continue
        ux = rx / angle
        uy = ry / angle
        uz = rz / angle
        cosine = math.cos(angle)
        sine = math.sin(angle)
        cross_x = uy * gz - uz * gy
        cross_y = uz * gx - ux * gz
        cross_z = ux * gy - uy * gx
        dot = ux * gx + uy * gy + uz * gz
        one_minus_cosine = 1.0 - cosine
        next_gx = gx * cosine + cross_x * sine + ux * dot * one_minus_cosine
        next_gy = gy * cosine + cross_y * sine + uy * dot * one_minus_cosine
        next_gz = gz * cosine + cross_z * sine + uz * dot * one_minus_cosine
        gx, gy, gz = next_gx, next_gy, next_gz
    norm = math.sqrt(gx * gx + gy * gy + gz * gz)
    return np.array([gx / norm, gy / norm, gz / norm])


def fit_gyroscope(
    data: dict[str, np.ndarray],
    segments: list[tuple[int, int]],
    accel_bias: np.ndarray,
    accel_matrix: np.ndarray,
    pose_codes: list[str] | None = None,
    excluded_transition_pairs: set[tuple[str, str]] | None = None,
) -> tuple[np.ndarray, np.ndarray, dict[str, object]]:
    static_indices = np.concatenate([np.arange(start, end) for start, end in segments])
    bias = np.median(data["gyro"][static_indices], axis=0)
    time_s = data["timestamp_us"].astype(np.float64) * 1e-6
    transitions: list[RotationTransition] = []
    excluded_transition_pairs = excluded_transition_pairs or set()
    for transition_index, ((start0, end0), (start1, end1)) in enumerate(
        zip(segments[:-1], segments[1:])
    ):
        if pose_codes is not None and (
            pose_codes[transition_index], pose_codes[transition_index + 1]
        ) in excluded_transition_pairs:
            continue
        if start1 - end0 < 3:
            continue
        gyro = data["gyro"][end0 - 1 : start1 + 1]
        local_time = time_s[end0 - 1 : start1 + 1]
        raw_angle = np.sum(
            np.linalg.norm(gyro[:-1] - bias, axis=1) * np.diff(local_time)
        )
        if (
            raw_angle < math.radians(10.0)
            or local_time[-1] - local_time[0] > MAX_GYRO_TRANSITION_SECONDS
        ):
            continue
        start_accel = np.mean(data["accel"][start0:end0], axis=0)
        end_accel = np.mean(data["accel"][start1:end1], axis=0)
        start_g = accel_matrix @ (start_accel - accel_bias)
        end_g = accel_matrix @ (end_accel - accel_bias)
        median_dt = float(np.median(np.diff(local_time)))
        rate_stride = int(round(1.0 / (GYRO_LM_TARGET_RATE_HZ * median_dt)))
        point_stride = math.ceil(len(local_time) / MAX_GYRO_TRANSITION_POINTS)
        stride = max(1, rate_stride, point_stride)
        indices = np.arange(0, len(local_time), stride)
        if indices[-1] != len(local_time) - 1:
            indices = np.r_[indices, len(local_time) - 1]
        transitions.append(
            RotationTransition(
                local_time[indices],
                gyro[indices],
                start_g / np.linalg.norm(start_g),
                end_g / np.linalg.norm(end_g),
            )
        )

    if len(transitions) < 6:
        return (
            bias,
            np.eye(3),
            {
                "transition_count": len(transitions),
                "status": "bias_only",
                "reason": "少于 6 个大于 10 度的静止端点旋转，陀螺矩阵不可辨识",
                "excluded_transition_pairs": [
                    list(pair) for pair in sorted(excluded_transition_pairs)
                ],
            },
        )

    def residual(flat_matrix: np.ndarray) -> np.ndarray:
        matrix = flat_matrix.reshape(3, 3)
        endpoint_errors = np.concatenate(
            [
                integrate_gravity(item, matrix, bias) - item.gravity_end
                for item in transitions
            ]
        )
        regularization = 0.01 * (matrix - np.eye(3)).ravel()
        return np.r_[endpoint_errors, regularization]

    lower = np.full(9, -0.3)
    upper = np.full(9, 0.3)
    lower[[0, 4, 8]] = 0.5
    upper[[0, 4, 8]] = 1.5
    result = least_squares(
        residual,
        np.eye(3).ravel(),
        bounds=(lower, upper),
        method="trf",
        loss="soft_l1",
        f_scale=0.01,
        max_nfev=1500,
    )
    matrix = result.x.reshape(3, 3)
    before = np.concatenate(
        [
            integrate_gravity(item, np.eye(3), bias) - item.gravity_end
            for item in transitions
        ]
    )
    after = np.concatenate(
        [
            integrate_gravity(item, matrix, bias) - item.gravity_end
            for item in transitions
        ]
    )
    return (
        bias,
        matrix,
        {
            "transition_count": len(transitions),
            "status": "matrix_and_bias" if result.success else "optimization_failed",
            "endpoint_rmse_before": float(np.sqrt(np.mean(before**2))),
            "endpoint_rmse_after": float(np.sqrt(np.mean(after**2))),
            "condition_number": float(np.linalg.cond(matrix)),
            "excluded_transition_pairs": [
                list(pair) for pair in sorted(excluded_transition_pairs)
            ],
        },
    )


def calibrate(
    data: dict[str, np.ndarray],
    gyro_threshold: float,
    min_static_seconds: float,
    segments: list[tuple[int, int]] | None = None,
    segment_source: str = "automatic_detection",
    pose_codes: list[str] | None = None,
    excluded_transition_pairs: set[tuple[str, str]] | None = None,
) -> dict:
    if segments is None:
        segments = detect_static_segments(data, gyro_threshold, min_static_seconds)
    if not segments:
        raise ValueError("未检测到满足时长与阈值的静止段")
    all_means = np.array(
        [np.mean(data["accel"][start:end], axis=0) for start, end in segments]
    )
    selected: list[np.ndarray] = []
    for mean in all_means:
        direction = mean / np.linalg.norm(mean)
        if not selected or max(
            float(np.dot(direction, item / np.linalg.norm(item))) for item in selected
        ) < math.cos(math.radians(8.0)):
            selected.append(mean)
    pose_means = np.asarray(selected)
    accel_bias, accel_matrix, accel_metrics = fit_accelerometer(pose_means)
    gyro_bias, gyro_matrix, gyro_metrics = fit_gyroscope(
        data,
        segments,
        accel_bias,
        accel_matrix,
        pose_codes=pose_codes,
        excluded_transition_pairs=excluded_transition_pairs,
    )
    return {
        "format_version": 1,
        "units": {"accel": "m/s^2", "gyro": "rad/s"},
        "model": "corrected = matrix @ (measured - bias)",
        "accelerometer": {
            "bias": accel_bias.tolist(),
            "matrix": accel_matrix.tolist(),
            "metrics": accel_metrics,
        },
        "gyroscope": {
            "bias": gyro_bias.tolist(),
            "matrix": gyro_matrix.tolist(),
            "metrics": gyro_metrics,
        },
        "detection": {
            "segment_source": segment_source,
            "static_segment_count": len(segments),
            "selected_pose_count": len(pose_means),
            "pose_codes": pose_codes or [],
            "excluded_gyro_transition_pairs": [
                list(pair) for pair in sorted(excluded_transition_pairs or set())
            ],
            "gyro_static_threshold_rps": gyro_threshold,
            "minimum_static_seconds": min_static_seconds,
            "maximum_gyro_transition_seconds": MAX_GYRO_TRANSITION_SECONDS,
            "maximum_gyro_transition_points": MAX_GYRO_TRANSITION_POINTS,
            "gyro_lm_target_rate_hz": GYRO_LM_TARGET_RATE_HZ,
        },
    }


def apply_calibration(values: np.ndarray, section: dict) -> np.ndarray:
    matrix = np.asarray(section["matrix"], dtype=np.float64)
    bias = np.asarray(section["bias"], dtype=np.float64)
    return (matrix @ (values - bias).T).T


def allan_deviation(
    values: np.ndarray, sample_interval_s: float, points: int = 60
) -> tuple[np.ndarray, np.ndarray]:
    count = len(values)
    max_cluster = max(1, count // 20)
    clusters = np.unique(
        np.logspace(0, math.log10(max_cluster), points).astype(np.int64)
    )
    cumulative = np.vstack([np.zeros(values.shape[1]), np.cumsum(values, axis=0)])
    tau_values, deviations = [], []
    for cluster in clusters:
        averages = (cumulative[cluster:] - cumulative[:-cluster]) / cluster
        if len(averages) <= cluster:
            continue
        delta = averages[cluster:] - averages[:-cluster]
        tau_values.append(cluster * sample_interval_s)
        deviations.append(np.sqrt(0.5 * np.mean(delta * delta, axis=0)))
    return np.asarray(tau_values), np.asarray(deviations)


def allan_coefficients(
    tau: np.ndarray, deviation: np.ndarray
) -> list[dict[str, float]]:
    output = []
    for axis in range(3):
        curve = deviation[:, axis]
        slope = np.gradient(np.log(curve), np.log(tau))
        white_mask = np.abs(slope + 0.5) < 0.15
        walk_mask = np.abs(slope - 0.5) < 0.15
        flat_mask = np.abs(slope) < 0.10
        white = (
            float(np.median(curve[white_mask] * np.sqrt(tau[white_mask])))
            if np.any(white_mask)
            else math.nan
        )
        walk = (
            float(np.median(curve[walk_mask] * np.sqrt(3.0 / tau[walk_mask])))
            if np.any(walk_mask)
            else math.nan
        )
        if np.any(flat_mask):
            flat_index = np.flatnonzero(flat_mask)[np.argmin(curve[flat_mask])]
        else:
            flat_index = int(np.argmin(curve))
        output.append(
            {
                "white_noise_density": white,
                "bias_instability": float(curve[flat_index] / 0.664),
                "random_walk_density": walk,
                "bias_instability_tau_s": float(tau[flat_index]),
            }
        )
    return output


def analyze_allan(
    data: dict[str, np.ndarray], calibration: dict, output_dir: Path
) -> dict:
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp_s = data["timestamp_us"].astype(np.float64) * 1e-6
    sequence_gap_count = int(
        np.count_nonzero(np.diff(data["sequence"].astype(np.int64)) != 1)
    )
    gyro = apply_calibration(data["gyro"], calibration["gyroscope"])
    accel = apply_calibration(data["accel"], calibration["accelerometer"])
    gyro_dt = float(np.median(np.diff(timestamp_s)))

    # BMI088 加速度计配置为 800 Hz、组合读取为 1 kHz；sensor-time 不变表示保持样本。
    sensor_time = data["accel_sensor_time"]
    if np.any(sensor_time != 0):
        accel_changed = np.r_[True, np.diff(sensor_time.astype(np.int64)) != 0]
    else:
        accel_changed = np.r_[
            True, np.any(np.diff(data["accel_raw"], axis=0) != 0, axis=1)
        ]
    accel_time = timestamp_s[accel_changed]
    accel = accel[accel_changed]
    accel_dt = float(np.median(np.diff(accel_time)))
    gyro_tau, gyro_adev = allan_deviation(gyro, gyro_dt)
    accel_tau, accel_adev = allan_deviation(accel, accel_dt)
    gyro_coeff = allan_coefficients(gyro_tau, gyro_adev)
    accel_coeff = allan_coefficients(accel_tau, accel_adev)

    figure, axes = plt.subplots(1, 2, figsize=(12, 5))
    for axis, label in enumerate("XYZ"):
        axes[0].loglog(gyro_tau, gyro_adev[:, axis], label=label)
        axes[1].loglog(accel_tau, accel_adev[:, axis], label=label)
    axes[0].set(title="BMI088 gyro Allan deviation", xlabel="tau (s)", ylabel="rad/s")
    axes[1].set(title="BMI088 accel Allan deviation", xlabel="tau (s)", ylabel="m/s^2")
    for axis in axes:
        axis.grid(True, which="both", alpha=0.3)
        axis.legend()
    figure.tight_layout()
    figure.savefig(output_dir / "allan_deviation.png", dpi=180)
    plt.close(figure)

    def conservative(coefficients: list[dict[str, float]], key: str) -> float | None:
        values = [item[key] for item in coefficients if math.isfinite(item[key])]
        return max(values) if values else None

    duration_s = float(timestamp_s[-1] - timestamp_s[0])
    warnings = []
    if duration_s < 1800.0:
        warnings.append(
            "记录短于 30 分钟；长期零偏不稳定性和随机游走结果不足以配置 ESKF"
        )
    if sequence_gap_count != 0:
        warnings.append(
            "IMU sample_sequence 存在跳变；先排查 DataRouter/SD 拥塞再采用结果"
        )
    result = {
        "format_version": 1,
        "warnings": warnings,
        "sample_quality": {
            "input_samples": len(timestamp_s),
            "accel_unique_samples": len(accel_time),
            "sequence_gap_count": sequence_gap_count,
            "gyro_sample_rate_hz": 1.0 / gyro_dt,
            "accel_effective_sample_rate_hz": 1.0 / accel_dt,
            "duration_s": duration_s,
        },
        "gyroscope": {"axes": gyro_coeff},
        "accelerometer": {"axes": accel_coeff},
        "eskf_conservative": {
            "gyro_noise": conservative(gyro_coeff, "white_noise_density"),
            "accel_noise": conservative(accel_coeff, "white_noise_density"),
            "gyro_bias_rw": conservative(gyro_coeff, "random_walk_density"),
            "accel_bias_rw": conservative(accel_coeff, "random_walk_density"),
        },
    }
    (output_dir / "allan_results.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    return result


def write_calibration_header(calibration: dict, path: Path) -> None:
    def literal(value: float) -> str:
        if not math.isfinite(value):
            raise ValueError("固定标定参数包含 NaN 或无穷值，拒绝生成固件头文件")
        return f"{value:.8e}f"

    def vector(values: Iterable[float]) -> str:
        return "{" + ", ".join(literal(value) for value in values) + "}"

    def matrix_declaration(name: str, rows: Iterable[Iterable[float]]) -> str:
        declaration = f"static const float {name}[3][3] = "
        rendered_rows = [vector(row) for row in rows]
        continuation = " " * (len(declaration) + 1)
        return (
            declaration
            + "{"
            + rendered_rows[0]
            + ",\n"
            + continuation
            + rendered_rows[1]
            + ",\n"
            + continuation
            + rendered_rows[2]
            + "};"
        )

    acc = calibration["accelerometer"]
    gyro = calibration["gyroscope"]
    content = f"""#ifndef BMI088_FIXED_CALIBRATION_GENERATED_H
#define BMI088_FIXED_CALIBRATION_GENERATED_H

/* Generated by Tools/imu_calibration/imu_calibration.py. Units: m/s^2 and rad/s. */
static const float bmi088_accel_bias_mps2[3] = {vector(acc['bias'])};

{matrix_declaration('bmi088_accel_matrix', acc['matrix'])}

static const float bmi088_gyro_bias_rps[3] = {vector(gyro['bias'])};

{matrix_declaration('bmi088_gyro_matrix', gyro['matrix'])}

#endif
"""
    path.write_text(content, encoding="utf-8")


def command_extract(args: argparse.Namespace) -> None:
    print(
        json.dumps(
            extract_blackbox(args.input, args.output), indent=2, ensure_ascii=False
        )
    )


def command_lm(args: argparse.Namespace) -> None:
    data = load_csv(args.input)
    if args.gyro_temperature_compensation:
        data = apply_gyro_temperature_compensation(data)
    segments = None
    pose_codes = None
    excluded_transition_pairs: set[tuple[str, str]] = set()
    segment_source = "automatic_detection"
    if args.guided_report is not None:
        segments, pose_codes = guided_static_segments(
            data, args.guided_report, args.min_static_seconds
        )
        guided_report = json.loads(args.guided_report.read_text(encoding="utf-8"))
        excluded_transition_pairs = {
            (str(pair[0]), str(pair[1]))
            for pair in guided_report.get("excluded_gyro_transition_pairs", [])
            if isinstance(pair, list) and len(pair) == 2
        }
        segment_source = "guided_report"
    result = calibrate(
        data,
        args.gyro_static_threshold,
        args.min_static_seconds,
        segments=segments,
        segment_source=segment_source,
        pose_codes=pose_codes,
        excluded_transition_pairs=excluded_transition_pairs,
    )
    result["preprocessing"] = {
        "gyro_temperature_compensation": (
            temperature_compensation_metadata()
            if args.gyro_temperature_compensation
            else {"enabled": False}
        )
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    if args.c_header:
        write_calibration_header(result, args.c_header)
    print(json.dumps(result, indent=2, ensure_ascii=False))


def command_allan(args: argparse.Namespace) -> None:
    calibration = json.loads(args.calibration.read_text(encoding="utf-8"))
    print(
        json.dumps(
            analyze_allan(load_csv(args.input), calibration, args.output_dir),
            indent=2,
            ensure_ascii=False,
        )
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    extract = subparsers.add_parser("extract", help="从 SD 黑匣子 BIN 提取 BMI088 CSV")
    extract.add_argument("input", type=Path)
    extract.add_argument("-o", "--output", type=Path, required=True)
    extract.set_defaults(func=command_extract)
    lm = subparsers.add_parser("lm", help="从多姿态/多旋转 CSV 做固定参数 LM 标定")
    lm.add_argument("input", type=Path)
    lm.add_argument("-o", "--output", type=Path, required=True)
    lm.add_argument("--c-header", type=Path)
    lm.add_argument("--gyro-static-threshold", type=float, default=0.035)
    lm.add_argument("--min-static-seconds", type=float, default=1.5)
    lm.add_argument(
        "--guided-report",
        type=Path,
        help="使用引导采集报告中的已验收姿态窗口，不再重新猜测静止段",
    )
    lm.add_argument(
        "--gyro-temperature-compensation",
        action="store_true",
        help="在 LM 求解前应用已验证的 BMI088 陀螺仪温漂模型",
    )
    lm.set_defaults(func=command_lm)
    allan = subparsers.add_parser(
        "allan", help="对固定校准后的长时间静止 CSV 做 Allan 分析"
    )
    allan.add_argument("input", type=Path)
    allan.add_argument("--calibration", type=Path, required=True)
    allan.add_argument("-o", "--output-dir", type=Path, required=True)
    allan.set_defaults(func=command_allan)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
