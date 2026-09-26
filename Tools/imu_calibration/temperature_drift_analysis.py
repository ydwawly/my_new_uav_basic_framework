#!/usr/bin/env python3
"""Analyze a stationary BMI088 cold-to-hot USB FLOG capture.

The analysis keeps the raw 1 kHz file as the source of truth, validates every
frame CRC, aggregates samples to one-second statistics, and fits a provisional
quadratic gyroscope bias-temperature model.  Accelerometer temperature fits are
diagnostic only because one stationary orientation cannot separate bias drift
from scale-factor drift.
"""

from __future__ import annotations

import argparse
import binascii
import csv
import json
import math
import mmap
import struct
from datetime import datetime
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares


FILE_MAGIC = 0x474F4C46
FRAME_MAGIC = 0xA55A
FILE_HEADER = struct.Struct("<IHHQIHH")
FRAME_SIZE = 74
FRAME_DATA_SIZE = 72
FRAME_HEADER_SIZE = 20
IMU_PAYLOAD_SIZE = 52
TEMPERATURE_VALID_MASK = 1 << 2

RECORD_DTYPE = np.dtype(
    {
        "names": (
            "magic",
            "message_id",
            "message_version",
            "payload_length",
            "header_flags",
            "frame_sequence",
            "timestamp_us",
            "sample_sequence",
            "sensor_time",
            "accel_raw_x",
            "accel_raw_y",
            "accel_raw_z",
            "gyro_raw_x",
            "gyro_raw_y",
            "gyro_raw_z",
            "temperature_raw",
            "validity_flags",
            "accel_x",
            "accel_y",
            "accel_z",
            "gyro_x",
            "gyro_y",
            "gyro_z",
            "temperature_c",
            "crc",
        ),
        "formats": (
            "<u2",
            "u1",
            "u1",
            "<u2",
            "<u2",
            "<u4",
            "<u8",
            "<u4",
            "<u4",
            "<i2",
            "<i2",
            "<i2",
            "<i2",
            "<i2",
            "<i2",
            "<i2",
            "<u2",
            "<f4",
            "<f4",
            "<f4",
            "<f4",
            "<f4",
            "<f4",
            "<f4",
            "<u2",
        ),
        "offsets": (
            0,
            2,
            3,
            4,
            6,
            8,
            12,
            20,
            24,
            28,
            30,
            32,
            34,
            36,
            38,
            40,
            42,
            44,
            48,
            52,
            56,
            60,
            64,
            68,
            72,
        ),
        "itemsize": FRAME_SIZE,
    }
)

AXES = ("x", "y", "z")
COLORS = ("#2f6fb0", "#d69b2d", "#c05a87")
LINE_STYLES = ("-", "--", ":")


def _crc16(data: bytes | memoryview) -> int:
    return binascii.crc_hqx(data, 0xFFFF)


def _sequence_stats(values: np.ndarray) -> tuple[int, int]:
    current = values[1:].astype(np.uint64)
    previous = values[:-1].astype(np.uint64)
    delta = (current - previous) & np.uint64(0xFFFFFFFF)
    duplicates = int(np.count_nonzero((delta == 0) | (delta >= 0x80000000)))
    gaps = int(np.sum(np.where((delta > 1) & (delta < 0x80000000), delta - 1, 0)))
    return gaps, duplicates


def _aggregate_fields(
    records: np.ndarray,
    second_index: np.ndarray,
    counts: np.ndarray,
    fields: tuple[str, str, str],
) -> tuple[np.ndarray, np.ndarray]:
    mean = np.empty((len(counts), 3), dtype=np.float64)
    std = np.empty_like(mean)
    for axis, field in enumerate(fields):
        values = records[field].astype(np.float64)
        total = np.bincount(second_index, weights=values, minlength=len(counts))
        total_square = np.bincount(
            second_index, weights=values * values, minlength=len(counts)
        )
        mean[:, axis] = total / counts
        variance = total_square / counts - mean[:, axis] * mean[:, axis]
        std[:, axis] = np.sqrt(np.maximum(variance, 0.0))
    return mean, std


def _fit_quadratic(
    temperature_c: np.ndarray,
    values: np.ndarray,
    reference_temperature_c: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    shifted = temperature_c - reference_temperature_c
    design = np.column_stack((np.ones_like(shifted), shifted, shifted * shifted))
    coefficients = np.empty((3, 3), dtype=np.float64)
    predictions = np.empty_like(values)
    for axis in range(3):
        initial = np.linalg.lstsq(design, values[:, axis], rcond=None)[0]
        initial_residual = values[:, axis] - design @ initial
        median_residual = np.median(initial_residual)
        robust_scale = max(
            1.4826 * np.median(np.abs(initial_residual - median_residual)), 1e-7
        )
        fit = least_squares(
            lambda candidate: design @ candidate - values[:, axis],
            initial,
            loss="soft_l1",
            f_scale=robust_scale,
        )
        coefficients[axis] = fit.x
        predictions[:, axis] = design @ fit.x
    return coefficients, predictions, values - predictions


def _rolling_stability(temperature_c: np.ndarray) -> tuple[np.ndarray, list[list[int]]]:
    stable = np.zeros(len(temperature_c), dtype=bool)
    for end in range(90, len(temperature_c)):
        start = max(0, end - 119)
        window = temperature_c[start : end + 1]
        seconds = np.arange(len(window), dtype=np.float64)
        slope_c_per_min = float(np.polyfit(seconds, window, 1)[0] * 60.0)
        stable[end] = (
            end - start >= 90
            and float(np.ptp(window)) <= 0.5
            and abs(slope_c_per_min) <= 0.1
        )
    changes = np.diff(np.r_[False, stable, False].astype(np.int8))
    runs = [
        [int(start), int(end)]
        for start, end in zip(
            np.flatnonzero(changes == 1), np.flatnonzero(changes == -1)
        )
    ]
    return stable, runs


def _hysteresis_summary(
    temperature_c: np.ndarray, gyro_dps: np.ndarray
) -> dict[str, Any]:
    smooth = np.convolve(temperature_c, np.ones(61) / 61.0, mode="same")
    slope_c_per_min = np.gradient(smooth) * 60.0
    temperature_half_degree = np.round(temperature_c * 2.0) / 2.0
    rows: list[dict[str, Any]] = []
    for temperature in np.unique(temperature_half_degree):
        heating = (temperature_half_degree == temperature) & (slope_c_per_min > 0.05)
        cooling = (temperature_half_degree == temperature) & (slope_c_per_min < -0.05)
        if np.count_nonzero(heating) < 10 or np.count_nonzero(cooling) < 10:
            continue
        difference = np.median(gyro_dps[heating], axis=0) - np.median(
            gyro_dps[cooling], axis=0
        )
        rows.append(
            {
                "temperature_c": float(temperature),
                "heating_seconds": int(np.count_nonzero(heating)),
                "cooling_seconds": int(np.count_nonzero(cooling)),
                **{
                    f"gyro_{axis}_heating_minus_cooling_dps": float(difference[index])
                    for index, axis in enumerate(AXES)
                },
            }
        )
    if not rows:
        maximum = [math.nan, math.nan, math.nan]
        median = [math.nan, math.nan, math.nan]
    else:
        differences = np.asarray(
            [
                [row[f"gyro_{axis}_heating_minus_cooling_dps"] for axis in AXES]
                for row in rows
            ]
        )
        maximum = np.max(np.abs(differences), axis=0).tolist()
        median = np.median(np.abs(differences), axis=0).tolist()
    return {
        "overlap_temperature_bins": len(rows),
        "maximum_absolute_dps": maximum,
        "median_absolute_dps": median,
        "bins": rows,
    }


def _tail_window(
    values: np.ndarray, full_seconds: np.ndarray, seconds: int
) -> np.ndarray:
    indices = np.flatnonzero(full_seconds)
    return values[indices[-seconds:]]


def _window_temperature_metrics(
    temperature_c: np.ndarray, seconds: int
) -> dict[str, float]:
    window = temperature_c[-seconds:]
    elapsed = np.arange(len(window), dtype=np.float64)
    return {
        "mean_c": float(np.mean(window)),
        "range_c": float(np.ptp(window)),
        "slope_c_per_min": float(np.polyfit(elapsed, window, 1)[0] * 60.0),
        "start_c": float(window[0]),
        "end_c": float(window[-1]),
    }


def _write_second_csv(
    path: Path,
    counts: np.ndarray,
    temperature_c: np.ndarray,
    accel_mean: np.ndarray,
    accel_std_norm: np.ndarray,
    gyro_dps: np.ndarray,
    gyro_std_dps_norm: np.ndarray,
    orientation_angle_deg: np.ndarray,
    static: np.ndarray,
    corrected_gyro_dps: np.ndarray,
) -> None:
    columns = [
        "second",
        "elapsed_min",
        "samples",
        "temperature_c",
        "accel_x_mps2",
        "accel_y_mps2",
        "accel_z_mps2",
        "accel_std_norm_mps2",
        "gyro_x_dps",
        "gyro_y_dps",
        "gyro_z_dps",
        "gyro_std_norm_dps",
        "orientation_angle_deg",
        "static_usable",
        "gyro_x_compensated_dps",
        "gyro_y_compensated_dps",
        "gyro_z_compensated_dps",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        for second in range(len(counts)):
            writer.writerow(
                [
                    second,
                    second / 60.0,
                    int(counts[second]),
                    temperature_c[second],
                    *accel_mean[second],
                    accel_std_norm[second],
                    *gyro_dps[second],
                    gyro_std_dps_norm[second],
                    orientation_angle_deg[second],
                    int(static[second]),
                    *corrected_gyro_dps[second],
                ]
            )


def _write_temperature_bin_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        raise ValueError("No temperature bins were produced")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _plot_overview(
    path: Path,
    temperature_c: np.ndarray,
    gyro_dps: np.ndarray,
    corrected_gyro_dps: np.ndarray,
    bin_rows: list[dict[str, Any]],
    coefficients_dps: np.ndarray,
    reference_temperature_c: float,
) -> None:
    matplotlib.rcParams["font.sans-serif"] = [
        "Microsoft YaHei",
        "SimHei",
        "DejaVu Sans",
    ]
    matplotlib.rcParams["axes.unicode_minus"] = False
    minutes = np.arange(len(temperature_c), dtype=np.float64) / 60.0
    figure, axes = plt.subplots(2, 2, figsize=(13.5, 9.0), dpi=150)
    figure.patch.set_facecolor("white")

    axes[0, 0].plot(minutes, temperature_c, color=COLORS[0], linewidth=1.5)
    axes[0, 0].set_title("BMI088 温度随时间变化")
    axes[0, 0].set_xlabel("采集时间 (min)")
    axes[0, 0].set_ylabel("温度 (°C)")

    for index, axis in enumerate(AXES):
        axes[0, 1].plot(
            minutes,
            gyro_dps[:, index],
            label=axis.upper(),
            color=COLORS[index],
            linestyle=LINE_STYLES[index],
            linewidth=1.1,
        )
    axes[0, 1].set_title("原始陀螺仪一秒均值")
    axes[0, 1].set_xlabel("采集时间 (min)")
    axes[0, 1].set_ylabel("角速度 (°/s)")
    axes[0, 1].legend(frameon=False, ncol=3)

    temperatures = np.asarray([row["temperature_c"] for row in bin_rows])
    shifted = temperatures - reference_temperature_c
    for index, axis in enumerate(AXES):
        observed = np.asarray([row[f"gyro_{axis}_median_dps"] for row in bin_rows])
        model = (
            coefficients_dps[index, 0]
            + coefficients_dps[index, 1] * shifted
            + coefficients_dps[index, 2] * shifted * shifted
        )
        axes[1, 0].scatter(
            temperatures,
            observed,
            color=COLORS[index],
            s=15,
            alpha=0.65,
            label=f"{axis.upper()} 温度分箱中位数",
        )
        axes[1, 0].plot(
            temperatures,
            model,
            color=COLORS[index],
            linestyle=LINE_STYLES[index],
            linewidth=1.5,
        )
    axes[1, 0].set_title("陀螺仪零偏与温度二次拟合")
    axes[1, 0].set_xlabel("温度 (°C)")
    axes[1, 0].set_ylabel("角速度 (°/s)")
    axes[1, 0].legend(frameon=False, fontsize=8)

    for index, axis in enumerate(AXES):
        axes[1, 1].plot(
            minutes,
            corrected_gyro_dps[:, index],
            label=axis.upper(),
            color=COLORS[index],
            linestyle=LINE_STYLES[index],
            linewidth=1.1,
        )
    axes[1, 1].set_title("应用候选温补后的陀螺仪一秒均值")
    axes[1, 1].set_xlabel("采集时间 (min)")
    axes[1, 1].set_ylabel("角速度 (°/s)")
    axes[1, 1].legend(frameon=False, ncol=3)

    for axis in axes.flat:
        axis.grid(color="#d7dde4", linewidth=0.6, alpha=0.8)
        axis.spines[["top", "right"]].set_visible(False)
    figure.suptitle(
        "BMI088 冷机升温温漂分析（1 秒聚合）", fontsize=15, fontweight="bold"
    )
    figure.tight_layout(rect=(0, 0, 1, 0.96))
    figure.savefig(path, bbox_inches="tight", facecolor="white")
    plt.close(figure)


def analyze_capture(source: Path, output_directory: Path) -> dict[str, Any]:
    source = source.resolve()
    output_directory = output_directory.resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    file_size = source.stat().st_size
    if file_size < FILE_HEADER.size + FRAME_SIZE:
        raise ValueError("Capture is too short")

    with source.open("rb") as stream:
        header = stream.read(FILE_HEADER.size)
    magic, version, header_size, session_token, start_timestamp_us, header_crc, _ = (
        FILE_HEADER.unpack(header)
    )
    if magic != FILE_MAGIC or version != 1 or header_size != FILE_HEADER.size:
        raise ValueError("Unsupported FLOG header")
    if _crc16(header[:20]) != header_crc:
        raise ValueError("FLOG header CRC mismatch")

    frame_count = (file_size - header_size) // FRAME_SIZE
    trailing_bytes = (file_size - header_size) % FRAME_SIZE
    mapped_bytes = np.memmap(source, mode="r", dtype=np.uint8)
    records = np.ndarray(
        frame_count, dtype=RECORD_DTYPE, buffer=mapped_bytes, offset=header_size
    )
    invalid_headers = int(
        np.count_nonzero(
            (records["magic"] != FRAME_MAGIC)
            | (records["message_id"] != 3)
            | (records["message_version"] != 1)
            | (records["payload_length"] != IMU_PAYLOAD_SIZE)
        )
    )
    if invalid_headers:
        raise ValueError(f"Found {invalid_headers} invalid or misaligned frame headers")

    crc_errors = 0
    with source.open("rb") as stream:
        mapped_file = mmap.mmap(stream.fileno(), 0, access=mmap.ACCESS_READ)
        view = memoryview(mapped_file)
        for frame_index in range(frame_count):
            position = header_size + frame_index * FRAME_SIZE
            if _crc16(view[position : position + FRAME_DATA_SIZE]) != int(
                records["crc"][frame_index]
            ):
                crc_errors += 1
        trailing_hex = bytes(view[-trailing_bytes:]).hex() if trailing_bytes else ""
        view.release()
        mapped_file.close()

    timestamp_us = records["timestamp_us"].astype(np.int64)
    intervals_us = np.diff(timestamp_us)
    frame_gaps, frame_duplicates = _sequence_stats(records["frame_sequence"])
    sample_gaps, sample_duplicates = _sequence_stats(records["sample_sequence"])
    second_index = ((timestamp_us - timestamp_us[0]) // 1_000_000).astype(np.int32)
    second_count = int(second_index[-1]) + 1
    counts = np.bincount(second_index, minlength=second_count).astype(np.float64)
    full_seconds = counts >= 0.8 * np.median(counts)

    accel_mean, accel_std = _aggregate_fields(
        records,
        second_index,
        counts,
        ("accel_x", "accel_y", "accel_z"),
    )
    gyro_mean_rps, gyro_std_rps = _aggregate_fields(
        records,
        second_index,
        counts,
        ("gyro_x", "gyro_y", "gyro_z"),
    )
    temperature_c = (
        np.bincount(
            second_index,
            weights=records["temperature_c"].astype(np.float64),
            minlength=second_count,
        )
        / counts
    )
    gyro_mean_dps = np.degrees(gyro_mean_rps)
    accel_std_norm = np.linalg.norm(accel_std, axis=1)
    gyro_std_dps_norm = np.degrees(np.linalg.norm(gyro_std_rps, axis=1))
    reference_accel = _tail_window(accel_mean, full_seconds, 120).mean(axis=0)
    reference_accel /= np.linalg.norm(reference_accel)
    accel_direction = accel_mean / np.linalg.norm(accel_mean, axis=1, keepdims=True)
    orientation_angle_deg = np.degrees(
        np.arccos(np.clip(accel_direction @ reference_accel, -1.0, 1.0))
    )
    static = (
        full_seconds
        & (accel_std_norm < 0.15)
        & (np.linalg.norm(gyro_std_rps, axis=1) < 0.012)
        & (orientation_angle_deg < 1.0)
    )

    reference_temperature_c = float(
        np.median(_tail_window(temperature_c, full_seconds, 120))
    )
    rounded_temperature = np.round(temperature_c * 4.0) / 4.0
    bin_indices: list[tuple[float, np.ndarray]] = []
    for temperature in np.unique(rounded_temperature[static]):
        indices = np.flatnonzero(static & (rounded_temperature == temperature))
        if len(indices) >= 3:
            bin_indices.append((float(temperature), indices))
    if len(bin_indices) < 20:
        raise ValueError("Temperature coverage is too sparse for a quadratic model")

    bin_temperature = np.asarray([item[0] for item in bin_indices])
    bin_gyro_dps = np.asarray(
        [np.median(gyro_mean_dps[indices], axis=0) for _, indices in bin_indices]
    )
    bin_accel = np.asarray(
        [np.median(accel_mean[indices], axis=0) for _, indices in bin_indices]
    )
    gyro_coefficients_dps, bin_gyro_prediction, bin_gyro_residual = _fit_quadratic(
        bin_temperature, bin_gyro_dps, reference_temperature_c
    )
    accel_coefficients, bin_accel_prediction, bin_accel_residual = _fit_quadratic(
        bin_temperature, bin_accel, reference_temperature_c
    )

    shifted_all = temperature_c - reference_temperature_c
    gyro_temperature_delta_dps = np.column_stack(
        [
            gyro_coefficients_dps[axis, 1] * shifted_all
            + gyro_coefficients_dps[axis, 2] * shifted_all * shifted_all
            for axis in range(3)
        ]
    )
    corrected_gyro_dps = gyro_mean_dps - gyro_temperature_delta_dps
    static_gyro = gyro_mean_dps[static]
    static_corrected_gyro = corrected_gyro_dps[static]
    raw_p90_span = np.diff(np.percentile(static_gyro, [5, 95], axis=0), axis=0)[0]
    corrected_p90_span = np.diff(
        np.percentile(static_corrected_gyro, [5, 95], axis=0), axis=0
    )[0]
    cold_gyro = gyro_mean_dps[np.flatnonzero(full_seconds)[:60]].mean(axis=0)
    hot_gyro = _tail_window(gyro_mean_dps, full_seconds, 120).mean(axis=0)
    cold_corrected = corrected_gyro_dps[np.flatnonzero(full_seconds)[:60]].mean(axis=0)
    hot_corrected = _tail_window(corrected_gyro_dps, full_seconds, 120).mean(axis=0)

    bin_rows: list[dict[str, Any]] = []
    for row_index, (temperature, indices) in enumerate(bin_indices):
        row: dict[str, Any] = {
            "temperature_c": temperature,
            "seconds": int(len(indices)),
        }
        for axis_index, axis in enumerate(AXES):
            row[f"gyro_{axis}_median_dps"] = float(bin_gyro_dps[row_index, axis_index])
            row[f"gyro_{axis}_model_dps"] = float(
                bin_gyro_prediction[row_index, axis_index]
            )
            row[f"gyro_{axis}_residual_dps"] = float(
                bin_gyro_residual[row_index, axis_index]
            )
            row[f"accel_{axis}_median_mps2"] = float(bin_accel[row_index, axis_index])
            row[f"accel_{axis}_model_mps2"] = float(
                bin_accel_prediction[row_index, axis_index]
            )
        bin_rows.append(row)

    stable, stable_runs = _rolling_stability(temperature_c)
    last_120 = _window_temperature_metrics(temperature_c, 120)
    last_300 = _window_temperature_metrics(temperature_c, 300)
    last_600 = _window_temperature_metrics(temperature_c, 600)
    temperature_valid_count = int(
        np.count_nonzero(
            (records["validity_flags"] & TEMPERATURE_VALID_MASK)
            == TEMPERATURE_VALID_MASK
        )
    )
    gyro_fit_rmse = np.sqrt(np.mean(bin_gyro_residual * bin_gyro_residual, axis=0))
    gyro_total_square = np.sum(
        (bin_gyro_dps - np.mean(bin_gyro_dps, axis=0)) ** 2, axis=0
    )
    gyro_r_squared = (
        1.0 - np.sum(bin_gyro_residual * bin_gyro_residual, axis=0) / gyro_total_square
    )

    results: dict[str, Any] = {
        "format_version": 1,
        "generated_at": datetime.now().astimezone().isoformat(),
        "source": {
            "path": str(source),
            "size_bytes": file_size,
            "session_token": session_token,
            "start_timestamp_us": start_timestamp_us,
        },
        "quality": {
            "overall_pass_for_temperature_fitting": bool(
                crc_errors == 0
                and invalid_headers == 0
                and frame_gaps == 0
                and sample_gaps == 0
                and temperature_valid_count == frame_count
                and np.mean(static[full_seconds]) >= 0.99
                and float(np.ptp(records["temperature_c"])) >= 10.0
            ),
            "frames": frame_count,
            "duration_s": float((timestamp_us[-1] - timestamp_us[0]) * 1e-6),
            "sample_rate_hz": float(
                (frame_count - 1) / ((timestamp_us[-1] - timestamp_us[0]) * 1e-6)
            ),
            "header_crc_valid": True,
            "frame_crc_errors": crc_errors,
            "invalid_headers": invalid_headers,
            "frame_sequence_gaps": frame_gaps,
            "frame_duplicates_or_backward": frame_duplicates,
            "sample_sequence_gaps": sample_gaps,
            "sample_duplicates_or_backward": sample_duplicates,
            "timestamp_nonpositive_intervals": int(np.count_nonzero(intervals_us <= 0)),
            "timestamp_interval_outliers": int(
                np.count_nonzero((intervals_us < 500) | (intervals_us > 2000))
            ),
            "minimum_interval_us": int(np.min(intervals_us)),
            "median_interval_us": float(np.median(intervals_us)),
            "maximum_interval_us": int(np.max(intervals_us)),
            "temperature_valid_frames": temperature_valid_count,
            "temperature_valid_rate": temperature_valid_count / frame_count,
            "trailing_bytes": trailing_bytes,
            "trailing_hex": trailing_hex,
            "full_seconds": int(np.count_nonzero(full_seconds)),
            "static_usable_seconds": int(np.count_nonzero(static)),
            "static_usable_rate": float(np.mean(static[full_seconds])),
            "excluded_seconds": np.flatnonzero(full_seconds & ~static).tolist(),
            "motion_thresholds": {
                "accel_std_norm_mps2": 0.15,
                "gyro_std_norm_rps": 0.012,
                "orientation_angle_deg": 1.0,
            },
        },
        "temperature": {
            "minimum_c": float(np.min(records["temperature_c"])),
            "maximum_c": float(np.max(records["temperature_c"])),
            "span_c": float(np.ptp(records["temperature_c"])),
            "unique_raw_levels": int(len(np.unique(records["temperature_c"]))),
            "reference_c": reference_temperature_c,
            "model_minimum_c": float(bin_temperature[0]),
            "model_maximum_c": float(bin_temperature[-1]),
            "model_temperature_bins": len(bin_temperature),
            "last_120_s": last_120,
            "last_300_s": last_300,
            "last_600_s": last_600,
            "final_rolling_window_stable": bool(stable[-1]),
            "first_stable_second": (
                int(np.flatnonzero(stable)[0]) if np.any(stable) else None
            ),
            "stable_runs": stable_runs,
        },
        "gyro_model": {
            "status": "provisional_validation_candidate",
            "degree": 2,
            "equation": "bias(T)=b_ref+c1*(T-T_ref)+c2*(T-T_ref)^2",
            "runtime_delta_equation": "delta_bias(T)=c1*(T-T_ref)+c2*(T-T_ref)^2",
            "temperature_input_policy": "clamp to observed model range before evaluation",
            "reference_temperature_c": reference_temperature_c,
            "coefficients_dps": {
                axis: {
                    "b_ref": float(gyro_coefficients_dps[index, 0]),
                    "c1_per_c": float(gyro_coefficients_dps[index, 1]),
                    "c2_per_c2": float(gyro_coefficients_dps[index, 2]),
                }
                for index, axis in enumerate(AXES)
            },
            "coefficients_rps": {
                axis: {
                    "b_ref": float(np.radians(gyro_coefficients_dps[index, 0])),
                    "c1_per_c": float(np.radians(gyro_coefficients_dps[index, 1])),
                    "c2_per_c2": float(np.radians(gyro_coefficients_dps[index, 2])),
                }
                for index, axis in enumerate(AXES)
            },
            "temperature_bin_rmse_dps": {
                axis: float(gyro_fit_rmse[index]) for index, axis in enumerate(AXES)
            },
            "temperature_bin_r_squared": {
                axis: float(gyro_r_squared[index]) for index, axis in enumerate(AXES)
            },
            "raw_cold_to_hot_delta_dps": {
                axis: float(hot_gyro[index] - cold_gyro[index])
                for index, axis in enumerate(AXES)
            },
            "corrected_cold_to_hot_delta_dps": {
                axis: float(hot_corrected[index] - cold_corrected[index])
                for index, axis in enumerate(AXES)
            },
            "raw_p90_span_dps": {
                axis: float(raw_p90_span[index]) for index, axis in enumerate(AXES)
            },
            "corrected_p90_span_dps": {
                axis: float(corrected_p90_span[index])
                for index, axis in enumerate(AXES)
            },
            "hysteresis": _hysteresis_summary(temperature_c, gyro_mean_dps),
        },
        "accelerometer_diagnostic": {
            "runtime_compensation_ready": False,
            "reason": (
                "A single fixed orientation cannot separate accelerometer bias "
                "temperature drift from scale-factor temperature drift."
            ),
            "coefficients_mps2": {
                axis: {
                    "value_at_reference": float(accel_coefficients[index, 0]),
                    "c1_per_c": float(accel_coefficients[index, 1]),
                    "c2_per_c2": float(accel_coefficients[index, 2]),
                }
                for index, axis in enumerate(AXES)
            },
            "temperature_bin_rmse_mps2": {
                axis: float(np.sqrt(np.mean(bin_accel_residual[:, index] ** 2)))
                for index, axis in enumerate(AXES)
            },
        },
        "decision": {
            "fit_is_usable_as_validation_candidate": True,
            "production_ready": False,
            "why_not_production_ready": (
                "Only one cold-to-hot run is available and no independent repeat "
                "capture has verified residual drift or coefficient repeatability."
            ),
            "recommended_next_step": (
                "Install the gyroscope model behind a compile-time switch, clamp "
                "temperature to the observed range, then repeat the stationary "
                "cold-to-hot capture to validate it before enabling by default."
            ),
        },
    }

    second_csv = output_directory / "temperature_seconds.csv"
    bin_csv = output_directory / "temperature_bins.csv"
    overview_png = output_directory / "temperature_drift_overview.png"
    result_json = output_directory / "temperature_drift_results.json"
    _write_second_csv(
        second_csv,
        counts,
        temperature_c,
        accel_mean,
        accel_std_norm,
        gyro_mean_dps,
        gyro_std_dps_norm,
        orientation_angle_deg,
        static,
        corrected_gyro_dps,
    )
    _write_temperature_bin_csv(bin_csv, bin_rows)
    _plot_overview(
        overview_png,
        temperature_c,
        gyro_mean_dps,
        corrected_gyro_dps,
        bin_rows,
        gyro_coefficients_dps,
        reference_temperature_c,
    )
    result_json.write_text(
        json.dumps(results, indent=2, ensure_ascii=False, allow_nan=False),
        encoding="utf-8",
    )
    return results


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="Stationary cold-to-hot FLOG BIN")
    parser.add_argument("-o", "--output-directory", type=Path, required=True)
    return parser


def main() -> None:
    arguments = build_parser().parse_args()
    result = analyze_capture(arguments.input, arguments.output_directory)
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
