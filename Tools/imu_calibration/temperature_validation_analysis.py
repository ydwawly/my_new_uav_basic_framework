#!/usr/bin/env python3
"""Cross-validate fixed BMI088 gyro temperature coefficients on a second run."""

from __future__ import annotations

import argparse
import json
from datetime import datetime
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


AXES = ("x", "y", "z")
TEMPERATURE_BIN_C = 0.25
DRIFT_LIMIT_DPS = 0.05


def apply_model(
    frame: pd.DataFrame, model: dict[str, object], minimum_c: float, maximum_c: float
) -> pd.DataFrame:
    corrected = frame.copy()
    temperature_c = np.clip(
        corrected["temperature_c"].to_numpy(dtype=float), minimum_c, maximum_c
    )
    delta_t = temperature_c - float(model["reference_temperature_c"])
    for axis in AXES:
        coefficients = model["coefficients_dps"][axis]
        temperature_delta = (
            float(coefficients["c1_per_c"]) * delta_t
            + float(coefficients["c2_per_c2"]) * delta_t * delta_t
        )
        corrected[f"gyro_{axis}_fixed_model_dps"] = (
            corrected[f"gyro_{axis}_dps"].to_numpy(dtype=float) - temperature_delta
        )
    return corrected


def usable_seconds(path: Path, minimum_c: float, maximum_c: float) -> pd.DataFrame:
    frame = pd.read_csv(path)
    return frame[
        (frame["samples"] >= 900)
        & (frame["static_usable"] == 1)
        & (frame["temperature_c"] >= minimum_c)
        & (frame["temperature_c"] <= maximum_c)
    ].copy()


def temperature_bins(frame: pd.DataFrame) -> pd.DataFrame:
    binned = frame.copy()
    binned["temperature_bin_c"] = (
        np.round(binned["temperature_c"] / TEMPERATURE_BIN_C) * TEMPERATURE_BIN_C
    )
    counts = binned.groupby("temperature_bin_c").size()
    valid_bins = counts[counts >= 3].index
    binned = binned[binned["temperature_bin_c"].isin(valid_bins)]
    output = binned.groupby("temperature_bin_c").median(numeric_only=True).reset_index()
    output["seconds"] = output["temperature_bin_c"].map(counts).astype(int)
    return output


def anchored_curve(
    coefficients: dict[str, float],
    temperature_c: np.ndarray,
    reference_c: float,
    anchor_c: float,
) -> np.ndarray:
    delta_t = temperature_c - reference_c
    anchor_delta_t = anchor_c - reference_c
    curve = (
        float(coefficients["c1_per_c"]) * delta_t
        + float(coefficients["c2_per_c2"]) * delta_t * delta_t
    )
    anchor = (
        float(coefficients["c1_per_c"]) * anchor_delta_t
        + float(coefficients["c2_per_c2"]) * anchor_delta_t * anchor_delta_t
    )
    return curve - anchor


def run_metrics(seconds: pd.DataFrame, bins: pd.DataFrame) -> dict[str, object]:
    result: dict[str, object] = {
        "seconds": int(len(seconds)),
        "temperature_bins": int(len(bins)),
        "temperature_minimum_c": float(bins["temperature_bin_c"].min()),
        "temperature_maximum_c": float(bins["temperature_bin_c"].max()),
        "axes": {},
    }
    hot_mask = bins["temperature_bin_c"] >= (
        float(bins["temperature_bin_c"].max()) - 0.5
    )
    for axis in AXES:
        raw_bins = bins[f"gyro_{axis}_dps"].to_numpy(dtype=float)
        corrected_bins = bins[f"gyro_{axis}_fixed_model_dps"].to_numpy(dtype=float)
        raw_seconds = seconds[f"gyro_{axis}_dps"].to_numpy(dtype=float)
        corrected_seconds = seconds[f"gyro_{axis}_fixed_model_dps"].to_numpy(
            dtype=float
        )
        raw_endpoint = float(raw_bins[-1] - raw_bins[0])
        corrected_endpoint = float(corrected_bins[-1] - corrected_bins[0])
        raw_p90_span = float(
            np.percentile(raw_seconds, 95) - np.percentile(raw_seconds, 5)
        )
        corrected_p90_span = float(
            np.percentile(corrected_seconds, 95) - np.percentile(corrected_seconds, 5)
        )
        corrected_hot_reference = float(np.median(corrected_bins[hot_mask]))
        corrected_drift_from_hot = corrected_bins - corrected_hot_reference
        result["axes"][axis] = {
            "raw_endpoint_delta_dps": raw_endpoint,
            "corrected_endpoint_delta_dps": corrected_endpoint,
            "endpoint_reduction_pct": 100.0
            * (1.0 - abs(corrected_endpoint) / max(abs(raw_endpoint), 1.0e-12)),
            "raw_p90_span_dps": raw_p90_span,
            "corrected_p90_span_dps": corrected_p90_span,
            "p90_span_reduction_pct": 100.0
            * (1.0 - corrected_p90_span / max(raw_p90_span, 1.0e-12)),
            "raw_bin_peak_to_peak_dps": float(np.ptp(raw_bins)),
            "corrected_bin_peak_to_peak_dps": float(np.ptp(corrected_bins)),
            "corrected_max_abs_from_hot_dps": float(
                np.max(np.abs(corrected_drift_from_hot))
            ),
            "corrected_p95_abs_from_hot_dps": float(
                np.percentile(np.abs(corrected_drift_from_hot), 95)
            ),
        }
    return result


def plot_validation(
    first_seconds: pd.DataFrame,
    second_seconds: pd.DataFrame,
    second_bins: pd.DataFrame,
    metrics: dict[str, object],
    output_path: Path,
) -> None:
    matplotlib.rcParams["font.sans-serif"] = [
        "Microsoft YaHei",
        "SimHei",
        "DejaVu Sans",
    ]
    matplotlib.rcParams["axes.unicode_minus"] = False
    colors = {"blue": "#2878B5", "gold": "#D99A22", "pink": "#C75D8D", "ink": "#30343B"}
    fig, axes = plt.subplots(2, 2, figsize=(15, 10), constrained_layout=True)

    for label, frame, color, style in (
        ("第一轮", first_seconds, colors["blue"], "-"),
        ("第二轮", second_seconds, colors["gold"], "--"),
    ):
        sampled = frame.iloc[::30]
        axes[0, 0].plot(
            sampled["elapsed_min"],
            sampled["temperature_c"],
            color=color,
            linestyle=style,
            linewidth=1.8,
            label=label,
        )
    axes[0, 0].set_title("两轮板载温度轨迹")
    axes[0, 0].set_xlabel("时间 (min)")
    axes[0, 0].set_ylabel("温度 (°C)")
    axes[0, 0].legend(frameon=False)

    axes[0, 1].plot(
        second_bins["temperature_bin_c"],
        second_bins["gyro_y_dps"],
        color=colors["pink"],
        linewidth=2.0,
        marker="o",
        markersize=3,
        label="第二轮原始 Y",
    )
    axes[0, 1].plot(
        second_bins["temperature_bin_c"],
        second_bins["gyro_y_fixed_model_dps"],
        color=colors["blue"],
        linewidth=2.0,
        linestyle="--",
        marker="o",
        markersize=3,
        label="第一轮固定模型补偿后",
    )
    axes[0, 1].set_title("第二轮 Y 轴外部验证")
    axes[0, 1].set_xlabel("温度 (°C)")
    axes[0, 1].set_ylabel("静止输出 (°/s)")
    axes[0, 1].legend(frameon=False)

    x = np.arange(3)
    width = 0.34
    raw_endpoint = [
        abs(metrics["second_run"]["axes"][axis]["raw_endpoint_delta_dps"])
        for axis in AXES
    ]
    corrected_endpoint = [
        abs(metrics["second_run"]["axes"][axis]["corrected_endpoint_delta_dps"])
        for axis in AXES
    ]
    axes[1, 0].bar(
        x - width / 2,
        raw_endpoint,
        width,
        color=colors["gold"],
        label="原始",
    )
    axes[1, 0].bar(
        x + width / 2,
        corrected_endpoint,
        width,
        color=colors["blue"],
        label="第一轮模型补偿后",
    )
    axes[1, 0].axhline(
        DRIFT_LIMIT_DPS,
        color=colors["ink"],
        linestyle=":",
        linewidth=1.5,
        label="0.05°/s 限值",
    )
    axes[1, 0].set_xticks(x, [axis.upper() for axis in AXES])
    axes[1, 0].set_title("第二轮冷端到热端绝对变化")
    axes[1, 0].set_ylabel("绝对变化 (°/s)")
    axes[1, 0].legend(frameon=False)

    first_max = [
        metrics["first_run"]["axes"][axis]["corrected_max_abs_from_hot_dps"]
        for axis in AXES
    ]
    second_max = [
        metrics["second_run"]["axes"][axis]["corrected_max_abs_from_hot_dps"]
        for axis in AXES
    ]
    axes[1, 1].bar(
        x - width / 2,
        first_max,
        width,
        color=colors["blue"],
        label="第一轮",
    )
    axes[1, 1].bar(
        x + width / 2,
        second_max,
        width,
        color=colors["gold"],
        label="第二轮",
    )
    axes[1, 1].axhline(
        DRIFT_LIMIT_DPS,
        color=colors["ink"],
        linestyle=":",
        linewidth=1.5,
        label="0.05°/s 限值",
    )
    axes[1, 1].set_xticks(x, [axis.upper() for axis in AXES])
    axes[1, 1].set_title("固定模型补偿后相对热态最大偏移")
    axes[1, 1].set_ylabel("最大绝对偏移 (°/s)")
    axes[1, 1].legend(frameon=False)

    for axis in axes.flat:
        axis.grid(True, axis="y", color="#D9DDE3", linewidth=0.8, alpha=0.8)
        axis.spines[["top", "right"]].set_visible(False)
    fig.suptitle("BMI088 第一轮温补模型的第二轮独立验证", fontsize=17)
    fig.savefig(output_path, dpi=180, facecolor="white")
    plt.close(fig)


def analyze(
    first_results_path: Path,
    first_seconds_path: Path,
    second_results_path: Path,
    second_seconds_path: Path,
    output_dir: Path,
) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    first_results = json.loads(first_results_path.read_text(encoding="utf-8"))
    second_results = json.loads(second_results_path.read_text(encoding="utf-8"))
    first_model = first_results["gyro_model"]
    common_minimum_c = max(
        float(first_results["temperature"]["model_minimum_c"]),
        float(second_results["temperature"]["model_minimum_c"]),
    )
    common_maximum_c = min(
        float(first_results["temperature"]["model_maximum_c"]),
        float(second_results["temperature"]["model_maximum_c"]),
    )

    first_seconds = usable_seconds(
        first_seconds_path, common_minimum_c, common_maximum_c
    )
    second_seconds = usable_seconds(
        second_seconds_path, common_minimum_c, common_maximum_c
    )
    first_seconds = apply_model(
        first_seconds,
        first_model,
        float(first_results["temperature"]["model_minimum_c"]),
        float(first_results["temperature"]["model_maximum_c"]),
    )
    second_seconds = apply_model(
        second_seconds,
        first_model,
        float(first_results["temperature"]["model_minimum_c"]),
        float(first_results["temperature"]["model_maximum_c"]),
    )
    first_bins = temperature_bins(first_seconds)
    second_bins = temperature_bins(second_seconds)

    metrics: dict[str, object] = {
        "format_version": 1,
        "generated_at": datetime.now().astimezone().isoformat(),
        "validation_design": "First-run coefficients are frozen and applied unchanged to the independent second run.",
        "first_results_path": str(first_results_path.resolve()),
        "second_results_path": str(second_results_path.resolve()),
        "common_temperature_range_c": [common_minimum_c, common_maximum_c],
        "drift_limit_dps": DRIFT_LIMIT_DPS,
        "first_run_quality": first_results["quality"],
        "second_run_quality": second_results["quality"],
        "first_run": run_metrics(first_seconds, first_bins),
        "second_run": run_metrics(second_seconds, second_bins),
        "curve_repeatability": {},
    }

    temperature_grid = np.arange(
        common_minimum_c, common_maximum_c + TEMPERATURE_BIN_C / 2.0, TEMPERATURE_BIN_C
    )
    for axis in AXES:
        first_curve = anchored_curve(
            first_model["coefficients_dps"][axis],
            temperature_grid,
            float(first_model["reference_temperature_c"]),
            common_maximum_c,
        )
        second_model = second_results["gyro_model"]
        second_curve = anchored_curve(
            second_model["coefficients_dps"][axis],
            temperature_grid,
            float(second_model["reference_temperature_c"]),
            common_maximum_c,
        )
        difference = first_curve - second_curve
        metrics["curve_repeatability"][axis] = {
            "maximum_absolute_difference_dps": float(np.max(np.abs(difference))),
            "rmse_difference_dps": float(np.sqrt(np.mean(difference * difference))),
        }

    second_axes = metrics["second_run"]["axes"]
    dominant_axis_pass = (
        abs(second_axes["y"]["corrected_endpoint_delta_dps"]) <= DRIFT_LIMIT_DPS
        and second_axes["y"]["endpoint_reduction_pct"] >= 80.0
        and second_axes["y"]["p90_span_reduction_pct"] >= 50.0
    )
    absolute_limit_pass = all(
        second_axes[axis]["corrected_max_abs_from_hot_dps"] <= DRIFT_LIMIT_DPS
        for axis in AXES
    )
    strict_peak_to_peak_pass = all(
        second_axes[axis]["corrected_bin_peak_to_peak_dps"] <= DRIFT_LIMIT_DPS
        for axis in AXES
    )
    no_axis_degradation = all(
        abs(second_axes[axis]["corrected_endpoint_delta_dps"])
        <= abs(second_axes[axis]["raw_endpoint_delta_dps"])
        for axis in AXES
    )
    transport_pass = bool(
        second_results["quality"]["overall_pass_for_temperature_fitting"]
        and second_results["quality"]["frame_crc_errors"] == 0
        and second_results["quality"]["frame_sequence_gaps"] == 0
        and second_results["quality"]["sample_sequence_gaps"] == 0
    )
    status = (
        "pass"
        if transport_pass
        and dominant_axis_pass
        and absolute_limit_pass
        and strict_peak_to_peak_pass
        and no_axis_degradation
        else (
            "pass_with_caveat"
            if transport_pass and dominant_axis_pass and absolute_limit_pass
            else "fail"
        )
    )
    metrics["decision"] = {
        "status": status,
        "transport_pass": transport_pass,
        "dominant_y_axis_cross_validation_pass": dominant_axis_pass,
        "all_axes_max_abs_from_hot_within_0_05_dps": absolute_limit_pass,
        "all_axes_peak_to_peak_within_0_05_dps": strict_peak_to_peak_pass,
        "no_axis_endpoint_degradation": no_axis_degradation,
        "ready_for_default_flight_enable": status == "pass",
        "reason": (
            "The frozen first-run model validates strongly on Y and every axis stays within the absolute 0.05 deg/s limit, "
            "but X/Z do not consistently improve and Z has a borderline peak-to-peak excursion."
            if status == "pass_with_caveat"
            else (
                "All cross-validation criteria passed."
                if status == "pass"
                else "One or more transport, dominant-axis, or absolute drift criteria failed."
            )
        ),
    }

    first_bins.insert(0, "run", "round1")
    second_bins.insert(0, "run", "round2")
    combined_bins = pd.concat([first_bins, second_bins], ignore_index=True)
    combined_bins.to_csv(output_dir / "temperature_validation_bins.csv", index=False)
    second_seconds.to_csv(
        output_dir / "temperature_validation_seconds.csv", index=False
    )
    (output_dir / "temperature_validation_results.json").write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    plot_validation(
        first_seconds,
        second_seconds,
        second_bins,
        metrics,
        output_dir / "temperature_validation_overview.png",
    )
    return metrics


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--first-results", type=Path, required=True)
    parser.add_argument("--first-seconds", type=Path, required=True)
    parser.add_argument("--second-results", type=Path, required=True)
    parser.add_argument("--second-seconds", type=Path, required=True)
    parser.add_argument("-o", "--output", type=Path, required=True)
    arguments = parser.parse_args()
    result = analyze(
        arguments.first_results,
        arguments.first_seconds,
        arguments.second_results,
        arguments.second_seconds,
        arguments.output,
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
