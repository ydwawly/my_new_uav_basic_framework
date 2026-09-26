#!/usr/bin/env python3
"""Validate the compiled round-3 BMI088 gyro temperature model across three runs."""

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
MODEL = {
    "reference_temperature_c": 45.7043125,
    "minimum_temperature_c": 27.25,
    "maximum_temperature_c": 46.0,
    "coefficients_rps": {
        "x": {"c1_per_c": -3.2220137e-05, "c2_per_c2": -1.2759631e-06},
        "y": {"c1_per_c": -2.6622598e-04, "c2_per_c2": -3.9388435e-06},
        "z": {"c1_per_c": -2.6712068e-05, "c2_per_c2": -3.7307752e-06},
    },
    "derivation": (
        "Round-1 Y coefficients retained; round-1 X/Z coefficients multiplied "
        "by 0.591 and 0.771 using rounds 1-2 only."
    ),
}


def load_usable_seconds(path: Path, minimum_c: float, maximum_c: float) -> pd.DataFrame:
    frame = pd.read_csv(path)
    return frame[
        (frame["samples"] >= 900)
        & (frame["static_usable"] == 1)
        & (frame["temperature_c"] >= minimum_c)
        & (frame["temperature_c"] <= maximum_c)
    ].copy()


def apply_model(frame: pd.DataFrame) -> pd.DataFrame:
    output = frame.copy()
    temperature_c = np.clip(
        output["temperature_c"].to_numpy(dtype=float),
        float(MODEL["minimum_temperature_c"]),
        float(MODEL["maximum_temperature_c"]),
    )
    delta_t = temperature_c - float(MODEL["reference_temperature_c"])
    for axis in AXES:
        coefficients = MODEL["coefficients_rps"][axis]
        delta_rps = (
            float(coefficients["c1_per_c"]) * delta_t
            + float(coefficients["c2_per_c2"]) * delta_t * delta_t
        )
        output[f"gyro_{axis}_round3_model_dps"] = (
            output[f"gyro_{axis}_dps"].to_numpy(dtype=float)
            - np.rad2deg(delta_rps)
        )
    return output


def make_bins(frame: pd.DataFrame) -> pd.DataFrame:
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


def calculate_metrics(seconds: pd.DataFrame, bins: pd.DataFrame) -> dict[str, object]:
    result: dict[str, object] = {
        "usable_seconds": int(len(seconds)),
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
        corrected_bins = bins[f"gyro_{axis}_round3_model_dps"].to_numpy(dtype=float)
        raw_seconds = seconds[f"gyro_{axis}_dps"].to_numpy(dtype=float)
        corrected_seconds = seconds[f"gyro_{axis}_round3_model_dps"].to_numpy(
            dtype=float
        )
        raw_endpoint = float(raw_bins[-1] - raw_bins[0])
        corrected_endpoint = float(corrected_bins[-1] - corrected_bins[0])
        raw_p90 = float(np.percentile(raw_seconds, 95) - np.percentile(raw_seconds, 5))
        corrected_p90 = float(
            np.percentile(corrected_seconds, 95)
            - np.percentile(corrected_seconds, 5)
        )
        hot_reference = float(np.median(corrected_bins[hot_mask]))
        drift_from_hot = corrected_bins - hot_reference
        result["axes"][axis] = {
            "raw_endpoint_delta_dps": raw_endpoint,
            "corrected_endpoint_delta_dps": corrected_endpoint,
            "endpoint_reduction_pct": 100.0
            * (1.0 - abs(corrected_endpoint) / max(abs(raw_endpoint), 1.0e-12)),
            "raw_p90_span_dps": raw_p90,
            "corrected_p90_span_dps": corrected_p90,
            "p90_span_reduction_pct": 100.0
            * (1.0 - corrected_p90 / max(raw_p90, 1.0e-12)),
            "raw_bin_peak_to_peak_dps": float(np.ptp(raw_bins)),
            "corrected_bin_peak_to_peak_dps": float(np.ptp(corrected_bins)),
            "corrected_max_abs_from_hot_dps": float(
                np.max(np.abs(drift_from_hot))
            ),
            "corrected_p95_abs_from_hot_dps": float(
                np.percentile(np.abs(drift_from_hot), 95)
            ),
        }
    return result


def plot_results(
    seconds_by_run: dict[str, pd.DataFrame],
    bins_by_run: dict[str, pd.DataFrame],
    metrics: dict[str, object],
    output_path: Path,
) -> None:
    matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False
    colors = {"round1": "#2878B5", "round2": "#D99A22", "round3": "#C75D8D"}
    labels = {"round1": "第一轮", "round2": "第二轮", "round3": "第三轮"}
    fig, axes = plt.subplots(2, 2, figsize=(15, 10), constrained_layout=True)

    for run, frame in seconds_by_run.items():
        sampled = frame.iloc[::30]
        axes[0, 0].plot(
            sampled["elapsed_min"],
            sampled["temperature_c"],
            color=colors[run],
            linewidth=1.8,
            label=labels[run],
        )
    axes[0, 0].set_title("三轮板载温度轨迹")
    axes[0, 0].set_xlabel("时间 (min)")
    axes[0, 0].set_ylabel("温度 (°C)")
    axes[0, 0].legend(frameon=False)

    third_bins = bins_by_run["round3"]
    axes[0, 1].plot(
        third_bins["temperature_bin_c"],
        third_bins["gyro_y_dps"],
        color=colors["round3"],
        linewidth=2.0,
        marker="o",
        markersize=3,
        label="第三轮原始 Y",
    )
    axes[0, 1].plot(
        third_bins["temperature_bin_c"],
        third_bins["gyro_y_round3_model_dps"],
        color=colors["round1"],
        linewidth=2.0,
        linestyle="--",
        marker="o",
        markersize=3,
        label="冻结第三版模型补偿后",
    )
    axes[0, 1].set_title("第三轮 Y 轴独立验证")
    axes[0, 1].set_xlabel("温度 (°C)")
    axes[0, 1].set_ylabel("静止输出 (°/s)")
    axes[0, 1].legend(frameon=False)

    x = np.arange(3)
    width = 0.34
    third_axes = metrics["runs"]["round3"]["axes"]
    raw_endpoint = [abs(third_axes[axis]["raw_endpoint_delta_dps"]) for axis in AXES]
    corrected_endpoint = [
        abs(third_axes[axis]["corrected_endpoint_delta_dps"]) for axis in AXES
    ]
    axes[1, 0].bar(x - width / 2, raw_endpoint, width, color="#D99A22", label="原始")
    axes[1, 0].bar(
        x + width / 2,
        corrected_endpoint,
        width,
        color="#2878B5",
        label="第三版补偿后",
    )
    axes[1, 0].axhline(
        DRIFT_LIMIT_DPS,
        color="#30343B",
        linestyle=":",
        linewidth=1.5,
        label="0.05°/s 限值",
    )
    axes[1, 0].set_xticks(x, [axis.upper() for axis in AXES])
    axes[1, 0].set_title("第三轮冷到热端点绝对变化")
    axes[1, 0].set_ylabel("绝对变化 (°/s)")
    axes[1, 0].legend(frameon=False)

    group_width = 0.22
    for index, run in enumerate(("round1", "round2", "round3")):
        values = [
            metrics["runs"][run]["axes"][axis]["corrected_bin_peak_to_peak_dps"]
            for axis in AXES
        ]
        axes[1, 1].bar(
            x + (index - 1) * group_width,
            values,
            group_width,
            color=colors[run],
            label=labels[run],
        )
    axes[1, 1].axhline(
        DRIFT_LIMIT_DPS,
        color="#30343B",
        linestyle=":",
        linewidth=1.5,
        label="0.05°/s 限值",
    )
    axes[1, 1].set_xticks(x, [axis.upper() for axis in AXES])
    axes[1, 1].set_title("三轮补偿后温度分箱峰峰值")
    axes[1, 1].set_ylabel("峰峰值 (°/s)")
    axes[1, 1].legend(frameon=False)

    for axis in axes.flat:
        axis.grid(True, axis="y", color="#D9DDE3", linewidth=0.8, alpha=0.8)
        axis.spines[["top", "right"]].set_visible(False)
    fig.suptitle("BMI088 第三版温补模型独立验证", fontsize=17)
    fig.savefig(output_path, dpi=180, facecolor="white")
    plt.close(fig)


def analyze(
    result_paths: dict[str, Path],
    seconds_paths: dict[str, Path],
    output_dir: Path,
) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    details = {
        run: json.loads(path.read_text(encoding="utf-8"))
        for run, path in result_paths.items()
    }
    common_minimum_c = max(
        float(detail["temperature"]["model_minimum_c"]) for detail in details.values()
    )
    common_maximum_c = min(
        float(detail["temperature"]["model_maximum_c"]) for detail in details.values()
    )
    seconds_by_run: dict[str, pd.DataFrame] = {}
    bins_by_run: dict[str, pd.DataFrame] = {}
    for run in result_paths:
        seconds = load_usable_seconds(
            seconds_paths[run], common_minimum_c, common_maximum_c
        )
        seconds = apply_model(seconds)
        seconds_by_run[run] = seconds
        bins_by_run[run] = make_bins(seconds)

    metrics: dict[str, object] = {
        "format_version": 1,
        "generated_at": datetime.now().astimezone().isoformat(),
        "validation_design": (
            "Round-3 model was selected from rounds 1-2 and applied unchanged "
            "to the independent round-3 capture."
        ),
        "model": MODEL,
        "common_temperature_range_c": [common_minimum_c, common_maximum_c],
        "drift_limit_dps": DRIFT_LIMIT_DPS,
        "quality": {run: details[run]["quality"] for run in details},
        "temperature": {run: details[run]["temperature"] for run in details},
        "runs": {
            run: calculate_metrics(seconds_by_run[run], bins_by_run[run])
            for run in result_paths
        },
    }

    all_transport_pass = all(
        detail["quality"]["overall_pass_for_temperature_fitting"]
        and detail["quality"]["frame_crc_errors"] == 0
        and detail["quality"]["frame_sequence_gaps"] == 0
        and detail["quality"]["sample_sequence_gaps"] == 0
        and detail["quality"]["timestamp_interval_outliers"] == 0
        for detail in details.values()
    )
    third_axes = metrics["runs"]["round3"]["axes"]
    round3_endpoint_pass = all(
        abs(third_axes[axis]["corrected_endpoint_delta_dps"]) <= DRIFT_LIMIT_DPS
        for axis in AXES
    )
    round3_no_endpoint_degradation = all(
        abs(third_axes[axis]["corrected_endpoint_delta_dps"])
        <= abs(third_axes[axis]["raw_endpoint_delta_dps"])
        for axis in AXES
    )
    y_independent_pass = (
        abs(third_axes["y"]["corrected_endpoint_delta_dps"]) <= 0.01
        and third_axes["y"]["endpoint_reduction_pct"] >= 90.0
        and third_axes["y"]["p90_span_reduction_pct"] >= 50.0
    )
    all_runs_p2p_pass = all(
        metrics["runs"][run]["axes"][axis]["corrected_bin_peak_to_peak_dps"]
        <= DRIFT_LIMIT_DPS
        for run in result_paths
        for axis in AXES
    )
    all_runs_hot_pass = all(
        metrics["runs"][run]["axes"][axis]["corrected_max_abs_from_hot_dps"]
        <= DRIFT_LIMIT_DPS
        for run in result_paths
        for axis in AXES
    )
    ready = bool(
        all_transport_pass
        and round3_endpoint_pass
        and round3_no_endpoint_degradation
        and y_independent_pass
        and all_runs_p2p_pass
        and all_runs_hot_pass
    )
    metrics["decision"] = {
        "status": "pass" if ready else "fail",
        "all_transport_pass": all_transport_pass,
        "independent_round3_all_axes_endpoint_within_0_05_dps": round3_endpoint_pass,
        "independent_round3_no_axis_endpoint_degradation": round3_no_endpoint_degradation,
        "independent_round3_y_axis_pass": y_independent_pass,
        "all_three_runs_peak_to_peak_within_0_05_dps": all_runs_p2p_pass,
        "all_three_runs_max_abs_from_hot_within_0_05_dps": all_runs_hot_pass,
        "ready_for_default_enable_on_this_board": ready,
        "reason": (
            "The model selected from rounds 1-2 passed all frozen-model criteria on the independent third run and kept every axis below 0.05 deg/s across all three runs."
            if ready
            else "One or more independent or all-run validation criteria failed."
        ),
    }

    combined_bins = []
    for run, bins in bins_by_run.items():
        output = bins.copy()
        output.insert(0, "run", run)
        combined_bins.append(output)
    pd.concat(combined_bins, ignore_index=True).to_csv(
        output_dir / "temperature_round3_validation_bins.csv", index=False
    )
    third_seconds = seconds_by_run["round3"].copy()
    third_seconds.to_csv(
        output_dir / "temperature_round3_validation_seconds.csv", index=False
    )
    (output_dir / "temperature_round3_validation_results.json").write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    plot_results(
        seconds_by_run,
        bins_by_run,
        metrics,
        output_dir / "temperature_round3_validation_overview.png",
    )
    return metrics


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    for run in ("round1", "round2", "round3"):
        parser.add_argument(f"--{run}-results", type=Path, required=True)
        parser.add_argument(f"--{run}-seconds", type=Path, required=True)
    parser.add_argument("-o", "--output", type=Path, required=True)
    arguments = parser.parse_args()
    result = analyze(
        {
            "round1": arguments.round1_results,
            "round2": arguments.round2_results,
            "round3": arguments.round3_results,
        },
        {
            "round1": arguments.round1_seconds,
            "round2": arguments.round2_seconds,
            "round3": arguments.round3_seconds,
        },
        arguments.output,
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
