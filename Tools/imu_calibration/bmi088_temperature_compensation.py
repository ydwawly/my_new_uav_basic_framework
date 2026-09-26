"""Shared BMI088 gyroscope temperature compensation for host-side tools."""

from __future__ import annotations

from typing import Any

import numpy as np


GYRO_TEMP_REFERENCE_C = 45.7043125
GYRO_TEMP_MINIMUM_C = 27.25
GYRO_TEMP_MAXIMUM_C = 46.0
GYRO_TEMP_C1_RPS_PER_C = np.array(
    [-3.2220137e-05, -2.6622598e-04, -2.6712068e-05], dtype=np.float64
)
GYRO_TEMP_C2_RPS_PER_C2 = np.array(
    [-1.2759631e-06, -3.9388435e-06, -3.7307752e-06], dtype=np.float64
)


def gyro_temperature_correction_rps(temperature_c: Any) -> np.ndarray:
    """Return modeled gyro bias change in rad/s, clamping temperature to fit range."""
    temperature = np.asarray(temperature_c, dtype=np.float64)
    if not np.all(np.isfinite(temperature)):
        raise ValueError("BMI088 温度包含 NaN 或无穷值")

    delta_temperature = (
        np.clip(temperature, GYRO_TEMP_MINIMUM_C, GYRO_TEMP_MAXIMUM_C)
        - GYRO_TEMP_REFERENCE_C
    )
    return (
        delta_temperature[..., np.newaxis] * GYRO_TEMP_C1_RPS_PER_C
        + np.square(delta_temperature)[..., np.newaxis] * GYRO_TEMP_C2_RPS_PER_C2
    )


def compensate_gyro_rps(gyro_rps: Any, temperature_c: Any) -> np.ndarray:
    """Subtract modeled temperature-dependent bias from gyro samples."""
    gyro = np.asarray(gyro_rps, dtype=np.float64)
    if gyro.ndim == 0 or gyro.shape[-1] != 3:
        raise ValueError("陀螺仪数据最后一维必须为 3")
    if not np.all(np.isfinite(gyro)):
        raise ValueError("陀螺仪数据包含 NaN 或无穷值")

    correction = gyro_temperature_correction_rps(temperature_c)
    try:
        return gyro - correction
    except ValueError as exc:
        raise ValueError("温度数量与陀螺仪样本数量不匹配") from exc


def temperature_compensation_metadata() -> dict[str, Any]:
    """Return serializable model metadata for calibration reports."""
    return {
        "enabled": True,
        "model": "quadratic_bias_delta_clamped",
        "reference_temperature_c": GYRO_TEMP_REFERENCE_C,
        "valid_temperature_range_c": [GYRO_TEMP_MINIMUM_C, GYRO_TEMP_MAXIMUM_C],
        "c1_rps_per_c": GYRO_TEMP_C1_RPS_PER_C.tolist(),
        "c2_rps_per_c2": GYRO_TEMP_C2_RPS_PER_C2.tolist(),
        "formula": "gyro_corrected = gyro_raw - (c1*dT + c2*dT^2)",
    }
