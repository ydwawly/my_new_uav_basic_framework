#!/usr/bin/env python3
"""Pure parser and acceptance logic for guided BMI088 LM data capture."""

from __future__ import annotations

import math
import struct
from binascii import crc_hqx
from collections import deque
from dataclasses import asdict, dataclass, field
from typing import Iterable

import numpy as np

from bmi088_temperature_compensation import (
    compensate_gyro_rps,
    temperature_compensation_metadata,
)


FRAME_MAGIC = 0xA55A
IMU_MESSAGE_ID = 3
FRAME_HEADER = struct.Struct("<HBBHHIQ")
IMU_PAYLOAD_V1 = struct.Struct("<II3h3hhH3f3ff")
FRAME_MAGIC_BYTES = struct.pack("<H", FRAME_MAGIC)
GRAVITY_MPS2 = 9.80665
TEMPERATURE_VALID_MASK = 1 << 2


def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    return crc_hqx(data, initial)


@dataclass(frozen=True)
class ImuSample:
    timestamp_us: int
    frame_sequence: int
    sample_sequence: int
    accel_sensor_time: int
    validity_flags: int
    accel_mps2: tuple[float, float, float]
    gyro_rps: tuple[float, float, float]
    temperature_c: float


@dataclass(frozen=True)
class ParsedFrame:
    raw: bytes
    sample: ImuSample | None


@dataclass
class ParserStats:
    frames: int = 0
    imu_samples: int = 0
    crc_errors: int = 0
    resync_bytes: int = 0
    sequence_gaps: int = 0
    duplicate_or_backward_frames: int = 0
    sample_sequence_gaps: int = 0
    duplicate_or_backward_samples: int = 0
    timestamp_nonmonotonic: int = 0
    timestamp_interval_outliers: int = 0
    minimum_interval_us: int = 0
    maximum_interval_us: int = 0


@dataclass(frozen=True)
class ThermalStatus:
    valid: bool
    stable: bool
    current_c: float
    range_c: float
    slope_c_per_min: float
    window_seconds: float
    reason: str


class TemperatureMonitor:
    """Downsample BMI088 temperature and judge whether self-heating has settled."""

    def __init__(
        self,
        window_seconds: float = 120.0,
        minimum_window_seconds: float = 90.0,
        maximum_range_c: float = 0.5,
        maximum_abs_slope_c_per_min: float = 0.1,
        entry_confirmation_seconds: float = 10.0,
        exit_maximum_range_c: float = 0.75,
        exit_maximum_abs_slope_c_per_min: float = 0.15,
        exit_confirmation_seconds: float = 30.0,
    ) -> None:
        if not 0.0 <= minimum_window_seconds <= window_seconds:
            raise ValueError("热稳定最短历史必须处于 0 到窗口时长之间")
        if exit_maximum_range_c < maximum_range_c:
            raise ValueError("退出稳定的温差门槛不能小于进入门槛")
        if exit_maximum_abs_slope_c_per_min < maximum_abs_slope_c_per_min:
            raise ValueError("退出稳定的斜率门槛不能小于进入门槛")
        if entry_confirmation_seconds < 0.0 or exit_confirmation_seconds < 0.0:
            raise ValueError("热稳定确认时间不能为负数")
        self.window_seconds = window_seconds
        self.minimum_window_seconds = minimum_window_seconds
        self.maximum_range_c = maximum_range_c
        self.maximum_abs_slope_c_per_min = maximum_abs_slope_c_per_min
        self.entry_confirmation_seconds = entry_confirmation_seconds
        self.exit_maximum_range_c = exit_maximum_range_c
        self.exit_maximum_abs_slope_c_per_min = exit_maximum_abs_slope_c_per_min
        self.exit_confirmation_seconds = exit_confirmation_seconds
        self._points: deque[tuple[int, float]] = deque()
        self._last_sampled_timestamp_us: int | None = None
        self._stable_latched = False
        self._entry_candidate_since_us: int | None = None
        self._exit_candidate_since_us: int | None = None
        self.status = self._invalid("等待 BMI088 有效温度")

    def reset(self) -> None:
        self._points.clear()
        self._last_sampled_timestamp_us = None
        self._stable_latched = False
        self._entry_candidate_since_us = None
        self._exit_candidate_since_us = None
        self.status = self._invalid("等待 BMI088 有效温度")

    def process(self, sample: ImuSample) -> ThermalStatus:
        if (
            (sample.validity_flags & TEMPERATURE_VALID_MASK) == 0
            or not math.isfinite(sample.temperature_c)
            or not -40.0 <= sample.temperature_c <= 85.0
        ):
            self.reset()
            self.status = self._invalid("固件未提供有效 BMI088 温度")
            return self.status

        if (
            self._last_sampled_timestamp_us is not None
            and sample.timestamp_us <= self._last_sampled_timestamp_us
        ):
            self.reset()
        should_sample = (
            self._last_sampled_timestamp_us is None
            or sample.timestamp_us - self._last_sampled_timestamp_us >= 1_000_000
            or not self.status.valid
        )
        if not should_sample:
            return self.status

        self._points.append((sample.timestamp_us, sample.temperature_c))
        self._last_sampled_timestamp_us = sample.timestamp_us
        cutoff = sample.timestamp_us - int(self.window_seconds * 1e6)
        while self._points and self._points[0][0] < cutoff:
            self._points.popleft()

        if not self._points:
            self.status = self._invalid("等待 BMI088 有效温度")
            return self.status
        window_seconds = (self._points[-1][0] - self._points[0][0]) * 1e-6
        temperatures = np.asarray([point[1] for point in self._points])
        temperature_range = float(np.ptp(temperatures))
        robust_temperature_range = float(
            np.percentile(temperatures, 95.0) - np.percentile(temperatures, 5.0)
        )
        slope_c_per_min = 0.0
        if len(self._points) >= 2 and window_seconds > 0.0:
            seconds = np.asarray(
                [(point[0] - self._points[0][0]) * 1e-6 for point in self._points]
            )
            centered_seconds = seconds - float(np.mean(seconds))
            denominator = float(np.dot(centered_seconds, centered_seconds))
            if denominator > 0.0:
                centered_temperatures = temperatures - float(np.mean(temperatures))
                slope_c_per_min = (
                    float(np.dot(centered_seconds, centered_temperatures))
                    / denominator
                    * 60.0
                )
        stable, reason = self._update_stability(
            sample.timestamp_us,
            window_seconds,
            temperature_range,
            robust_temperature_range,
            slope_c_per_min,
        )
        self.status = ThermalStatus(
            valid=True,
            stable=stable,
            current_c=sample.temperature_c,
            range_c=temperature_range,
            slope_c_per_min=slope_c_per_min,
            window_seconds=window_seconds,
            reason=reason,
        )
        return self.status

    def _update_stability(
        self,
        timestamp_us: int,
        window_seconds: float,
        temperature_range: float,
        robust_temperature_range: float,
        slope_c_per_min: float,
    ) -> tuple[bool, str]:
        enough_history = window_seconds >= self.minimum_window_seconds
        entry_conditions = (
            enough_history
            and temperature_range <= self.maximum_range_c
            and abs(slope_c_per_min) <= self.maximum_abs_slope_c_per_min
        )
        exit_conditions = (
            not enough_history
            or robust_temperature_range > self.exit_maximum_range_c
            or abs(slope_c_per_min) > self.exit_maximum_abs_slope_c_per_min
        )

        if not self._stable_latched:
            self._exit_candidate_since_us = None
            if not entry_conditions:
                self._entry_candidate_since_us = None
                if not enough_history:
                    return (
                        False,
                        f"正在建立热稳定窗口（{window_seconds:.0f}/{self.minimum_window_seconds:.0f} s）",
                    )
                if temperature_range > self.maximum_range_c:
                    return False, f"温度波动 {temperature_range:.2f}°C，继续等待"
                return False, f"温度仍以 {slope_c_per_min:+.2f}°C/min 变化"

            if self._entry_candidate_since_us is None:
                self._entry_candidate_since_us = timestamp_us
            confirmation_seconds = (
                timestamp_us - self._entry_candidate_since_us
            ) * 1e-6
            if confirmation_seconds < self.entry_confirmation_seconds:
                return (
                    False,
                    f"热稳定确认中（{confirmation_seconds:.0f}/{self.entry_confirmation_seconds:.0f} s）",
                )
            self._stable_latched = True
            self._entry_candidate_since_us = None
            return True, "温度已稳定，可以开始 LM 姿态采集"

        self._entry_candidate_since_us = None
        if not exit_conditions:
            self._exit_candidate_since_us = None
            if entry_conditions:
                return True, "温度已稳定，可以开始 LM 姿态采集"
            return True, "温度轻微波动，迟滞保护中，仍可采集"

        if self._exit_candidate_since_us is None:
            self._exit_candidate_since_us = timestamp_us
        violation_seconds = (timestamp_us - self._exit_candidate_since_us) * 1e-6
        if violation_seconds < self.exit_confirmation_seconds:
            return (
                True,
                f"温度短时波动，观察中（{violation_seconds:.0f}/{self.exit_confirmation_seconds:.0f} s）",
            )

        self._stable_latched = False
        self._exit_candidate_since_us = None
        return False, "温度持续超限，已暂停姿态计时"

    @staticmethod
    def _invalid(reason: str) -> ThermalStatus:
        return ThermalStatus(
            valid=False,
            stable=False,
            current_c=math.nan,
            range_c=math.inf,
            slope_c_per_min=math.inf,
            window_seconds=0.0,
            reason=reason,
        )


class FlogStreamParser:
    """Incrementally validate and decode FLOG frames from arbitrarily split chunks."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self._last_frame_sequence: int | None = None
        self._last_sample_sequence: int | None = None
        self._last_timestamp_us: int | None = None
        self.stats = ParserStats()

    def reset_statistics(self, clear_buffer: bool = False) -> None:
        if clear_buffer:
            self._buffer.clear()
        self._last_frame_sequence = None
        self._last_sample_sequence = None
        self._last_timestamp_us = None
        self.stats = ParserStats()

    def feed(self, chunk: bytes) -> list[ParsedFrame]:
        self._buffer.extend(chunk)
        output: list[ParsedFrame] = []
        minimum_frame_size = FRAME_HEADER.size + 2

        while len(self._buffer) >= minimum_frame_size:
            if self._buffer[:2] != FRAME_MAGIC_BYTES:
                offset = self._buffer.find(FRAME_MAGIC_BYTES, 1)
                if offset < 0:
                    keep = 1 if self._buffer[-1] == FRAME_MAGIC_BYTES[0] else 0
                    discarded = len(self._buffer) - keep
                    self.stats.resync_bytes += discarded
                    del self._buffer[:discarded]
                    break
                self.stats.resync_bytes += offset
                del self._buffer[:offset]
                if len(self._buffer) < minimum_frame_size:
                    break

            header = FRAME_HEADER.unpack_from(self._buffer)
            payload_length = header[3]
            if payload_length > 300:
                self.stats.resync_bytes += 1
                del self._buffer[0]
                continue
            frame_size = FRAME_HEADER.size + payload_length + 2
            if len(self._buffer) < frame_size:
                break

            raw = bytes(self._buffer[:frame_size])
            stored_crc = struct.unpack_from("<H", raw, frame_size - 2)[0]
            if crc16_ccitt(raw[:-2]) != stored_crc:
                self.stats.crc_errors += 1
                del self._buffer[0]
                continue
            del self._buffer[:frame_size]

            _, message_id, message_version, _, _, frame_sequence, timestamp_us = header
            self._track_sequence(frame_sequence)
            self.stats.frames += 1
            payload = raw[FRAME_HEADER.size : -2]
            sample = self._decode_sample(
                message_id, message_version, frame_sequence, timestamp_us, payload
            )
            if sample is not None:
                self.stats.imu_samples += 1
                self._track_sample(sample)
            output.append(ParsedFrame(raw=raw, sample=sample))
        return output

    def _track_sequence(self, sequence: int) -> None:
        if self._last_frame_sequence is not None:
            delta = (sequence - self._last_frame_sequence) & 0xFFFFFFFF
            if delta == 0 or delta >= 0x80000000:
                self.stats.duplicate_or_backward_frames += 1
            elif delta > 1:
                self.stats.sequence_gaps += delta - 1
        self._last_frame_sequence = sequence

    def _track_sample(self, sample: ImuSample) -> None:
        if self._last_sample_sequence is not None:
            delta = (sample.sample_sequence - self._last_sample_sequence) & 0xFFFFFFFF
            if delta == 0 or delta >= 0x80000000:
                self.stats.duplicate_or_backward_samples += 1
            elif delta > 1:
                self.stats.sample_sequence_gaps += delta - 1
        if self._last_timestamp_us is not None:
            interval = sample.timestamp_us - self._last_timestamp_us
            if interval <= 0:
                self.stats.timestamp_nonmonotonic += 1
            else:
                if self.stats.minimum_interval_us == 0:
                    self.stats.minimum_interval_us = interval
                else:
                    self.stats.minimum_interval_us = min(
                        self.stats.minimum_interval_us, interval
                    )
                self.stats.maximum_interval_us = max(
                    self.stats.maximum_interval_us, interval
                )
                if not 500 <= interval <= 2_000:
                    self.stats.timestamp_interval_outliers += 1
        self._last_sample_sequence = sample.sample_sequence
        self._last_timestamp_us = sample.timestamp_us

    @staticmethod
    def _decode_sample(
        message_id: int,
        message_version: int,
        frame_sequence: int,
        timestamp_us: int,
        payload: bytes,
    ) -> ImuSample | None:
        if (
            message_id != IMU_MESSAGE_ID
            or message_version != 1
            or len(payload) != IMU_PAYLOAD_V1.size
        ):
            return None
        fields = IMU_PAYLOAD_V1.unpack(payload)
        return ImuSample(
            timestamp_us=timestamp_us,
            frame_sequence=frame_sequence,
            sample_sequence=fields[0],
            accel_sensor_time=fields[1],
            validity_flags=fields[9],
            accel_mps2=tuple(float(value) for value in fields[10:13]),
            gyro_rps=tuple(float(value) for value in fields[13:16]),
            temperature_c=float(fields[16]),
        )


@dataclass(frozen=True)
class TargetPose:
    code: str
    label: str
    direction: tuple[float, float, float]


def _normalized(values: Iterable[float]) -> tuple[float, float, float]:
    vector = np.asarray(tuple(values), dtype=np.float64)
    vector /= np.linalg.norm(vector)
    return tuple(float(value) for value in vector)


def _pose(code: str, label: str, direction: tuple[int, int, int]) -> TargetPose:
    return TargetPose(code=code, label=label, direction=_normalized(direction))


# The route keeps adjacent targets at least 60 degrees apart and strongly excites all gyro axes.
TARGET_POSES: tuple[TargetPose, ...] = (
    _pose("-Z", "-Z 朝上（飞控板正常平放）", (0, 0, -1)),
    _pose("-X+Z", "-X 与 +Z 斜向朝上", (-1, 0, 1)),
    _pose("+X-Y", "+X 与 -Y 斜向朝上", (1, -1, 0)),
    _pose("+Y-Z", "+Y 与 -Z 斜向朝上", (0, 1, -1)),
    _pose("-X-Y", "-X 与 -Y 斜向朝上", (-1, -1, 0)),
    _pose("+X+Z", "+X 与 +Z 斜向朝上", (1, 0, 1)),
    _pose("-Y-Z", "-Y 与 -Z 斜向朝上", (0, -1, -1)),
    _pose("-X", "-X 朝上", (-1, 0, 0)),
    _pose("+X+Y", "+X 与 +Y 斜向朝上", (1, 1, 0)),
    _pose("-Y+Z", "-Y 与 +Z 斜向朝上", (0, -1, 1)),
    _pose("-X+Y", "-X 与 +Y 斜向朝上", (-1, 1, 0)),
    _pose("-X-Z", "-X 与 -Z 斜向朝上", (-1, 0, -1)),
    _pose("-Y", "-Y 朝上", (0, -1, 0)),
    _pose("+Y+Z", "+Y 与 +Z 斜向朝上", (0, 1, 1)),
    _pose("+X", "+X 朝上", (1, 0, 0)),
    _pose("+Z", "+Z 朝上（飞控板倒扣）", (0, 0, 1)),
    _pose("+Y", "+Y 朝上", (0, 1, 0)),
    _pose("+X-Z", "+X 与 -Z 斜向朝上", (1, 0, -1)),
)


@dataclass(frozen=True)
class GuidedCriteria:
    angle_tolerance_deg: float = 8.0
    gyro_rms_threshold_rps: float = 0.035
    accel_norm_min_mps2: float = 0.88 * GRAVITY_MPS2
    accel_norm_max_mps2: float = 1.12 * GRAVITY_MPS2
    accel_rms_deviation_threshold_mps2: float = 0.12
    hold_seconds: float = 3.0
    hold_dropout_grace_seconds: float = 0.35
    evaluation_window_seconds: float = 0.20
    coverage_min_eigenvalue: float = 0.20
    max_gyro_bias_drift_dps: float = 0.05
    minimum_usable_transitions: int = 12
    minimum_axis_excitation_deg: float = 180.0
    recommended_transition_seconds: float = 30.0
    maximum_transition_seconds: float = 90.0
    maximum_pose_temperature_span_c: float = 1.0
    pose_temperature_warning_span_c: float = 0.8


@dataclass
class PoseResult:
    code: str
    label: str
    target_direction: list[float]
    mean_accel_mps2: list[float]
    mean_gyro_rps: list[float]
    mean_temperature_c: float
    accepted_timestamp_us: int
    samples: int
    angle_error_deg: float


@dataclass
class TransitionResult:
    from_code: str
    to_code: str
    duration_s: float
    endpoint_angle_deg: float
    integrated_angle_deg: float
    axis_excitation_deg: list[float]
    usable: bool


@dataclass
class LiveStatus:
    state: str
    target_index: int
    target_count: int
    target_code: str
    target_label: str
    target_direction: list[float]
    current_direction: list[float]
    angle_error_deg: float
    accel_norm_mps2: float
    accel_rms_deviation_mps2: float
    gyro_rms_rps: float
    gyro_bias_residual_rps: float
    hold_seconds: float
    hold_fraction: float
    transition_seconds: float
    hold_required_seconds: float
    transition_recommended_seconds: float
    transition_maximum_seconds: float
    pose_temperature_span_c: float
    temperature_budget_remaining_c: float
    temperature_budget_warning: bool
    qualified: bool
    reason: str
    completed_codes: list[str]


@dataclass
class _TransitionAccumulator:
    from_code: str
    start_timestamp_us: int
    last_timestamp_us: int
    last_corrected_gyro: np.ndarray
    axis_excitation_rad: np.ndarray = field(
        default_factory=lambda: np.zeros(3, dtype=np.float64)
    )
    integrated_angle_rad: float = 0.0

    def update(self, timestamp_us: int, corrected_gyro: np.ndarray) -> None:
        dt = (timestamp_us - self.last_timestamp_us) * 1e-6
        if 0.0 < dt < 0.1:
            average_abs = 0.5 * (
                np.abs(self.last_corrected_gyro) + np.abs(corrected_gyro)
            )
            self.axis_excitation_rad += average_abs * dt
            average_norm = 0.5 * (
                float(np.linalg.norm(self.last_corrected_gyro))
                + float(np.linalg.norm(corrected_gyro))
            )
            self.integrated_angle_rad += average_norm * dt
        self.last_timestamp_us = timestamp_us
        self.last_corrected_gyro = corrected_gyro.copy()


class GuidedLmSession:
    """Accept target poses only after a filtered, continuous static hold."""

    def __init__(
        self,
        criteria: GuidedCriteria | None = None,
        targets: tuple[TargetPose, ...] = TARGET_POSES,
    ) -> None:
        if not targets:
            raise ValueError("至少需要一个目标姿态")
        self.criteria = criteria or GuidedCriteria()
        self.targets = targets
        self.state = "running"
        self.target_index = 0
        self.pose_results: list[PoseResult] = []
        self.transition_results: list[TransitionResult] = []
        self._window: deque[tuple[int, np.ndarray, np.ndarray]] = deque()
        self._window_accel_sum = np.zeros(3, dtype=np.float64)
        self._window_gyro_sum = np.zeros(3, dtype=np.float64)
        self._window_accel_squared_norm_sum = 0.0
        self._window_gyro_squared_norm_sum = 0.0
        self._qualifying_since_us: int | None = None
        self._hold_accel_sum = np.zeros(3, dtype=np.float64)
        self._hold_gyro_sum = np.zeros(3, dtype=np.float64)
        self._hold_temperature_sum = 0.0
        self._hold_temperature_samples = 0
        self._hold_samples = 0
        self._hold_accumulated_seconds = 0.0
        self._last_qualified_timestamp_us: int | None = None
        self._hold_dropout_since_us: int | None = None
        self._transition: _TransitionAccumulator | None = None
        self._transition_candidate: tuple[int, np.ndarray, float] | None = None
        self._gyro_bias_residual_rps = math.nan
        self._current_temperature_c = math.nan
        self._last_status = self._empty_status("等待有效 IMU 数据")

    @property
    def live_status(self) -> LiveStatus:
        return self._last_status

    def suspend_for_temperature(self, reason: str) -> None:
        """Reset a partial pose hold when the thermal gate is no longer valid."""
        if self.state != "running":
            return
        if (
            not self._window
            and self._qualifying_since_us is None
            and self._last_status.reason == reason
        ):
            return
        self._clear_window()
        self._reset_hold(reason)

    def process(self, sample: ImuSample) -> PoseResult | None:
        if self.state != "running":
            return None
        accel = np.asarray(sample.accel_mps2, dtype=np.float64)
        if (sample.validity_flags & TEMPERATURE_VALID_MASK) == 0 or not math.isfinite(
            sample.temperature_c
        ):
            self._reset_hold("温度无效，无法进行陀螺仪温漂补偿")
            return None
        try:
            gyro = compensate_gyro_rps(sample.gyro_rps, sample.temperature_c)
        except ValueError as exc:
            self._reset_hold(str(exc))
            return None
        if not np.all(np.isfinite(accel)):
            self._reset_hold("数据包含 NaN 或无穷值")
            return None

        self._current_temperature_c = sample.temperature_c

        if self._transition is not None:
            bias = np.asarray(self.pose_results[-1].mean_gyro_rps, dtype=np.float64)
            self._transition.update(sample.timestamp_us, gyro - bias)

        self._window.append((sample.timestamp_us, accel, gyro))
        self._window_accel_sum += accel
        self._window_gyro_sum += gyro
        self._window_accel_squared_norm_sum += float(np.dot(accel, accel))
        self._window_gyro_squared_norm_sum += float(np.dot(gyro, gyro))
        cutoff = sample.timestamp_us - int(
            self.criteria.evaluation_window_seconds * 1e6
        )
        while self._window and self._window[0][0] < cutoff:
            _, old_accel, old_gyro = self._window.popleft()
            self._window_accel_sum -= old_accel
            self._window_gyro_sum -= old_gyro
            self._window_accel_squared_norm_sum -= float(
                np.dot(old_accel, old_accel)
            )
            self._window_gyro_squared_norm_sum -= float(np.dot(old_gyro, old_gyro))

        qualified, reason, current_direction, angle, accel_norm, accel_rms, gyro_rms = (
            self._evaluate_window(sample)
        )
        if qualified:
            if self._qualifying_since_us is None:
                self._qualifying_since_us = sample.timestamp_us
                self._hold_accel_sum.fill(0.0)
                self._hold_gyro_sum.fill(0.0)
                self._hold_samples = 0
                self._hold_accumulated_seconds = 0.0
                self._snapshot_transition_candidate(sample.timestamp_us)
            elif (
                self._hold_dropout_since_us is None
                and self._last_qualified_timestamp_us is not None
            ):
                qualifying_dt = (
                    sample.timestamp_us - self._last_qualified_timestamp_us
                ) * 1e-6
                if 0.0 < qualifying_dt < 0.1:
                    self._hold_accumulated_seconds += qualifying_dt
            self._hold_dropout_since_us = None
            self._last_qualified_timestamp_us = sample.timestamp_us
            self._hold_accel_sum += accel
            self._hold_gyro_sum += gyro
            if (sample.validity_flags & TEMPERATURE_VALID_MASK) != 0 and math.isfinite(
                sample.temperature_c
            ):
                self._hold_temperature_sum += sample.temperature_c
                self._hold_temperature_samples += 1
            self._hold_samples += 1
        else:
            if self._qualifying_since_us is None:
                self._reset_hold(reason)
            else:
                if self._hold_dropout_since_us is None:
                    self._hold_dropout_since_us = sample.timestamp_us
                dropout_seconds = (
                    sample.timestamp_us - self._hold_dropout_since_us
                ) * 1e-6
                if dropout_seconds > self.criteria.hold_dropout_grace_seconds:
                    self._reset_hold(reason)
                else:
                    reason = f"{reason}；短暂抖动，保持进度已暂停"

        hold_seconds = self._hold_accumulated_seconds
        self._last_status = self._status(
            current_direction,
            angle,
            accel_norm,
            accel_rms,
            gyro_rms,
            hold_seconds,
            sample.timestamp_us,
            qualified,
            reason,
        )
        if qualified and hold_seconds >= self.criteria.hold_seconds:
            return self._accept_pose(sample.timestamp_us)
        return None

    def _evaluate_window(
        self, sample: ImuSample
    ) -> tuple[bool, str, np.ndarray, float, float, float, float]:
        target = np.asarray(self.targets[self.target_index].direction, dtype=np.float64)
        if (sample.validity_flags & 0x3) != 0x3:
            return (
                False,
                "加速度计或陀螺仪数据无效",
                np.zeros(3),
                math.inf,
                math.nan,
                math.inf,
                math.inf,
            )
        if len(self._window) < 2:
            return (
                False,
                "正在建立 0.2 秒判定窗口",
                np.zeros(3),
                math.inf,
                math.nan,
                math.inf,
                math.inf,
            )
        duration = (self._window[-1][0] - self._window[0][0]) * 1e-6
        if duration < 0.85 * self.criteria.evaluation_window_seconds:
            return (
                False,
                "正在建立 0.2 秒判定窗口",
                np.zeros(3),
                math.inf,
                math.nan,
                math.inf,
                math.inf,
            )

        window_samples = len(self._window)
        mean_accel = self._window_accel_sum / window_samples
        accel_norm = float(np.linalg.norm(mean_accel))
        accel_variance = max(
            0.0,
            self._window_accel_squared_norm_sum / window_samples
            - float(np.dot(mean_accel, mean_accel)),
        )
        accel_rms = math.sqrt(accel_variance)
        gyro_rms = math.sqrt(
            max(0.0, self._window_gyro_squared_norm_sum / window_samples)
        )
        mean_gyro = self._window_gyro_sum / window_samples
        bias_reference = self._gyro_bias_reference()
        self._gyro_bias_residual_rps = (
            math.nan
            if bias_reference is None
            else float(np.max(np.abs(mean_gyro - bias_reference)))
        )
        if accel_norm <= 1e-6:
            return (
                False,
                "加速度模长无效",
                np.zeros(3),
                math.inf,
                accel_norm,
                accel_rms,
                gyro_rms,
            )
        current_direction = mean_accel / accel_norm
        angle = math.degrees(
            math.acos(float(np.clip(np.dot(current_direction, target), -1.0, 1.0)))
        )

        temperature_span_c, _, _ = self._temperature_budget()
        if temperature_span_c > self.criteria.maximum_pose_temperature_span_c:
            return (
                False,
                "整组温差预算已耗尽，请在当前温度重新开始本组采集",
                current_direction,
                angle,
                accel_norm,
                accel_rms,
                gyro_rms,
            )

        if (
            not self.criteria.accel_norm_min_mps2
            <= accel_norm
            <= self.criteria.accel_norm_max_mps2
        ):
            return (
                False,
                "加速度模长偏离 1 g，请保持静止",
                current_direction,
                angle,
                accel_norm,
                accel_rms,
                gyro_rms,
            )
        if accel_rms >= self.criteria.accel_rms_deviation_threshold_mps2:
            return (
                False,
                "加速度仍在抖动，请放稳",
                current_direction,
                angle,
                accel_norm,
                accel_rms,
                gyro_rms,
            )
        if gyro_rms >= self.criteria.gyro_rms_threshold_rps:
            return (
                False,
                "仍在转动，请停稳",
                current_direction,
                angle,
                accel_norm,
                accel_rms,
                gyro_rms,
            )
        if angle > self.criteria.angle_tolerance_deg:
            return (
                False,
                "方向未对准，让当前四旋翼接近目标姿态",
                current_direction,
                angle,
                accel_norm,
                accel_rms,
                gyro_rms,
            )
        return (
            True,
            "方向和静止条件合格，请继续保持",
            current_direction,
            angle,
            accel_norm,
            accel_rms,
            gyro_rms,
        )

    def _snapshot_transition_candidate(self, timestamp_us: int) -> None:
        if self._transition is None:
            self._transition_candidate = None
            return
        self._transition_candidate = (
            timestamp_us,
            self._transition.axis_excitation_rad.copy(),
            self._transition.integrated_angle_rad,
        )

    def _gyro_bias_reference(self) -> np.ndarray | None:
        if not self.pose_results:
            return None
        return np.median(
            np.asarray(
                [item.mean_gyro_rps for item in self.pose_results], dtype=np.float64
            ),
            axis=0,
        )

    def _temperature_budget(self) -> tuple[float, float, bool]:
        temperatures = [
            item.mean_temperature_c
            for item in self.pose_results
            if math.isfinite(item.mean_temperature_c)
        ]
        if math.isfinite(self._current_temperature_c):
            temperatures.append(self._current_temperature_c)
        span_c = float(np.ptp(temperatures)) if len(temperatures) >= 2 else 0.0
        remaining_c = max(
            0.0, self.criteria.maximum_pose_temperature_span_c - span_c
        )
        warning = span_c >= self.criteria.pose_temperature_warning_span_c
        return span_c, remaining_c, warning

    def _clear_window(self) -> None:
        self._window.clear()
        self._window_accel_sum.fill(0.0)
        self._window_gyro_sum.fill(0.0)
        self._window_accel_squared_norm_sum = 0.0
        self._window_gyro_squared_norm_sum = 0.0

    def _reset_hold(self, reason: str) -> None:
        self._qualifying_since_us = None
        self._hold_accel_sum.fill(0.0)
        self._hold_gyro_sum.fill(0.0)
        self._hold_temperature_sum = 0.0
        self._hold_temperature_samples = 0
        self._hold_samples = 0
        self._hold_accumulated_seconds = 0.0
        self._last_qualified_timestamp_us = None
        self._hold_dropout_since_us = None
        self._transition_candidate = None
        if self.state == "running" and self.target_index < len(self.targets):
            self._last_status = self._empty_status(reason)

    def _accept_pose(self, timestamp_us: int) -> PoseResult:
        target = self.targets[self.target_index]
        mean_accel = self._hold_accel_sum / max(self._hold_samples, 1)
        mean_gyro = self._hold_gyro_sum / max(self._hold_samples, 1)
        mean_temperature_c = (
            self._hold_temperature_sum / self._hold_temperature_samples
            if self._hold_temperature_samples > 0
            else math.nan
        )
        direction = mean_accel / np.linalg.norm(mean_accel)
        angle = math.degrees(
            math.acos(
                float(
                    np.clip(np.dot(direction, np.asarray(target.direction)), -1.0, 1.0)
                )
            )
        )
        result = PoseResult(
            code=target.code,
            label=target.label,
            target_direction=list(target.direction),
            mean_accel_mps2=mean_accel.tolist(),
            mean_gyro_rps=mean_gyro.tolist(),
            mean_temperature_c=mean_temperature_c,
            accepted_timestamp_us=timestamp_us,
            samples=self._hold_samples,
            angle_error_deg=angle,
        )
        self.pose_results.append(result)
        self._finish_transition(target, direction)
        self.target_index += 1
        self._clear_window()
        self._reset_hold("姿态已记录，请在 30 秒内转向下一个目标")

        if self.target_index >= len(self.targets):
            self.state = "complete"
            self._transition = None
            self._last_status = self._completed_status()
        else:
            self._transition = _TransitionAccumulator(
                from_code=target.code,
                start_timestamp_us=timestamp_us,
                last_timestamp_us=timestamp_us,
                last_corrected_gyro=np.zeros(3, dtype=np.float64),
            )
            self._last_status = self._empty_status(
                "姿态已记录，请在 30 秒内转向下一个目标"
            )
        return result

    def _finish_transition(
        self, target: TargetPose, current_direction: np.ndarray
    ) -> None:
        if self._transition is None or self._transition_candidate is None:
            return
        entered_timestamp_us, axis_excitation, integrated_angle = (
            self._transition_candidate
        )
        previous_direction = np.asarray(
            self.pose_results[-2].mean_accel_mps2, dtype=np.float64
        )
        previous_direction /= np.linalg.norm(previous_direction)
        endpoint_angle = math.degrees(
            math.acos(
                float(np.clip(np.dot(previous_direction, current_direction), -1.0, 1.0))
            )
        )
        duration = (entered_timestamp_us - self._transition.start_timestamp_us) * 1e-6
        integrated_deg = math.degrees(integrated_angle)
        axis_deg = np.degrees(axis_excitation)
        usable = (
            duration <= self.criteria.maximum_transition_seconds
            and endpoint_angle >= 15.0
            and integrated_deg >= 10.0
        )
        self.transition_results.append(
            TransitionResult(
                from_code=self._transition.from_code,
                to_code=target.code,
                duration_s=duration,
                endpoint_angle_deg=endpoint_angle,
                integrated_angle_deg=integrated_deg,
                axis_excitation_deg=axis_deg.tolist(),
                usable=usable,
            )
        )

    def _status(
        self,
        direction: np.ndarray,
        angle: float,
        accel_norm: float,
        accel_rms: float,
        gyro_rms: float,
        hold_seconds: float,
        timestamp_us: int,
        qualified: bool,
        reason: str,
    ) -> LiveStatus:
        target = self.targets[self.target_index]
        temperature_span_c, temperature_remaining_c, temperature_warning = (
            self._temperature_budget()
        )
        transition_seconds = 0.0
        if self._transition is not None:
            transition_end_us = (
                self._transition_candidate[0]
                if self._transition_candidate is not None
                else timestamp_us
            )
            transition_seconds = max(
                0.0,
                (transition_end_us - self._transition.start_timestamp_us) * 1e-6,
            )
        return LiveStatus(
            state=self.state,
            target_index=self.target_index,
            target_count=len(self.targets),
            target_code=target.code,
            target_label=target.label,
            target_direction=list(target.direction),
            current_direction=direction.tolist(),
            angle_error_deg=angle,
            accel_norm_mps2=accel_norm,
            accel_rms_deviation_mps2=accel_rms,
            gyro_rms_rps=gyro_rms,
            gyro_bias_residual_rps=self._gyro_bias_residual_rps,
            hold_seconds=hold_seconds,
            hold_fraction=min(1.0, hold_seconds / self.criteria.hold_seconds),
            transition_seconds=transition_seconds,
            hold_required_seconds=self.criteria.hold_seconds,
            transition_recommended_seconds=self.criteria.recommended_transition_seconds,
            transition_maximum_seconds=self.criteria.maximum_transition_seconds,
            pose_temperature_span_c=temperature_span_c,
            temperature_budget_remaining_c=temperature_remaining_c,
            temperature_budget_warning=temperature_warning,
            qualified=qualified,
            reason=reason,
            completed_codes=[item.code for item in self.pose_results],
        )

    def _empty_status(self, reason: str) -> LiveStatus:
        index = min(self.target_index, len(self.targets) - 1)
        target = self.targets[index]
        temperature_span_c, temperature_remaining_c, temperature_warning = (
            self._temperature_budget()
        )
        return LiveStatus(
            state=self.state,
            target_index=self.target_index,
            target_count=len(self.targets),
            target_code=target.code,
            target_label=target.label,
            target_direction=list(target.direction),
            current_direction=[0.0, 0.0, 0.0],
            angle_error_deg=math.inf,
            accel_norm_mps2=math.nan,
            accel_rms_deviation_mps2=math.inf,
            gyro_rms_rps=math.inf,
            gyro_bias_residual_rps=math.nan,
            hold_seconds=0.0,
            hold_fraction=0.0,
            transition_seconds=0.0,
            hold_required_seconds=self.criteria.hold_seconds,
            transition_recommended_seconds=self.criteria.recommended_transition_seconds,
            transition_maximum_seconds=self.criteria.maximum_transition_seconds,
            pose_temperature_span_c=temperature_span_c,
            temperature_budget_remaining_c=temperature_remaining_c,
            temperature_budget_warning=temperature_warning,
            qualified=False,
            reason=reason,
            completed_codes=[item.code for item in self.pose_results],
        )

    def _completed_status(self) -> LiveStatus:
        target = self.targets[-1]
        temperature_span_c, temperature_remaining_c, temperature_warning = (
            self._temperature_budget()
        )
        return LiveStatus(
            state="complete",
            target_index=len(self.targets),
            target_count=len(self.targets),
            target_code=target.code,
            target_label="18 个目标姿态已完成",
            target_direction=list(target.direction),
            current_direction=list(target.direction),
            angle_error_deg=0.0,
            accel_norm_mps2=float(
                np.linalg.norm(self.pose_results[-1].mean_accel_mps2)
            ),
            accel_rms_deviation_mps2=0.0,
            gyro_rms_rps=float(np.linalg.norm(self.pose_results[-1].mean_gyro_rps)),
            gyro_bias_residual_rps=math.nan,
            hold_seconds=self.criteria.hold_seconds,
            hold_fraction=1.0,
            transition_seconds=0.0,
            hold_required_seconds=self.criteria.hold_seconds,
            transition_recommended_seconds=self.criteria.recommended_transition_seconds,
            transition_maximum_seconds=self.criteria.maximum_transition_seconds,
            pose_temperature_span_c=temperature_span_c,
            temperature_budget_remaining_c=temperature_remaining_c,
            temperature_budget_warning=temperature_warning,
            qualified=True,
            reason="正在进行整组数据质量判定",
            completed_codes=[item.code for item in self.pose_results],
        )

    def build_report(self, parser_stats: ParserStats) -> dict:
        if not self.pose_results:
            raise ValueError("尚未记录任何姿态")
        directions = np.asarray(
            [item.mean_accel_mps2 for item in self.pose_results], dtype=np.float64
        )
        directions /= np.linalg.norm(directions, axis=1, keepdims=True)
        coverage = float(
            np.linalg.eigvalsh(directions.T @ directions / len(directions))[0]
        )
        axial_codes = {"-X", "+X", "-Y", "+Y", "-Z", "+Z"}
        axial_pose_results = [
            item for item in self.pose_results if item.code in axial_codes
        ]
        bias_check_results = (
            axial_pose_results if len(axial_pose_results) == 6 else self.pose_results
        )
        gyro_means = np.asarray(
            [item.mean_gyro_rps for item in bias_check_results], dtype=np.float64
        )
        drift_axis_dps = np.degrees(
            np.max(gyro_means, axis=0) - np.min(gyro_means, axis=0)
        )
        max_drift_dps = float(np.max(drift_axis_dps))
        pose_temperatures = np.asarray(
            [item.mean_temperature_c for item in self.pose_results], dtype=np.float64
        )
        temperature_complete = bool(np.all(np.isfinite(pose_temperatures)))
        temperature_span_c = (
            float(np.ptp(pose_temperatures)) if temperature_complete else math.inf
        )
        usable_transition_results = [
            item for item in self.transition_results if item.usable
        ]
        usable_transitions = len(usable_transition_results)
        if usable_transition_results:
            axis_excitation = np.sum(
                np.asarray(
                    [item.axis_excitation_deg for item in usable_transition_results],
                    dtype=np.float64,
                ),
                axis=0,
            )
        else:
            axis_excitation = np.zeros(3, dtype=np.float64)
        link_quality = {
            "crc_errors": parser_stats.crc_errors,
            "resync_bytes": parser_stats.resync_bytes,
            "frame_sequence_gaps": parser_stats.sequence_gaps,
            "frame_duplicate_or_backward": parser_stats.duplicate_or_backward_frames,
            "sample_sequence_gaps": parser_stats.sample_sequence_gaps,
            "sample_duplicate_or_backward": parser_stats.duplicate_or_backward_samples,
            "timestamp_nonmonotonic": parser_stats.timestamp_nonmonotonic,
            "timestamp_interval_outliers": parser_stats.timestamp_interval_outliers,
        }
        link_ok = all(value == 0 for value in link_quality.values())
        checks = [
            self._check(
                "pose_count",
                len(self.pose_results) == len(self.targets),
                len(self.pose_results),
                f"= {len(self.targets)}",
            ),
            self._check(
                "coverage_min_eigenvalue",
                coverage >= self.criteria.coverage_min_eigenvalue,
                coverage,
                f">= {self.criteria.coverage_min_eigenvalue:.2f}",
            ),
            self._check(
                "gyro_bias_drift_dps",
                max_drift_dps <= self.criteria.max_gyro_bias_drift_dps,
                max_drift_dps,
                f"<= {self.criteria.max_gyro_bias_drift_dps:.3f} deg/s",
            ),
            self._check(
                "temperature_valid",
                temperature_complete,
                int(np.count_nonzero(np.isfinite(pose_temperatures))),
                f"= {len(self.pose_results)} poses",
            ),
            self._check(
                "pose_temperature_span_c",
                temperature_span_c <= self.criteria.maximum_pose_temperature_span_c,
                temperature_span_c,
                f"<= {self.criteria.maximum_pose_temperature_span_c:.1f} °C",
            ),
            self._check(
                "usable_rotation_transitions",
                usable_transitions >= self.criteria.minimum_usable_transitions,
                usable_transitions,
                f">= {self.criteria.minimum_usable_transitions}",
            ),
            self._check(
                "rotation_excitation_each_axis_deg",
                bool(
                    np.all(axis_excitation >= self.criteria.minimum_axis_excitation_deg)
                ),
                axis_excitation.tolist(),
                f"each >= {self.criteria.minimum_axis_excitation_deg:.0f} deg",
            ),
            self._check("usb_link_errors", link_ok, link_quality, "all fields = 0"),
        ]
        return {
            "format_version": 1,
            "overall_pass": all(item["pass"] for item in checks),
            "gyro_temperature_compensation": temperature_compensation_metadata(),
            "criteria": asdict(self.criteria),
            "checks": checks,
            "metrics": {
                "coverage_min_eigenvalue": coverage,
                "gyro_bias_drift_axis_dps": drift_axis_dps.tolist(),
                "gyro_bias_drift_max_dps": max_drift_dps,
                "gyro_bias_drift_pose_codes": [
                    item.code for item in bias_check_results
                ],
                "pose_temperature_c": pose_temperatures.tolist(),
                "pose_temperature_span_c": temperature_span_c,
                "usable_rotation_transitions": usable_transitions,
                "rotation_excitation_axis_deg": axis_excitation.tolist(),
                "parser": asdict(parser_stats),
            },
            "poses": [asdict(item) for item in self.pose_results],
            "transitions": [asdict(item) for item in self.transition_results],
        }

    @staticmethod
    def _check(name: str, passed: bool, value: object, requirement: str) -> dict:
        return {
            "name": name,
            "pass": bool(passed),
            "value": value,
            "requirement": requirement,
        }
