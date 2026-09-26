/**
 * @file Adaptive_Notch.c
 * @brief 使用 Hann 窗和 Q15 实数 FFT 增量跟踪三轴陀螺主振动频率
 */
#include "Adaptive_Notch.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define ADAPTIVE_NOTCH_TWO_PI_F  6.28318530717958647692f
#define ADAPTIVE_NOTCH_Q15_SCALE (32767.0f / ADAPTIVE_NOTCH_Q15_FULL_SCALE_RPS)

/** @brief 将数值限制在闭区间内。 */
static float Adaptive_Notch_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

/** @brief 校验频带、Nyquist 上限、门限和分析周期是否自洽。 */
static bool Adaptive_Notch_ConfigValid(const AdaptiveNotchConfig_t *config)
{
    return (config != NULL) && isfinite(config->sample_rate_hz) && isfinite(config->initial_center_hz) &&
           isfinite(config->minimum_hz) && isfinite(config->maximum_hz) && isfinite(config->minimum_rms) &&
           isfinite(config->minimum_snr) && isfinite(config->maximum_slew_hz_per_s) &&
           (config->sample_rate_hz > 0.0f) && (config->minimum_hz > 0.0f) &&
           (config->maximum_hz < 0.5f * config->sample_rate_hz) && (config->minimum_hz < config->maximum_hz) &&
           (config->initial_center_hz >= config->minimum_hz) && (config->initial_center_hz <= config->maximum_hz) &&
           (config->minimum_rms > 0.0f) && (config->minimum_snr > 1.0f) && (config->maximum_slew_hz_per_s > 0.0f) &&
           (config->update_samples > 0U) && (config->update_samples <= ADAPTIVE_NOTCH_FFT_SIZE);
}

/** @brief 把 32 位中间值饱和到 Q15 范围。 */
static q15_t Adaptive_Notch_SaturateQ15(int32_t value)
{
    if (value > INT16_MAX)
    {
        return INT16_MAX;
    }
    if (value < INT16_MIN)
    {
        return INT16_MIN;
    }
    return (q15_t)value;
}

/** @brief 将 rad/s 陀螺数据量化为 Q15，对超出传感器物理量程的值饱和。 */
static q15_t Adaptive_Notch_GyroToQ15(float gyro_rps)
{
    const float scaled = gyro_rps * ADAPTIVE_NOTCH_Q15_SCALE;
    if (scaled >= (float)INT16_MAX)
    {
        return INT16_MAX;
    }
    if (scaled <= (float)INT16_MIN)
    {
        return INT16_MIN;
    }
    return (q15_t)scaled;
}

/** @brief 初始化 Q15 FFT、Hann 窗和三轴中心频率。 */
bool Adaptive_Notch_Init(AdaptiveNotchTracker_t *tracker, const AdaptiveNotchConfig_t *config)
{
    if ((tracker == NULL) || (!Adaptive_Notch_ConfigValid(config)))
    {
        return false;
    }

    memset(tracker, 0, sizeof(*tracker));
    tracker->config = *config;
    for (uint32_t axis = 0U; axis < ADAPTIVE_NOTCH_AXIS_COUNT; axis++)
    {
        tracker->center_hz[axis] = config->initial_center_hz;
    }
    if (arm_rfft_init_q15(&tracker->fft, ADAPTIVE_NOTCH_FFT_SIZE, 0U, 1U) != ARM_MATH_SUCCESS)
    {
        return false;
    }

    for (uint32_t index = 0U; index < ADAPTIVE_NOTCH_FFT_SIZE; index++)
    {
        const float phase        = ADAPTIVE_NOTCH_TWO_PI_F * (float)index / (float)(ADAPTIVE_NOTCH_FFT_SIZE - 1U);
        const float window_value = 0.5f - 0.5f * cosf(phase);
        tracker->window[index]   = (q15_t)(window_value * 32767.0f + 0.5f);
    }
    tracker->initialized = 1U;
    return true;
}

void Adaptive_Notch_ResetSamples(AdaptiveNotchTracker_t *tracker)
{
    if ((tracker == NULL) || (tracker->initialized == 0U))
    {
        return;
    }

    memset(tracker->samples, 0, sizeof(tracker->samples));
    memset(tracker->tracking_valid, 0, sizeof(tracker->tracking_valid));
    tracker->write_index           = 0U;
    tracker->sample_count          = 0U;
    tracker->samples_since_update  = 0U;
    tracker->analysis_timestamp_us = 0ULL;
    tracker->analysis_pending      = 0U;
    tracker->next_analysis_axis    = 0U;
    tracker->analysis_changed_mask = 0U;
}

/**
 * @brief 按时间顺序展开一个轴的 Q15 环形样本，去直流并乘 Hann 窗。
 * @return 换算回 rad/s 的去均值 RMS，保持 minimum_rms 参数的物理单位不变
 */
static float Adaptive_Notch_PrepareAxis(AdaptiveNotchTracker_t *tracker, uint32_t axis)
{
    int64_t sum = 0;
    for (uint32_t index = 0U; index < ADAPTIVE_NOTCH_FFT_SIZE; index++)
    {
        const uint32_t sample_index = (tracker->write_index + index) & (ADAPTIVE_NOTCH_FFT_SIZE - 1U);
        sum += tracker->samples[axis][sample_index];
    }
    const int32_t mean = (int32_t)(sum / (int64_t)ADAPTIVE_NOTCH_FFT_SIZE);

    int64_t sum_squares = 0;
    for (uint32_t index = 0U; index < ADAPTIVE_NOTCH_FFT_SIZE; index++)
    {
        const uint32_t sample_index = (tracker->write_index + index) & (ADAPTIVE_NOTCH_FFT_SIZE - 1U);
        const int32_t  centered     = (int32_t)tracker->samples[axis][sample_index] - mean;
        const q15_t    centered_q15 = Adaptive_Notch_SaturateQ15(centered);
        const int32_t  windowed     = ((int32_t)centered_q15 * (int32_t)tracker->window[index]) >> 15;
        sum_squares += (int64_t)centered * (int64_t)centered;
        tracker->fft_input[index] = Adaptive_Notch_SaturateQ15(windowed);
    }
    const float rms_q15 = sqrtf((float)sum_squares / (float)ADAPTIVE_NOTCH_FFT_SIZE);
    return rms_q15 * (ADAPTIVE_NOTCH_Q15_FULL_SCALE_RPS / 32767.0f);
}

/**
 * @brief 执行实数 FFT，并计算搜索频带内各频点的功率
 *
 * CMSIS Q15 RFFT 按“实部、虚部”交错存放：2*bin 为实部，2*bin+1 为虚部。
 * 后续只比较相对功率和 SNR，因此无需把 FFT 幅值还原成真实物理量。
 */
static void Adaptive_Notch_AccumulatePower(AdaptiveNotchTracker_t *tracker, uint32_t minimum_bin, uint32_t maximum_bin)
{
    arm_rfft_q15(&tracker->fft, tracker->fft_input, tracker->fft_output);
    for (uint32_t bin = minimum_bin; bin <= maximum_bin; bin++)
    {
        const float real             = (float)tracker->fft_output[2U * bin];
        const float imag             = (float)tracker->fft_output[2U * bin + 1U];
        tracker->spectrum_power[bin] = real * real + imag * imag;
    }
}

/** @brief 在限定频带内查找功率最大的离散频点。 */
static uint32_t Adaptive_Notch_FindPeak(const AdaptiveNotchTracker_t *tracker, uint32_t minimum_bin,
                                        uint32_t maximum_bin)
{
    uint32_t peak_bin   = minimum_bin;
    float    peak_power = tracker->spectrum_power[minimum_bin];
    for (uint32_t bin = minimum_bin + 1U; bin <= maximum_bin; bin++)
    {
        if (tracker->spectrum_power[bin] > peak_power)
        {
            peak_power = tracker->spectrum_power[bin];
            peak_bin   = bin;
        }
    }
    return peak_bin;
}

/**
 * @brief 计算谱峰与背景噪声平均功率之比。
 * @note 峰值左右各两个 bin 不参与噪声估计，避免主瓣泄漏抬高背景值。
 */
static float Adaptive_Notch_CalculateSnr(const AdaptiveNotchTracker_t *tracker, uint32_t peak_bin, uint32_t minimum_bin,
                                         uint32_t maximum_bin)
{
    float    noise_power = 0.0f;
    uint32_t noise_bins  = 0U;
    for (uint32_t bin = minimum_bin; bin <= maximum_bin; bin++)
    {
        const uint32_t distance = (bin > peak_bin) ? (bin - peak_bin) : (peak_bin - bin);
        if (distance > 2U)
        {
            noise_power += tracker->spectrum_power[bin];
            noise_bins++;
        }
    }
    if (noise_bins == 0U)
    {
        return 0.0f;
    }
    noise_power /= (float)noise_bins;
    return tracker->spectrum_power[peak_bin] / (noise_power + 1.0e-12f);
}

/** @brief 使用峰值及左右相邻点做抛物线插值，提高频率估计分辨率。 */
static float Adaptive_Notch_InterpolateBin(const AdaptiveNotchTracker_t *tracker, uint32_t peak_bin,
                                           uint32_t minimum_bin, uint32_t maximum_bin)
{
    if ((peak_bin <= minimum_bin) || (peak_bin >= maximum_bin))
    {
        return (float)peak_bin;
    }

    const float left        = tracker->spectrum_power[peak_bin - 1U];
    const float center      = tracker->spectrum_power[peak_bin];
    const float right       = tracker->spectrum_power[peak_bin + 1U];
    const float denominator = left - 2.0f * center + right;
    if (fabsf(denominator) <= 1.0e-12f)
    {
        return (float)peak_bin;
    }
    const float offset = Adaptive_Notch_Clamp(0.5f * (left - right) / denominator, -0.5f, 0.5f);
    return (float)peak_bin + offset;
}

/**
 * @brief 按最大 Hz/s 变化率逼近新的中心频率。
 * @return true 本次变化达到需要重算 IIR 系数的最小幅度
 */
static bool Adaptive_Notch_UpdateCenter(AdaptiveNotchTracker_t *tracker, uint32_t axis, float target_hz)
{
    const float update_period_s = (float)tracker->config.update_samples / tracker->config.sample_rate_hz;
    const float maximum_step_hz = tracker->config.maximum_slew_hz_per_s * update_period_s;
    const float frequency_error = target_hz - tracker->center_hz[axis];
    const float center_step_hz  = Adaptive_Notch_Clamp(frequency_error, -maximum_step_hz, maximum_step_hz);
    tracker->center_hz[axis] += center_step_hz;
    tracker->tracking_valid[axis] = 1U;
    tracker->update_count[axis]++;
    return fabsf(center_step_hz) >= 0.01f;
}

/**
 * @brief 分析一个轴的完整窗口，并更新通过 RMS 与 SNR 门限的该轴。
 */
static bool Adaptive_Notch_AnalyseAxis(AdaptiveNotchTracker_t *tracker, uint32_t axis)
{
    /* 将配置的 Hz 搜索范围换算成 FFT 频点下标，并限制分析范围。 */
    const uint32_t minimum_bin = (uint32_t)ceilf(tracker->config.minimum_hz * (float)ADAPTIVE_NOTCH_FFT_SIZE /
                                                 tracker->config.sample_rate_hz);
    const uint32_t maximum_bin = (uint32_t)floorf(tracker->config.maximum_hz * (float)ADAPTIVE_NOTCH_FFT_SIZE /
                                                  tracker->config.sample_rate_hz);
    const float    rms_rps     = Adaptive_Notch_PrepareAxis(tracker, axis);
    memset(tracker->spectrum_power, 0, sizeof(tracker->spectrum_power));
    Adaptive_Notch_AccumulatePower(tracker, minimum_bin, maximum_bin);

    /* 先找最强离散频点，再以频带背景平均功率估算谱峰可信度。 */
    const uint32_t peak_bin = Adaptive_Notch_FindPeak(tracker, minimum_bin, maximum_bin);
    tracker->peak_snr[axis] = Adaptive_Notch_CalculateSnr(tracker, peak_bin, minimum_bin, maximum_bin);
    if ((rms_rps < tracker->config.minimum_rms) || (tracker->peak_snr[axis] < tracker->config.minimum_snr))
    {
        tracker->tracking_valid[axis] = 0U;
        return false;
    }

    /* 通过门限后才插值并限速更新，避免静止噪声驱动陷波中心漂移。 */
    const float peak      = Adaptive_Notch_InterpolateBin(tracker, peak_bin, minimum_bin, maximum_bin);
    const float target_hz = Adaptive_Notch_Clamp(peak * tracker->config.sample_rate_hz / (float)ADAPTIVE_NOTCH_FFT_SIZE,
                                                 tracker->config.minimum_hz, tracker->config.maximum_hz);
    return Adaptive_Notch_UpdateCenter(tracker, axis, target_hz);
}

/**
 * @brief 判断 FFT 窗口是否填满且达到本轮更新间隔
 *
 * 上电后先收满 512 点；之后保留旧窗口，每新增 update_samples 点触发一次分析，
 * 因此形成带重叠的滑动 FFT 窗，而不是每轮重新等待完整的 512 个样本。
 */
static bool Adaptive_Notch_WindowReady(AdaptiveNotchTracker_t *tracker)
{
    if (tracker->sample_count < ADAPTIVE_NOTCH_FFT_SIZE)
    {
        tracker->sample_count++;
        if (tracker->sample_count < ADAPTIVE_NOTCH_FFT_SIZE)
        {
            return false;
        }
        tracker->samples_since_update = tracker->config.update_samples;
    }
    else
    {
        tracker->samples_since_update++;
    }

    if (tracker->samples_since_update < tracker->config.update_samples)
    {
        return false;
    }
    tracker->samples_since_update = 0U;
    return true;
}

/** @brief 将一组三轴样本写入环形窗口，并在需要时触发频谱分析。 */
bool Adaptive_Notch_PushSample(AdaptiveNotchTracker_t *tracker, uint64_t timestamp_us, const float gyro_rps[3])
{
    if ((tracker == NULL) || (gyro_rps == NULL) || (tracker->initialized == 0U) || (!isfinite(gyro_rps[0])) ||
        (!isfinite(gyro_rps[1])) || (!isfinite(gyro_rps[2])) || (timestamp_us == 0ULL) ||
        (tracker->analysis_pending != 0U))
    {
        return false;
    }

    for (uint32_t axis = 0U; axis < ADAPTIVE_NOTCH_AXIS_COUNT; axis++)
    {
        tracker->samples[axis][tracker->write_index] = Adaptive_Notch_GyroToQ15(gyro_rps[axis]);
    }
    tracker->write_index = (tracker->write_index + 1U) & (ADAPTIVE_NOTCH_FFT_SIZE - 1U);
    if (Adaptive_Notch_WindowReady(tracker))
    {
        /* 锁定当前分析窗：三轴算完前不再向 tracker 写新样本。
         * 新样本由上层 SPSC 环形缓冲区暂存，保证三轴使用同一时间窗。 */
        tracker->analysis_timestamp_us = timestamp_us;
        tracker->analysis_pending      = 1U;
        tracker->next_analysis_axis    = 0U;
        tracker->analysis_changed_mask = 0U;
    }
    return true;
}

bool Adaptive_Notch_IsAnalysisPending(const AdaptiveNotchTracker_t *tracker)
{
    return (tracker != NULL) && (tracker->initialized != 0U) && (tracker->analysis_pending != 0U);
}

bool Adaptive_Notch_ProcessNextAxis(AdaptiveNotchTracker_t *tracker, AdaptiveNotchResult_t *completed_result)
{
    if ((tracker == NULL) || (completed_result == NULL) || (tracker->initialized == 0U) ||
        (tracker->analysis_pending == 0U) || (tracker->next_analysis_axis >= ADAPTIVE_NOTCH_AXIS_COUNT))
    {
        return false;
    }

    /* 每次调用只处理一个轴，调用方可将三轴计算分散到三个任务调度周期。 */
    const uint32_t axis = tracker->next_analysis_axis;
    if (Adaptive_Notch_AnalyseAxis(tracker, axis))
    {
        tracker->analysis_changed_mask |= (uint8_t)(1U << axis);
    }
    tracker->next_analysis_axis++;

    if (tracker->next_analysis_axis < ADAPTIVE_NOTCH_AXIS_COUNT)
    {
        /* 仍有轴未完成，不输出不完整结果。 */
        return false;
    }

    /* 三轴全部完成后一次性生成不可变快照，供姿态任务更新滤波器系数。 */
    memset(completed_result, 0, sizeof(*completed_result));
    completed_result->timestamp_us        = tracker->analysis_timestamp_us;
    completed_result->center_changed_mask = tracker->analysis_changed_mask;
    for (uint32_t result_axis = 0U; result_axis < ADAPTIVE_NOTCH_AXIS_COUNT; result_axis++)
    {
        completed_result->center_hz[result_axis] = tracker->center_hz[result_axis];
        completed_result->peak_snr[result_axis]  = tracker->peak_snr[result_axis];
        if (tracker->tracking_valid[result_axis] != 0U)
        {
            completed_result->tracking_valid_mask |= (uint8_t)(1U << result_axis);
        }
    }

    /* 释放当前窗口；下一帧样本到来后继续推进重叠滑动窗。 */
    tracker->analysis_pending      = 0U;
    tracker->next_analysis_axis    = 0U;
    tracker->analysis_changed_mask = 0U;
    return true;
}
