/**
 * @file Biquad_Notch.c
 * @brief 二阶陷波器的系数计算与逐样本状态更新
 */
#include "Biquad_Notch.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define BIQUAD_NOTCH_TWO_PI_F 6.28318530717958647692f

/**
 * @brief 根据采样率、中心频率和 Q 值计算归一化陷波系数。
 * @note 参数非法时保留原系数，避免运行中一次异常配置破坏现有滤波状态。
 */
static void Biquad_Notch_SetCoefficients(BiquadNotchFilter_t *filter, float sample_rate_hz, float center_hz, float q)
{
    if ((!isfinite(sample_rate_hz)) || (!isfinite(center_hz)) || (!isfinite(q)) || (sample_rate_hz <= 0.0f) ||
        (center_hz <= 0.0f) || (center_hz >= (0.5f * sample_rate_hz)) || (q <= 0.0f))
    {
        return;
    }

    const float omega   = BIQUAD_NOTCH_TWO_PI_F * center_hz / sample_rate_hz;
    const float alpha   = sinf(omega) / (2.0f * q);
    const float inv_a0  = 1.0f / (1.0f + alpha);
    const float two_cos = -2.0f * cosf(omega);

    filter->b0 = inv_a0;
    filter->b1 = two_cos * inv_a0;
    filter->b2 = inv_a0;
    filter->a1 = filter->b1;
    filter->a2 = (1.0f - alpha) * inv_a0;
}

/** @brief 初始化系数并清空 Direct Form II 延迟状态。 */
void Biquad_Notch_Init(BiquadNotchFilter_t *filter, float sample_rate_hz, float center_hz, float q)
{
    if (filter == NULL)
    {
        return;
    }

    memset(filter, 0, sizeof(*filter));
    filter->b0 = 1.0f;
    Biquad_Notch_SetCoefficients(filter, sample_rate_hz, center_hz, q);
}

/** @brief 在线刷新系数；延迟状态保持不变以减小切频瞬态。 */
void Biquad_Notch_Update(BiquadNotchFilter_t *filter, float sample_rate_hz, float center_hz, float q)
{
    if (filter == NULL)
    {
        return;
    }

    Biquad_Notch_SetCoefficients(filter, sample_rate_hz, center_hz, q);
}

/**
 * @brief 对单个采样执行 Direct Form II Transposed 二阶 IIR。
 * @note 首个有效样本用于预置延迟状态，并原样输出以避免启动冲击。
 */
float Biquad_Notch_Apply(BiquadNotchFilter_t *filter, float input)
{
    if ((filter == NULL) || (!isfinite(input)))
    {
        return input;
    }

    if (filter->initialized == 0U)
    {
        filter->z1          = (1.0f - filter->b0) * input;
        filter->z2          = (filter->b2 - filter->a2) * input;
        filter->initialized = 1U;
        return input;
    }

    const float output = filter->b0 * input + filter->z1;
    filter->z1         = filter->b1 * input - filter->a1 * output + filter->z2;
    filter->z2         = filter->b2 * input - filter->a2 * output;
    return output;
}
