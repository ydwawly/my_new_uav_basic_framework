#include "PT1_Filter.h"

/**
 * @file PT1_Filter.c
 * @brief 采用实际采样周期计算系数的一阶低通滤波器实现。
 */

#include <math.h>
#include <stddef.h>

#define PT1_TWO_PI_F 6.28318530717958647692f

/** @brief 清零滤波状态并保存截止频率。 */
void PT1_Filter_Init(PT1_Filter_t *filter, float cutoff_freq_hz)
{
    if (filter == NULL)
    {
        return;
    }

    filter->state          = 0.0f;
    filter->cutoff_freq_hz = cutoff_freq_hz;
    filter->initialized    = 0U;
}

/**
 * @brief 对一个样本执行离散 RC 一阶低通更新。
 *
 * 首个合法样本直接作为初值，防止启动瞬间从零缓慢爬升；参数非法时旁路返回输入。
 */
float PT1_Filter_Apply(PT1_Filter_t *filter, float input, float dt_s)
{
    if ((filter == NULL) || (!isfinite(input)) || (!isfinite(dt_s)) || (dt_s <= 0.0f) ||
        (!isfinite(filter->cutoff_freq_hz)) || (filter->cutoff_freq_hz <= 0.0f))
    {
        return input;
    }

    if (filter->initialized == 0U)
    {
        filter->state       = input;
        filter->initialized = 1U;
        return input;
    }

    const float rc_s  = 1.0f / (PT1_TWO_PI_F * filter->cutoff_freq_hz);
    const float alpha = dt_s / (dt_s + rc_s);

    filter->state = filter->state + alpha * (input - filter->state);
    return filter->state;
}
