/**
 * @file Biquad_Notch.h
 * @brief 二阶 IIR 陷波器，用于抑制电机振动等窄带噪声
 */
#ifndef BIQUAD_NOTCH_H
#define BIQUAD_NOTCH_H

#include <stdint.h>

/** @brief Direct Form II Transposed 实现所需的系数与延迟状态。 */
typedef struct
{
    float   b0;          /**< 前向系数 b0 */
    float   b1;          /**< 前向系数 b1 */
    float   b2;          /**< 前向系数 b2 */
    float   a1;          /**< 反馈系数 a1，计算时已包含符号约定 */
    float   a2;          /**< 反馈系数 a2，计算时已包含符号约定 */
    float   z1;          /**< 一级延迟状态 */
    float   z2;          /**< 二级延迟状态 */
    uint8_t initialized; /**< 1 表示系数已通过合法参数初始化 */
} BiquadNotchFilter_t;

/** @brief 计算滤波器系数并清空历史状态。 */
void Biquad_Notch_Init(BiquadNotchFilter_t *filter, float sample_rate_hz, float center_hz, float q);

/** @brief 在线更新中心频率和 Q 值，同时保留延迟状态以避免输出突跳。 */
void Biquad_Notch_Update(BiquadNotchFilter_t *filter, float sample_rate_hz, float center_hz, float q);

/** @brief 对一个输入样本执行陷波；滤波器未初始化时原样返回输入。 */
float Biquad_Notch_Apply(BiquadNotchFilter_t *filter, float input);

#endif /* BIQUAD_NOTCH_H */
