#ifndef STM32H743_UAV_FLIGHT_CONTROLLER_PT1_FILTER_H
#define STM32H743_UAV_FLIGHT_CONTROLLER_PT1_FILTER_H

/**
 * @file PT1_Filter.h
 * @brief 一阶低通滤波器状态和公共接口。
 */

#include <stdint.h>

typedef struct
{
    float   state;          /**< 上一次滤波输出。 */
    float   cutoff_freq_hz; /**< -3 dB 截止频率，单位 Hz。 */
    uint8_t initialized;    /**< 已接收首个合法样本时为 1。 */
} PT1_Filter_t;

/** @brief 初始化滤波器并设置截止频率。 */
void PT1_Filter_Init(PT1_Filter_t *filter, float cutoff_freq_hz);

/** @brief 输入采样值和实际周期 dt_s，返回本次滤波输出。 */
float PT1_Filter_Apply(PT1_Filter_t *filter, float input, float dt_s);

#endif /* STM32H743_UAV_FLIGHT_CONTROLLER_PT1_FILTER_H */
