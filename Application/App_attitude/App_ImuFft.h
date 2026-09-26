/**
 * @file App_ImuFft.h
 * @brief Attitude 与 SensorHub 之间的 IMU 振动分析管线
 *
 * 数据所有权：
 *   - Attitude 是原始陀螺样本环形缓冲区的唯一生产者；
 *   - SensorHub 是唯一消费者，并独占 AdaptiveNotchTracker_t；
 *   - FFT 结果通过消息中心发布，Attitude 只做非阻塞读取。
 */
#ifndef APP_IMU_FFT_H
#define APP_IMU_FFT_H

#include <stdbool.h>
#include <stdint.h>

#include "Adaptive_Notch.h"

/** @brief 供 Attitude 消费的低频 FFT 结果。 */
typedef struct
{
    AdaptiveNotchResult_t notch;
    uint32_t              generation; /**< 每完成一个三轴分析窗加 1 */
} AppImuFftResult_t;

/** @brief 可在调试器中直接观察的管线健康计数。 */
typedef struct
{
    uint32_t pushed_samples;       /**< Attitude 成功入队的样本数 */
    uint32_t consumed_samples;     /**< SensorHub 成功消费的样本数 */
    uint32_t dropped_samples;      /**< 环形缓冲区满或输入非法造成的丢样数 */
    uint32_t discontinuity_resets; /**< 序号/时间戳不连续后重置 FFT 窗的次数 */
    uint32_t axis_fft_count;       /**< 已执行的单轴 FFT 次数 */
    uint32_t completed_windows;    /**< 已完成的三轴分析窗数 */
    uint32_t published_results;    /**< 已发布的 FFT 结果数 */
} AppImuFftStatus_t;

extern volatile AppImuFftStatus_t app_imu_fft_status;

/** @brief 初始化静态 SPSC 队列、Q15 FFT 跟踪器和结果主题。 */
bool App_ImuFft_Init(const AdaptiveNotchConfig_t *config);

/**
 * @brief Attitude 热路径调用：将陷波前的三轴陀螺样本无锁入队。
 * @return true 入队成功；false 管线未初始化、数据非法或队列已满
 */
bool App_ImuFft_PushSample(uint64_t timestamp_us, const float gyro_rps[3]);

/**
 * @brief SensorHub 每轮调用一次：有界搬运样本，并且最多计算一个轴的 FFT。
 * @note 三轴分析期间暂停向 tracker 写入，后续样本留在 SPSC 队列中，
 *       保证三轴分析同一个时间窗。
 */
void App_ImuFft_ProcessOneStep(void);

/** @brief Attitude 非阻塞读取最新 FFT 结果；无新结果立即返回 false。 */
bool App_ImuFft_TryGetResult(AppImuFftResult_t *result);

#endif /* APP_IMU_FFT_H */
