#ifndef APP_ATTITUDE_INTERNAL_H
#define APP_ATTITUDE_INTERNAL_H

/**
 * @file App_attitude_internal.h
 * @brief 姿态主任务与真实传感器/HIL 观测子模块共享的内部类型。
 * @note 这些类型不是应用层公共接口，只应在 App_attitude 模块内部使用。
 */

#include <stdbool.h>
#include <stdint.h>

#include "App_attitude_config.h"
#include "Biquad_Notch.h"
#include "ESKF.h"
#include "PT1_Filter.h"
#include "VQF_C.h"

/* HIL 与真实传感器统一使用的 IMU 样本。 */
typedef struct
{
    uint64_t timestamp_us;  /**< 单调采样时间，单位 us。 */
    float    gyro_rps[3];   /**< 机体系三轴角速度，单位 rad/s。 */
    float    accel_mps2[3]; /**< 机体系三轴比力，单位 m/s²。 */
} AppAttitudeImuSample_t;

/* 姿态任务及观测子模块共享的最小运行状态。 */
typedef struct
{
    /* IMU 积分与滤波时间基准。 */
    uint64_t previous_imu_timestamp_us;
    uint64_t previous_filter_timestamp_us;

    /* 各轴独立的一阶低通状态。 */
    PT1_Filter_t gyro_lpf[3];
    PT1_Filter_t accel_lpf[3];

#if ((APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_REAL) && (ATTITUDE_IMU_NOTCH_ENABLE == 1U))
    BiquadNotchFilter_t gyro_notch[3];
    BiquadNotchFilter_t accel_notch[3];
#endif

#if (APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_HIL)
    /* HIL 静止对准期间的加速度计/陀螺仪累加值。 */
    float alignment_accel_sum[3];
    float alignment_gyro_sum[3];
#endif
    uint16_t alignment_sample_count;

#if (APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_REAL)
    /* 主 VQF 姿态解算的最新输出快照。 */
    VqfCOutput_t vqf_output;
#endif

#if ((APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_REAL) && (ATTITUDE_PARALLEL_ESKF_ENABLE == 1U))
    /* 并行 ESKF 的光流积分窗口及测距观测时间戳。 */
    uint64_t last_mtf_flow_timestamp_us;
    uint64_t last_tfmini_timestamp_us;
    float    flow_gyro_integral_rad[2];
    float    flow_gyro_interval_s;
#endif

    /* 融合节流时间戳及估计器生命周期状态。 */
    uint64_t last_range_fusion_timestamp_us;
    uint8_t  estimator_initialized;
    uint8_t  parallel_eskf_initialized;
    uint8_t  eskf_predict_valid;
} AppAttitudeRuntime_t;

#endif /* APP_ATTITUDE_INTERNAL_H */
