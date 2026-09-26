#ifndef VQF_C_H
#define VQF_C_H

/**
 * @file VQF_C.h
 * @brief VQF C++ 姿态算法面向 C 飞控模块的无堆单例封装。
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    float    quat_ned_from_body[4];      /**< 机体系到 NED 的四元数 [w,x,y,z]。 */
    float    gyro_bias_rps[3];           /**< 三轴陀螺零偏估计，单位 rad/s。 */
    float    bias_sigma_rps;             /**< 零偏估计标准差，单位 rad/s。 */
    float    relative_rest_deviation[2]; /**< 陀螺/加速度静止检测相对偏差。 */
    uint32_t update_count;               /**< 成功更新累计次数。 */
    uint32_t invalid_output_count;       /**< 非有限输出累计次数。 */
    uint8_t  rest_detected;              /**< 非零表示 VQF 判定载体静止。 */
    uint8_t  initialized;                /**< 非零表示封装实例已初始化。 */
    uint8_t  reserved[2];                /**< 对齐及后续扩展保留。 */
} VqfCOutput_t;

/** @brief 使用静态存储初始化单例 VQF，sample_period_s 单位为 s。 */
bool VqfC_Init(float sample_period_s);

/** @brief 保留采样周期配置并清空 VQF 运行状态。 */
void VqfC_Reset(void);

/** @brief 将当前 6D 航向置零，不改变横滚和俯仰。 */
bool VqfC_ZeroHeading(void);

/**
 * @brief 输入一帧 FRD IMU 数据并输出控制器 NED 坐标系下的 6D 姿态。
 * @note 四元数按标量在前顺序存储：[w, x, y, z]。
 */
bool VqfC_Update(const float gyro_rps[3], const float accel_mps2[3], VqfCOutput_t *output);

#ifdef __cplusplus
}
#endif

#endif /* VQF_C_H */
