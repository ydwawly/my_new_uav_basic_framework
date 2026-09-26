/**
 * @file App_attitude_observations.h
 * @brief 真实传感器光流与测距观测的 ESKF 融合接口
 */
#ifndef APP_ATTITUDE_OBSERVATIONS_H
#define APP_ATTITUDE_OBSERVATIONS_H

#include <stdbool.h>

#include "App_attitude_internal.h"

/**
 * @brief 注册启用的光流、测距 Topic 订阅者。
 * @return true 所有编译期启用的观测源均注册成功
 */
bool App_Attitude_Observations_Init(void);

/**
 * @brief 非阻塞消费最新外部观测并更新导航 ESKF。
 * @param gyro_rps 本轮滤波后的机体系角速度，单位 rad/s
 * @param dt_s 本轮 IMU 实测时间步长，单位 s
 */
void App_Attitude_Observations_Update(NavESKF *eskf, AppAttitudeRuntime_t *runtime, const float gyro_rps[3],
                                      float dt_s);

#endif /* APP_ATTITUDE_OBSERVATIONS_H */
