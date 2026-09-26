/**
 * @file App_attitude_hil.h
 * @brief HIL 数据源与姿态估计器之间的适配接口
 *
 * 该接口只在编译期选择 HIL 数据源时可见，负责订阅仿真 IMU/GPS/状态 Topic，
 * 并把 MAVLink HIL 数据转换成 ESKF 可以使用的初始状态和观测量。
 */
#ifndef APP_ATTITUDE_HIL_H
#define APP_ATTITUDE_HIL_H

#include "App_attitude_internal.h"
#include "hil_service.h"

#if (APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_HIL)

/** @brief 注册 HIL Topic 订阅者并初始化观测状态。 */
bool App_Attitude_Hil_Init(void);

/** @brief 清空一次仿真运行产生的原点、气压基准与观测限频状态。 */
void App_Attitude_Hil_Reset(void);

/**
 * @brief 尝试从 HIL_STATE_QUATERNION 真值建立一次性 ESKF 初始状态。
 * @param initial_state 输出的 ESKF 初始状态
 * @return true 已取得并转换有效初始状态，false 当前没有可用状态
 */
bool App_Attitude_Hil_TryGetInitialState(NavESKFState *initial_state);

/** @brief 阻塞等待下一帧 HIL_SENSOR 数据。 */
bool App_Attitude_Hil_WaitForSensor(HIL_Sensor_Data_t *sensor);

/** @brief 检查 HIL_SENSOR 更新位是否同时包含完整三轴陀螺和加速度。 */
bool App_Attitude_Hil_HasCompleteImu(uint32_t fields_updated);

/**
 * @brief 把磁力计、气压计和 GPS 慢速观测非阻塞地融合到 ESKF。
 * @note 仅在估计器初始化完成后执行，各观测内部带有有效性和更新间隔检查。
 */
void App_Attitude_Hil_UpdateObservations(NavESKF *eskf, const HIL_Sensor_Data_t *sensor, uint64_t timestamp_us,
                                         bool estimator_initialized);

#endif

#endif /* APP_ATTITUDE_HIL_H */
