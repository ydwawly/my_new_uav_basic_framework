/**
 * @file log_service.h
 * @brief SD 飞行日志生产与 MAVLink 日志下载服务
 */
#ifndef STM32H743_UAV_FLIGHT_CONTROLLER_LOG_SERVICE_H
#define STM32H743_UAV_FLIGHT_CONTROLLER_LOG_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "mavlink_user.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 注册控制日志 Topic 订阅者并复位日志节流状态。
 * @return true 初始化成功，false Topic 注册失败
 */
bool LogService_Init(void);

/** @brief 处理 MAVLink 日志枚举、读取、结束和擦除请求。 */
void LogService_HandleMavlinkMessage(const mavlink_message_t *msg);

/**
 * @brief 周期生成飞行控制日志，并把 SD 任务返回的数据封装后交给 DataRouter。
 * @param now_us 当前单调微秒时间戳
 */
void LogService_Update(uint64_t now_us);

#ifdef __cplusplus
}
#endif

#endif
