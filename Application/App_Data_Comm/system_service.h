#ifndef SYSTEM_SERVICE_H
#define SYSTEM_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "mavlink_user.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
     * @brief 初始化系统服务
     * @return 初始化成功返回 true，否则返回 false
     */
bool SystemService_Init(void);

/**
     * @brief 处理与系统服务相关的 MAVLink 消息
     * @param msg 接收到的 MAVLink 消息指针
     *
     * 负责解析和响应 PING (延迟检测) 和 TIMESYNC (时间同步) 等基础链路维护消息。
     */
void SystemService_HandleMavlinkMessage(const mavlink_message_t *msg);

/**
     * @brief 系统服务周期性更新任务
     * @param now_us 当前的系统时间戳（微秒）
     *
     * 需要在系统主循环或 Service 任务中周期调用，负责以固定的频率（1Hz）向外发送心跳包 (Heartbeat)。
     */
void SystemService_Update(uint64_t now_us);

#ifdef __cplusplus
}
#endif

#endif // SYSTEM_SERVICE_H