#ifndef COMMAND_SERVICE_H
#define COMMAND_SERVICE_H

#include <stdbool.h>
#include "mavlink_user.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
     * @brief 初始化命令服务
     * @return 初始化成功返回 true，否则返回 false
     */
bool CommandService_Init(void);

/**
     * @brief 处理与命令相关的 MAVLink 消息
     * @param msg 接收到的 MAVLink 消息指针
     *
     * 核心用于拦截处理 MAVLINK_MSG_ID_COMMAND_LONG，
     * 负责解析具体命令，如模式切换、重启等，并自动回复 ACK 确认。
     */
void CommandService_HandleMavlinkMessage(const mavlink_message_t *msg);

#ifdef __cplusplus
}
#endif

#endif // COMMAND_SERVICE_H