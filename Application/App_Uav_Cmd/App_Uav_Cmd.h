/**
 * @file App_Uav_Cmd.h
 * @brief 遥控器控制指令的公共数据和接口
 */

#ifndef APP_UAV_CMD_H
#define APP_UAV_CMD_H

#include <stdint.h>

#define CONTROL_COMMAND_TOPIC_NAME "control_command"
#define MAVLINK_RC_COMMAND_TOPIC_NAME "mavlink_rc_command"

typedef struct
{
    float    throttle;              /* 归一化油门，范围 0.0～1.0 */
    float    roll_ref;              /* Roll 角目标，rad */
    float    pitch_ref;             /* Pitch 角目标，rad */
    float    yaw_rate_ref;          /* Yaw 角速度输入，rad/s */
    uint16_t throttle_raw;          /* 接收机原始油门通道值，用于诊断和安全判定。 */
    uint16_t altitude_mode_raw;     /* 接收机原始定高开关通道值。 */
    uint8_t  arm_request;           /* 非零表示驾驶员请求解锁。 */
    uint8_t  altitude_hold_request; /* 非零表示驾驶员请求进入定高模式。 */
    uint8_t  failsafe;              /* 非零表示接收机失控保护或帧丢失。 */
    uint8_t  reserved;              /* 保留用于后续协议扩展，当前必须为 0。 */
    uint64_t timestamp_us;          /* 对应本次控制输入帧的单调时间戳，单位 us。 */
} Uav_Cmd_t;

/** @brief 初始化消息中心端点。 */
uint8_t Uav_Cmd_Init(void);

/** @brief 创建遥控解析任务。 */
uint8_t Uav_Cmd_StartTask(void);

#endif /* APP_UAV_CMD_H */
