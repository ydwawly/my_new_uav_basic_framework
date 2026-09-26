/**
 * @file App_Uav_Cmd_config.h
 * @brief 遥控器通道、标定值和控制指令限幅
 */

#ifndef APP_UAV_CMD_CONFIG_H
#define APP_UAV_CMD_CONFIG_H

/* SBUS 数组下标从 0 开始；定高模式使用物理 CH6。 */
#define RC_CHANNEL_ROLL     0U
#define RC_CHANNEL_PITCH    1U
#define RC_CHANNEL_THROTTLE 2U
#define RC_CHANNEL_YAW      3U
#define RC_CHANNEL_ALT_HOLD 5U
#define RC_CHANNEL_ARM      7U

/* 当前 FS-i6X/SBUS 实测标定值。 */
#define RC_CHANNEL_MIN      250U
#define RC_CHANNEL_CENTER   1025U
#define RC_CHANNEL_MAX      1807U
#define RC_CHANNEL_DEADBAND 20U

#define RC_ARM_SWITCH_THRESHOLD      1400U
#define RC_ALT_HOLD_SWITCH_THRESHOLD RC_CHANNEL_CENTER

#define CONTROL_MAX_ROLL_ANGLE_RAD  0.34906585f /* 20 deg */
#define CONTROL_MAX_PITCH_ANGLE_RAD 0.34906585f /* 20 deg */
#define CONTROL_MAX_YAW_RATE_RAD_S  0.50f       /* 28.6 deg/s */

#endif /* APP_UAV_CMD_CONFIG_H */
