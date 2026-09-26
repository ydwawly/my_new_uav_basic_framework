#include "command_service.h"

#include <stdint.h>
#include <string.h>

#include "App_Uav_Cmd.h"
#include "App_Uav_Cmd_config.h"
#include "DataRouterTask.h"
#include "bsp_timestamp.h"
#include "hil_service.h"
#include "modules_Message_center.h"
#include "user_math.h"

/* MAVLink RC_CHANNELS_OVERRIDE 使用标准 PWM 范围，不沿用 SBUS 的原始量程。 */
#define MAVLINK_RC_CHANNEL_COUNT 8U
#define MAVLINK_RC_PWM_MIN       1000U
#define MAVLINK_RC_PWM_CENTER    1500U
#define MAVLINK_RC_PWM_MAX       2000U
#define MAVLINK_RC_PWM_DEADBAND  20U
#define MAVLINK_RC_SWITCH_HIGH   1500U
#define MAVLINK_RC_VALUE_RELEASE 0U
#define MAVLINK_RC_VALUE_IGNORE  UINT16_MAX

/*
 * MAVLink 接收任务只发布到独立输入主题；Uav_Cmd 任务负责与实体 SBUS 仲裁，
 * 再单点写入 control_command，保持顺序锁的单写者约束。
 */
static Publisher_t *command_publisher;

/* 未覆盖或单通道释放时采用的安全初值：姿态中位、油门最低、全部开关低位。 */
static uint16_t mavlink_rc_channels[MAVLINK_RC_CHANNEL_COUNT] = {
    MAVLINK_RC_PWM_CENTER, MAVLINK_RC_PWM_CENTER, MAVLINK_RC_PWM_MIN, MAVLINK_RC_PWM_CENTER,
    MAVLINK_RC_PWM_MIN,    MAVLINK_RC_PWM_MIN,    MAVLINK_RC_PWM_MIN, MAVLINK_RC_PWM_MIN,
};

static const uint16_t mavlink_rc_safe_channels[MAVLINK_RC_CHANNEL_COUNT] = {
    MAVLINK_RC_PWM_CENTER, MAVLINK_RC_PWM_CENTER, MAVLINK_RC_PWM_MIN, MAVLINK_RC_PWM_CENTER,
    MAVLINK_RC_PWM_MIN,    MAVLINK_RC_PWM_MIN,    MAVLINK_RC_PWM_MIN, MAVLINK_RC_PWM_MIN,
};

/** @brief 把输入 PWM 限制到 MAVLink 虚拟遥控器允许的标准范围。 */
static uint16_t CommandService_ClampRcPwm(uint16_t pwm)
{
    if (pwm < MAVLINK_RC_PWM_MIN)
    {
        return MAVLINK_RC_PWM_MIN;
    }
    if (pwm > MAVLINK_RC_PWM_MAX)
    {
        return MAVLINK_RC_PWM_MAX;
    }
    return pwm;
}

/**
 * @brief 处理 MAVLink 虚拟遥控器覆盖消息
 *
 * 0 表示释放对应通道，UINT16_MAX 表示保持该通道当前值。若八个基础通道全部
 * 为 0，则立即发布安全上锁指令。链路意外中断时，Control 自身的 100 ms 指令
 * 超时仍会强制上锁，因此这里不需要创建额外的软件定时器。
 */
static void CommandService_HandleRcOverride(const mavlink_message_t *msg)
{
    mavlink_rc_channels_override_t rc_override;
    mavlink_msg_rc_channels_override_decode(msg, &rc_override);

    if (!Mavlink_IsTarget(rc_override.target_system, rc_override.target_component) || (command_publisher == NULL))
    {
        return;
    }

    const uint16_t received[MAVLINK_RC_CHANNEL_COUNT] = {
        rc_override.chan1_raw, rc_override.chan2_raw, rc_override.chan3_raw, rc_override.chan4_raw,
        rc_override.chan5_raw, rc_override.chan6_raw, rc_override.chan7_raw, rc_override.chan8_raw,
    };

    uint8_t release_all = 1U;
    for (uint32_t i = 0U; i < MAVLINK_RC_CHANNEL_COUNT; i++)
    {
        if (received[i] != MAVLINK_RC_VALUE_RELEASE)
        {
            release_all = 0U;
        }

        if (received[i] == MAVLINK_RC_VALUE_IGNORE)
        {
            continue;
        }
        if (received[i] == MAVLINK_RC_VALUE_RELEASE)
        {
            mavlink_rc_channels[i] = mavlink_rc_safe_channels[i];
            continue;
        }
        mavlink_rc_channels[i] = CommandService_ClampRcPwm(received[i]);
    }

    Uav_Cmd_t command;
    memset(&command, 0, sizeof(command));
    command.timestamp_us      = Bsp_Timestamp_us_Get();
    command.throttle_raw      = mavlink_rc_channels[RC_CHANNEL_THROTTLE];
    command.altitude_mode_raw = mavlink_rc_channels[RC_CHANNEL_ALT_HOLD];

    const float roll_stick = Math_NormalizeCentered(mavlink_rc_channels[RC_CHANNEL_ROLL], MAVLINK_RC_PWM_MIN,
                                                    MAVLINK_RC_PWM_MAX, MAVLINK_RC_PWM_CENTER, MAVLINK_RC_PWM_DEADBAND);
    const float pitch_stick = Math_NormalizeCentered(mavlink_rc_channels[RC_CHANNEL_PITCH], MAVLINK_RC_PWM_MIN,
                                                     MAVLINK_RC_PWM_MAX, MAVLINK_RC_PWM_CENTER,
                                                     MAVLINK_RC_PWM_DEADBAND);
    const float yaw_stick = Math_NormalizeCentered(mavlink_rc_channels[RC_CHANNEL_YAW], MAVLINK_RC_PWM_MIN,
                                                   MAVLINK_RC_PWM_MAX, MAVLINK_RC_PWM_CENTER, MAVLINK_RC_PWM_DEADBAND);

    command.roll_ref     = roll_stick * CONTROL_MAX_ROLL_ANGLE_RAD;
    command.pitch_ref    = -pitch_stick * CONTROL_MAX_PITCH_ANGLE_RAD;
    command.yaw_rate_ref = yaw_stick * CONTROL_MAX_YAW_RATE_RAD_S;
    command.throttle = Math_Normalize(mavlink_rc_channels[RC_CHANNEL_THROTTLE], MAVLINK_RC_PWM_MIN, MAVLINK_RC_PWM_MAX);
    command.arm_request           = (mavlink_rc_channels[RC_CHANNEL_ARM] > MAVLINK_RC_SWITCH_HIGH) ? 1U : 0U;
    command.altitude_hold_request = (mavlink_rc_channels[RC_CHANNEL_ALT_HOLD] > MAVLINK_RC_SWITCH_HIGH) ? 1U : 0U;

    if (release_all != 0U)
    {
        memcpy(mavlink_rc_channels, mavlink_rc_safe_channels, sizeof(mavlink_rc_channels));
        command.throttle_raw = MAVLINK_RC_PWM_MIN;
        command.throttle     = 0.0f;
        command.arm_request  = 0U;
    }

    (void)PubPushMessage(command_publisher, &command);
}

/**
 * @brief 发送命令应答 (COMMAND_ACK)
 * @param request 触发该应答的原始请求消息指针（用于提取请求方的 sysid 和 compid）
 * @param command 具体响应的命令枚举值 (例如 MAV_CMD_DO_SET_MODE)
 * @param result  命令执行的结果状态 (例如 MAV_RESULT_ACCEPTED 或 MAV_RESULT_UNSUPPORTED)
 */
static void CommandService_SendAck(const mavlink_message_t *request, uint16_t command, uint8_t result)
{
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    /*
     * 设置命令执行进度：
     * 如果命令被接受并执行成功，进度为 100%；
     * 如果失败或不支持，使用 UINT8_MAX (255) 表示无进度或进度无效。
     */
    const uint8_t progress = (result == MAV_RESULT_ACCEPTED) ? 100U : UINT8_MAX;

    /* 编码 COMMAND_ACK 回复消息，将应答靶向发送给原本的请求方 */
    if (!Mavlink_TxTransactionBegin())
    {
        return;
    }

    const uint16_t length = Mavlink_EncodeCommandAck(buffer, sizeof(buffer), command, result, progress, request->sysid,
                                                     request->compid);

    if (length > 0U)
    {
        (void)DataRouter_PostFrame(buffer, length);
    }

    /* 必须在入队后再释放，防止其他任务的后一序号先进入发送队列。 */
    Mavlink_TxTransactionEnd();
}

/**
 * @brief 处理切换系统模式的指令 (MAV_CMD_DO_SET_MODE)
 * @param command 解码后的长命令结构体
 * @return 返回 MAV_RESULT 类型的执行结果
 */
static uint8_t CommandService_HandleSetMode(const mavlink_command_long_t *command)
{
    /*
     * 根据 MAVLink 协议约定：
     * 对于 DO_SET_MODE 命令，param1 携带了要设置的基础模式位掩码 (base_mode)
     */
    const uint8_t base_mode = (uint8_t)command->param1;

    /* 从 base_mode 掩码中提取出 HIL (硬件在环仿真) 模式标志位 */
    const bool hil_active = ((base_mode & MAV_MODE_FLAG_HIL_ENABLED) != 0U);

    /* 激活或关闭系统的 HIL 服务 */
    HilService_SetActive(hil_active);

    /* 模式切换执行成功 */
    return MAV_RESULT_ACCEPTED;
}

/**
 * @brief 解析并分发 MAV_CMD 长命令 (COMMAND_LONG)
 * @param msg 接收到的原始 MAVLink 消息
 */
static void CommandService_HandleCommandLong(const mavlink_message_t *msg)
{
    mavlink_command_long_t command;
    mavlink_msg_command_long_decode(msg, &command);

    /* 验证目标系统与组件：如果该命令不是发给本机的，则直接忽略，不作任何响应 */
    if (!Mavlink_IsTarget(command.target_system, command.target_component))
    {
        return;
    }

    /* 默认处理结果为不支持该命令 */
    uint8_t result = MAV_RESULT_UNSUPPORTED;

    /*
     * 集中分发所有支持的 MAV_CMD 指令
     */
    switch (command.command)
    {
    case MAV_CMD_DO_SET_MODE:
        result = CommandService_HandleSetMode(&command);
        break;

        /*
         * TODO: 后续可在此处扩展其他常用指令支持，例如：
         *
         * case MAV_CMD_COMPONENT_ARM_DISARM:       // 电机解锁/上锁
         * case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:  // 飞控重启/关机
         * case MAV_CMD_SET_MESSAGE_INTERVAL:       // 设置指定消息的发布频率
         */

    default:
        /* 对于不支持的命令，保持 result = MAV_RESULT_UNSUPPORTED */
        break;
    }

    /*
     * 无论命令执行成功、失败，还是根本不支持，
     * 都必须向地面站回复对应的 ACK 消息，以确保状态机闭环。
     */
    CommandService_SendAck(msg, command.command, result);
}

/* ========================================================= */
/*                     Public API (公共接口)                 */
/* ========================================================= */

bool CommandService_Init(void)
{
    command_publisher = PubRegister(MAVLINK_RC_COMMAND_TOPIC_NAME, sizeof(Uav_Cmd_t));
    return (command_publisher != NULL);
}

void CommandService_HandleMavlinkMessage(const mavlink_message_t *msg)
{
    if (msg == NULL)
    {
        return;
    }

    /* 拦截特定类型的 MAVLink 消息进入命令处理流 */
    switch (msg->msgid)
    {
    case MAVLINK_MSG_ID_COMMAND_LONG:
        CommandService_HandleCommandLong(msg);
        break;

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
        CommandService_HandleRcOverride(msg);
        break;

    default:
        break;
    }
}
