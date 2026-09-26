#include "App_Uav_Cmd.h"
#include "App_Uav_Cmd_config.h"
#include "App_TaskPriorities.h"

/**
 * @file App_Uav_Cmd.c
 * @brief 将 SBUS 遥控通道转换为飞控统一指令并发布到消息中心。
 *
 * 该层集中处理通道归一化、死区、轴向约定、解锁开关和失控保护，使控制器不依赖
 * 接收机原始量程。解析与发布均在独立 FreeRTOS 任务中完成。
 */

#include "mudules_Sbus.h"
#include "modules_Message_center.h"
#include "bsp_RTT.h"
#include "task.h"

#include <string.h>

#include "user_math.h"

typedef struct
{
    Subscriber_t *sbus_subscriber;
    Subscriber_t *mavlink_rc_subscriber;
    Publisher_t  *uav_cmd_publisher;
} Uav_Cmd_Instance_t;

/* debug 镜像仅供调试器观察，控制链路始终使用消息中心中的正式数据。 */
static Uav_Cmd_Instance_t   uav_cmd_instance;
static volatile SBUS_Data_t sbus_debug;
static volatile Uav_Cmd_t   uav_cmd_debug;
#define UAV_CMD_TASK_STACK_WORDS 384U
#define UAV_CMD_WAIT_TIMEOUT_MS  10U

static StaticTask_t uav_cmd_task_control;
static StackType_t  uav_cmd_task_stack[UAV_CMD_TASK_STACK_WORDS];
static TaskHandle_t uav_cmd_task_handle = NULL;

/** @brief 注册 SBUS 订阅者和控制指令发布者。 */
uint8_t Uav_Cmd_Init(void)
{
    memset(&uav_cmd_instance, 0, sizeof(uav_cmd_instance));

    uav_cmd_instance.sbus_subscriber = SubRegister("sbus_data", sizeof(SBUS_Data_t));

    if (uav_cmd_instance.sbus_subscriber == NULL)
    {
        RTTERROR("[Uav_Cmd] SBUS SubRegister Failed!");

        return 0U;
    }

    /*
     * MAVLink 虚拟遥控器使用独立输入主题，最终仍由本任务单点发布控制指令。
     * 这样 control_command 顺序锁始终只有一个写者，不会出现多任务嵌套写入。
     */
    uav_cmd_instance.mavlink_rc_subscriber = SubRegister(MAVLINK_RC_COMMAND_TOPIC_NAME, sizeof(Uav_Cmd_t));
    if (uav_cmd_instance.mavlink_rc_subscriber == NULL)
    {
        RTTERROR("[Uav_Cmd] MAVLink RC SubRegister Failed!");
        return 0U;
    }

    uav_cmd_instance.uav_cmd_publisher = PubRegister(CONTROL_COMMAND_TOPIC_NAME, sizeof(Uav_Cmd_t));

    if (uav_cmd_instance.uav_cmd_publisher == NULL)
    {
        RTTERROR("[Uav_Cmd] Command PubRegister Failed!");

        return 0U;
    }

    RTTINFO("[Uav_Cmd] Init Success! Alt hold: SBUS channels[5] > 1025.");

    return 1U;
}

/**
 * @brief 把一帧 SBUS 数据解析为归一化控制指令。
 * @note failsafe 或丢帧标志出现时，强制撤销解锁请求并把油门置零。
 */
static void Uav_Cmd_ParseSBUS(const SBUS_Data_t *sbus_data, Uav_Cmd_t *uav_cmd)
{
    if (sbus_data == NULL || uav_cmd == NULL)
    {
        return;
    }

    memset(uav_cmd, 0, sizeof(Uav_Cmd_t));
    uav_cmd->timestamp_us      = sbus_data->Sbus_Timestamp;
    uav_cmd->throttle_raw      = sbus_data->channels[RC_CHANNEL_THROTTLE];
    uav_cmd->altitude_mode_raw = sbus_data->channels[RC_CHANNEL_ALT_HOLD];

    /* 接收机报告异常时立即发布锁定命令。 */
    if (sbus_data->failsafe != 0 || sbus_data->frame_lost != 0)
    {
        uav_cmd->failsafe    = 1U;
        uav_cmd->arm_request = 0U;
        uav_cmd->throttle    = 0.0f;
        return;
    }

    const float roll_stick  = Math_NormalizeCentered(sbus_data->channels[RC_CHANNEL_ROLL], RC_CHANNEL_MIN,
                                                     RC_CHANNEL_MAX, RC_CHANNEL_CENTER, RC_CHANNEL_DEADBAND);
    const float pitch_stick = Math_NormalizeCentered(sbus_data->channels[RC_CHANNEL_PITCH], RC_CHANNEL_MIN,
                                                     RC_CHANNEL_MAX, RC_CHANNEL_CENTER, RC_CHANNEL_DEADBAND);
    const float yaw_stick = Math_NormalizeCentered(sbus_data->channels[RC_CHANNEL_YAW], RC_CHANNEL_MIN, RC_CHANNEL_MAX,
                                                   RC_CHANNEL_CENTER, RC_CHANNEL_DEADBAND);

    uav_cmd->roll_ref = roll_stick * CONTROL_MAX_ROLL_ANGLE_RAD;
    /* 只在遥控入口反向 Pitch：前推摇杆对应负俯仰（机头向下），内部轴系不变。 */
    uav_cmd->pitch_ref    = -pitch_stick * CONTROL_MAX_PITCH_ANGLE_RAD;
    uav_cmd->yaw_rate_ref = yaw_stick * CONTROL_MAX_YAW_RATE_RAD_S;
    uav_cmd->throttle     = Math_Normalize(sbus_data->channels[RC_CHANNEL_THROTTLE], RC_CHANNEL_MIN, RC_CHANNEL_MAX);

    uav_cmd->arm_request           = (sbus_data->channels[RC_CHANNEL_ARM] > RC_ARM_SWITCH_THRESHOLD) ? 1U : 0U;
    uav_cmd->altitude_hold_request = (uav_cmd->altitude_mode_raw > RC_ALT_HOLD_SWITCH_THRESHOLD) ? 1U : 0U;
}

/** @brief 等待最新 SBUS 帧，解析后发布 CONTROL_COMMAND_TOPIC_NAME。 */
static void Uav_Cmd_Task(void *argument)
{
    (void)argument;
    SBUS_Data_t sbus_data;
    Uav_Cmd_t   uav_cmd;
    Uav_Cmd_t   mavlink_rc_cmd;

    for (;;)
    {
        /*
         * 正常由 SBUS 发布通知立即唤醒；10 ms 超时用于兜底读取最新值，
         * 避免一次通知异常使控制指令永久停在旧帧。超时本身不发布旧数据。
         */
        if ((uav_cmd_instance.sbus_subscriber == NULL) || (uav_cmd_instance.mavlink_rc_subscriber == NULL) ||
            (uav_cmd_instance.uav_cmd_publisher == NULL))
        {
            continue;
        }

        /*
         * 先读取远程覆盖；随后若同一周期也收到实体 SBUS，则 SBUS 最后发布并优先。
         * MAVLink 到达正在等待 SBUS 的间隙时，最坏在 10 ms 超时后处理。
         */
        if (SubGetMessage(uav_cmd_instance.mavlink_rc_subscriber, &mavlink_rc_cmd, 0U) != 0U)
        {
            uav_cmd_debug = mavlink_rc_cmd;
            (void)PubPushMessage(uav_cmd_instance.uav_cmd_publisher, &mavlink_rc_cmd);
        }

        if (SubGetMessage(uav_cmd_instance.sbus_subscriber, &sbus_data, pdMS_TO_TICKS(UAV_CMD_WAIT_TIMEOUT_MS)) == 0U)
        {
            continue;
        }

        Uav_Cmd_ParseSBUS(&sbus_data, &uav_cmd);
        sbus_debug    = sbus_data;
        uav_cmd_debug = uav_cmd;
        PubPushMessage(uav_cmd_instance.uav_cmd_publisher, &uav_cmd);
    }
}

/** @brief 使用静态栈创建遥控解析任务；重复调用不会重复创建。 */
uint8_t Uav_Cmd_StartTask(void)
{
    if (uav_cmd_task_handle != NULL)
    {
        return 1U;
    }

    /* 静态任务内存不占用 FreeRTOS 通用堆；任务通常阻塞在消息订阅上。 */
    uav_cmd_task_handle = xTaskCreateStatic(Uav_Cmd_Task, "uav_cmd", UAV_CMD_TASK_STACK_WORDS, NULL,
                                            APP_TASK_PRIORITY_UAV_CMD, uav_cmd_task_stack, &uav_cmd_task_control);
    return (uav_cmd_task_handle != NULL) ? 1U : 0U;
}
