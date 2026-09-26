/**
 * @file App_Control.c
 * @brief 四旋翼姿态、角速度与定高控制
 */

#include "App_Control.h"

#include "bsp_RTT.h"
#include "bsp_timestamp.h"
#include "module_pwm_motor.h"
#include "user_math.h"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <string.h>

static Control_Instance_t control;
/* 由 Control_Task 启动时登记，姿态任务发布新反馈后通过任务通知唤醒它。 */
static TaskHandle_t volatile control_task_handle;
/* ============================== PID 配置 ============================== */
static const PID_Init_Config_s rate_rp_config = {
    .Kp            = CONTROL_RATE_RP_KP,
    .Ki            = CONTROL_RATE_RP_KI,
    .Kd            = 0.0f,
    .MaxOut        = CONTROL_RATE_RP_MAX_OUT,
    .IntegralLimit = CONTROL_RATE_RP_INTEGRAL_LIMIT,
    .DeadBand      = 0.0f,
    .Improve       = PID_Integral_Limit,
};
static const PID_Init_Config_s rate_yaw_config = {
    .Kp            = CONTROL_RATE_YAW_KP,
    .Ki            = CONTROL_RATE_YAW_KI,
    .Kd            = 0.0f,
    .MaxOut        = CONTROL_RATE_YAW_MAX_OUT,
    .IntegralLimit = CONTROL_RATE_YAW_INTEGRAL_LIMIT,
    .DeadBand      = 0.0f,
    .Improve       = PID_Integral_Limit,
};
static const PID_Init_Config_s altitude_config = {
    .Kp            = CONTROL_ALT_VELOCITY_KP,
    .Ki            = CONTROL_ALT_VELOCITY_KI,
    .Kd            = 0.0f,
    .MaxOut        = CONTROL_ALT_MAX_CORRECTION,
    .IntegralLimit = CONTROL_ALT_INTEGRAL_LIMIT,
    .DeadBand      = 0.0f,
    .Improve       = PID_Integral_Limit | PID_Trapezoid_Integral,
};
/* ============================== 初始化 ============================== */
static void Control_InitPID(void)
{
    PIDInit(&control.roll_rate_pid, &rate_rp_config);
    PIDInit(&control.pitch_rate_pid, &rate_rp_config);
    PIDInit(&control.yaw_rate_pid, &rate_yaw_config);
    PIDInit(&control.altitude_velocity_pid, &altitude_config);
}
uint8_t Control_Init(void)
{
    memset(&control, 0, sizeof(control));
    control_task_handle = NULL;

    control.uav_cmd_subscriber = SubRegister(CONTROL_COMMAND_TOPIC_NAME, sizeof(Uav_Cmd_t));
    if (control.uav_cmd_subscriber == NULL)
    {
        RTTERROR("[Control] Command subscriber register failed.");
        return 0U;
    }

    control.log_publisher = PubRegister(CONTROL_LOG_TOPIC_NAME, sizeof(Control_Log_t));
    if (control.log_publisher == NULL)
    {
        RTTERROR("[Control] Log publisher register failed.");
        return 0U;
    }

    Control_InitPID();
    RTTINFO("[Control] Init success.");
    return 1U;
}
/* ============================== 反馈快照 ============================== */

void Control_SetFeedback(const Control_Feedback_t *feedback)
{
    if (feedback == NULL)
        return;

    /*
     * Attitude 是该反馈的唯一写者，且优先级高于 Control。写入期间低优先级
     * Control 不可能抢占，因此写端无需关闭可被 FreeRTOS 管理的中断。
     * Control 读端仍保留短临界区，防止其复制快照时被下一帧 Attitude 抢占。
     */
    control.feedback                       = *feedback;
    control.feedback.received_timestamp_us = Bsp_Timestamp_us_Get();
    control.feedback_valid                 = 1U;

    /*
     * 当前接口由 Attitude 任务调用。先完成反馈快照，再发送计数型通知，确保
     * Control_Task 被唤醒后一定能读取到完整的新数据。任务尚未启动时允许跳过通知，
     * Control_Task 会在最多 1 ms 后通过超时路径读取这份快照。
     */
    TaskHandle_t task_handle = control_task_handle;
    if (task_handle != NULL)
    {
        xTaskNotifyGive(task_handle);
    }
}

/* ============================== 输入检查 ============================== */

static uint8_t Control_TimeFresh(uint64_t now, uint64_t timestamp, uint64_t timeout)
{
    if (now >= timestamp)
        return ((now - timestamp) <= timeout) ? 1U : 0U;

    return ((timestamp - now) <= CONTROL_TIMESTAMP_FUTURE_TOLERANCE_US) ? 1U : 0U;
}

static uint8_t Control_CommandReady(uint64_t now)
{
    /* 没有新消息时可以继续使用上一帧，直到超过超时时间。 */
    if (SubGetMessage(control.uav_cmd_subscriber, &control.uav_cmd, 0U) != 0U)
        control.uav_cmd_valid = 1U;

    if (control.uav_cmd_valid == 0U)
        return 0U;
    if (control.uav_cmd.failsafe != 0U)
        return 0U;

    return Control_TimeFresh(now, control.uav_cmd.timestamp_us, CONTROL_CMD_TIMEOUT_US);
}

static uint8_t Control_FeedbackReady(uint64_t now, Control_Feedback_t *feedback)
{
    uint8_t valid;

    if (feedback == NULL)
        return 0U;

    taskENTER_CRITICAL();
    *feedback = control.feedback;
    valid     = control.feedback_valid;
    taskEXIT_CRITICAL();

    if (valid == 0U)
        return 0U;

    /*
     * received_timestamp_us 检查 Control 最近是否收到反馈；
     * timestamp_us 检查反馈本身代表的状态是否已经过时。
     */
    if (!Control_TimeFresh(now, feedback->received_timestamp_us, CONTROL_FEEDBACK_TIMEOUT_US))
        return 0U;
    if (!Control_TimeFresh(now, feedback->timestamp_us, CONTROL_FEEDBACK_TIMEOUT_US))
        return 0U;

    if ((!Math_IsFinite(feedback->dt_s)) || (feedback->dt_s <= 0.0f) || (feedback->dt_s > 0.02f))
        return 0U;
    if (!Math_Vector3IsFinite(feedback->gyro_rps))
        return 0U;
    if (!Math_IsFinite(feedback->yaw_rad))
        return 0U;

    /* 正常姿态四元数模长平方应该接近 1。 */
    const float q_norm2 = feedback->q_nb[0] * feedback->q_nb[0] + feedback->q_nb[1] * feedback->q_nb[1] +
                          feedback->q_nb[2] * feedback->q_nb[2] + feedback->q_nb[3] * feedback->q_nb[3];

    return (Math_IsFinite(q_norm2) && (q_norm2 > 0.5f) && (q_norm2 < 1.5f)) ? 1U : 0U;
}

/* ============================== 导航检查 ============================== */

static uint8_t Control_NavigationReady(const Control_Feedback_t *feedback)
{
    const uint8_t required_flags = CONTROL_NAVIGATION_ESKF_VALID | CONTROL_NAVIGATION_RANGE_FUSED;

    if ((feedback->navigation_flags & required_flags) != required_flags)
        return 0U;
    if (feedback->last_range_fusion_timestamp_us == 0ULL)
        return 0U;
    if (feedback->last_range_fusion_timestamp_us > feedback->timestamp_us)
        return 0U;
    if ((feedback->timestamp_us - feedback->last_range_fusion_timestamp_us) > CONTROL_ALT_RANGE_TIMEOUT_US)
        return 0U;

    if ((!Math_IsFinite(feedback->pos_ned_m[2])) || (!Math_IsFinite(feedback->vel_ned_mps[2])))
        return 0U;

    /* NED 的 Z 轴向下，因此实际高度为 -Z。 */
    const float height = -feedback->pos_ned_m[2];

    return ((height >= 0.0f) && (height <= CONTROL_ALT_MAX_HEIGHT_M + 1.0f)) ? 1U : 0U;
}

/* ============================== 安全状态 ============================== */

static void Control_Reset(void)
{
    if (Motor_GetState() == MOTOR_STATE_ARMED)
        Motor_Disarm();

    /* 控制器真正运行过时才清 PID，避免锁定状态每 1 ms 重复初始化。 */
    if ((control.rate_loop_active != 0U) || (control.altitude_hold_active != 0U))
        Control_InitPID();

    control.rate_loop_active     = 0U;
    control.altitude_hold_active = 0U;

    control.yaw_target_rad = 0.0f;

    control.altitude_target_m               = 0.0f;
    control.altitude_base_throttle          = 0.0f;
    control.altitude_stick_center           = 0.0f;
    control.altitude_vertical_speed_ref_mps = 0.0f;
    control.altitude_correction             = 0.0f;
}

static uint8_t Control_EnsureArmed(void)
{
    Motor_State_e state = Motor_GetState();

    /* Motor Fault 属于锁存故障，只允许上层显式恢复。 */
    if (state == MOTOR_STATE_FAULT)
        return 0U;

    if (state == MOTOR_STATE_DISARMED)
    {
        if (control.uav_cmd.throttle > CONTROL_ARM_THROTTLE_MAX)
            return 0U;
        if (Motor_Arm() == 0U)
            return 0U;

        state = Motor_GetState();
    }

    return (state == MOTOR_STATE_ARMED) ? 1U : 0U;
}

/* ============================== 定高控制 ============================== */

static float Control_VerticalSpeedCommand(float throttle, float center)
{
    const float delta = throttle - center;

    if (fabsf(delta) <= CONTROL_ALT_STICK_DEADBAND)
        return 0.0f;

    if (delta > 0.0f)
    {
        const float range = fmaxf(0.01f, 1.0f - center - CONTROL_ALT_STICK_DEADBAND);
        const float value = Math_ClampFloat((delta - CONTROL_ALT_STICK_DEADBAND) / range, 0.0f, 1.0f);

        return value * CONTROL_ALT_MAX_CLIMB_RATE_MPS;
    }

    const float range = fmaxf(0.01f, center - CONTROL_ALT_STICK_DEADBAND);
    const float value = Math_ClampFloat((-delta - CONTROL_ALT_STICK_DEADBAND) / range, 0.0f, 1.0f);

    return -value * CONTROL_ALT_MAX_DESCENT_RATE_MPS;
}

static float Control_UpdateAltitude(const Control_Feedback_t *feedback, uint8_t navigation_valid)
{
    const float throttle       = control.uav_cmd.throttle;
    const float height         = -feedback->pos_ned_m[2];
    const float vertical_speed = -feedback->vel_ned_mps[2];

    /*
     * 精简版本不负责自动起飞。
     * 未请求定高、导航异常或还没离地时直接使用人工油门。
     */
    if ((!control.uav_cmd.altitude_hold_request) || (!navigation_valid) || (height < CONTROL_ALT_ENTRY_MIN_HEIGHT_M))
    {
        if (control.altitude_hold_active != 0U)
            PIDInit(&control.altitude_velocity_pid, &altitude_config);

        control.altitude_hold_active            = 0U;
        control.altitude_vertical_speed_ref_mps = 0.0f;
        control.altitude_correction             = 0.0f;

        return throttle;
    }

    /*
     * 切入定高时：
     * 当前高度作为目标高度；
     * 当前实际油门作为基础油门；
     * 当前摇杆位置作为之后的垂直速度零点。
     */
    if (control.altitude_hold_active == 0U)
    {
        control.altitude_target_m      = height;
        control.altitude_base_throttle = throttle;
        control.altitude_stick_center  = throttle;
        control.altitude_correction    = 0.0f;
        control.altitude_hold_active   = 1U;

        PIDInit(&control.altitude_velocity_pid, &altitude_config);
    }

    float pilot_vz = Control_VerticalSpeedCommand(throttle, control.altitude_stick_center);

    /* 油门杆控制垂直速度，再积分形成新的高度目标。 */
    control.altitude_target_m += pilot_vz * feedback->dt_s;
    control.altitude_target_m = Math_ClampFloat(control.altitude_target_m, CONTROL_ALT_MIN_HEIGHT_M,
                                                CONTROL_ALT_MAX_HEIGHT_M);

    const float height_error = control.altitude_target_m - height;

    /* 高度 P 外环输出垂直速度目标。 */
    control.altitude_vertical_speed_ref_mps = Math_ClampFloat(pilot_vz + CONTROL_ALT_POSITION_KP * height_error,
                                                              -CONTROL_ALT_MAX_DESCENT_RATE_MPS,
                                                              CONTROL_ALT_MAX_CLIMB_RATE_MPS);

    /* 垂直速度 PI 输出基础油门附近的修正量。 */
    control.altitude_correction = PIDCalculate(&control.altitude_velocity_pid, vertical_speed,
                                               control.altitude_vertical_speed_ref_mps);

    return Math_ClampFloat(control.altitude_base_throttle + control.altitude_correction, CONTROL_ALT_MIN_THROTTLE,
                           CONTROL_ALT_MAX_THROTTLE);
}

/* ============================== 姿态外环 ============================== */

static void Control_GetRateReference(const Control_Feedback_t *feedback, float attitude_error[3], float rate_ref[3])
{
    /* Yaw 摇杆控制转动速度，积分后得到持续的航向角目标。 */
    control.yaw_target_rad = Math_WrapPi(control.yaw_target_rad + control.uav_cmd.yaw_rate_ref * feedback->dt_s);

    const MathQuaternionf current_q = {.w = feedback->q_nb[0],
                                       .x = feedback->q_nb[1],
                                       .y = feedback->q_nb[2],
                                       .z = feedback->q_nb[3]};

    const MathQuaternionf target_q = Math_QuaternionFromEuler(control.uav_cmd.roll_ref, control.uav_cmd.pitch_ref,
                                                              control.yaw_target_rad);

    Math_QuaternionErrorRotationVector(current_q, target_q, attitude_error);

    /* Roll / Pitch：姿态误差经过 P 控制得到目标角速度。 */
    rate_ref[0] = Math_ClampFloat(CONTROL_ATTITUDE_ROLL_KP * attitude_error[0], -CONTROL_ATTITUDE_MAX_RATE_RPS,
                                  CONTROL_ATTITUDE_MAX_RATE_RPS);

    rate_ref[1] = Math_ClampFloat(CONTROL_ATTITUDE_PITCH_KP * attitude_error[1], -CONTROL_ATTITUDE_MAX_RATE_RPS,
                                  CONTROL_ATTITUDE_MAX_RATE_RPS);

    /* Yaw 同时加入摇杆角速度前馈，提高打杆响应。 */
    rate_ref[2] = Math_ClampFloat(CONTROL_ATTITUDE_YAW_KP * attitude_error[2] + control.uav_cmd.yaw_rate_ref,
                                  -CONTROL_ATTITUDE_MAX_YAW_RATE_RPS, CONTROL_ATTITUDE_MAX_YAW_RATE_RPS);
}

/* ============================== Mixer ============================== */

static void Control_Mixer(float throttle, const float control_output[3], float motor[4])
{
    const float roll  = control_output[0];
    const float pitch = control_output[1];
    const float yaw   = control_output[2];

    /* X 型四旋翼：左前、右前、右后、左后。 */
    motor[0] = throttle + roll + pitch + yaw;
    motor[1] = throttle - roll + pitch - yaw;
    motor[2] = throttle - roll - pitch + yaw;
    motor[3] = throttle + roll - pitch - yaw;

    for (uint8_t i = 0U; i < 4U; i++)
        motor[i] = Math_ClampFloat(motor[i], 0.0f, 1.0f);
}

/* ============================== 日志 ============================== */

static void Control_PublishLog(const Control_Feedback_t *feedback, const float attitude_error[3],
                               const float rate_ref[3], const float rate_output[3], const float motor[4],
                               float throttle, uint8_t navigation_valid)
{
    Control_Log_t log = {0};

    log.timestamp_us = feedback->timestamp_us;
    log.sequence     = control.log_sequence++;

    memcpy(log.attitude_error_rad, attitude_error, sizeof(log.attitude_error_rad));
    memcpy(log.rate_ref_rps, rate_ref, sizeof(log.rate_ref_rps));
    memcpy(log.gyro_rps, feedback->gyro_rps, sizeof(log.gyro_rps));
    memcpy(log.rate_output, rate_output, sizeof(log.rate_output));
    memcpy(log.motor_output, motor, sizeof(log.motor_output));

    log.pilot_throttle     = control.uav_cmd.throttle;
    log.effective_throttle = throttle;
    log.yaw_target_rad     = control.yaw_target_rad;

    log.height_target_m  = control.altitude_target_m;
    log.height_measure_m = -feedback->pos_ned_m[2];

    log.vertical_speed_ref_mps     = control.altitude_vertical_speed_ref_mps;
    log.vertical_speed_measure_mps = -feedback->vel_ned_mps[2];

    log.altitude_correction = control.altitude_correction;

    log.navigation_valid     = navigation_valid;
    log.altitude_hold_active = control.altitude_hold_active;
    log.motor_armed          = (Motor_GetState() == MOTOR_STATE_ARMED) ? 1U : 0U;
    log.rate_loop_active     = control.rate_loop_active;

    (void)PubPushMessage(control.log_publisher, &log);
}

/* ============================== 控制输出 ============================== */

static void Control_Run(const Control_Feedback_t *feedback, float throttle, uint8_t navigation_valid)
{
    float attitude_error[3] = {0.0f};
    float rate_ref[3]       = {0.0f};
    float rate_output[3]    = {0.0f};
    float motor[4];

    /*
     * 极低油门不执行姿态闭环，防止飞机在地面无法响应控制时产生积分。
     * 同时持续把当前 Yaw 作为之后的目标航向。
     */
    if (throttle < CONTROL_RATE_LOOP_MIN_THROTTLE)
    {
        if (control.rate_loop_active != 0U)
        {
            PIDInit(&control.roll_rate_pid, &rate_rp_config);
            PIDInit(&control.pitch_rate_pid, &rate_rp_config);
            PIDInit(&control.yaw_rate_pid, &rate_yaw_config);

            control.rate_loop_active = 0U;
        }

        control.yaw_target_rad = feedback->yaw_rad;

        for (uint8_t i = 0U; i < 4U; i++)
            motor[i] = throttle;
    }
    else
    {
        /* 第一次进入闭环时以当前真实航向作为目标，避免 Yaw 突然跳变。 */
        if (control.rate_loop_active == 0U)
        {
            PIDInit(&control.roll_rate_pid, &rate_rp_config);
            PIDInit(&control.pitch_rate_pid, &rate_rp_config);
            PIDInit(&control.yaw_rate_pid, &rate_yaw_config);

            control.yaw_target_rad   = feedback->yaw_rad;
            control.rate_loop_active = 1U;
        }

        Control_GetRateReference(feedback, attitude_error, rate_ref);

        rate_output[0] = PIDCalculate(&control.roll_rate_pid, feedback->gyro_rps[0], rate_ref[0]);
        rate_output[1] = PIDCalculate(&control.pitch_rate_pid, feedback->gyro_rps[1], rate_ref[1]);
        rate_output[2] = PIDCalculate(&control.yaw_rate_pid, feedback->gyro_rps[2], rate_ref[2]);

        Control_Mixer(throttle, rate_output, motor);
    }

    if (Motor_SetOutput(motor) == 0U)
    {
        Motor_EmergencyStop();
        Control_Reset();
        return;
    }

    Control_PublishLog(feedback, attitude_error, rate_ref, rate_output, motor, throttle, navigation_valid);
}

/* ============================== 主控制任务 ============================== */

void Control_Task(void *argument)
{
    (void)argument;
    control_task_handle = xTaskGetCurrentTaskHandle();

    for (;;)
    {
        /*
         * 正常情况下由每次 Attitude 输出通知立即唤醒；若通知丢失或姿态链停止，
         * 最多等待 1 Tick 后进入安全检查。超时不运行正常控制律，避免使用上一帧
         * 姿态反馈重复更新 PID 和电机输出。pdTRUE 会合并等待期间累积的通知，
         * 收到通知后控制器只读取最新反馈，不追赶已经过时的中间样本。
        */
        const uint32_t notification_count = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(CONTROL_TASK_MAX_WAIT_MS));

        if (notification_count == 0U)
        {
            /*
             * 1 Tick 超时只负责安全监控：刷新遥控输入并检查反馈时效；若遥控失联、
             * 姿态链超时或飞手请求上锁，则立即复位控制器并锁定电机。反馈仍有效且
             * 仍请求解锁时保持上一拍输出，等待下一次真实姿态通知再更新控制律。
             */
            Control_Feedback_t timeout_feedback       = {0};
            const uint64_t     timeout_now            = Bsp_Timestamp_us_Get();
            const uint8_t      timeout_command_ready  = Control_CommandReady(timeout_now);
            const uint8_t      timeout_feedback_ready = Control_FeedbackReady(timeout_now, &timeout_feedback);

            if ((timeout_command_ready == 0U) || (timeout_feedback_ready == 0U) || (control.uav_cmd.arm_request == 0U))
            {
                Control_Reset();
            }
            continue;
        }

        Control_Feedback_t feedback       = {0};
        const uint64_t     now            = Bsp_Timestamp_us_Get();
        const uint8_t      feedback_ready = Control_FeedbackReady(now, &feedback);
        const uint8_t      command_ready  = Control_CommandReady(now);

        /* 遥控或姿态反馈失效属于基础飞行条件失效，立即锁定电机。 */
        if ((command_ready == 0U) || (feedback_ready == 0U))
        {
            Control_Reset();
            goto next_cycle;
        }

        if (control.uav_cmd.arm_request == 0U)
        {
            Control_Reset();
            goto next_cycle;
        }

        if (Control_EnsureArmed() == 0U)
            goto next_cycle;

        /*
         * Navigation 只影响定高。
         * 即使 ESKF / Range 不可用，姿态和 Rate 控制仍可继续运行。
         */
        const uint8_t navigation_valid = Control_NavigationReady(&feedback);

        /* 普通模式直接使用飞手油门，定高模式则得到修正后的有效油门。 */
        const float throttle = Control_UpdateAltitude(&feedback, navigation_valid);

        Control_Run(&feedback, throttle, navigation_valid);

    next_cycle:
        continue;
    }
}
