/**
 * @file App_Control.h
 * @brief 四旋翼姿态、角速度与定高控制模块
 */

#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include <stdint.h>

#include "App_Uav_Cmd.h"
#include "PID.h"
#include "modules_Message_center.h"
/* ============================== Topic ============================== */
#define CONTROL_LOG_TOPIC_NAME "control_log"
/* ============================== 基本参数 ============================== */
/* 姿态通知缺失时的最长等待时间；超时只执行安全检查，不运行正常控制律。 */
#define CONTROL_TASK_MAX_WAIT_MS 1U

#define CONTROL_CMD_TIMEOUT_US                100000ULL
#define CONTROL_FEEDBACK_TIMEOUT_US           10000ULL
#define CONTROL_TIMESTAMP_FUTURE_TOLERANCE_US 5000ULL

#define CONTROL_ARM_THROTTLE_MAX       0.05f
#define CONTROL_RATE_LOOP_MIN_THROTTLE 0.08f

/* ============================== 姿态外环 ============================== */

#define CONTROL_ATTITUDE_ROLL_KP  4.0f
#define CONTROL_ATTITUDE_PITCH_KP 4.0f
#define CONTROL_ATTITUDE_YAW_KP   2.0f

#define CONTROL_ATTITUDE_MAX_RATE_RPS     1.6f
#define CONTROL_ATTITUDE_MAX_YAW_RATE_RPS 1.0f

/* ============================== 角速度内环 ============================== */

#define CONTROL_RATE_RP_KP             0.08f
#define CONTROL_RATE_RP_KI             0.015f
#define CONTROL_RATE_RP_MAX_OUT        0.15f
#define CONTROL_RATE_RP_INTEGRAL_LIMIT 0.04f

#define CONTROL_RATE_YAW_KP             0.14f
#define CONTROL_RATE_YAW_KI             0.015f
#define CONTROL_RATE_YAW_MAX_OUT        0.10f
#define CONTROL_RATE_YAW_INTEGRAL_LIMIT 0.04f

/* ============================== 定高控制 ============================== */

#define CONTROL_ALT_RANGE_TIMEOUT_US 150000ULL

#define CONTROL_ALT_ENTRY_MIN_HEIGHT_M 0.15f
#define CONTROL_ALT_MIN_HEIGHT_M       0.15f
#define CONTROL_ALT_MAX_HEIGHT_M       4.00f

#define CONTROL_ALT_STICK_DEADBAND 0.04f

#define CONTROL_ALT_MAX_CLIMB_RATE_MPS   0.80f
#define CONTROL_ALT_MAX_DESCENT_RATE_MPS 0.50f

#define CONTROL_ALT_POSITION_KP 0.80f

#define CONTROL_ALT_VELOCITY_KP 0.15f
#define CONTROL_ALT_VELOCITY_KI 0.08f

#define CONTROL_ALT_MAX_CORRECTION 0.15f
#define CONTROL_ALT_INTEGRAL_LIMIT 0.10f

#define CONTROL_ALT_MIN_THROTTLE 0.15f
#define CONTROL_ALT_MAX_THROTTLE 0.70f

/* ============================== 导航状态 ============================== */

typedef enum
{
    CONTROL_NAVIGATION_ESKF_VALID  = (1U << 0U),
    CONTROL_NAVIGATION_RANGE_FUSED = (1U << 1U)
} ControlNavigationFlag_e;

/* ============================== 控制反馈 ============================== */

/**
 * @brief Control 任务使用的统一状态快照
 *
 * q_nb：
 *     当前机体姿态四元数，用于姿态误差计算。
 *
 * yaw_rad：
 *     当前航向角，只用于进入闭环时初始化 Yaw 目标。
 *
 * gyro_rps：
 *     三轴实际角速度，是 Rate PI 的直接反馈。
 *
 * pos_ned_m / vel_ned_mps：
 *     ESKF 的 NED 位置和速度，目前定高只使用 Z 轴。
 *
 * navigation_flags：
 *     判断 ESKF 和测距融合是否可以支持定高。
 */
typedef struct
{
    uint64_t timestamp_us;
    uint64_t received_timestamp_us;
    uint64_t last_range_fusion_timestamp_us;

    float dt_s;

    float q_nb[4];
    float yaw_rad;
    float gyro_rps[3];
    float accel_mps2[3];
    float pos_ned_m[3];
    float vel_ned_mps[3];

    uint8_t navigation_flags;
} Control_Feedback_t;

/* ============================== 控制日志 ============================== */

/**
 * @brief Control 模块自身产生的关键控制日志
 *
 * 完整 IMU、姿态、ESKF、遥控数据由 Logger 从对应 Topic 单独记录。
 * Control Topic 只保留控制器计算过程中的关键量。
 */
typedef struct
{
    uint64_t timestamp_us;
    uint32_t sequence;

    float attitude_error_rad[3];
    float rate_ref_rps[3];
    float gyro_rps[3];
    float rate_output[3];

    float motor_output[4];

    float pilot_throttle;
    float effective_throttle;
    float yaw_target_rad;

    float height_target_m;
    float height_measure_m;

    float vertical_speed_ref_mps;
    float vertical_speed_measure_mps;

    float altitude_correction;

    uint8_t navigation_valid;
    uint8_t altitude_hold_active;
    uint8_t motor_armed;
    uint8_t rate_loop_active;
} Control_Log_t;

/* ============================== 模块实例 ============================== */

typedef struct
{
    Subscriber_t *uav_cmd_subscriber;
    Publisher_t  *log_publisher;

    Uav_Cmd_t          uav_cmd;
    Control_Feedback_t feedback;

    PIDInstance roll_rate_pid;
    PIDInstance pitch_rate_pid;
    PIDInstance yaw_rate_pid;
    PIDInstance altitude_velocity_pid;

    float yaw_target_rad;

    float altitude_target_m;
    float altitude_base_throttle;
    float altitude_stick_center;
    float altitude_vertical_speed_ref_mps;
    float altitude_correction;

    uint32_t log_sequence;

    uint8_t uav_cmd_valid;
    uint8_t feedback_valid;
    uint8_t rate_loop_active;
    uint8_t altitude_hold_active;
} Control_Instance_t;

/* ============================== 公共接口 ============================== */

uint8_t Control_Init(void);

/* 更新姿态、角速度以及导航状态统一快照，并通知控制任务处理最新数据。 */
void Control_SetFeedback(const Control_Feedback_t *feedback);

/* 姿态更新事件驱动的主控制任务；超时唤醒仅检查失联、上锁等安全条件。 */
void Control_Task(void *argument);

#endif /* APP_CONTROL_H */
