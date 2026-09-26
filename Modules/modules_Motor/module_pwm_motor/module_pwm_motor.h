//
// Created by Administrator on 2026/7/26.
//

#ifndef MY_NEW_UAV_BASIC_FRAMEWORK_MODULE_PWM_MOTOR_H
#define MY_NEW_UAV_BASIC_FRAMEWORK_MODULE_PWM_MOTOR_H

#include <stdint.h>

#include "bsp_pwm.h"

/* ========================== 基础配置 ========================== */

/*
 * 四旋翼电机数量。
 */
#define MOTOR_COUNT 4U

/*
 * 普通 PWM 电调输出频率为 400 Hz：
 *
 * period = 1 / 400 = 2500 us
 */
#define MOTOR_PWM_PERIOD_US 2500U

/*
 * 普通单向 PWM 电调脉宽。
 *
 * 锁定状态：1000 us
 * 解锁怠速：1050 us
 * 最大输出：2000 us
 */
#define MOTOR_PWM_MIN_PULSE_US  1000U
#define MOTOR_PWM_IDLE_PULSE_US 1050U
#define MOTOR_PWM_MAX_PULSE_US  2000U

/*
 * 电机输出刷新超时时间。
 *
 * 如果已经解锁，但超过 100 ms 没有调用
 * Motor_SetOutput()，则进入故障状态。
 *
 * 需要由独立的守护任务周期调用 Motor_SafetyCheck()。
 */
#define MOTOR_OUTPUT_TIMEOUT_US 100000ULL

/* ========================== 电机编号 ========================== */

/**
 * @brief 四旋翼电机编号
 *
 * @note
 * 当前编号约定：
 *
 * 0：左前
 * 1：右前
 * 2：右后
 * 3：左后
 */
typedef enum
{
    MOTOR_ID_FRONT_LEFT = 0,
    MOTOR_ID_FRONT_RIGHT,
    MOTOR_ID_REAR_RIGHT,
    MOTOR_ID_REAR_LEFT,

    MOTOR_ID_COUNT

} Motor_ID_e;

/* ========================== 电机状态 ========================== */

/**
 * @brief 电机状态
 */
typedef enum
{
    /*
     * 锁定状态。
     *
     * 四路电调固定输出最小脉宽。
     */
    MOTOR_STATE_DISARMED = 0,

    /*
     * 解锁状态。
     *
     * 允许控制任务输出 0.0～1.0 的电机命令。
     */
    MOTOR_STATE_ARMED,

    /*
     * 故障状态。
     *
     * 四路电调固定输出最小脉宽，
     * 必须调用 Motor_ResetFault() 才能恢复。
     */
    MOTOR_STATE_FAULT

} Motor_State_e;

/* ========================== 故障类型 ========================== */

/**
 * @brief 电机模块故障类型
 */
typedef enum
{
    MOTOR_FAULT_NONE = 0,

    /*
     * PWM 注册、配置或者启动失败。
     */
    MOTOR_FAULT_PWM_INIT,

    /*
     * 电机输出出现 NaN 或无穷大。
     */
    MOTOR_FAULT_INVALID_COMMAND,

    /*
     * 写入 PWM 比较值失败。
     */
    MOTOR_FAULT_PWM_WRITE,

    /*
     * 电机输出长时间没有刷新。
     */
    MOTOR_FAULT_OUTPUT_TIMEOUT,

    /*
     * 外部触发紧急停机。
     */
    MOTOR_FAULT_EMERGENCY_STOP

} Motor_Fault_e;

/**
 * @brief 单路电机硬件与脉宽配置
 */
typedef struct
{
    TIM_HandleTypeDef *htim;          /* HAL 定时器句柄 */
    uint32_t           channel;       /* TIM_CHANNEL_1～TIM_CHANNEL_4 */
    uint32_t           period_us;     /* PWM 周期，单位 us */
    uint16_t           min_pulse_us;  /* 锁定脉宽，单位 us */
    uint16_t           idle_pulse_us; /* 解锁怠速脉宽，单位 us */
    uint16_t           max_pulse_us;  /* 最大输出脉宽，单位 us */
} Motor_Channel_Config_t;

/**
 * @brief 四路电机模块运行状态
 *
 * 该类型放在头文件便于调试器观察；实际单例仍由 module_pwm_motor.c 私有管理。
 */
typedef struct
{
    PWMInstance  *pwm_instance[MOTOR_COUNT];   /* 四路 BSP PWM 实例 */
    float         output[MOTOR_COUNT];         /* 当前归一化输出，范围 0.0～1.0 */
    uint16_t      pulse_width_us[MOTOR_COUNT]; /* 当前实际脉宽，单位 us */
    Motor_State_e state;                       /* 锁定、解锁或故障 */
    Motor_Fault_e fault;                       /* 最近一次故障原因 */
    uint64_t      last_output_timestamp;       /* 最近一次有效输出时间，单位 us */
    uint8_t       initialized;                 /* 1 表示四路 PWM 均已启动 */
} Motor_Instance_t;

/* ========================== 公有接口声明 ========================== */

/**
 * @brief 初始化四路 PWM 电调
 *
 * @retval 1 初始化成功
 * @retval 0 初始化失败
 *
 * @note
 * 初始化过程：
 *
 * 1. 注册四路 PWM；
 * 2. 设置最小油门脉宽；
 * 3. 启动四路 PWM；
 * 4. 进入 DISARMED 状态。
 */
uint8_t Motor_Init(void);

/**
 * @brief 解锁电机
 *
 * @retval 1 解锁成功
 * @retval 0 当前不允许解锁
 *
 * @note
 * 调用者应当提前保证：
 *
 * 1. 遥控器有效；
 * 2. IMU 有效；
 * 3. 没有 failsafe；
 * 4. 油门位于低位。
 */
uint8_t Motor_Arm(void);

/**
 * @brief 锁定电机
 *
 * @note
 * 立即将四路电调恢复为最小脉宽。
 */
void Motor_Disarm(void);

/**
 * @brief 设置四路电机输出
 *
 * @param output 四路归一化输出，范围为 0.0～1.0
 *
 * @retval 1 输出成功
 * @retval 0 输出失败或电机未解锁
 *
 * @note
 * 本函数直接更新四路 PWM，不再需要调用
 * Motor_Task_Handler()。
 */
uint8_t Motor_SetOutput(const float output[MOTOR_COUNT]);

/**
 * @brief 电机安全超时检查
 *
 * @note
 * 建议由独立 DaemonTask 每 10～20 ms 调用一次。
 */
void Motor_SafetyCheck(void);

/**
 * @brief 紧急停止电机
 *
 * @note
 * 立即输出最小脉宽并进入 FAULT 状态。
 */
void Motor_EmergencyStop(void);

/**
 * @brief 清除电机故障
 *
 * @retval 1 清除成功，进入 DISARMED
 * @retval 0 清除失败
 */
uint8_t Motor_ResetFault(void);

/**
 * @brief 判断电机模块是否初始化成功
 */
uint8_t Motor_IsInitialized(void);

/**
 * @brief 判断电机是否已经解锁
 */
uint8_t Motor_IsArmed(void);

/**
 * @brief 获取当前电机状态
 */
Motor_State_e Motor_GetState(void);

/**
 * @brief 获取当前故障类型
 */
Motor_Fault_e Motor_GetFault(void);

/**
 * @brief 获取某一路电机当前归一化输出
 *
 * @param motor_id 电机编号
 *
 * @return 当前输出，范围为 0.0～1.0
 */
float Motor_GetOutput(Motor_ID_e motor_id);

/**
 * @brief 获取某一路电机当前 PWM 脉宽
 *
 * @param motor_id 电机编号
 *
 * @return PWM 脉宽，单位 us
 */
uint16_t Motor_GetPulseWidthUs(Motor_ID_e motor_id);

#endif // MY_NEW_UAV_BASIC_FRAMEWORK_MODULE_PWM_MOTOR_H
