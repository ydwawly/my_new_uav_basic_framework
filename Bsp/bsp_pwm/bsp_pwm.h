/** @file bsp_pwm.h @brief 定时器 PWM 通道注册与占空比输出接口。 */

#ifndef STM32H743_UAV_FLIGHT_CONTROLLER_BSP_PWM_H
#define STM32H743_UAV_FLIGHT_CONTROLLER_BSP_PWM_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

/* ========================== 宏定义 ========================== */

/*
 * 工程中允许注册的最大 PWM 通道数量。
 *
 * 四旋翼通常至少需要 4 个电机通道，
 * 如果还包含舵机、蜂鸣器等，可以适当增大。
 */
#define PWM_DEVICE_CNT 16U

/* ========================== 数据类型定义 ========================== */

/**
 * @brief PWM 初始化配置
 *
 * @note
 * TIM 的 Prescaler、Counter Period、PWM Mode 和 Polarity
 * 应当提前在 CubeMX 中完成配置。
 */
typedef struct
{
    /*
     * HAL 定时器句柄。
     */
    TIM_HandleTypeDef *htim;

    /*
     * PWM 通道：
     *
     * TIM_CHANNEL_1
     * TIM_CHANNEL_2
     * TIM_CHANNEL_3
     * TIM_CHANNEL_4
     */
    uint32_t channel;

    /*
     * PWM 实际周期，单位微秒。
     *
     * 例如：
     *
     * 50 Hz   -> 20000 us
     * 400 Hz  -> 2500 us
     * 500 Hz  -> 2000 us
     *
     * 此参数必须和 CubeMX 中的 PSC、ARR 配置一致。
     */
    uint32_t period_us;

} PWM_Init_Config_s;

/**
 * @brief PWM 实例
 */
typedef struct
{
    /*
     * HAL 定时器句柄。
     */
    TIM_HandleTypeDef *htim;

    /*
     * 对应的定时器 PWM 通道。
     */
    uint32_t channel;

    /*
     * PWM 周期，单位微秒。
     */
    uint32_t period_us;

    /*
     * 一个 PWM 周期对应的定时器计数值。
     *
     * period_ticks = ARR + 1
     */
    uint64_t period_ticks;

    /*
     * PWM 当前是否已经启动。
     */
    uint8_t is_started;

} PWMInstance;

/* ========================== 公有接口声明 ========================== */

/**
 * @brief 注册一个 PWM 通道
 *
 * @param init_config PWM 初始化配置
 *
 * @return 成功返回 PWM 实例指针
 * @return 失败返回 NULL
 *
 * @note
 * 注册完成后默认：
 *
 * 1. CCR 设置为 0；
 * 2. PWM 不自动启动；
 * 3. 需要上层设置安全输出后再调用 PWMStart()。
 */
PWMInstance *PWMRegister(const PWM_Init_Config_s *init_config);

/**
 * @brief 启动 PWM 输出
 *
 * @param instance PWM 实例
 *
 * @return HAL_OK    启动成功
 * @return HAL_ERROR 参数错误或启动失败
 */
HAL_StatusTypeDef PWMStart(PWMInstance *instance);

/**
 * @brief 停止 PWM 输出
 *
 * @param instance PWM 实例
 *
 * @return HAL_OK    停止成功
 * @return HAL_ERROR 参数错误或停止失败
 *
 * @note
 * 停止前会将 CCR 清零，确保重新启动时不会恢复旧输出。
 */
HAL_StatusTypeDef PWMStop(PWMInstance *instance);

/**
 * @brief 直接设置 PWM 比较值
 *
 * @param instance PWM 实例
 * @param compare  CCR 比较值
 *
 * @return HAL_OK    设置成功
 * @return HAL_ERROR 参数错误
 */
HAL_StatusTypeDef PWMSetCompare(PWMInstance *instance, uint32_t compare);

/**
 * @brief 设置 PWM 占空比
 *
 * @param instance   PWM 实例
 * @param duty_ratio 占空比，范围为 0.0f～1.0f
 *
 * @return HAL_OK    设置成功
 * @return HAL_ERROR 参数错误
 */
HAL_StatusTypeDef PWMSetDutyRatio(PWMInstance *instance, float duty_ratio);

/**
 * @brief 设置 PWM 高电平脉宽
 *
 * @param instance       PWM 实例
 * @param pulse_width_us 高电平脉宽，单位微秒
 *
 * @return HAL_OK    设置成功
 * @return HAL_ERROR 参数错误
 *
 * @example
 * 普通 PWM 电调：
 *
 * 1000 us：最小油门
 * 1500 us：中间值
 * 2000 us：最大油门
 */
HAL_StatusTypeDef PWMSetPulseWidthUs(PWMInstance *instance, uint32_t pulse_width_us);

/**
 * @brief 获取当前 CCR 比较值
 */
uint32_t PWMGetCompare(const PWMInstance *instance);

/**
 * @brief 获取当前占空比
 *
 * @return 占空比，范围为 0.0f～1.0f
 */
float PWMGetDutyRatio(const PWMInstance *instance);

/**
 * @brief 获取当前高电平脉宽
 *
 * @return 高电平脉宽，单位微秒
 */
uint32_t PWMGetPulseWidthUs(const PWMInstance *instance);

/**
 * @brief 获取 PWM 周期
 *
 * @return PWM 周期，单位微秒
 */
uint32_t PWMGetPeriodUs(const PWMInstance *instance);

/**
 * @brief 获取 PWM 频率
 *
 * @return PWM 频率，单位 Hz
 */
float PWMGetFrequencyHz(const PWMInstance *instance);

/**
 * @brief 判断 PWM 是否已经启动
 *
 * @retval 0 未启动
 * @retval 1 已启动
 */
uint8_t PWMIsStarted(const PWMInstance *instance);

/**
 * @brief 紧急停止所有已经注册的 PWM 通道
 *
 * @note
 * 适合在飞控锁定、故障或紧急停机时调用。
 */
void PWMEmergencyStopAll(void);

#endif /* STM32H743_UAV_FLIGHT_CONTROLLER_BSP_PWM_H */
