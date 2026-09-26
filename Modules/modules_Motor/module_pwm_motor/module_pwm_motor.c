/** @file module_pwm_motor.c @brief 四路电机 PWM 映射、解锁与故障保护实现。 */

#include "module_pwm_motor.h"

#include "bsp_pwm.h"
#include "bsp_RTT.h"
#include "bsp_timestamp.h"
#include "user_math.h"

#include "tim.h"

#include <stddef.h>
#include <string.h>

/* ========================== 电机硬件配置 ========================== */

/*
 * 根据实际飞控硬件修改这里。
 *
 * 当前配置：
 *
 * 左前电机：TIM1_CH4（PE14）
 * 右前电机：TIM1_CH3（PE13）
 * 右后电机：TIM1_CH2（PE11）
 * 左后电机：TIM1_CH1（PE9）
 *
 * 同一个定时器的四个通道共享 PSC 和 ARR，
 * 因此必须使用相同 PWM 周期。
 */
static const Motor_Channel_Config_t motor_channel_config[MOTOR_COUNT] = {
    {
        .htim          = &htim1,
        .channel       = TIM_CHANNEL_4,
        .period_us     = MOTOR_PWM_PERIOD_US,
        .min_pulse_us  = MOTOR_PWM_MIN_PULSE_US,
        .idle_pulse_us = MOTOR_PWM_IDLE_PULSE_US,
        .max_pulse_us  = MOTOR_PWM_MAX_PULSE_US,
    },
    {
        .htim          = &htim1,
        .channel       = TIM_CHANNEL_3,
        .period_us     = MOTOR_PWM_PERIOD_US,
        .min_pulse_us  = MOTOR_PWM_MIN_PULSE_US,
        .idle_pulse_us = MOTOR_PWM_IDLE_PULSE_US,
        .max_pulse_us  = MOTOR_PWM_MAX_PULSE_US,
    },
    {
        .htim          = &htim1,
        .channel       = TIM_CHANNEL_2,
        .period_us     = MOTOR_PWM_PERIOD_US,
        .min_pulse_us  = MOTOR_PWM_MIN_PULSE_US,
        .idle_pulse_us = MOTOR_PWM_IDLE_PULSE_US,
        .max_pulse_us  = MOTOR_PWM_MAX_PULSE_US,
    },
    {
        .htim          = &htim1,
        .channel       = TIM_CHANNEL_1,
        .period_us     = MOTOR_PWM_PERIOD_US,
        .min_pulse_us  = MOTOR_PWM_MIN_PULSE_US,
        .idle_pulse_us = MOTOR_PWM_IDLE_PULSE_US,
        .max_pulse_us  = MOTOR_PWM_MAX_PULSE_US,
    },
};

/* ========================== 全局静态实例 ========================== */

static Motor_Instance_t motor_instance;

/* ========================== 私有函数声明 ========================== */

static uint8_t Motor_IsConfigValid(const Motor_Channel_Config_t *config);

static uint16_t Motor_OutputToPulseWidth(uint8_t motor_id, float output);

static uint8_t Motor_WriteMinimumOutput(void);

static void Motor_EnterFault(Motor_Fault_e fault);

/* ========================== 私有函数实现 ========================== */

/**
 * @brief 检查单路电机配置是否合法
 */
static uint8_t Motor_IsConfigValid(const Motor_Channel_Config_t *config)
{
    if (config == NULL || config->htim == NULL)
    {
        return 0U;
    }

    if (config->period_us == 0U)
    {
        return 0U;
    }

    /*
     * 必须满足：
     *
     * min <= idle < max < period
     */
    if (config->min_pulse_us > config->idle_pulse_us)
    {
        return 0U;
    }

    if (config->idle_pulse_us >= config->max_pulse_us)
    {
        return 0U;
    }

    if (config->max_pulse_us >= config->period_us)
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 将归一化输出转换成 PWM 脉宽
 *
 * @note
 * 已解锁状态下：
 *
 * output = 0.0 -> idle_pulse_us
 * output = 1.0 -> max_pulse_us
 */
static uint16_t Motor_OutputToPulseWidth(uint8_t motor_id, float output)
{
    const Motor_Channel_Config_t *config;

    float pulse_range;
    float pulse_width;

    if (motor_id >= MOTOR_COUNT)
    {
        return MOTOR_PWM_MIN_PULSE_US;
    }

    config = &motor_channel_config[motor_id];

    output = Math_ClampFloat(output, 0.0f, 1.0f);

    pulse_range = (float)(config->max_pulse_us - config->idle_pulse_us);

    pulse_width = (float)config->idle_pulse_us + output * pulse_range;

    /*
     * 加 0.5f 实现四舍五入。
     */
    return (uint16_t)(pulse_width + 0.5f);
}

/**
 * @brief 四路电调输出最小脉宽
 */
static uint8_t Motor_WriteMinimumOutput(void)
{
    HAL_StatusTypeDef ret;

    uint8_t i;
    uint8_t result = 1U;

    for (i = 0U; i < MOTOR_COUNT; i++)
    {
        if (motor_instance.pwm_instance[i] == NULL)
        {
            result = 0U;
            continue;
        }

        ret = PWMSetPulseWidthUs(motor_instance.pwm_instance[i], motor_channel_config[i].min_pulse_us);

        if (ret != HAL_OK)
        {
            result = 0U;
            continue;
        }

        motor_instance.output[i] = 0.0f;

        motor_instance.pulse_width_us[i] = motor_channel_config[i].min_pulse_us;
    }

    return result;
}

/**
 * @brief 电机模块进入故障状态
 */
static void Motor_EnterFault(Motor_Fault_e fault)
{
    motor_instance.fault = fault;
    motor_instance.state = MOTOR_STATE_FAULT;

    /*
     * 故障发生后立即恢复最小油门。
     */
    (void)Motor_WriteMinimumOutput();
}

/* ========================== 初始化 ========================== */

uint8_t Motor_Init(void)
{
    PWM_Init_Config_s pwm_config;

    HAL_StatusTypeDef ret;

    uint8_t i;
    uint8_t started_count = 0U;

    memset(&motor_instance, 0, sizeof(Motor_Instance_t));

    motor_instance.state = MOTOR_STATE_DISARMED;

    motor_instance.fault = MOTOR_FAULT_NONE;

    /* ========== 阶段 1：注册并设置四路 PWM ========== */

    for (i = 0U; i < MOTOR_COUNT; i++)
    {
        if (Motor_IsConfigValid(&motor_channel_config[i]) == 0U)
        {
            RTTERROR("[MOTOR] Invalid config, motor=%u.", (unsigned)i);

            motor_instance.fault = MOTOR_FAULT_PWM_INIT;

            return 0U;
        }

        memset(&pwm_config, 0, sizeof(PWM_Init_Config_s));

        pwm_config.htim = motor_channel_config[i].htim;

        pwm_config.channel = motor_channel_config[i].channel;

        pwm_config.period_us = motor_channel_config[i].period_us;

        motor_instance.pwm_instance[i] = PWMRegister(&pwm_config);

        if (motor_instance.pwm_instance[i] == NULL)
        {
            RTTERROR("[MOTOR] PWM register failed, motor=%u.", (unsigned)i);

            motor_instance.fault = MOTOR_FAULT_PWM_INIT;

            return 0U;
        }

        /*
         * 必须先设置最小油门，再启动 PWM。
         */
        ret = PWMSetPulseWidthUs(motor_instance.pwm_instance[i], motor_channel_config[i].min_pulse_us);

        if (ret != HAL_OK)
        {
            RTTERROR("[MOTOR] Set minimum pulse failed, motor=%u.", (unsigned)i);

            motor_instance.fault = MOTOR_FAULT_PWM_INIT;

            return 0U;
        }

        motor_instance.output[i] = 0.0f;

        motor_instance.pulse_width_us[i] = motor_channel_config[i].min_pulse_us;
    }

    /* ========== 阶段 2：启动四路 PWM ========== */

    for (i = 0U; i < MOTOR_COUNT; i++)
    {
        ret = PWMStart(motor_instance.pwm_instance[i]);

        if (ret != HAL_OK)
        {
            RTTERROR("[MOTOR] PWM start failed, motor=%u.", (unsigned)i);

            /*
             * 停止此前已经成功启动的 PWM 通道。
             */
            while (started_count > 0U)
            {
                started_count--;

                (void)PWMStop(motor_instance.pwm_instance[started_count]);
            }

            motor_instance.fault = MOTOR_FAULT_PWM_INIT;

            return 0U;
        }

        started_count++;
    }

    /* ========== 阶段 3：初始化状态 ========== */

    motor_instance.initialized = 1U;

    motor_instance.state = MOTOR_STATE_DISARMED;

    motor_instance.fault = MOTOR_FAULT_NONE;

    motor_instance.last_output_timestamp = Bsp_Timestamp_us_Get();

    RTTINFO("[MOTOR] PWM ESC motor init success.");

    return 1U;
}

/* ========================== 解锁与锁定 ========================== */

uint8_t Motor_Arm(void)
{
    if (motor_instance.initialized == 0U)
    {
        return 0U;
    }

    /*
     * 已经解锁时直接返回成功。
     */
    if (motor_instance.state == MOTOR_STATE_ARMED)
    {
        return 1U;
    }

    /*
     * 故障状态不允许直接解锁。
     */
    if (motor_instance.state == MOTOR_STATE_FAULT || motor_instance.fault != MOTOR_FAULT_NONE)
    {
        RTTWARNING("[MOTOR] Arm rejected: fault=%u.", (unsigned)motor_instance.fault);

        return 0U;
    }

    /*
     * 解锁之前再次确保四路输出为最小脉宽。
     */
    if (Motor_WriteMinimumOutput() == 0U)
    {
        Motor_EnterFault(MOTOR_FAULT_PWM_WRITE);

        return 0U;
    }

    motor_instance.last_output_timestamp = Bsp_Timestamp_us_Get();

    /*
     * 简化版本不再经过 ARMING，
     * 直接进入 ARMED。
     */
    motor_instance.state = MOTOR_STATE_ARMED;

    RTTINFO("[MOTOR] Motor armed.");

    return 1U;
}

void Motor_Disarm(void)
{
    Motor_State_e previous_state;

    if (motor_instance.initialized == 0U)
    {
        return;
    }

    previous_state = motor_instance.state;

    /*
     * 立即恢复最小输出。
     */
    (void)Motor_WriteMinimumOutput();

    /*
     * 普通锁定操作不清除已有故障。
     */
    if (motor_instance.state != MOTOR_STATE_FAULT)
    {
        motor_instance.state = MOTOR_STATE_DISARMED;
    }

    /*
     * 只有从 ARMED 切换到 DISARMED 时打印日志，
     * 避免控制任务重复打印。
     */
    if (previous_state == MOTOR_STATE_ARMED)
    {
        RTTINFO("[MOTOR] Motor disarmed.");
    }
}

/* ========================== 电机输出 ========================== */

uint8_t Motor_SetOutput(const float output[MOTOR_COUNT])
{
    float limited_output[MOTOR_COUNT];

    uint16_t pulse_width_us[MOTOR_COUNT];

    HAL_StatusTypeDef ret;

    uint8_t i;

    if (motor_instance.initialized == 0U || output == NULL)
    {
        return 0U;
    }

    /*
     * 未解锁时禁止输出。
     */
    if (motor_instance.state != MOTOR_STATE_ARMED)
    {
        return 0U;
    }

    /* ========== 阶段 1：检查并计算整帧输出 ========== */

    /*
     * 先检查完整的四路命令，
     * 防止只更新一部分电机。
     */
    for (i = 0U; i < MOTOR_COUNT; i++)
    {
        if (!Math_IsFinite(output[i]))
        {
            RTTERROR("[MOTOR] Invalid output, motor=%u.", (unsigned)i);

            Motor_EnterFault(MOTOR_FAULT_INVALID_COMMAND);

            return 0U;
        }

        limited_output[i] = Math_ClampFloat(output[i], 0.0f, 1.0f);

        pulse_width_us[i] = Motor_OutputToPulseWidth(i, limited_output[i]);
    }

    /* ========== 阶段 2：写入四路 PWM ========== */

    for (i = 0U; i < MOTOR_COUNT; i++)
    {
        ret = PWMSetPulseWidthUs(motor_instance.pwm_instance[i], pulse_width_us[i]);

        if (ret != HAL_OK)
        {
            RTTERROR("[MOTOR] PWM write failed, motor=%u.", (unsigned)i);

            /*
             * 如果任意一路写入失败，
             * 所有电机立即恢复最小输出。
             */
            Motor_EnterFault(MOTOR_FAULT_PWM_WRITE);

            return 0U;
        }
    }

    /* ========== 阶段 3：更新运行状态 ========== */

    memcpy(motor_instance.output, limited_output, sizeof(limited_output));

    memcpy(motor_instance.pulse_width_us, pulse_width_us, sizeof(pulse_width_us));

    motor_instance.last_output_timestamp = Bsp_Timestamp_us_Get();

    return 1U;
}

/* ========================== 安全处理 ========================== */

void Motor_SafetyCheck(void)
{
    uint64_t current_timestamp;

    if (motor_instance.initialized == 0U)
    {
        return;
    }

    /*
     * 只有 ARMED 状态才需要检查输出刷新超时。
     */
    if (motor_instance.state != MOTOR_STATE_ARMED)
    {
        return;
    }

    current_timestamp = Bsp_Timestamp_us_Get();

    if ((current_timestamp - motor_instance.last_output_timestamp) > MOTOR_OUTPUT_TIMEOUT_US)
    {
        RTTERROR("[MOTOR] Output timeout: %llu us.",
                 (unsigned long long)(current_timestamp - motor_instance.last_output_timestamp));

        Motor_EnterFault(MOTOR_FAULT_OUTPUT_TIMEOUT);
    }
}

void Motor_EmergencyStop(void)
{
    /*
     * 如果模块尚未完成初始化，
     * 尝试停止所有 BSP PWM。
     */
    if (motor_instance.initialized == 0U)
    {
        PWMEmergencyStopAll();
        return;
    }

    Motor_EnterFault(MOTOR_FAULT_EMERGENCY_STOP);

    RTTERROR("[MOTOR] Emergency stop!");
}

uint8_t Motor_ResetFault(void)
{
    if (motor_instance.initialized == 0U)
    {
        return 0U;
    }

    if (motor_instance.state != MOTOR_STATE_FAULT)
    {
        return 0U;
    }

    /*
     * 先恢复最小油门。
     */
    if (Motor_WriteMinimumOutput() == 0U)
    {
        return 0U;
    }

    motor_instance.fault = MOTOR_FAULT_NONE;

    motor_instance.state = MOTOR_STATE_DISARMED;

    motor_instance.last_output_timestamp = Bsp_Timestamp_us_Get();

    RTTINFO("[MOTOR] Fault reset success.");

    return 1U;
}

/* ========================== 状态获取 ========================== */

uint8_t Motor_IsInitialized(void)
{
    return motor_instance.initialized;
}

uint8_t Motor_IsArmed(void)
{
    return (motor_instance.state == MOTOR_STATE_ARMED) ? 1U : 0U;
}

Motor_State_e Motor_GetState(void)
{
    return motor_instance.state;
}

Motor_Fault_e Motor_GetFault(void)
{
    return motor_instance.fault;
}

float Motor_GetOutput(Motor_ID_e motor_id)
{
    if ((uint8_t)motor_id >= MOTOR_COUNT)
    {
        return 0.0f;
    }

    return motor_instance.output[motor_id];
}

uint16_t Motor_GetPulseWidthUs(Motor_ID_e motor_id)
{
    if ((uint8_t)motor_id >= MOTOR_COUNT)
    {
        return 0U;
    }

    return motor_instance.pulse_width_us[motor_id];
}
