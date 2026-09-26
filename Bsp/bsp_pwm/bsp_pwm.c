/** @file bsp_pwm.c @brief 定时器 PWM 通道管理实现。 */

#include "bsp_pwm.h"

#include "FreeRTOS.h"
#include "bsp_RTT.h"

#include <stddef.h>
#include <string.h>

/* ========================== 私有变量 ========================== */

/*
 * 已注册的 PWM 实例数量。
 */
static uint8_t pwm_count = 0U;

/*
 * PWM 实例表。
 */
static PWMInstance *pwm_instance[PWM_DEVICE_CNT] = {NULL};

/* ========================== 私有函数声明 ========================== */

static uint8_t      PWMIsChannelValid(uint32_t channel);
static uint8_t      PWMIsSameTimerPeriodValid(const PWM_Init_Config_s *init_config);
static uint8_t      PWMRegisterConfigIsValid(const PWM_Init_Config_s *init_config);
static PWMInstance *PWMFindRegisteredChannel(const PWM_Init_Config_s *init_config);
static PWMInstance *PWMCreateInstance(const PWM_Init_Config_s *init_config);
static uint32_t     PWMCalculateCompareFromDuty(const PWMInstance *instance, float duty_ratio);
static uint32_t     PWMCalculateCompareFromPulse(const PWMInstance *instance, uint32_t pulse_width_us);

/* ========================== 私有函数实现 ========================== */

/**
 * @brief 检查 PWM 通道是否合法
 */
static uint8_t PWMIsChannelValid(uint32_t channel)
{
    switch (channel)
    {
    case TIM_CHANNEL_1:
    case TIM_CHANNEL_2:
    case TIM_CHANNEL_3:
    case TIM_CHANNEL_4:
        return 1U;

#if defined(TIM_CHANNEL_5)
    case TIM_CHANNEL_5:
        return 1U;
#endif

#if defined(TIM_CHANNEL_6)
    case TIM_CHANNEL_6:
        return 1U;
#endif

    default:
        return 0U;
    }
}

/**
 * @brief 检查同一个定时器下所有通道的周期是否一致
 *
 * @note
 * 同一个 TIM 的多个通道共享：
 *
 * Prescaler
 * ARR
 * 计数频率
 * PWM 周期
 *
 * 因此同一个 TIM 的不同通道不能具有不同 PWM 周期。
 */
static uint8_t PWMIsSameTimerPeriodValid(const PWM_Init_Config_s *init_config)
{
    uint8_t i;

    for (i = 0U; i < pwm_count; i++)
    {
        if (pwm_instance[i]->htim != init_config->htim)
        {
            continue;
        }

        if (pwm_instance[i]->period_us != init_config->period_us)
        {
            return 0U;
        }
    }

    return 1U;
}

/**
 * @brief 根据占空比计算 CCR
 */
static uint32_t PWMCalculateCompareFromDuty(const PWMInstance *instance, float duty_ratio)
{
    uint64_t compare;
    uint32_t auto_reload;

    auto_reload = __HAL_TIM_GET_AUTORELOAD(instance->htim);

    /*
     * compare = duty × (ARR + 1)
     */
    compare = (uint64_t)(duty_ratio * (float)instance->period_ticks + 0.5f);

    /*
     * CCR 对于普通 PWM 输出最大设置到 ARR。
     *
     * 这意味着 duty=1.0 时输出会非常接近 100%，
     * 避免在 16 位定时器中写入 ARR+1 发生溢出。
     */
    if (compare > auto_reload)
    {
        compare = auto_reload;
    }

    return (uint32_t)compare;
}

/**
 * @brief 根据微秒脉宽计算 CCR
 */
static uint32_t PWMCalculateCompareFromPulse(const PWMInstance *instance, uint32_t pulse_width_us)
{
    uint64_t compare;
    uint32_t auto_reload;

    auto_reload = __HAL_TIM_GET_AUTORELOAD(instance->htim);

    /*
     *             pulse_width_us
     * CCR = ---------------------------- × (ARR + 1)
     *              period_us
     */
    compare = (uint64_t)pulse_width_us * instance->period_ticks / instance->period_us;

    if (compare > auto_reload)
    {
        compare = auto_reload;
    }

    return (uint32_t)compare;
}

/**
 * @brief 检查PWM注册参数和定时器基础配置
 */
static uint8_t PWMRegisterConfigIsValid(const PWM_Init_Config_s *init_config)
{
    if ((init_config == NULL) || (init_config->htim == NULL))
    {
        RTTERROR("[bsp_pwm] Register failed: NULL config or timer handle.");
        return 0U;
    }

    if ((PWMIsChannelValid(init_config->channel) == 0U) || (init_config->period_us == 0U))
    {
        RTTERROR("[bsp_pwm] Register failed: invalid channel or zero period.");
        return 0U;
    }

    if (pwm_count >= PWM_DEVICE_CNT)
    {
        RTTERROR("[bsp_pwm] Register failed: max instance count reached.");
        configASSERT(0);
        return 0U;
    }

    if (PWMIsSameTimerPeriodValid(init_config) == 0U)
    {
        RTTERROR("[bsp_pwm] Register failed: channels on the same timer must use the same period.");
        configASSERT(0);
        return 0U;
    }

    if (__HAL_TIM_GET_AUTORELOAD(init_config->htim) == 0U)
    {
        RTTERROR("[bsp_pwm] Register failed: timer ARR is zero.");
        return 0U;
    }
    return 1U;
}

static PWMInstance *PWMFindRegisteredChannel(const PWM_Init_Config_s *init_config)
{
    for (uint8_t i = 0U; i < pwm_count; i++)
    {
        if ((pwm_instance[i]->htim == init_config->htim) && (pwm_instance[i]->channel == init_config->channel))
        {
            return pwm_instance[i];
        }
    }
    return NULL;
}

static PWMInstance *PWMCreateInstance(const PWM_Init_Config_s *init_config)
{
    PWMInstance *instance = (PWMInstance *)pvPortMalloc(sizeof(PWMInstance));
    if (instance == NULL)
    {
        RTTERROR("[bsp_pwm] Register failed: memory allocation failed.");
        configASSERT(0);
        return NULL;
    }

    memset(instance, 0, sizeof(*instance));
    instance->htim         = init_config->htim;
    instance->channel      = init_config->channel;
    instance->period_us    = init_config->period_us;
    instance->period_ticks = (uint64_t)__HAL_TIM_GET_AUTORELOAD(init_config->htim) + 1ULL;

    /* 注册时先清零CCR，避免后续启动PWM时输出未知的旧比较值。 */
    __HAL_TIM_SET_COMPARE(instance->htim, instance->channel, 0U);
    return instance;
}

/* ========================== 公有接口实现 ========================== */

/**
 * @brief 注册一个 PWM 实例
 */
PWMInstance *PWMRegister(const PWM_Init_Config_s *init_config)
{
    if (!PWMRegisterConfigIsValid(init_config))
    {
        return NULL;
    }

    PWMInstance *instance = PWMFindRegisteredChannel(init_config);
    if (instance != NULL)
    {
        RTTWARNING("[bsp_pwm] Timer channel already registered.");
        return instance;
    }

    instance = PWMCreateInstance(init_config);
    if (instance == NULL)
    {
        return NULL;
    }
    pwm_instance[pwm_count++] = instance;
    return instance;
}

HAL_StatusTypeDef PWMStart(PWMInstance *instance)
{
    HAL_StatusTypeDef ret;

    if (instance == NULL || instance->htim == NULL)
    {
        return HAL_ERROR;
    }

    if (instance->is_started != 0U)
    {
        return HAL_OK;
    }

    ret = HAL_TIM_PWM_Start(instance->htim, instance->channel);

    if (ret == HAL_OK)
    {
        instance->is_started = 1U;
    }
    else
    {
        RTTERROR("[bsp_pwm] PWMStart failed, HAL ret=%d.", (int)ret);
    }

    return ret;
}

HAL_StatusTypeDef PWMStop(PWMInstance *instance)
{
    HAL_StatusTypeDef ret;

    if (instance == NULL || instance->htim == NULL)
    {
        return HAL_ERROR;
    }

    /*
     * 先清空输出命令，再停止通道。
     *
     * 避免下次重新启动后恢复旧的电机输出。
     */
    __HAL_TIM_SET_COMPARE(instance->htim, instance->channel, 0U);

    if (instance->is_started == 0U)
    {
        return HAL_OK;
    }

    ret = HAL_TIM_PWM_Stop(instance->htim, instance->channel);

    if (ret == HAL_OK)
    {
        instance->is_started = 0U;
    }
    else
    {
        RTTERROR("[bsp_pwm] PWMStop failed, HAL ret=%d.", (int)ret);
    }

    return ret;
}

HAL_StatusTypeDef PWMSetCompare(PWMInstance *instance, uint32_t compare)
{
    uint32_t auto_reload;

    if (instance == NULL || instance->htim == NULL)
    {
        return HAL_ERROR;
    }

    auto_reload = __HAL_TIM_GET_AUTORELOAD(instance->htim);

    if (compare > auto_reload)
    {
        RTTWARNING("[bsp_pwm] Compare value out of range: "
                   "compare=%lu, ARR=%lu.",
                   (unsigned long)compare, (unsigned long)auto_reload);

        return HAL_ERROR;
    }

    __HAL_TIM_SET_COMPARE(instance->htim, instance->channel, compare);

    return HAL_OK;
}

HAL_StatusTypeDef PWMSetDutyRatio(PWMInstance *instance, float duty_ratio)
{
    uint32_t compare;

    if (instance == NULL || instance->htim == NULL)
    {
        return HAL_ERROR;
    }

    if (duty_ratio < 0.0f || duty_ratio > 1.0f)
    {
        RTTWARNING("[bsp_pwm] Invalid duty ratio: %.4f.", (double)duty_ratio);

        return HAL_ERROR;
    }

    compare = PWMCalculateCompareFromDuty(instance, duty_ratio);

    __HAL_TIM_SET_COMPARE(instance->htim, instance->channel, compare);

    return HAL_OK;
}

HAL_StatusTypeDef PWMSetPulseWidthUs(PWMInstance *instance, uint32_t pulse_width_us)
{
    uint32_t compare;

    if (instance == NULL || instance->htim == NULL || instance->period_us == 0U)
    {
        return HAL_ERROR;
    }

    if (pulse_width_us > instance->period_us)
    {
        RTTWARNING("[bsp_pwm] Pulse width out of range: "
                   "pulse=%lu us, period=%lu us.",
                   (unsigned long)pulse_width_us, (unsigned long)instance->period_us);

        return HAL_ERROR;
    }

    compare = PWMCalculateCompareFromPulse(instance, pulse_width_us);

    __HAL_TIM_SET_COMPARE(instance->htim, instance->channel, compare);

    return HAL_OK;
}

uint32_t PWMGetCompare(const PWMInstance *instance)
{
    if (instance == NULL || instance->htim == NULL)
    {
        return 0U;
    }

    return __HAL_TIM_GET_COMPARE(instance->htim, instance->channel);
}

float PWMGetDutyRatio(const PWMInstance *instance)
{
    uint32_t compare;

    if (instance == NULL || instance->htim == NULL || instance->period_ticks == 0ULL)
    {
        return 0.0f;
    }

    compare = PWMGetCompare(instance);

    return (float)compare / (float)instance->period_ticks;
}

uint32_t PWMGetPulseWidthUs(const PWMInstance *instance)
{
    uint64_t pulse_width_us;
    uint32_t compare;

    if (instance == NULL || instance->htim == NULL || instance->period_ticks == 0ULL)
    {
        return 0U;
    }

    compare = PWMGetCompare(instance);

    pulse_width_us = (uint64_t)compare * instance->period_us / instance->period_ticks;

    return (uint32_t)pulse_width_us;
}

uint32_t PWMGetPeriodUs(const PWMInstance *instance)
{
    if (instance == NULL)
    {
        return 0U;
    }

    return instance->period_us;
}

float PWMGetFrequencyHz(const PWMInstance *instance)
{
    if (instance == NULL || instance->period_us == 0U)
    {
        return 0.0f;
    }

    return 1000000.0f / (float)instance->period_us;
}

uint8_t PWMIsStarted(const PWMInstance *instance)
{
    if (instance == NULL)
    {
        return 0U;
    }

    return instance->is_started;
}

void PWMEmergencyStopAll(void)
{
    uint8_t i;

    /*
     * 第一阶段：先把所有 CCR 清零。
     *
     * 尽可能快地消除所有 PWM 有效输出。
     */
    for (i = 0U; i < pwm_count; i++)
    {
        if (pwm_instance[i] == NULL || pwm_instance[i]->htim == NULL)
        {
            continue;
        }

        __HAL_TIM_SET_COMPARE(pwm_instance[i]->htim, pwm_instance[i]->channel, 0U);
    }

    /*
     * 第二阶段：停止各 PWM 通道。
     */
    for (i = 0U; i < pwm_count; i++)
    {
        if (pwm_instance[i] == NULL || pwm_instance[i]->htim == NULL)
        {
            continue;
        }

        if (pwm_instance[i]->is_started == 0U)
        {
            continue;
        }

        (void)HAL_TIM_PWM_Stop(pwm_instance[i]->htim, pwm_instance[i]->channel);

        pwm_instance[i]->is_started = 0U;
    }
}
