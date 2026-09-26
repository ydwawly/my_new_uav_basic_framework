/** @file PID.h @brief 通用 PID 控制器配置、状态与计算接口。 */

#ifndef STM32H743_UAV_FLIGHT_CONTROLLER_PID_H
#define STM32H743_UAV_FLIGHT_CONTROLLER_PID_H

#include "stdint.h"
#include <math.h>
#include "bsp_timestamp.h"

// 如果你的项目中包含了 bsp_dwt.h，请保留；否则请确保有 DWT_GetDeltaT 的定义

// PID 优化环节使能标志位
typedef enum
{
    PID_IMPROVE_NONE                = 0b00000000,
    PID_Integral_Limit              = 0b00000001, // 积分限幅
    PID_Derivative_On_Measurement   = 0b00000010, // 微分先行
    PID_Trapezoid_Integral          = 0b00000100, // 梯形积分 (拼写修正)
    PID_Proportional_On_Measurement = 0b00001000, // 比例先行 (预留，暂未实现)
    PID_OutputFilter                = 0b00010000, // 输出滤波
    PID_ChangingIntegrationRate     = 0b00100000, // 变速积分
    PID_DerivativeFilter            = 0b01000000, // 微分滤波
} PID_Improvement_e;

/* PID 报错类型枚举 */
typedef enum
{
    PID_ERROR_NONE          = 0x00U,
    PID_MOTOR_BLOCKED_ERROR = 0x01U
} ErrorType_e;

typedef struct
{
    uint64_t    ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;

/* 用于PID初始化的结构体 (配置参数) */
typedef struct
{
    // --- 基础参数 ---
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;   // 输出限幅
    float DeadBand; // 死区

    // --- 优化参数 ---
    PID_Improvement_e Improve;
    float             IntegralLimit;     // 积分限幅
    float             CoefA;             // 变速积分参数A
    float             CoefB;             // 变速积分参数B: ITerm = Err*((A-abs(err)+B)/A)
    float             Output_LPF_RC;     // 输出滤波器系数 RC
    float             Derivative_LPF_RC; // 微分滤波器系数 RC

} PID_Init_Config_s;

/* PID 运行实例结构体 */
typedef struct
{
    // 拷贝自 Config 的参数
    float             Kp;
    float             Ki;
    float             Kd;
    float             MaxOut;
    float             DeadBand;
    PID_Improvement_e Improve;
    float             IntegralLimit;
    float             CoefA;
    float             CoefB;
    float             Output_LPF_RC;
    float             Derivative_LPF_RC;

    // 堵转检测参数
    float    Block_OutputThres;
    float    Block_RefThres;
    float    Block_DiffThres;
    uint16_t Block_TimeThres;

    // 运行时变量
    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;
    float Err_rate;

    float Pout;
    float Iout;
    float Dout;
    float ITerm;

    float Output;
    float Last_Output;
    float Last_Dout;

    float Ref;

    uint32_t DWT_CNT;
    float    dt;

    PID_ErrorHandler_t ERRORHandler;
} PIDInstance;

/**
 * @brief 初始化PID实例
 * @param pid    PID实例指针
 * @param config PID初始化配置
 */
void PIDInit(PIDInstance *pid, const PID_Init_Config_s *config);

/**
 * @brief 计算PID输出
 * @param pid     PID实例指针
 * @param measure 反馈值
 * @param ref     设定值
 * @return float  PID计算输出
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref);

#endif /* STM32H743_UAV_FLIGHT_CONTROLLER_PID_H */
