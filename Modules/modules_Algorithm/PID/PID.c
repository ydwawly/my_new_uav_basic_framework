/** @file PID.c @brief 支持积分限幅等改进项的 PID 控制器实现。 */
/**
 * @file pid.c
 * @brief PID控制器实现 (优化版)
 */
#include "PID.h"
#include <string.h> // for memset

/* ---------------------------- 内部优化函数 ---------------------------- */
//梯形积分
static void f_Trapezoid_Integral(PIDInstance *pid)
{
    // 计算梯形的面积,(上底+下底)*高/2
    pid->ITerm = pid->Ki * (pid->Err + pid->Last_Err) * pid->dt / 2.0f;
}

// 变速积分 (误差小时积分作用更强)
static void f_Changing_Integration_Rate(PIDInstance *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        // 积分呈累积趋势
        if (fabsf(pid->Err) <= pid->CoefB)
            return; // Full integral
        if (fabsf(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - fabsf(pid->Err) + pid->CoefB) / pid->CoefA;
        else // 最大阈值,不使用积分
            pid->ITerm = 0;
    }
}

// 积分限幅 & 抗饱和
static void f_Integral_Limit(PIDInstance *pid)
{
    float temp_Output, temp_Iout;
    temp_Iout   = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;

    // 抗饱和：如果输出饱和且积分在加剧饱和，则停止积分
    if (fabsf(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0)
        {
            pid->ITerm = 0;
        }
    }

    // 强制限幅
    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout  = pid->IntegralLimit;
    }
    else if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout  = -pid->IntegralLimit;
    }
}

// 微分先行 (仅使用反馈值微分)
static void f_Derivative_On_Measurement(PIDInstance *pid)
{
    // KD * -d(Measure)/dt
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}

// 微分滤波 (低通滤波)
static void f_Derivative_Filter(PIDInstance *pid)
{
    pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
                pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}

// 输出滤波 (低通滤波)
static void f_Output_Filter(PIDInstance *pid)
{
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                  pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

// 输出限幅
static void f_Output_Limit(PIDInstance *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    else if (pid->Output < -pid->MaxOut)
    {
        pid->Output = -pid->MaxOut;
    }
}

/* --------------------------- PID 外部接口 --------------------------- */

void PIDInit(PIDInstance *pid, const PID_Init_Config_s *config)
{
    // 1. 清空所有运行状态
    memset(pid, 0, sizeof(PIDInstance));

    // 2. 显式赋值配置参数 (替代 memcpy，更安全)
    pid->Kp                = config->Kp;
    pid->Ki                = config->Ki;
    pid->Kd                = config->Kd;
    pid->MaxOut            = config->MaxOut;
    pid->DeadBand          = config->DeadBand;
    pid->Improve           = config->Improve;
    pid->IntegralLimit     = config->IntegralLimit;
    pid->CoefA             = config->CoefA;
    pid->CoefB             = config->CoefB;
    pid->Output_LPF_RC     = config->Output_LPF_RC;
    pid->Derivative_LPF_RC = config->Derivative_LPF_RC;

    // 4. 初始化时间戳
    Bsp_DWT_Get_DeltaT(&pid->DWT_CNT);
}

float PIDCalculate(PIDInstance *pid, float measure, float ref)
{
    // 1. 计算时间间隔 dt
    pid->dt = Bsp_DWT_Get_DeltaT(&pid->DWT_CNT);

    // [Safety] 防止除以零或时间间隔过小导致浮点异常
    if (pid->dt < 1e-6f)
    {
        return pid->Last_Output;
    }

    // 2. 更新基础数据
    pid->Measure = measure;
    pid->Ref     = ref;
    pid->Err     = pid->Ref - pid->Measure;
    // 这里也可以加一个 dt 检查，或者直接使用计算好的 dt
    pid->Err_rate = (pid->Err - pid->Last_Err) / pid->dt;

    // 3. 死区判断
    if (fabsf(pid->Err) > pid->DeadBand)
    {
        // 3.1 P 项计算
        pid->Pout = pid->Kp * pid->Err;

        // 3.2 I 项计算
        if (pid->Improve & PID_Trapezoid_Integral)
            f_Trapezoid_Integral(pid); // 梯形积分
        else
            pid->ITerm = pid->Ki * pid->Err * pid->dt; // 标准积分

        // 变速积分处理
        if (pid->Improve & PID_ChangingIntegrationRate)
            f_Changing_Integration_Rate(pid);

        // 积分限幅
        if (pid->Improve & PID_Integral_Limit)
            f_Integral_Limit(pid);

        // 累加积分输出
        pid->Iout += pid->ITerm;

        // 3.3 D 项计算
        if (pid->Improve & PID_Derivative_On_Measurement)
        {
            f_Derivative_On_Measurement(pid); // 微分先行
        }
        else
        {
            pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt; // 标准微分
        }

        // 微分滤波
        if (pid->Improve & PID_DerivativeFilter)
            f_Derivative_Filter(pid);

        // 4. 总输出计算
        pid->Output = pid->Pout + pid->Iout + pid->Dout;

        // 输出滤波
        if (pid->Improve & PID_OutputFilter)
            f_Output_Filter(pid);

        // 输出限幅
        f_Output_Limit(pid);
    }
    else // 进入死区
    {
        pid->Output = 0;
        pid->ITerm  = 0;
        // 注意：进入死区是否清除 Iout 取决于具体需求，通常保持不变或清零
        // 原代码是清零积分增量，这里我们简单处理为输出清零
    }

    // 5. 保存状态供下次使用
    pid->Last_Measure = pid->Measure;
    pid->Last_Output  = pid->Output;
    pid->Last_Dout    = pid->Dout;
    pid->Last_Err     = pid->Err;
    // pid->Last_ITerm 未被实际使用，可以保留或移除

    return pid->Output;
}
