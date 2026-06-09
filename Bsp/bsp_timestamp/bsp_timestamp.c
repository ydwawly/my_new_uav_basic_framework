//
// Created by Administrator on 2026/6/8.
//

#include "bsp_timestamp.h"

// 记录 TIM2 (32位定时器) 溢出的次数，作为 64 位时间戳的高 32 位
// 使用 volatile 修饰是因为它在中断中被修改，防止编译器对其进行缓存优化
static volatile uint32_t time_overflow_count = 0;

void Bsp_Timestamp_Init(void)
{
    // 1. 启动 TIM2 并开启溢出中断 (大约每 71.5 分钟溢出一次)
    HAL_TIM_Base_Start_IT(&htim2);

    // 2. 初始化 Cortex-M 内核的 DWT (Data Watchpoint and Trace) 外设
    // 使能 DWT 模块
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // 【关键】对于 Cortex-M7 内核 (STM32H7)，必须解锁 LAR (Lock Access Register) 才能使用 DWT
#if defined(__CORTEX_M) && (__CORTEX_M == 7U)
    DWT->LAR = 0xC5ACCE55;
#endif

    // 清零周期计数器
    DWT->CYCCNT = 0;
    // 启动周期计数器，使其开始随 CPU 主频同步计数
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint64_t Bsp_Timestamp_us_Get(void)
{
    uint32_t high1, high2, low;

    // 使用无锁(Lock-free)双重读取机制，防止高低位拼接时发生读写冲突 (Race Condition)
    // 确保在读取低 32 位 (TIM2->CNT) 的瞬间，高 32 位没有因为中断而发生改变
    do
    {
        high1 = time_overflow_count;
        low = TIM2->CNT;              // 直接读取底层寄存器，速度极快
        high2 = time_overflow_count;
    } while (high1 != high2);         // 如果前后高位不一致，说明读取期间被溢出中断打断，重读

    // 将高 32 位左移，并与低 32 位进行按位或操作，拼接成完整的 64 位时间戳
    return ((uint64_t)high1 << 32) | low;
}


uint64_t Bsp_Timestamp_ms_Get(void)
{
    // 直接复用 us 级别函数，除以 1000
    // 注意：整数除法比较耗时，仅在需要 ms 级时间戳的低频任务中使用
    return Bsp_Timestamp_us_Get() / 1000ULL;
}


void Bsp_Delay_us(uint32_t us)
{
    uint64_t start = Bsp_Timestamp_us_Get();
    // 利用当前时间减去起始时间，与目标延时进行比较
    // 由于使用的是无符号 64 位整型，因此不存在溢出导致逻辑错误的风险
    while ((Bsp_Timestamp_us_Get() - start) < (uint64_t)us)
    {
        __NOP(); // 插入汇编空指令(No Operation)，防止编译器将死循环过度优化掉
    }
}

void Bsp_Delay_ms(uint32_t ms)
{
    Bsp_Delay_us(ms * 1000);
}


uint32_t Bsp_DWT_Get_Cycle(void)
{
    // 直接读取 DWT 寄存器，耗时仅 1 个机器周期
    return DWT->CYCCNT;
}

float Bsp_DWT_Get_DeltaT(uint32_t *last_cycle)
{
    uint32_t current_cycle = DWT->CYCCNT;
    uint32_t delta_cycles;

    // 利用 32 位无符号整型相减的自然回绕特性，安全处理 DWT 大约每 10.7 秒溢出一次的情况
    delta_cycles = current_cycle - *last_cycle;

    // 更新上一次的周期计数值，为下一次计算做准备
    *last_cycle = current_cycle;

    // 将经历的周期数除以系统主频 (如 240,000,000)，得到精确到纳秒级的浮点型秒数
    return (float)delta_cycles / (float)SystemCoreClock;
}

void Bsp_Timestamp_Update(void)
{
    time_overflow_count++; // 溢出次数加 1，即 64位时间戳的高 32 位进位
}