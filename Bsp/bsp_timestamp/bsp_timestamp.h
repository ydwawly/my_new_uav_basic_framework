//
// Created by Administrator on 2026/6/8.
//

#ifndef MY_NEW_UAV_BAICE_FRAMEWORK_BSP_TIMESTAMP_H
#define MY_NEW_UAV_BAICE_FRAMEWORK_BSP_TIMESTAMP_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

// 引用外部由 CubeMX 生成的 TIM2 句柄
extern TIM_HandleTypeDef htim2;
// 引用外部系统核心频率变量 (通常在 system_stm32h7xx.c 中定义，STM32H7 通常为 240MHz、400MHz 或 480MHz)
extern uint32_t SystemCoreClock;

/**
 * @brief  初始化时间戳系统 (包括 TIM2 和 DWT)
 * @note   必须在 SystemClock_Config() 执行完毕后调用
 */
void Bsp_Timestamp_Init(void);

/**
 * @brief  获取 64位 全局微秒时间戳 (绝对时间)
 * @return 64位无符号微秒数，设备连续运行 58 万年才会溢出
 */
uint64_t Bsp_Timestamp_us_Get(void);

/**
 * @brief  获取 64位 全局毫秒时间戳 (绝对时间)
 * @return 64位无符号毫秒数
 */
uint64_t Bsp_Timestamp_ms_Get(void);

/**
 * @brief  微秒级硬件阻塞延时
 * @param  us 需要延时的微秒数
 */
void Bsp_Delay_us(uint32_t us);

/**
 * @brief  毫秒级硬件阻塞延时
 * @param  ms 需要延时的毫秒数
 */
void Bsp_Delay_ms(uint32_t ms);

/**
 * @brief  获取 DWT 当前周期的原始计数值
 * @return 32位 CPU 周期计数值 (纳秒级精度)
 */
uint32_t Bsp_DWT_Get_Cycle(void);

/**
 * @brief  获取两次调用之间的高精度时间差 (dt)
 * @param  last_cycle 传入记录上一次运行周期的变量指针，函数内部会自动更新它
 * @return 两次调用的时间差，单位为秒 (float)，专用于姿态解算和控制闭环
 */
float Bsp_DWT_Get_DeltaT(uint32_t *last_cycle);

#endif //MY_NEW_UAV_BAICE_FRAMEWORK_BSP_TIMESTAMP_H