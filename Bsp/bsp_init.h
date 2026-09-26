#ifndef MY_NEW_UAV_BAICE_FRAMEWORK_BSP_INIT_H
#define MY_NEW_UAV_BAICE_FRAMEWORK_BSP_INIT_H

/**
 * @file bsp_init.h
 * @brief 板级基础设施的早期初始化入口。
 */

#include "bsp_RTT.h"
#include "bsp_timestamp.h"

/** @brief 先启动单调时间戳，再初始化 RTT 调试输出。 */
void Bsp_Init(void)
{
    Bsp_Timestamp_Init();
    Bsp_RTT_Init();
}

#endif /* MY_NEW_UAV_BAICE_FRAMEWORK_BSP_INIT_H */
