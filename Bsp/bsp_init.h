//
// Created by Administrator on 2026/6/9.
//

#ifndef MY_NEW_UAV_BAICE_FRAMEWORK_BSP_INIT_H
#define MY_NEW_UAV_BAICE_FRAMEWORK_BSP_INIT_H
#include "bsp_RTT.h"
#include "bsp_timestamp.h"

void Bsp_Init(void)
{
    Bsp_Timestamp_Init();
    Bsp_RTT_Init();
}

#endif //MY_NEW_UAV_BAICE_FRAMEWORK_BSP_INIT_H