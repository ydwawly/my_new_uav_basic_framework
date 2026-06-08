//
// Created by Administrator on 2026/6/8.
//

#include "bsp_mpu.h"
#include "stm32h7xx_hal.h"

static MPU_Region_InitTypeDef MPU_InitStruct;

void Bsp_MPU_Config(void)
{
    HAL_MPU_Disable();

    /************************************************
     * Region0
     * FLASH
     ************************************************/

    MPU_InitStruct.Enable = MPU_REGION_ENABLE;

    MPU_InitStruct.Number = MPU_REGION_NUMBER0;

    MPU_InitStruct.BaseAddress = 0x08000000;

    MPU_InitStruct.Size = MPU_REGION_SIZE_2MB;

    MPU_InitStruct.SubRegionDisable = 0;

    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;

    MPU_InitStruct.AccessPermission =
        MPU_REGION_FULL_ACCESS;

    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;

    MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /************************************************
     * Region1
     * AXI SRAM D1
     ************************************************/

    MPU_InitStruct.Number = MPU_REGION_NUMBER1;

    MPU_InitStruct.BaseAddress = 0x24000000;

    MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;

    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;

    MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /************************************************
     * Region2
     * D2 SRAM
     * DMA Buffer
     ************************************************/

    MPU_InitStruct.Number = MPU_REGION_NUMBER2;

    MPU_InitStruct.BaseAddress = 0x30000000;

    MPU_InitStruct.Size = MPU_REGION_SIZE_256KB;

    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;

    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;

    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /************************************************
     * Region3
     * D2 SRAM3
     ************************************************/

    MPU_InitStruct.Number = MPU_REGION_NUMBER3;

    MPU_InitStruct.BaseAddress = 0x30040000;

    MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;

    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;

    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /************************************************
     * Region4
     * D3 SRAM
     ************************************************/

    MPU_InitStruct.Number = MPU_REGION_NUMBER4;

    MPU_InitStruct.BaseAddress = 0x38000000;

    MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;

    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;

    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}