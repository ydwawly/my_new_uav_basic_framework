#include "bsp_mpu.h"
#include "stm32h7xx_hal.h"

/**
 * @file bsp_mpu.c
 * @brief STM32H743 各内存域的 MPU 可执行性、缓存性和共享属性配置。
 *
 * D1 AXI SRAM 用于 CPU 高频访问并启用缓存；D2/D3 SRAM 用于 DMA 共享缓冲区，配置为
 * 不可缓存以避免显式 Cache clean/invalidate 遗漏造成数据不一致。
 */

static MPU_Region_InitTypeDef MPU_InitStruct;

void Bsp_MPU_Config(void)
{
    HAL_MPU_Disable();

    /************************************************
     * 区域 0：内部 FLASH，允许取指并启用缓存/缓冲。
     ************************************************/

    MPU_InitStruct.Enable = MPU_REGION_ENABLE;

    MPU_InitStruct.Number = MPU_REGION_NUMBER0;

    MPU_InitStruct.BaseAddress = 0x08000000;

    MPU_InitStruct.Size = MPU_REGION_SIZE_2MB;

    MPU_InitStruct.SubRegionDisable = 0;

    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;

    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;

    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;

    MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /************************************************
     * 区域 1：D1 AXI SRAM，供 CPU 使用，启用缓存/缓冲。
     ************************************************/

    MPU_InitStruct.Number = MPU_REGION_NUMBER1;

    MPU_InitStruct.BaseAddress = 0x24000000;

    MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;

    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;

    MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /************************************************
     * 区域 2：D2 SRAM1/2，DMA 共享区，不可缓存且可共享。
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
     * 区域 3：D2 SRAM3，DMA 共享区，不可缓存且可共享。
     ************************************************/

    MPU_InitStruct.Number = MPU_REGION_NUMBER3;

    MPU_InitStruct.BaseAddress = 0x30040000;

    MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;

    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;

    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /************************************************
     * 区域 4：D3 SRAM，跨总线主设备共享，不可缓存且可共享。
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
