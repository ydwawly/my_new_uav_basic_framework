/** @file bsp_memory_section.h @brief DMA 内存池与缓存一致性辅助接口。 */

#ifndef STM32H743_UAV_FLIGHT_CONTROLLER_BSP_MEMORY_SECTION_H
#define STM32H743_UAV_FLIGHT_CONTROLLER_BSP_MEMORY_SECTION_H

#include <stddef.h>
#include <stdint.h>

#include "bsp_RTT.h"

/* ========================== 内存池配置 ========================== */

// #define BSP_DMA_MEM_POOL_SIZE 4096U
#define BSP_DMA_MEM_POOL_SIZE (64U * 1024U)

/* ========================== 链接段属性 ========================== */

/* 高频执行代码放入 ITCM。 */
#define ITCM_CODE __attribute__((section(".itcm")))

/* 仅由 CPU 访问的高速数据放入 DTCM，DMA 外设不能访问该区域。 */
#define DTCM_DATA __attribute__((section(".dtcm")))

/* 普通 DMA 缓冲区放入非缓存 D2 SRAM，并按 Cortex-M7 Cache Line 对齐。 */
#define DMA_BUFFER __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)))

/*
 * SDMMC1 使用外设内部 IDMA，STM32H743 上必须把传输缓冲区放在 AXI SRAM
 * （D1）中。D1 为 Write-back Cache 区域，因此 sd_diskio 会在每次传输前后
 * 显式执行 32 字节粒度的 D-Cache clean/invalidate。
 */
#define SDMMC_DMA_BUFFER __attribute__((section(".bss.sdmmc_dma_buffer"))) __attribute__((aligned(32)))

/* 高频任务栈使用专用快速内存段。 */
#define FAST_STACK __attribute__((section(".fast_stack")))

/* ========================== 接口声明 ========================== */

/* 从非缓存 D2 SRAM 单向内存池分配 DMA 缓冲区。 */
void *BSP_DMA_Malloc(size_t size);

/* 重置内存池，仅允许在所有 DMA 停止后的初始化阶段调用。 */
void    BSP_DMA_ResetPool(void);
size_t  BSP_DMA_GetFreeSize(void);
uint8_t BSP_DMA_GetUsagePercent(void);

#endif /* STM32H743_UAV_FLIGHT_CONTROLLER_BSP_MEMORY_SECTION_H */
