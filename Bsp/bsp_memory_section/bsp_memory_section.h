//
// Created by Administrator on 2026/6/15.
//

#ifndef MY_NEW_UAV_BAICE_FRAMEWORK_BSP_MEMORY_SECTION_H
#define MY_NEW_UAV_BAICE_FRAMEWORK_BSP_MEMORY_SECTION_H

#include <stddef.h>
#include <stdint.h>
#include "bsp_RTT.h"

#define BSP_DMA_MEM_POOL_SIZE 4096U

/* ITCM代码 */
#define ITCM_CODE \
__attribute__((section(".itcm")))

/* DTCM数据 */
#define DTCM_DATA \
__attribute__((section(".dtcm")))

/* DMA Buffer */
#define DMA_BUFFER \
__attribute__((section(".dma_buffer"))) \
__attribute__((aligned(32)))

/* Fast Stack */
#define FAST_STACK \
__attribute__((section(".fast_stack")))

/* 对外提供的函数声明 */
void *BSP_DMA_Malloc(size_t size);
/* 内存池管理函数 */
void BSP_DMA_ResetPool(void);
size_t BSP_DMA_GetFreeSize(void);
uint8_t BSP_DMA_GetUsagePercent(void);

#endif //MY_NEW_UAV_BAICE_FRAMEWORK_BSP_MEMORY_SECTION_H