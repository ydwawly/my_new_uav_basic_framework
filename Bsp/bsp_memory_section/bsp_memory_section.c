//
// Created by Administrator on 2026/6/15.
//

#include "bsp_memory_section.h"

/* 静态变量定义在 .c 文件中，确保全局只有一份，并且仅被此文件内部直接访问 */
DMA_BUFFER static uint8_t bsp_dma_mem_pool[BSP_DMA_MEM_POOL_SIZE];
static size_t bsp_dma_mem_offset = 0U;

void *BSP_DMA_Malloc(size_t size)
{
    size_t aligned_size;
    void *ptr;

    if (size == 0U) {
        return NULL;
    }

    /* 32字节对齐 */
    aligned_size = (size + 31U) & ~(size_t)31U;

    if ((bsp_dma_mem_offset + aligned_size) > BSP_DMA_MEM_POOL_SIZE) {
        RTTERROR("[bsp_alloc] DMA pool exhausted: %u bytes.", (unsigned int)size);
        return NULL;
    }

    ptr = &bsp_dma_mem_pool[bsp_dma_mem_offset];
    bsp_dma_mem_offset += aligned_size;

    return ptr;
}

/* 重置 DMA 内存池 */
void BSP_DMA_ResetPool(void)
{
    bsp_dma_mem_offset = 0U;
    /* 可选：将内存池清零，防止脏数据，但会消耗一点 CPU 时间 */
    // memset(bsp_dma_mem_pool, 0, BSP_DMA_MEM_POOL_SIZE);
}

/* 获取剩余可用内存 */
size_t BSP_DMA_GetFreeSize(void)
{
    return BSP_DMA_MEM_POOL_SIZE - bsp_dma_mem_offset;
}

/* 获取内存使用率 (0~100) */
uint8_t BSP_DMA_GetUsagePercent(void)
{
    return (uint8_t)((bsp_dma_mem_offset * 100U) / BSP_DMA_MEM_POOL_SIZE);
}