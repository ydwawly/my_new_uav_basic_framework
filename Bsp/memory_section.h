//
// Created by Administrator on 2026/6/8.
//

#ifndef MY_NEW_UAV_BAICE_FRAMEWORK_MEMORY_SECTION_H
#define MY_NEW_UAV_BAICE_FRAMEWORK_MEMORY_SECTION_H

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


#endif //MY_NEW_UAV_BAICE_FRAMEWORK_MEMORY_SECTION_H