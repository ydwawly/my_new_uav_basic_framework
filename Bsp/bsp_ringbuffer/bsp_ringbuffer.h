//
// Created by Administrator on 2026/6/9.
//

#ifndef MY_NEW_UAV_BAICE_FRAMEWORK_BSP_RINGBUFFER_H
#define MY_NEW_UAV_BAICE_FRAMEWORK_BSP_RINGBUFFER_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 无锁环形缓冲区结构体 (单生产者-单消费者安全)
 */
typedef struct
{
    uint8_t *buffer;         // 指向用户分配的底层数据内存池
    uint32_t size;           // 缓冲区总大小 (必须是2的幂次方)
    uint32_t mask;           // 掩码，等于 size - 1，用于快速位运算计算物理地址
    volatile uint32_t head;  // 写入游标 (生产者专用)，只增不减，依靠无符号溢出
    volatile uint32_t tail;  // 读取游标 (消费者专用)，只增不减，依靠无符号溢出
} RingBuffer_t;

/**
 * @brief  初始化环形缓冲区
 * @param  rb:   环形缓冲区句柄指针
 * @param  pool: 用户提供的底层数据内存池数组
 * @param  size: 内存池大小 (强制要求：必须是 2、4、8...1024、2048 等 2 的幂次方)
 * @return true: 初始化成功 / false: 初始化失败(指针为空或大小不符合要求)
 */
bool RingBuffer_Init(RingBuffer_t *rb, uint8_t* pool, uint32_t size);

/**
 * @brief  获取缓冲区当前已存储的数据量 (字节数)
 * @param  rb: 环形缓冲区句柄
 * @return 已使用的字节数
 */
uint32_t RingBuffer_GetUsed(RingBuffer_t* rb);

/**
 * @brief  获取缓冲区当前的剩余可用空间 (字节数)
 * @param  rb: 环形缓冲区句柄
 * @return 剩余空闲字节数
 */
uint32_t RingBuffer_GetFree(RingBuffer_t* rb);

/**
 * @brief  单字节高效写入 (极其适合串口 RX 接收中断)
 * @param  rb:   环形缓冲区句柄
 * @param  data: 要写入的单个字节数据
 * @return true: 写入成功 / false: 缓冲区已满，写入失败
 */
bool RingBuffer_PushByte(RingBuffer_t* rb, uint8_t data);

/**
 * @brief  单字节高效读取 (适合主循环逐字节处理)
 * @param  rb:       环形缓冲区句柄
 * @param  out_data: 用于接收读出数据的指针
 * @return true: 读取成功 / false: 缓冲区为空，无数据可读
 */
bool RingBuffer_PopByte(RingBuffer_t* rb, uint8_t* out_data);

/**
 * @brief  数据块批量写入 (按块内存拷贝)
 * @param  rb:   环形缓冲区句柄
 * @param  data: 要写入的数据源指针
 * @param  len:  要写入的字节长度
 * @return true: 写入成功 / false: 剩余空间不足，拒绝写入
 */
bool RingBuffer_Push(RingBuffer_t* rb, const void* data, uint32_t len);

/**
 * @brief  数据块批量读取 (按块内存拷贝)
 * @param  rb:      环形缓冲区句柄
 * @param  out_data:用于接收读出数据的目标指针
 * @param  max_len: 期望读取的最大字节长度
 * @return 实际成功读取的字节数 (可能小于期望的 max_len)
 */
uint32_t RingBuffer_Pop(RingBuffer_t* rb, void* out_data, uint32_t max_len);

#endif //MY_NEW_UAV_BAICE_FRAMEWORK_BSP_RINGBUFFER_H