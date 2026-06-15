//
// Created by Administrator on 2026/6/15.
//

#ifndef MY_NEW_UAV_BAICE_FRAMEWORK_BSP_UTILS_SEQLOCK_H
#define MY_NEW_UAV_BAICE_FRAMEWORK_BSP_UTILS_SEQLOCK_H

#include <stdint.h>
#include <stdbool.h>
#include "cmsis_compiler.h" /* 提供 __DMB() 指令支持 */

/**
 * @brief 序列锁结构体
 * @note  必须保证 sequence 变量被 volatile 修饰，防止编译器将其优化到寄存器中
 */
typedef struct {
    volatile uint32_t sequence;
} SeqLock_t;

/**
 * @brief 初始化序列锁
 */
static inline void SeqLock_Init(SeqLock_t *lock)
{
    lock->sequence = 0;
}

/**
 * @brief 写者开始写入（通常在 DMA/EXTI 中断中调用）
 */
static inline void SeqLock_WriteBegin(SeqLock_t *lock)
{
    lock->sequence++;
    /* 硬件数据内存屏障：确保 sequence 的奇数状态在写入数据前被刷入内存，对外可见 */
    __DMB();
}

/**
 * @brief 写者完成写入
 */
static inline void SeqLock_WriteEnd(SeqLock_t *lock)
{
    /* 硬件数据内存屏障：确保数据写入完成并落盘后，再去更新 sequence 为偶数 */
    __DMB();
    lock->sequence++;
}

/**
 * @brief 读者开始读取（尝试获取当前版本号）
 * @return 获取到的起始序列号
 */
static inline uint32_t SeqLock_ReadBegin(const SeqLock_t *lock)
{
    uint32_t seq;
    do {
        seq = lock->sequence;
    } while ((seq & 1U) != 0U); /* 如果是奇数，说明写者正在更新数据，自旋等待其写完 */

    /* 硬件数据内存屏障：确保先拿到正确的版本号，再去搬运共享内存的数据 */
    __DMB();
    return seq;
}

/**
 * @brief 读者检查读取期间是否被打断
 * @param start_seq 读取前获取的起始序列号
 * @retval true  被打断（数据不一致，需要重试）
 * @retval false 未被打断（数据安全可用）
 */
static inline bool SeqLock_ReadRetry(const SeqLock_t *lock, uint32_t start_seq)
{
    /* 硬件数据内存屏障：确保共享内存的数据全部搬运完毕后，再去检查版本号是否发生变化 */
    __DMB();
    return (lock->sequence != start_seq);
}

/**
 * @brief 读者尝试开始读取（非阻塞）
 * @return 起始序列号，若正在写入则返回奇数，由上层决定是否重试
 */
static inline uint32_t SeqLock_TryReadBegin(const SeqLock_t *lock)
{
    uint32_t seq = lock->sequence;
    __DMB();
    return seq;
}

#endif //MY_NEW_UAV_BAICE_FRAMEWORK_BSP_UTILS_SEQLOCK_H