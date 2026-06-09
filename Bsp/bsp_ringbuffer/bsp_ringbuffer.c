//
// Created by Administrator on 2026/6/9.
//

#include "bsp_ringbuffer.h"
#include <string.h>
#include <sys/types.h>

#include "cmsis_compiler.h" // 包含硬件内存屏障指令 __DMB()

bool RingBuffer_Init(RingBuffer_t* rb, uint8_t* pool, uint32_t size)
{
    // 参数合法性检查
    if (rb == NULL || pool == NULL || size == 0) return false;

    // 核心安全检查：确保 size 是 2 的幂次方 (例如 256, 1024)
    // 原理：2的幂次方减1后，二进制全为1，这样才能用位与(&)替代取模(%)运算
    if ((size & (size - 1)) != 0) return false;

    rb->buffer = pool;
    rb->size = size;
    rb->mask = rb->size - 1; // 预先算好掩码，极致压榨计算性能
    rb->head = 0;            // 初始化读写游标
    rb->tail = 0;

    // 清空用户提供的内存池
    memset(rb->buffer, 0, size);
    return true;
}

uint32_t RingBuffer_GetUsed(RingBuffer_t* rb)
{
    // 利用无符号数的自然溢出特性，直接相减即可得到正确差值
    return rb->head - rb->tail;
}

uint32_t RingBuffer_GetFree(RingBuffer_t* rb)
{
    return rb->size - (rb->head - rb->tail);
}

bool RingBuffer_Push(RingBuffer_t* rb, const void* data, uint32_t len)
{
    uint32_t head = rb->head;
    uint32_t tail = rb->tail;

    // 检查剩余可用空间是否能够容纳本次写入的长度
    uint32_t free_space = rb->size - (head - tail);
    if (free_space < len) return false; // 空间不足，放弃写入

    // 计算当前 head 游标对应的实际物理数组下标
    uint32_t head_start = head & rb->mask;
    // 计算从当前物理下标到数组物理末尾，还有多少连续的可用空间
    uint32_t space_to_end = rb->size - head_start;

    const uint8_t* ptr = (const uint8_t*)data;

    if (len <= space_to_end)
    {
        // 情况 1：物理连续空间足够，不需要折返，一次性拷贝完成
        memcpy(rb->buffer + head_start, ptr, len);
    }
    else
    {
        // 情况 2：数据跨越了物理数组边界，需要分成两段拷贝 (已修复原代码 BUG)
        // 第一段：把数据塞满物理数组的末尾剩余空间
        memcpy(rb->buffer + head_start, ptr, space_to_end);
        // 第二段：绕回到物理数组的开头 (rb->buffer)，拷贝剩下的数据
        // 注意源数据指针 ptr 也需要加上已经拷贝过的偏移量 space_to_end
        memcpy(rb->buffer, ptr + space_to_end, len - space_to_end);
    }

    // 数据内存屏障：强制 CPU 确保上面的 memcpy 数据彻底写入 SRAM，再执行下一步
    // 这是实现无锁单生产单消费(SPSC)安全的绝对核心！
    __DMB();

    // 更新游标：允许数值溢出，配合 2 的幂次方 mask 可以完美无限循环
    rb->head = head + len;
    return true;
}

uint32_t RingBuffer_Pop(RingBuffer_t* rb, void* out_data, uint32_t max_len)
{
    uint32_t tail = rb->tail;
    uint32_t head = rb->head;

    // 计算当前缓冲区内有多少数据可读
    uint32_t available_space = head - tail;
    if (available_space == 0) return 0; // 缓冲区为空

    // 确定实际要读取的长度（不能超过现存数据量，也不能超过用户请求的最大量）
    uint32_t read_len = (available_space > max_len) ? max_len : available_space;

    // 计算当前 tail 游标对应的实际物理数组下标
    uint32_t read_start = tail & rb->mask;
    // 计算从当前物理下标到数组物理末尾的连续可读空间
    uint32_t space_to_end = rb->size - read_start;

    uint8_t* ptr = (uint8_t*)out_data;

    if (read_len <= space_to_end)
    {
        // 情况 1：要读的数据都在一段连续的物理空间内，一次拷贝完成
        memcpy(ptr, rb->buffer + read_start, read_len);
    }
    else
    {
        // 情况 2：要读的数据跨越了物理边界，需要分两段读取 (已修复原代码 BUG)
        // 第一段：读到物理数组的末尾
        memcpy(ptr, rb->buffer + read_start, space_to_end);
        // 第二段：绕回到物理数组的开头 (rb->buffer) 继续读剩下的数据
        // 注意目标数据指针 ptr 也需要加上已经存入的偏移量 space_to_end
        memcpy(ptr + space_to_end, rb->buffer, read_len - space_to_end);
    }

    // 数据内存屏障：确保数据已经被完全读出并存入目标指针，再去释放空间
    __DMB();

    rb->tail = tail + read_len;
    return read_len;
}

bool RingBuffer_PushByte(RingBuffer_t* rb, uint8_t data) {
    uint32_t head = rb->head;
    uint32_t tail = rb->tail;

    // 检查缓冲区是否已满
    if ((head - tail) >= rb->size) {
        return false;
    }

    // 利用位与(&)运算快速求出物理地址并写入单字节
    rb->buffer[head & rb->mask] = data;

    // 屏障：确保数据真正写进 SRAM 后，再去更新 head 游标
    __DMB();

    rb->head = head + 1;
    return true;
}


bool RingBuffer_PopByte(RingBuffer_t* rb, uint8_t* out_data) {
    uint32_t head = rb->head;
    uint32_t tail = rb->tail;

    // 检查缓冲区是否为空
    if (head == tail) {
        return false;
    }

    // 利用位与(&)运算快速求出物理地址并读出单字节
    *out_data = rb->buffer[tail & rb->mask];

    // 屏障：确保数据真正被读取后，再去更新 tail 游标
    __DMB();

    rb->tail = tail + 1;
    return true;
}