# 基础框架：无锁环形缓冲区 (Lock-Free RingBuffer)

## 1. 概述
本模块提供了一个针对嵌入式系统（MCU）高度优化的无锁环形缓冲区组件。采用纯 C 语言编写，具备执行效率极高、资源占用小等特点，特别适用于 **单生产者 - 单消费者（SPSC）** 场景下的跨线程或中断-主循环通信（如串口收发、传感器数据流缓冲等）。

## 2. 核心特性
* **无锁安全 (Lock-Free)：** 在 "一写一读" 的并发场景下，无需使用关闭中断、互斥锁等高开销的临界区保护，利用 `volatile` 和 `__DMB()` 硬件级内存屏障保障数据绝对一致性。
* **极致性能：** 强制规定缓冲区大小为 2 的幂次方，从而将极其耗时的“取模/除法 `%`”运算替换为单时钟周期的“位与 `&`”运算。
* **指针免回绕处理：** 底层利用 `uint32_t` 类型的自然溢出机制，读写指针 `head` 和 `tail` 无限递增，无需复杂的边界复位逻辑，大大简化了代码复杂度。
* **丰富的 API：** 同时提供针对批量数据的高效内存块读写接口 (`Push` / `Pop`) 以及针对单个字节的极速读写接口 (`PushByte` / `PopByte`)。

## 3. 使用约束与注意事项 (必读)
1.  **内存池大小限制：** 初始化时传入的 `size` 参数 **必须** 是 2 的幂次方（如 128, 256, 1024, 2048 等），否则初始化将失败或导致严重越界。
2.  **单生产/单消费模型：** * 本模块仅防范“一个往里写、一个往外读”的冲突。
    * **严禁** 两个或以上的中断/线程同时调用 `Push` 接口写入数据。
    * **严禁** 两个或以上的中断/线程同时调用 `Pop` 接口读取数据。
3.  **D-Cache 一致性：** 如果部署在搭载 Cortex-M7 等具备数据缓存（D-Cache）的高级内核上，并在 DMA 场景下使用该缓冲区，请务必配合 MPU 配置或手动执行 Cache Clean/Invalidate 操作，`__DMB()` 无法代替 Cache 同步。

## 4. 快速上手示例

### 4.1 初始化缓冲区
用户需要自行静态或动态分配一块内存，然后将其交给 RingBuffer 管理。

```c
#include "bsp_ringbuffer.h"

// 1. 定义控制句柄和内存池 (注意大小必须是 2 的 N 次方)
#define RX_BUFFER_SIZE 1024
RingBuffer_t uart_rx_fifo;
uint8_t uart_rx_pool[RX_BUFFER_SIZE];

void System_Init(void) {
    // 2. 初始化环形缓冲区
    bool ret = RingBuffer_Init(&uart_rx_fifo, uart_rx_pool, RX_BUFFER_SIZE);
    if (!ret) {
        // 初始化失败，通常是因为大小不是 2 的幂次方
        Error_Handler();
    }
}

// 生产者：串口接收中断
void USART1_IRQHandler(void) {
    if (UART_GetITStatus(USART1, UART_IT_RXNE) != RESET) {
        uint8_t rx_byte = UART_ReceiveData(USART1);
        
        // 推入数据，如果满了会返回 false 直接丢弃
        RingBuffer_PushByte(&uart_rx_fifo, rx_byte);
    }
}

// 消费者：主循环/业务任务处理
void App_Task(void) {
    uint8_t process_byte;
    
    // 如果返回 true，说明成功读出了一个字节
    while (RingBuffer_PopByte(&uart_rx_fifo, &process_byte)) {
        // 解析 process_byte 数据...
        Parse_Protocol(process_byte);
    }
}

// 定义要发送或接收的数组
uint8_t sensor_data[32] = {0x01, 0x02, ...};
uint8_t read_buffer[64];

// --- 批量写入 ---
// 将 32 字节数据压入 FIFO，如果剩余空间不足 32 字节，则写入失败返回 false
bool push_ok = RingBuffer_Push(&sensor_data_fifo, sensor_data, sizeof(sensor_data));


// --- 批量读取 ---
// 尝试从 FIFO 读取 64 字节数据。
// 返回值为实际成功读取到的字节数 (如果 FIFO 里只有 10 字节，就会返回 10)
uint32_t actual_read = RingBuffer_Pop(&sensor_data_fifo, read_buffer, sizeof(read_buffer));

if (actual_read > 0) {
    // 处理读出来的数据...
}