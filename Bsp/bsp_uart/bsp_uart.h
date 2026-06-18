//
// Created by Administrator on 2026/6/15.
//

#ifndef MY_NEW_UAV_BAICE_FRAMEWORK_BSP_UART_H
#define MY_NEW_UAV_BAICE_FRAMEWORK_BSP_UART_H

#include "main.h"
#include "stm32h7xx_hal_uart.h"

#include <stdint.h>

/* ========================== 宏配置 ========================== */

#define DEVICE_USART_CNT     8U    /* 最多注册的串口实例数量 */
#define USART_RXBUFF_LIMIT   1024U /* 单个接收缓冲区的最大字节数 */

/* ========================== 类型定义 ========================== */

/**
 * @brief 接收模式
 *
 * NORMAL   : 普通 IDLE 帧模式，适合变长协议帧，DMA 配置为 Normal 模式
 * CIRCULAR : 循环 DMA 模式，适合高速连续流，DMA 必须配置为 Circular 模式，
 * HT / TC / IDLE 三类事件均会触发数据处理
 */
typedef enum
{
    USART_RX_MODE_NORMAL   = 0,
    USART_RX_MODE_CIRCULAR = 1,
} USART_RX_MODE;

/**
 * @brief 发送模式
 *
 * @note DMA / IT 模式下，send_buf 指向的缓冲区必须在发送完成前保持有效，
 * 调用方不得在发送完成前释放或复用该缓冲区
 */
typedef enum
{
    USART_TRANSFER_BLOCKING = 0,
    USART_TRANSFER_IT       = 1,
    USART_TRANSFER_DMA      = 2,
} USART_TRANSFER_MODE;

/**
 * @brief 串口事件枚举 (新增：事件驱动核心)
 */
typedef enum
{
    USART_EVENT_RX_CPLT = 0, /* 接收完成（或 IDLE 帧截断） */
    USART_EVENT_TX_CPLT,     /* 发送完成 */
    USART_EVENT_ERROR,       /* 传输出错 (ORE, FE, NE, PE 等) */
} USART_Event_e;

/* 前置声明串口实例结构体 */
typedef struct USARTInstance_t USARTInstance;

/**
 * @brief 串口统一事件回调函数类型 (新增：统一回调接口)
 *
 * @warning 该回调运行于 ISR 上下文，必须保证：
 * 1. 执行时间极短
 * 2. 不得调用任何阻塞函数
 * 3. 使用 FreeRTOS API 时须使用 FromISR 版本
 * 4. 不得在回调内修改 data_ptr 指向的缓冲区（DMA 仍可能使用）
 *
 * @note    【中断防御铁律】为了防止 RX 和 TX 中断相互抢占导致的数据踩踏，
 * 请务必在 CubeMX 中将该串口的全局中断、RX_DMA 中断、TX_DMA 中断
 * 的“抢占优先级(Preemption Priority)”设置为完全相同！(利用 ARM 尾链技术)
 *
 * @param inst      触发事件的串口实例指针
 * @param event     触发的具体事件枚举
 * @param data_ptr  数据指针（仅在 RX_CPLT 事件中有效；TX/ERROR 事件中传入 NULL）
 * @param data_len  数据长度（仅在 RX_CPLT 事件中有效；TX/ERROR 事件中传入 0）
 */
typedef void (*usart_event_callback)(USARTInstance *inst, USART_Event_e event, uint8_t *data_ptr, uint16_t data_len);

/**
 * @brief 串口实例结构体
 *
 * @note 由 USARTRegister() 分配并初始化，调用方不应直接修改任何字段
 */
struct USARTInstance_t
{
    UART_HandleTypeDef   *usart_handle;   /* HAL 串口句柄 */
    usart_event_callback  event_callback; /* 新增：统一事件回调，运行于 ISR */

    USART_RX_MODE rx_mode;        /* 接收模式 */
    uint16_t      recv_buff_size; /* 接收缓冲区大小（字节），CIRCULAR 下为环形区大小 */

    uint8_t *recv_buff;    /* DMA 接收缓冲区（须位于 DMA 可访问内存区域）*/
    uint8_t *process_buff; /* 中间拷贝缓冲区（仅 CIRCULAR 模式使用，供 callback 读取）*/

    uint16_t last_dma_pos; /* 上次处理结束时 DMA 已写入位置（仅 CIRCULAR 模式使用）*/
};

/**
 * @brief 串口注册初始化配置
 */
typedef struct
{
    UART_HandleTypeDef   *usart_handle;   /* HAL 串口句柄，不得为 NULL */
    usart_event_callback  event_callback; /* 新增：统一事件回调，可为 NULL（仅收发不处理）*/
    USART_RX_MODE         rx_mode;        /* 接收模式 */
    uint16_t              recv_buff_size; /* 接收缓冲区大小，范围 (0, USART_RXBUFF_LIMIT] */
} USART_Init_Config_s;

/* ========================== 接口声明 ========================== */

/**
 * @brief  注册一个串口实例并启动接收
 *
 * @param  init_config  初始化配置，不得为 NULL
 * @return 成功返回实例指针，失败返回 NULL
 *
 * @note   CIRCULAR 模式下，CubeMX 中对应 DMA 通道必须配置为 Circular 模式，
 * 否则注册时会断言失败
 */
USARTInstance *USARTRegister(USART_Init_Config_s *init_config);

/**
 * @brief  （重）启动串口 DMA 接收服务
 *
 * @param  instance  串口实例，不得为 NULL
 *
 * @note   通常由内部在错误恢复时调用，外部如需手动重启接收可调用此函数
 */
void USARTServiceInit(USARTInstance *instance);

/**
 * @brief  向串口发送数据
 *
 * @param  instance   串口实例，不得为 NULL
 * @param  send_buf   发送数据缓冲区，不得为 NULL
 * @param  send_size  发送字节数，不得为 0
 * @param  mode       发送模式
 *
 * @note   IT/DMA 模式下，send_buf 必须在发送完成前保持有效（不得为局部变量）
 * @note   调用前建议先用 USARTIsTransmitReady() 确认发送链路空闲
 */
void USARTSend(USARTInstance    *instance,
               uint8_t          *send_buf,
               uint16_t          send_size,
               USART_TRANSFER_MODE mode);

/**
 * @brief  查询发送链路是否空闲
 *
 * @param  instance  串口实例，不得为 NULL
 * @return 1 表示可发送，0 表示忙碌或实例无效
 */
uint8_t USARTIsTransmitReady(USARTInstance *instance);

#endif //MY_NEW_UAV_BAICE_FRAMEWORK_BSP_UART_H