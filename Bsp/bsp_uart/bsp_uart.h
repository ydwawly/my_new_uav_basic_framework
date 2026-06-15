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
 * @brief 串口数据接收完成回调
 *
 * @warning 该回调运行于 ISR 上下文，必须保证：
 *          1. 执行时间极短
 *          2. 不得调用任何阻塞函数
 *          3. 使用 FreeRTOS API 时须使用 FromISR 版本
 *          4. 不得在回调内修改 data_ptr 指向的缓冲区（DMA 仍可能使用）
 *
 * @param data_ptr  本次接收到的数据首地址
 * @param data_len  本次接收到的数据长度（字节）
 */
typedef void (*usart_module_callback)(uint8_t *data_ptr, uint16_t data_len);

/**
 * @brief 接收模式
 *
 * NORMAL   : 普通 IDLE 帧模式，适合变长协议帧，DMA 配置为 Normal 模式
 * CIRCULAR : 循环 DMA 模式，适合高速连续流，DMA 必须配置为 Circular 模式，
 *            HT / TC / IDLE 三类事件均会触发数据处理
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
 *       调用方不得在发送完成前释放或复用该缓冲区
 */
typedef enum
{
    USART_TRANSFER_BLOCKING = 0,
    USART_TRANSFER_IT       = 1,
    USART_TRANSFER_DMA      = 2,
} USART_TRANSFER_MODE;

/**
 * @brief 串口实例结构体
 *
 * @note 由 USARTRegister() 分配并初始化，调用方不应直接修改任何字段
 */
typedef struct
{
    UART_HandleTypeDef   *usart_handle;    /* HAL 串口句柄 */
    usart_module_callback module_callback; /* 接收完成回调，运行于 ISR */

    USART_RX_MODE rx_mode;      /* 接收模式 */
    uint16_t      recv_buff_size; /* 接收缓冲区大小（字节），CIRCULAR 下为环形区大小 */

    uint8_t *recv_buff;    /* DMA 接收缓冲区（须位于 DMA 可访问内存区域）*/
    uint8_t *process_buff; /* 中间拷贝缓冲区（仅 CIRCULAR 模式使用，供 callback 读取）*/

    uint16_t last_dma_pos; /* 上次处理结束时 DMA 已写入位置（仅 CIRCULAR 模式使用）*/
} USARTInstance;

/**
 * @brief 串口注册初始化配置
 */
typedef struct
{
    UART_HandleTypeDef   *usart_handle;    /* HAL 串口句柄，不得为 NULL */
    usart_module_callback module_callback; /* 接收完成回调，可为 NULL（仅接收不处理）*/
    USART_RX_MODE         rx_mode;         /* 接收模式 */
    uint16_t              recv_buff_size;  /* 接收缓冲区大小，范围 (0, USART_RXBUFF_LIMIT] */
} USART_Init_Config_s;

/* ========================== 接口声明 ========================== */

/**
 * @brief  注册一个串口实例并启动接收
 *
 * @param  init_config  初始化配置，不得为 NULL
 * @return 成功返回实例指针，失败返回 NULL
 *
 * @note   CIRCULAR 模式下，CubeMX 中对应 DMA 通道必须配置为 Circular 模式，
 *         否则注册时会断言失败
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