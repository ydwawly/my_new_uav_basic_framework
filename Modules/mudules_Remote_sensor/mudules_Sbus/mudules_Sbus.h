//
// Created by Administrator on 2026/6/18.
//

#ifndef MY_NEW_UAV_BAICE_FRAMEWORK_MUDULES_SBUS_H
#define MY_NEW_UAV_BAICE_FRAMEWORK_MUDULES_SBUS_H

#include <stdint.h>

#include "bsp_uart.h"
#include "bsp_utils_seqlock.h"
#include "modules_Message_center.h"
#include "usart.h"

/* ========================== 私有配置 ========================== */

#define SBUS_UART_HANDLE         huart1
#define SBUS_SEQLOCK_MAX_RETRY   3U
#define SBUS_TOPIC_NAME          "sbus_data"
/* ========================== 协议定义 ========================== */

/*
 * SBUS 物理参数：
 *   波特率 100000, 8E2 (8数据位, 偶校验, 2停止位)
 *   信号反相（需硬件反相器或 UART 寄存器配置反相）
 *
 * SBUS 帧结构 (固定 25 字节)：
 *   [0]       帧头 0x0F
 *   [1..22]   16 通道数据, 每通道 11 bit, 小端连续排列, 共 176 bit = 22 字节
 *   [23]      标志位字节
 *             bit0: CH17 (数字通道)
 *             bit1: CH18 (数字通道)
 *             bit2: Frame Lost
 *             bit3: Failsafe
 *             bit4~7: 保留
 *   [24]      帧尾 0x00
 *
 * 通道值范围：
 *   原始值: 0 ~ 2047 (11 bit)
 *   典型范围: 172 ~ 1811
 *   中点: 992
 */

#define SBUS_FRAME_LEN              25U
#define SBUS_HEADER                 0x0FU
#define SBUS_FOOTER                 0x00U
#define SBUS_CHANNEL_COUNT          16U
#define SBUS_DIGITAL_CHANNEL_COUNT  2U

/* 帧内字段偏移 */
#define SBUS_IDX_HEADER             0U
#define SBUS_IDX_PAYLOAD_START      1U
#define SBUS_IDX_PAYLOAD_END        22U
#define SBUS_IDX_FLAGS              23U
#define SBUS_IDX_FOOTER             24U

/* 标志位定义 */
#define SBUS_FLAG_CH17              (1U << 0)
#define SBUS_FLAG_CH18              (1U << 1)
#define SBUS_FLAG_FRAME_LOST        (1U << 2)
#define SBUS_FLAG_FAILSAFE          (1U << 3)

/* 通道值参考范围 */
#define SBUS_CHANNEL_VALUE_MIN      172U
#define SBUS_CHANNEL_VALUE_MID      992U
#define SBUS_CHANNEL_VALUE_MAX      1811U

/* ========================== 对外发布数据结构 ========================== */

/**
 * @brief SBUS 解析后的遥控数据
 *
 * @note channels[0..15] 对应 CH1~CH16, 原始值范围 0~2047
 *       ch17 / ch18 为数字通道, 0 或 1
 *       frame_lost == 1 表示接收机丢帧
 *       failsafe   == 1 表示接收机进入 Failsafe
 */
typedef struct
{
    uint16_t channels[SBUS_CHANNEL_COUNT];

    uint8_t  ch17;
    uint8_t  ch18;
    uint8_t  frame_lost;
    uint8_t  failsafe;

    uint64_t Sbus_Timestamp;
} SBUS_Data_t;

/* ========================== 私有实例结构体 ========================== */

typedef struct
{
    SeqLock_t data_lock;

    /* ISR 中唯一一次 memcpy 落地的原始帧 */
    uint8_t  raw_rx_buf[SBUS_FRAME_LEN];
    uint16_t raw_len;
    uint64_t capture_timestamp;
    uint32_t rx_frame_count;
    uint32_t last_proc_frame_count;

    volatile uint32_t rx_err_count;
    volatile uint32_t parse_err_count;

    Publisher_t   *publisher;
    USARTInstance *usart_instance;
} SBUS_Instance_t;

/* ========================== 对外接口 ========================== */

/**
 * @brief 初始化 SBUS 驱动
 *
 * @return 1 成功
 * @return 0 失败
 *
 * @note 内部自动完成：
 *       1. 初始化 SeqLock
 *       2. 注册消息中心发布者（话题名 "sbus_data"）
 *       3. 注册对应 UART，DMA Normal 模式，固定 25 字节帧长
 *
 * @warning CubeMX 串口配置必须为：
 *          波特率 100000, Word Length 8bit, Even Parity, 2 Stop Bits
 *          若使用硬件反相器则 UART 保持正常极性
 *          若无硬件反相器则需在 UART 寄存器中配置 RX 反相
 */
uint8_t SBUS_Init(void);

/**
 * @brief SBUS 任务处理函数
 *
 * @return 1 成功解析并发布了一帧新数据
 * @return 0 无新帧或解析失败
 *
 * @note 在任务中周期调用，建议调用周期 <= 5ms
 */
uint8_t SBUS_Task_Handler(void);

#endif //MY_NEW_UAV_BAICE_FRAMEWORK_MUDULES_SBUS_H