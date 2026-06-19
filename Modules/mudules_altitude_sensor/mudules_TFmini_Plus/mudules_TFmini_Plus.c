//
// Created by Administrator on 2026/6/18.
//

#include "mudules_TFmini_Plus.h"
#include <string.h>
#include "bsp_RTT.h"
#include "bsp_timestamp.h"

/* ========================== 全局静态实例 ========================== */
static TFmini_Instance_t tfmini_instance;
/* ========================== 私有函数声明 ========================== */
static uint8_t  TFmini_Checksum(const uint8_t *buf);
static void TFmini_ParseRawFrame(const uint8_t *buf, TFminiPlus_Data_t *out);
static void TFmini_UART_EventCallback(USARTInstance *ins,USART_Event_e event,uint8_t *data_ptr,uint16_t data_len);
/* ========================== 私有函数实现 ========================== */

static uint8_t TFmini_Checksum(const uint8_t *buf)
{
    uint8_t sum = 0U;
    uint8_t i;

    for (i = 0U; i < (TFMINI_FRAME_SIZE - 1U); i++)
    {
        sum = (uint8_t)(sum + buf[i]);
    }

    return sum;
}

/**
 * @brief 解析 TFmini 原始 9 字节帧
 *
 * @note buf 已经通过帧头和校验检查
 */
static void TFmini_ParseRawFrame(const uint8_t *buf, TFminiPlus_Data_t *out)
{
    uint16_t raw_temp;

    out->distance = (uint16_t)((uint16_t)buf[TFMINI_IDX_DISTANCE_L] |
                              ((uint16_t)buf[TFMINI_IDX_DISTANCE_H] << 8));

    out->strength = (uint16_t)((uint16_t)buf[TFMINI_IDX_STRENGTH_L] |
                              ((uint16_t)buf[TFMINI_IDX_STRENGTH_H] << 8));

    raw_temp = (uint16_t)((uint16_t)buf[TFMINI_IDX_TEMP_L] |
                         ((uint16_t)buf[TFMINI_IDX_TEMP_H] << 8));

    out->temperature = ((float)raw_temp / 8.0f) - 256.0f;

    /*
     * 有效性判定沿用你原来的逻辑：
     * strength >= 100 且 != 0xFFFF 认为有效
     *
     * 如需更严格，也可以附加 distance > 0 判定
     */
    if ((out->strength >= 100U) && (out->strength != 0xFFFFU))
    {
        out->is_valid = 1U;
    }
    else
    {
        out->is_valid = 0U;
    }
}

/* ========================== 串口事件回调（内部私有） ========================== */

static void TFmini_UART_EventCallback(USARTInstance *ins,USART_Event_e event,uint8_t *data_ptr,uint16_t data_len)
{
    (void)ins;

    switch (event)
    {
    case USART_EVENT_RX_CPLT:
        if ((data_ptr == NULL) || (data_len == 0U))
        {
            return;
        }

        /*
         * ISR 中只做一次原始帧拷贝
         * 不做协议解析、不做字段处理、不做发布
         */
        SeqLock_WriteBegin(&tfmini_instance.data_lock);

        tfmini_instance.raw_len = (data_len > TFMINI_FRAME_SIZE) ? TFMINI_FRAME_SIZE : data_len;
        memcpy(tfmini_instance.raw_rx_buf, data_ptr, tfmini_instance.raw_len);
        tfmini_instance.capture_timestamp = Bsp_Timestamp_us_Get();
        tfmini_instance.rx_frame_count++;

        SeqLock_WriteEnd(&tfmini_instance.data_lock);
        break;

    case USART_EVENT_ERROR:
        tfmini_instance.rx_err_count++;
        break;

    case USART_EVENT_TX_CPLT:
    default:
        break;
    }
}

/* ========================== 任务级处理 ========================== */

uint8_t TFmini_Task_Handler(void)
{
    uint32_t start_seq;
    uint32_t frame_count;
    uint16_t raw_len;
    uint64_t timestamp_us;
    uint8_t  local_buf[TFMINI_FRAME_SIZE];
    uint8_t  checksum;
    uint8_t  retry;
    TFminiPlus_Data_t data;

    if (tfmini_instance.publisher == NULL)
    {
        return 0U;
    }

    for (retry = 0U; retry < TFMINI_SEQLOCK_MAX_RETRY; retry++)
    {
        /* ========== 阶段 1：在 SeqLock 保护下获取稳定快照 ========== */

        start_seq = SeqLock_ReadBegin(&tfmini_instance.data_lock);

        frame_count  = tfmini_instance.rx_frame_count;
        raw_len      = tfmini_instance.raw_len;
        timestamp_us = tfmini_instance.capture_timestamp;

        if (raw_len > TFMINI_FRAME_SIZE)
        {
            raw_len = TFMINI_FRAME_SIZE;
        }

        if (raw_len > 0U)
        {
            memcpy(local_buf, tfmini_instance.raw_rx_buf, raw_len);
        }

        if (SeqLock_ReadRetry(&tfmini_instance.data_lock, start_seq))
        {
            continue;
        }

        /* ========== 阶段 2：基于本地快照处理 ========== */

        /* 无新帧 */
        if ((frame_count == 0U) ||(frame_count == tfmini_instance.last_seen_frame_count))
        {
            return 0U;
        }

        /*
         * 注意：
         * 这里先记录“这份原始帧已经看过了”
         * 这样即使它是坏帧，也不会在任务高频调用时被反复统计多次
         */
        tfmini_instance.last_seen_frame_count = frame_count;

        /* 帧长度检查 */
        if (raw_len != TFMINI_FRAME_SIZE)
        {
            tfmini_instance.parse_err_count++;
            return 0U;
        }

        /* 帧头检查 */
        if ((local_buf[TFMINI_IDX_HEADER1] != TFMINI_HEADER) ||(local_buf[TFMINI_IDX_HEADER2] != TFMINI_HEADER))
        {
            tfmini_instance.parse_err_count++;
            return 0U;
        }

        /* 校验和检查 */
        checksum = TFmini_Checksum(local_buf);
        if (checksum != local_buf[TFMINI_IDX_CHECKSUM])
        {
            tfmini_instance.parse_err_count++;
            return 0U;
        }

        /* ========== 阶段 3：解析 + 发布 ========== */

        memset(&data, 0, sizeof(TFminiPlus_Data_t));

        TFmini_ParseRawFrame(local_buf, &data);
        data.timestamp_us = timestamp_us;

        tfmini_instance.last_pub_frame_count = frame_count;
        PubPushMessage(tfmini_instance.publisher, &data);

        return 1U;
    }

    return 0U;
}

/* ========================== 初始化 ========================== */

uint8_t TFmini_Init(void)
{
    USART_Init_Config_s  uart_config;

    memset(&tfmini_instance, 0, sizeof(TFmini_Instance_t));
    SeqLock_Init(&tfmini_instance.data_lock);

    /* 注册消息中心发布者 */
    tfmini_instance.publisher = PubRegister(TFMINI_TOPIC_NAME, sizeof(TFminiPlus_Data_t));
    if (tfmini_instance.publisher == NULL)
    {
        RTTERROR("[TFmini] PubRegister Failed!");
        return 0U;
    }

    /* 注册串口：固定 9 字节帧，DMA Normal 模式 */
    memset(&uart_config, 0, sizeof(uart_config));
    uart_config.usart_handle   = &TFMINI_UART_HANDLE;
    uart_config.recv_buff_size = TFMINI_FRAME_SIZE;
    uart_config.event_callback = TFmini_UART_EventCallback;
    uart_config.rx_mode        = USART_RX_MODE_NORMAL;

    tfmini_instance.usart_instance = USARTRegister(&uart_config);
    if (tfmini_instance.usart_instance == NULL)
    {
        RTTERROR("[TFmini] UART Register Failed!");
        return 0U;
    }

    RTTINFO("[TFmini] TFmini Plus Init Success!");
    return 1U;
}