//
// Created by Administrator on 2026/6/18.
//

#include "mudules_Sbus.h"
#include <string.h>
#include "bsp_RTT.h"
#include "bsp_timestamp.h"

/* ========================== 全局静态实例 ========================== */

static SBUS_Instance_t sbus_instance;

/* ========================== 私有函数声明 ========================== */

static void SBUS_ParseRawFrame(const uint8_t *buf, SBUS_Data_t *out);
static void SBUS_UART_EventCallback(USARTInstance *ins,
                                    USART_Event_e event,
                                    uint8_t *data_ptr,
                                    uint16_t data_len);

/* ========================== 私有函数实现 ========================== */

/**
 * @brief 从 25 字节原始帧中解析 16 个 11-bit 通道和标志位
 *
 * @note SBUS 通道编码格式：
 *       16 个通道，每通道 11 bit，小端连续紧密排列在 buf[1..22] 中
 *       共 16 × 11 = 176 bit = 22 字节
 *
 *       CH1  = buf[1]      | buf[2]<<8          & 0x07FF
 *       CH2  = buf[2]>>3   | buf[3]<<5          & 0x07FF
 *       CH3  = buf[3]>>6   | buf[4]<<2 | buf[5]<<10  & 0x07FF
 *       ... 以此类推
 */
static void SBUS_ParseRawFrame(const uint8_t *buf, SBUS_Data_t *out)
{
    const uint8_t *d = &buf[SBUS_IDX_PAYLOAD_START];

    /*
     * 16 通道解码
     *
     * 每个通道 11 bit，跨字节小端排列
     * 使用位操作逐个提取
     */
    out->channels[0]  = (uint16_t)(((uint16_t)d[0]       | (uint16_t)d[1]  << 8)                          & 0x07FFU);
    out->channels[1]  = (uint16_t)(((uint16_t)d[1]  >> 3 | (uint16_t)d[2]  << 5)                          & 0x07FFU);
    out->channels[2]  = (uint16_t)(((uint16_t)d[2]  >> 6 | (uint16_t)d[3]  << 2 | (uint16_t)d[4]  << 10)  & 0x07FFU);
    out->channels[3]  = (uint16_t)(((uint16_t)d[4]  >> 1 | (uint16_t)d[5]  << 7)                          & 0x07FFU);
    out->channels[4]  = (uint16_t)(((uint16_t)d[5]  >> 4 | (uint16_t)d[6]  << 4)                          & 0x07FFU);
    out->channels[5]  = (uint16_t)(((uint16_t)d[6]  >> 7 | (uint16_t)d[7]  << 1 | (uint16_t)d[8]  << 9)   & 0x07FFU);
    out->channels[6]  = (uint16_t)(((uint16_t)d[8]  >> 2 | (uint16_t)d[9]  << 6)                          & 0x07FFU);
    out->channels[7]  = (uint16_t)(((uint16_t)d[9]  >> 5 | (uint16_t)d[10] << 3)                          & 0x07FFU);
    out->channels[8]  = (uint16_t)(((uint16_t)d[11]      | (uint16_t)d[12] << 8)                          & 0x07FFU);
    out->channels[9]  = (uint16_t)(((uint16_t)d[12] >> 3 | (uint16_t)d[13] << 5)                          & 0x07FFU);
    out->channels[10] = (uint16_t)(((uint16_t)d[13] >> 6 | (uint16_t)d[14] << 2 | (uint16_t)d[15] << 10)  & 0x07FFU);
    out->channels[11] = (uint16_t)(((uint16_t)d[15] >> 1 | (uint16_t)d[16] << 7)                          & 0x07FFU);
    out->channels[12] = (uint16_t)(((uint16_t)d[16] >> 4 | (uint16_t)d[17] << 4)                          & 0x07FFU);
    out->channels[13] = (uint16_t)(((uint16_t)d[17] >> 7 | (uint16_t)d[18] << 1 | (uint16_t)d[19] << 9)   & 0x07FFU);
    out->channels[14] = (uint16_t)(((uint16_t)d[19] >> 2 | (uint16_t)d[20] << 6)                          & 0x07FFU);
    out->channels[15] = (uint16_t)(((uint16_t)d[20] >> 5 | (uint16_t)d[21] << 3)                          & 0x07FFU);

    /* 标志位 */
    uint8_t flags = buf[SBUS_IDX_FLAGS];

    out->ch17       = (flags & SBUS_FLAG_CH17)       ? 1U : 0U;
    out->ch18       = (flags & SBUS_FLAG_CH18)       ? 1U : 0U;
    out->frame_lost = (flags & SBUS_FLAG_FRAME_LOST) ? 1U : 0U;
    out->failsafe   = (flags & SBUS_FLAG_FAILSAFE)   ? 1U : 0U;
}

/* ========================== 串口事件回调（内部私有） ========================== */

static void SBUS_UART_EventCallback(USARTInstance *ins,
                                    USART_Event_e event,
                                    uint8_t *data_ptr,
                                    uint16_t data_len)
{
    (void)ins;

    switch (event)
    {
    case USART_EVENT_RX_CPLT:
        if (data_ptr == NULL || data_len == 0U)
        {
            return;
        }

        /*
         * ISR 中只做一次原始帧拷贝
         * 不做协议解析、不做通道计算、不做发布
         */
        SeqLock_WriteBegin(&sbus_instance.data_lock);

        sbus_instance.raw_len = (data_len > SBUS_FRAME_LEN) ? SBUS_FRAME_LEN : data_len;
        memcpy(sbus_instance.raw_rx_buf, data_ptr, sbus_instance.raw_len);
        sbus_instance.capture_timestamp = Bsp_Timestamp_us_Get();
        sbus_instance.rx_frame_count++;

        SeqLock_WriteEnd(&sbus_instance.data_lock);
        break;

    case USART_EVENT_ERROR:
        sbus_instance.rx_err_count++;
        break;

    case USART_EVENT_TX_CPLT:
    default:
        break;
    }
}

/* ========================== 任务级处理 ========================== */

uint8_t SBUS_Task_Handler(void)
{
    uint32_t   start_seq;
    uint32_t   frame_count;
    uint16_t   raw_len;
    uint64_t   timestamp;
    uint8_t    local_buf[SBUS_FRAME_LEN];
    uint8_t    retry;
    SBUS_Data_t data;

    if (sbus_instance.publisher == NULL)
    {
        return 0U;
    }

    for (retry = 0U; retry < SBUS_SEQLOCK_MAX_RETRY; retry++)
    {
        /* ========== 阶段 1：在序列锁保护下获取稳定快照 ========== */

        start_seq = SeqLock_ReadBegin(&sbus_instance.data_lock);

        frame_count = sbus_instance.rx_frame_count;
        raw_len     = sbus_instance.raw_len;
        timestamp   = sbus_instance.capture_timestamp;

        if (raw_len > SBUS_FRAME_LEN)
        {
            raw_len = SBUS_FRAME_LEN;
        }

        if (raw_len > 0U)
        {
            memcpy(local_buf, sbus_instance.raw_rx_buf, raw_len);
        }

        if (SeqLock_ReadRetry(&sbus_instance.data_lock, start_seq))
        {
            continue;
        }

        /* ========== 阶段 2：基于本地快照做校验 ========== */

        /* 检查：是否有新帧 */
        if ((frame_count == 0U) ||
            (frame_count == sbus_instance.last_proc_frame_count))
        {
            return 0U;
        }

        /* 检查：帧长度 */
        if (raw_len != SBUS_FRAME_LEN)
        {
            sbus_instance.parse_err_count++;
            return 0U;
        }

        /* 检查：帧头 */
        if (local_buf[SBUS_IDX_HEADER] != SBUS_HEADER)
        {
            sbus_instance.parse_err_count++;
            return 0U;
        }

        /* 检查：帧尾 */
        if (local_buf[SBUS_IDX_FOOTER] != SBUS_FOOTER)
        {
            sbus_instance.parse_err_count++;
            return 0U;
        }

        /* ========== 阶段 3：所有校验通过，解析 + 发布 ========== */

        SBUS_ParseRawFrame(local_buf, &data);
        data.Sbus_Timestamp = timestamp;

        sbus_instance.last_proc_frame_count = frame_count;
        PubPushMessage(sbus_instance.publisher, &data);
        return 1U;
    }

    return 0U;
}

/* ========================== 初始化 ========================== */

uint8_t SBUS_Init(void)
{
    USART_Init_Config_s config;

    memset(&sbus_instance, 0, sizeof(SBUS_Instance_t));
    SeqLock_Init(&sbus_instance.data_lock);

    /* 注册消息中心发布者 */
    sbus_instance.publisher = PubRegister(SBUS_TOPIC_NAME, sizeof(SBUS_Data_t));
    if (sbus_instance.publisher == NULL)
    {
        RTTERROR("[SBUS] PubRegister Failed!");
        return 0U;
    }

    /* 注册串口，DMA Normal 模式，固定帧长 */
    memset(&config, 0, sizeof(config));
    config.usart_handle   = &SBUS_UART_HANDLE;
    config.recv_buff_size = SBUS_FRAME_LEN;
    config.event_callback = SBUS_UART_EventCallback;
    config.rx_mode        = USART_RX_MODE_NORMAL;

    sbus_instance.usart_instance = USARTRegister(&config);
    if (sbus_instance.usart_instance == NULL)
    {
        RTTERROR("[SBUS] UART Register Failed!");
        return 0U;
    }

    RTTINFO("[SBUS] SBUS Receiver Init Success!");
    return 1U;
}