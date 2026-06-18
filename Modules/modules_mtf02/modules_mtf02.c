//
// Created by Administrator on 2026/6/15.
//
#include "modules_mtf02.h"
#include <string.h>
#include "bsp_RTT.h"
#include "bsp_timestamp.h"
#include "bsp_utils_seqlock.h"
#include "usart.h"

/* ========================== 全局静态实例 ========================== */

static MTF02_Instance_t mtf02_instance;

/* ========================== 私有函数声明 ========================== */

static uint8_t MTF02_Checksum(const uint8_t *buf);
static uint16_t MTF02_ReadU16LE(const uint8_t *p);
static int16_t MTF02_ReadS16LE(const uint8_t *p);
static uint32_t MTF02_ReadU32LE(const uint8_t *p);
static void MTF02_ParseRawFrame(const uint8_t *buf, MTF02_Data_t *out);
static void MTF02_UART_EventCallback(USARTInstance *ins,USART_Event_e event,uint8_t *data_ptr,uint16_t data_len);

/* ========================== 私有函数实现 ========================== */

static uint8_t MTF02_Checksum(const uint8_t *buf)
{
    uint8_t sum = 0U;
    uint8_t i;

    for (i = 0U; i < (MTF02_FRAME_LEN - 1U); i++)
    {
        sum += buf[i];
    }
    return sum;
}

static uint16_t MTF02_ReadU16LE(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] |
                     ((uint16_t)p[1] << 8));
}

static int16_t MTF02_ReadS16LE(const uint8_t *p)
{
    return (int16_t)MTF02_ReadU16LE(p);
}

static uint32_t MTF02_ReadU32LE(const uint8_t *p)
{
    return ((uint32_t)p[0]      ) |
           ((uint32_t)p[1] <<  8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static void MTF02_ParseRawFrame(const uint8_t *buf, MTF02_Data_t *out)
{
    out->dev_id  = buf[MTF02_IDX_DEV_ID];
    out->sys_id  = buf[MTF02_IDX_SYS_ID];
    out->msg_id  = buf[MTF02_IDX_MSG_ID];
    out->seq     = buf[MTF02_IDX_SEQ];

    out->sensor_time_ms = MTF02_ReadU32LE(&buf[MTF02_IDX_PAYLOAD + MTF02_PL_TIME_MS]);
    out->distance_mm    = MTF02_ReadU32LE(&buf[MTF02_IDX_PAYLOAD + MTF02_PL_DISTANCE]);

    out->strength       = buf[MTF02_IDX_PAYLOAD + MTF02_PL_STRENGTH];
    out->precision      = buf[MTF02_IDX_PAYLOAD + MTF02_PL_PRECISION];
    out->tof_status     = buf[MTF02_IDX_PAYLOAD + MTF02_PL_TOF_STATUS];

    out->flow_vel_x     = MTF02_ReadS16LE(&buf[MTF02_IDX_PAYLOAD + MTF02_PL_FLOW_VEL_X]);
    out->flow_vel_y     = MTF02_ReadS16LE(&buf[MTF02_IDX_PAYLOAD + MTF02_PL_FLOW_VEL_Y]);
    out->flow_quality   = buf[MTF02_IDX_PAYLOAD + MTF02_PL_FLOW_QUALITY];
    out->flow_status    = buf[MTF02_IDX_PAYLOAD + MTF02_PL_FLOW_STATUS];
}

/* ========================== 串口事件回调（内部私有） ========================== */

static void MTF02_UART_EventCallback(USARTInstance *ins,USART_Event_e event,uint8_t *data_ptr,uint16_t data_len)
{
    (void)ins;

    switch (event)
    {
    case USART_EVENT_RX_CPLT:
        if (data_ptr == NULL || data_len == 0U)
        {
            return;
        }

        SeqLock_WriteBegin(&mtf02_instance.data_lock);

        mtf02_instance.raw_len = (data_len > MTF02_FRAME_LEN) ? MTF02_FRAME_LEN : data_len;
        memcpy(mtf02_instance.raw_rx_buf, data_ptr, mtf02_instance.raw_len);
        mtf02_instance.capture_timestamp = Bsp_Timestamp_us_Get();
        mtf02_instance.rx_frame_count++;

        SeqLock_WriteEnd(&mtf02_instance.data_lock);
        break;

    case USART_EVENT_ERROR:
        mtf02_instance.rx_err_count++;
        break;

    case USART_EVENT_TX_CPLT:
    default:
        break;
    }
}

/* ========================== 任务级处理 ========================== */

uint8_t MTF02_Task_Handler(void)
{
    uint32_t start_seq;
    uint32_t frame_count;
    uint16_t raw_len;
    uint64_t timestamp;
    uint8_t  local_buf[MTF02_FRAME_LEN];
    uint8_t  checksum;
    uint8_t  retry;
    MTF02_Data_t data = {0};

    if (mtf02_instance.publisher == NULL)
    {
        return 0U;
    }

    for (retry = 0U; retry < MTF02_SEQLOCK_MAX_RETRY; retry++)
    {
        /* ---------- 在序列锁保护下获取稳定快照 ---------- */
        start_seq = SeqLock_ReadBegin(&mtf02_instance.data_lock);

        frame_count = mtf02_instance.rx_frame_count;
        raw_len     = mtf02_instance.raw_len;
        timestamp   = mtf02_instance.capture_timestamp;

        if (raw_len > MTF02_FRAME_LEN)
        {
            raw_len = MTF02_FRAME_LEN;
        }

        if (raw_len > 0U)
        {
            memcpy(local_buf, mtf02_instance.raw_rx_buf, raw_len);
        }

        if (SeqLock_ReadRetry(&mtf02_instance.data_lock, start_seq))
        {
            continue;
        }

        /* ========================================================
         * 快照一致，后续全部基于本地数据处理
         * ======================================================== */

        /* ---- 1. 是否有新帧需要处理 ---- */
        if ((frame_count == 0U) ||
            (frame_count == mtf02_instance.last_seen_frame_count))
        {
            return 0U;
        }

        /*
         * 走到这里说明有新帧，无论后面解析成功还是失败，
         * 都要标记"已看过这帧"，防止同一个坏帧被反复重试。
         *
         * 但 last_proc_frame_count 只在成功发布后才更新。
         */
        mtf02_instance.last_seen_frame_count = frame_count;

        /* ---- 2. 帧长度检查 ---- */
        if (raw_len != MTF02_FRAME_LEN)
        {
            mtf02_instance.parse_err_count++;
            return 0U;
        }

        /* ---- 3. 帧头 / 消息ID / 负载长度检查 ---- */
        if ((local_buf[MTF02_IDX_HEAD]   != MTF02_MSG_HEAD) ||
            (local_buf[MTF02_IDX_MSG_ID] != MTF02_MSG_ID_RANGE_SENSOR) ||
            (local_buf[MTF02_IDX_LEN]    != MTF02_PAYLOAD_LEN))
        {
            mtf02_instance.parse_err_count++;
            return 0U;
        }

        /* ---- 4. 校验和检查 ---- */
        checksum = MTF02_Checksum(local_buf);
        if (checksum != local_buf[MTF02_IDX_CHECKSUM])
        {
            mtf02_instance.parse_err_count++;
            return 0U;
        }

        /* ---- 5. 解析并发布 ---- */
        MTF02_ParseRawFrame(local_buf, &data);
        data.Mtf02_Timestamp = timestamp;

        mtf02_instance.last_proc_frame_count = frame_count;
        PubPushMessage(mtf02_instance.publisher, &data);
        return 1U;
    }

    return 0U;
}

/* ========================== 初始化 ========================== */

uint8_t MTF02_Init(void)
{
    USART_Init_Config_s config;

    memset(&mtf02_instance, 0, sizeof(MTF02_Instance_t));
    SeqLock_Init(&mtf02_instance.data_lock);

    mtf02_instance.publisher = PubRegister(MTF02_TOPIC_NAME, sizeof(MTF02_Data_t));
    if (mtf02_instance.publisher == NULL)
    {
        RTTERROR("[MTF02] PubRegister Failed!");
        return 0U;
    }

    memset(&config, 0, sizeof(config));
    config.usart_handle   = &huart4;
    config.recv_buff_size = MTF02_FRAME_LEN;
    config.event_callback = MTF02_UART_EventCallback;
    config.rx_mode        = USART_RX_MODE_NORMAL;

    mtf02_instance.usart_instance = USARTRegister(&config);
    if (mtf02_instance.usart_instance == NULL)
    {
        RTTERROR("[MTF02] UART Register Failed!");
        return 0U;
    }

    RTTINFO("[MTF02] MTF02 Optical Flow Init Success!");
    return 1U;
}