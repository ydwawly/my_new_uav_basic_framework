/**
 * @file modules_mtf02.c
 * @brief MTF-02 MicoLink/MSPv2 串口流解析与统一数据发布
 */

#include "modules_mtf02.h"

#include <limits.h>
#include <math.h>
#include <string.h>

#include "bsp_RTT.h"
#include "bsp_timestamp.h"
#include "usart.h"

#define MTF02_DEG_TO_RAD 0.01745329251994329577f

/* ========================== 全局静态实例 ========================== */

static MTF02_Instance_t mtf02_instance;
static TaskHandle_t     mtf02_ready_task = NULL;

/* ========================== 私有函数声明 ========================== */

static uint8_t  MTF02_AdditiveChecksum(const uint8_t *data, uint16_t length);
static uint8_t  MTF02_Crc8DvbS2(uint8_t crc, uint8_t data);
static uint16_t MTF02_ReadU16LE(const uint8_t *data);
static int16_t  MTF02_ReadS16LE(const uint8_t *data);
static uint32_t MTF02_ReadU32LE(const uint8_t *data);
static int32_t  MTF02_ReadS32LE(const uint8_t *data);
static void     MTF02_NotifyReadyFromISR(void);
static void     MTF02_ResetMicoLinkParser(uint8_t current_byte);
static void     MTF02_ResetMspParser(uint8_t current_byte);
static void     MTF02_ResetStreamParsers(void);
static void     MTF02_AcceptMicoLinkFrame(const uint8_t *frame, uint64_t timestamp_us);
static void     MTF02_AcceptMspFrame(uint16_t command, const uint8_t *payload, uint16_t payload_size,
                                    uint64_t timestamp_us);
static void     MTF02_ConsumeMicoLinkByte(uint8_t byte, uint64_t timestamp_us);
static void     MTF02_ConsumeMspByte(uint8_t byte, uint64_t timestamp_us);
static void     MTF02_DrainRxChunks(void);
static void     MTF02_UART_EventCallback(USARTInstance *ins, USART_Event_e event, uint8_t *data_ptr, uint16_t data_len);

/* ========================== 字节序与校验 ========================== */

static uint8_t MTF02_AdditiveChecksum(const uint8_t *data, uint16_t length)
{
    uint8_t checksum = 0U;

    for (uint16_t index = 0U; index < length; index++)
    {
        checksum = (uint8_t)(checksum + data[index]);
    }

    return checksum;
}

/**
 * @brief MSPv2 使用的 CRC-8/DVB-S2，生成多项式为 0xD5
 */
static uint8_t MTF02_Crc8DvbS2(uint8_t crc, uint8_t data)
{
    crc ^= data;

    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
        crc = ((crc & 0x80U) != 0U) ? (uint8_t)((crc << 1U) ^ 0xD5U) : (uint8_t)(crc << 1U);
    }

    return crc;
}

static uint16_t MTF02_ReadU16LE(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static int16_t MTF02_ReadS16LE(const uint8_t *data)
{
    return (int16_t)MTF02_ReadU16LE(data);
}

static uint32_t MTF02_ReadU32LE(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) | ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

static int32_t MTF02_ReadS32LE(const uint8_t *data)
{
    return (int32_t)MTF02_ReadU32LE(data);
}

/* ========================== ISR 辅助函数 ========================== */

static void MTF02_NotifyReadyFromISR(void)
{
    if (mtf02_ready_task != NULL)
    {
        BaseType_t higher_priority_task_woken = pdFALSE;

        xTaskNotifyFromISR(mtf02_ready_task, NOTIFY_BIT_MTF02, eSetBits, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

static void MTF02_ResetMicoLinkParser(uint8_t current_byte)
{
    MTF02_MicoLinkParser_t *parser = &mtf02_instance.micolink_parser;

    parser->length = 0U;
    if (current_byte == MTF02_MICOLINK_HEAD)
    {
        parser->buffer[0] = current_byte;
        parser->length    = 1U;
    }
}

static void MTF02_ResetMspParser(uint8_t current_byte)
{
    MTF02_MSPParser_t *parser = &mtf02_instance.msp_parser;

    memset(parser, 0, sizeof(*parser));
    if (current_byte == (uint8_t)'$')
    {
        parser->state = MTF02_MSP_STATE_HEADER_X;
    }
}

/**
 * @brief 在任务上下文复位两个流解析器
 *
 * UART 错误或槽池溢出后，前后两段数据不再连续。此时不能让状态机继续使用
 * 断点前的半帧，否则可能把两段数据误拼成一帧。
 */
static void MTF02_ResetStreamParsers(void)
{
    MTF02_ResetMicoLinkParser(0U);
    MTF02_ResetMspParser(0U);
    mtf02_instance.last_msp_flow_timestamp_us = 0ULL;
}

/* ========================== 合法帧入库 ========================== */

static void MTF02_AcceptMicoLinkFrame(const uint8_t *frame, uint64_t timestamp_us)
{
    const uint8_t *payload = &frame[MTF02_MICOLINK_IDX_PAYLOAD];

    SeqLock_WriteBegin(&mtf02_instance.data_lock);

    MTF02_Data_t *data = &mtf02_instance.latest_data;

    data->protocol       = MTF02_PROTOCOL_MICOLINK;
    data->range_protocol = MTF02_PROTOCOL_MICOLINK;
    data->flow_protocol  = MTF02_PROTOCOL_MICOLINK;
    data->dev_id         = frame[1];
    data->sys_id         = frame[2];
    data->msg_id         = frame[3];
    data->seq            = frame[4];

    data->sensor_time_ms = MTF02_ReadU32LE(&payload[0]);
    data->distance_mm    = MTF02_ReadU32LE(&payload[4]);
    data->strength       = payload[8];
    data->range_quality  = payload[8];
    data->precision      = payload[9];
    data->tof_status     = payload[10];

    data->flow_vel_x         = MTF02_ReadS16LE(&payload[12]);
    data->flow_vel_y         = MTF02_ReadS16LE(&payload[14]);
    data->motion_x           = 0;
    data->motion_y           = 0;
    data->flow_rate_rad_s[0] = 0.0f;
    data->flow_rate_rad_s[1] = 0.0f;
    data->flow_delta_time_us = 0U;
    data->flow_quality       = payload[16];
    data->flow_status        = payload[17];

    data->range_timestamp_us = timestamp_us;
    data->flow_timestamp_us  = timestamp_us;
    data->Mtf02_Timestamp    = timestamp_us;

    mtf02_instance.rx_frame_count++;
    mtf02_instance.micolink_frame_count++;

    SeqLock_WriteEnd(&mtf02_instance.data_lock);
}

static void MTF02_AcceptMspFrame(uint16_t command, const uint8_t *payload, uint16_t payload_size,
                                uint64_t timestamp_us)
{
    if ((command == MTF02_MSP2_SENSOR_RANGEFINDER) && (payload_size == MTF02_MSP_RANGE_PAYLOAD_LEN))
    {
        const int32_t distance_mm = MTF02_ReadS32LE(&payload[1]);

        SeqLock_WriteBegin(&mtf02_instance.data_lock);

        MTF02_Data_t *data = &mtf02_instance.latest_data;

        data->protocol           = MTF02_PROTOCOL_MSP_V2;
        data->range_protocol     = MTF02_PROTOCOL_MSP_V2;
        data->distance_mm        = (distance_mm > 0) ? (uint32_t)distance_mm : 0U;
        data->range_quality      = payload[0];
        data->strength           = 0U;
        data->precision          = 0U;
        data->tof_status         = (distance_mm > 0) ? 0U : 1U;
        data->range_timestamp_us = timestamp_us;
        data->Mtf02_Timestamp    = timestamp_us;

        mtf02_instance.rx_frame_count++;
        mtf02_instance.msp_range_frame_count++;

        SeqLock_WriteEnd(&mtf02_instance.data_lock);
        return;
    }

    if ((command == MTF02_MSP2_SENSOR_OPTICAL_FLOW) && (payload_size == MTF02_MSP_FLOW_PAYLOAD_LEN))
    {
        uint32_t delta_time_us = 0U;

        if ((mtf02_instance.last_msp_flow_timestamp_us != 0ULL) &&
            (timestamp_us > mtf02_instance.last_msp_flow_timestamp_us))
        {
            const uint64_t delta = timestamp_us - mtf02_instance.last_msp_flow_timestamp_us;
            delta_time_us        = (delta <= UINT32_MAX) ? (uint32_t)delta : 0U;
        }

        mtf02_instance.last_msp_flow_timestamp_us = timestamp_us;

        SeqLock_WriteBegin(&mtf02_instance.data_lock);

        MTF02_Data_t *data = &mtf02_instance.latest_data;

        data->protocol           = MTF02_PROTOCOL_MSP_V2;
        data->flow_protocol      = MTF02_PROTOCOL_MSP_V2;
        data->motion_x           = MTF02_ReadS32LE(&payload[1]);
        data->motion_y           = MTF02_ReadS32LE(&payload[5]);
        data->flow_vel_x         = 0;
        data->flow_vel_y         = 0;
        data->flow_delta_time_us = delta_time_us;
        data->flow_quality       = payload[0];
        data->flow_status        = 0U;
        data->flow_timestamp_us  = timestamp_us;
        data->Mtf02_Timestamp    = timestamp_us;

        mtf02_instance.rx_frame_count++;
        mtf02_instance.msp_flow_frame_count++;

        SeqLock_WriteEnd(&mtf02_instance.data_lock);
    }
}

/* ========================== 串口流状态机 ========================== */

static void MTF02_ConsumeMicoLinkByte(uint8_t byte, uint64_t timestamp_us)
{
    MTF02_MicoLinkParser_t *parser = &mtf02_instance.micolink_parser;

    if (parser->length == 0U)
    {
        if (byte == MTF02_MICOLINK_HEAD)
        {
            parser->buffer[0] = byte;
            parser->length    = 1U;
        }
        return;
    }

    parser->buffer[parser->length++] = byte;

    if (((parser->length == 4U) && (parser->buffer[3] != MTF02_MICOLINK_MSG_ID)) ||
        ((parser->length == 6U) && (parser->buffer[5] != MTF02_MICOLINK_PAYLOAD_LEN)))
    {
        mtf02_instance.format_error_count++;
        mtf02_instance.sync_drop_count++;
        MTF02_ResetMicoLinkParser(byte);
        return;
    }

    if (parser->length < MTF02_MICOLINK_FRAME_LEN)
    {
        return;
    }

    if (MTF02_AdditiveChecksum(parser->buffer, MTF02_MICOLINK_FRAME_LEN - 1U) ==
        parser->buffer[MTF02_MICOLINK_IDX_CHECKSUM])
    {
        MTF02_AcceptMicoLinkFrame(parser->buffer, timestamp_us);
    }
    else
    {
        mtf02_instance.crc_error_count++;
    }

    MTF02_ResetMicoLinkParser(0U);
}

/**
 * @brief 向MSPv2显式状态机输入一个字节
 *
 * @note 本函数有意保持完整状态机结构：每个case对应协议中的唯一接收阶段，
 *       集中排列比拆成多个小函数更容易核对状态转移、CRC覆盖范围和异常复位路径。
 */
static void MTF02_ConsumeMspByte(uint8_t byte, uint64_t timestamp_us)
{
    MTF02_MSPParser_t *parser = &mtf02_instance.msp_parser;

    switch ((MTF02_MSPParserState_e)parser->state)
    {
    case MTF02_MSP_STATE_IDLE:
        if (byte == (uint8_t)'$')
        {
            parser->state = MTF02_MSP_STATE_HEADER_X;
        }
        break;

    case MTF02_MSP_STATE_HEADER_X:
        if (byte == (uint8_t)'X')
        {
            parser->state = MTF02_MSP_STATE_DIRECTION;
        }
        else
        {
            MTF02_ResetMspParser(byte);
        }
        break;

    case MTF02_MSP_STATE_DIRECTION:
        if (byte == (uint8_t)'<')
        {
            parser->state = MTF02_MSP_STATE_FLAGS;
            parser->crc   = 0U;
        }
        else
        {
            MTF02_ResetMspParser(byte);
        }
        break;

    case MTF02_MSP_STATE_FLAGS:
        parser->flags = byte;
        parser->crc   = MTF02_Crc8DvbS2(parser->crc, byte);
        parser->state = MTF02_MSP_STATE_COMMAND_L;
        break;

    case MTF02_MSP_STATE_COMMAND_L:
        parser->command = byte;
        parser->crc     = MTF02_Crc8DvbS2(parser->crc, byte);
        parser->state   = MTF02_MSP_STATE_COMMAND_H;
        break;

    case MTF02_MSP_STATE_COMMAND_H:
        parser->command |= (uint16_t)byte << 8U;
        parser->crc   = MTF02_Crc8DvbS2(parser->crc, byte);
        parser->state = MTF02_MSP_STATE_SIZE_L;
        break;

    case MTF02_MSP_STATE_SIZE_L:
        parser->payload_size = byte;
        parser->crc          = MTF02_Crc8DvbS2(parser->crc, byte);
        parser->state        = MTF02_MSP_STATE_SIZE_H;
        break;

    case MTF02_MSP_STATE_SIZE_H:
        parser->payload_size |= (uint16_t)byte << 8U;
        parser->crc = MTF02_Crc8DvbS2(parser->crc, byte);

        if (parser->payload_size > MTF02_MSP_MAX_PAYLOAD_LEN)
        {
            mtf02_instance.format_error_count++;
            MTF02_ResetMspParser(byte);
        }
        else
        {
            parser->payload_index = 0U;
            parser->state         = (parser->payload_size == 0U) ? MTF02_MSP_STATE_CHECKSUM : MTF02_MSP_STATE_PAYLOAD;
        }
        break;

    case MTF02_MSP_STATE_PAYLOAD:
        parser->payload[parser->payload_index++] = byte;
        parser->crc                              = MTF02_Crc8DvbS2(parser->crc, byte);
        if (parser->payload_index >= parser->payload_size)
        {
            parser->state = MTF02_MSP_STATE_CHECKSUM;
        }
        break;

    case MTF02_MSP_STATE_CHECKSUM:
        if (byte == parser->crc)
        {
            MTF02_AcceptMspFrame(parser->command, parser->payload, parser->payload_size, timestamp_us);
        }
        else
        {
            mtf02_instance.crc_error_count++;
        }
        MTF02_ResetMspParser(0U);
        break;

    default:
        mtf02_instance.format_error_count++;
        MTF02_ResetMspParser(byte);
        break;
    }
}

/**
 * @brief 在 SensorHub 任务上下文中排空当前已提交的原始 UART 数据槽
 *
 * @note ISR 必须先写完槽内容，再通过 write_count 发布；任务处理完槽内容后才
 *       推进 read_count。内存屏障保证编译器和 Cortex-M7 不会重排这两个发布点。
 */
static void MTF02_DrainRxChunks(void)
{
    uint32_t read_count  = mtf02_instance.rx_chunk_read_count;
    uint32_t write_count = mtf02_instance.rx_chunk_write_count;
    uint32_t available   = write_count - read_count;

    /* 正常情况下 available 不会超过槽池深度；此分支仅用于防御计数异常。 */
    if (available > MTF02_RX_CHUNK_QUEUE_DEPTH)
    {
        mtf02_instance.sync_drop_count++;
        read_count = write_count - MTF02_RX_CHUNK_QUEUE_DEPTH;
        available  = MTF02_RX_CHUNK_QUEUE_DEPTH;
        MTF02_ResetStreamParsers();
    }

    for (uint32_t chunk_index = 0U; chunk_index < available; chunk_index++)
    {
        MTF02_RxChunk_t *chunk =
            &mtf02_instance.rx_chunk_queue[read_count & (MTF02_RX_CHUNK_QUEUE_DEPTH - 1U)];

        __DMB();

        if (chunk->stream_epoch != mtf02_instance.processed_stream_epoch)
        {
            MTF02_ResetStreamParsers();
            mtf02_instance.processed_stream_epoch = chunk->stream_epoch;
        }

        for (uint16_t byte_index = 0U; byte_index < chunk->length; byte_index++)
        {
            MTF02_ConsumeMicoLinkByte(chunk->data[byte_index], chunk->timestamp_us);
            MTF02_ConsumeMspByte(chunk->data[byte_index], chunk->timestamp_us);
        }

        read_count++;
        __DMB();
        mtf02_instance.rx_chunk_read_count = read_count;
    }

    /* UART 错误发生后可能暂时没有新数据；排空旧槽后也应立即丢弃半帧状态。 */
    if (mtf02_instance.processed_stream_epoch != mtf02_instance.rx_stream_epoch)
    {
        MTF02_ResetStreamParsers();
        mtf02_instance.processed_stream_epoch = mtf02_instance.rx_stream_epoch;
    }
}

static void MTF02_UART_EventCallback(USARTInstance *ins, USART_Event_e event, uint8_t *data_ptr, uint16_t data_len)
{
    (void)ins;

    switch (event)
    {
    case USART_EVENT_RX_CPLT:
    {
        const uint64_t timestamp_us = Bsp_Timestamp_us_Get();
        uint32_t       write_count;
        uint32_t       used_count;

        mtf02_instance.rx_chunk_count++;
        if ((data_ptr == NULL) || (data_len == 0U))
        {
            mtf02_instance.rx_stream_epoch++;
            mtf02_instance.sync_drop_count++;
            MTF02_NotifyReadyFromISR();
            return;
        }

        if (data_len > MTF02_UART_RX_BUFFER_LEN)
        {
            mtf02_instance.rx_chunk_oversize_count++;
            mtf02_instance.rx_stream_epoch++;
            mtf02_instance.sync_drop_count++;
            MTF02_NotifyReadyFromISR();
            return;
        }

        write_count = mtf02_instance.rx_chunk_write_count;
        used_count  = write_count - mtf02_instance.rx_chunk_read_count;

        if (used_count >= MTF02_RX_CHUNK_QUEUE_DEPTH)
        {
            /* 槽池满时丢弃新块，不覆盖任务尚未读取的数据。 */
            mtf02_instance.rx_chunk_drop_count++;
            mtf02_instance.rx_stream_epoch++;
            mtf02_instance.sync_drop_count++;
            MTF02_NotifyReadyFromISR();
            return;
        }

        MTF02_RxChunk_t *chunk =
            &mtf02_instance.rx_chunk_queue[write_count & (MTF02_RX_CHUNK_QUEUE_DEPTH - 1U)];

        memcpy(chunk->data, data_ptr, data_len);
        chunk->length       = data_len;
        chunk->timestamp_us = timestamp_us;
        chunk->stream_epoch = mtf02_instance.rx_stream_epoch;

        /* 槽内容完全写好后再发布 write_count，任务不会看到半写入的数据。 */
        __DMB();
        mtf02_instance.rx_chunk_write_count = write_count + 1U;

        used_count++;
        if (used_count > mtf02_instance.rx_chunk_high_watermark)
        {
            mtf02_instance.rx_chunk_high_watermark = used_count;
        }

        MTF02_NotifyReadyFromISR();
        break;
    }

    case USART_EVENT_ERROR:
        mtf02_instance.rx_error_count++;
        mtf02_instance.rx_stream_epoch++;
        MTF02_NotifyReadyFromISR();
        break;

    case USART_EVENT_TX_CPLT:
    default:
        break;
    }
}

/* ========================== 初始化与任务级发布 ========================== */

uint8_t MTF02_Init(void)
{
    USART_Init_Config_s config;

    memset(&mtf02_instance, 0, sizeof(mtf02_instance));
    SeqLock_Init(&mtf02_instance.data_lock);

    mtf02_instance.publisher = PubRegister(MTF02_TOPIC_NAME, sizeof(MTF02_Data_t));
    if (mtf02_instance.publisher == NULL)
    {
        RTTERROR("[MTF02] PubRegister failed.");
        return 0U;
    }

    memset(&config, 0, sizeof(config));
    config.usart_handle   = &huart4;
    config.recv_buff_size = MTF02_UART_RX_BUFFER_LEN;
    config.event_callback = MTF02_UART_EventCallback;
    config.rx_mode        = USART_RX_MODE_NORMAL;

    mtf02_instance.usart_instance = USARTRegister(&config);
    if (mtf02_instance.usart_instance == NULL)
    {
        RTTERROR("[MTF02] UART register failed.");
        return 0U;
    }

    RTTINFO("[MTF02] MicoLink/MSPv2 auto-detect init success.");
    return 1U;
}

uint8_t MTF02_Task_Handler(void)
{
    MTF02_Data_t data;
    uint32_t     micolink_count;
    uint32_t     msp_range_count;
    uint32_t     msp_flow_count;

    if (mtf02_instance.publisher == NULL)
    {
        return 0U;
    }

    /* 所有协议状态机、CRC 校验和数据换算均从此处开始在任务上下文执行。 */
    MTF02_DrainRxChunks();

    for (uint8_t retry = 0U; retry < MTF02_SEQLOCK_MAX_RETRY; retry++)
    {
        const uint32_t start_seq = SeqLock_ReadBegin(&mtf02_instance.data_lock);

        data            = mtf02_instance.latest_data;
        micolink_count  = mtf02_instance.micolink_frame_count;
        msp_range_count = mtf02_instance.msp_range_frame_count;
        msp_flow_count  = mtf02_instance.msp_flow_frame_count;

        if (SeqLock_ReadRetry(&mtf02_instance.data_lock, start_seq))
        {
            continue;
        }

        data.update_flags = MTF02_UPDATE_NONE;

        if (micolink_count != mtf02_instance.processed_micolink_count)
        {
            data.update_flags |= MTF02_UPDATE_RANGE | MTF02_UPDATE_FLOW;
        }
        if (msp_range_count != mtf02_instance.processed_msp_range_count)
        {
            data.update_flags |= MTF02_UPDATE_RANGE;
        }
        if (msp_flow_count != mtf02_instance.processed_msp_flow_count)
        {
            data.update_flags |= MTF02_UPDATE_FLOW;
        }

        if (data.update_flags == MTF02_UPDATE_NONE)
        {
            return 0U;
        }

        /*
         * MSPv2 的 motion 是一帧曝光周期内的积分量。按 INAV 的定义：
         * rate_deg_s = motion * (1e6 / dt_us) / opflow_scale。
         * 首帧没有可靠 dt，保持角速度为 0，由应用层拒绝该帧并只用于建立时间基准。
         */
        if (((data.update_flags & MTF02_UPDATE_FLOW) != 0U) && (data.flow_protocol == MTF02_PROTOCOL_MSP_V2))
        {
            if ((data.flow_delta_time_us > 0U) && (MTF02_MSP_OPFLOW_SCALE > 0.0f))
            {
                const float rate_scale = (1000000.0f / (float)data.flow_delta_time_us) / MTF02_MSP_OPFLOW_SCALE *
                                         MTF02_DEG_TO_RAD;

                data.flow_rate_rad_s[0] = (float)data.motion_x * rate_scale;
                data.flow_rate_rad_s[1] = (float)data.motion_y * rate_scale;
            }
            else
            {
                data.flow_rate_rad_s[0] = 0.0f;
                data.flow_rate_rad_s[1] = 0.0f;
            }
        }

        mtf02_instance.processed_micolink_count  = micolink_count;
        mtf02_instance.processed_msp_range_count = msp_range_count;
        mtf02_instance.processed_msp_flow_count  = msp_flow_count;
        mtf02_instance.published_count++;

        (void)PubPushMessage(mtf02_instance.publisher, &data);
        return 1U;
    }

    return 0U;
}

void Mtf02_RegisterReadyTask(TaskHandle_t task_handle)
{
    mtf02_ready_task = task_handle;
}
