//
// Created by Administrator on 2026/6/18.
//

#include "mudules_Microvoid_MG.h"
#include <string.h>
#include "bsp_RTT.h"
#include "bsp_timestamp.h"

/* ========================== 全局静态实例 ========================== */
static GPS_Instance_t gps_instance;
/* ========================== 私有函数声明 ========================== */

static void GPS_UART_EventCallback(USARTInstance *ins, USART_Event_e event,uint8_t *data_ptr, uint16_t data_len);
static uint8_t UBX_ParseByte(UBX_Parser_t *parser, uint8_t byte);
static void UBX_ParserReset(UBX_Parser_t *parser);
static void GPS_ConvertPVT(const UBX_NAV_PVT_Payload_t *pvt, GPS_Data_t *out);
static void GPS_SendUBX(uint8_t msg_class, uint8_t msg_id,
                           const uint8_t *payload, uint16_t payload_len);
static void GPS_CfgPort(uint32_t baudrate);
static void GPS_CfgRate(uint16_t interval_ms);
static void GPS_CfgNav5(void);
static void GPS_EnableMessage(uint8_t msg_class, uint8_t msg_id, uint8_t rate);

/* ========================== UBX 解析器实现 ========================== */

static void UBX_ParserReset(UBX_Parser_t *parser)
{
    parser->state = UBX_STATE_SYNC1;
    parser->count = 0U;
}

/**
 * @brief 逐字节 UBX 协议解析
 *
 * @return 1 解析出一个完整且校验通过的帧(class/id/payload 可用)
 * @return 0 需要继续喂数据
 *
 * @note  校验算法: Fletcher-8
 *        校验范围: class + id + length(2字节) + payload
 */
static uint8_t UBX_ParseByte(UBX_Parser_t *parser, uint8_t byte)
{
    switch (parser->state)
    {
    case UBX_STATE_SYNC1:
        if (byte == UBX_SYNC1)
        {
            parser->state = UBX_STATE_SYNC2;
        }
        break;

    case UBX_STATE_SYNC2:
        if (byte == UBX_SYNC2)
        {
            parser->state = UBX_STATE_CLASS;
        }
        else
        {
            parser->state = UBX_STATE_SYNC1;
        }
        break;

    case UBX_STATE_CLASS:
        parser->msg_class = byte;
        parser->ck_a = byte;
        parser->ck_b = byte;
        parser->state = UBX_STATE_ID;
        break;

    case UBX_STATE_ID:
        parser->msg_id = byte;
        parser->ck_a += byte;
        parser->ck_b += parser->ck_a;
        parser->state = UBX_STATE_LEN1;
        break;

    case UBX_STATE_LEN1:
        parser->length = byte;
        parser->ck_a += byte;
        parser->ck_b += parser->ck_a;
        parser->state = UBX_STATE_LEN2;
        break;

    case UBX_STATE_LEN2:
        parser->length |= (uint16_t)byte << 8;
        parser->ck_a += byte;
        parser->ck_b += parser->ck_a;

        if (parser->length > UBX_MAX_PAYLOAD_LEN)
        {
            UBX_ParserReset(parser);
        }
        else if (parser->length == 0U)
        {
            parser->state = UBX_STATE_CK_A;
        }
        else
        {
            parser->count = 0U;
            parser->state = UBX_STATE_PAYLOAD;
        }
        break;

    case UBX_STATE_PAYLOAD:
        parser->payload[parser->count++] = byte;
        parser->ck_a += byte;
        parser->ck_b += parser->ck_a;

        if (parser->count >= parser->length)
        {
            parser->state = UBX_STATE_CK_A;
        }
        break;

    case UBX_STATE_CK_A:
        if (byte == parser->ck_a)
        {
            parser->state = UBX_STATE_CK_B;
        }
        else
        {
            UBX_ParserReset(parser);
        }
        break;

    case UBX_STATE_CK_B:
        UBX_ParserReset(parser);
        if (byte == parser->ck_b)
        {
            return 1U;
        }
        break;

    default:
        UBX_ParserReset(parser);
        break;
    }

    return 0U;
}

/* ========================== PVT 数据转换 ========================== */

static void GPS_ConvertPVT(const UBX_NAV_PVT_Payload_t *pvt, GPS_Data_t *out)
{
    out->latitude       = (double)pvt->lat * 1e-7;
    out->longitude      = (double)pvt->lon * 1e-7;
    out->altitude_msl   = (float)pvt->hMSL * 0.001f;
    out->altitude_ellip = (float)pvt->height * 0.001f;

    out->velN           = (float)pvt->velN * 0.001f;
    out->velE           = (float)pvt->velE * 0.001f;
    out->velD           = (float)pvt->velD * 0.001f;
    out->ground_speed   = (float)pvt->gSpeed * 0.001f;
    out->heading        = (float)pvt->headMot * 1e-5f;

    out->hAcc           = (float)pvt->hAcc * 0.001f;
    out->vAcc           = (float)pvt->vAcc * 0.001f;
    out->sAcc           = (float)pvt->sAcc * 0.001f;
    out->headAcc        = (float)pvt->headAcc * 1e-5f;
    out->pDOP           = (float)pvt->pDOP * 0.01f;

    out->fix_type       = pvt->fixType;
    out->num_sv         = pvt->numSV;
    out->fix_flags      = pvt->flags;

    out->year           = pvt->year;
    out->month          = pvt->month;
    out->day            = pvt->day;
    out->hour           = pvt->hour;
    out->min            = pvt->min;
    out->sec            = pvt->sec;
    out->time_valid     = pvt->valid;
}

/* ========================== UBX 帧发送 ========================== */

/**
 * @brief 通用 UBX 帧构建 + 阻塞发送
 *
 * @note 帧结构: [SYNC1][SYNC2][CLASS][ID][LEN_L][LEN_H][PAYLOAD...][CK_A][CK_B]
 *       校验范围: CLASS + ID + LEN(2) + PAYLOAD
 */
static void GPS_SendUBX(uint8_t msg_class, uint8_t msg_id,const uint8_t *payload, uint16_t payload_len)
{
    uint8_t frame[64];
    uint16_t total_len;
    uint16_t i;
    uint8_t ck_a = 0U;
    uint8_t ck_b = 0U;

    if (gps_instance.usart_instance == NULL)
    {
        return;
    }

    total_len = 6U + payload_len + 2U;
    if (total_len > sizeof(frame))
    {
        return;
    }

    frame[0] = UBX_SYNC1;
    frame[1] = UBX_SYNC2;
    frame[2] = msg_class;
    frame[3] = msg_id;
    frame[4] = (uint8_t)(payload_len & 0xFFU);
    frame[5] = (uint8_t)((payload_len >> 8) & 0xFFU);

    if (payload != NULL && payload_len > 0U)
    {
        memcpy(&frame[6], payload, payload_len);
    }

    for (i = 2U; i < (6U + payload_len); i++)
    {
        ck_a += frame[i];
        ck_b += ck_a;
    }

    frame[6U + payload_len]      = ck_a;
    frame[6U + payload_len + 1U] = ck_b;

    USARTSend(gps_instance.usart_instance, frame, total_len, USART_TRANSFER_BLOCKING);
}

/* ========================== UBX 配置命令 ========================== */

/**
 * @brief 配置 UART 端口
 *        输入协议: UBX + NMEA + RTCM
 *        输出协议: 仅 UBX
 */
static void GPS_CfgPort(uint32_t baudrate)
{
    uint8_t pl[20];
    memset(pl, 0, sizeof(pl));

    pl[0]  = 0x01U;                              /* portID: UART1 */
    pl[4]  = 0xD0U;                              /* mode: 8N1 */
    pl[5]  = 0x08U;
    pl[8]  = (uint8_t)(baudrate);
    pl[9]  = (uint8_t)(baudrate >> 8);
    pl[10] = (uint8_t)(baudrate >> 16);
    pl[11] = (uint8_t)(baudrate >> 24);
    pl[12] = 0x07U;                              /* inProtoMask: UBX+NMEA+RTCM */
    pl[14] = 0x01U;                              /* outProtoMask: UBX */

    GPS_SendUBX(UBX_CFG_CLASS, UBX_CFG_PRT, pl, 20U);
}

/**
 * @brief 配置输出速率
 *
 * @param interval_ms 输出间隔 ms (125 = 8Hz)
 *
 * @note 当 >= 10Hz 时, M9 限制导航卫星数为 16 颗
 *       8Hz 及以下可使用最大 32 颗卫星
 */
static void GPS_CfgRate(uint16_t interval_ms)
{
    uint8_t pl[6];

    pl[0] = (uint8_t)(interval_ms);
    pl[1] = (uint8_t)(interval_ms >> 8);
    pl[2] = 0x01U;                                /* navRate: 1 */
    pl[3] = 0x00U;
    pl[4] = 0x01U;                                /* timeRef: GPS time */
    pl[5] = 0x00U;

    GPS_SendUBX(UBX_CFG_CLASS, UBX_CFG_RATE, pl, 6U);
}

/**
 * @brief 配置导航模式为 AIRBORNE_2G
 *
 * @note 无人机必须使用 AIRBORNE 模式, 否则高速机动时定位会跑飞
 */
static void GPS_CfgNav5(void)
{
    uint8_t pl[36];
    memset(pl, 0, sizeof(pl));

    pl[0]  = 0xFFU;                               /* mask */
    pl[1]  = 0xFFU;
    pl[2]  = 7U;                                  /* dynModel: AIRBORNE_2G */
    pl[3]  = 0x03U;                               /* fixMode: auto 2D/3D */
    pl[12] = 0x0AU;                               /* minElev: 10 deg */
    pl[14] = (uint8_t)(250U);                     /* pDop */
    pl[15] = (uint8_t)(250U >> 8);
    pl[16] = (uint8_t)(250U);                     /* tDop */
    pl[17] = (uint8_t)(250U >> 8);
    pl[18] = (uint8_t)(100U);                     /* pAcc */
    pl[19] = (uint8_t)(100U >> 8);
    pl[20] = (uint8_t)(350U);                     /* tAcc */
    pl[21] = (uint8_t)(350U >> 8);
    pl[23] = 60U;                                 /* dgpsTimeOut */
    pl[25] = 25U;                                 /* cnoThresh */

    GPS_SendUBX(UBX_CFG_CLASS, UBX_CFG_NAV5, pl, 36U);
}

/**
 * @brief 使能/禁用指定消息输出
 *
 * @param rate 0=禁用, 1=每周期输出一次(最快), 值越大输出越慢
 */
static void GPS_EnableMessage(uint8_t msg_class, uint8_t msg_id, uint8_t rate)
{
    uint8_t pl[3];

    pl[0] = msg_class;
    pl[1] = msg_id;
    pl[2] = rate;

    GPS_SendUBX(UBX_CFG_CLASS, UBX_CFG_MSG, pl, 3U);
}

/* ========================== 串口事件回调（内部私有） ========================== */

static void GPS_UART_EventCallback(USARTInstance *ins,USART_Event_e event,uint8_t *data_ptr,uint16_t data_len)
{
    switch (event)
    {
    case USART_EVENT_RX_CPLT:
        if (data_ptr == NULL || data_len == 0U)
        {
            return;
        }

        /*
         * ISR 中只做一次原始数据拷贝
         * 不做协议解析、不做状态机、不做发布
         */
        SeqLock_WriteBegin(&gps_instance.data_lock);

        gps_instance.raw_len = (data_len > GPS_RX_BUF_SIZE) ? GPS_RX_BUF_SIZE : data_len;
        memcpy(gps_instance.raw_rx_buf, data_ptr, gps_instance.raw_len);
        gps_instance.capture_timestamp = Bsp_Timestamp_us_Get();
        gps_instance.rx_frame_count++;

        SeqLock_WriteEnd(&gps_instance.data_lock);
        break;

    case USART_EVENT_ERROR:
        gps_instance.rx_err_count++;
        break;

    case USART_EVENT_TX_CPLT:
    default:
        break;
    }
}

/* ========================== 任务级处理 ========================== */

/**
 * @brief GPS 任务处理
 *
 * @note 与 MTF02/SBUS 不同, GPS 使用 UBX 变长协议,
 *       需要状态机逐字节解析而非直接帧校验.
 *
 *       流程:
 *       1. SeqLock 保护下快照 raw_rx_buf 到本地
 *       2. 逐字节喂给 UBX 状态机
 *       3. 状态机解析出完整 NAV-PVT 帧后, 转换数据并发布
 *
 *       状态机保存在实例中, 跨多次 Task_Handler 调用仍能正确拼接帧
 */
uint8_t GPS_Task_Handler(void)
{
    uint32_t start_seq;
    uint32_t frame_count;
    uint16_t raw_len;
    uint64_t timestamp;
    uint8_t  local_buf[GPS_RX_BUF_SIZE];
    uint8_t  retry;
    uint16_t i;
    uint8_t  published = 0U;

    if (gps_instance.publisher == NULL)
    {
        return 0U;
    }

    for (retry = 0U; retry < GPS_SEQLOCK_MAX_RETRY; retry++)
    {
        /* ========== 阶段 1: SeqLock 保护下获取稳定快照 ========== */

        start_seq = SeqLock_ReadBegin(&gps_instance.data_lock);

        frame_count = gps_instance.rx_frame_count;
        raw_len     = gps_instance.raw_len;
        timestamp   = gps_instance.capture_timestamp;

        if (raw_len > GPS_RX_BUF_SIZE)
        {
            raw_len = GPS_RX_BUF_SIZE;
        }

        if (raw_len > 0U)
        {
            memcpy(local_buf, gps_instance.raw_rx_buf, raw_len);
        }

        if (SeqLock_ReadRetry(&gps_instance.data_lock, start_seq))
        {
            continue;
        }

        /* ========== 阶段 2: 基于本地快照处理 ========== */

        if ((frame_count == 0U) ||
            (frame_count == gps_instance.last_proc_frame_count))
        {
            return 0U;
        }

        gps_instance.last_proc_frame_count = frame_count;

        /* ========== 阶段 3: 逐字节喂给 UBX 状态机 ========== */

        for (i = 0U; i < raw_len; i++)
        {
            if (UBX_ParseByte(&gps_instance.parser, local_buf[i]))
            {
                /* 解析出一个完整且校验通过的帧 */
                if (gps_instance.parser.msg_class == UBX_NAV_CLASS &&
                    gps_instance.parser.msg_id    == UBX_NAV_PVT   &&
                    gps_instance.parser.length    == UBX_PVT_PAYLOAD_LEN)
                {
                    GPS_Data_t data;
                    UBX_NAV_PVT_Payload_t pvt;

                    memcpy(&pvt, gps_instance.parser.payload, sizeof(pvt));
                    GPS_ConvertPVT(&pvt, &data);
                    data.GPS_Timestamp = timestamp;

                    PubPushMessage(gps_instance.publisher, &data);
                    gps_instance.pvt_count++;
                    published = 1U;
                }
            }
        }

        return published;
    }

    return 0U;
}

/* ========================== 初始化 ========================== */

uint8_t GPS_Init(void)
{
    USART_Init_Config_s config;

    memset(&gps_instance, 0, sizeof(GPS_Instance_t));
    SeqLock_Init(&gps_instance.data_lock);
    UBX_ParserReset(&gps_instance.parser);

    /* 注册消息中心发布者 */
    gps_instance.publisher = PubRegister(GPS_TOPIC_NAME, sizeof(GPS_Data_t));
    if (gps_instance.publisher == NULL)
    {
        RTTERROR("[GPS] PubRegister Failed!");
        return 0U;
    }

    /* 注册 UART, DMA Normal 模式 */
    memset(&config, 0, sizeof(config));
    config.usart_handle   = &GPS_UART_HANDLE;
    config.recv_buff_size = GPS_RX_BUF_SIZE;
    config.event_callback = GPS_UART_EventCallback;
    config.rx_mode        = USART_RX_MODE_NORMAL;

    gps_instance.usart_instance = USARTRegister(&config);
    if (gps_instance.usart_instance == NULL)
    {
        RTTERROR("[GPS] UART Register Failed!");
        return 0U;
    }

    /*
     * GPS 模块上电后需等待至少 300ms 才能接受配置
     *
     * 微空模组出厂默认: UBX 协议, NAV-PVT, 115200
     * 以下配置确保:
     *   - 输出协议仅 UBX
     *   - 输出速率 8Hz (125ms)
     *   - 导航模式 AIRBORNE_2G (无人机必须)
     *   - 仅输出 NAV-PVT 语句
     */
    Bsp_Delay_ms(300);

    GPS_CfgPort(115200U);
    Bsp_Delay_ms(5);

    GPS_CfgRate(125U);
    Bsp_Delay_ms(5);

    GPS_CfgNav5();
    Bsp_Delay_ms(5);

    GPS_EnableMessage(UBX_NAV_CLASS, UBX_NAV_PVT, 1U);
    Bsp_Delay_ms(5);

    RTTINFO("[GPS] UBX GPS Init Success! (AIRBORNE_2G, 8Hz, PVT)");
    return 1U;
}