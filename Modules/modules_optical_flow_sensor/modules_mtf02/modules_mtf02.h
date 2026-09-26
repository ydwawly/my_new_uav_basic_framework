/**
 * @file modules_mtf02.h
 * @brief MTF-02 测距/光流模块的 MicoLink 与 MSPv2 双协议驱动
 */

#ifndef STM32H743_UAV_FLIGHT_CONTROLLER_MODULES_MTF02_H
#define STM32H743_UAV_FLIGHT_CONTROLLER_MODULES_MTF02_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "bsp_uart.h"
#include "bsp_utils_seqlock.h"
#include "modules_Message_center.h"
#include "task.h"

/* ========================== 公共配置 ========================== */

#define MTF02_TOPIC_NAME "mtf02_data"
#define NOTIFY_BIT_MTF02 (1UL << 7)

/*
 * INAV 对 MSP2_SENSOR_OPTIC_FLOW 的标定参数默认值为 10.5。
 * motion / dt / scale 得到 deg/s，再乘 DEG_TO_RAD 得到 rad/s。
 * 后续应使用实机定速平移数据重新标定该比例，而不是长期依赖默认值。
 */
#ifndef MTF02_MSP_OPFLOW_SCALE
#define MTF02_MSP_OPFLOW_SCALE 10.5f
#endif

/* ========================== 协议常量 ========================== */

#define MTF02_MICOLINK_HEAD            0xEFU
#define MTF02_MICOLINK_MSG_ID          0x51U
#define MTF02_MICOLINK_PAYLOAD_LEN     20U
#define MTF02_MICOLINK_FRAME_LEN       27U
#define MTF02_MICOLINK_IDX_PAYLOAD     6U
#define MTF02_MICOLINK_IDX_CHECKSUM    26U
#define MTF02_MSP_MAX_PAYLOAD_LEN      9U
#define MTF02_MSP_RANGE_PAYLOAD_LEN    5U
#define MTF02_MSP_FLOW_PAYLOAD_LEN     9U
#define MTF02_MSP2_SENSOR_RANGEFINDER  0x1F01U
#define MTF02_MSP2_SENSOR_OPTICAL_FLOW 0x1F02U
#define MTF02_UART_RX_BUFFER_LEN       64U
/*
 * 上电初始化阶段调度器尚未运行，MTF-02 已可能连续送来约 25 个 DMA 数据块。
 * 32 槽能够覆盖该启动积压，同时仍保持 2 的幂，便于环形索引使用位掩码。
 */
#define MTF02_RX_CHUNK_QUEUE_DEPTH 32U
#define MTF02_SEQLOCK_MAX_RETRY    3U

/* 环形槽池使用位掩码取下标，深度必须为 2 的幂。 */
#if ((MTF02_RX_CHUNK_QUEUE_DEPTH == 0U) || ((MTF02_RX_CHUNK_QUEUE_DEPTH & (MTF02_RX_CHUNK_QUEUE_DEPTH - 1U)) != 0U))
#error "MTF02_RX_CHUNK_QUEUE_DEPTH must be a power of two"
#endif

typedef enum
{
    MTF02_PROTOCOL_NONE     = 0U,
    MTF02_PROTOCOL_MICOLINK = 1U,
    MTF02_PROTOCOL_MSP_V2   = 2U
} MTF02_Protocol_e;

typedef enum
{
    MTF02_UPDATE_NONE  = 0U,
    MTF02_UPDATE_RANGE = (1UL << 0),
    MTF02_UPDATE_FLOW  = (1UL << 1)
} MTF02_UpdateFlag_e;

/**
 * @brief 驱动向消息中心发布的统一数据
 *
 * MicoLink 的 flow_vel_x/y 是“cm/s @ 1 m”；MSPv2 的 motion_x/y 是本次
 * 曝光周期内的积分光流量。驱动同时保留原始值和换算后的 rad/s，便于离线做
 * 方差、尺度因子和安装方向分析，避免丢失原始观测。
 */
typedef struct
{
    uint8_t protocol;       /* MTF02_Protocol_e，标识本次最新数据来自哪种协议 */
    uint8_t update_flags;   /* MTF02_UpdateFlag_e，本次发布包含哪些新观测 */
    uint8_t range_protocol; /* 最新测距数据所属协议 */
    uint8_t flow_protocol;  /* 最新光流数据所属协议 */
    uint8_t dev_id;         /* MicoLink 设备 ID；MSPv2 下为 0 */
    uint8_t sys_id;         /* MicoLink 系统 ID；MSPv2 下为 0 */
    uint8_t msg_id;         /* MicoLink 消息 ID；MSPv2 下为 0 */
    uint8_t seq;            /* MicoLink 帧序号；MSPv2 下为 0 */

    uint32_t sensor_time_ms; /* MicoLink 传感器时间，单位 ms；MSPv2 下为 0 */
    uint32_t distance_mm;    /* 下视测距，单位 mm；无效或超量程时为 0 */
    uint8_t  range_quality;  /* MSPv2 测距质量 0~255；MicoLink 使用 strength */
    uint8_t  strength;       /* MicoLink ToF 回波强度 */
    uint8_t  precision;      /* MicoLink 测距精度指标 */
    uint8_t  tof_status;     /* 0 表示正常；MSPv2 的负距离会映射为非 0 */

    int16_t flow_vel_x; /* MicoLink 原始 X 光流，单位 cm/s @ 1 m */
    int16_t flow_vel_y; /* MicoLink 原始 Y 光流，单位 cm/s @ 1 m */
    int32_t motion_x;   /* MSPv2 原始 X 积分光流量 */
    int32_t motion_y;   /* MSPv2 原始 Y 积分光流量 */

    float flow_rate_rad_s[2]; /* MSPv2 换算后的 X/Y 光流角速度，单位 rad/s */

    uint32_t flow_delta_time_us; /* 相邻 MSPv2 光流帧的本地时间差，单位 us */
    uint8_t  flow_quality;       /* 光流质量，0~255 */
    uint8_t  flow_status;        /* 0 表示协议层数据有效 */
    uint16_t reserved2;

    uint64_t range_timestamp_us; /* 最新测距帧的本地接收时间戳，单位 us */
    uint64_t flow_timestamp_us;  /* 最新光流帧的本地接收时间戳，单位 us */
    uint64_t Mtf02_Timestamp;    /* 本次发布时最新 MTF-02 数据时间戳，单位 us */
} MTF02_Data_t;

typedef struct
{
    uint8_t  buffer[MTF02_MICOLINK_FRAME_LEN];
    uint16_t length;
} MTF02_MicoLinkParser_t;

typedef enum
{
    MTF02_MSP_STATE_IDLE = 0U,
    MTF02_MSP_STATE_HEADER_X,
    MTF02_MSP_STATE_DIRECTION,
    MTF02_MSP_STATE_FLAGS,
    MTF02_MSP_STATE_COMMAND_L,
    MTF02_MSP_STATE_COMMAND_H,
    MTF02_MSP_STATE_SIZE_L,
    MTF02_MSP_STATE_SIZE_H,
    MTF02_MSP_STATE_PAYLOAD,
    MTF02_MSP_STATE_CHECKSUM
} MTF02_MSPParserState_e;

typedef struct
{
    uint8_t  state;
    uint8_t  flags;
    uint16_t command;
    uint16_t payload_size;
    uint16_t payload_index;
    uint8_t  payload[MTF02_MSP_MAX_PAYLOAD_LEN];
    uint8_t  crc;
} MTF02_MSPParser_t;

/**
 * @brief UART 中断与 SensorHub 任务之间传递的静态原始数据槽
 *
 * 中断只复制本次 DMA 收到的原始字节，并记录 DMA 完成时刻；协议解析、校验和
 * 数据换算全部由任务完成。stream_epoch 用于标记 UART 错误或槽池溢出造成的
 * 字节流断点，任务遇到新 epoch 时会先复位两个协议解析器，避免拼接跨断点帧。
 */
typedef struct
{
    uint8_t  data[MTF02_UART_RX_BUFFER_LEN];
    uint16_t length;
    uint16_t reserved;
    uint32_t stream_epoch;
    uint64_t timestamp_us;
} MTF02_RxChunk_t;

/**
 * @brief 单个 MTF-02 驱动实例
 *
 * UART 回调在中断上下文只向静态槽池复制原始字节、记录时间戳并通知任务；
 * SensorHub 任务负责协议解析、更新 latest_data 和发布。槽池采用单生产者
 * （UART ISR）/单消费者（SensorHub 任务）计数器，不经过 FreeRTOS 大块队列。
 * 所有结构体均放在头文件中，便于 Ozone/GDB 展开成员。
 */
typedef struct
{
    SeqLock_t data_lock;

    MTF02_MicoLinkParser_t micolink_parser;
    MTF02_MSPParser_t      msp_parser;
    MTF02_Data_t           latest_data;

    MTF02_RxChunk_t rx_chunk_queue[MTF02_RX_CHUNK_QUEUE_DEPTH];

    /* write_count 仅由 ISR 修改，read_count 仅由 SensorHub 任务修改。 */
    volatile uint32_t rx_chunk_write_count;
    volatile uint32_t rx_chunk_read_count;
    volatile uint32_t rx_stream_epoch;
    volatile uint32_t rx_chunk_drop_count;
    volatile uint32_t rx_chunk_oversize_count;
    volatile uint32_t rx_chunk_high_watermark;
    uint32_t          processed_stream_epoch;

    uint64_t last_msp_flow_timestamp_us;

    volatile uint32_t rx_chunk_count;
    volatile uint32_t rx_frame_count;
    volatile uint32_t micolink_frame_count;
    volatile uint32_t msp_range_frame_count;
    volatile uint32_t msp_flow_frame_count;
    volatile uint32_t sync_drop_count;
    volatile uint32_t crc_error_count;
    volatile uint32_t format_error_count;
    volatile uint32_t rx_error_count;
    volatile uint32_t published_count;

    uint32_t processed_micolink_count;
    uint32_t processed_msp_range_count;
    uint32_t processed_msp_flow_count;

    Publisher_t   *publisher;
    USARTInstance *usart_instance;
} MTF02_Instance_t;

uint8_t MTF02_Init(void);
uint8_t MTF02_Task_Handler(void);
void    Mtf02_RegisterReadyTask(TaskHandle_t task_handle);

#endif /* STM32H743_UAV_FLIGHT_CONTROLLER_MODULES_MTF02_H */
