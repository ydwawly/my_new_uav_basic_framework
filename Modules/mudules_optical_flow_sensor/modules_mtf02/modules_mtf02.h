//
// Created by Administrator on 2026/6/15.
//

#ifndef MY_NEW_UAV_BAICE_FRAMEWORK_MODULES_MTF02_H
#define MY_NEW_UAV_BAICE_FRAMEWORK_MODULES_MTF02_H

#include <stdint.h>

#include "bsp_uart.h"
#include "bsp_utils_seqlock.h"
#include "modules_Message_center.h"

/* ========================== 私有配置 ========================== */

#define MTF02_SEQLOCK_MAX_RETRY  3U
#define MTF02_TOPIC_NAME         "mtf02_data"

/* ========================== 协议定义 ========================== */

#define MTF02_MSG_HEAD              0xEFU
#define MTF02_MSG_ID_RANGE_SENSOR   0x51U

#define MTF02_PAYLOAD_LEN           20U
#define MTF02_FRAME_LEN             27U

/* 帧字段偏移 */
#define MTF02_IDX_HEAD              0U
#define MTF02_IDX_DEV_ID            1U
#define MTF02_IDX_SYS_ID            2U
#define MTF02_IDX_MSG_ID            3U
#define MTF02_IDX_SEQ               4U
#define MTF02_IDX_LEN               5U
#define MTF02_IDX_PAYLOAD           6U
#define MTF02_IDX_CHECKSUM          26U

/* payload 字段偏移 */
#define MTF02_PL_TIME_MS            0U
#define MTF02_PL_DISTANCE           4U
#define MTF02_PL_STRENGTH           8U
#define MTF02_PL_PRECISION          9U
#define MTF02_PL_TOF_STATUS         10U
#define MTF02_PL_FLOW_VEL_X         12U
#define MTF02_PL_FLOW_VEL_Y         14U
#define MTF02_PL_FLOW_QUALITY       16U
#define MTF02_PL_FLOW_STATUS        17U

/* ========================== 对外发布数据结构 ========================== */

typedef struct
{
    uint8_t  dev_id;
    uint8_t  sys_id;
    uint8_t  msg_id;
    uint8_t  seq;

    uint32_t sensor_time_ms;
    uint32_t distance_mm;

    uint8_t  strength;
    uint8_t  precision;
    uint8_t  tof_status;

    int16_t  flow_vel_x;
    int16_t  flow_vel_y;
    uint8_t  flow_quality;
    uint8_t  flow_status;

    uint64_t Mtf02_Timestamp;
} MTF02_Data_t;

/* ========================== 私有实例结构体 ========================== */

typedef struct
{
    SeqLock_t data_lock;

    /* ISR 中唯一一次 memcpy 落地的原始帧 */
    uint8_t  raw_rx_buf[MTF02_FRAME_LEN];
    uint16_t raw_len;
    uint64_t capture_timestamp;
    volatile uint32_t rx_frame_count;

    /*
     * last_seen_frame_count : 上次尝试处理时看到的帧号（无论成功失败）
     *                         用于判断"有没有新帧需要处理"，防止坏帧被反复重试
     *
     * last_proc_frame_count : 上次成功解析并发布的帧号
     *                         只在 PubPushMessage 成功后才更新
     *                         可用于统计丢帧率：rx_frame_count - last_proc_frame_count
     */
    volatile uint32_t last_seen_frame_count;
    volatile uint32_t last_proc_frame_count;

    volatile uint32_t rx_err_count;
    volatile uint32_t parse_err_count;

    Publisher_t   *publisher;
    USARTInstance *usart_instance;
} MTF02_Instance_t;

/* ========================== 对外接口 ========================== */

uint8_t MTF02_Init(void);
uint8_t MTF02_Task_Handler(void);

#endif //MY_NEW_UAV_BAICE_FRAMEWORK_MODULES_MTF02_H