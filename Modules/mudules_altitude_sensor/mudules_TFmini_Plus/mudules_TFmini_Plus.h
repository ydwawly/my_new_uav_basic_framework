//
// Created by Administrator on 2026/6/18.
//

#ifndef MY_NEW_UAV_BAICE_FRAMEWORK_MUDULES_TFMINI_PLUS_H
#define MY_NEW_UAV_BAICE_FRAMEWORK_MUDULES_TFMINI_PLUS_H

#include <stdint.h>
#include "usart.h"
#include "bsp_uart.h"
#include "bsp_utils_seqlock.h"
#include "modules_Message_center.h"

/* ========================== 私有配置 ========================== */

#define TFMINI_TOPIC_NAME          "tfmini_data"
#define TFMINI_SEQLOCK_MAX_RETRY   3U
/* ========================== 私有配置 ========================== */

#define TFMINI_UART_HANDLE             huart1
#define TFMINI_MONITOR_TIMEOUT_US      100000U
#define TFMINI_MONITOR_INIT_GRACE_US   500000U

/* ========================== 协议定义 ========================== */

#define TFMINI_FRAME_SIZE          9U
#define TFMINI_HEADER              0x59U

/* 帧字段偏移
 * [0] Header1 = 0x59
 * [1] Header2 = 0x59
 * [2] Dist_L
 * [3] Dist_H
 * [4] Strength_L
 * [5] Strength_H
 * [6] Temp_L
 * [7] Temp_H
 * [8] Checksum
 */
#define TFMINI_IDX_HEADER1         0U
#define TFMINI_IDX_HEADER2         1U
#define TFMINI_IDX_DISTANCE_L      2U
#define TFMINI_IDX_DISTANCE_H      3U
#define TFMINI_IDX_STRENGTH_L      4U
#define TFMINI_IDX_STRENGTH_H      5U
#define TFMINI_IDX_TEMP_L          6U
#define TFMINI_IDX_TEMP_H          7U
#define TFMINI_IDX_CHECKSUM        8U

/* ========================== 对外发布数据结构 ========================== */

typedef struct
{
    uint16_t distance;         /* cm */
    uint16_t strength;         /* 信号强度 */
    float    temperature;      /* 摄氏度 */
    uint8_t  is_valid;         /* 1=有效, 0=无效 */
    uint64_t timestamp_us;     /* 本地时间戳 */
} TFminiPlus_Data_t;

typedef struct
{
    SeqLock_t data_lock;

    /* ISR 中唯一一次 memcpy 落地的原始帧 */
    uint8_t  raw_rx_buf[TFMINI_FRAME_SIZE];
    uint16_t raw_len;
    uint64_t capture_timestamp;
    uint32_t rx_frame_count;

    /*
     * last_seen_frame_count:
     *  任务最后一次“已经消费过”的原始帧编号
     *  无论成功解析还是失败，都会更新它，避免同一坏帧被重复统计
     *
     * last_pub_frame_count:
     *  最后一次成功解析并发布的帧编号
     *  仅用于统计/调试语义，不参与“是否有新帧”的判断
     */
    uint32_t last_seen_frame_count;
    uint32_t last_pub_frame_count;

    volatile uint32_t rx_err_count;
    volatile uint32_t parse_err_count;

    Publisher_t     *publisher;
    USARTInstance   *usart_instance;
} TFmini_Instance_t;
/* ========================== 对外接口 ========================== */

/**
 * @brief 初始化 TFmini Plus 驱动
 *
 * @return 1 成功
 * @return 0 失败
 *
 * @note 内部自动完成：
 *       1. 初始化 SeqLock
 *       2. 注册消息中心发布者
 *       3. 注册 UART
 *       4. 注册 Monitor
 */
uint8_t TFmini_Init(void);

/**
 * @brief TFmini 任务处理函数
 *
 * @return 1 成功解析并发布了一帧新数据
 * @return 0 无新数据或解析失败
 *
 * @note 在任务中周期调用即可
 */
uint8_t TFmini_Task_Handler(void);


#endif //MY_NEW_UAV_BAICE_FRAMEWORK_MUDULES_TFMINI_PLUS_H