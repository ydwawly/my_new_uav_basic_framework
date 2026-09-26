//
// Created by Administrator on 2026/6/18.
//

#ifndef STM32H743_UAV_FLIGHT_CONTROLLER_MODULES_TFMINI_PLUS_H
#define STM32H743_UAV_FLIGHT_CONTROLLER_MODULES_TFMINI_PLUS_H

#include <stdint.h>
#include "usart.h"
#include "bsp_uart.h"
#include "bsp_utils_seqlock.h"
#include "modules_Message_center.h"
#include "FreeRTOS.h"
#include "task.h"
/* ========================== 任务与运行配置 ========================== */

#define TFMINI_TOPIC_NAME        "tfmini_data"
#define TFMINI_SEQLOCK_MAX_RETRY 3U
#define NOTIFY_BIT_TFMINI        (1 << 5)

#define TFMINI_UART_HANDLE           huart1
#define TFMINI_MONITOR_TIMEOUT_US    100000U
#define TFMINI_MONITOR_INIT_GRACE_US 500000U

/* ========================== 协议定义 ========================== */

#define TFMINI_FRAME_SIZE                     9U
#define TFMINI_HEADER                         0x59U
#define TFMINI_COMMAND_HEADER                 0x5AU
#define TFMINI_COMMAND_MAX_SIZE               6U
#define TFMINI_COMMAND_RETRY_COUNT            3U
#define TFMINI_COMMAND_RESPONSE_TIMEOUT_MS    100U
#define TFMINI_OUTPUT_RATE_HZ                 100U
#define TFMINI_SET_OUTPUT_RATE_COMMAND_ID     0x03U
#define TFMINI_SET_OUTPUT_SWITCH_COMMAND_ID   0x07U
#define TFMINI_SET_OUTPUT_RATE_COMMAND_SIZE   6U
#define TFMINI_SET_OUTPUT_SWITCH_COMMAND_SIZE 5U

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
#define TFMINI_IDX_HEADER1    0U
#define TFMINI_IDX_HEADER2    1U
#define TFMINI_IDX_DISTANCE_L 2U
#define TFMINI_IDX_DISTANCE_H 3U
#define TFMINI_IDX_STRENGTH_L 4U
#define TFMINI_IDX_STRENGTH_H 5U
#define TFMINI_IDX_TEMP_L     6U
#define TFMINI_IDX_TEMP_H     7U
#define TFMINI_IDX_CHECKSUM   8U

/* ========================== 对外发布数据结构 ========================== */

typedef struct
{
    uint16_t distance;     /* 测距值，单位：cm */
    uint16_t strength;     /* 回波信号强度，无量纲 */
    float    temperature;  /* 雷达内部温度，单位：摄氏度 */
    uint8_t  is_valid;     /* 数据有效标志：1=有效，0=无效 */
    uint64_t timestamp_us; /* UART 完成接收时的本地时间戳，单位：μs */
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

    /*
     * 配置应答匹配状态：
     * TFmini 配置帧以 0x5A 开头，长度与普通 0x59 0x59 测距帧不同。
     * 初始化任务写入 expected_response，UART ISR 逐字节匹配并置位
     * response_received，避免配置应答被普通 9 字节解析器误判为坏帧。
     */
    volatile uint8_t config_waiting_response;
    volatile uint8_t config_response_received;
    volatile uint8_t config_match_index;
    volatile uint8_t config_expected_length;
    volatile uint8_t config_expected_response[TFMINI_COMMAND_MAX_SIZE];

    uint16_t          configured_rate_hz;   /* 已由飞控确认设置的输出频率，单位：Hz */
    volatile uint32_t config_success_count; /* 完整配置成功次数 */
    volatile uint32_t config_error_count;   /* 配置或应答失败次数 */

    Publisher_t   *publisher;
    USARTInstance *usart_instance;
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
 *       4. 将连续测距反馈频率配置为 TFMINI_OUTPUT_RATE_HZ
 *
 * @note 频率配置每次上电都会执行，但不会反复发送“保存设置”指令，
 *       因此既能保证飞控运行期间始终使用目标频率，又能避免频繁写雷达非易失存储器。
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

void Tfmini_RegisterReadyTask(TaskHandle_t task_handle);

#endif /* STM32H743_UAV_FLIGHT_CONTROLLER_MODULES_TFMINI_PLUS_H */
