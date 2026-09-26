/**
 * @file mavlink_user.h
 * @brief MAVLink 字节接收、报文解析、序列化及常用消息编码接口
 *
 * USB 接收中断只写入无锁环形缓冲区并通知解析任务；完整消息在任务上下文中分发。
 */

#ifndef STM32H743_UAV_FLIGHT_CONTROLLER_MAVLINK_USER_H
#define STM32H743_UAV_FLIGHT_CONTROLLER_MAVLINK_USER_H

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "bsp_ringbuffer.h"
#include "common/mavlink.h"
#include "task.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define MAVLINK_DEFAULT_SYSTEM_ID     1U
#define MAVLINK_DEFAULT_COMPONENT_ID  MAV_COMP_ID_AUTOPILOT1
#define MAVLINK_RX_CHANNEL            MAVLINK_COMM_0
#define MAVLINK_TX_CHANNEL            MAVLINK_COMM_1
#define MAVLINK_RX_BUF_SIZE           4096U
#define MAVLINK_RX_PROCESS_CHUNK_SIZE 256U
#define MAVLINK_RX_MAX_BYTES_PER_WAKE 2048U

typedef void (*Mavlink_Message_Callback_t)(const mavlink_message_t *msg);

/** @brief 协议栈初始化参数与完整报文回调。 */
typedef struct
{
    uint8_t                    system_id;
    uint8_t                    component_id;
    Mavlink_Message_Callback_t message_callback;
} Mavlink_Init_Config_s;

/** @brief 接收、序列号丢包以及发送入队统计。 */
typedef struct
{
    uint32_t rx_bytes;
    uint32_t rx_messages;
    uint32_t rx_dropped_bytes;
    uint32_t rx_sequence_drops;
    uint32_t tx_enqueued_messages;
    uint32_t tx_enqueue_drops;
} Mavlink_Stats_t;

/**
 * @brief MAVLink 协议栈单例运行状态
 *
 * 结构体定义放在头文件中便于调试器展开查看；实例本身仍由 mavlink_user.c 私有持有，
 * 外部代码不应直接修改其中字段。
 */
/** @brief ATTITUDE 消息编码所需的 SI 单位姿态与角速度。 */
typedef struct
{
    RingBuffer_t               rx_ringbuffer;                /* USB ISR 到解析任务的无锁字节环形缓冲区 */
    uint8_t                    rx_pool[MAVLINK_RX_BUF_SIZE]; /* 接收池，单位 byte */
    TaskHandle_t               rx_task_handle;               /* 收到新字节时需要唤醒的解析任务 */
    Mavlink_Message_Callback_t message_callback;             /* 完整报文的应用层分发入口 */
    Mavlink_Stats_t            stats;                        /* 收发、丢包与入队统计 */
    uint8_t                    system_id;                    /* 本机 MAVLink system id */
    uint8_t                    component_id;                 /* 本机 MAVLink component id */
    uint8_t                    initialized;                  /* 1 表示初始化完成 */
} MavlinkInstance_t;

/** @brief HEARTBEAT 消息的飞行器类型、模式和系统状态。 */
typedef struct
{
    uint32_t time_boot_ms;
    float    roll;
    float    pitch;
    float    yaw;
    float    rollspeed;
    float    pitchspeed;
    float    yawspeed;
} Mavlink_Attitude_Data_t;

typedef struct
{
    uint8_t  type;
    uint8_t  autopilot;
    uint8_t  base_mode;
    uint32_t custom_mode;
    uint8_t  system_status;
} Mavlink_Heartbeat_Data_t;

/** @brief 初始化单例、接收环形缓冲区和本机 system/component id。 */
bool Mavlink_Init(const Mavlink_Init_Config_s *init_config);

/**
 * @brief 开始/结束完整的 MAVLink 发送事务
 * @note 调用方应将编码和 DataRouter_PostFrame() 包在同一事务中，保证发送序号与入队顺序一致。
 *       该互斥量支持递归获取，因此编码函数内部仍可独立保护 pack_chan 的共享状态。
 */
bool Mavlink_TxTransactionBegin(void);
void Mavlink_TxTransactionEnd(void);

/** @brief 从 ISR 投递原始字节并唤醒解析任务；缓冲区满时统计丢弃字节。 */
void Mavlink_InputBytesFromISR(const uint8_t *data, uint16_t size);

/** @brief MAVLink 接收解析任务入口。 */
void MavlinkRxTask(void *pvParameters);

uint16_t Mavlink_EncodeAttitude(uint8_t *buffer, uint16_t buffer_size, const Mavlink_Attitude_Data_t *data);

uint16_t Mavlink_EncodeHeartbeat(uint8_t *buffer, uint16_t buffer_size, const Mavlink_Heartbeat_Data_t *data);

uint16_t Mavlink_EncodeCommandAck(uint8_t *buffer, uint16_t buffer_size, uint16_t command, uint8_t result,
                                  uint8_t progress, uint8_t target_system, uint8_t target_component);

uint16_t Mavlink_EncodePing(uint8_t *buffer, uint16_t buffer_size, uint64_t time_usec, uint32_t sequence,
                            uint8_t target_system, uint8_t target_component);

uint16_t Mavlink_EncodeTimesync(uint8_t *buffer, uint16_t buffer_size, int64_t tc1, int64_t ts1, uint8_t target_system,
                                uint8_t target_component);

uint16_t Mavlink_EncodeHilActuatorControls(uint8_t *buffer, uint16_t buffer_size, uint64_t time_usec,
                                           const float controls[16], uint8_t mode, uint64_t flags);

uint16_t Mavlink_EncodeLogEntry(uint8_t *buffer, uint16_t buffer_size, uint16_t id, uint16_t num_logs,
                                uint16_t last_log_num, uint32_t time_utc, uint32_t size);

uint16_t Mavlink_EncodeLogData(uint8_t *buffer, uint16_t buffer_size, uint16_t id, uint32_t offset, uint8_t count,
                               const uint8_t data[90]);

/** @brief 序列化任意已填充的 MAVLink 消息。 */
uint16_t Mavlink_SerializeMessage(uint8_t *buffer, uint16_t buffer_size, const mavlink_message_t *msg);

/** @brief 记录 DataRouter 是否成功接收一个待发送帧。 */
void Mavlink_ReportTxResult(bool enqueue_success);

/** @brief 取得一致的收发统计快照。 */
void Mavlink_GetStats(Mavlink_Stats_t *out);

/** @brief 返回 ISR 接收环中尚未解析的字节数。 */
uint32_t Mavlink_GetRxBufferedBytes(void);

/** @brief 判断消息目标是否匹配本机，0 表示 MAVLink 广播地址。 */
bool Mavlink_IsTarget(uint8_t target_system, uint8_t target_component);

#ifdef __cplusplus
}
#endif

#endif
