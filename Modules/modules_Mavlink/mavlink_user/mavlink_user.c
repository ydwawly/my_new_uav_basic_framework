#include "mavlink_user.h"

/**
 * @file mavlink_user.c
 * @brief MAVLink 收包解析、消息编码和链路统计的框架适配层。
 *
 * 中断只把串口字节写入环形缓冲区并唤醒接收任务；完整帧解析和业务回调在任务上下文
 * 执行。发送接口仅负责编码为线格式，真正的发送排队由 DataRouter 统一完成。
 */

#include <string.h>

#include "bsp_RTT.h"
#include "semphr.h"
#include "task.h"

static MavlinkInstance_t mavlink_instance = {0};
static StaticSemaphore_t mavlink_tx_mutex_control;
static SemaphoreHandle_t mavlink_tx_mutex;

/**
 * @brief 开始一次 MAVLink 发送事务
 * @note 事务必须覆盖“打包 + 序列化 + DataRouter 入队”，否则任务抢占会使线上的序号倒序。
 */
bool Mavlink_TxTransactionBegin(void)
{
    if (mavlink_tx_mutex == NULL) return false;
    const BaseType_t taken = xSemaphoreTakeRecursive(mavlink_tx_mutex, portMAX_DELAY);
    return taken == pdTRUE;
}

/** @brief 结束一次 MAVLink 发送事务。 */
void Mavlink_TxTransactionEnd(void)
{
    if (mavlink_tx_mutex != NULL)
    {
        (void)xSemaphoreGiveRecursive(mavlink_tx_mutex);
    }
}

/** @brief 将已打包的 MAVLink 消息序列化到调用者缓冲区。 */
static uint16_t Mavlink_Serialize(uint8_t *buffer, uint16_t buffer_size, const mavlink_message_t *msg)
{
    uint16_t length;

    if (buffer == NULL || msg == NULL || buffer_size < MAVLINK_MAX_PACKET_LEN)
    {
        return 0U;
    }

    length = mavlink_msg_to_send_buffer(buffer, msg);
    return (length <= buffer_size) ? length : 0U;
}

/**
 * @brief 初始化全局 MAVLink 实例、接收环形缓冲区和本机标识。
 * @note 本模块是单实例设计，重复初始化会返回 false。
 */
bool Mavlink_Init(const Mavlink_Init_Config_s *init_config)
{
    MavlinkInstance_t *instance = &mavlink_instance;
    mavlink_status_t  *tx_status;

    if (instance->initialized)
    {
        return false;
    }

    memset(instance, 0, sizeof(*instance));

    if (!RingBuffer_Init(&instance->rx_ringbuffer, instance->rx_pool, sizeof(instance->rx_pool)))
    {
        RTTERROR("[MAVLINK] RX RingBuffer init failed.");
        return false;
    }

    mavlink_tx_mutex = xSemaphoreCreateRecursiveMutexStatic(&mavlink_tx_mutex_control);
    if (mavlink_tx_mutex == NULL)
    {
        RTTERROR("[MAVLINK] TX mutex init failed.");
        return false;
    }

    instance->system_id    = (init_config != NULL && init_config->system_id != 0U) ? init_config->system_id
                                                                                   : MAVLINK_DEFAULT_SYSTEM_ID;
    instance->component_id = (init_config != NULL && init_config->component_id != 0U) ? init_config->component_id
                                                                                      : MAVLINK_DEFAULT_COMPONENT_ID;
    instance->message_callback = (init_config != NULL) ? init_config->message_callback : NULL;

    /* 强制发送通道使用 MAVLink 2，避免状态残留使报文退回 MAVLink 1。 */
    tx_status = mavlink_get_channel_status(MAVLINK_TX_CHANNEL);
    tx_status->flags &= (uint8_t)~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;

    instance->initialized = 1U;

    RTTINFO("[MAVLINK] Init OK. RX=%uB, sysid=%u, compid=%u.", (unsigned)MAVLINK_RX_BUF_SIZE,
            (unsigned)instance->system_id, (unsigned)instance->component_id);

    return true;
}

/**
 * @brief 从串口接收 ISR 投递原始字节并通知解析任务。
 * @note 环形缓冲区空间不足时整批丢弃，并累计 rx_dropped_bytes；本函数不得阻塞。
 */
void Mavlink_InputBytesFromISR(const uint8_t *data, uint16_t size)
{
    MavlinkInstance_t *instance   = &mavlink_instance;
    BaseType_t         task_woken = pdFALSE;

    if (!instance->initialized || data == NULL || size == 0U)
    {
        return;
    }

    if (!RingBuffer_Push(&instance->rx_ringbuffer, data, size))
    {
        instance->stats.rx_dropped_bytes += size;
        return;
    }

    instance->stats.rx_bytes += size;

    if (instance->rx_task_handle != NULL)
    {
        vTaskNotifyGiveFromISR(instance->rx_task_handle, &task_woken);
        portYIELD_FROM_ISR(task_woken);
    }
}

/**
 * @brief MAVLink 接收任务：分批取出字节、解析完整帧并调用上层消息回调。
 *
 * 单次唤醒最多处理 MAVLINK_RX_MAX_BYTES_PER_WAKE 字节；若仍有积压则重新通知自己，
 * 在保持吞吐量的同时给同优先级任务留下调度机会。
 */
void MavlinkRxTask(void *pvParameters)
{
    MavlinkInstance_t *instance = &mavlink_instance;
    mavlink_message_t  message  = {0};
    mavlink_status_t   status   = {0};
    uint8_t            buffer[MAVLINK_RX_PROCESS_CHUNK_SIZE];
    uint16_t           last_drop_count = 0U;

    (void)pvParameters;

    if (!instance->initialized)
    {
        vTaskDelete(NULL);
        return;
    }

    instance->rx_task_handle = xTaskGetCurrentTaskHandle();
    xTaskNotifyGive(instance->rx_task_handle);

    for (;;)
    {
        uint32_t processed = 0U;
        uint32_t size;

        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        do
        {
            size = RingBuffer_Pop(&instance->rx_ringbuffer, buffer, sizeof(buffer));

            for (uint32_t i = 0U; i < size; i++)
            {
                if (mavlink_parse_char(MAVLINK_RX_CHANNEL, buffer[i], &message, &status))
                {
                    instance->stats.rx_messages++;

                    if (instance->message_callback != NULL)
                    {
                        instance->message_callback(&message);
                    }
                }
            }

            processed += size;
        } while (size > 0U && processed < MAVLINK_RX_MAX_BYTES_PER_WAKE);

        instance->stats.rx_sequence_drops += (uint16_t)(status.packet_rx_drop_count - last_drop_count);
        last_drop_count = status.packet_rx_drop_count;

        if (RingBuffer_GetUsed(&instance->rx_ringbuffer) > 0U)
        {
            xTaskNotifyGive(instance->rx_task_handle);
        }
    }
}

/** @brief 编码 ATTITUDE，角度单位 rad，角速度单位 rad/s。 */
uint16_t Mavlink_EncodeAttitude(uint8_t *buffer, uint16_t buffer_size, const Mavlink_Attitude_Data_t *data)
{
    MavlinkInstance_t *instance = &mavlink_instance;
    mavlink_message_t  message;

    if (!instance->initialized || data == NULL || !Mavlink_TxTransactionBegin())
    {
        return 0U;
    }

    mavlink_msg_attitude_pack_chan(instance->system_id, instance->component_id, MAVLINK_TX_CHANNEL, &message,
                                   data->time_boot_ms, data->roll, data->pitch, data->yaw, data->rollspeed,
                                   data->pitchspeed, data->yawspeed);

    const uint16_t length = Mavlink_Serialize(buffer, buffer_size, &message);
    Mavlink_TxTransactionEnd();
    return length;
}

/** @brief 编码 HEARTBEAT，用于广播飞控类型、模式和系统状态。 */
uint16_t Mavlink_EncodeHeartbeat(uint8_t *buffer, uint16_t buffer_size, const Mavlink_Heartbeat_Data_t *data)
{
    MavlinkInstance_t *instance = &mavlink_instance;
    mavlink_message_t  message;

    if (!instance->initialized || data == NULL || !Mavlink_TxTransactionBegin())
    {
        return 0U;
    }

    mavlink_msg_heartbeat_pack_chan(instance->system_id, instance->component_id, MAVLINK_TX_CHANNEL, &message,
                                    data->type, data->autopilot, data->base_mode, data->custom_mode,
                                    data->system_status);

    const uint16_t length = Mavlink_Serialize(buffer, buffer_size, &message);
    Mavlink_TxTransactionEnd();
    return length;
}

/** @brief 编码 COMMAND_ACK，并把应答定向到原命令发送方。 */
uint16_t Mavlink_EncodeCommandAck(uint8_t *buffer, uint16_t buffer_size, uint16_t command, uint8_t result,
                                  uint8_t progress, uint8_t target_system, uint8_t target_component)
{
    MavlinkInstance_t *instance = &mavlink_instance;
    mavlink_message_t  message;

    if (!instance->initialized || !Mavlink_TxTransactionBegin())
    {
        return 0U;
    }

    mavlink_msg_command_ack_pack_chan(instance->system_id, instance->component_id, MAVLINK_TX_CHANNEL, &message,
                                      command, result, progress, 0, target_system, target_component);

    const uint16_t length = Mavlink_Serialize(buffer, buffer_size, &message);
    Mavlink_TxTransactionEnd();
    return length;
}

/** @brief 编码 PING；time_usec 为发送端时间戳，sequence 用于往返匹配。 */
uint16_t Mavlink_EncodePing(uint8_t *buffer, uint16_t buffer_size, uint64_t time_usec, uint32_t sequence,
                            uint8_t target_system, uint8_t target_component)
{
    MavlinkInstance_t *instance = &mavlink_instance;
    mavlink_message_t  message;

    if (!instance->initialized || !Mavlink_TxTransactionBegin())
    {
        return 0U;
    }

    mavlink_msg_ping_pack_chan(instance->system_id, instance->component_id, MAVLINK_TX_CHANNEL, &message, time_usec,
                               sequence, target_system, target_component);

    const uint16_t length = Mavlink_Serialize(buffer, buffer_size, &message);
    Mavlink_TxTransactionEnd();
    return length;
}

/** @brief 编码 TIMESYNC；tc1/ts1 的含义遵循 MAVLink 时间同步握手约定。 */
uint16_t Mavlink_EncodeTimesync(uint8_t *buffer, uint16_t buffer_size, int64_t tc1, int64_t ts1, uint8_t target_system,
                                uint8_t target_component)
{
    MavlinkInstance_t *instance = &mavlink_instance;
    mavlink_message_t  message;

    if (!instance->initialized || !Mavlink_TxTransactionBegin())
    {
        return 0U;
    }

    mavlink_msg_timesync_pack_chan(instance->system_id, instance->component_id, MAVLINK_TX_CHANNEL, &message, tc1, ts1,
                                   target_system, target_component);

    const uint16_t length = Mavlink_Serialize(buffer, buffer_size, &message);
    Mavlink_TxTransactionEnd();
    return length;
}

/** @brief 编码 HIL_ACTUATOR_CONTROLS，controls 固定包含 16 个归一化执行器通道。 */
uint16_t Mavlink_EncodeHilActuatorControls(uint8_t *buffer, uint16_t buffer_size, uint64_t time_usec,
                                           const float controls[16], uint8_t mode, uint64_t flags)
{
    MavlinkInstance_t *instance = &mavlink_instance;
    mavlink_message_t  message;

    if (!instance->initialized || controls == NULL || !Mavlink_TxTransactionBegin())
    {
        return 0U;
    }

    mavlink_msg_hil_actuator_controls_pack_chan(instance->system_id, instance->component_id, MAVLINK_TX_CHANNEL,
                                                &message, time_usec, controls, mode, flags);

    const uint16_t length = Mavlink_Serialize(buffer, buffer_size, &message);
    Mavlink_TxTransactionEnd();
    return length;
}

/** @brief 编码单个 LOG_ENTRY 目录项及日志集合元数据。 */
uint16_t Mavlink_EncodeLogEntry(uint8_t *buffer, uint16_t buffer_size, uint16_t id, uint16_t num_logs,
                                uint16_t last_log_num, uint32_t time_utc, uint32_t size)
{
    MavlinkInstance_t *instance = &mavlink_instance;
    mavlink_message_t  message;

    if (!instance->initialized || !Mavlink_TxTransactionBegin())
    {
        return 0U;
    }

    mavlink_msg_log_entry_pack_chan(instance->system_id, instance->component_id, MAVLINK_TX_CHANNEL, &message, id,
                                    num_logs, last_log_num, time_utc, size);
    const uint16_t length = Mavlink_Serialize(buffer, buffer_size, &message);
    Mavlink_TxTransactionEnd();
    return length;
}

/** @brief 编码 LOG_DATA；协议单包数据区最多 90 字节。 */
uint16_t Mavlink_EncodeLogData(uint8_t *buffer, uint16_t buffer_size, uint16_t id, uint32_t offset, uint8_t count,
                               const uint8_t data[90])
{
    MavlinkInstance_t *instance = &mavlink_instance;
    mavlink_message_t  message;

    if (!instance->initialized || (data == NULL && count > 0U) || count > 90U || !Mavlink_TxTransactionBegin())
    {
        return 0U;
    }

    mavlink_msg_log_data_pack_chan(instance->system_id, instance->component_id, MAVLINK_TX_CHANNEL, &message, id,
                                   offset, count, data);
    const uint16_t length = Mavlink_Serialize(buffer, buffer_size, &message);
    Mavlink_TxTransactionEnd();
    return length;
}

/** @brief 序列化调用方已构造的 MAVLink 消息。 */
uint16_t Mavlink_SerializeMessage(uint8_t *buffer, uint16_t buffer_size, const mavlink_message_t *msg)
{
    return Mavlink_Serialize(buffer, buffer_size, msg);
}

/** @brief 在临界区内累计发送入队成功数或丢弃数。 */
void Mavlink_ReportTxResult(bool enqueue_success)
{
    MavlinkInstance_t *instance = &mavlink_instance;

    taskENTER_CRITICAL();
    if (enqueue_success)
    {
        instance->stats.tx_enqueued_messages++;
    }
    else
    {
        instance->stats.tx_enqueue_drops++;
    }
    taskEXIT_CRITICAL();
}

/** @brief 在临界区内取得一致的链路统计快照。 */
void Mavlink_GetStats(Mavlink_Stats_t *out)
{
    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    memcpy(out, &mavlink_instance.stats, sizeof(*out));
    taskEXIT_CRITICAL();
}

/** @brief 返回尚未被解析任务消费的接收字节数。 */
uint32_t Mavlink_GetRxBufferedBytes(void)
{
    if (!mavlink_instance.initialized)
    {
        return 0U;
    }

    return RingBuffer_GetUsed(&mavlink_instance.rx_ringbuffer);
}

/**
 * @brief 判断目标系统/组件是否指向本机。
 * @note MAVLink 目标值 0 表示广播，因此系统或组件为 0 时也视为匹配。
 */
bool Mavlink_IsTarget(uint8_t target_system, uint8_t target_component)
{
    const bool system_match = (target_system == 0U) || (target_system == MAVLINK_DEFAULT_SYSTEM_ID);

    const bool component_match = (target_component == 0U) || (target_component == MAVLINK_DEFAULT_COMPONENT_ID);

    return system_match && component_match;
}
