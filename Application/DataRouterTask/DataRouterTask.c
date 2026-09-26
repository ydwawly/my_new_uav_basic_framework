/** @file DataRouterTask.c @brief MAVLink 与 SD 日志数据的异步路由任务。 */

#include "DataRouterTask.h"
#include <stddef.h>
#include <string.h>
#include "bsp_RTT.h"
#include "bsp_timestamp.h"
#include "modules_SD_Card.h"
#include "user_math.h"

#define DATA_ROUTER_USB_STREAM_BATCH_SIZE 512U

static const uint8_t usb_capture_start_prefix[] = {'I', 'M', 'U', 'C', 'A', 'P', '1'};

/* ========================== 私有变量 ========================== */

static DataRouterInstance      data_router = {0};
volatile DataRouter_LogStats_t data_router_log_stats;
volatile DataRouter_TxStats_t  data_router_tx_stats;
static volatile uint8_t        usb_capture_start_requested;
static volatile uint16_t       usb_capture_session_token;
static uint8_t                 usb_capture_command_index;
static uint16_t                usb_capture_candidate_token;
static uint8_t                 usb_stream_header_pending;

/* ========================== 私有函数 ========================== */

/**
 * @brief 使用同一时间戳执行所有已经注册的业务服务更新函数
 */
static void DataRouter_ProcessUpdates(uint64_t now_us)
{
    if (data_router.update_callbacks == NULL)
    {
        return;
    }

    for (uint8_t i = 0U; i < data_router.update_callback_count; i++)
    {
        DataRouter_UpdateCallback_t callback = data_router.update_callbacks[i];
        if (callback != NULL)
        {
            callback(now_us);
        }
    }
}

#if (DATA_ROUTER_ENABLE_USB_IMU_STREAM == 1U)
static uint16_t DataRouter_BuildUsbFileHeader(uint8_t *output, uint16_t capacity)
{
    if ((output == NULL) || (capacity < sizeof(SDCard_FileHeader_t)))
    {
        return 0U;
    }

    SDCard_FileHeader_t header = {
        .magic              = SD_CARD_FILE_MAGIC,
        .format_version     = SD_CARD_FORMAT_VERSION,
        .header_size        = sizeof(SDCard_FileHeader_t),
        .start_timestamp_us = Bsp_Timestamp_us_Get(),
        .format_flags       = SD_CARD_FORMAT_FLAG_LITTLE_ENDIAN | SD_CARD_FORMAT_FLAG_IEEE754_FLOAT |
                              ((uint32_t)usb_capture_session_token << 16U),
        .header_crc16       = 0U,
        .reserved           = 0U,
    };

    header.header_crc16 = Math_Crc16Ccitt(&header, offsetof(SDCard_FileHeader_t, header_crc16), SD_CARD_CRC_INITIAL);
    memcpy(output, &header, sizeof(header));
    return sizeof(header);
}

static uint16_t DataRouter_BuildUsbImuFrame(const DataRouter_ImuLogItem_t *item, uint8_t *output, uint16_t capacity)
{
    const uint16_t frame_size = sizeof(SDCard_FrameHeader_t) + sizeof(item->payload) + sizeof(uint16_t);

    if ((item == NULL) || (output == NULL) || (capacity < frame_size))
    {
        return 0U;
    }

    const SDCard_FrameHeader_t header = {
        .magic           = SD_CARD_FRAME_MAGIC,
        .message_id      = SD_CARD_MSG_BMI088_IMU,
        .message_version = 1U,
        .payload_length  = sizeof(item->payload),
        .flags           = item->payload.validity_flags,
        .sequence        = item->payload.sample_sequence,
        .timestamp_us    = item->timestamp_us,
    };

    memcpy(output, &header, sizeof(header));
    memcpy(output + sizeof(header), &item->payload, sizeof(item->payload));

    const uint16_t crc_offset = sizeof(header) + sizeof(item->payload);
    const uint16_t crc        = Math_Crc16Ccitt(output, crc_offset, SD_CARD_CRC_INITIAL);
    memcpy(output + crc_offset, &crc, sizeof(crc));
    return frame_size;
}

static bool DataRouter_SendUsbStreamBatch(const uint8_t *batch, uint16_t size, uint32_t frame_count)
{
    if (USBSend(data_router.usb_instance, batch, size))
    {
        data_router_log_stats.usb_stream_frames += frame_count;
        return true;
    }

    data_router_log_stats.usb_stream_dropped += frame_count;
    return false;
}

static bool DataRouter_PrepareUsbImuBatch(uint8_t *batch, uint16_t capacity)
{
    if (!USBIsConnected())
    {
        data_router_log_stats.usb_stream_active = 0U;
        usb_capture_session_token               = 0U;
        usb_stream_header_pending               = 0U;
        return false;
    }

    if (usb_capture_start_requested != 0U)
    {
        usb_capture_start_requested             = 0U;
        usb_stream_header_pending               = 1U;
        data_router_log_stats.usb_stream_active = 0U;
    }

    if (usb_stream_header_pending != 0U)
    {
        const uint16_t header_size = DataRouter_BuildUsbFileHeader(batch, capacity);
        if ((header_size > 0U) && USBSend(data_router.usb_instance, batch, header_size))
        {
            usb_stream_header_pending               = 0U;
            data_router_log_stats.usb_stream_active = 1U;
            data_router_log_stats.usb_stream_sessions++;
        }
        else
        {
            data_router_log_stats.usb_header_send_failures++;
        }
    }

    return (data_router_log_stats.usb_stream_active != 0U);
}

static bool DataRouter_AppendUsbImuFrame(uint8_t *batch, uint16_t *batch_size, uint32_t *batch_count,
                                         const DataRouter_ImuLogItem_t *item)
{
    const uint16_t frame_size = sizeof(SDCard_FrameHeader_t) + sizeof(item->payload) + sizeof(uint16_t);

    if ((*batch_size > 0U) && ((uint32_t)*batch_size + frame_size > DATA_ROUTER_USB_STREAM_BATCH_SIZE))
    {
        const bool sent = DataRouter_SendUsbStreamBatch(batch, *batch_size, *batch_count);
        *batch_size     = 0U;
        *batch_count    = 0U;
        if (!sent)
        {
            data_router_log_stats.usb_stream_active = 0U;
            data_router_log_stats.usb_stream_dropped++;
            return false;
        }
    }

    const uint16_t encoded = DataRouter_BuildUsbImuFrame(item, batch + *batch_size,
                                                         (uint16_t)(DATA_ROUTER_USB_STREAM_BATCH_SIZE - *batch_size));
    if (encoded == 0U)
    {
        data_router_log_stats.usb_stream_dropped++;
        return true;
    }
    *batch_size += encoded;
    (*batch_count)++;
    return true;
}

static void DataRouter_FlushUsbImuBatch(const uint8_t *batch, uint16_t batch_size, uint32_t batch_count)
{
    if ((batch_size > 0U) && (!DataRouter_SendUsbStreamBatch(batch, batch_size, batch_count)))
    {
        data_router_log_stats.usb_stream_active = 0U;
    }
}
#endif

#if (DATA_ROUTER_ENABLE_SD_IMU_LOG == 1U)
static void DataRouter_RecordSdImu(const DataRouter_ImuLogItem_t *item)
{
    if (SDCard_EnqueueBMI088Imu(item->timestamp_us, &item->payload))
    {
        data_router_log_stats.sd_imu_frames++;
    }
    else
    {
        data_router_log_stats.sd_imu_dropped++;
    }
}
#endif

static void DataRouter_ProcessImuLogs(void)
{
#if (DATA_ROUTER_ENABLE_USB_IMU_STREAM == 1U)
    uint8_t  batch[DATA_ROUTER_USB_STREAM_BATCH_SIZE];
    uint16_t batch_size       = 0U;
    uint32_t batch_count      = 0U;
    bool     usb_stream_ready = DataRouter_PrepareUsbImuBatch(batch, sizeof(batch));
#endif

    DataRouter_ImuLogItem_t item;

    while (xQueueReceive(data_router.imu_log_queue, &item, 0U) == pdPASS)
    {
#if (DATA_ROUTER_ENABLE_SD_IMU_LOG == 1U)
        DataRouter_RecordSdImu(&item);
#endif

#if (DATA_ROUTER_ENABLE_USB_IMU_STREAM == 1U)
        if (usb_stream_ready)
        {
            usb_stream_ready = DataRouter_AppendUsbImuFrame(batch, &batch_size, &batch_count, &item);
        }
        else
        {
            data_router_log_stats.usb_stream_dropped++;
        }
#endif
    }

#if (DATA_ROUTER_ENABLE_USB_IMU_STREAM == 1U)
    DataRouter_FlushUsbImuBatch(batch, batch_size, batch_count);
#endif
}

/**
 * @brief 将单个发送槽同时记录到黑匣子并通过 USB 发出
 *
 * SD 记录发生在 DataRouterTask 上下文，因此无论有多少业务任务调用
 * DataRouter_PostFrame()，SD 环形缓冲区始终只有一个生产者。
 */
static void DataRouter_SendSlot(DataRouter_TxSlot_t *slot)
{
    if (slot == NULL)
    {
        return;
    }

    __atomic_fetch_add(&data_router_tx_stats.dispatched_frames, 1U, __ATOMIC_RELAXED);

    /*
     * DataRouterTask 是 SD 环形缓冲区的唯一生产者。这里先把完整 MAVLink
     * 字节流封装成统一日志帧，再执行 USB 发送；写环形缓冲区失败时只丢日志，
     * 不影响通信链路和飞控实时任务。
     */
#if (DATA_ROUTER_ENABLE_SD_MAVLINK_LOG == 1U)
    (void)SDCard_EnqueueFrame(SD_CARD_MSG_MAVLINK_TX, 1U, 0U, slot->timestamp_us, slot->data, slot->length);
#endif

    /* DataRouterTask 是物理发送接口的唯一所有者，杜绝 USB 并发访问。 */
#if ((DATA_ROUTER_ENABLE_USB_IMU_STREAM == 0U) || (DATA_ROUTER_ENABLE_MAVLINK_DURING_IMU_STREAM == 1U))
    const bool result = USBSend(data_router.usb_instance, slot->data, slot->length);
    if (!result)
    {
        __atomic_fetch_add(&data_router_tx_stats.usb_enqueue_drops, 1U, __ATOMIC_RELAXED);
    }
    Mavlink_ReportTxResult(result);
#endif
}

/**
 * @brief 将已经发送完成的槽归还空闲池
 */
static void DataRouter_ReleaseSlot(DataRouter_TxSlot_t *slot)
{
    if (slot == NULL)
    {
        return;
    }

    /*
     * 理论上归还时 free_queue 绝对有空间。
     * 因为系统的总槽数是固定的，一个槽要么在 free_queue，要么在 tx_queue，
     * 所以这里的发送延时可以直接写 portMAX_DELAY。
     */
    (void)xQueueSend(data_router.free_queue, &slot, portMAX_DELAY);
}

/**
 * @brief 连续处理发送队列中已经积压的全部帧
 * @param first_slot 第一次因为阻塞等待被唤醒时拿到的那个 Slot
 */
static void DataRouter_ProcessPendingFrames(DataRouter_TxSlot_t *first_slot)
{
    DataRouter_TxSlot_t *slot = first_slot;

    /* 1. 先处理把任务从沉睡中唤醒的那一帧 */
    if (slot != NULL)
    {
        DataRouter_SendSlot(slot);
        DataRouter_ReleaseSlot(slot);
    }

    /* 2. 继续检查是否有后续积压的帧，使用 0 阻塞等待。
     *    能拿到就一直发，拿不到就立刻退出循环。
     */
    while (xQueueReceive(data_router.tx_queue, &slot, 0U) == pdPASS)
    {
        DataRouter_SendSlot(slot);
        DataRouter_ReleaseSlot(slot);
    }
}

/* ========================== 初始化接口 ========================== */

bool DataRouter_Init(const DataRouter_Config_t *config)
{
    if (data_router.initialized)
    {
        return false;
    }

    if ((config == NULL) || (config->usb_instance == NULL))
    {
        return false;
    }

    /* 安全检查：如果有回调数量，则回调指针数组不能为空 */
    if ((config->update_callback_count > 0U) && (config->update_callbacks == NULL))
    {
        return false;
    }

    /* 清理内存环境，保存配置 */
    memset(&data_router, 0, sizeof(data_router));
    data_router.usb_instance          = config->usb_instance;
    data_router.update_callbacks      = config->update_callbacks;
    data_router.update_callback_count = config->update_callback_count;

    /* ---------------------------------------------------------
     * 队列初始化 (静态分配机制，无内存碎片风险)
     * --------------------------------------------------------- */
    data_router.free_queue = xQueueCreateStatic(DATA_ROUTER_TX_SLOT_COUNT, sizeof(DataRouter_TxSlot_t *),
                                                data_router.free_queue_storage, &data_router.free_queue_control);

    data_router.tx_queue = xQueueCreateStatic(DATA_ROUTER_TX_SLOT_COUNT, sizeof(DataRouter_TxSlot_t *),
                                              data_router.tx_queue_storage, &data_router.tx_queue_control);

    data_router.imu_log_queue = xQueueCreateStatic(DATA_ROUTER_IMU_LOG_DEPTH, sizeof(DataRouter_ImuLogItem_t),
                                                   data_router.imu_log_queue_storage,
                                                   &data_router.imu_log_queue_control);

    if ((data_router.free_queue == NULL) || (data_router.tx_queue == NULL) || (data_router.imu_log_queue == NULL))
    {
        RTTERROR("[ROUTER] Queue create failed.");
        return false;
    }

    /* ---------------------------------------------------------
     * 将物理内存池里的所有 Slot 初始化并放入 free_queue 当中
     * --------------------------------------------------------- */
    for (uint8_t i = 0U; i < DATA_ROUTER_TX_SLOT_COUNT; i++)
    {
        DataRouter_TxSlot_t *slot = &data_router.slots[i];

        if (xQueueSend(data_router.free_queue, &slot, 0U) != pdPASS)
        {
            RTTERROR("[ROUTER] Slot pool init failed.");
            return false;
        }
    }

    data_router.initialized     = 1U;
    data_router_log_stats       = (DataRouter_LogStats_t){0};
    data_router_tx_stats        = (DataRouter_TxStats_t){0};
    usb_capture_start_requested = 0U;
    usb_capture_session_token   = 0U;
    usb_capture_command_index   = 0U;
    usb_capture_candidate_token = 0U;
    usb_stream_header_pending   = 0U;
    RTTINFO("[ROUTER] Initialized.");

    return true;
}

/* ========================== 数据提交接口 ========================== */

bool DataRouter_PostFrame(const uint8_t *data, uint16_t length)
{
    DataRouter_TxSlot_t *slot = NULL;

    __atomic_fetch_add(&data_router_tx_stats.post_attempts, 1U, __ATOMIC_RELAXED);

    /* --- 参数安全校验 --- */
    if (!data_router.initialized)
    {
        __atomic_fetch_add(&data_router_tx_stats.invalid_rejects, 1U, __ATOMIC_RELAXED);
        return false;
    }
    if ((data == NULL) || (length == 0U) || (length > MAVLINK_MAX_PACKET_LEN))
    {
        __atomic_fetch_add(&data_router_tx_stats.invalid_rejects, 1U, __ATOMIC_RELAXED);
        return false;
    }

    /*
     * 1. 获取一个空闲的发送槽 (0 延时，不阻塞)
     * 如果拿不到，说明系统发送通道已经拥塞（超出8帧未发），直接返回 false 丢帧，保护业务任务不被卡死。
     */
    if (xQueueReceive(data_router.free_queue, &slot, 0U) != pdPASS)
    {
        __atomic_fetch_add(&data_router_tx_stats.slot_exhaustion_drops, 1U, __ATOMIC_RELAXED);
        return false;
    }

    /*
     * 2. 数据拷贝入槽
     * 注意：数据拷贝是发生在 FreeRTOS Queue 锁以外的，因此不会影响 RTOS 调度实时性。
     */
    memcpy(slot->data, data, length);
    slot->length       = length;
    slot->timestamp_us = Bsp_Timestamp_us_Get();

    /*
     * 3. 将装满数据的槽推入发送队列
     */
    if (xQueueSend(data_router.tx_queue, &slot, 0U) != pdPASS)
    {
        /* 异常保护：拿到了空闲槽，却放不进发送队列（理论上不会发生）。
         * 为防万一，把槽还回 free_queue，避免内存泄露。
         */
        (void)xQueueSend(data_router.free_queue, &slot, 0U);
        __atomic_fetch_add(&data_router_tx_stats.queue_send_drops, 1U, __ATOMIC_RELAXED);
        return false;
    }

    __atomic_fetch_add(&data_router_tx_stats.posted_frames, 1U, __ATOMIC_RELAXED);

    const uint32_t queued         = (uint32_t)uxQueueMessagesWaiting(data_router.tx_queue);
    uint32_t       high_watermark = __atomic_load_n(&data_router_tx_stats.queue_high_watermark, __ATOMIC_RELAXED);
    while ((queued > high_watermark) &&
           (!__atomic_compare_exchange_n(&data_router_tx_stats.queue_high_watermark, &high_watermark, queued, false,
                                         __ATOMIC_RELAXED, __ATOMIC_RELAXED)))
    {
        /* 其他生产者刚更新过峰值，使用 compare-exchange 返回的新值继续比较。 */
    }

    return true;
}

bool DataRouter_PostBMI088Imu(uint64_t timestamp_us, const SDCard_BMI088ImuPayloadV1_t *sample)
{
    if ((!data_router.initialized) || (sample == NULL) || (timestamp_us == 0ULL))
    {
        return false;
    }

    const DataRouter_ImuLogItem_t item = {.timestamp_us = timestamp_us, .payload = *sample};
    if (xQueueSend(data_router.imu_log_queue, &item, 0U) != pdPASS)
    {
        data_router_log_stats.imu_log_dropped++;
        return false;
    }

    data_router_log_stats.imu_log_enqueued++;
    return true;
}

void DataRouter_UsbCalibrationInputFromISR(const uint8_t *data, uint16_t size)
{
    if ((data == NULL) || (size == 0U))
    {
        return;
    }

    for (uint16_t index = 0U; index < size; index++)
    {
        const uint8_t byte = data[index];
        if (usb_capture_command_index < sizeof(usb_capture_start_prefix))
        {
            if (byte == usb_capture_start_prefix[usb_capture_command_index])
            {
                usb_capture_command_index++;
            }
            else
            {
                usb_capture_command_index = (byte == usb_capture_start_prefix[0]) ? 1U : 0U;
            }
            continue;
        }

        if (usb_capture_command_index == sizeof(usb_capture_start_prefix))
        {
            usb_capture_candidate_token = byte;
            usb_capture_command_index++;
            continue;
        }

        if (usb_capture_command_index == (sizeof(usb_capture_start_prefix) + 1U))
        {
            usb_capture_candidate_token |= (uint16_t)byte << 8U;
            usb_capture_command_index++;
            continue;
        }

        if ((byte == '\n') && (usb_capture_candidate_token != 0U) &&
            (usb_capture_candidate_token != usb_capture_session_token))
        {
            usb_capture_session_token   = usb_capture_candidate_token;
            usb_capture_start_requested = 1U;
        }
        usb_capture_candidate_token = 0U;
        usb_capture_command_index   = 0U;
    }
}

/* ========================== FreeRTOS 路由任务 ========================== */

void DataRouterTask(void *argument)
{
    (void)argument;
    DataRouter_TxSlot_t *slot           = NULL;
    uint64_t             last_update_us = 0U;

    /* 1ms 的 Tick 超时时间 */
    const TickType_t update_wait_ticks = pdMS_TO_TICKS(DATA_ROUTER_UPDATE_PERIOD_MS);

    if (!data_router.initialized)
    {
        vTaskDelete(NULL);
        return;
    }

    for (;;)
    {
        DataRouter_ProcessImuLogs();

        /*
         * 1. 挂起等待需要发送的数据
         * 巧妙之处：利用超时机制实现事件驱动和时间驱动的结合。
         * 如果队列有数据：立即被唤醒执行发送。
         * 如果队列没数据：最多死等 1ms 后醒来，保证后面的周期性 Service 不会被饿死。
        */
        const BaseType_t received = xQueueReceive(data_router.tx_queue, &slot, update_wait_ticks);

        /* --- 处理发送任务 --- */
        if (received == pdPASS)
        {
            DataRouter_ProcessPendingFrames(slot);
        }

        DataRouter_ProcessImuLogs();

        /*
         * 2. 检查是否到了 1ms 的周期调用窗口
         * 各 Service 内部会自己去拿时间戳判断是否要发心跳包等，此处仅提供基础时钟 Tick。
         */
        const uint64_t now_us = Bsp_Timestamp_us_Get();

        if ((last_update_us == 0U) || ((now_us - last_update_us) >= ((uint64_t)DATA_ROUTER_UPDATE_PERIOD_MS * 1000ULL)))
        {
            last_update_us = now_us;
            DataRouter_ProcessUpdates(now_us);
        }
    }
}
