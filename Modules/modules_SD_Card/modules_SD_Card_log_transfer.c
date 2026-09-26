#include "modules_SD_Card_internal.h"

/**
 * @file modules_SD_Card_log_transfer.c
 * @brief 在通信任务与 SD 卡所有者任务之间异步传递日志请求和响应。
 *
 * 通信侧只负责把 MAVLink 请求写入静态队列；目录遍历、文件打开和分块读取全部在
 * SD 卡任务中执行，避免多个任务同时调用 FatFs。响应再通过队列返回通信侧编码发送。
 */

typedef struct
{
    SDCard_LogResponse_t response;
} SDCard_LogResponseSlot_t;

/*
 * 请求体较小，继续按值入队。响应体包含 90 字节数据区，改用静态槽池：
 * FreeRTOS 队列中只复制 32 位槽指针，大块响应的复制在队列临界区外完成。
 */
static QueueHandle_t sd_log_request_queue;
static QueueHandle_t sd_log_response_free_queue;
static QueueHandle_t sd_log_response_ready_queue;
static StaticQueue_t sd_log_request_queue_control;
static StaticQueue_t sd_log_response_free_queue_control;
static StaticQueue_t sd_log_response_ready_queue_control;
static uint8_t       sd_log_request_queue_storage[SD_CARD_LOG_REQUEST_DEPTH * sizeof(SDCard_LogRequest_t)];
static uint8_t sd_log_response_free_queue_storage[SD_CARD_LOG_RESPONSE_DEPTH * sizeof(SDCard_LogResponseSlot_t *)];
static uint8_t sd_log_response_ready_queue_storage[SD_CARD_LOG_RESPONSE_DEPTH * sizeof(SDCard_LogResponseSlot_t *)];
static SDCard_LogResponseSlot_t sd_log_response_slots[SD_CARD_LOG_RESPONSE_DEPTH];

static SDCard_LogTransferContext_t sd_log_context;
static SDCard_LogTransfer_t        sd_log_transfer;

/**
 * @brief 校验 LOGxxxxx.BIN 命名并可选解析五位十进制日志编号。
 * @note 文件名大小写和长度均严格匹配，避免把其他文件误报为飞行日志。
 */
static bool SDCard_ParseLogName(const char *name, uint16_t *id)
{
    if ((name == NULL) || (strlen(name) != 12U) || (name[0] != 'L') || (name[1] != 'O') || (name[2] != 'G') ||
        (name[8] != '.') || (name[9] != 'B') || (name[10] != 'I') || (name[11] != 'N'))
    {
        return false;
    }

    uint32_t value = 0U;
    for (uint8_t index = 3U; index < 8U; index++)
    {
        if ((name[index] < '0') || (name[index] > '9'))
        {
            return false;
        }
        value = value * 10U + (uint32_t)(name[index] - '0');
    }
    if (id == NULL)
    {
        return true;
    }
    if (value > UINT16_MAX)
    {
        return false;
    }

    *id = (uint16_t)value;
    return true;
}

/** @brief 判断目录项是否符合框架日志文件命名规则。 */
bool SDCard_LogTransfer_IsLogFileName(const char *name)
{
    return SDCard_ParseLogName(name, NULL);
}

/** @brief 保存 SD 所有者上下文，并创建请求队列及响应槽指针队列。 */
bool SDCard_LogTransfer_Init(const SDCard_LogTransferContext_t *context)
{
    if ((context == NULL) || (context->live_log_file == NULL) || (context->state == NULL) ||
        (context->sync_live_file == NULL) || (context->erase_logs == NULL))
    {
        return false;
    }

    sd_log_context = *context;
    memset(&sd_log_transfer, 0, sizeof(sd_log_transfer));

    sd_log_request_queue        = xQueueCreateStatic(SD_CARD_LOG_REQUEST_DEPTH, sizeof(SDCard_LogRequest_t),
                                                     sd_log_request_queue_storage, &sd_log_request_queue_control);
    sd_log_response_free_queue  = xQueueCreateStatic(SD_CARD_LOG_RESPONSE_DEPTH, sizeof(SDCard_LogResponseSlot_t *),
                                                     sd_log_response_free_queue_storage,
                                                     &sd_log_response_free_queue_control);
    sd_log_response_ready_queue = xQueueCreateStatic(SD_CARD_LOG_RESPONSE_DEPTH, sizeof(SDCard_LogResponseSlot_t *),
                                                     sd_log_response_ready_queue_storage,
                                                     &sd_log_response_ready_queue_control);

    if ((sd_log_request_queue == NULL) || (sd_log_response_free_queue == NULL) || (sd_log_response_ready_queue == NULL))
    {
        return false;
    }

    memset(sd_log_response_slots, 0, sizeof(sd_log_response_slots));
    for (uint8_t index = 0U; index < SD_CARD_LOG_RESPONSE_DEPTH; index++)
    {
        SDCard_LogResponseSlot_t *slot = &sd_log_response_slots[index];
        if (xQueueSend(sd_log_response_free_queue, &slot, 0U) != pdPASS)
        {
            return false;
        }
    }
    return true;
}

/** @brief 关闭当前目录或独立读句柄，并复位正在进行的列表/下载状态。 */
void SDCard_LogTransfer_Close(void)
{
    if (sd_log_transfer.list_active != 0U)
    {
        (void)f_closedir(&sd_log_transfer.list_dir);
    }
    if (sd_log_transfer.read_file_open == 1U)
    {
        (void)f_close(&sd_log_transfer.read_file);
    }
    memset(&sd_log_transfer, 0, sizeof(sd_log_transfer));
}

/** @brief 非阻塞取得空闲槽，在临界区外复制响应，再把槽指针提交给通信任务。 */
static bool SDCard_PushLogResponse(const SDCard_LogResponse_t *response)
{
    SDCard_LogResponseSlot_t *slot = NULL;

    if ((response == NULL) || (sd_log_response_free_queue == NULL) || (sd_log_response_ready_queue == NULL) ||
        (xQueueReceive(sd_log_response_free_queue, &slot, 0U) != pdPASS) || (slot == NULL))
    {
        return false;
    }

    /* 百字节级响应体的复制不再位于 FreeRTOS 队列临界区内。 */
    memcpy(&slot->response, response, sizeof(slot->response));

    if (xQueueSend(sd_log_response_ready_queue, &slot, 0U) != pdPASS)
    {
        /* 两个指针队列容量相同，正常所有权流转下不会失败；异常时归还槽避免泄漏。 */
        (void)xQueueSend(sd_log_response_free_queue, &slot, 0U);
        return false;
    }
    return true;
}

/** @brief 判断是否还有空闲响应槽，避免读取文件后因无槽可用而丢失分片。 */
static bool SDCard_HasFreeResponseSlot(void)
{
    return (sd_log_response_free_queue != NULL) && (uxQueueMessagesWaiting(sd_log_response_free_queue) > 0U);
}

/**
 * @brief 打开指定编号的日志用于读取。
 *
 * 若目标正是当前写入文件，FatFs 可能因锁冲突拒绝再次打开，此时复用 SD 任务持有的
 * 活跃句柄，并用 read_file_open=2 标记后续读取需要恢复原写入偏移。
 */
static bool SDCard_OpenReadFile(uint16_t id)
{
    const int length = snprintf(sd_log_transfer.read_path, sizeof(sd_log_transfer.read_path), SD_CARD_LOG_PATH_FORMAT,
                                (unsigned long)id);
    if ((length <= 0) || ((size_t)length >= sizeof(sd_log_transfer.read_path)))
    {
        return false;
    }

    if (f_open(&sd_log_transfer.read_file, sd_log_transfer.read_path, FA_READ) == FR_OK)
    {
        sd_log_transfer.read_file_open = 1U;
        return true;
    }

    /* A locked live file is read through its existing handle by the SD owner task. */
    if (strncmp(sd_log_transfer.read_path, sd_log_context.state->file_path, sizeof(sd_log_context.state->file_path)) ==
        0)
    {
        sd_log_transfer.read_file_open = 2U;
        return true;
    }
    return false;
}

/**
 * @brief 从当前日志的绝对偏移读取一块数据。
 * @note 复用活跃日志句柄时，读取完成后恢复文件偏移，避免破坏后续追加写入位置。
 */
static UINT SDCard_ReadAt(uint32_t offset, uint8_t *data, UINT capacity)
{
    FIL *file = (sd_log_transfer.read_file_open == 1U) ? &sd_log_transfer.read_file : sd_log_context.live_log_file;
    if ((sd_log_transfer.read_file_open == 0U) || (sd_log_transfer.read_file_open > 2U))
    {
        return 0U;
    }

    const bool    restore_live_offset = (sd_log_transfer.read_file_open == 2U);
    const FSIZE_t saved_offset        = restore_live_offset ? f_tell(file) : 0U;
    UINT          read                = 0U;
    if ((f_lseek(file, offset) != FR_OK) || (f_read(file, data, capacity, &read) != FR_OK))
    {
        read = 0U;
    }
    if (restore_live_offset)
    {
        (void)f_lseek(file, saved_offset);
    }
    return read;
}

/**
 * @brief 启动日志目录枚举并预先统计日志总数及最大编号。
 * @note 目录为空时立即产生一条 num_logs=0 的 LOG_ENTRY 响应。
 */
static void SDCard_StartLogList(uint16_t start, uint16_t end)
{
    FILINFO  info;
    DIR      directory;
    uint16_t id;
    uint32_t total   = 0U;
    uint16_t last_id = 0U;

    SDCard_LogTransfer_Close();
    if (sd_log_context.state->file_open != 0U)
    {
        (void)sd_log_context.sync_live_file();
    }
    if (f_opendir(&directory, SD_CARD_LOG_DIRECTORY) != FR_OK)
    {
        return;
    }

    for (;;)
    {
        if ((f_readdir(&directory, &info) != FR_OK) || (info.fname[0] == '\0'))
        {
            break;
        }
        if (SDCard_ParseLogName(info.fname, &id))
        {
            total++;
            if (id > last_id)
            {
                last_id = id;
            }
        }
    }
    (void)f_closedir(&directory);

    if (total == 0U)
    {
        const SDCard_LogResponse_t response = {
            .type         = SD_CARD_LOG_RESPONSE_ENTRY,
            .id           = 0U,
            .num_logs     = 0U,
            .last_log_num = 0U,
            .time_utc     = 0U,
            .size         = 0U,
        };
        (void)SDCard_PushLogResponse(&response);
        return;
    }

    if (f_opendir(&sd_log_transfer.list_dir, SD_CARD_LOG_DIRECTORY) != FR_OK)
    {
        return;
    }
    sd_log_transfer.list_active  = 1U;
    sd_log_transfer.list_start   = start;
    sd_log_transfer.list_end     = end;
    sd_log_transfer.list_count   = (uint16_t)((total > UINT16_MAX) ? UINT16_MAX : total);
    sd_log_transfer.list_last_id = last_id;
}

/** @brief 同步活跃日志后启动指定编号、偏移和长度的数据读取状态机。 */
static void SDCard_StartLogData(uint16_t id, uint32_t offset, uint32_t count)
{
    FILINFO info;

    SDCard_LogTransfer_Close();
    if (count == 0U)
    {
        return;
    }
    if (sd_log_context.state->file_open != 0U)
    {
        (void)sd_log_context.sync_live_file();
    }
    if (!SDCard_OpenReadFile(id))
    {
        return;
    }
    if (f_stat(sd_log_transfer.read_path, &info) != FR_OK)
    {
        SDCard_LogTransfer_Close();
        return;
    }

    sd_log_transfer.id        = id;
    sd_log_transfer.offset    = offset;
    sd_log_transfer.remaining = count;
    if (offset >= info.fsize)
    {
        sd_log_transfer.remaining = 0U;
        SDCard_LogTransfer_Close();
    }
}

/**
 * @brief 在 SD 卡任务中消费全部待处理请求并切换传输状态。
 * @note 新的列表或数据请求会先关闭上一传输；擦除也只在 SD 所有者任务执行。
 */
void SDCard_LogTransfer_ProcessRequests(void)
{
    SDCard_LogRequest_t request;

    while (xQueueReceive(sd_log_request_queue, &request, 0U) == pdPASS)
    {
        switch (request.type)
        {
        case SD_CARD_LOG_REQUEST_LIST:
            SDCard_StartLogList(request.start, request.end);
            break;
        case SD_CARD_LOG_REQUEST_DATA:
            SDCard_StartLogData(request.id, request.offset, request.count);
            break;
        case SD_CARD_LOG_REQUEST_ERASE:
            (void)sd_log_context.erase_logs();
            break;
        case SD_CARD_LOG_REQUEST_END:
        default:
            SDCard_LogTransfer_Close();
            break;
        }
    }
}

/**
 * @brief 增量推进目录枚举或文件下载，每次调用最多生成四条响应。
 *
 * 响应队列无空间时立即让出，下一轮继续，防止日志传输长时间占用 SD 任务。
 * LOG_DATA 每包最多携带 SD_CARD_LOG_DATA_CHUNK_SIZE 字节。
 */
void SDCard_LogTransfer_Process(void)
{
    SDCard_LogResponse_t response = {0};
    FILINFO              info;
    uint16_t             id;

    for (uint8_t budget = 0U; budget < 4U; budget++)
    {
        if (sd_log_transfer.list_active != 0U)
        {
            if (!SDCard_HasFreeResponseSlot())
            {
                return;
            }
            if ((f_readdir(&sd_log_transfer.list_dir, &info) != FR_OK) || (info.fname[0] == '\0'))
            {
                SDCard_LogTransfer_Close();
                continue;
            }
            if (!SDCard_ParseLogName(info.fname, &id) || (id < sd_log_transfer.list_start) ||
                (id > sd_log_transfer.list_end))
            {
                continue;
            }

            response.type         = SD_CARD_LOG_RESPONSE_ENTRY;
            response.id           = id;
            response.num_logs     = sd_log_transfer.list_count;
            response.last_log_num = sd_log_transfer.list_last_id;
            response.time_utc     = 0U;
            response.size         = info.fsize;
            if (!SDCard_PushLogResponse(&response))
            {
                return;
            }
            continue;
        }

        if ((sd_log_transfer.read_file_open == 0U) || (sd_log_transfer.remaining == 0U))
        {
            if (sd_log_transfer.read_file_open != 0U)
            {
                SDCard_LogTransfer_Close();
            }
            return;
        }
        if (!SDCard_HasFreeResponseSlot())
        {
            return;
        }

        memset(&response, 0, sizeof(response));
        response.type       = SD_CARD_LOG_RESPONSE_DATA;
        response.id         = sd_log_transfer.id;
        response.offset     = sd_log_transfer.offset;
        const UINT capacity = (UINT)((sd_log_transfer.remaining > SD_CARD_LOG_DATA_CHUNK_SIZE)
                                         ? SD_CARD_LOG_DATA_CHUNK_SIZE
                                         : sd_log_transfer.remaining);
        const UINT read     = SDCard_ReadAt(sd_log_transfer.offset, response.data, capacity);
        if (read == 0U)
        {
            SDCard_LogTransfer_Close();
            return;
        }

        response.count = (uint8_t)read;
        if (!SDCard_PushLogResponse(&response))
        {
            return;
        }
        sd_log_transfer.offset += read;
        sd_log_transfer.remaining -= read;
    }
}

/** @brief 从非 SD 任务上下文非阻塞投递请求。 */
static bool SDCard_QueueLogRequest(const SDCard_LogRequest_t *request)
{
    return (request != NULL) && (sd_log_request_queue != NULL) &&
           (xQueueSend(sd_log_request_queue, request, 0U) == pdPASS);
}

/** @brief 请求枚举闭区间 [start, end] 内的日志条目。 */
bool SDCard_RequestLogList(uint16_t start, uint16_t end)
{
    const SDCard_LogRequest_t request = {
        .type  = SD_CARD_LOG_REQUEST_LIST,
        .start = start,
        .end   = end,
    };
    return SDCard_QueueLogRequest(&request);
}

/** @brief 请求读取日志数据；单次请求长度会被限制到模块允许的最大值。 */
bool SDCard_RequestLogData(uint16_t id, uint32_t offset, uint32_t count)
{
    const SDCard_LogRequest_t request = {
        .type   = SD_CARD_LOG_REQUEST_DATA,
        .id     = id,
        .offset = offset,
        .count  = (count > SD_CARD_LOG_MAX_REQUEST_BYTES) ? SD_CARD_LOG_MAX_REQUEST_BYTES : count,
    };
    return SDCard_QueueLogRequest(&request);
}

/** @brief 请求终止当前目录枚举或文件下载。 */
bool SDCard_RequestLogEnd(void)
{
    const SDCard_LogRequest_t request = {.type = SD_CARD_LOG_REQUEST_END};
    return SDCard_QueueLogRequest(&request);
}

/** @brief 请求由 SD 卡任务擦除日志，调用本身不执行文件系统操作。 */
bool SDCard_RequestLogErase(void)
{
    const SDCard_LogRequest_t request = {.type = SD_CARD_LOG_REQUEST_ERASE};
    return SDCard_QueueLogRequest(&request);
}

/** @brief 供通信任务复制一条就绪响应，并立即把静态槽归还空闲池。 */
bool SDCard_PopLogResponse(SDCard_LogResponse_t *response)
{
    SDCard_LogResponseSlot_t *slot = NULL;

    if ((response == NULL) || (sd_log_response_free_queue == NULL) || (sd_log_response_ready_queue == NULL) ||
        (xQueueReceive(sd_log_response_ready_queue, &slot, 0U) != pdPASS) || (slot == NULL))
    {
        return false;
    }

    /* 调用方得到独立副本后，生产者即可安全复用该槽。 */
    memcpy(response, &slot->response, sizeof(*response));
    return xQueueSend(sd_log_response_free_queue, &slot, 0U) == pdPASS;
}
