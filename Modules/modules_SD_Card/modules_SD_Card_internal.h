/**
 * @file modules_SD_Card_internal.h
 * @brief SD 日志文件传输子模块的内部接口与容量约束
 */
#ifndef MODULES_SD_CARD_INTERNAL_H
#define MODULES_SD_CARD_INTERNAL_H

#include "modules_SD_Card.h"
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"

#define SD_CARD_LOG_DIRECTORY         "0:/BLACKBOX"
#define SD_CARD_LOG_PATH_FORMAT       "0:/BLACKBOX/LOG%05lu.BIN"
#define SD_CARD_LOG_REQUEST_DEPTH     8U
#define SD_CARD_LOG_RESPONSE_DEPTH    32U
#define SD_CARD_LOG_MAX_REQUEST_BYTES 8190U

typedef enum
{
    SD_CARD_LOG_REQUEST_LIST = 0U,
    SD_CARD_LOG_REQUEST_DATA,
    SD_CARD_LOG_REQUEST_END,
    SD_CARD_LOG_REQUEST_ERASE
} SDCard_LogRequestType_e;

typedef struct
{
    SDCard_LogRequestType_e type;
    uint16_t                id;
    uint16_t                start;
    uint16_t                end;
    uint32_t                offset;
    uint32_t                count;
} SDCard_LogRequest_t;

typedef struct
{
    uint8_t  list_active;
    uint8_t  read_file_open;
    uint16_t id;
    uint16_t list_start;
    uint16_t list_end;
    uint16_t list_count;
    uint16_t list_last_id;
    uint32_t offset;
    uint32_t remaining;
    DIR      list_dir;
    FIL      read_file;
    char     read_path[32];
} SDCard_LogTransfer_t;

/**
 * @brief 日志传输层访问实时日志文件和擦除操作所需的依赖。
 * @note FatFS 文件所有权仍属于 SD 任务，其他任务只能通过请求/响应队列交互。
 */
typedef struct
{
    FIL            *live_log_file;
    SDCard_State_t *state;
    bool (*sync_live_file)(void);
    bool (*erase_logs)(void);
} SDCard_LogTransferContext_t;

/** @brief 初始化日志请求队列、静态响应槽池及指针队列，并绑定 SD 文件上下文。 */
bool SDCard_LogTransfer_Init(const SDCard_LogTransferContext_t *context);

/** @brief 关闭当前下载文件并清理正在进行的传输状态。 */
void SDCard_LogTransfer_Close(void);

/** @brief 非阻塞消费上层请求队列，建立新的枚举、读取、结束或擦除操作。 */
void SDCard_LogTransfer_ProcessRequests(void);

/** @brief 在 SD 任务中推进当前日志枚举或分块读取状态机。 */
void SDCard_LogTransfer_Process(void);

/** @brief 检查文件名是否严格符合 LOGxxxxx.BIN 格式。 */
bool SDCard_LogTransfer_IsLogFileName(const char *name);

#endif /* MODULES_SD_CARD_INTERNAL_H */
