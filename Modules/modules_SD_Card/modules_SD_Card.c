/**
 * @file modules_SD_Card.c
 * @brief 基于 FatFS、SDMMC DMA 与 SPSC 环形缓冲区的最小黑匣子实现。
 */

#include "modules_SD_Card_internal.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "App_TaskPriorities.h"
#include "FreeRTOS.h"
#include "bsp_RTT.h"
#include "bsp_memory_section.h"
#include "bsp_ringbuffer.h"
#include "bsp_timestamp.h"
#include "fatfs.h"
#include "module_pwm_motor.h"
#include "task.h"
#include "user_math.h"

/* ========================== 文件与任务配置 ========================== */

#define SD_CARD_MAX_FILE_INDEX 99999UL
#define SD_CARD_IDLE_DELAY_MS  5U

/* ========================== 编译期约束 ========================== */

/* 文件格式属于离线解析协议，结构体尺寸变化时必须显式升级格式版本。 */
_Static_assert(sizeof(SDCard_FileHeader_t) == 24U, "SD log file header layout changed");
_Static_assert(sizeof(SDCard_FrameHeader_t) == 20U, "SD log frame header layout changed");
_Static_assert(sizeof(SDCard_BMI088ImuPayloadV1_t) == 52U, "BMI088 log payload layout changed");
_Static_assert((SD_CARD_RING_BUFFER_SIZE & (SD_CARD_RING_BUFFER_SIZE - 1U)) == 0U,
               "SD ring buffer size must be a power of two");

/* ========================== 私有变量 ========================== */

/*
 * 状态快照也放在非缓存 D2 SRAM，使 J-Link/VS Code 通过调试端口读取时不会
 * 看到 D1 Write-back Cache 中尚未回写的旧值。
 */
SDCard_State_t sd_card_state DMA_BUFFER;

/* DataRouter 单生产者写入，SDCard_Task 单消费者读出，不需要互斥锁。 */
static RingBuffer_t sd_ring_buffer;
static uint8_t      sd_ring_pool[SD_CARD_RING_BUFFER_SIZE];

/*
 * FatFS 最终会把扇区交给 SDMMC 内部 IDMA。STM32H743 要求该 IDMA 缓冲区
 * 位于 AXI SRAM（D1）；sd_diskio 在传输边界负责 D-Cache clean/invalidate。
 */
static uint8_t sd_write_buffer[SD_CARD_WRITE_BUFFER_SIZE] SDMMC_DMA_BUFFER;

static StaticTask_t sd_task_control;
static StackType_t  sd_task_stack[SD_CARD_TASK_STACK_WORDS];
static TaskHandle_t sd_task_handle;
static FIL          sd_log_file;
static uint8_t      sd_module_initialized;

/* ========================== 状态与文件辅助函数 ========================== */

/**
 * @brief 同时更新模块阶段状态和对应的 FatFS 返回值
 *
 * 将两项诊断信息集中更新，避免各错误分支只修改其中一个字段而造成误判。
 */
static void SDCard_SetStatus(SDCard_Status_e status, FRESULT result)
{
    sd_card_state.status       = status;
    sd_card_state.last_fresult = result;
}

/**
 * @brief 刷新环形缓冲区当前占用量和运行期峰值
 *
 * 统计值只供调试观察，不参与生产者与消费者之间的同步。
 */
static void SDCard_UpdateRingUsage(void)
{
    const uint32_t used = RingBuffer_GetUsed(&sd_ring_buffer);

    sd_card_state.ring_used_bytes = used;
    if (used > sd_card_state.ring_peak_bytes)
    {
        sd_card_state.ring_peak_bytes = used;
    }
}

/**
 * @brief 顺序查找第一个尚未存在的日志文件路径
 *
 * 仅检查文件是否存在，不删除或覆盖旧日志；最终打开文件时仍使用
 * FA_CREATE_NEW，防止路径检查与创建之间出现意外覆盖。
 */
static bool SDCard_SelectFilePath(void)
{
    FILINFO file_info;

    for (uint32_t index = 0U; index <= SD_CARD_MAX_FILE_INDEX; index++)
    {
        const int length = snprintf(sd_card_state.file_path, sizeof(sd_card_state.file_path), SD_CARD_LOG_PATH_FORMAT,
                                    (unsigned long)index);

        if ((length <= 0) || ((size_t)length >= sizeof(sd_card_state.file_path)))
        {
            return false;
        }

        const FRESULT result = f_stat(sd_card_state.file_path, &file_info);
        if (result == FR_NO_FILE)
        {
            return true;
        }
        if (result != FR_OK)
        {
            sd_card_state.last_fresult = result;
            return false;
        }
    }

    return false;
}

/**
 * @brief 完整写入一批数据并记录写入耗时
 *
 * @param data   待写数据首地址
 * @param length 期望写入字节数
 * @return true 表示 FatFS 返回成功且实际写入长度完全相等
 *
 * 该函数只允许由 SDCard_Task 调用，因此不需要文件互斥锁。FatFS 返回短写
 * 也按错误处理，避免离线日志出现无法定位的半帧。
 */
static bool SDCard_WriteExact(const void *data, uint32_t length)
{
    UINT written = 0U;

    const uint64_t start_us   = Bsp_Timestamp_us_Get();
    const FRESULT  result     = f_write(&sd_log_file, data, length, &written);
    const uint32_t elapsed_us = (uint32_t)(Bsp_Timestamp_us_Get() - start_us);

    sd_card_state.last_write_time_us = elapsed_us;
    if (elapsed_us > sd_card_state.max_write_time_us)
    {
        sd_card_state.max_write_time_us = elapsed_us;
    }

    if ((result != FR_OK) || (written != length))
    {
        sd_card_state.write_error_count++;
        sd_card_state.dropped_bytes += length - written;
        SDCard_SetStatus(SD_CARD_STATUS_DATA_WRITE_FAILED, result);
        return false;
    }

    sd_card_state.write_count++;
    sd_card_state.bytes_written += written;
    sd_card_state.current_file_size = f_size(&sd_log_file);
    return true;
}

/**
 * @brief 将 FatFS 缓存强制同步到 SD 卡并记录阻塞耗时
 *
 * 周期同步限制掉电时可能丢失的数据窗口；同步失败后停止记录，避免继续生成
 * 表面递增但实际无法落盘的统计值。
 */
static bool SDCard_Sync(void)
{
    const uint64_t start_us   = Bsp_Timestamp_us_Get();
    const FRESULT  result     = f_sync(&sd_log_file);
    const uint32_t elapsed_us = (uint32_t)(Bsp_Timestamp_us_Get() - start_us);

    sd_card_state.last_sync_time_us = elapsed_us;
    if (elapsed_us > sd_card_state.max_sync_time_us)
    {
        sd_card_state.max_sync_time_us = elapsed_us;
    }

    if (result != FR_OK)
    {
        sd_card_state.sync_error_count++;
        SDCard_SetStatus(SD_CARD_STATUS_SYNC_FAILED, result);
        return false;
    }

    sd_card_state.sync_count++;
    sd_card_state.current_file_size = f_size(&sd_log_file);
    return true;
}

/**
 * @brief 写入固定文件头并立即执行同步、回读和逐字节比较
 *
 * 只有该自检通过后模块才进入 RUNNING，能够同时验证文件写入、SDMMC DMA、
 * D-Cache 一致性、文件定位和读取链路。
 */
static bool SDCard_WriteAndVerifyFileHeader(void)
{
    SDCard_FileHeader_t expected_header = {
        .magic              = SD_CARD_FILE_MAGIC,
        .format_version     = SD_CARD_FORMAT_VERSION,
        .header_size        = sizeof(SDCard_FileHeader_t),
        .start_timestamp_us = Bsp_Timestamp_us_Get(),
        .format_flags       = SD_CARD_FORMAT_FLAG_LITTLE_ENDIAN | SD_CARD_FORMAT_FLAG_IEEE754_FLOAT,
        .header_crc16       = 0U,
        .reserved           = 0U,
    };

    expected_header.header_crc16 = Math_Crc16Ccitt(&expected_header, offsetof(SDCard_FileHeader_t, header_crc16),
                                                   SD_CARD_CRC_INITIAL);

    memcpy(sd_write_buffer, &expected_header, sizeof(expected_header));
    if (!SDCard_WriteExact(sd_write_buffer, sizeof(expected_header)) || !SDCard_Sync())
    {
        SDCard_SetStatus(SD_CARD_STATUS_HEADER_WRITE_FAILED, sd_card_state.last_fresult);
        return false;
    }

    FRESULT result = f_lseek(&sd_log_file, 0U);
    if (result != FR_OK)
    {
        SDCard_SetStatus(SD_CARD_STATUS_SELF_TEST_FAILED, result);
        return false;
    }

    UINT bytes_read = 0U;
    result          = f_read(&sd_log_file, sd_write_buffer, sizeof(expected_header), &bytes_read);

    if ((result != FR_OK) || (bytes_read != sizeof(expected_header)) ||
        (memcmp(sd_write_buffer, &expected_header, sizeof(expected_header)) != 0))
    {
        SDCard_SetStatus(SD_CARD_STATUS_SELF_TEST_FAILED, result);
        return false;
    }

    result = f_lseek(&sd_log_file, f_size(&sd_log_file));
    if (result != FR_OK)
    {
        SDCard_SetStatus(SD_CARD_STATUS_SELF_TEST_FAILED, result);
        return false;
    }

    sd_card_state.self_test_passed = 1U;
    return true;
}

/**
 * @brief 挂载文件系统、建立日志目录、创建新文件并完成文件头自检
 *
 * 本函数不会自动格式化 SD 卡，避免在文件系统异常时破坏已有数据。
 */
static bool SDCard_OpenLogFile(void)
{
    FRESULT result = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
    if (result != FR_OK)
    {
        SDCard_SetStatus(SD_CARD_STATUS_MOUNT_FAILED, result);
        return false;
    }
    sd_card_state.mounted = 1U;

    result = f_mkdir(SD_CARD_LOG_DIRECTORY);
    if ((result != FR_OK) && (result != FR_EXIST))
    {
        SDCard_SetStatus(SD_CARD_STATUS_DIRECTORY_FAILED, result);
        return false;
    }

    if (!SDCard_SelectFilePath())
    {
        SDCard_SetStatus(SD_CARD_STATUS_FILE_OPEN_FAILED, sd_card_state.last_fresult);
        return false;
    }

    result = f_open(&sd_log_file, sd_card_state.file_path, FA_CREATE_NEW | FA_READ | FA_WRITE);
    if (result != FR_OK)
    {
        SDCard_SetStatus(SD_CARD_STATUS_FILE_OPEN_FAILED, result);
        return false;
    }
    sd_card_state.file_open = 1U;

    return SDCard_WriteAndVerifyFileHeader();
}

static FRESULT SDCard_PrepareErase(void)
{
    FRESULT result = FR_OK;

    if (sd_card_state.file_open != 0U)
    {
        result                          = f_close(&sd_log_file);
        sd_card_state.file_open         = 0U;
        sd_card_state.current_file_size = 0U;
        if (result != FR_OK)
        {
            return result;
        }
    }

    result = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
    if (result != FR_OK)
    {
        sd_card_state.mounted = 0U;
        return result;
    }
    sd_card_state.mounted = 1U;
    return FR_OK;
}

static FRESULT SDCard_DeleteLogFiles(uint32_t *deleted)
{
    DIR     directory;
    FILINFO info;
    FRESULT result;
    char    path[sizeof(sd_card_state.file_path)];

    if (deleted == NULL)
    {
        return FR_INVALID_PARAMETER;
    }
    *deleted = 0U;

    result = f_opendir(&directory, SD_CARD_LOG_DIRECTORY);
    if (result == FR_NO_PATH)
    {
        return FR_OK;
    }
    if (result != FR_OK)
    {
        return result;
    }

    for (;;)
    {
        if (Motor_IsArmed() != 0U)
        {
            result = FR_LOCKED;
            break;
        }

        result = f_readdir(&directory, &info);
        if ((result != FR_OK) || (info.fname[0] == '\0'))
        {
            break;
        }
        if (((info.fattrib & AM_DIR) != 0U) || !SDCard_LogTransfer_IsLogFileName(info.fname))
        {
            continue;
        }

        const int length = snprintf(path, sizeof(path), "%s/%s", SD_CARD_LOG_DIRECTORY, info.fname);
        if ((length <= 0) || ((size_t)length >= sizeof(path)))
        {
            result = FR_INVALID_NAME;
            break;
        }

        result = f_unlink(path);
        if (result != FR_OK)
        {
            break;
        }
        (*deleted)++;
    }

    (void)f_closedir(&directory);
    return result;
}

static bool SDCard_RestartAfterErase(uint32_t deleted)
{
    if (!RingBuffer_Init(&sd_ring_buffer, sd_ring_pool, sizeof(sd_ring_pool)))
    {
        SDCard_SetStatus(SD_CARD_STATUS_ERASE_FAILED, FR_INVALID_PARAMETER);
        return false;
    }
    sd_card_state.ring_used_bytes  = 0U;
    sd_card_state.ring_peak_bytes  = 0U;
    sd_card_state.self_test_passed = 0U;
    sd_card_state.file_path[0]     = '\0';

    SDCard_SetStatus(SD_CARD_STATUS_STARTING, FR_OK);
    if (!SDCard_OpenLogFile())
    {
        const FRESULT result = sd_card_state.last_fresult;
        SDCard_SetStatus(SD_CARD_STATUS_ERASE_FAILED, result);
        RTTERROR("[SD] Deleted %lu logs but blackbox restart failed, FR=%u.", (unsigned long)deleted, (unsigned)result);
        return false;
    }

    SDCard_SetStatus(SD_CARD_STATUS_RUNNING, FR_OK);
    RTTINFO("[SD] Deleted %lu logs; binary blackbox restarted.", (unsigned long)deleted);
    return true;
}

/**
 * @brief 删除 BLACKBOX 中的日志并重新启动黑匣子
 *
 * 仅由 SDCard_Task 调用。删除期间停止生产者入队，并在每个文件前复查电机状态，
 * 避免飞行过程中执行耗时的 FAT 元数据更新。
 */
static bool SDCard_EraseLogs(void)
{
    uint32_t deleted = 0U;

    if (Motor_IsArmed() != 0U)
    {
        RTTWARNING("[SD] Log erase rejected while motor is armed.");
        return false;
    }

    SDCard_SetStatus(SD_CARD_STATUS_ERASING, FR_OK);
    SDCard_LogTransfer_Close();

    FRESULT result = SDCard_PrepareErase();
    if (result == FR_OK)
    {
        result = SDCard_DeleteLogFiles(&deleted);
    }
    if ((result == FR_OK) && (Motor_IsArmed() != 0U))
    {
        result = FR_LOCKED;
    }
    if (result != FR_OK)
    {
        SDCard_SetStatus(SD_CARD_STATUS_ERASE_FAILED, result);
        RTTERROR("[SD] Log erase stopped after %lu files, FR=%u.", (unsigned long)deleted, (unsigned)result);
        return false;
    }
    return SDCard_RestartAfterErase(deleted);
}

/* ========================== 初始化接口 ========================== */

/**
 * @brief 初始化 SPSC 环形缓冲区并创建低优先级写卡任务
 */
bool SDCard_Init(void)
{
    /*
     * sd_card_state 位于 NOLOAD 的非缓存 D2 区，上电值不保证为 0，因此不能
     * 用其中的 initialized 作为首次初始化守卫。该普通 .bss 变量会由启动代码
     * 清零，专门负责防止同一次运行中重复创建任务。
     */
    if (sd_module_initialized != 0U)
    {
        return true;
    }

    memset((void *)&sd_card_state, 0, sizeof(sd_card_state));
    SDCard_SetStatus(SD_CARD_STATUS_STARTING, FR_OK);

    if (!RingBuffer_Init(&sd_ring_buffer, sd_ring_pool, sizeof(sd_ring_pool)))
    {
        SDCard_SetStatus(SD_CARD_STATUS_NOT_STARTED, FR_INVALID_PARAMETER);
        return false;
    }

    const SDCard_LogTransferContext_t transfer_context = {
        .live_log_file  = &sd_log_file,
        .state          = &sd_card_state,
        .sync_live_file = SDCard_Sync,
        .erase_logs     = SDCard_EraseLogs,
    };
    if (!SDCard_LogTransfer_Init(&transfer_context))
    {
        SDCard_SetStatus(SD_CARD_STATUS_NOT_STARTED, FR_NOT_ENOUGH_CORE);
        return false;
    }

    sd_card_state.initialized = 1U;
    sd_task_handle            = xTaskCreateStatic(SDCard_Task, "sd_card", SD_CARD_TASK_STACK_WORDS, NULL,
                                                  APP_TASK_PRIORITY_SD_CARD, sd_task_stack, &sd_task_control);

    if (sd_task_handle == NULL)
    {
        sd_card_state.initialized = 0U;
        SDCard_SetStatus(SD_CARD_STATUS_NOT_STARTED, FR_NOT_ENOUGH_CORE);
        return false;
    }

    sd_module_initialized = 1U;
    RTTINFO("[SD] Logger task created.");
    return true;
}

/* ========================== 数据记录接口 ========================== */

/**
 * @brief 在调用者上下文中构造完整日志帧并一次性写入环形缓冲区
 *
 * 帧在任务栈中完成帧头、payload 和 CRC 拼接后再整体发布，消费者不会看到
 * 尚未构造完成的半帧。空间不足时立即丢弃新帧，不阻塞 DataRouter。
 */
bool SDCard_EnqueueFrame(uint8_t message_id, uint8_t message_version, uint16_t flags, uint64_t timestamp_us,
                         const void *payload, uint16_t payload_length)
{
    uint8_t frame[sizeof(SDCard_FrameHeader_t) + SD_CARD_MAX_PAYLOAD_SIZE + sizeof(uint16_t)];

    if ((sd_card_state.initialized == 0U) ||
        ((sd_card_state.status != SD_CARD_STATUS_STARTING) && (sd_card_state.status != SD_CARD_STATUS_RUNNING)))
    {
        return false;
    }

    if ((message_id == 0U) || ((payload == NULL) && (payload_length > 0U)) ||
        (payload_length > SD_CARD_MAX_PAYLOAD_SIZE))
    {
        sd_card_state.invalid_frame_count++;
        return false;
    }

    const SDCard_FrameHeader_t header = {
        .magic           = SD_CARD_FRAME_MAGIC,
        .message_id      = message_id,
        .message_version = message_version,
        .payload_length  = payload_length,
        .flags           = flags,
        .sequence        = sd_card_state.next_sequence++,
        .timestamp_us    = timestamp_us,
    };

    memcpy(frame, &header, sizeof(header));
    if (payload_length > 0U)
    {
        memcpy(frame + sizeof(header), payload, payload_length);
    }

    const uint32_t crc_offset = sizeof(header) + payload_length;
    const uint16_t crc        = Math_Crc16Ccitt(frame, crc_offset, SD_CARD_CRC_INITIAL);
    frame[crc_offset]         = (uint8_t)(crc & 0xFFU);
    frame[crc_offset + 1U]    = (uint8_t)(crc >> 8U);

    const uint32_t frame_length = crc_offset + sizeof(crc);
    if (!RingBuffer_Push(&sd_ring_buffer, frame, frame_length))
    {
        sd_card_state.dropped_frames++;
        sd_card_state.dropped_bytes += frame_length;
        SDCard_UpdateRingUsage();
        return false;
    }

    sd_card_state.enqueued_frames++;
    SDCard_UpdateRingUsage();
    return true;
}

bool SDCard_EnqueueBMI088Imu(uint64_t timestamp_us, const SDCard_BMI088ImuPayloadV1_t *sample)
{
    if (sample == NULL)
    {
        sd_card_state.invalid_frame_count++;
        return false;
    }

    return SDCard_EnqueueFrame(SD_CARD_MSG_BMI088_IMU, 1U, sample->validity_flags, timestamp_us, sample,
                               sizeof(*sample));
}

/* ========================== FreeRTOS 写卡任务 ========================== */

/**
 * @brief 独占日志文件并执行“挂载自检、批量写入、周期同步”循环
 *
 * 任务使用统一优先级表中的 APP_TASK_PRIORITY_SD_CARD，低于控制、传感器和通信任务。
 * SD 卡变慢或失败时只影响旁路记录系统，不阻塞传感器、
 * 姿态估计、控制和通信任务。
 */
void SDCard_Task(void *argument)
{
    (void)argument;

    if (!SDCard_OpenLogFile())
    {
        RTTERROR("[SD] Mount, file creation or readback test failed.");
        if (sd_card_state.file_open != 0U)
        {
            (void)f_close(&sd_log_file);
            sd_card_state.file_open = 0U;
        }
        RTTWARNING("[SD] Logger is waiting for a disarmed LOG_ERASE request.");
    }
    else
    {
        SDCard_SetStatus(SD_CARD_STATUS_RUNNING, FR_OK);
        RTTINFO("[SD] Binary blackbox is running.");
    }

    TickType_t last_sync_tick = xTaskGetTickCount();

    for (;;)
    {
        SDCard_LogTransfer_ProcessRequests();

        if (sd_card_state.status != SD_CARD_STATUS_RUNNING)
        {
            SDCard_LogTransfer_Process();
            vTaskDelay(pdMS_TO_TICKS(SD_CARD_IDLE_DELAY_MS));
            continue;
        }

        const uint32_t read_length = RingBuffer_Pop(&sd_ring_buffer, sd_write_buffer, sizeof(sd_write_buffer));
        SDCard_UpdateRingUsage();

        if ((read_length > 0U) && !SDCard_WriteExact(sd_write_buffer, read_length))
        {
            RTTERROR("[SD] Data write failed; logger stopped to protect real-time tasks.");
            (void)f_close(&sd_log_file);
            sd_card_state.file_open = 0U;
            continue;
        }

        const TickType_t now_tick = xTaskGetTickCount();
        if ((now_tick - last_sync_tick) >= pdMS_TO_TICKS(SD_CARD_SYNC_PERIOD_MS))
        {
            if (!SDCard_Sync())
            {
                RTTERROR("[SD] File sync failed; logger stopped.");
                (void)f_close(&sd_log_file);
                sd_card_state.file_open = 0U;
                continue;
            }
            last_sync_tick = now_tick;
        }

        SDCard_LogTransfer_Process();

        if (read_length == 0U)
        {
            vTaskDelay(pdMS_TO_TICKS(SD_CARD_IDLE_DELAY_MS));
        }
    }
}
