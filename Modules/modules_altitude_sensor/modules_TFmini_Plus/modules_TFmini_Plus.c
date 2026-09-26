/** @file modules_TFmini_Plus.c @brief TFmini Plus UART 帧解析与一致性快照实现。 */

#include "modules_TFmini_Plus.h"
#include <string.h>
#include "bsp_RTT.h"
#include "bsp_timestamp.h"

/* ========================== 全局静态实例 ========================== */
static TFmini_Instance_t tfmini_instance;
static TaskHandle_t      tfmini_ready_task = NULL;

/* ========================== 私有函数声明 ========================== */
static uint8_t TFmini_CalculateChecksum(const uint8_t *buf, uint16_t data_len);
static void    TFmini_ParseRawFrame(const uint8_t *buf, TFminiPlus_Data_t *out);
static void    TFmini_MatchConfigResponse(const uint8_t *data, uint16_t data_len);
static uint8_t TFmini_SendCommandAndWait(uint8_t *command, uint8_t command_len);
static uint8_t TFmini_SetOutputEnabled(uint8_t enabled);
static uint8_t TFmini_SetOutputRate(uint16_t rate_hz);
static uint8_t TFmini_ConfigureOutputRate(void);
static void    TFmini_UART_EventCallback(USARTInstance *ins, USART_Event_e event, uint8_t *data_ptr, uint16_t data_len);

/* ========================== 私有函数实现 ========================== */

/**
 * @brief 计算 TFmini 数据帧或配置指令的累加校验和
 *
 * @param buf      待计算的数据首地址
 * @param data_len 不包含末尾校验字节的数据长度
 * @return 所有输入字节累加结果的低 8 位
 */
static uint8_t TFmini_CalculateChecksum(const uint8_t *buf, uint16_t data_len)
{
    uint8_t sum = 0U;

    for (uint16_t i = 0U; i < data_len; i++)
    {
        sum = (uint8_t)(sum + buf[i]);
    }

    return sum;
}

/**
 * @brief 解析 TFmini 原始 9 字节帧
 *
 * @note buf 已经通过帧头和校验检查
 */
static void TFmini_ParseRawFrame(const uint8_t *buf, TFminiPlus_Data_t *out)
{
    uint16_t raw_temp;

    out->distance = (uint16_t)((uint16_t)buf[TFMINI_IDX_DISTANCE_L] | ((uint16_t)buf[TFMINI_IDX_DISTANCE_H] << 8));

    out->strength = (uint16_t)((uint16_t)buf[TFMINI_IDX_STRENGTH_L] | ((uint16_t)buf[TFMINI_IDX_STRENGTH_H] << 8));

    raw_temp = (uint16_t)((uint16_t)buf[TFMINI_IDX_TEMP_L] | ((uint16_t)buf[TFMINI_IDX_TEMP_H] << 8));

    out->temperature = ((float)raw_temp / 8.0f) - 256.0f;

    /*
     * 有效性判定沿用你原来的逻辑：
     * strength >= 100 且 != 0xFFFF 认为有效
     *
     * 如需更严格，也可以附加 distance > 0 判定
     */
    if ((out->strength >= 100U) && (out->strength != 0xFFFFU))
    {
        out->is_valid = 1U;
    }
    else
    {
        out->is_valid = 0U;
    }
}

/**
 * @brief 在 UART 字节流中匹配当前等待的配置应答
 *
 * @note 配置阶段可能仍夹杂最后几帧 0x59 测距数据，因此不能只按本次
 *       DMA 回调长度判断。这里使用轻量级逐字节匹配，允许应答跨回调到达。
 */
static void TFmini_MatchConfigResponse(const uint8_t *data, uint16_t data_len)
{
    uint8_t match_index = tfmini_instance.config_match_index;

    for (uint16_t i = 0U; i < data_len; i++)
    {
        if (data[i] == tfmini_instance.config_expected_response[match_index])
        {
            match_index++;
            if (match_index >= tfmini_instance.config_expected_length)
            {
                tfmini_instance.config_match_index       = 0U;
                tfmini_instance.config_response_received = 1U;
                return;
            }
        }
        else
        {
            match_index = (data[i] == tfmini_instance.config_expected_response[0]) ? 1U : 0U;
        }
    }

    tfmini_instance.config_match_index = match_index;
}

/**
 * @brief 发送配置指令并等待雷达返回完全一致的确认帧
 *
 * @note 使用阻塞发送只占用不到 1 ms，等待应答期间主动让出 CPU。
 *       指令最多重试三次，应对上电初期偶发的 UART 帧边界或应答丢失。
 */
static uint8_t TFmini_SendCommandAndWait(uint8_t *command, uint8_t command_len)
{
    TickType_t wait_start;
    TickType_t timeout_ticks = pdMS_TO_TICKS(TFMINI_COMMAND_RESPONSE_TIMEOUT_MS);

    if ((command == NULL) || (command_len == 0U) || (command_len > TFMINI_COMMAND_MAX_SIZE))
    {
        return 0U;
    }

    for (uint8_t retry = 0U; retry < TFMINI_COMMAND_RETRY_COUNT; retry++)
    {
        for (uint8_t i = 0U; i < command_len; i++)
        {
            tfmini_instance.config_expected_response[i] = command[i];
        }

        tfmini_instance.config_expected_length   = command_len;
        tfmini_instance.config_match_index       = 0U;
        tfmini_instance.config_response_received = 0U;
        __DMB();
        tfmini_instance.config_waiting_response = 1U;

        if (USARTIsTransmitReady(tfmini_instance.usart_instance) != 0U)
        {
            USARTSend(tfmini_instance.usart_instance, command, command_len, USART_TRANSFER_BLOCKING);
        }

        wait_start = xTaskGetTickCount();
        while ((xTaskGetTickCount() - wait_start) < timeout_ticks)
        {
            if (tfmini_instance.config_response_received != 0U)
            {
                tfmini_instance.config_waiting_response = 0U;
                return 1U;
            }
            vTaskDelay(1U);
        }

        tfmini_instance.config_waiting_response = 0U;
        vTaskDelay(pdMS_TO_TICKS(5U));
    }

    return 0U;
}

/**
 * @brief 打开或关闭 TFmini 连续测距输出
 */
static uint8_t TFmini_SetOutputEnabled(uint8_t enabled)
{
    uint8_t command[TFMINI_SET_OUTPUT_SWITCH_COMMAND_SIZE] = {
        TFMINI_COMMAND_HEADER,
        TFMINI_SET_OUTPUT_SWITCH_COMMAND_SIZE,
        TFMINI_SET_OUTPUT_SWITCH_COMMAND_ID,
        (enabled != 0U) ? 1U : 0U,
        0U,
    };

    command[TFMINI_SET_OUTPUT_SWITCH_COMMAND_SIZE - 1U] =
        TFmini_CalculateChecksum(command, TFMINI_SET_OUTPUT_SWITCH_COMMAND_SIZE - 1U);
    return TFmini_SendCommandAndWait(command, TFMINI_SET_OUTPUT_SWITCH_COMMAND_SIZE);
}

/**
 * @brief 设置 TFmini 连续测距反馈频率
 *
 * @note 手册要求频率位于 1～1000 Hz，且必须满足 1000/n，n 为正整数。
 */
static uint8_t TFmini_SetOutputRate(uint16_t rate_hz)
{
    uint8_t command[TFMINI_SET_OUTPUT_RATE_COMMAND_SIZE] = {
        TFMINI_COMMAND_HEADER,
        TFMINI_SET_OUTPUT_RATE_COMMAND_SIZE,
        TFMINI_SET_OUTPUT_RATE_COMMAND_ID,
        (uint8_t)(rate_hz & 0xFFU),
        (uint8_t)(rate_hz >> 8),
        0U,
    };

    if ((rate_hz == 0U) || (rate_hz > 1000U) || ((1000U % rate_hz) != 0U))
    {
        return 0U;
    }

    command[sizeof(command) - 1U] = TFmini_CalculateChecksum(command, sizeof(command) - 1U);
    return TFmini_SendCommandAndWait(command, TFMINI_SET_OUTPUT_RATE_COMMAND_SIZE);
}

/**
 * @brief 完成“停止输出—修改频率—恢复输出”的原子配置流程
 *
 * @note 不发送保存设置命令。飞控每次启动都会重新下发目标频率，避免反复写入
 *       雷达内部非易失存储器，同时不受雷达历史配置影响。
 */
static uint8_t TFmini_ConfigureOutputRate(void)
{
    uint8_t stop_ok;
    uint8_t rate_ok;
    uint8_t start_ok;

    stop_ok  = TFmini_SetOutputEnabled(0U);
    rate_ok  = TFmini_SetOutputRate(TFMINI_OUTPUT_RATE_HZ);
    start_ok = TFmini_SetOutputEnabled(1U);

    if ((rate_ok != 0U) && (start_ok != 0U))
    {
        tfmini_instance.configured_rate_hz = TFMINI_OUTPUT_RATE_HZ;
        tfmini_instance.config_success_count++;
        RTTINFO("[TFmini] Output rate configured to %u Hz.", (unsigned)TFMINI_OUTPUT_RATE_HZ);
        return 1U;
    }

    tfmini_instance.config_error_count++;
    RTTWARNING("[TFmini] Rate config incomplete: stop=%u rate=%u start=%u.", (unsigned)stop_ok, (unsigned)rate_ok,
               (unsigned)start_ok);
    return 0U;
}

/* ========================== 串口事件回调（内部私有） ========================== */

static void TFmini_UART_EventCallback(USARTInstance *ins, USART_Event_e event, uint8_t *data_ptr, uint16_t data_len)
{
    (void)ins;

    switch (event)
    {
    case USART_EVENT_RX_CPLT:
        if ((data_ptr == NULL) || (data_len == 0U))
        {
            return;
        }

        /*
         * 初始化配置期间，UART 接收只用于识别 0x5A 配置应答，不把应答或
         * 夹杂的旧测距帧投递给传感器任务，避免制造伪解析错误。
         */
        if (tfmini_instance.config_waiting_response != 0U)
        {
            TFmini_MatchConfigResponse(data_ptr, data_len);
            return;
        }

        /*
         * ISR 中只做一次原始帧拷贝
         * 不做协议解析、不做字段处理、不做发布
         */
        SeqLock_WriteBegin(&tfmini_instance.data_lock);

        tfmini_instance.raw_len = (data_len > TFMINI_FRAME_SIZE) ? TFMINI_FRAME_SIZE : data_len;
        memcpy(tfmini_instance.raw_rx_buf, data_ptr, tfmini_instance.raw_len);
        tfmini_instance.capture_timestamp = Bsp_Timestamp_us_Get();
        tfmini_instance.rx_frame_count++;

        SeqLock_WriteEnd(&tfmini_instance.data_lock);
        if (tfmini_ready_task != NULL)
        {
            BaseType_t higher_priority_task_woken = pdFALSE;
            xTaskNotifyFromISR(tfmini_ready_task, NOTIFY_BIT_TFMINI, eSetBits, &higher_priority_task_woken);
            portYIELD_FROM_ISR(higher_priority_task_woken);
        }
        break;

    case USART_EVENT_ERROR:
        tfmini_instance.rx_err_count++;
        break;

    case USART_EVENT_TX_CPLT:
    default:
        break;
    }
}

/* ========================== 任务级处理 ========================== */

uint8_t TFmini_Task_Handler(void)
{
    uint32_t          start_seq;
    uint32_t          frame_count;
    uint16_t          raw_len;
    uint64_t          timestamp_us;
    uint8_t           local_buf[TFMINI_FRAME_SIZE];
    uint8_t           checksum;
    uint8_t           retry;
    TFminiPlus_Data_t data;

    if (tfmini_instance.publisher == NULL)
    {
        return 0U;
    }

    for (retry = 0U; retry < TFMINI_SEQLOCK_MAX_RETRY; retry++)
    {
        /* ========== 阶段 1：在 SeqLock 保护下获取稳定快照 ========== */

        start_seq = SeqLock_ReadBegin(&tfmini_instance.data_lock);

        frame_count  = tfmini_instance.rx_frame_count;
        raw_len      = tfmini_instance.raw_len;
        timestamp_us = tfmini_instance.capture_timestamp;

        if (raw_len > TFMINI_FRAME_SIZE)
        {
            raw_len = TFMINI_FRAME_SIZE;
        }

        if (raw_len > 0U)
        {
            memcpy(local_buf, tfmini_instance.raw_rx_buf, raw_len);
        }

        if (SeqLock_ReadRetry(&tfmini_instance.data_lock, start_seq))
        {
            continue;
        }

        /* ========== 阶段 2：基于本地快照处理 ========== */

        /* 无新帧 */
        if ((frame_count == 0U) || (frame_count == tfmini_instance.last_seen_frame_count))
        {
            return 0U;
        }

        /*
         * 注意：
         * 这里先记录“这份原始帧已经看过了”
         * 这样即使它是坏帧，也不会在任务高频调用时被反复统计多次
         */
        tfmini_instance.last_seen_frame_count = frame_count;

        /* 帧长度检查 */
        if (raw_len != TFMINI_FRAME_SIZE)
        {
            tfmini_instance.parse_err_count++;
            return 0U;
        }

        /* 帧头检查 */
        if ((local_buf[TFMINI_IDX_HEADER1] != TFMINI_HEADER) || (local_buf[TFMINI_IDX_HEADER2] != TFMINI_HEADER))
        {
            tfmini_instance.parse_err_count++;
            return 0U;
        }

        /* 校验和检查 */
        checksum = TFmini_CalculateChecksum(local_buf, TFMINI_FRAME_SIZE - 1U);
        if (checksum != local_buf[TFMINI_IDX_CHECKSUM])
        {
            tfmini_instance.parse_err_count++;
            return 0U;
        }

        /* ========== 阶段 3：解析 + 发布 ========== */

        memset(&data, 0, sizeof(TFminiPlus_Data_t));

        TFmini_ParseRawFrame(local_buf, &data);
        data.timestamp_us = timestamp_us;

        tfmini_instance.last_pub_frame_count = frame_count;
        PubPushMessage(tfmini_instance.publisher, &data);

        return 1U;
    }

    return 0U;
}

void Tfmini_RegisterReadyTask(TaskHandle_t task_handle)
{
    tfmini_ready_task = task_handle;
}

/* ========================== 初始化 ========================== */

uint8_t TFmini_Init(void)
{
    USART_Init_Config_s uart_config;

    memset(&tfmini_instance, 0, sizeof(TFmini_Instance_t));
    SeqLock_Init(&tfmini_instance.data_lock);

    /* 注册消息中心发布者 */
    tfmini_instance.publisher = PubRegister(TFMINI_TOPIC_NAME, sizeof(TFminiPlus_Data_t));
    if (tfmini_instance.publisher == NULL)
    {
        RTTERROR("[TFmini] PubRegister Failed!");
        return 0U;
    }

    /* 注册串口：固定 9 字节帧，DMA Normal 模式 */
    memset(&uart_config, 0, sizeof(uart_config));
    uart_config.usart_handle   = &TFMINI_UART_HANDLE;
    uart_config.recv_buff_size = TFMINI_FRAME_SIZE;
    uart_config.event_callback = TFmini_UART_EventCallback;
    uart_config.rx_mode        = USART_RX_MODE_NORMAL;

    tfmini_instance.usart_instance = USARTRegister(&uart_config);
    if (tfmini_instance.usart_instance == NULL)
    {
        RTTERROR("[TFmini] UART Register Failed!");
        return 0U;
    }

    /*
     * 配置失败不直接注销设备：串口和测距解析仍然可用，运行统计中的
     * config_error_count 会保留故障证据。这样飞控不会因为一次应答丢失
     * 完全失去激光高度数据，后续还可通过实测帧率判断雷达是否已经生效。
     */
    (void)TFmini_ConfigureOutputRate();

    RTTINFO("[TFmini] TFmini Plus Init Success!");
    return 1U;
}
