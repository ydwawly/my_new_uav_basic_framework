//
// Created by Administrator on 2026/6/15.
//

#include "bsp_uart.h"
#include "FreeRTOS.h"
#include "bsp_RTT.h"
#include <string.h>
#include "bsp_memory_section.h"
#include "task.h"
/* ========================== 私有变量 ========================== */

static uint8_t usart_count = 0U;
static USARTInstance *usart_instance[DEVICE_USART_CNT] = {NULL};

/* ========================== 私有函数声明 ========================== */

static void USARTNormalRxHandler(USARTInstance *inst, uint16_t size);
static void USARTCircularRxHandler(USARTInstance *inst, uint16_t dma_pos);
static uint8_t USARTIsDMACircular(USARTInstance *inst);
static void USARTClearErrors(USARTInstance *inst);

/* ========================== 公有接口实现 ========================== */

/**
 * @brief 清除 UART 错误标志并冲刷 RDR，防止首帧乱码
 */
static void USARTClearErrors(USARTInstance *inst)
{
    volatile uint32_t dummy;

    /* 清除 ORE / NE / FE / PE 标志 */
    __HAL_UART_CLEAR_FLAG(inst->usart_handle,
                          UART_CLEAR_OREF |
                          UART_CLEAR_NEF  |
                          UART_CLEAR_FEF  |
                          UART_CLEAR_PEF);

    /* 读一次 RDR 冲刷残留字节 */
    dummy = inst->usart_handle->Instance->RDR;
    (void)dummy;
}

/**
 * @brief 检查 DMA 是否配置为 Circular 模式
 */
static uint8_t USARTIsDMACircular(USARTInstance *inst)
{
    if (inst->usart_handle->hdmarx == NULL)
    {
        return 0U;
    }
    return (inst->usart_handle->hdmarx->Init.Mode == DMA_CIRCULAR) ? 1U : 0U;
}

void USARTServiceInit(USARTInstance *instance)
{
    HAL_StatusTypeDef ret;

    if (instance == NULL || instance->usart_handle == NULL)
    {
        return;
    }

    USARTClearErrors(instance);

    if (instance->rx_mode == USART_RX_MODE_CIRCULAR)
    {
        /*
         * Circular 模式：DMA 持续循环写入 recv_buff，
         * 通过 HT / TC / IDLE 三类事件触发位置差分处理。
         */
        instance->last_dma_pos = 0U;

        ret = HAL_UARTEx_ReceiveToIdle_DMA(instance->usart_handle,
                                           instance->recv_buff,
                                           instance->recv_buff_size);
        if (ret != HAL_OK)
        {
            RTTERROR("[bsp_usart] CIRCULAR DMA start failed, HAL ret=%d", (int)ret);
            return;
        }
    }
    else
    {
        /*
         * Normal 模式：等待一帧（IDLE 触发），回调后重新启动。
         */
        ret = HAL_UARTEx_ReceiveToIdle_DMA(instance->usart_handle,
                                           instance->recv_buff,
                                           instance->recv_buff_size);
        if (ret != HAL_OK)
        {
            RTTERROR("[bsp_usart] NORMAL DMA start failed, HAL ret=%d", (int)ret);
            return;
        }
        __HAL_DMA_DISABLE_IT(instance->usart_handle->hdmarx, DMA_IT_HT);
    }
}

USARTInstance *USARTRegister(USART_Init_Config_s *init_config)
{
    USARTInstance *instance;
    uint8_t        i;

    /* ---------- 参数合法性检查 ---------- */
    if (init_config == NULL || init_config->usart_handle == NULL)
    {
        RTTERROR("[bsp_usart] Register failed: NULL config or handle.");
        return NULL;
    }

    if (init_config->recv_buff_size == 0U ||
        init_config->recv_buff_size > USART_RXBUFF_LIMIT)
    {
        RTTERROR("[bsp_usart] Register failed: recv_buff_size=%u out of range.",
                 (unsigned)init_config->recv_buff_size);
        return NULL;
    }

    if (usart_count >= DEVICE_USART_CNT)
    {
        RTTERROR("[bsp_usart] Register failed: max instance count reached.");
        configASSERT(0);
        return NULL;
    }

    for (i = 0U; i < usart_count; i++) {
        if (usart_instance[i]->usart_handle == init_config->usart_handle)
        {
            RTTERROR("[bsp_usart] Register failed: handle already registered.");
            configASSERT(0);
            return NULL;
        }
    }

    if (init_config->rx_mode == USART_RX_MODE_CIRCULAR)
    {
        if (init_config->usart_handle->hdmarx == NULL ||
            init_config->usart_handle->hdmarx->Init.Mode != DMA_CIRCULAR)
        {
            RTTERROR("[bsp_usart] Register failed: CIRCULAR mode requires DMA Circular.");
            configASSERT(0);
            return NULL;
        }
    }

    /* ---------- 分配实例内存 ---------- */
    instance = (USARTInstance *)pvPortMalloc(sizeof(USARTInstance));
    if (instance == NULL)
    {
        configASSERT(0);
        return NULL;
    }

    /* ---------- 填写基本字段 ---------- */
    instance->usart_handle   = init_config->usart_handle;
    instance->recv_buff_size = init_config->recv_buff_size;
    instance->event_callback = init_config->event_callback; /* 新增：绑定统一事件回调 */
    instance->rx_mode        = init_config->rx_mode;
    instance->last_dma_pos   = 0U;

    /* ---------- 分配 DMA 接收缓冲区 ---------- */
    instance->recv_buff = (uint8_t *)BSP_DMA_Malloc(instance->recv_buff_size);
    if (instance->recv_buff == NULL)
    {
        configASSERT(0);
        return NULL;
    }
    memset(instance->recv_buff, 0, instance->recv_buff_size);

    /* ---------- 分配中间处理缓冲区 (仅 CIRCULAR) ---------- */
    if (instance->rx_mode == USART_RX_MODE_CIRCULAR)
    {
        instance->process_buff = (uint8_t *)pvPortMalloc(instance->recv_buff_size);
        if (instance->process_buff == NULL)
        {
            configASSERT(0);
            return NULL;
        }
    }

    /* ---------- 注册并启动 ---------- */
    usart_instance[usart_count++] = instance;
    USARTServiceInit(instance);

    return instance;
}

void USARTSend(USARTInstance    *instance,
               uint8_t          *send_buf,
               uint16_t          send_size,
               USART_TRANSFER_MODE mode)
{
    HAL_StatusTypeDef ret = HAL_OK;

    if (instance == NULL || instance->usart_handle == NULL ||
        send_buf == NULL || send_size == 0U)
    {
        return;
    }

    switch (mode)
    {
    case USART_TRANSFER_BLOCKING:
        ret = HAL_UART_Transmit(instance->usart_handle, send_buf, send_size, 100U);
        break;

    case USART_TRANSFER_IT:
        ret = HAL_UART_Transmit_IT(instance->usart_handle, send_buf, send_size);
        break;

    case USART_TRANSFER_DMA:
        ret = HAL_UART_Transmit_DMA(instance->usart_handle, send_buf, send_size);
        break;

    default:
        return;
    }

    if (ret == HAL_BUSY)
    {
        RTTWARNING("[bsp_usart] USARTSend: transmitter busy.");
    }
}

uint8_t USARTIsTransmitReady(USARTInstance *instance)
{
    if (instance == NULL || instance->usart_handle == NULL)
    {
        return 0U;
    }

    return ((instance->usart_handle->gState == HAL_UART_STATE_READY) ||
            (instance->usart_handle->gState == HAL_UART_STATE_BUSY_RX)) ? 1U : 0U;
}

/* ========================== 私有处理函数 ========================== */

/**
 * @brief  Normal 模式接收处理
 */
static void USARTNormalRxHandler(USARTInstance *inst, uint16_t size)
{
    if (size == 0U)
    {
        USARTServiceInit(inst);
        return;
    }

    /* 触发统一事件：接收完成 */
    if (inst->event_callback != NULL)
    {
        inst->event_callback(inst, USART_EVENT_RX_CPLT, inst->recv_buff, size);
    }

    USARTServiceInit(inst);
}

/**
 * @brief  Circular 模式接收处理（位置差分法）
 */
static void USARTCircularRxHandler(USARTInstance *inst, uint16_t dma_pos)
{
    uint16_t copy_len;
    uint16_t tail_part;
    uint16_t head_part;

    if (dma_pos >= inst->recv_buff_size)
    {
        dma_pos = 0U;
    }

    if (dma_pos == inst->last_dma_pos)
    {
        return;
    }

    if (dma_pos > inst->last_dma_pos)
    {
        copy_len = (uint16_t)(dma_pos - inst->last_dma_pos);
        memcpy(inst->process_buff,
               &inst->recv_buff[inst->last_dma_pos],
               copy_len);
    }
    else
    {
        tail_part = (uint16_t)(inst->recv_buff_size - inst->last_dma_pos);
        head_part = dma_pos;
        copy_len  = (uint16_t)(tail_part + head_part);

        memcpy(inst->process_buff,
               &inst->recv_buff[inst->last_dma_pos],
               tail_part);
        memcpy(&inst->process_buff[tail_part],
               &inst->recv_buff[0],
               head_part);
    }

    inst->last_dma_pos = dma_pos;

    /* 触发统一事件：接收完成 */
    if (inst->event_callback != NULL)
    {
        inst->event_callback(inst, USART_EVENT_RX_CPLT, inst->process_buff, copy_len);
    }
}

/* ========================== HAL 回调重写 ========================== */

/**
 * @brief HAL UART 接收事件回调 (处理 IDLE / HT / TC)
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    uint8_t       i;
    USARTInstance *inst;

    for (i = 0U; i < usart_count; i++)
    {
        inst = usart_instance[i];

        if (huart != inst->usart_handle)
        {
            continue;
        }

        if (inst->rx_mode == USART_RX_MODE_NORMAL)
        {
            USARTNormalRxHandler(inst, size);
        } else {
            USARTCircularRxHandler(inst, size);
        }
        return;
    }
}

/**
 * @brief HAL UART 发送完成回调 (新增：处理 TX_DMA 或 IT 传输完成)
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t i;

    for (i = 0U; i < usart_count; i++)
    {
        if (huart == usart_instance[i]->usart_handle)
        {
            /* 触发统一事件：发送完成 (不需要传递数据指针和长度，设为 NULL/0) */
            if (usart_instance[i]->event_callback != NULL)
            {
                usart_instance[i]->event_callback(usart_instance[i], USART_EVENT_TX_CPLT, NULL, 0);
            }
            return;
        }
    }
}

/**
 * @brief HAL UART 错误回调
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint8_t       i;
    USARTInstance *inst;

    for (i = 0U; i < usart_count; i++)
    {
        inst = usart_instance[i];

        if (huart != inst->usart_handle)
        {
            continue;
        }

        RTTWARNING("[bsp_usart] ErrorCallback: error=0x%08X", (unsigned)huart->ErrorCode);

        /* 触发统一事件：传输出错 (供上层记录日志或特殊处理) */
        if (inst->event_callback != NULL)
        {
            inst->event_callback(inst, USART_EVENT_ERROR, NULL, 0);
        }

        /* 硬件层面的错误恢复：中止并重启接收 */
        HAL_UART_AbortReceive(huart);
        USARTServiceInit(inst);
        return;
    }
}