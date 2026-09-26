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

static uint8_t        usart_count                      = 0U;
static USARTInstance *usart_instance[DEVICE_USART_CNT] = {NULL};

/* ========================== 私有函数声明 ========================== */

static void           USARTNormalRxHandler(USARTInstance *inst, uint16_t size);
static void           USARTCircularRxHandler(USARTInstance *inst, uint16_t dma_pos);
static void           USARTClearErrors(USARTInstance *inst);
static uint8_t        USARTDmaModeMatches(USART_RX_MODE rx_mode, const DMA_HandleTypeDef *hdmarx);
static uint8_t        USARTServiceConfigIsValid(const USARTInstance *instance);
static uint8_t        USARTRegisterConfigIsValid(const USART_Init_Config_s *init_config);
static USARTInstance *USARTAllocateInstance(const USART_Init_Config_s *init_config);
static void           USARTReleaseInstance(USARTInstance *instance);

/* ========================== 公有接口实现 ========================== */

/**
 * @brief 清除 UART 错误标志并冲刷 RDR，防止首帧乱码
 */
static void USARTClearErrors(USARTInstance *inst)
{
    /*
     * STM32H7 通过写 ICR 清除 ORE/NE/FE/PE，不需要再读取 RDR。
     * 重启接收时主动读 RDR 反而可能吞掉已经到达的首字节。
     */
    __HAL_UART_CLEAR_FLAG(inst->usart_handle, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
}

/**
 * @brief 检查软件接收模式是否与 CubeMX DMA 模式一致
 */
static uint8_t USARTDmaModeMatches(USART_RX_MODE rx_mode, const DMA_HandleTypeDef *hdmarx)
{
    if ((hdmarx == NULL) || (hdmarx->Instance == NULL))
    {
        return 0U;
    }

    if (rx_mode == USART_RX_MODE_NORMAL)
    {
        return (hdmarx->Init.Mode == DMA_NORMAL) ? 1U : 0U;
    }

    if (rx_mode == USART_RX_MODE_CIRCULAR)
    {
        return (hdmarx->Init.Mode == DMA_CIRCULAR) ? 1U : 0U;
    }

    return 0U;
}

/**
 * @brief 检查已创建串口实例启动 DMA 接收所需的全部资源
 */
static uint8_t USARTServiceConfigIsValid(const USARTInstance *instance)
{
    if ((instance == NULL) || (instance->usart_handle == NULL) || (instance->usart_handle->Instance == NULL))
    {
        RTTERROR("[bsp_usart] ServiceInit failed: invalid instance or UART.");
        return 0U;
    }

    if ((instance->recv_buff == NULL) || (instance->recv_buff_size == 0U))
    {
        RTTERROR("[bsp_usart] ServiceInit failed: invalid RX buffer.");
        return 0U;
    }

    if (!USARTDmaModeMatches(instance->rx_mode, instance->usart_handle->hdmarx))
    {
        RTTERROR("[bsp_usart] ServiceInit failed: software RX mode does not match DMA mode.");
        return 0U;
    }

    return 1U;
}

/**
 * @brief 检查注册参数、实例容量、重复句柄和 DMA 模式
 */
static uint8_t USARTRegisterConfigIsValid(const USART_Init_Config_s *init_config)
{
    if ((init_config == NULL) || (init_config->usart_handle == NULL) || (init_config->usart_handle->Instance == NULL))
    {
        RTTERROR("[bsp_usart] Register failed: invalid config or uninitialized UART.");
        return 0U;
    }

    if ((init_config->recv_buff_size == 0U) || (init_config->recv_buff_size > USART_RXBUFF_LIMIT))
    {
        RTTERROR("[bsp_usart] Register failed: recv_buff_size=%u out of range.", (unsigned)init_config->recv_buff_size);
        return 0U;
    }

    if (usart_count >= DEVICE_USART_CNT)
    {
        RTTERROR("[bsp_usart] Register failed: max instance count reached.");
        configASSERT(0);
        return 0U;
    }

    for (uint8_t i = 0U; i < usart_count; i++)
    {
        if (usart_instance[i]->usart_handle == init_config->usart_handle)
        {
            RTTERROR("[bsp_usart] Register failed: handle already registered.");
            configASSERT(0);
            return 0U;
        }
    }

    if (!USARTDmaModeMatches(init_config->rx_mode, init_config->usart_handle->hdmarx))
    {
        RTTERROR("[bsp_usart] Register failed: software RX mode does not match DMA mode.");
        return 0U;
    }
    return 1U;
}

/**
 * @brief 释放普通堆资源
 *
 * DMA 接收缓冲区来自单向递增内存池，启动阶段分配后不能单独归还。
 */
static void USARTReleaseInstance(USARTInstance *instance)
{
    if (instance == NULL)
    {
        return;
    }

    if (instance->process_buff != NULL)
    {
        vPortFree(instance->process_buff);
    }
    vPortFree(instance);
}

/**
 * @brief 分配实例及其接收缓冲区，并复制只读配置
 */
static USARTInstance *USARTAllocateInstance(const USART_Init_Config_s *init_config)
{
    USARTInstance *instance = (USARTInstance *)pvPortMalloc(sizeof(USARTInstance));
    if (instance == NULL)
    {
        configASSERT(0);
        return NULL;
    }

    memset(instance, 0, sizeof(*instance));
    instance->usart_handle   = init_config->usart_handle;
    instance->recv_buff_size = init_config->recv_buff_size;
    instance->event_callback = init_config->event_callback;
    instance->rx_mode        = init_config->rx_mode;

    instance->recv_buff = (uint8_t *)BSP_DMA_Malloc(instance->recv_buff_size);
    if (instance->recv_buff == NULL)
    {
        USARTReleaseInstance(instance);
        return NULL;
    }
    memset(instance->recv_buff, 0, instance->recv_buff_size);

    if (instance->rx_mode == USART_RX_MODE_CIRCULAR)
    {
        instance->process_buff = (uint8_t *)pvPortMalloc(instance->recv_buff_size);
        if (instance->process_buff == NULL)
        {
            USARTReleaseInstance(instance);
            return NULL;
        }
    }
    return instance;
}

/**
 * @brief 启动或重新启动串口 DMA 接收服务
 */
uint8_t USARTServiceInit(USARTInstance *instance)
{
    if (!USARTServiceConfigIsValid(instance))
    {
        return 0U;
    }

    UART_HandleTypeDef *huart = instance->usart_handle;

    /*
     * ReceiveToIdle 已运行时 RxState 为 BUSY_RX。重复调用只会得到 HAL_BUSY，
     * 因此直接视为接收服务已经正常运行，避免错误日志持续刷屏。
     */
    if (huart->RxState == HAL_UART_STATE_BUSY_RX)
    {
        return 1U;
    }

    USARTClearErrors(instance);

    if (instance->rx_mode == USART_RX_MODE_CIRCULAR)
    {
        /* Circular 模式通过 HT、TC、IDLE 三类事件触发位置差分处理。 */
        instance->last_dma_pos      = 0U;
        const HAL_StatusTypeDef ret = HAL_UARTEx_ReceiveToIdle_DMA(huart, instance->recv_buff,
                                                                   instance->recv_buff_size);
        if (ret != HAL_OK)
        {
            RTTERROR("[bsp_usart] CIRCULAR DMA start failed, HAL ret=%d", (int)ret);
            return 0U;
        }
        return 1U;
    }

    /* Normal 模式仅在 IDLE/TC 后处理一帧，因此关闭半传输中断。 */
    const HAL_StatusTypeDef ret = HAL_UARTEx_ReceiveToIdle_DMA(huart, instance->recv_buff, instance->recv_buff_size);
    if (ret != HAL_OK)
    {
        RTTERROR("[bsp_usart] NORMAL DMA start failed: ret=%d RxState=%d Error=0x%08X", (int)ret, (int)huart->RxState,
                 (unsigned)huart->ErrorCode);
        return 0U;
    }
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    return 1U;
}

USARTInstance *USARTRegister(USART_Init_Config_s *init_config)
{
    if (!USARTRegisterConfigIsValid(init_config))
    {
        return NULL;
    }

    USARTInstance *instance = USARTAllocateInstance(init_config);
    if (instance == NULL)
    {
        return NULL;
    }

    usart_instance[usart_count++] = instance;
    if (USARTServiceInit(instance) == 0U)
    {
        usart_instance[--usart_count] = NULL;
        USARTReleaseInstance(instance);
        return NULL;
    }
    return instance;
}

void USARTSend(USARTInstance *instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE mode)
{
    HAL_StatusTypeDef ret = HAL_OK;

    if (instance == NULL || instance->usart_handle == NULL || send_buf == NULL || send_size == 0U)
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
            (instance->usart_handle->gState == HAL_UART_STATE_BUSY_RX))
               ? 1U
               : 0U;
}

/* ========================== 私有处理函数 ========================== */

/**
 * @brief  Normal 模式接收处理
 */
static void USARTNormalRxHandler(USARTInstance *inst, uint16_t size)
{
    if (size == 0U)
    {
        (void)USARTServiceInit(inst);
        return;
    }

    /* 触发统一事件：接收完成 */
    if (inst->event_callback != NULL)
    {
        inst->event_callback(inst, USART_EVENT_RX_CPLT, inst->recv_buff, size);
    }

    (void)USARTServiceInit(inst);
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
        memcpy(inst->process_buff, &inst->recv_buff[inst->last_dma_pos], copy_len);
    }
    else
    {
        tail_part = (uint16_t)(inst->recv_buff_size - inst->last_dma_pos);
        head_part = dma_pos;
        copy_len  = (uint16_t)(tail_part + head_part);

        memcpy(inst->process_buff, &inst->recv_buff[inst->last_dma_pos], tail_part);
        memcpy(&inst->process_buff[tail_part], &inst->recv_buff[0], head_part);
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
    uint8_t                     i;
    USARTInstance              *inst;
    HAL_UART_RxEventTypeTypeDef event_type;

    event_type = HAL_UARTEx_GetRxEventType(huart);

    for (i = 0U; i < usart_count; i++)
    {
        inst = usart_instance[i];

        if (huart != inst->usart_handle)
        {
            continue;
        }

        if (inst->rx_mode == USART_RX_MODE_NORMAL)
        {
            /*
             * HT 时 DMA 仍处于 BUSY_RX，不能重新启动。
             * Normal 模式只处理 IDLE 和 TC。
             */
            if (event_type == HAL_UART_RXEVENT_HT)
            {
                return;
            }

            USARTNormalRxHandler(inst, size);
        }
        else
        {
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
    uint8_t        i;
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
        (void)USARTServiceInit(inst);
        return;
    }
}
