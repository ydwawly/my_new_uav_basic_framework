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

static uint8_t        usart_count                    = 0U;
static USARTInstance *usart_instance[DEVICE_USART_CNT] = {NULL};


/* ========================== 私有函数声明 ========================== */

static void     USARTNormalRxHandler    (USARTInstance *inst, uint16_t size);
static void     USARTCircularRxHandler  (USARTInstance *inst, uint16_t dma_pos);
static uint8_t  USARTIsDMACircular      (USARTInstance *inst);
static void     USARTClearErrors        (USARTInstance *inst);

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
 *
 * @return 1 是 Circular，0 不是
 */
static uint8_t USARTIsDMACircular(USARTInstance *inst)
{
    if (inst->usart_handle->hdmarx == NULL) {
        return 0U;
    }
    return (inst->usart_handle->hdmarx->Init.Mode == DMA_CIRCULAR) ? 1U : 0U;
}

void USARTServiceInit(USARTInstance *instance)
{
    HAL_StatusTypeDef ret;

    if (instance == NULL || instance->usart_handle == NULL) {
        return;
    }

    USARTClearErrors(instance);

    if (instance->rx_mode == USART_RX_MODE_CIRCULAR)
    {
        /*
         * Circular 模式：DMA 持续循环写入 recv_buff，
         * 通过 HT / TC / IDLE 三类事件触发位置差分处理。
         * last_dma_pos 重置为 0，与 DMA 起始位置同步。
         */
        instance->last_dma_pos = 0U;

        ret = HAL_UARTEx_ReceiveToIdle_DMA(instance->usart_handle,
                                           instance->recv_buff,
                                           instance->recv_buff_size);
        if (ret != HAL_OK) {
            RTTERROR("[bsp_usart] CIRCULAR DMA start failed, HAL ret=%d", (int)ret);
            return;
        }
    }
    else
    {
        /*
         * Normal 模式：等待一帧（IDLE 触发），回调后重新启动。
         * 关闭 HT 中断，只依赖 IDLE 和 TC。
         */
        ret = HAL_UARTEx_ReceiveToIdle_DMA(instance->usart_handle,
                                           instance->recv_buff,
                                           instance->recv_buff_size);
        if (ret != HAL_OK) {
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
    if (init_config == NULL || init_config->usart_handle == NULL) {
        RTTERROR("[bsp_usart] Register failed: NULL config or handle.");
        return NULL;
    }

    if (init_config->recv_buff_size == 0U ||
        init_config->recv_buff_size > USART_RXBUFF_LIMIT)
    {
        RTTERROR("[bsp_usart] Register failed: recv_buff_size=%u out of range (0, %u].",
                 (unsigned)init_config->recv_buff_size,
                 (unsigned)USART_RXBUFF_LIMIT);
        return NULL;
    }

    /* ---------- 实例数量限制检查 ---------- */
    if (usart_count >= DEVICE_USART_CNT) {
        RTTERROR("[bsp_usart] Register failed: reached max instance count %u.", DEVICE_USART_CNT);
        configASSERT(0);
        return NULL;
    }

    /* ---------- 重复注册检查 ---------- */
    for (i = 0U; i < usart_count; i++) {
        if (usart_instance[i]->usart_handle == init_config->usart_handle) {
            RTTERROR("[bsp_usart] Register failed: handle already registered at index %u.", i);
            configASSERT(0);
            return NULL;
        }
    }

    /* ---------- CIRCULAR 模式必须配置 DMA Circular ---------- */
    if (init_config->rx_mode == USART_RX_MODE_CIRCULAR)
    {
        if (init_config->usart_handle->hdmarx == NULL ||
            init_config->usart_handle->hdmarx->Init.Mode != DMA_CIRCULAR)
        {
            RTTERROR("[bsp_usart] Register failed: CIRCULAR mode requires DMA Circular, "
                     "please check CubeMX configuration.");
            configASSERT(0);
            return NULL;
        }
    }

    /* ---------- 分配实例内存 ---------- */
    instance = (USARTInstance *)pvPortMalloc(sizeof(USARTInstance));
    if (instance == NULL) {
        RTTERROR("[bsp_usart] Register failed: instance allocation failed.");
        configASSERT(0);
        return NULL;
    }

    /* ---------- 填写基本字段 ---------- */
    instance->usart_handle   = init_config->usart_handle;
    instance->recv_buff_size = init_config->recv_buff_size;
    instance->module_callback = init_config->module_callback;
    instance->rx_mode        = init_config->rx_mode;
    instance->last_dma_pos   = 0U;

    /* ---------- 分配 DMA 接收缓冲区 ---------- */
    /*
     * recv_buff 须位于 DMA 可访问区域。
     * H7 上如果开启了 D-Cache，须确保 BSP_DMA_Malloc 返回
     * non-cacheable 区域（如 SRAM4 / 配置了 Non-Cacheable 的 MPU 区域），
     * 否则需在读取前手动调用 SCB_InvalidateDCache_by_Addr()。
     */
    instance->recv_buff = (uint8_t *)BSP_DMA_Malloc(instance->recv_buff_size);
    if (instance->recv_buff == NULL) {
        RTTERROR("[bsp_usart] Register failed: recv_buff DMA allocation failed.");
        configASSERT(0);
        return NULL;
    }
    memset(instance->recv_buff, 0, instance->recv_buff_size);

    /* ---------- CIRCULAR 模式分配中间处理缓冲区 ---------- */
    /*
     * process_buff 由 CPU 通过 memcpy 写入，不需要 DMA 访问，
     * 使用普通内存即可。
     */
    if (instance->rx_mode == USART_RX_MODE_CIRCULAR)
    {
        instance->process_buff = (uint8_t *)pvPortMalloc(instance->recv_buff_size);
        if (instance->process_buff == NULL) {
            RTTERROR("[bsp_usart] Register failed: process_buff allocation failed.");
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

    /* ---------- 参数检查 ---------- */
    if (instance == NULL || instance->usart_handle == NULL ||
        send_buf == NULL || send_size == 0U)
    {
        RTTWARNING("[bsp_usart] USARTSend: invalid parameter.");
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
        RTTWARNING("[bsp_usart] USARTSend: unknown transfer mode %d.", (int)mode);
        return;
    }

    if (ret == HAL_BUSY) {
        RTTWARNING("[bsp_usart] USARTSend: transmitter busy, data dropped.");
    } else if (ret != HAL_OK) {
        RTTWARNING("[bsp_usart] USARTSend: HAL transmit failed, ret=%d.", (int)ret);
    }
}

uint8_t USARTIsTransmitReady(USARTInstance *instance)
{
    if (instance == NULL || instance->usart_handle == NULL) {
        return 0U;
    }

    /*
     * gState 反映发送链路状态：
     * HAL_UART_STATE_READY      -> TX 空闲
     * HAL_UART_STATE_BUSY_RX    -> 仅 RX 忙，TX 仍可用
     * HAL_UART_STATE_BUSY_TX    -> TX 忙
     * HAL_UART_STATE_BUSY_TX_RX -> TX RX 均忙
     *
     * 注意：HAL_UART_GetState() 返回的是 gState | RxState，
     * 这里只关心 TX 链路，所以直接看 gState。
     */
    return ((instance->usart_handle->gState == HAL_UART_STATE_READY) ||
            (instance->usart_handle->gState == HAL_UART_STATE_BUSY_RX)) ? 1U : 0U;
}

/* ========================== 私有处理函数 ========================== */

/**
 * @brief  Normal 模式接收处理
 *
 * @param  inst  串口实例
 * @param  size  本次接收到的有效字节数（HAL 回调提供）
 *
 * @note   调用 callback 后立即重启 DMA，最小化帧间空窗期
 */
static void USARTNormalRxHandler(USARTInstance *inst, uint16_t size)
{
    if (size == 0U) {
        /* 空帧，不触发回调，直接重启 */
        USARTServiceInit(inst);
        return;
    }

    if (inst->module_callback != NULL) {
        inst->module_callback(inst->recv_buff, size);
    }

    /*
     * 重启放在 callback 之后，
     * 因为 Normal 模式下 callback 里通常会立即拷贝数据，
     * 拷贝完成后再让 DMA 覆盖 recv_buff 是安全的。
     */
    USARTServiceInit(inst);
}

/**
 * @brief  Circular 模式接收处理（位置差分法）
 *
 * @param  inst     串口实例
 * @param  dma_pos  当前 DMA 已写入位置（由 HAL 回调提供的 size 转换而来）
 * NDTR初始化为 recv_buff_size = NDTR
 * 收到count字节：NDTR = NDTR - count;
 * dma_pos = recv_buff_size - NDTR;
 *
 * @note   dma_pos 范围：[0, recv_buff_size)
 *         当 dma_pos == last_dma_pos 时，表示没有新数据，不触发 callback。
 *         数据已拷贝至 process_buff，callback 持有的是 process_buff 指针，
 *         DMA 继续写 recv_buff 不会影响 callback 读取的数据。
 */
static void USARTCircularRxHandler(USARTInstance *inst, uint16_t dma_pos)
{
    uint16_t copy_len;
    uint16_t tail_part;
    uint16_t head_part;

    /* dma_pos 由 HAL 提供的 size 映射而来，size == recv_buff_size 时对应位置 0 */
    if (dma_pos >= inst->recv_buff_size) {
        dma_pos = 0U;
    }

    if (dma_pos == inst->last_dma_pos) {
        /* 没有新数据 */
        return;
    }

    if (dma_pos > inst->last_dma_pos)
    {
        /* 线性区段：last_dma_pos -> dma_pos */
        copy_len = (uint16_t)(dma_pos - inst->last_dma_pos);

        memcpy(inst->process_buff,
               &inst->recv_buff[inst->last_dma_pos],
               copy_len);
    }
    else
    {
        /* 环绕区段：last_dma_pos -> end，0 -> dma_pos */
        tail_part = (uint16_t)(inst->recv_buff_size - inst->last_dma_pos);
        head_part = dma_pos;
        copy_len  = (uint16_t)(tail_part + head_part);

        /*
         * process_buff 大小为 recv_buff_size，
         * tail_part + head_part <= recv_buff_size，拷贝不会越界。
         */
        memcpy(inst->process_buff,
               &inst->recv_buff[inst->last_dma_pos],
               tail_part);
        memcpy(&inst->process_buff[tail_part],
               &inst->recv_buff[0],
               head_part);
    }

    inst->last_dma_pos = dma_pos;

    if (inst->module_callback != NULL) {
        inst->module_callback(inst->process_buff, copy_len);
    }
}

/* ========================== HAL 回调重写 ========================== */

/**
 * @brief HAL UART 接收事件回调（由 HAL 在 ISR 中调用）
 *
 * @param huart  触发事件的 UART 句柄
 * @param size   当前 DMA 已写入字节数（从缓冲区起始位置算起）
 *
 * @note  运行于 ISR，禁止在此函数（及其调用链）中做任何阻塞操作
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    uint8_t       i;
    USARTInstance *inst;

    for (i = 0U; i < usart_count; i++)
    {
        inst = usart_instance[i];

        if (huart != inst->usart_handle) {
            continue;
        }

        if (inst->rx_mode == USART_RX_MODE_NORMAL)
        {
            /*
             * Normal 模式：
             * HAL_UART_RXEVENT_IDLE -> 收到一帧，size 为帧长
             * HAL_UART_RXEVENT_TC   -> DMA 搬满缓冲区（超长帧截断），也当一帧处理
             * HAL_UART_RXEVENT_HT   -> 已被禁用，不会触发
             */
            USARTNormalRxHandler(inst, size);
        }
        else /* USART_RX_MODE_CIRCULAR */
        {
            /*
             * Circular 模式：
             * HAL_UART_RXEVENT_HT   -> DMA 写到半程，size = recv_buff_size / 2（约）
             * HAL_UART_RXEVENT_TC   -> DMA 写满一圈，size = recv_buff_size
             * HAL_UART_RXEVENT_IDLE -> 发送方停顿，size 为当前写入位置
             *
             * 三类事件统一走位置差分处理，size 即为 DMA 当前写入位置。
             * 当 size == recv_buff_size 时（TC 事件），DMA 已绕回 0，
             * 在 USARTCircularRxHandler 内将其归一化为 0。
             */
            USARTCircularRxHandler(inst, size);
        }

        return;
    }

    /* 没有找到匹配实例，可能是未注册的串口触发了回调 */
    RTTWARNING("[bsp_usart] RxEventCallback: no matching instance for huart=0x%p.", (void *)huart);
}
/**
 * @brief HAL UART 错误回调（由 HAL 在 ISR 中调用）
 *
 * @param huart  发生错误的 UART 句柄
 *
 * @note  遇到 ORE / FE / NE / PE 等错误时，中止接收并重启 DMA，
 *        尽量减少错误扩散
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint8_t       i;
    USARTInstance *inst;

    for (i = 0U; i < usart_count; i++)
    {
        inst = usart_instance[i];

        if (huart != inst->usart_handle) {
            continue;
        }

        RTTWARNING("[bsp_usart] ErrorCallback: error=0x%08X, restarting RX.",
                   (unsigned)huart->ErrorCode);

        HAL_UART_AbortReceive(huart);
        USARTServiceInit(inst);
        return;
    }

    RTTWARNING("[bsp_usart] ErrorCallback: no matching instance for huart=0x%p.", (void *)huart);
}
