//
// Created by Administrator on 2026/6/14.
//

#include "bsp_spi.h"
#include "string.h"
#include "FreeRTOS.h"

static SPIInstance   *spi_instance[SPI_DEVICE_CNT];
static uint8_t spi_dev_idx = 0;
/*
 * 总线上下文池
 * 一个 SPI_HandleTypeDef 对应一条物理总线，
 * 同一条总线同时只能有一个 owner（串行化保证）
 * 最多允许和 SPI_DEVICE_CNT 等量的总线（最坏情况每个设备一条独立总线）
 */
static SPIBusContext_t spi_bus_ctx[SPI_DEVICE_CNT];
static uint8_t spi_bus_cnt = 0;

/* ===================== 私有函数声明 ===================== */

static SPIBusContext_t *SPI_GetBusCtx(SPI_HandleTypeDef *hspi);
static SPIBusContext_t *SPI_GetOrCreateBusCtx(SPI_HandleTypeDef *hspi);
static void SPI_ReleaseBus(SPIBusContext_t *ctx);
static void SPI_Complete_Routing(SPI_HandleTypeDef *hspi, SPI_Event_e event);

/* ===================== 私有函数实现 ===================== */
/**
 * @brief 根据 hspi 查找对应的总线上下文，不存在返回 NULL
 */
static SPIBusContext_t *SPI_GetBusCtx(SPI_HandleTypeDef *hspi)
{
    for (uint8_t i = 0; i < spi_bus_cnt; i++)
    {
        if (spi_bus_ctx[i].hspi == hspi)
        {
            return &spi_bus_ctx[i];
        }
    }
    return NULL;
}

/**
 * @brief 根据 hspi 获取总线上下文，如果不存在则创建一个
 *        只在 SPIRegister() 阶段调用，此后只用 SPI_GetBusCtx()
 *
 * @retval 成功返回指针，池满返回 NULL
 */
static SPIBusContext_t *SPI_GetOrCreateBusCtx(SPI_HandleTypeDef *hspi)
{
    /* 先查找已有 */
    SPIBusContext_t *ctx = SPI_GetBusCtx(hspi);
    if (ctx != NULL)
    {
        return ctx;
    }

    /* 没有则新建 */
    if (spi_bus_cnt >= SPI_DEVICE_CNT)
    {
        return NULL; // 总线上下文池已满（理论上不会超过设备数）
    }

    spi_bus_ctx[spi_bus_cnt].hspi          = hspi;
    spi_bus_ctx[spi_bus_cnt].owner         = NULL;
    spi_bus_ctx[spi_bus_cnt].pending_event = SPI_EVENT_TX_CPLT;
    return &spi_bus_ctx[spi_bus_cnt++];
}

/**
 * @brief 释放总线：拉高 CS，清 busy，清 owner
 *
 * @param ctx 总线上下文指针
 */
static void SPI_ReleaseBus(SPIBusContext_t *ctx)
{
    if (ctx == NULL || ctx->owner == NULL)
    {
        return;
    }

    /* 拉高 CS，释放从机 */
    HAL_GPIO_WritePin(ctx->owner->GPIOx, ctx->owner->cs_pin, GPIO_PIN_SET);

    /* 清除忙标志和 owner */
    ctx->owner->is_busy = 0;
    ctx->owner          = NULL;
}

/**
 * @brief 中断/DMA 完成路由核心
 *        根据 hspi 找到 owner，释放总线，触发用户回调
 *
 * @param hspi  触发回调的 SPI 句柄
 * @param event 本次完成的事件类型
 */
static void SPI_Complete_Routing(SPI_HandleTypeDef *hspi, SPI_Event_e event)
{
    SPIBusContext_t *ctx = SPI_GetBusCtx(hspi);
    if (ctx == NULL || ctx->owner == NULL)
    {
        /* 没有找到对应的总线上下文，或者总线当前没有 owner，忽略 */
        return;
    }

    /* 保存 owner 指针，因为 SPI_ReleaseBus 会清空 ctx->owner */
    SPIInstance *owner = ctx->owner;

    /* 释放总线（拉高 CS，清 busy，清 owner）*/
    SPI_ReleaseBus(ctx);

    /* 触发用户回调（回调在中断上下文，禁止阻塞/耗时操作）*/
    if (owner->callback != NULL)
    {
        owner->callback(owner, event);
    }
}

/* ===================== 对外接口实现 ===================== */
/**
 * @brief 注册一个 SPI 从机实例
 */
SPIInstance *SPIRegister(const SPI_Init_Config_s *conf)
{
    /* 参数合法性检查 */
    if (conf == NULL || conf->spi_handle == NULL || conf->GPIOx == NULL)
    {
        return NULL;
    }

    /* 实例池越界检查 */
    if (spi_dev_idx >= SPI_DEVICE_CNT)
    {
        return NULL;
    }

    /* 检查是否重复注册（同一 hspi + 同一 cs_pin 视为重复）*/
    for (uint8_t i = 0; i < spi_dev_idx; i++)
    {
        if (spi_instance[i]->spi_handle == conf->spi_handle &&
            spi_instance[i]->GPIOx      == conf->GPIOx      &&
            spi_instance[i]->cs_pin     == conf->cs_pin)
        {
            /* 重复注册，直接返回已有实例 */
            return spi_instance[i];
        }
    }

    /* 申请并初始化总线上下文（同一 hspi 只创建一次）*/
    if (SPI_GetOrCreateBusCtx(conf->spi_handle) == NULL)
    {
        /* 总线上下文池满，注册失败 */
        return NULL;
    }

    /* 从静态池取出新实例并清零 */
    SPIInstance *instance = pvPortMalloc(sizeof(SPIInstance));
    memset(instance, 0, sizeof(SPIInstance));

    /* 填充配置 */
    instance->spi_handle   = conf->spi_handle;
    instance->GPIOx        = conf->GPIOx;
    instance->cs_pin       = conf->cs_pin;
    instance->spi_work_mode = conf->spi_work_mode;
    instance->callback     = conf->callback;
    instance->id           = conf->id;
    instance->is_busy      = 0;

    /* 注册时主动拉高 CS，确保初始状态为非选中 */
    HAL_GPIO_WritePin(instance->GPIOx, instance->cs_pin, GPIO_PIN_SET);

    spi_instance[spi_dev_idx++] = instance;
    return instance;
}

/**
 * @brief 通过 SPI 向从机发送数据
 */
HAL_StatusTypeDef SPITransmit(SPIInstance *spi_ins, const uint8_t *ptr_data, uint16_t len)
{
    if (spi_ins == NULL || ptr_data == NULL || len == 0)
    {
        return HAL_ERROR;
    }

    /* 总线忙检查 */
    if (spi_ins->is_busy)
    {
        return HAL_BUSY;
    }

    HAL_StatusTypeDef ret = HAL_OK;

    /* 阻塞模式：不需要 owner 管理，直接完成 */
    if (spi_ins->spi_work_mode == SPI_BLOCK_MODE)
    {
        HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_RESET);
        ret = HAL_SPI_Transmit(spi_ins->spi_handle, (uint8_t *)ptr_data, len, SPI_BLOCK_TIMEOUT);
        HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_SET);
        return ret;
    }

    /* 异步模式（DMA/IT）：获取总线上下文，设置 owner */
    SPIBusContext_t *ctx = SPI_GetBusCtx(spi_ins->spi_handle);
    if (ctx == NULL)
    {
        /* 正常使用流程中不应发生（注册时已创建），此处做保护 */
        return HAL_ERROR;
    }

    /* 总线级忙检查：防止同一总线上其他从机的事务还未完成 */
    if (ctx->owner != NULL)
    {
        return HAL_BUSY;
    }

    /* 记录本次事务信息 */
    spi_ins->tx_buffer    = ptr_data;
    spi_ins->tx_size      = len;
    spi_ins->is_busy      = 1;
    ctx->owner            = spi_ins;
    ctx->pending_event    = SPI_EVENT_TX_CPLT;

    /* 拉低 CS，启动传输 */
    HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_RESET);

    switch (spi_ins->spi_work_mode)
    {
    case SPI_DMA_MODE:
        ret = HAL_SPI_Transmit_DMA(spi_ins->spi_handle, (uint8_t *)ptr_data, len);
        break;
    case SPI_IT_MODE:
        ret = HAL_SPI_Transmit_IT(spi_ins->spi_handle, (uint8_t *)ptr_data, len);
        break;
    default:
        ret = HAL_ERROR;
        break;
    }

    /* 启动失败：立即恢复总线和 CS，避免死锁 */
    if (ret != HAL_OK)
    {
        HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_SET);
        spi_ins->is_busy = 0;
        ctx->owner       = NULL;
    }

    return ret;
}

/**
 * @brief 通过 SPI 从从机接收数据
 */
HAL_StatusTypeDef SPIRecv(SPIInstance *spi_ins, uint8_t *ptr_data, uint16_t len)
{
    if (spi_ins == NULL || ptr_data == NULL || len == 0)
    {
        return HAL_ERROR;
    }

    if (spi_ins->is_busy)
    {
        return HAL_BUSY;
    }

    HAL_StatusTypeDef ret = HAL_OK;

    /* 阻塞模式 */
    if (spi_ins->spi_work_mode == SPI_BLOCK_MODE)
    {
        HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_RESET);
        ret = HAL_SPI_Receive(spi_ins->spi_handle, ptr_data, len, SPI_BLOCK_TIMEOUT);
        HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_SET);
        return ret;
    }

    /* 异步模式 */
    SPIBusContext_t *ctx = SPI_GetBusCtx(spi_ins->spi_handle);
    if (ctx == NULL)
    {
        return HAL_ERROR;
    }

    if (ctx->owner != NULL)
    {
        return HAL_BUSY;
    }

    spi_ins->rx_buffer    = ptr_data;
    spi_ins->rx_size      = len;
    spi_ins->is_busy      = 1;
    ctx->owner            = spi_ins;
    ctx->pending_event    = SPI_EVENT_RX_CPLT;

    HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_RESET);

    switch (spi_ins->spi_work_mode)
    {
    case SPI_DMA_MODE:
        ret = HAL_SPI_Receive_DMA(spi_ins->spi_handle, ptr_data, len);
        break;
    case SPI_IT_MODE:
        ret = HAL_SPI_Receive_IT(spi_ins->spi_handle, ptr_data, len);
        break;
    default:
        ret = HAL_ERROR;
        break;
    }

    if (ret != HAL_OK)
    {
        HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_SET);
        spi_ins->is_busy = 0;
        ctx->owner       = NULL;
    }

    return ret;
}

/**
 * @brief 通过 SPI 全双工同时收发数据
 */
HAL_StatusTypeDef SPITransRecv(SPIInstance *spi_ins, const uint8_t *ptr_data_tx,
                                uint8_t *ptr_data_rx, uint16_t len)
{
    if (spi_ins == NULL || ptr_data_tx == NULL || ptr_data_rx == NULL || len == 0)
    {
        return HAL_ERROR;
    }

    if (spi_ins->is_busy)
    {
        return HAL_BUSY;
    }

    HAL_StatusTypeDef ret = HAL_OK;

    /* 阻塞模式 */
    if (spi_ins->spi_work_mode == SPI_BLOCK_MODE)
    {
        HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_RESET);
        ret = HAL_SPI_TransmitReceive(spi_ins->spi_handle, (uint8_t *)ptr_data_tx,
                                      ptr_data_rx, len, SPI_BLOCK_TIMEOUT);
        HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_SET);
        return ret;
    }

    /* 异步模式 */
    SPIBusContext_t *ctx = SPI_GetBusCtx(spi_ins->spi_handle);
    if (ctx == NULL)
    {
        return HAL_ERROR;
    }

    if (ctx->owner != NULL)
    {
        return HAL_BUSY;
    }

    spi_ins->tx_buffer    = ptr_data_tx;
    spi_ins->tx_size      = len;
    spi_ins->rx_buffer    = ptr_data_rx;
    spi_ins->rx_size      = len;
    spi_ins->is_busy      = 1;
    ctx->owner            = spi_ins;
    ctx->pending_event    = SPI_EVENT_TXRX_CPLT;

    HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_RESET);

    switch (spi_ins->spi_work_mode)
    {
    case SPI_DMA_MODE:
        ret = HAL_SPI_TransmitReceive_DMA(spi_ins->spi_handle, (uint8_t *)ptr_data_tx,
                                          ptr_data_rx, len);
        break;
    case SPI_IT_MODE:
        ret = HAL_SPI_TransmitReceive_IT(spi_ins->spi_handle, (uint8_t *)ptr_data_tx,
                                         ptr_data_rx, len);
        break;
    default:
        ret = HAL_ERROR;
        break;
    }

    if (ret != HAL_OK)
    {
        HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_SET);
        spi_ins->is_busy = 0;
        ctx->owner       = NULL;
    }

    return ret;
}

/**
 * @brief 动态修改 SPI 工作模式
 */
HAL_StatusTypeDef SPISetMode(SPIInstance *spi_ins, SPI_TXRX_MODE_e spi_mode)
{
    if (spi_ins == NULL)
    {
        return HAL_ERROR;
    }

    /* 总线忙时禁止切换，防止状态异常 */
    if (spi_ins->is_busy)
    {
        return HAL_BUSY;
    }

    if (spi_mode != SPI_BLOCK_MODE &&
        spi_mode != SPI_IT_MODE    &&
        spi_mode != SPI_DMA_MODE)
    {
        return HAL_ERROR;
    }

    spi_ins->spi_work_mode = spi_mode;
    return HAL_OK;
}

/* ================== HAL 回调重写 ================== */

/**
 * @brief 仅发送完成（Tx only）
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    SPI_Complete_Routing(hspi, SPI_EVENT_TX_CPLT);
}

/**
 * @brief 仅接收完成（Rx only）
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    SPI_Complete_Routing(hspi, SPI_EVENT_RX_CPLT);
}

/**
 * @brief 全双工收发完成（TxRx）
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    SPI_Complete_Routing(hspi, SPI_EVENT_TXRX_CPLT);
}

/**
 * @brief SPI 错误回调
 *        无论哪种模式出错，都在此处释放总线，通知上层 ERROR 事件
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    SPI_Complete_Routing(hspi, SPI_EVENT_ERROR);
}