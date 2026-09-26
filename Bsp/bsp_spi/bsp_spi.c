/**
 * @file bsp_spi.c
 * @brief SPI 设备注册、共享总线仲裁、CS 管理和 HAL 回调路由
 *
 * 核心设计：
 *
 * SPIInstance
 *     表示一个逻辑 SPI 从设备，例如 BMI088 Gyro、BMI088 Accel。
 *
 * SPIBusContext_t
 *     表示一条物理 SPI 总线，例如 SPI1。
 *
 * 多个 SPIInstance 可以共享一个 SPIBusContext_t，但同一时刻一条
 * 物理 SPI 总线只能由一个设备 owner 独占。
 */
#include "bsp_spi.h"
#include <string.h>
/* ============================== 内部类型 ============================== */

/**
 * @brief BSP 内部使用的事务类型
 *
 * 上层只需要调用 SPITransmit / SPIRecv / SPITransRecv，
 * 不需要知道内部如何统一调度三类事务。
 */
typedef enum
{
    SPI_TRANSFER_TX = 0U,
    SPI_TRANSFER_RX,
    SPI_TRANSFER_TX_RX
} SPITransfer_e;
/**
 * @brief 一条物理 SPI 总线的运行状态
 *
 * handle：
 *     对应 STM32 HAL SPI 外设。
 *
 * owner：
 *     当前拥有该物理总线的逻辑设备。
 *
 * owner == NULL 表示总线空闲。
 */
typedef struct
{
    SPI_HandleTypeDef *handle;
    SPIInstance *owner;
} SPIBusContext_t;
/* ============================== 静态资源 ============================== */
/*
 * SPI 设备在系统初始化阶段注册，之后长期存在且不注销。
 * 因此直接使用静态池，避免 BSP 依赖 malloc / FreeRTOS Heap。
 */
static SPIInstance spi_instances[SPI_DEVICE_CNT];
/*
 * 最坏情况下每个设备分别使用一条不同 SPI 总线，
 * 因此总线表容量直接使用 SPI_DEVICE_CNT。
 */
static SPIBusContext_t spi_buses[SPI_DEVICE_CNT];

static uint8_t spi_device_count;
static uint8_t spi_bus_count;

/* ============================== 总线查找 ============================== */

/**
 * @brief 根据 HAL SPI 句柄查找对应的物理总线状态
 */
static SPIBusContext_t *SPI_FindBus(SPI_HandleTypeDef *handle)
{
    for (uint8_t i = 0U; i < spi_bus_count; i++)
    {
        if (spi_buses[i].handle == handle) return &spi_buses[i];
    }

    return NULL;
}

/**
 * @brief 查找已有总线，没有则在注册阶段新建一条总线记录
 *
 * 只有设备注册阶段会创建新的 SPIBusContext_t。
 */
static SPIBusContext_t *SPI_FindOrAddBus(SPI_HandleTypeDef *handle)
{
    SPIBusContext_t *bus = SPI_FindBus(handle);

    if (bus != NULL) return bus;
    if (spi_bus_count >= SPI_DEVICE_CNT) return NULL;

    bus = &spi_buses[spi_bus_count++];

    bus->handle = handle;
    bus->owner = NULL;

    return bus;
}

/* ============================== 总线仲裁 ============================== */

/**
 * @brief 原子获取一条物理 SPI 总线的所有权
 *
 * 这里必须保证：
 *
 *     检查 owner
 *          +
 *     设置 owner
 *
 * 是一个不可被任务切换或中断打断的整体操作。
 *
 * 否则可能出现：
 *
 * Task A：发现 owner == NULL
 *          ↓ 被抢占
 * Task B：发现 owner == NULL
 * Task B：owner = B
 *          ↓
 * Task A：owner = A
 *
 * 最终两个设备都会认为自己成功获得 SPI 总线。
 *
 * 临界区只保护几个变量操作，不保护真正的 SPI 传输，
 * 因此对实时性的影响非常小。
 */
static HAL_StatusTypeDef SPI_AcquireBus(SPIInstance *instance, SPIBusContext_t **bus_out)
{
    SPIBusContext_t *bus = SPI_FindBus(instance->spi_handle);

    if ((bus == NULL) || (bus_out == NULL)) return HAL_ERROR;

    /*
     * 保存进入函数前的中断状态。
     * 如果本来就在中断上下文中，则退出时不能错误地重新开中断。
     */
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    __DMB();

    /*
     * bus->owner 防止同一物理 SPI 被其他设备同时使用。
     * instance->is_busy 防止同一个设备重复发起未完成事务。
     */
    if ((bus->owner != NULL) || (instance->is_busy != 0U))
    {
        __DMB();
        if ((primask & 1U) == 0U) __enable_irq();
        return HAL_BUSY;
    }

    bus->owner = instance;
    instance->is_busy = 1U;

    __DMB();

    if ((primask & 1U) == 0U) __enable_irq();

    *bus_out = bus;

    return HAL_OK;
}

/**
 * @brief 结束当前 SPI 事务并释放总线
 *
 * 释放顺序：
 *
 * 1. CS 拉高，结束从设备事务；
 * 2. 清除设备 busy；
 * 3. 清除 bus owner。
 *
 * BLOCK 模式由同步调用结束后释放；
 * IT / DMA 模式由 HAL 完成或错误回调释放。
 */
static void SPI_ReleaseBus(SPIBusContext_t *bus)
{
    if ((bus == NULL) || (bus->owner == NULL)) return;

    SPIInstance *owner = bus->owner;

    /*
     * CS 必须在整个 SPI 物理事务期间保持 LOW。
     * 特别是 DMA / IT 模式，不能在启动函数返回后立即拉高 CS。
     */
    HAL_GPIO_WritePin(owner->GPIOx, owner->cs_pin, GPIO_PIN_SET);

    owner->is_busy = 0U;
    bus->owner = NULL;

    __DMB();
}

/* ============================== HAL 传输封装 ============================== */

/**
 * @brief 执行一次阻塞 SPI 事务
 *
 * 本函数只负责调用 HAL，不负责 CS 和总线所有权。
 * CS 生命周期统一由 SPI_RunTransfer() 管理。
 */
static HAL_StatusTypeDef SPI_RunBlocking(SPIInstance *instance, const uint8_t *tx_data, uint8_t *rx_data,
                                         uint16_t length, SPITransfer_e transfer)
{
    switch (transfer)
    {
        case SPI_TRANSFER_TX:
            return HAL_SPI_Transmit(instance->spi_handle, (uint8_t *)tx_data, length, SPI_BLOCK_TIMEOUT);

        case SPI_TRANSFER_RX:
            return HAL_SPI_Receive(instance->spi_handle, rx_data, length, SPI_BLOCK_TIMEOUT);

        case SPI_TRANSFER_TX_RX:
            return HAL_SPI_TransmitReceive(instance->spi_handle, (uint8_t *)tx_data, rx_data, length,
                                           SPI_BLOCK_TIMEOUT);

        default:
            return HAL_ERROR;
    }
}

/**
 * @brief 启动中断或 DMA 事务
 *
 * 本函数只负责选择具体 HAL 接口。
 * 启动成功后 SPI 事务尚未结束，总线和 CS 将继续保持占用，
 * 直到 HAL 完成回调调用 SPI_ReleaseBus()。
 */
static HAL_StatusTypeDef SPI_StartAsync(SPIInstance *instance, const uint8_t *tx_data, uint8_t *rx_data,
                                        uint16_t length, SPITransfer_e transfer)
{
    switch (transfer)
    {
        case SPI_TRANSFER_TX:
            if (instance->spi_work_mode == SPI_DMA_MODE)
                return HAL_SPI_Transmit_DMA(instance->spi_handle, (uint8_t *)tx_data, length);

            if (instance->spi_work_mode == SPI_IT_MODE)
                return HAL_SPI_Transmit_IT(instance->spi_handle, (uint8_t *)tx_data, length);

            break;

        case SPI_TRANSFER_RX:
            if (instance->spi_work_mode == SPI_DMA_MODE)
                return HAL_SPI_Receive_DMA(instance->spi_handle, rx_data, length);

            if (instance->spi_work_mode == SPI_IT_MODE)
                return HAL_SPI_Receive_IT(instance->spi_handle, rx_data, length);

            break;

        case SPI_TRANSFER_TX_RX:
            if (instance->spi_work_mode == SPI_DMA_MODE)
                return HAL_SPI_TransmitReceive_DMA(instance->spi_handle, (uint8_t *)tx_data, rx_data, length);

            if (instance->spi_work_mode == SPI_IT_MODE)
                return HAL_SPI_TransmitReceive_IT(instance->spi_handle, (uint8_t *)tx_data, rx_data, length);

            break;

        default:
            break;
    }

    return HAL_ERROR;
}

/* ============================== 统一事务入口 ============================== */

/**
 * @brief 完成一次 SPI 事务的统一调度
 *
 * 所有工作模式都必须遵守同样的物理总线规则：
 *
 *          获取总线
 *             ↓
 *           CS LOW
 *             ↓
 *          SPI Transfer
 *             ↓
 *           CS HIGH
 *             ↓
 *          释放总线
 *
 * BLOCK：
 *     HAL 调用返回时传输已经结束，本函数直接释放总线。
 *
 * IT / DMA：
 *     HAL 启动函数只代表“开始传输”，真正释放动作由完成回调执行。
 */
static HAL_StatusTypeDef SPI_RunTransfer(SPIInstance *instance, const uint8_t *tx_data, uint8_t *rx_data,
                                         uint16_t length, SPITransfer_e transfer)
{
    SPIBusContext_t *bus = NULL;

    HAL_StatusTypeDef status = SPI_AcquireBus(instance, &bus);

    if (status != HAL_OK) return status;

    /* 获得总线所有权以后才能选中目标 SPI 从设备。 */
    HAL_GPIO_WritePin(instance->GPIOx, instance->cs_pin, GPIO_PIN_RESET);

    if (instance->spi_work_mode == SPI_BLOCK_MODE)
    {
        status = SPI_RunBlocking(instance, tx_data, rx_data, length, transfer);

        /* 阻塞调用返回时事务已经结束，可以立即释放 CS 和总线。 */
        SPI_ReleaseBus(bus);

        return status;
    }

    /*
     * IT / DMA 只启动事务。
     * 成功后不能在这里释放，否则 CS 会在真正传输结束前被拉高。
     */
    status = SPI_StartAsync(instance, tx_data, rx_data, length, transfer);

    /*
     * 如果 HAL 连异步事务都没有成功启动，
     * 必须立即回滚 owner、busy 和 CS，否则总线会永久保持占用。
     */
    if (status != HAL_OK) SPI_ReleaseBus(bus);

    return status;
}

/* ============================== HAL 回调路由 ============================== */

/**
 * @brief 将 HAL SPI 完成事件路由到真正发起事务的设备
 *
 * HAL 回调只能告诉我们“哪个 SPI 外设完成了”，例如 hspi1，
 * 但 SPI1 上可能挂载多个逻辑设备。
 *
 * 因此需要通过：
 *
 *     hspi1
 *       ↓
 *     SPI Bus
 *       ↓
 *     bus->owner
 *       ↓
 *     具体 SPIInstance
 *
 * 才能知道当前完成的是哪个设备的事务。
 */
static void SPI_CompleteRouting(SPI_HandleTypeDef *handle, SPI_Event_e event)
{
    SPIBusContext_t *bus = SPI_FindBus(handle);

    if ((bus == NULL) || (bus->owner == NULL)) return;

    /*
     * ReleaseBus 会清空 bus->owner，
     * 因此必须先保存当前设备实例。
     */
    SPIInstance *owner = bus->owner;

    /*
     * 先结束上一笔事务，再调用设备回调。
     *
     * 这样设备 callback 中如果需要立即启动下一笔 SPI DMA，
     * 可以正常重新获取同一条总线。
     */
    SPI_ReleaseBus(bus);

    if (owner->callback != NULL) owner->callback(owner, event);
}

/* ============================== 设备注册 ============================== */

/**
 * @brief 注册一个 SPI 逻辑设备
 *
 * 设备身份由：
 *
 *     SPI Handle + GPIO Port + CS Pin
 *
 * 唯一确定。
 *
 * 例如 BMI088 Gyro 和 Accel 虽然共用 SPI1，
 * 但因为 CS 不同，所以是两个不同 SPIInstance。
 */
SPIInstance *SPIRegister(const SPI_Init_Config_s *config)
{
    if ((config == NULL) || (config->spi_handle == NULL) || (config->GPIOx == NULL)) return NULL;

    if ((config->spi_work_mode != SPI_BLOCK_MODE) &&
        (config->spi_work_mode != SPI_IT_MODE) &&
        (config->spi_work_mode != SPI_DMA_MODE))
    {
        return NULL;
    }

    /* 防止同一个物理设备被重复创建多个实例。 */
    for (uint8_t i = 0U; i < spi_device_count; i++)
    {
        SPIInstance *instance = &spi_instances[i];

        if ((instance->spi_handle == config->spi_handle) &&
            (instance->GPIOx == config->GPIOx) &&
            (instance->cs_pin == config->cs_pin))
        {
            return instance;
        }
    }

    if (spi_device_count >= SPI_DEVICE_CNT) return NULL;

    /*
     * 第一个挂到某个 SPI Handle 的设备会创建对应 Bus，
     * 后续同一 SPI 外设的设备直接共享该 Bus。
     */
    if (SPI_FindOrAddBus(config->spi_handle) == NULL) return NULL;

    SPIInstance *instance = &spi_instances[spi_device_count++];

    memset(instance, 0, sizeof(*instance));

    instance->spi_handle = config->spi_handle;
    instance->GPIOx = config->GPIOx;
    instance->cs_pin = config->cs_pin;
    instance->spi_work_mode = config->spi_work_mode;
    instance->callback = config->callback;
    instance->id = config->id;

    /* 注册完成后默认保持设备未选中。 */
    HAL_GPIO_WritePin(instance->GPIOx, instance->cs_pin, GPIO_PIN_SET);

    return instance;
}

/* ============================== 公共传输接口 ============================== */

HAL_StatusTypeDef SPITransmit(SPIInstance *instance, const uint8_t *data, uint16_t length)
{
    if ((instance == NULL) || (data == NULL) || (length == 0U)) return HAL_ERROR;

    return SPI_RunTransfer(instance, data, NULL, length, SPI_TRANSFER_TX);
}

HAL_StatusTypeDef SPIRecv(SPIInstance *instance, uint8_t *data, uint16_t length)
{
    if ((instance == NULL) || (data == NULL) || (length == 0U)) return HAL_ERROR;

    return SPI_RunTransfer(instance, NULL, data, length, SPI_TRANSFER_RX);
}

HAL_StatusTypeDef SPITransRecv(SPIInstance *instance, const uint8_t *tx_data, uint8_t *rx_data, uint16_t length)
{
    if ((instance == NULL) || (tx_data == NULL) || (rx_data == NULL) || (length == 0U)) return HAL_ERROR;

    return SPI_RunTransfer(instance, tx_data, rx_data, length, SPI_TRANSFER_TX_RX);
}

/**
 * @brief 修改该设备下一次 SPI 事务采用的模式
 *
 * 只检查当前设备自身是否 busy。
 *
 * 即使同一总线上其他设备正在传输，只要本设备没有事务，
 * 修改它“下一次传输要使用的模式”也是安全的。
 */
HAL_StatusTypeDef SPISetMode(SPIInstance *instance, SPI_TXRX_MODE_e mode)
{
    if (instance == NULL) return HAL_ERROR;

    if ((mode != SPI_BLOCK_MODE) && (mode != SPI_IT_MODE) && (mode != SPI_DMA_MODE)) return HAL_ERROR;

    if (instance->is_busy != 0U) return HAL_BUSY;

    instance->spi_work_mode = mode;

    return HAL_OK;
}

/* ============================== HAL 弱回调入口 ============================== */

/*
 * HAL 只提供按 SPI 外设划分的统一完成回调。
 * BSP 根据 bus->owner 再把事件分发给真正的逻辑设备。
 */

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *handle)
{
    SPI_CompleteRouting(handle, SPI_EVENT_TX_CPLT);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *handle)
{
    SPI_CompleteRouting(handle, SPI_EVENT_RX_CPLT);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *handle)
{
    SPI_CompleteRouting(handle, SPI_EVENT_TXRX_CPLT);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *handle)
{
    /*
     * 即使 DMA / IT 发生错误，也必须释放 CS 和 Bus，
     * 否则一次通信异常就可能导致整条 SPI 总线永久锁死。
     */
    SPI_CompleteRouting(handle, SPI_EVENT_ERROR);
}