#include "bsp_iic.h"

/**
 * @file bsp_iic.c
 * @brief I²C 设备注册、共享总线仲裁以及阻塞/中断/DMA 传输实现。
 *
 * 对外统一使用 7 位从机地址，调用 STM32 HAL 前再左移一位。异步事务由物理总线记录
 * 唯一拥有者，完成或错误回调通过 HAL 句柄把事件路由回对应设备。
 */

#include <string.h>

#define IIC_DEVICE_COUNT  8U
#define IIC_BUS_COUNT     4U
#define IIC_BLOCK_TIMEOUT 100U

typedef enum
{
    IIC_OPERATION_MASTER_TX = 0U,
    IIC_OPERATION_MASTER_RX,
    IIC_OPERATION_MEMORY_TX,
    IIC_OPERATION_MEMORY_RX
} IICOperation_e;

typedef struct
{
    I2C_HandleTypeDef *handle;
    IICInstance       *owner;
} IICBusContext_t;

/* 设备与总线均使用静态表，初始化完成后不需要动态内存。 */
static IICInstance     iic_instances[IIC_DEVICE_COUNT];
static IICBusContext_t iic_buses[IIC_BUS_COUNT];
static uint8_t         iic_device_count;
static uint8_t         iic_bus_count;

/** @brief 按 HAL 句柄查找共享总线上下文。 */
static IICBusContext_t *IIC_FindBus(I2C_HandleTypeDef *handle)
{
    for (uint8_t index = 0U; index < iic_bus_count; index++)
    {
        if (iic_buses[index].handle == handle)
        {
            return &iic_buses[index];
        }
    }
    return NULL;
}

/** @brief 返回已有总线记录，或在容量允许时创建一条未占用记录。 */
static IICBusContext_t *IIC_FindOrAddBus(I2C_HandleTypeDef *handle)
{
    IICBusContext_t *bus = IIC_FindBus(handle);
    if (bus != NULL)
    {
        return bus;
    }
    if ((handle == NULL) || (iic_bus_count >= IIC_BUS_COUNT))
    {
        return NULL;
    }

    bus         = &iic_buses[iic_bus_count++];
    bus->handle = handle;
    bus->owner  = NULL;
    return bus;
}

/* 阻塞调用不能插入 BSP 已经占用总线的异步事务。 */
static HAL_StatusTypeDef IIC_CheckBlockingBus(const IICInstance *instance)
{
    if ((instance == NULL) || (instance->i2c_handle == NULL))
    {
        return HAL_ERROR;
    }

    const IICBusContext_t *bus = IIC_FindBus(instance->i2c_handle);
    return ((bus != NULL) && (bus->owner != NULL)) ? HAL_BUSY : HAL_OK;
}

/** @brief 为异步事务独占物理总线，并同步设置设备 busy 标志。 */
static HAL_StatusTypeDef IIC_ClaimBus(IICInstance *instance)
{
    if ((instance == NULL) || (instance->i2c_handle == NULL))
    {
        return HAL_ERROR;
    }

    IICBusContext_t *bus = IIC_FindBus(instance->i2c_handle);
    if (bus == NULL)
    {
        return HAL_ERROR;
    }

    /* 当前 SPL06 路径保证每条 I2C 总线只有一个异步事务生产者。 */
    if ((bus->owner != NULL) || (instance->busy != 0U))
    {
        return HAL_BUSY;
    }

    bus->owner     = instance;
    instance->busy = 1U;
    return HAL_OK;
}

/** @brief 清除当前异步事务的设备 busy 标志和总线拥有者。 */
static void IIC_ReleaseBus(IICBusContext_t *bus)
{
    if ((bus == NULL) || (bus->owner == NULL))
    {
        return;
    }

    bus->owner->busy = 0U;
    bus->owner       = NULL;
}

/** @brief 从 HAL 回调反查事务拥有者，先释放总线，再通知设备层。 */
static void IIC_CompleteRouting(I2C_HandleTypeDef *handle, IIC_Event_e event)
{
    IICBusContext_t *bus = IIC_FindBus(handle);
    if ((bus == NULL) || (bus->owner == NULL))
    {
        return;
    }

    IICInstance *owner = bus->owner;
    IIC_ReleaseBus(bus);
    if (owner->callback != NULL)
    {
        owner->callback(owner, event);
    }
}

/** @brief 将框架约定的 7 位地址转换为 STM32 HAL 使用的左对齐地址。 */
static uint16_t IIC_HalAddress(const IICInstance *instance)
{
    return (uint16_t)instance->address_7bit << 1U;
}

/** @brief 在调用者上下文执行阻塞式主机或寄存器读写。 */
static HAL_StatusTypeDef IIC_RunBlocking(IICInstance *instance, IICOperation_e operation, uint16_t memory_address,
                                         IIC_MemoryAddressSize_e memory_address_size, const uint8_t *tx_data,
                                         uint8_t *rx_data, uint16_t size)
{
    switch (operation)
    {
    case IIC_OPERATION_MASTER_TX:
        return HAL_I2C_Master_Transmit(instance->i2c_handle, IIC_HalAddress(instance), (uint8_t *)tx_data, size,
                                       IIC_BLOCK_TIMEOUT);
    case IIC_OPERATION_MASTER_RX:
        return HAL_I2C_Master_Receive(instance->i2c_handle, IIC_HalAddress(instance), rx_data, size, IIC_BLOCK_TIMEOUT);
    case IIC_OPERATION_MEMORY_TX:
        return HAL_I2C_Mem_Write(instance->i2c_handle, IIC_HalAddress(instance), memory_address, memory_address_size,
                                 (uint8_t *)tx_data, size, IIC_BLOCK_TIMEOUT);
    case IIC_OPERATION_MEMORY_RX:
        return HAL_I2C_Mem_Read(instance->i2c_handle, IIC_HalAddress(instance), memory_address, memory_address_size,
                                rx_data, size, IIC_BLOCK_TIMEOUT);
    }
    return HAL_ERROR;
}

/** @brief 根据设备配置启动中断或 DMA 异步传输。 */
static HAL_StatusTypeDef IIC_StartAsync(IICInstance *instance, IICOperation_e operation, uint16_t memory_address,
                                        IIC_MemoryAddressSize_e memory_address_size, const uint8_t *tx_data,
                                        uint8_t *rx_data, uint16_t size)
{
    const uint16_t address = IIC_HalAddress(instance);
    const uint8_t  use_dma = (instance->transfer_mode == IIC_TRANSFER_DMA) ? 1U : 0U;

    switch (operation)
    {
    case IIC_OPERATION_MASTER_TX:
        return (use_dma != 0U) ? HAL_I2C_Master_Transmit_DMA(instance->i2c_handle, address, (uint8_t *)tx_data, size)
                               : HAL_I2C_Master_Transmit_IT(instance->i2c_handle, address, (uint8_t *)tx_data, size);
    case IIC_OPERATION_MASTER_RX:
        return (use_dma != 0U) ? HAL_I2C_Master_Receive_DMA(instance->i2c_handle, address, rx_data, size)
                               : HAL_I2C_Master_Receive_IT(instance->i2c_handle, address, rx_data, size);
    case IIC_OPERATION_MEMORY_TX:
        return (use_dma != 0U) ? HAL_I2C_Mem_Write_DMA(instance->i2c_handle, address, memory_address,
                                                       memory_address_size, (uint8_t *)tx_data, size)
                               : HAL_I2C_Mem_Write_IT(instance->i2c_handle, address, memory_address,
                                                      memory_address_size, (uint8_t *)tx_data, size);
    case IIC_OPERATION_MEMORY_RX:
        return (use_dma != 0U) ? HAL_I2C_Mem_Read_DMA(instance->i2c_handle, address, memory_address,
                                                      memory_address_size, rx_data, size)
                               : HAL_I2C_Mem_Read_IT(instance->i2c_handle, address, memory_address, memory_address_size,
                                                     rx_data, size);
    }
    return HAL_ERROR;
}

/**
 * @brief 统一调度一次 I²C 事务。
 *
 * 阻塞事务只检查总线是否被异步事务占用；异步事务先声明总线所有权，若 HAL 启动失败，
 * 会立即释放所有权，避免设备永久停留在 busy 状态。
 */
static HAL_StatusTypeDef IIC_RunTransfer(IICInstance *instance, IICOperation_e operation, uint16_t memory_address,
                                         IIC_MemoryAddressSize_e memory_address_size, const uint8_t *tx_data,
                                         uint8_t *rx_data, uint16_t size)
{
    if (instance->transfer_mode == IIC_TRANSFER_BLOCKING)
    {
        const HAL_StatusTypeDef available = IIC_CheckBlockingBus(instance);
        if (available != HAL_OK)
        {
            return HAL_BUSY;
        }
        return IIC_RunBlocking(instance, operation, memory_address, memory_address_size, tx_data, rx_data, size);
    }

    HAL_StatusTypeDef status = IIC_ClaimBus(instance);
    if (status != HAL_OK)
    {
        return status;
    }

    status = IIC_StartAsync(instance, operation, memory_address, memory_address_size, tx_data, rx_data, size);
    if (status != HAL_OK)
    {
        IIC_ReleaseBus(IIC_FindBus(instance->i2c_handle));
    }
    return status;
}

/**
 * @brief 注册由“HAL I²C 句柄 + 7 位地址”唯一标识的从设备。
 * @return 重复注册返回原实例；配置非法或静态表已满时返回 NULL。
 */
IICInstance *IICRegister(const IIC_Init_Config_s *config)
{
    if ((config == NULL) || (config->i2c_handle == NULL) || (config->address_7bit > 0x7FU))
    {
        return NULL;
    }

    for (uint8_t index = 0U; index < iic_device_count; index++)
    {
        IICInstance *instance = &iic_instances[index];
        if ((instance->i2c_handle == config->i2c_handle) && (instance->address_7bit == config->address_7bit))
        {
            return instance;
        }
    }

    if ((iic_device_count >= IIC_DEVICE_COUNT) || (IIC_FindOrAddBus(config->i2c_handle) == NULL))
    {
        return NULL;
    }

    IICInstance *instance = &iic_instances[iic_device_count++];
    memset(instance, 0, sizeof(*instance));
    instance->i2c_handle    = config->i2c_handle;
    instance->address_7bit  = config->address_7bit;
    instance->transfer_mode = config->transfer_mode;
    instance->callback      = config->callback;
    instance->id            = config->id;
    return instance;
}

/** @brief 仅在设备及其所在总线空闲时切换传输模式。 */
HAL_StatusTypeDef IICSetMode(IICInstance *instance, IIC_TransferMode_e mode)
{
    if ((instance == NULL) || (mode > IIC_TRANSFER_DMA))
    {
        return HAL_ERROR;
    }

    IICBusContext_t *bus = IIC_FindBus(instance->i2c_handle);
    if ((instance->busy != 0U) || ((bus != NULL) && (bus->owner != NULL)))
    {
        return HAL_BUSY;
    }

    instance->transfer_mode = mode;
    return HAL_OK;
}

/** @brief 在总线无异步事务时更新设备的 7 位从机地址。 */
HAL_StatusTypeDef IICSetAddress(IICInstance *instance, uint8_t address_7bit)
{
    if ((instance == NULL) || (address_7bit > 0x7FU))
    {
        return HAL_ERROR;
    }
    if (IIC_CheckBlockingBus(instance) != HAL_OK)
    {
        return HAL_BUSY;
    }

    instance->address_7bit = address_7bit;
    return HAL_OK;
}

/** @brief 使用 HAL 轮询设备是否响应；探测期间不允许异步事务占用总线。 */
HAL_StatusTypeDef IICIsDeviceReady(IICInstance *instance, uint32_t trials, uint32_t timeout_ms)
{
    if ((instance == NULL) || (instance->i2c_handle == NULL) || (trials == 0U))
    {
        return HAL_ERROR;
    }

    IICBusContext_t *bus = IIC_FindBus(instance->i2c_handle);
    if ((bus != NULL) && (bus->owner != NULL))
    {
        return HAL_BUSY;
    }
    return HAL_I2C_IsDeviceReady(instance->i2c_handle, IIC_HalAddress(instance), trials, timeout_ms);
}

/** @brief 向当前从机地址发送原始字节流。 */
HAL_StatusTypeDef IICTransmit(IICInstance *instance, const uint8_t *data, uint16_t size)
{
    if ((instance == NULL) || (data == NULL) || (size == 0U))
    {
        return HAL_ERROR;
    }
    return IIC_RunTransfer(instance, IIC_OPERATION_MASTER_TX, 0U, IIC_MEMORY_ADDRESS_8BIT, data, NULL, size);
}

/** @brief 从当前从机地址接收原始字节流。 */
HAL_StatusTypeDef IICReceive(IICInstance *instance, uint8_t *data, uint16_t size)
{
    if ((instance == NULL) || (data == NULL) || (size == 0U))
    {
        return HAL_ERROR;
    }
    return IIC_RunTransfer(instance, IIC_OPERATION_MASTER_RX, 0U, IIC_MEMORY_ADDRESS_8BIT, NULL, data, size);
}

/** @brief 写入从机寄存器，寄存器地址宽度由 memory_address_size 指定。 */
HAL_StatusTypeDef IICMemWrite(IICInstance *instance, uint16_t memory_address,
                              IIC_MemoryAddressSize_e memory_address_size, const uint8_t *data, uint16_t size)
{
    if ((instance == NULL) || (data == NULL) || (size == 0U))
    {
        return HAL_ERROR;
    }
    return IIC_RunTransfer(instance, IIC_OPERATION_MEMORY_TX, memory_address, memory_address_size, data, NULL, size);
}

/** @brief 读取从机寄存器，寄存器地址宽度由 memory_address_size 指定。 */
HAL_StatusTypeDef IICMemRead(IICInstance *instance, uint16_t memory_address,
                             IIC_MemoryAddressSize_e memory_address_size, uint8_t *data, uint16_t size)
{
    if ((instance == NULL) || (data == NULL) || (size == 0U))
    {
        return HAL_ERROR;
    }
    return IIC_RunTransfer(instance, IIC_OPERATION_MEMORY_RX, memory_address, memory_address_size, NULL, data, size);
}

/* HAL 弱回调统一进入总线路由器，保证共享总线上的回调归属正确。 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *handle)
{
    IIC_CompleteRouting(handle, IIC_EVENT_MASTER_TX_COMPLETE);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *handle)
{
    IIC_CompleteRouting(handle, IIC_EVENT_MASTER_RX_COMPLETE);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *handle)
{
    IIC_CompleteRouting(handle, IIC_EVENT_MEMORY_TX_COMPLETE);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *handle)
{
    IIC_CompleteRouting(handle, IIC_EVENT_MEMORY_RX_COMPLETE);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *handle)
{
    IIC_CompleteRouting(handle, IIC_EVENT_ERROR);
}
