/**
 * @file bsp_iic.h
 * @brief 支持多设备共享总线的 I²C 阻塞、中断和 DMA 访问接口
 */
#ifndef STM32H743_UAV_FLIGHT_CONTROLLER_BSP_IIC_H
#define STM32H743_UAV_FLIGHT_CONTROLLER_BSP_IIC_H

#include <stdint.h>

#include "i2c.h"

/** @brief 一次 I²C 事务采用的 HAL 执行模式。 */
typedef enum
{
    IIC_TRANSFER_BLOCKING = 0U,
    IIC_TRANSFER_IT,
    IIC_TRANSFER_DMA
} IIC_TransferMode_e;

/** @brief 寄存器地址宽度，取值与 STM32 HAL 保持一致。 */
typedef enum
{
    IIC_MEMORY_ADDRESS_8BIT  = I2C_MEMADD_SIZE_8BIT,
    IIC_MEMORY_ADDRESS_16BIT = I2C_MEMADD_SIZE_16BIT
} IIC_MemoryAddressSize_e;

/** @brief 异步事务完成或失败时上报给设备层的事件。 */
typedef enum
{
    IIC_EVENT_MASTER_TX_COMPLETE = 0U,
    IIC_EVENT_MASTER_RX_COMPLETE,
    IIC_EVENT_MEMORY_TX_COMPLETE,
    IIC_EVENT_MEMORY_RX_COMPLETE,
    IIC_EVENT_ERROR
} IIC_Event_e;

typedef struct IICInstance_t IICInstance;
typedef void (*iic_event_callback_t)(IICInstance *instance, IIC_Event_e event);

/** @brief 已注册 I²C 设备的运行时上下文。 */
struct IICInstance_t
{
    I2C_HandleTypeDef   *i2c_handle;
    uint8_t              address_7bit;
    IIC_TransferMode_e   transfer_mode;
    iic_event_callback_t callback;
    void                *id;
    volatile uint8_t     busy;
};

/** @brief 注册 I²C 设备所需的静态配置。 */
typedef struct
{
    I2C_HandleTypeDef   *i2c_handle;
    uint8_t              address_7bit;
    IIC_TransferMode_e   transfer_mode;
    iic_event_callback_t callback;
    void                *id;
} IIC_Init_Config_s;

/** @brief 从静态池注册一个 7 位地址的 I²C 设备。 */
IICInstance *IICRegister(const IIC_Init_Config_s *config);

/** @brief 总线空闲时修改该设备后续事务使用的传输模式。 */
HAL_StatusTypeDef IICSetMode(IICInstance *instance, IIC_TransferMode_e mode);

/** @brief 总线空闲时修改设备 7 位地址。 */
HAL_StatusTypeDef IICSetAddress(IICInstance *instance, uint8_t address_7bit);

/** @brief 使用阻塞 HAL 探测目标设备是否应答。 */
HAL_StatusTypeDef IICIsDeviceReady(IICInstance *instance, uint32_t trials, uint32_t timeout_ms);

/* DMA/IT 缓冲区必须保持有效，直到中断上下文中的回调执行完毕。 */
HAL_StatusTypeDef IICTransmit(IICInstance *instance, const uint8_t *data, uint16_t size);
HAL_StatusTypeDef IICReceive(IICInstance *instance, uint8_t *data, uint16_t size);
HAL_StatusTypeDef IICMemWrite(IICInstance *instance, uint16_t memory_address,
                              IIC_MemoryAddressSize_e memory_address_size, const uint8_t *data, uint16_t size);
HAL_StatusTypeDef IICMemRead(IICInstance *instance, uint16_t memory_address,
                             IIC_MemoryAddressSize_e memory_address_size, uint8_t *data, uint16_t size);

#endif /* STM32H743_UAV_FLIGHT_CONTROLLER_BSP_IIC_H */
