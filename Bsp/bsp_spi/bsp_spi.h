/**
 * @file bsp_spi.h
 * @brief 共享 SPI 总线的设备抽象，支持阻塞、中断和 DMA 模式
 *
 * 一个物理 SPI 外设可以挂载多个从设备，不同设备通过独立 CS 区分。
 * BSP 负责统一管理：
 * 1. SPI 设备注册；
 * 2. 物理总线所有权；
 * 3. CS 生命周期；
 * 4. BLOCK / IT / DMA 传输；
 * 5. HAL 完成回调到具体设备的路由。
 */

#ifndef STM32H743_UAV_FLIGHT_CONTROLLER_BSP_SPI_H
#define STM32H743_UAV_FLIGHT_CONTROLLER_BSP_SPI_H

#include <stdint.h>

#include "gpio.h"
#include "spi.h"

/* 最大 SPI 逻辑设备数量。设备启动后常驻，因此采用固定静态容量。 */
#define SPI_DEVICE_CNT 8U

/* 阻塞传输最大等待时间，单位 ms。 */
#define SPI_BLOCK_TIMEOUT 50U

/**
 * @brief SPI 设备工作模式
 *
 * BLOCK：调用函数直到传输完成才返回。
 * IT   ：启动中断传输后立即返回。
 * DMA  ：启动 DMA 传输后立即返回。
 */
typedef enum
{
    SPI_BLOCK_MODE = 0U,
    SPI_IT_MODE,
    SPI_DMA_MODE
} SPI_TXRX_MODE_e;

/**
 * @brief 异步 SPI 事务完成事件
 *
 * IT / DMA 模式完成后，BSP 根据当前总线 owner 将事件路由给对应设备。
 */
typedef enum
{
    SPI_EVENT_TX_CPLT = 0U,
    SPI_EVENT_RX_CPLT,
    SPI_EVENT_TXRX_CPLT,
    SPI_EVENT_ERROR
} SPI_Event_e;

typedef struct SPIInstance_t SPIInstance;

/**
 * @brief SPI 设备回调函数
 *
 * @note 回调由 HAL SPI 中断回调触发，因此运行在中断上下文。
 *       回调中应只做置标志、任务通知、启动下一笔异步事务等短操作，
 *       不应执行阻塞等待或耗时处理。
 */
typedef void (*spi_callback_t)(SPIInstance *instance, SPI_Event_e event);

/**
 * @brief 一个逻辑 SPI 从设备的运行时实例
 *
 * 同一个 spi_handle 可以对应多个 SPIInstance，
 * 每个实例通过 GPIOx + cs_pin 区分具体从设备。
 */
struct SPIInstance_t
{
    SPI_HandleTypeDef *spi_handle;

    GPIO_TypeDef *GPIOx;
    uint16_t      cs_pin;

    SPI_TXRX_MODE_e spi_work_mode;

    /*
     * 表示当前设备是否有未完成事务。
     * IT / DMA 模式下从启动传输持续到 HAL 完成回调。
     */
    volatile uint8_t is_busy;

    spi_callback_t callback;

    /*
     * 上层私有上下文指针。
     * 可用于在统一回调中反查具体设备对象，相当于 C 中的 this 指针。
     */
    void *id;
};

/**
 * @brief SPI 设备注册配置
 *
 * SPI 外设 + GPIO Port + CS Pin 唯一确定一个逻辑 SPI 设备。
 */
typedef struct
{
    SPI_HandleTypeDef *spi_handle;

    GPIO_TypeDef *GPIOx;
    uint16_t      cs_pin;

    SPI_TXRX_MODE_e spi_work_mode;
    spi_callback_t  callback;
    void           *id;
} SPI_Init_Config_s;

/**
 * @brief 注册一个 SPI 从设备
 *
 * 重复注册同一个 SPI + GPIO + CS 时直接返回已有实例。
 *
 * @return 成功返回实例地址，配置非法或静态池耗尽返回 NULL。
 */
SPIInstance *SPIRegister(const SPI_Init_Config_s *config);

/**
 * @brief 发起仅发送事务
 *
 * BLOCK 模式下函数在传输完成后返回。
 * IT / DMA 模式下函数启动事务后立即返回。
 *
 * @note IT / DMA 模式下 data 必须保持有效直到完成回调执行。
 */
HAL_StatusTypeDef SPITransmit(SPIInstance *instance, const uint8_t *data, uint16_t length);

/**
 * @brief 发起仅接收事务
 *
 * @note IT / DMA 模式下 data 必须保持有效直到完成回调执行。
 */
HAL_StatusTypeDef SPIRecv(SPIInstance *instance, uint8_t *data, uint16_t length);

/**
 * @brief 发起全双工收发事务
 *
 * @note IT / DMA 模式下 tx_data 和 rx_data 都必须保持有效，
 *       直到完成回调执行。
 */
HAL_StatusTypeDef SPITransRecv(SPIInstance *instance, const uint8_t *tx_data, uint8_t *rx_data, uint16_t length);

/**
 * @brief 修改设备后续传输使用的工作模式
 *
 * 当前设备存在未完成事务时禁止修改。
 */
HAL_StatusTypeDef SPISetMode(SPIInstance *instance, SPI_TXRX_MODE_e mode);

#endif /* STM32H743_UAV_FLIGHT_CONTROLLER_BSP_SPI_H */
