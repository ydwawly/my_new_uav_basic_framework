//
// Created by Administrator on 2026/6/14.
//

#ifndef MY_NEW_UAV_BAICE_FRAMEWORK_BSP_SPI_H
#define MY_NEW_UAV_BAICE_FRAMEWORK_BSP_SPI_H

#include "spi.h"
#include "gpio.h"
#include "stdint.h"

/* ===================== 用户配置区 ===================== */

#define SPI_DEVICE_CNT     8    // 全局 SPI 实例最大数量（按实际从机数设置）
#define SPI_BLOCK_TIMEOUT  50   // 阻塞模式超时时间 (ms)

/* ===================================================== */

/**
 * @brief SPI 工作模式枚举
 */
typedef enum
{
    SPI_BLOCK_MODE = 0, // 阻塞模式（默认，适合低速/简单场景）
    SPI_IT_MODE,        // 中断模式
    SPI_DMA_MODE,       // DMA 模式（高速/大数据量推荐）
} SPI_TXRX_MODE_e;

/**
 * @brief SPI 完成事件枚举（用于回调通知上层是哪种操作完成/出错）
 */
typedef enum
{
    SPI_EVENT_TX_CPLT   = 0, // 仅发送完成
    SPI_EVENT_RX_CPLT,       // 仅接收完成
    SPI_EVENT_TXRX_CPLT,     // 全双工收发完成
    SPI_EVENT_ERROR,         // 传输出错
} SPI_Event_e;

/* 前置声明 */
typedef struct SPIInstance_t SPIInstance;

/**
 * @brief SPI 回调函数类型
 *        运行于中断上下文，禁止耗时操作、禁止阻塞
 *
 * @param ins   触发事件的 SPI 实例指针
 * @param event 触发的事件类型
 */
typedef void (*spi_callback_t)(SPIInstance *ins, SPI_Event_e event);

/**
 * @brief SPI 实例结构体
 */
struct SPIInstance_t
{
    /* --- 硬件绑定 --- */
    SPI_HandleTypeDef *spi_handle; // HAL SPI 句柄
    GPIO_TypeDef      *GPIOx;      // CS 片选 GPIO 端口
    uint16_t           cs_pin;     // CS 片选引脚号

    /* --- 传输配置 --- */
    SPI_TXRX_MODE_e spi_work_mode; // 当前工作模式

    /* --- 总线状态（内部维护，外部不要直接修改） --- */
    volatile uint8_t is_busy;      // 总线忙标志（1=忙，0=空闲）

    /* --- 缓冲区信息（异步模式下回调时可用） --- */
    uint16_t  tx_size;             // 本次发送字节数
    uint16_t  rx_size;             // 本次接收字节数
    uint8_t  *rx_buffer;           // 接收缓冲区指针
    const uint8_t *tx_buffer;      // 发送缓冲区指针

    /* --- 用户层绑定 --- */
    spi_callback_t callback;       // 完成/出错回调（可为 NULL）
    void          *id;             // 上层模块指针（回调时便于区分设备）
};
/**
 * @brief SPI 注册初始化配置结构体
 */
typedef struct
{
    SPI_HandleTypeDef *spi_handle; // HAL SPI 句柄
    GPIO_TypeDef      *GPIOx;      // CS 片选 GPIO 端口
    uint16_t           cs_pin;     // CS 片选引脚号
    SPI_TXRX_MODE_e    spi_work_mode;
    spi_callback_t     callback;   // 可为 NULL
    void              *id;         // 上层模块指针，可为 NULL
} SPI_Init_Config_s;


/**
 * @brief 总线上下文：记录每条 SPI 总线当前正在服务哪个实例
 *        用于中断/DMA 完成回调时，快速找到 owner 而不靠读 GPIO 电平
 */
typedef struct
{
    SPI_HandleTypeDef *hspi;   // 总线句柄
    SPIInstance       *owner;  // 当前持有总线的实例（NULL = 总线空闲）
    SPI_Event_e        pending_event; // 本次传输对应的事件类型
} SPIBusContext_t;

/* ================== 对外接口声明 ================== */

/**
 * @brief  注册一个 SPI 从机实例
 * @param  conf 初始化配置，不得为 NULL
 * @retval 成功返回实例指针；失败（参数非法/池满/重复注册）返回 NULL
 * @note   注册时会自动将 CS 拉高（释放/非选中状态）
 */
SPIInstance *SPIRegister(const SPI_Init_Config_s *conf);

/**
 * @brief  通过 SPI 向从机发送数据
 * @param  spi_ins  目标 SPI 实例
 * @param  ptr_data 发送缓冲区（DMA/IT 模式下必须保持有效直到回调触发）
 * @param  len      发送字节数（1~65535）
 * @retval HAL_OK / HAL_ERROR / HAL_BUSY / HAL_TIMEOUT
 * @note   DMA/IT 模式下函数立即返回，完成后触发 SPI_EVENT_TX_CPLT 回调
 * @note   阻塞模式下函数返回时传输已完成
 */
HAL_StatusTypeDef SPITransmit(SPIInstance *spi_ins, const uint8_t *ptr_data, uint16_t len);

/**
 * @brief  通过 SPI 从从机接收数据
 * @param  spi_ins  目标 SPI 实例
 * @param  ptr_data 接收缓冲区（DMA/IT 模式下必须保持有效直到回调触发）
 * @param  len      接收字节数（1~65535）
 * @retval HAL_OK / HAL_ERROR / HAL_BUSY / HAL_TIMEOUT
 * @note   DMA/IT 模式下函数立即返回，完成后触发 SPI_EVENT_RX_CPLT 回调
 */
HAL_StatusTypeDef SPIRecv(SPIInstance *spi_ins, uint8_t *ptr_data, uint16_t len);

/**
 * @brief  通过 SPI 全双工同时收发数据
 * @param  spi_ins     目标 SPI 实例
 * @param  ptr_data_tx 发送缓冲区
 * @param  ptr_data_rx 接收缓冲区
 * @param  len         收发字节数（1~65535）
 * @retval HAL_OK / HAL_ERROR / HAL_BUSY / HAL_TIMEOUT
 * @note   DMA/IT 模式下函数立即返回，完成后触发 SPI_EVENT_TXRX_CPLT 回调
 */
HAL_StatusTypeDef SPITransRecv(SPIInstance *spi_ins, const uint8_t *ptr_data_tx,
                                uint8_t *ptr_data_rx, uint16_t len);

/**
 * @brief  动态修改 SPI 工作模式
 * @param  spi_ins  目标实例
 * @param  spi_mode 新工作模式
 * @retval HAL_OK / HAL_ERROR（实例为 NULL 或总线忙时不允许切换）
 * @note   请勿在传输过程中切换模式
 */
HAL_StatusTypeDef SPISetMode(SPIInstance *spi_ins, SPI_TXRX_MODE_e spi_mode);

#endif //MY_NEW_UAV_BAICE_FRAMEWORK_BSP_SPI_H