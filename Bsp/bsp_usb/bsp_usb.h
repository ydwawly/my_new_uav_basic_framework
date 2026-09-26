//
// Created by Administrator on 2026/6/20.
//

#ifndef STM32H743_UAV_FLIGHT_CONTROLLER_BSP_USB_H
#define STM32H743_UAV_FLIGHT_CONTROLLER_BSP_USB_H

#include <stdbool.h>
#include <stdint.h>
#include "bsp_ringbuffer.h"
#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================== 配置项 ========================== */

#ifndef USB_TX_RINGBUF_SIZE
#define USB_TX_RINGBUF_SIZE (32U * 1024U)
#endif

#ifndef USB_TX_CHUNK_SIZE
#define USB_TX_CHUNK_SIZE 512U
#endif

#if (USB_TX_RINGBUF_SIZE == 0U)
#error "USB_TX_RINGBUF_SIZE must be greater than 0."
#endif

#if ((USB_TX_RINGBUF_SIZE & (USB_TX_RINGBUF_SIZE - 1U)) != 0U)
#error "USB_TX_RINGBUF_SIZE must be a power of two."
#endif

#if (USB_TX_CHUNK_SIZE == 0U)
#error "USB_TX_CHUNK_SIZE must be greater than 0."
#endif

#if (USB_TX_CHUNK_SIZE > USB_TX_RINGBUF_SIZE)
#error "USB_TX_CHUNK_SIZE must not exceed USB_TX_RINGBUF_SIZE."
#endif

/* ========================== 类型定义 ========================== */

typedef struct USBInstance USBInstance;

typedef enum
{
    USB_EVENT_RX_CPLT = 0,
    USB_EVENT_TX_CPLT,
    USB_EVENT_ERROR
} USB_EVENT_e;

/**
 * @brief USB CDC 统一事件回调
 *
 * @note 该回调通常运行在 USB 中断上下文。
 *       回调内禁止阻塞，只进行数据复制、状态更新或任务通知。
 */
typedef void (*USB_Event_Callback_t)(USBInstance *instance, USB_EVENT_e event, uint8_t *data, uint16_t size);
/* ========================== 私有类型 ========================== */
struct USBInstance
{
    USB_Event_Callback_t event_callback;
    RingBuffer_t         tx_ringbuffer;
    uint8_t             *tx_ringbuffer_pool;
    uint8_t             *tx_pending_buffer;

    TaskHandle_t tx_task_handle;

    volatile uint16_t tx_pending_size;
    volatile uint8_t  tx_busy;
    uint8_t           initialized;

    volatile uint32_t total_tx_bytes;
    volatile uint32_t dropped_tx_bytes;
};

typedef struct
{
    USB_Event_Callback_t event_callback;
} USB_Init_Config_s;

/* ========================== 公有接口 ========================== */

/**
 * @brief 注册 USB CDC BSP 实例
 *
 * @return 成功返回 USBInstance 指针，失败返回 NULL
 */
USBInstance *USBRegister(const USB_Init_Config_s *init_config);

/**
 * @brief 异步发送 USB CDC 数据
 *
 * 数据会被复制到内部 RingBuffer，函数返回后调用者可立即复用 data。
 *
 * @note 该接口必须满足单生产者约束：
 *       只能由一个固定任务调用，不能在中断中调用。
 */
bool USBSend(USBInstance *instance, const uint8_t *data, uint16_t size);

/**
 * @brief 查询 USB CDC 是否已经完成枚举
 */
uint8_t USBIsConnected(void);

/**
 * @brief 获取 USB 发送统计信息
 */
void USBGetTxStats(USBInstance *instance, uint32_t *total_bytes, uint32_t *dropped_bytes, float *buffer_usage);

/**
 * @brief USB CDC 异步发送任务
 *
 * 创建任务时，将 USBRegister() 的返回值作为 pvParameters 传入。
 */
void USBTxTask(void *pvParameters);

/* ========================== USB 中间件钩子 ========================== */

/**
 * @brief 在 CDC_Receive_FS() 中调用
 */
void USB_CDC_RxHook(uint8_t *buf, uint32_t size);

/**
 * @brief 在 CDC_TransmitCplt_FS() 中调用
 */
void USB_CDC_TxCpltHook(void);

#ifdef __cplusplus
}
#endif
#endif /* STM32H743_UAV_FLIGHT_CONTROLLER_BSP_USB_H */
