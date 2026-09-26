/** @file bsp_usb.c @brief USB CDC 环形缓冲、DMA 缓冲与回调路由实现。 */

#include "bsp_usb.h"
#include "bsp_memory_section.h"
#include "bsp_RTT.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <string.h>

/* CubeMX 默认 USB Device 句柄 */
extern USBD_HandleTypeDef hUsbDeviceFS;
/* ========================== 私有变量 ========================== */
static USBInstance usb_instance = {0};
/* ========================== 私有函数声明 ========================== */
static void USBTransmitHandler(USBInstance *instance);
static void USBDiscardTxData(USBInstance *instance);
static void USBAddDroppedBytes(USBInstance *instance, uint32_t size);
/* ========================== 公有接口实现 ========================== */
USBInstance *USBRegister(const USB_Init_Config_s *init_config)
{
    USBInstance *instance = &usb_instance;

    /* ---------- 参数合法性检查 ---------- */
    if (init_config == NULL)
    {
        RTTERROR("[bsp_usb] Register failed: NULL config.");
        return NULL;
    }

    if (instance->initialized)
    {
        RTTERROR("[bsp_usb] Register failed: USB CDC already registered.");
        return NULL;
    }

    memset(instance, 0, sizeof(USBInstance));

    /* ---------- 填写基本字段 ---------- */
    instance->event_callback = init_config->event_callback;

    /* ---------- 分配发送缓存 ---------- */
    instance->tx_ringbuffer_pool = (uint8_t *)BSP_DMA_Malloc(USB_TX_RINGBUF_SIZE);

    instance->tx_pending_buffer = (uint8_t *)BSP_DMA_Malloc(USB_TX_CHUNK_SIZE);

    if (instance->tx_ringbuffer_pool == NULL || instance->tx_pending_buffer == NULL)
    {
        RTTERROR("[bsp_usb] Register failed: DMA malloc returned NULL.");
        configASSERT(0);
        return NULL;
    }

    /* ---------- 初始化发送 RingBuffer ---------- */
    if (!RingBuffer_Init(&instance->tx_ringbuffer, instance->tx_ringbuffer_pool, USB_TX_RINGBUF_SIZE))
    {
        RTTERROR("[bsp_usb] Register failed: RingBuffer_Init failed.");
        configASSERT(0);
        return NULL;
    }

    memset(instance->tx_pending_buffer, 0, USB_TX_CHUNK_SIZE);

    instance->initialized = 1U;

    RTTINFO("[bsp_usb] Registered OK. TX ring=%uB, chunk=%uB.", (unsigned)USB_TX_RINGBUF_SIZE,
            (unsigned)USB_TX_CHUNK_SIZE);

    return instance;
}

bool USBSend(USBInstance *instance, const uint8_t *data, uint16_t size)
{
    if (instance == NULL || instance != &usb_instance || !instance->initialized || data == NULL || size == 0U)
    {
        return false;
    }

    /*
     * 未连接时不缓存旧数据，避免主机重新连接后收到过期数据。
     */
    if (!USBIsConnected())
    {
        USBAddDroppedBytes(instance, size);
        return false;
    }

    /*
     * SPSC 无锁模型：
     * - 唯一生产者任务调用 RingBuffer_Push()，只更新 head；
     * - USBTxTask 调用 RingBuffer_Pop()，只更新 tail。
     */
    if (!RingBuffer_Push(&instance->tx_ringbuffer, data, size))
    {
        USBAddDroppedBytes(instance, size);
        return false;
    }

    /* 唤醒 USB 发送任务 */
    if (instance->tx_task_handle != NULL)
    {
        xTaskNotifyGive(instance->tx_task_handle);
    }

    return true;
}

uint8_t USBIsConnected(void)
{
    return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) ? 1U : 0U;
}

void USBGetTxStats(USBInstance *instance, uint32_t *total_bytes, uint32_t *dropped_bytes, float *buffer_usage)
{
    uint32_t used;

    if (instance == NULL || instance != &usb_instance || !instance->initialized)
    {
        return;
    }

    /*
     * STM32 为单核 MCU，短临界区可获得一致的统计快照，
     * 不参与 RingBuffer 正常 Push/Pop 路径。
     */
    taskENTER_CRITICAL();

    if (total_bytes != NULL)
    {
        *total_bytes = instance->total_tx_bytes;
    }

    if (dropped_bytes != NULL)
    {
        *dropped_bytes = instance->dropped_tx_bytes;
    }

    used = RingBuffer_GetUsed(&instance->tx_ringbuffer);

    taskEXIT_CRITICAL();

    if (buffer_usage != NULL)
    {
        *buffer_usage = (float)used / (float)USB_TX_RINGBUF_SIZE;
    }
}

/* ========================== 发送任务 ========================== */

void USBTxTask(void *pvParameters)
{
    USBInstance *instance = (USBInstance *)pvParameters;

    if (instance == NULL || instance != &usb_instance || !instance->initialized)
    {
        RTTERROR("[bsp_usb] TX task failed: invalid instance.");
        vTaskDelete(NULL);
        return;
    }

    instance->tx_task_handle = xTaskGetCurrentTaskHandle();

    RTTINFO("[bsp_usb] TX task started.");

    for (;;)
    {
        /*
         * 唤醒来源：
         * 1. USBSend() 写入新数据；
         * 2. USB_CDC_TxCpltHook() 完成一包发送；
         * 3. 20 ms 超时兜底。
        */
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20U));

        USBTransmitHandler(instance);
    }
}

/* ========================== 私有处理函数 ========================== */

/**
 * @brief 尝试启动一块 USB CDC 数据发送
 */
static void USBTransmitHandler(USBInstance *instance)
{
    uint8_t ret;

    if (!USBIsConnected())
    {
        USBDiscardTxData(instance);
        return;
    }

    /* 上一块数据尚未发送完成 */
    if (instance->tx_busy)
    {
        return;
    }

    /* 没有 pending 数据时，从 RingBuffer 读取下一块 */
    if (instance->tx_pending_size == 0U)
    {
        instance->tx_pending_size = (uint16_t)RingBuffer_Pop(&instance->tx_ringbuffer, instance->tx_pending_buffer,
                                                             USB_TX_CHUNK_SIZE);

        if (instance->tx_pending_size == 0U)
        {
            return;
        }
    }

    /*
     * 先置 busy，再调用 CDC_Transmit_FS，避免发送完成中断
     * 极快到来时造成 busy 状态覆盖。
     */
    instance->tx_busy = 1U;

    ret = CDC_Transmit_FS(instance->tx_pending_buffer, instance->tx_pending_size);

    if (ret == USBD_OK)
    {
        /*
         * pending_buffer 在 USB_CDC_TxCpltHook() 前不能被覆盖，
         * pending_size 也只在发送完成钩子中清零。
         */
        return;
    }

    instance->tx_busy = 0U;

    if (ret == USBD_BUSY)
    {
        /*
         * 保留 pending 数据，等待下一次任务通知或超时后重试。
         */
        return;
    }

    /* USBD_FAIL：丢弃当前 pending 数据 */
    RTTWARNING("[bsp_usb] CDC transmit failed.");

    USBAddDroppedBytes(instance, instance->tx_pending_size);
    instance->tx_pending_size = 0U;

    if (instance->event_callback != NULL)
    {
        instance->event_callback(instance, USB_EVENT_ERROR, NULL, 0U);
    }
}

/**
 * @brief USB 断开时丢弃所有未发送数据
 */
static void USBDiscardTxData(USBInstance *instance)
{
    uint8_t  discard_buffer[64];
    uint16_t discard_size;
    uint32_t dropped = 0U;

    if (instance->tx_pending_size > 0U)
    {
        dropped += instance->tx_pending_size;
        instance->tx_pending_size = 0U;
    }

    instance->tx_busy = 0U;

    do
    {
        discard_size = (uint16_t)RingBuffer_Pop(&instance->tx_ringbuffer, discard_buffer, sizeof(discard_buffer));

        dropped += discard_size;
    } while (discard_size > 0U);

    if (dropped > 0U)
    {
        USBAddDroppedBytes(instance, dropped);
    }
}

/**
 * @brief 原子地累加丢弃字节统计
 */
static void USBAddDroppedBytes(USBInstance *instance, uint32_t size)
{
    taskENTER_CRITICAL();
    instance->dropped_tx_bytes += size;
    taskEXIT_CRITICAL();
}

/* ========================== USB 中间件钩子 ========================== */

/**
 * @brief CDC 接收完成钩子
 *
 * @note 通常运行在 USB 中断上下文。
 */
void USB_CDC_RxHook(uint8_t *buf, uint32_t size)
{
    USBInstance *instance = &usb_instance;

    if (!instance->initialized || buf == NULL || size == 0U)
    {
        return;
    }

    if (size > UINT16_MAX)
    {
        size = UINT16_MAX;
    }

    if (instance->event_callback != NULL)
    {
        instance->event_callback(instance, USB_EVENT_RX_CPLT, buf, (uint16_t)size);
    }
}

/**
 * @brief CDC 发送完成钩子
 *
 * @note 通常运行在 USB 中断上下文。
 */
void USB_CDC_TxCpltHook(void)
{
    USBInstance *instance                   = &usb_instance;
    BaseType_t   higher_priority_task_woken = pdFALSE;
    uint16_t     completed_size;

    if (!instance->initialized)
    {
        return;
    }

    completed_size = instance->tx_pending_size;

    instance->tx_pending_size = 0U;
    instance->tx_busy         = 0U;
    instance->total_tx_bytes += completed_size;

    if (instance->event_callback != NULL)
    {
        instance->event_callback(instance, USB_EVENT_TX_CPLT, NULL, completed_size);
    }

    if (instance->tx_task_handle != NULL)
    {
        vTaskNotifyGiveFromISR(instance->tx_task_handle, &higher_priority_task_woken);

        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}
