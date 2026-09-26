/**
 * @file DataRouterTask.h
 * @brief MAVLink 发送队列、周期服务调度与黑匣子数据入口
 */

#ifndef STM32H743_UAV_FLIGHT_CONTROLLER_DATA_ROUTER_TASK_H
#define STM32H743_UAV_FLIGHT_CONTROLLER_DATA_ROUTER_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "bsp_USB.h"
#include "mavlink_user.h"
#include "modules_SD_Card.h"
#include "queue.h"
#include "task.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================== 公共配置 ========================== */

#define DATA_ROUTER_TX_SLOT_COUNT    8U   /* 最多允许 8 帧 MAVLink 同时等待发送 */
#define DATA_ROUTER_UPDATE_PERIOD_MS 1U   /* 周期服务的基础调度间隔，单位 ms */
#define DATA_ROUTER_IMU_LOG_DEPTH    256U /* 最多缓存约 256 ms 的 1 kHz BMI088 标定样本 */

#ifndef DATA_ROUTER_ENABLE_USB_IMU_STREAM
#define DATA_ROUTER_ENABLE_USB_IMU_STREAM 0U
#endif

#ifndef DATA_ROUTER_ENABLE_MAVLINK_DURING_IMU_STREAM
#define DATA_ROUTER_ENABLE_MAVLINK_DURING_IMU_STREAM 0U
#endif

#ifndef DATA_ROUTER_ENABLE_SD_MAVLINK_LOG
#define DATA_ROUTER_ENABLE_SD_MAVLINK_LOG 0U
#endif

#ifndef DATA_ROUTER_ENABLE_SD_IMU_LOG
#define DATA_ROUTER_ENABLE_SD_IMU_LOG 1U
#endif

/* ========================== 类型定义 ========================== */

/**
 * @brief 业务服务周期更新函数
 *
 * DataRouter 只提供周期执行时机，不依赖 SystemService、HilService 或
 * ParamService 的具体实现，从而保持物理发送层与上层业务解耦。
 *
 * @param now_us 当前单调时间戳，单位 us
 */
typedef void (*DataRouter_UpdateCallback_t)(uint64_t now_us);

/**
 * @brief DataRouter 初始化配置
 */
typedef struct
{
    USBInstance *usb_instance; /* USB BSP 实例，不得为 NULL */

    const DataRouter_UpdateCallback_t *update_callbacks;      /* 周期更新回调表，允许为 NULL */
    uint8_t                            update_callback_count; /* 回调表中的有效函数数量 */
} DataRouter_Config_t;

/**
 * @brief 固定大小的发送槽
 *
 * 业务层提交数据后立即复制到槽内，因此 DataRouter_PostFrame() 返回后，
 * 调用者可以释放或复用原始缓冲区。
 */
typedef struct
{
    uint64_t timestamp_us;                 /* 进入 DataRouter 的时间戳，单位 us */
    uint16_t length;                       /* data 中的有效字节数 */
    uint8_t  data[MAVLINK_MAX_PACKET_LEN]; /* 完整 MAVLink 字节流 */
} DataRouter_TxSlot_t;

typedef struct
{
    uint64_t                    timestamp_us;
    SDCard_BMI088ImuPayloadV1_t payload;
} DataRouter_ImuLogItem_t;

typedef struct
{
    volatile uint32_t imu_log_enqueued;
    volatile uint32_t imu_log_dropped;
    volatile uint32_t sd_imu_frames;
    volatile uint32_t sd_imu_dropped;
    volatile uint32_t usb_stream_frames;
    volatile uint32_t usb_stream_dropped;
    volatile uint32_t usb_stream_sessions;
    volatile uint32_t usb_header_send_failures;
    volatile uint8_t  usb_stream_active;
} DataRouter_LogStats_t;

extern volatile DataRouter_LogStats_t data_router_log_stats;

/** @brief MAVLink 发送槽池和 USB 投递路径的拥塞统计。 */
typedef struct
{
    volatile uint32_t post_attempts;          /* 业务层提交总次数 */
    volatile uint32_t posted_frames;          /* 成功进入待发送队列的帧数 */
    volatile uint32_t invalid_rejects;        /* 参数或初始化状态无效 */
    volatile uint32_t slot_exhaustion_drops;  /* 8 个静态发送槽全部占用 */
    volatile uint32_t queue_send_drops;       /* 取得槽后发送队列异常满 */
    volatile uint32_t dispatched_frames;      /* 已从槽池取出并尝试交给 USB */
    volatile uint32_t usb_enqueue_drops;      /* USB TX 环形缓冲区拒绝帧 */
    volatile uint32_t queue_high_watermark;   /* 待发送队列历史最高帧数 */
} DataRouter_TxStats_t;

extern volatile DataRouter_TxStats_t data_router_tx_stats;

/**
 * @brief DataRouter 单例运行结构
 *
 * free_queue 保存空闲槽指针，tx_queue 保存等待发送的槽指针。两个队列与全部
 * 槽均静态分配，运行期不申请堆内存。
 */
typedef struct
{
    USBInstance *usb_instance; /* 唯一物理发送接口 */

    QueueHandle_t free_queue;    /* 空闲槽指针队列 */
    QueueHandle_t tx_queue;      /* 待发送槽指针队列 */
    QueueHandle_t imu_log_queue; /* BMI088 标定日志队列 */

    StaticQueue_t free_queue_control; /* FreeRTOS 空闲队列静态控制块 */
    StaticQueue_t tx_queue_control;   /* FreeRTOS 发送队列静态控制块 */
    StaticQueue_t imu_log_queue_control;

    uint8_t free_queue_storage[DATA_ROUTER_TX_SLOT_COUNT * sizeof(DataRouter_TxSlot_t *)]; /* 空闲队列存储区 */
    uint8_t tx_queue_storage[DATA_ROUTER_TX_SLOT_COUNT * sizeof(DataRouter_TxSlot_t *)];   /* 发送队列存储区 */
    uint8_t imu_log_queue_storage[DATA_ROUTER_IMU_LOG_DEPTH * sizeof(DataRouter_ImuLogItem_t)];

    DataRouter_TxSlot_t slots[DATA_ROUTER_TX_SLOT_COUNT]; /* 固定发送槽池 */

    const DataRouter_UpdateCallback_t *update_callbacks;      /* 周期更新回调表 */
    uint8_t                            update_callback_count; /* 周期更新回调数量 */
    uint8_t                            initialized;           /* 1 表示队列和槽池初始化完成 */
} DataRouterInstance;

/* ========================== 接口声明 ========================== */

/**
 * @brief 初始化 USB 所有权、静态队列和发送槽池
 * @param config 初始化配置
 * @return true 初始化成功，false 参数错误、重复初始化或队列创建失败
 */
bool DataRouter_Init(const DataRouter_Config_t *config);

/**
 * @brief 非阻塞提交一帧待发送数据
 *
 * 函数内部立即复制数据并记录进入路由器的时间戳。没有空闲槽时直接返回 false，
 * 不允许发送拥塞反向阻塞传感器或控制任务。
 *
 * @param data   待发送字节流首地址
 * @param length 有效字节数
 * @return true 成功加入发送队列，false 参数错误或发送槽耗尽
 */
bool DataRouter_PostFrame(const uint8_t *data, uint16_t length);

/**
 * @brief 非阻塞提交 BMI088 标定日志，由 DataRouter 统一写入 SD 环形缓冲区
 */
bool DataRouter_PostBMI088Imu(uint64_t timestamp_us, const SDCard_BMI088ImuPayloadV1_t *sample);

/**
 * @brief 从 USB RX 中断输入标定握手字节；命令携带会话令牌，回调只置位请求
 */
void DataRouter_UsbCalibrationInputFromISR(const uint8_t *data, uint16_t size);

/**
 * @brief DataRouter 核心任务
 *
 * 该任务是 USB 发送接口和 SD 日志环形缓冲区的唯一生产者，同时利用队列
 * 1 ms 超时周期调用各业务服务的更新函数。
 *
 * @param argument FreeRTOS 任务参数，当前未使用
 */
void DataRouterTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* STM32H743_UAV_FLIGHT_CONTROLLER_DATA_ROUTER_TASK_H */
