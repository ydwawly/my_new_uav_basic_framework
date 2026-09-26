/**
 * @file modules_SPL06.h
 * @brief SPL06-001 气压计的 I2C2、中断和 DMA 驱动
 */

#ifndef STM32H743_UAV_FLIGHT_CONTROLLER_MODULES_SPL06_H
#define STM32H743_UAV_FLIGHT_CONTROLLER_MODULES_SPL06_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "bsp_iic.h"
#include "bsp_utils_seqlock.h"
#include "modules_Message_center.h"
#include "task.h"

/* ========================== 公共配置 ========================== */

#define SPL06_TOPIC_NAME         "spl06_data"
#define NOTIFY_BIT_SPL06         (1UL << 8)
#define SPL06_DMA_FRAME_SIZE     11U
#define SPL06_COEFFICIENT_SIZE   18U
#define SPL06_SEQLOCK_MAX_RETRY  3U
#define SPL06_I2C_ADDRESS_LOW    0x76U
#define SPL06_I2C_ADDRESS_HIGH   0x77U
#define SPL06_SEA_LEVEL_PRESSURE 101325.0f

/**
 * @brief 初始化阶段状态，便于在 VS Code Watch 中直接判断失败位置
 */
typedef enum
{
    SPL06_INIT_NOT_STARTED = 0U,
    SPL06_INIT_IIC_REGISTER_FAILED,
    SPL06_INIT_DEVICE_NOT_FOUND,
    SPL06_INIT_RESET_FAILED,
    SPL06_INIT_READY_TIMEOUT,
    SPL06_INIT_ID_INVALID,
    SPL06_INIT_COEFFICIENT_FAILED,
    SPL06_INIT_CONFIG_FAILED,
    SPL06_INIT_OK
} SPL06_InitStatus_e;

/**
 * @brief DMA 事务状态
 */
typedef enum
{
    SPL06_DMA_IDLE = 0U,
    SPL06_DMA_READING
} SPL06_DmaState_e;

/**
 * @brief SPL06 出厂补偿系数
 *
 * c0、c1 为 12 位有符号数；c00、c10 为 20 位有符号数；
 * 其余系数为 16 位有符号数。驱动解析后统一扩展到 int32_t，
 * 上层无需关心寄存器中的跨字节位拼接。
 */
typedef struct
{
    int32_t c0;
    int32_t c1;
    int32_t c00;
    int32_t c10;
    int32_t c01;
    int32_t c11;
    int32_t c20;
    int32_t c21;
    int32_t c30;
} SPL06_Coefficients_t;

/**
 * @brief 向消息中心发布的 SPL06 物理量
 */
typedef struct
{
    uint64_t timestamp_us;     /* PD0 数据就绪中断时间戳，单位 us */
    int32_t  raw_pressure;     /* 24 位有符号原始压力 ADC 值 */
    int32_t  raw_temperature;  /* 24 位有符号原始温度 ADC 值 */
    float    pressure_pa;      /* 出厂系数补偿后的气压，单位 Pa */
    float    temperature_c;    /* 出厂系数补偿后的温度，单位 ℃ */
    float    altitude_m;       /* 按标准海平面气压换算的绝对气压高度，单位 m */
    uint8_t  interrupt_status; /* INT_STS 原始值，用于排查 DRDY 来源 */
    uint8_t  device_id;        /* PROD_ID/REV_ID 原始寄存器值 */
    uint8_t  address_7bit;     /* 实际探测到的 7 位 I2C 地址 */
    uint8_t  valid;            /* 1 表示数值有限且处于传感器合理量程 */
} SPL06_Data_t;

/**
 * @brief 从上电开始采用 Welford 算法累计的在线统计量
 *
 * variance 为样本方差（除以 N-1）。它包含真实高度变化、气流与温漂，只适合
 * 判断当前静置数据是否稳定和导出离线分析，不能未经静态分段就直接写入 ESKF R。
 */
typedef struct
{
    uint32_t sample_count;
    double   pressure_mean_pa;
    double   pressure_m2_pa2;
    double   pressure_variance_pa2;
    double   temperature_mean_c;
    double   temperature_m2_c2;
    double   temperature_variance_c2;
    double   altitude_mean_m;
    double   altitude_m2_m2;
    double   altitude_variance_m2;
} SPL06_Statistics_t;

/**
 * @brief SPL06 单例运行状态
 *
 * DMA 只写 dma_rx_buffer；DMA 完成回调进行第一次拷贝，将 11 字节帧写入
 * raw_frame；SensorHub 再在序列锁保护下进行第二次拷贝并解析。该方案没有
 * 双缓冲区，同时能保证任务不会读到一半新、一半旧的数据。
 */
typedef struct
{
    IICInstance *iic;
    Publisher_t *publisher;
    SeqLock_t    data_lock;

    uint8_t *dma_rx_buffer;                   /* 位于非缓存 DMA 专用内存段 */
    uint8_t  raw_frame[SPL06_DMA_FRAME_SIZE]; /* ISR 落地的一致原始快照 */

    SPL06_Coefficients_t coefficients;
    SPL06_Data_t         latest_data; /* 最近一次通过量程检查并发布的数据 */
    SPL06_Statistics_t   statistics;  /* 有效样本在线均值和方差 */
    float                pressure_scale;
    float                temperature_scale;

    uint64_t pending_drdy_timestamp_us;
    uint64_t capture_timestamp_us;

    volatile uint32_t drdy_count;
    volatile uint32_t dma_complete_count;
    volatile uint32_t dma_busy_drop_count;
    volatile uint32_t dma_start_error_count;
    volatile uint32_t dma_error_count;
    volatile uint32_t parse_error_count;
    volatile uint32_t published_count;
    uint32_t          processed_dma_count;

    volatile uint8_t dma_state;   /* SPL06_DmaState_e */
    uint8_t          init_status; /* SPL06_InitStatus_e */
    uint8_t          device_id;
    uint8_t          address_7bit;
} SPL06_Instance_t;

/**
 * @brief 初始化 SPL06 并进入压力/温度连续测量模式
 * @return 1 初始化成功，0 初始化失败
 */
uint8_t SPL06_Init(void);

/**
 * @brief 处理一帧 DMA 原始数据并发布补偿后的物理量
 * @return 1 成功发布新数据，0 无新数据或数据无效
 */
uint8_t SPL06_Task_Handler(void);

/**
 * @brief 注册接收 SPL06 任务通知的 SensorHub 任务
 */
void SPL06_RegisterReadyTask(TaskHandle_t task_handle);

/**
 * @brief PD0 数据就绪中断入口，只启动一次 I2C2 RX DMA
 */
void SPL06_DRDY_Handler(void);

/**
 * @brief 返回 SPL06 单例，供调试器查看初始化状态和统计量
 */
const SPL06_Instance_t *SPL06_GetInstance(void);

#endif /* STM32H743_UAV_FLIGHT_CONTROLLER_MODULES_SPL06_H */
