/** @file modules_BMI088.h @brief BMI088 六轴 IMU 驱动接口。 */

#ifndef STM32H743_UAV_FLIGHT_CONTROLLER_MODULES_BMI088_H
#define STM32H743_UAV_FLIGHT_CONTROLLER_MODULES_BMI088_H

#include "bsp_spi.h"
#include "modules_BMI088_reg.h"
#include "stdint.h"
#include "FreeRTOS.h"
#include "task.h"
/* ===================== 用户配置区 ===================== */

/*
 * BMI088 引脚配置（根据你的硬件表）
 *
 * SPI2:
 *   MOSI  = PC3
 *   MISO  = PC2
 *   SCLK  = PD3
 *
 * CS:
 *   GYRO_CS  = PD5  (陀螺仪片选)
 *   ACCEL_CS = PD4  (加速度计片选)
 *
 * DRDY (EXTI):
 *   GYRO_DR  = PC15 (陀螺仪数据就绪中断)
 *   ACCEL_DR = PC14 (加速度计数据就绪中断)
 */

#define BMI088_SPI_HANDLE &hspi2

#define BMI088_GYRO_CS_PORT GPIOD
#define BMI088_GYRO_CS_PIN  GPIO_PIN_5

#define BMI088_ACCEL_CS_PORT GPIOD
#define BMI088_ACCEL_CS_PIN  GPIO_PIN_4

#define BMI088_GYRO_DR_PORT GPIOC
#define BMI088_GYRO_DR_PIN  GPIO_PIN_15

#define BMI088_ACCEL_DR_PORT GPIOC
#define BMI088_ACCEL_DR_PIN  GPIO_PIN_14

/* 当前飞控采样配置保持不变：陀螺仪 1 kHz、加速度计 800 Hz。 */
#define BMI088_GYRO_OUTPUT_RATE_HZ      1000U
#define BMI088_ACCEL_OUTPUT_RATE_HZ     800U
#define BMI088_STABILITY_SAMPLE_DIVIDER 8U
#define BMI088_TIMING_WARMUP_SAMPLES    1000U
#define BMI088_TIMING_WINDOW_SAMPLES    30000U
#define BMI088_EXPECTED_INTERVAL_US     (1000000U / BMI088_GYRO_OUTPUT_RATE_HZ)
#define BMI088_INTERVAL_TOLERANCE_US    100U

/* DMA 缓冲区大小 */
#define BMI088_GYRO_DMA_BUF_SIZE 7
/* 1B 地址 + 1B dummy + 0x12..0x23 共 18B，温度随六轴 DMA 原子采集。 */
#define BMI088_ACCEL_DMA_BUF_SIZE 20

/* ===================== 数据结构 ===================== */
/**
 * @brief BMI088 驱动状态枚举
 */
typedef enum
{
    BMI088_OK              = 0x00,
    BMI088_ERR_SPI         = 0x01, // SPI 通信失败
    BMI088_ERR_ID          = 0x02, // 芯片 ID 校验失败
    BMI088_ERR_INIT        = 0x04, // 初始化序列失败
    BMI088_ERR_SELF_TEST   = 0x08,
    BMI088_NOT_INITIALIZED = 0x80, // 初始化期间禁止 DRDY 中断启动 DMA
} BMI088_Status_e;

/**
 * @brief IMU 原始数据（6轴 + 温度 + 时间戳）
 */
typedef struct
{
    /* 加速度计原始值（单位：LSB） */
    int16_t accel_raw[3]; // [0]=X, [1]=Y, [2]=Z

    /* 陀螺仪原始值（单位：LSB） */
    int16_t gyro_raw[3]; // [0]=X, [1]=Y, [2]=Z

    /* 温度原始值 */
    int16_t temp_raw;
    uint8_t temperature_valid;

    /* 加速度计 24 位 sensor-time，仅低 24 位有效。 */
    uint32_t accel_sensor_time;

    /* FRD/SI 映射后、LM 固定参数校准前的物理量，仅供标定日志使用。 */
    float accel_uncalibrated[3];
    float gyro_uncalibrated[3];

    /* LM 固定参数校准后的物理量（单位：m/s²、rad/s、℃）。 */
    float accel[3];    // m/s²
    float gyro[3];     /* 三轴角速度，单位 rad/s */
    float temperature; // ℃

    uint64_t Bim088_Timestamp;

    /* 数据更新标志位（由中断回调置位，主循环读取后清零）*/
    volatile uint8_t gyro_update_flag;
    volatile uint8_t accel_update_flag;
} BMI088_Data_t;

/**
 * @brief BMI088 DMA 状态机状态
 */
typedef enum
{
    BMI088_DMA_IDLE = 0,   // 空闲
    BMI088_DMA_GYRO_BUSY,  // 正在读陀螺仪
    BMI088_DMA_ACCEL_BUSY, // 正在读加速度计
} BMI088_DMA_State_e;

/**
 * @brief BMI088 运行稳定性统计
 *
 * 中断/DMA 计数用于判断最高 ODR 下是否发生总线拥塞或丢帧。均值与 M2
 * 使用 Welford 算法低频更新，方差可按 M2/(sample_count-1) 计算。
 */
typedef struct
{
    volatile uint32_t gyro_drdy_count;              /* 陀螺仪 DRDY 中断总数 */
    volatile uint32_t dma_complete_count;           /* 完成一套六轴 DMA 的样本数 */
    volatile uint32_t dma_busy_drop_count;          /* DRDY 到达时上一套 DMA 尚未完成的次数 */
    volatile uint32_t dma_start_error_count;        /* SPI DMA 启动失败次数 */
    volatile uint32_t gyro_dma_error_count;         /* 陀螺仪 DMA 传输错误次数 */
    volatile uint32_t accel_dma_error_count;        /* 加速度计 DMA 传输错误次数 */
    volatile uint32_t temperature_valid_read_count; /* 有效温度寄存器读取次数 */
    volatile uint32_t temperature_invalid_count;    /* 0x80 无效温度状态次数 */
    volatile uint32_t data_read_count;              /* 姿态任务实际取走的样本数 */
    volatile uint32_t basepri_active_entry_count;   /* DRDY 在 FreeRTOS BASEPRI 临界区内抢占执行的次数 */
    volatile uint32_t max_entry_basepri;            /* DRDY 入口观察到的最大 BASEPRI 原始寄存器值 */

    uint64_t last_drdy_timestamp_us; /* 上一个 DRDY 本地时间戳，单位：μs */
    uint64_t interval_total_us;      /* 有效 DRDY 间隔累计值，单位：μs */
    uint32_t interval_sample_count;  /* DRDY 间隔样本数量 */
    uint32_t interval_min_us;        /* 最短 DRDY 间隔，单位：μs */
    uint32_t interval_max_us;        /* 最长 DRDY 间隔，单位：μs */
    uint32_t interval_short_count;   /* 小于期望周期减 100 μs 的次数 */
    uint32_t interval_long_count;    /* 大于期望周期加 100 μs 的次数 */
    uint8_t  timing_window_complete; /* 固定稳态统计窗口已封存，调试器连接不会再污染结果 */

    uint32_t stability_sample_count; /* 参与静态均值/方差统计的抽取样本数 */
    float    accel_mean_mps2[3];     /* 三轴加速度在线均值，单位：m/s² */
    float    accel_m2[3];            /* 三轴加速度离均差平方和 */
    float    gyro_mean_rps[3];       /* 三轴角速度在线均值，单位：rad/s */
    float    gyro_m2[3];             /* 三轴角速度离均差平方和 */
} BMI088_RuntimeStats_t;

/**
 * @brief BMI088 驱动实例
 */
typedef struct
{
    /* SPI 实例 */
    SPIInstance *spi_gyro;  // 陀螺仪 SPI
    SPIInstance *spi_accel; // 加速度计 SPI

    /* DMA 状态机 */
    volatile BMI088_DMA_State_e dma_state;

    /*
     * DMA 双缓冲区
     *
     * 陀螺仪：1字节地址 + 6字节数据 = 7字节
     * 加速度计：1字节地址 + 1字节dummy + 6字节加速度 + 3字节时间 + 2字节温度 = 13字节
     *
     * (加速度计 SPI 读取时，第一个返回字节是 dummy，需要跳过)
     */
    // uint8_t gyro_tx_buf[7];
    // uint8_t gyro_rx_buf[2][7];     // 双缓冲
    // uint8_t gyro_buf_idx;          // 当前写入缓冲区索引
    //
    // uint8_t accel_tx_buf[13];
    // uint8_t accel_rx_buf[2][13];   // 双缓冲
    // uint8_t accel_buf_idx;         // 当前写入缓冲区索引
    uint8_t *gyro_tx_buf;
    uint8_t (*gyro_rx_buf)[BMI088_GYRO_DMA_BUF_SIZE];
    uint8_t  gyro_buf_idx; // 当前写入缓冲区索引
    uint8_t *accel_tx_buf;
    uint8_t (*accel_rx_buf)[BMI088_ACCEL_DMA_BUF_SIZE];
    uint8_t accel_buf_idx; // 当前写入缓冲区索引
    /* 解析后的数据 */
    BMI088_Data_t data;
    uint64_t      capture_timestamp;

    /* 量程灵敏度 */
    float accel_sensitivity; // LSB → m/s² 的系数
    float gyro_sensitivity;  /* 原始 LSB 到 rad/s 的换算系数 */

    uint16_t              configured_gyro_rate_hz;  /* 寄存器回读确认的陀螺仪 ODR */
    uint16_t              configured_accel_rate_hz; /* 寄存器回读确认的加速度计 ODR */
    BMI088_RuntimeStats_t runtime_stats;            /* 最高 ODR 稳定性诊断数据 */

    /* 初始化状态 */
    volatile BMI088_Status_e init_status;
} BMI088_Instance_t;

/* ===================== 对外接口 ===================== */

/**
 * @brief  初始化 BMI088
 *         包括 SPI 注册、芯片复位、ID 校验、寄存器配置、中断使能
 *
 * @retval BMI088_OK        初始化成功
 * @retval BMI088_ERR_xxx   失败原因
 */
BMI088_Status_e BMI088_Init(void);

/**
 * @brief  获取 BMI088 驱动实例指针
 *         用于外部模块读取数据或检查状态
 *
 * @retval BMI088_Instance_t* 指向全局唯一实例
 */
BMI088_Instance_t *BMI088_GetInstance(void);

/**
 * @brief  获取最新的 IMU 数据
 *         由主循环/任务周期性调用
 *         此函数会检查 update_flag，解析缓冲区并转换为物理量
 *
 * @param  data 输出数据指针
 * @retval 1=有新数据，0=无更新
 */
uint8_t BMI088_GetData(BMI088_Data_t *data);

/**
 * @brief  GYRO DRDY 外部中断回调
 *         应在 EXTI 回调中调用此函数（PC15 中断）
 *         此函数会触发 DMA 读取陀螺仪数据
 */
void BMI088_GYRO_DRDY_Handler(void);

/**
 * @brief  ACCEL DRDY 外部中断回调（可选）
 *         应在 EXTI 回调中调用此函数（PC14 中断）
 *         如果不使用加速度计独立中断，可在陀螺仪 DMA 完成后级联读取
 */
void BMI088_ACCEL_DRDY_Handler(void);

void BMI088_RegisterReadyTask(TaskHandle_t task_handle);

#endif /* STM32H743_UAV_FLIGHT_CONTROLLER_MODULES_BMI088_H */
