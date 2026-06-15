//
// Created by Administrator on 2026/6/14.
//

#ifndef MY_NEW_UAV_BAICE_FRAMEWORK_MODULES_BMI270_H
#define MY_NEW_UAV_BAICE_FRAMEWORK_MODULES_BMI270_H

#include "bsp_spi.h"
#include "stdint.h"
#include "bsp_utils_seqlock.h"

/* ===================== 硬件引脚配置 ===================== */

/*
 * BMI270 引脚配置（根据你的硬件表）：
 *
 * SPI3:
 * MOSI  = PD6
 * MISO  = PB4
 * SCLK  = PB3
 *
 * CS:
 * BMI270_CS = PA15 (统一片选)
 *
 * DRDY (EXTI):
 * BMI270_DR = PB7  (数据就绪中断)
 */

#define BMI270_SPI_HANDLE       &hspi3

#define BMI270_CS_PORT          GPIOA
#define BMI270_CS_PIN           GPIO_PIN_15

#define BMI270_DR_PORT          GPIOB
#define BMI270_DR_PIN           GPIO_PIN_7

/* 序列锁最大读取重试次数 */
#define BMI270_SEQLOCK_MAX_RETRY  3
/* ===================== 数据结构 ===================== */

/**
 * @brief BMI270 驱动状态枚举
 */
typedef enum
{
    BMI270_OK             = 0x00,
    BMI270_ERR_SPI        = 0x01,
    BMI270_ERR_ID         = 0x02,
    BMI270_ERR_INIT       = 0x04,
    BMI270_ERR_CONFIG     = 0x08, // 配置文件上传失败
} BMI270_Status_e;

/**
 * @brief IMU 原始数据 + 物理量
 */
typedef struct
{
    /* 加速度计原始值 (LSB) */
    int16_t accel_raw[3]; // [0]=X, [1]=Y, [2]=Z

    /* 陀螺仪原始值 (LSB) */
    int16_t gyro_raw[3];  // [0]=X, [1]=Y, [2]=Z

    /* 温度原始值 */
    int16_t temp_raw;

    /* 物理量 */
    float accel[3];       // m/s²
    float gyro[3];        // deg/s
    float temperature;    // ℃

    /* 时间戳（系统 tick，用于计算 dt）*/
    uint64_t Bim270_Timestamp;
} BMI270_Data_t;

/**
 * @brief DMA 状态
 */
typedef enum
{
    BMI270_DMA_IDLE = 0,
    BMI270_DMA_BUSY,
} BMI270_DMA_State_e;

/**
 * @brief BMI270 驱动实例
 */
typedef struct
{
    /* SPI 实例 */
    SPIInstance *spi;

    /* DMA 状态 */
    volatile BMI270_DMA_State_e dma_state;

    /*
     * DMA 双缓冲区
     *
     * BMI270 一次 burst read：
     * TX: [addr|0x80, dummy, 0xFF x12] = 14 字节
     * RX: [dummy_addr, dummy_proto, AccX_L, AccX_H, AccY_L, AccY_H, AccZ_L, AccZ_H,
     * GyrX_L, GyrX_H, GyrY_L, GyrY_H, GyrZ_L, GyrZ_H] = 14 字节
     *
     * 如果还要读 sensortime (3B)：14 + 3 = 17
     * 但 sensortime 地址不连续 (0x18~0x1A vs 数据 0x0C~0x17)
     * 实际上 0x17 后面就是 0x18，所以是连续的！
     *
     * 完整布局：
     * TX: [addr|0x80, dummy, 0xFF x15] = 17 字节
     * RX: [dummy, dummy, AccX_L..AccZ_H(6B), GyrX_L..GyrZ_H(6B),
     * STime0, STime1, STime2] = 17 字节
     */
    #define BMI270_DMA_BUF_SIZE  17  // 1(addr) + 1(dummy) + 12(6轴) + 3(sensortime)

    // uint8_t tx_buf[BMI270_DMA_BUF_SIZE];
    // uint8_t rx_buf[2][BMI270_DMA_BUF_SIZE]; // 双缓冲
    uint8_t *tx_buf;
    uint8_t (*rx_buf)[BMI270_DMA_BUF_SIZE]; // 指向一维数组的指针
    uint8_t buf_write_idx;                   // DMA 当前写入的缓冲区索引
    uint8_t raw_rx_buf[BMI270_DMA_BUF_SIZE];
    /*
     * 解析后的数据（受序列锁保护）
     * 中断写，主循环读
     */
    BMI270_Data_t data;
    SeqLock_t     data_lock;
    uint64_t capture_timestamp;

    /* 量程灵敏度系数 */
    float accel_sensitivity;  // LSB → m/s²
    float gyro_sensitivity;   // LSB → deg/s

    /* DRDY 计数器（调试用）*/
    volatile uint32_t drdy_count;
    volatile uint32_t dma_cplt_count;
    volatile uint32_t dma_error_count;

    /* 初始化状态 */
    BMI270_Status_e init_status;
} BMI270_Instance_t;

/* ===================== 对外接口 ===================== */

/**
 * @brief  初始化 BMI270
 * 包括：SPI 注册、芯片复位、ID 校验、上传配置文件、
 * 寄存器配置、DRDY 中断使能
 *
 * @retval BMI270_OK        成功
 * @retval BMI270_ERR_xxx   失败原因
 */
BMI270_Status_e BMI270_Init(void);

/**
 * @brief  获取 BMI270 驱动实例指针
 * @retval 全局唯一实例
 */
BMI270_Instance_t *BMI270_GetInstance(void);

/**
 * @brief  获取最新 IMU 数据（带序列锁保护，保证数据一致性）
 *
 * @param  data 输出数据指针（不可为 NULL）
 * @retval 1 = 成功获取一致数据
 * @retval 0 = 无新数据或超过最大重试次数
 *
 * @note   此函数不阻塞、不关中断
 * 如果读取期间被中断写入打断，会自动重试（最多 3 次）
 */
uint8_t BMI270_GetData(BMI270_Data_t *data);

/**
 * @brief  DRDY 外部中断回调
 * 应在 EXTI 回调中调用（PB7 中断触发时）
 * 此函数会启动 DMA 读取 6 轴数据
 */
void BMI270_DRDY_Handler(void);

#endif //MY_NEW_UAV_BAICE_FRAMEWORK_MODULES_BMI270_H