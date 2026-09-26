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

#define BMI270_SPI_HANDLE &hspi3

#define BMI270_CS_PORT GPIOA
#define BMI270_CS_PIN  GPIO_PIN_15

#define BMI270_DR_PORT GPIOB
#define BMI270_DR_PIN  GPIO_PIN_7

/* 序列锁最大读取重试次数 */
#define BMI270_SEQLOCK_MAX_RETRY 3

/* ===================== 数据结构 ===================== */

/**
 * @brief BMI270 驱动状态枚举
 */
typedef enum
{
    BMI270_OK              = 0x00,
    BMI270_ERR_SPI         = 0x01,
    BMI270_ERR_ID          = 0x02,
    BMI270_ERR_INIT        = 0x04,
    BMI270_ERR_CONFIG      = 0x08,
    BMI270_NOT_INITIALIZED = 0x80,
} BMI270_Status_e;

/**
 * @brief IMU 原始数据 + 物理量
 */
typedef struct
{
    /* 加速度计原始值 (LSB) */
    int16_t accel_raw[3]; // [0]=X, [1]=Y, [2]=Z

    /* 陀螺仪原始值 (LSB) */
    int16_t gyro_raw[3]; // [0]=X, [1]=Y, [2]=Z

    /* 温度原始值 */
    int16_t temp_raw;

    /* 物理量 */
    float accel[3];    // m/s²
    float gyro[3];     // rad/s
    float temperature; // ℃

    /* 时间戳（us，与本帧数据严格对应）*/
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
     * 完整布局（17 字节）：
     * TX: [addr|0x80, 0xFF x16]
     * RX: [dummy, dummy, AccX_L, AccX_H, AccY_L, AccY_H, AccZ_L, AccZ_H,
     *      GyrX_L, GyrX_H, GyrY_L, GyrY_H, GyrZ_L, GyrZ_H,
     *      STime0, STime1, STime2]
     */
#define BMI270_DMA_BUF_SIZE 17

    uint8_t *tx_buf;
    uint8_t (*rx_buf)[BMI270_DMA_BUF_SIZE]; // 指向二维数组首行的指针
    uint8_t buf_write_idx;                  // 当前 DMA 写入的缓冲区索引

    /*
     * 优化：去掉 raw_rx_buf，改用 latest_completed_idx
     * 回调只发布"最新完成的缓冲区索引"
     * 主循环从对应 rx_buf 读一份本地副本再解析
     *
     * 同时把 capture_timestamp 也纳入序列锁保护，
     * 保证主循环拿到的时间戳和当帧数据严格对应
     */
    volatile uint8_t latest_completed_idx; // 最新完成帧的缓冲区索引
    uint64_t         capture_timestamp;    // 本帧 DRDY 时间戳（受 SeqLock 保护）

    /* 序列锁：保护 latest_completed_idx + capture_timestamp 的一致性读取 */
    SeqLock_t data_lock;

    /* 量程灵敏度系数 */
    float accel_sensitivity; // LSB → m/s²
    float gyro_sensitivity;  // LSB → rad/s

    /* 计数器（调试用）*/
    volatile uint32_t drdy_count;
    volatile uint32_t dma_cplt_count;
    volatile uint32_t dma_error_count;

    /* 初始化状态 */
    volatile BMI270_Status_e init_status;

} BMI270_Instance_t;

/* ===================== 对外接口 ===================== */

/**
 * @brief  初始化 BMI270
 * @retval BMI270_OK / BMI270_ERR_xxx
 */
BMI270_Status_e BMI270_Init(void);

/**
 * @brief  获取 BMI270 驱动实例指针
 */
BMI270_Instance_t *BMI270_GetInstance(void);

/**
 * @brief  获取最新 IMU 数据（带序列锁保护）
 *
 * @param  data 输出数据指针（不可为 NULL）
 * @retval 1 = 成功，0 = 无数据或重试超限
 */
uint8_t BMI270_GetData(BMI270_Data_t *data);

/**
 * @brief  DRDY 外部中断回调（在 EXTI 回调中调用）
 */
void BMI270_DRDY_Handler(void);

#endif // MY_NEW_UAV_BAICE_FRAMEWORK_MODULES_BMI270_H
