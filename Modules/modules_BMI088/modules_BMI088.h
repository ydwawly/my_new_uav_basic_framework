//
// Created by Administrator on 2026/6/14.
//

#ifndef MY_NEW_UAV_BAICE_FRAMEWORK_MODULES_BMI088_H
#define MY_NEW_UAV_BAICE_FRAMEWORK_MODULES_BMI088_H

#include "bsp_spi.h"
#include "modules_BMI088_reg.h"
#include "stdint.h"

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

#define BMI088_SPI_HANDLE       &hspi2

#define BMI088_GYRO_CS_PORT     GPIOD
#define BMI088_GYRO_CS_PIN      GPIO_PIN_5

#define BMI088_ACCEL_CS_PORT    GPIOD
#define BMI088_ACCEL_CS_PIN     GPIO_PIN_4

#define BMI088_GYRO_DR_PORT     GPIOC
#define BMI088_GYRO_DR_PIN      GPIO_PIN_15

#define BMI088_ACCEL_DR_PORT    GPIOC
#define BMI088_ACCEL_DR_PIN     GPIO_PIN_14

/* 定义 DMA 缓冲区大小 */
#define BMI088_GYRO_DMA_BUF_SIZE   7
#define BMI088_ACCEL_DMA_BUF_SIZE  13
/* ===================== 数据结构 ===================== */

/**
 * @brief BMI088 驱动状态枚举
 */
typedef enum
{
    BMI088_OK       = 0x00,
    BMI088_ERR_SPI  = 0x01,  // SPI 通信失败
    BMI088_ERR_ID   = 0x02,  // 芯片 ID 校验失败
    BMI088_ERR_INIT = 0x04,  // 初始化序列失败
    BMI088_ERR_SELF_TEST = 0x08,
} BMI088_Status_e;

/**
 * @brief IMU 原始数据（6轴 + 温度 + 时间戳）
 */
typedef struct
{
    /* 加速度计原始值（单位：LSB） */
    int16_t accel_raw[3];    // [0]=X, [1]=Y, [2]=Z

    /* 陀螺仪原始值（单位：LSB） */
    int16_t gyro_raw[3];     // [0]=X, [1]=Y, [2]=Z

    /* 温度原始值 */
    int16_t temp_raw;

    /* 物理量（单位：m/s², deg/s, ℃）*/
    float accel[3];          // m/s²
    float gyro[3];           // deg/s (或 rad/s，看你的需求)
    float temperature;       // ℃

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
    BMI088_DMA_IDLE = 0,     // 空闲
    BMI088_DMA_GYRO_BUSY,    // 正在读陀螺仪
    BMI088_DMA_ACCEL_BUSY,   // 正在读加速度计
} BMI088_DMA_State_e;

/**
 * @brief BMI088 驱动实例
 */
typedef struct
{
    /* SPI 实例 */
    SPIInstance *spi_gyro;         // 陀螺仪 SPI
    SPIInstance *spi_accel;        // 加速度计 SPI

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
    uint8_t gyro_buf_idx;          // 当前写入缓冲区索引
    uint8_t *accel_tx_buf;
    uint8_t (*accel_rx_buf)[BMI088_ACCEL_DMA_BUF_SIZE];
    uint8_t accel_buf_idx;          // 当前写入缓冲区索引
    /* 解析后的数据 */
    BMI088_Data_t data;
    uint64_t capture_timestamp;

    /* 量程灵敏度 */
    float accel_sensitivity;       // LSB → m/s² 的系数
    float gyro_sensitivity;        // LSB → deg/s 的系数

    /* 初始化状态 */
    BMI088_Status_e init_status;
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



#endif //MY_NEW_UAV_BAICE_FRAMEWORK_MODULES_BMI088_H