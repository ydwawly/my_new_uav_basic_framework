//
// Created by Administrator on 2026/6/14.
//

#ifndef MY_NEW_UAV_BAICE_FRAMEWORK_MODULES_BMI270_H
#define MY_NEW_UAV_BAICE_FRAMEWORK_MODULES_BMI270_H

#include "bsp_spi.h"
#include "stdint.h"

/* ===================== 硬件引脚配置 ===================== */

/*
 * BMI270 引脚配置（根据你的硬件表）：
 *
 * SPI3:
 *   MOSI  = PD6
 *   MISO  = PB4
 *   SCLK  = PB3
 *
 * CS:
 *   BMI270_CS = PA15 (统一片选)
 *
 * DRDY (EXTI):
 *   BMI270_DR = PB7  (数据就绪中断)
 */

#define BMI270_SPI_HANDLE       &hspi3

#define BMI270_CS_PORT          GPIOA
#define BMI270_CS_PIN           GPIO_PIN_15

#define BMI270_DR_PORT          GPIOB
#define BMI270_DR_PIN           GPIO_PIN_7

/* ===================== 序列锁 (SeqLock) ===================== */

/**
 * @brief 序列锁结构体
 *
 * 设计原理：
 *   - 写者（中断上下文）每次写入数据前 sequence++，写完后再 sequence++
 *   - 读者（主循环）读取前记录 seq_before，读取后记录 seq_after
 *   - 如果 seq_before == seq_after 且为偶数，说明读取期间没有被打断，数据一致
 *   - 如果不一致，读者重试
 *
 * 优势：
 *   - 写者零等待（中断不会被阻塞）
 *   - 读者可能重试，但不会拿到"半新半旧"的脏数据
 *   - 不需要关中断，不影响中断响应延迟
 *   - 比互斥锁轻量得多，适合"一写多读"场景
 *
 * 适用条件：
 *   - 只有一个写者（中断），可以有多个读者（主循环/多任务）
 *   - 写者不会被抢占（中断优先级最高 or 不会嵌套）
 */
typedef struct
{
    volatile uint32_t sequence; // 序列号：偶数=稳定态，奇数=正在写入
} SeqLock_t;

/* 序列锁操作宏 */

/** 写者：开始写入前调用 */
static inline void SeqLock_WriteBegin(SeqLock_t *lock)
{
    lock->sequence++;
    __DMB(); // 数据内存屏障，确保 sequence 的更新对读者可见
}

/** 写者：写入完成后调用 */
static inline void SeqLock_WriteEnd(SeqLock_t *lock)
{
    __DMB();
    lock->sequence++;
}

/** 读者：读取前调用，返回当前序列号 */
static inline uint32_t SeqLock_ReadBegin(const SeqLock_t *lock)
{
    uint32_t seq;
    do {
        seq = lock->sequence;
        __DMB();
    } while (seq & 1); // 如果是奇数，说明写者正在写，自旋等待
    return seq;
}

/** 读者：读取后调用，检查数据是否一致 */
static inline uint8_t SeqLock_ReadRetry(const SeqLock_t *lock, uint32_t start_seq)
{
    __DMB();
    return (lock->sequence != start_seq); // 不一致则需要重试
}

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
     *   TX: [addr|0x80, dummy, 0xFF x12] = 14 字节
     *   RX: [dummy_addr, dummy_proto, AccX_L, AccX_H, AccY_L, AccY_H, AccZ_L, AccZ_H,
     *        GyrX_L, GyrX_H, GyrY_L, GyrY_H, GyrZ_L, GyrZ_H] = 14 字节
     *
     * 如果还要读 sensortime (3B)：14 + 3 = 17
     * 但 sensortime 地址不连续 (0x18~0x1A vs 数据 0x0C~0x17)
     * 实际上 0x17 后面就是 0x18，所以是连续的！
     *
     * 完整布局：
     *   TX: [addr|0x80, dummy, 0xFF x15] = 17 字节
     *   RX: [dummy, dummy, AccX_L..AccZ_H(6B), GyrX_L..GyrZ_H(6B),
     *        STime0, STime1, STime2] = 17 字节
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
 *         包括：SPI 注册、芯片复位、ID 校验、上传配置文件、
 *               寄存器配置、DRDY 中断使能
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
 *         如果读取期间被中断写入打断，会自动重试（最多 3 次）
 */
uint8_t BMI270_GetData(BMI270_Data_t *data);

/**
 * @brief  DRDY 外部中断回调
 *         应在 EXTI 回调中调用（PB7 中断触发时）
 *         此函数会启动 DMA 读取 6 轴数据
 */
void BMI270_DRDY_Handler(void);

#endif //MY_NEW_UAV_BAICE_FRAMEWORK_MODULES_BMI270_H