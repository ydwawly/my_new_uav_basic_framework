//
// Created by Administrator on 2026/6/14.
//

#include "modules_BMI088.h"

#include <string.h>

#include "bsp_timestamp.h"
#include "memory_section.h"
#include "user_math.h"

/* ===================== 全局唯一实例 ===================== */
static BMI088_Instance_t bmi088;
/* 在 .c 文件内部定义静态的 DMA 专属内存，并强制分配到非 Cache 段 */
static uint8_t bmi088_gyro_dma_tx_buf[BMI088_GYRO_DMA_BUF_SIZE] DMA_BUFFER;
static uint8_t bmi088_gyro_dma_rx_buf[2][BMI088_GYRO_DMA_BUF_SIZE] DMA_BUFFER;
static uint8_t bmi088_accel_dma_tx_buf[BMI088_ACCEL_DMA_BUF_SIZE] DMA_BUFFER;
static uint8_t bmi088_accel_dma_rx_buf[2][BMI088_ACCEL_DMA_BUF_SIZE] DMA_BUFFER;
/* ===================== 私有函数声明 ===================== */

/* 阻塞式读写（初始化阶段使用）*/
static HAL_StatusTypeDef BMI088_Accel_WriteReg(uint8_t reg, uint8_t val);
static HAL_StatusTypeDef BMI088_Accel_ReadReg(uint8_t reg, uint8_t *val);
static HAL_StatusTypeDef BMI088_Gyro_WriteReg(uint8_t reg, uint8_t val);
static HAL_StatusTypeDef BMI088_Gyro_ReadReg(uint8_t reg, uint8_t *val);

/* 初始化子流程 */
static BMI088_Status_e BMI088_Accel_Init(void);
static BMI088_Status_e BMI088_Gyro_Init(void);

/* DMA 完成回调 */
static void BMI088_Gyro_DMA_Callback(SPIInstance *ins, SPI_Event_e event);
static void BMI088_Accel_DMA_Callback(SPIInstance *ins, SPI_Event_e event);

/* 数据解析 */
static void BMI088_ParseGyro(const uint8_t *buf);
static void BMI088_ParseAccel(const uint8_t *buf);

/* ===================== 阻塞式底层读写 ===================== */

/**
 * @brief 加速度计写寄存器（阻塞）
 *        加速度计 SPI 协议：发送 [reg_addr, value]
 */
static HAL_StatusTypeDef BMI088_Accel_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = {reg & 0x7F, val}; // 最高位 0 = 写
    uint8_t rx[2];

    /* 临时切阻塞模式 */
    SPI_TXRX_MODE_e saved_mode = bmi088.spi_accel->spi_work_mode;
    SPISetMode(bmi088.spi_accel, SPI_BLOCK_MODE);

    HAL_StatusTypeDef ret = SPITransRecv(bmi088.spi_accel, tx, rx, 2);

    SPISetMode(bmi088.spi_accel, saved_mode);
    return ret;
}

/**
 * @brief 加速度计读寄存器（阻塞）
 *        加速度计 SPI 读取协议：发送 [reg|0x80, dummy]，
 *        返回 [dummy, dummy, value]
 *        注意：加速度计读操作返回的第一个字节是 dummy，第二个字节才是数据
 *              所以需要发 3 字节才能读 1 字节数据
 */
static HAL_StatusTypeDef BMI088_Accel_ReadReg(uint8_t reg, uint8_t *val)
{
    uint8_t tx[3] = {reg | BMI088_SPI_READ, 0xFF, 0xFF};
    uint8_t rx[3] = {0};

    SPI_TXRX_MODE_e saved_mode = bmi088.spi_accel->spi_work_mode;
    SPISetMode(bmi088.spi_accel, SPI_BLOCK_MODE);

    HAL_StatusTypeDef ret = SPITransRecv(bmi088.spi_accel, tx, rx, 3);

    SPISetMode(bmi088.spi_accel, saved_mode);

    if (ret == HAL_OK)
    {
        *val = rx[2]; // 加速度计：跳过 1 字节 dummy
    }
    return ret;
}

/**
 * @brief 陀螺仪写寄存器（阻塞）
 *        陀螺仪 SPI 协议：发送 [reg_addr, value]
 */
static HAL_StatusTypeDef BMI088_Gyro_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = {reg & 0x7F, val};
    uint8_t rx[2];

    SPI_TXRX_MODE_e saved_mode = bmi088.spi_gyro->spi_work_mode;
    SPISetMode(bmi088.spi_gyro, SPI_BLOCK_MODE);

    HAL_StatusTypeDef ret = SPITransRecv(bmi088.spi_gyro, tx, rx, 2);

    SPISetMode(bmi088.spi_gyro, saved_mode);
    return ret;
}

/**
 * @brief 陀螺仪读寄存器（阻塞）
 *        陀螺仪 SPI 读取协议：发送 [reg|0x80, dummy]，返回 [dummy, value]
 *        注意：陀螺仪没有额外 dummy 字节，和加速度计不同
 */
static HAL_StatusTypeDef BMI088_Gyro_ReadReg(uint8_t reg, uint8_t *val)
{
    uint8_t tx[2] = {reg | BMI088_SPI_READ, 0xFF};
    uint8_t rx[2] = {0};

    SPI_TXRX_MODE_e saved_mode = bmi088.spi_gyro->spi_work_mode;
    SPISetMode(bmi088.spi_gyro, SPI_BLOCK_MODE);

    HAL_StatusTypeDef ret = SPITransRecv(bmi088.spi_gyro, tx, rx, 2);

    SPISetMode(bmi088.spi_gyro, saved_mode);

    if (ret == HAL_OK)
    {
        *val = rx[1]; // 陀螺仪：无额外 dummy
    }
    return ret;
}

/* ===================== 初始化序列 ===================== */

/**
 * @brief 加速度计初始化序列
 *
 * BMI088 加速度计上电默认在挂起模式，必须按特定顺序唤醒：
 * 1. 软复位
 * 2. 等待 >= 1ms
 * 3. 发一次 dummy read (唤醒 SPI 接口)
 * 4. 读 ID 校验
 * 5. 配置电源模式
 * 6. 配置量程/ODR
 * 7. 配置中断
 */
static BMI088_Status_e BMI088_Accel_Init(void)
{
    uint8_t chip_id = 0;

    /* 1. 软复位 */
    BMI088_Accel_WriteReg(ACC_SOFTRESET_REG, ACC_SOFTRESET_VALUE);
    Bsp_Delay_ms(50); // 复位后等待至少 1ms，保险起见用 50ms

    /* 2. Dummy read：加速度计上电/复位后需要一次 SPI 读操作来激活接口 */
    BMI088_Accel_ReadReg(ACC_CHIP_ID_REG, &chip_id);
    Bsp_Delay_ms(1);

    /* 3. 读 Chip ID 校验 */
    BMI088_Accel_ReadReg(ACC_CHIP_ID_REG, &chip_id);
    if (chip_id != ACC_CHIP_ID_VALUE)
    {
        return BMI088_ERR_ID;
    }

    /* 4. 退出挂起模式 → 正常模式 */
    BMI088_Accel_WriteReg(ACC_PWR_CONF_REG, ACC_PWR_CONF_ACTIVE);
    Bsp_Delay_ms(5);

    /* 5. 开启加速度计 */
    BMI088_Accel_WriteReg(ACC_PWR_CTRL_REG, ACC_PWR_CTRL_ON);
    Bsp_Delay_ms(50); // 开启后等待 > 50ms

    /* 6. 配置 ODR 和带宽 */
    BMI088_Accel_WriteReg(ACC_CONF_REG, ACC_CONF_ODR_800HZ);
    Bsp_Delay_ms(1);

    /* 7. 配置量程 ±6g */
    BMI088_Accel_WriteReg(ACC_RANGE_REG, ACC_RANGE_6G);
    Bsp_Delay_ms(1);

    /* 8. 配置 INT1 引脚：推挽输出，高电平有效 */
    BMI088_Accel_WriteReg(ACC_INT1_IO_CTRL_REG, ACC_INT1_IO_PP_AL);
    Bsp_Delay_ms(1);

    /* 9. 将 DRDY 映射到 INT1 */
    BMI088_Accel_WriteReg(ACC_INT1_INT2_MAP_DATA_REG, ACC_INT1_DRDY_MAP);
    Bsp_Delay_ms(1);

    /*
     * 设置灵敏度系数
     * ±6g 量程：1 LSB = 6 / 32768 * 9.80665 m/s²
     * 加速度计16位有符号，满量程 ±range_g
     */
    bmi088.accel_sensitivity = 6.0f / 32768.0f * 9.80665f;

    return BMI088_OK;
}

/**
 * @brief 陀螺仪初始化序列
 *
 * 1. 软复位
 * 2. 等待 >= 30ms
 * 3. 读 ID 校验
 * 4. 配置量程
 * 5. 配置 ODR/带宽
 * 6. 配置 DRDY 中断
 */
static BMI088_Status_e BMI088_Gyro_Init(void)
{
    uint8_t chip_id = 0;

    /* 1. 软复位 */
    BMI088_Gyro_WriteReg(GYRO_SOFTRESET_REG, GYRO_SOFTRESET_VALUE);
    Bsp_Delay_ms(80); // 陀螺仪复位后等待 >= 30ms，保险用 80ms

    /* 2. 读 Chip ID */
    BMI088_Gyro_ReadReg(GYRO_CHIP_ID_REG, &chip_id);
    if (chip_id != GYRO_CHIP_ID_VALUE)
    {
        return BMI088_ERR_ID;
    }

    /* 3. 设置量程 ±2000 dps */
    BMI088_Gyro_WriteReg(GYRO_RANGE_REG, GYRO_RANGE_2000DPS);
    Bsp_Delay_ms(1);

    /* 4. 设置 ODR 和带宽：1000Hz ODR, 116Hz BW */
    BMI088_Gyro_WriteReg(GYRO_BANDWIDTH_REG, GYRO_BW_116HZ_1000HZ_ODR);
    Bsp_Delay_ms(1);

    /* 5. 正常工作模式 */
    BMI088_Gyro_WriteReg(GYRO_LPM1_REG, GYRO_LPM1_NORMAL);
    Bsp_Delay_ms(1);

    /* 6. 使能 DRDY 中断 */
    BMI088_Gyro_WriteReg(GYRO_INT_CTRL_REG, GYRO_INT_CTRL_DRDY_EN);
    Bsp_Delay_ms(1);

    /* 7. INT3 引脚配置：推挽，高电平有效 */
    BMI088_Gyro_WriteReg(GYRO_INT3_INT4_IO_CONF_REG, GYRO_INT3_IO_PP_AL);
    Bsp_Delay_ms(1);

    /* 8. 将 DRDY 映射到 INT3（对应 GYRO_DR 引脚 PC15）*/
    BMI088_Gyro_WriteReg(GYRO_INT3_INT4_IO_MAP_REG, GYRO_INT3_DRDY_MAP);
    Bsp_Delay_ms(1);

    /*
     * 设置灵敏度系数
     * ±2000 dps 量程：1 LSB = 2000 / 32768 deg/s
     */
    bmi088.gyro_sensitivity = ( 2000.0f / 32768.0f) *  DEG_TO_RAD;

    return BMI088_OK;
}

/* ===================== DMA 回调 ===================== */

/**
 * @brief 陀螺仪 DMA 读取完成回调
 *
 * 工作流程：
 *   GYRO DRDY (EXTI) → 启动 DMA 读陀螺仪 → 完成回调 → 切换缓冲区
 *                                                     → 级联启动 DMA 读加速度计
 *
 * 这样一次 GYRO DRDY 中断就能读完 6 轴数据，最大化利用 DMA 带宽
 */
static void BMI088_Gyro_DMA_Callback(SPIInstance *ins, SPI_Event_e event)
{
    (void)ins;

    if (event == SPI_EVENT_TXRX_CPLT)
    {
        /* 切换双缓冲区索引（下次 DMA 写入另一个缓冲区）*/
        bmi088.gyro_buf_idx ^= 1;

        /* 置位更新标志 */
        bmi088.data.gyro_update_flag = 1;

        /*
         * 级联读取加速度计
         * 陀螺仪读完后，立刻启动加速度计 DMA 读取
         * 这保证了两个传感器的数据时间差最小
         */
        bmi088.dma_state = BMI088_DMA_ACCEL_BUSY;

        uint8_t write_idx = bmi088.accel_buf_idx;

        HAL_StatusTypeDef ret = SPITransRecv(
            bmi088.spi_accel,
            bmi088.accel_tx_buf,
            bmi088.accel_rx_buf[write_idx],
            BMI088_ACCEL_DMA_BUF_SIZE);

        if (ret != HAL_OK)
        {
            /* 启动失败，标记空闲，下一个 DRDY 周期重试 */
            bmi088.dma_state = BMI088_DMA_IDLE;
        }
    }
    else if (event == SPI_EVENT_ERROR)
    {
        /* SPI 错误恢复 */
        bmi088.dma_state = BMI088_DMA_IDLE;
    }
}

/**
 * @brief 加速度计 DMA 读取完成回调（由陀螺仪回调级联触发）
 */
static void BMI088_Accel_DMA_Callback(SPIInstance *ins, SPI_Event_e event)
{
    (void)ins;

    if (event == SPI_EVENT_TXRX_CPLT)
    {
        /* 切换双缓冲区 */
        bmi088.accel_buf_idx ^= 1;

        /* 置位更新标志 */
        bmi088.data.accel_update_flag = 1;
    }

    /* 无论成功还是失败，DMA 状态机都回到空闲，允许下次 DRDY 触发 */
    bmi088.dma_state = BMI088_DMA_IDLE;
}

/* ===================== 数据解析 ===================== */

/**
 * @brief 解析陀螺仪 DMA 缓冲区
 *
 * 缓冲区布局（7 字节）：
 *   [0]       = dummy（SPI 发送地址时的返回值，丢弃）
 *   [1][2]    = GYRO_X (LSB, MSB)
 *   [3][4]    = GYRO_Y (LSB, MSB)
 *   [5][6]    = GYRO_Z (LSB, MSB)
 */
static void BMI088_ParseGyro(const uint8_t *buf)
{
    bmi088.data.gyro_raw[0] = (int16_t)((uint16_t)buf[2] << 8 | buf[1]);
    bmi088.data.gyro_raw[1] = (int16_t)((uint16_t)buf[4] << 8 | buf[3]);
    bmi088.data.gyro_raw[2] = (int16_t)((uint16_t)buf[6] << 8 | buf[5]);

    bmi088.data.gyro[IMU_Y] = -bmi088.data.gyro_raw[0] * bmi088.gyro_sensitivity;
    bmi088.data.gyro[IMU_X] = -bmi088.data.gyro_raw[1] * bmi088.gyro_sensitivity;
    bmi088.data.gyro[IMU_Z] = -bmi088.data.gyro_raw[2] * bmi088.gyro_sensitivity;
}

/**
 * @brief 解析加速度计 DMA 缓冲区
 *
 * 缓冲区布局（13 字节）：
 *   [0]       = dummy（SPI 地址字节返回）
 *   [1]       = dummy（加速度计额外 dummy 字节）
 *   [2][3]    = ACC_X (LSB, MSB)
 *   [4][5]    = ACC_Y (LSB, MSB)
 *   [6][7]    = ACC_Z (LSB, MSB)
 *   [8][9][10]= SENSORTIME (byte0, byte1, byte2)
 *   [11][12]  = TEMP (MSB, LSB)  -- 从 TEMP_MSB=0x22 开始读
 *
 * 注意：这里的布局取决于 accel_tx_buf 发出的起始寄存器地址
 *       我们从 ACC_X_LSB(0x12) 开始连续读 12 字节：
 *       0x12~0x17 (6B accel) + 0x18~0x1A (3B sensortime) ...
 *       温度寄存器不连续(0x22)，所以在这个实现中我们只读加速度+时间
 *       温度可以单独低频读取
 *
 * 简化版：只读 6 字节加速度数据
 *   实际 tx_buf 长度 = 1(addr) + 1(dummy) + 6(data) = 8
 */
static void BMI088_ParseAccel(const uint8_t *buf)
{
    /* 跳过 buf[0]=地址回显, buf[1]=dummy */
    bmi088.data.accel_raw[0] = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
    bmi088.data.accel_raw[1] = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);
    bmi088.data.accel_raw[2] = (int16_t)((uint16_t)buf[7] << 8 | buf[6]);

    bmi088.data.accel[IMU_Y] = -bmi088.data.accel_raw[0] * bmi088.accel_sensitivity;
    bmi088.data.accel[IMU_X] = -bmi088.data.accel_raw[1] * bmi088.accel_sensitivity;
    bmi088.data.accel[IMU_Z] = -bmi088.data.accel_raw[2] * bmi088.accel_sensitivity;
}

/* ===================== 对外接口实现 ===================== */

/**
 * @brief BMI088 初始化
 */
BMI088_Status_e BMI088_Init(void)
{
    memset(&bmi088, 0, sizeof(BMI088_Instance_t));
    bmi088.gyro_tx_buf = bmi088_gyro_dma_tx_buf;
    bmi088.gyro_rx_buf = bmi088_gyro_dma_rx_buf;
    bmi088.accel_tx_buf = bmi088_accel_dma_tx_buf;
    bmi088.accel_rx_buf = bmi088_accel_dma_rx_buf;
    /* ---- 注册两个 SPI 实例（同一 SPI 总线，不同 CS）---- */

    SPI_Init_Config_s gyro_conf = {
        .spi_handle   = BMI088_SPI_HANDLE,
        .GPIOx        = BMI088_GYRO_CS_PORT,
        .cs_pin       = BMI088_GYRO_CS_PIN,
        .spi_work_mode = SPI_DMA_MODE,
        .callback     = BMI088_Gyro_DMA_Callback,
        .id           = &bmi088,
    };
    bmi088.spi_gyro = SPIRegister(&gyro_conf);

    SPI_Init_Config_s accel_conf = {
        .spi_handle   = BMI088_SPI_HANDLE,
        .GPIOx        = BMI088_ACCEL_CS_PORT,
        .cs_pin       = BMI088_ACCEL_CS_PIN,
        .spi_work_mode = SPI_DMA_MODE,
        .callback     = BMI088_Accel_DMA_Callback,
        .id           = &bmi088,
    };
    bmi088.spi_accel = SPIRegister(&accel_conf);

    if (bmi088.spi_gyro == NULL || bmi088.spi_accel == NULL)
    {
        bmi088.init_status = BMI088_ERR_SPI;
        return BMI088_ERR_SPI;
    }

    /* ---- 初始化加速度计 ---- */
    BMI088_Status_e status = BMI088_Accel_Init();
    if (status != BMI088_OK)
    {
        bmi088.init_status = status;
        return status;
    }

    /* ---- 初始化陀螺仪 ---- */
    status = BMI088_Gyro_Init();
    if (status != BMI088_OK)
    {
        bmi088.init_status = status;
        return status;
    }

    /* ---- 预填充 DMA TX 缓冲区（只需初始化一次）---- */

    /*
     * 陀螺仪 TX：从 GYRO_X_LSB (0x02) 开始，连续读 6 字节
     * 总共发 7 字节：[addr|0x80, 0xFF x6]
     */
    memset(bmi088.gyro_tx_buf, 0xFF, BMI088_GYRO_DMA_BUF_SIZE);
    bmi088.gyro_tx_buf[0] = GYRO_X_LSB_REG | BMI088_SPI_READ;

    /*
     * 加速度计 TX：从 ACC_X_LSB (0x12) 开始
     * 加速度计 SPI 读取有 1 字节 dummy，所以：
     *   实际读出：[dummy_addr, dummy_protocol, AccX_L, AccX_H, AccY_L, AccY_H, AccZ_L, AccZ_H,
     *             SensorTime0, SensorTime1, SensorTime2, ...]
     *
     * 我们读 1(addr) + 1(dummy) + 6(accel) + 3(sensortime) = 11 字节
     * 再多读 2 字节凑温度的话就是 13 字节
     * 此处先读 11 字节（不含温度），温度可以单独低频读
     */
    memset(bmi088.accel_tx_buf, 0xFF, BMI088_ACCEL_DMA_BUF_SIZE);
    bmi088.accel_tx_buf[0] = ACC_X_LSB_REG | BMI088_SPI_READ;

    /* 初始化双缓冲区索引 */
    bmi088.gyro_buf_idx  = 0;
    bmi088.accel_buf_idx = 0;
    bmi088.dma_state     = BMI088_DMA_IDLE;

    bmi088.init_status = BMI088_OK;
    return BMI088_OK;
}

/**
 * @brief 获取驱动实例指针
 */
BMI088_Instance_t *BMI088_GetInstance(void)
{
    return &bmi088;
}

/**
 * @brief GYRO DRDY 中断处理
 *
 * 调用时机：在 EXTI 中断回调中，当 PC15 触发时调用此函数
 *
 * 工作流程：
 *   1. 检查 DMA 状态机是否空闲
 *   2. 如果空闲，启动陀螺仪 DMA 读取
 *   3. 陀螺仪 DMA 完成后，在回调中级联启动加速度计 DMA 读取
 *   4. 加速度计 DMA 完成后，状态机回到空闲
 *
 *   整个流程在中断/DMA 上下文中自动完成，主循环只需检查 update_flag
 */
void BMI088_GYRO_DRDY_Handler(void)
{
    /* 如果上一次 DMA 还没完成，跳过本次（丢帧保护）*/
    if (bmi088.dma_state != BMI088_DMA_IDLE)
    {
        return;
    }
    bmi088.capture_timestamp = Bsp_Timestamp_us_Get();
    bmi088.dma_state = BMI088_DMA_GYRO_BUSY;

    uint8_t write_idx = bmi088.gyro_buf_idx;

    HAL_StatusTypeDef ret = SPITransRecv(
        bmi088.spi_gyro,
        bmi088.gyro_tx_buf,
        bmi088.gyro_rx_buf[write_idx],
        BMI088_GYRO_DMA_BUF_SIZE);

    if (ret != HAL_OK)
    {
        /* 启动失败，恢复空闲，等下一个 DRDY */
        bmi088.dma_state = BMI088_DMA_IDLE;
    }
}

/**
 * @brief ACCEL DRDY 中断处理（独立使用，可选）
 *        如果不使用级联模式（即不在 GYRO 完成后自动读 ACCEL），
 *        可以独立响应加速度计的 DRDY 中断
 *
 * 默认架构下不需要调用此函数，因为加速度计由陀螺仪 DMA 完成回调级联触发
 */
void BMI088_ACCEL_DRDY_Handler(void)
{
    /* 独立模式下的加速度计读取 */
    if (bmi088.dma_state != BMI088_DMA_IDLE)
    {
        return;
    }

    bmi088.dma_state = BMI088_DMA_ACCEL_BUSY;

    uint8_t write_idx = bmi088.accel_buf_idx;

    HAL_StatusTypeDef ret = SPITransRecv(
        bmi088.spi_accel,
        bmi088.accel_tx_buf,
        bmi088.accel_rx_buf[write_idx],
        BMI088_ACCEL_DMA_BUF_SIZE);

    if (ret != HAL_OK)
    {
        bmi088.dma_state = BMI088_DMA_IDLE;
    }
}

/**
 * @brief 获取最新 IMU 数据
 *
 * 由主循环或 RTOS 任务周期性调用
 * 从"上一次 DMA 写入完成的缓冲区"（即非当前写入索引）读取数据并解析
 *
 * @param  data  输出数据，可以为 NULL（此时仅内部更新）
 * @retval 1 = 有新数据，0 = 无更新
 */
uint8_t BMI088_GetData(BMI088_Data_t *data)
{
    uint8_t updated = 0;

    /* 检查陀螺仪是否有新数据 */
    if (bmi088.data.gyro_update_flag)
    {
        bmi088.data.gyro_update_flag = 0;
        bmi088.data.Bim088_Timestamp = bmi088.capture_timestamp;
        /*
         * 双缓冲区逻辑：
         * gyro_buf_idx 指向 "下一次 DMA 将要写入" 的缓冲区
         * 所以 "已经写完的" 是 gyro_buf_idx ^ 1
         */
        uint8_t read_idx = bmi088.gyro_buf_idx ^ 1;
        BMI088_ParseGyro(bmi088.gyro_rx_buf[read_idx]);
        updated = 1;
    }

    /* 检查加速度计是否有新数据 */
    if (bmi088.data.accel_update_flag)
    {
        bmi088.data.accel_update_flag = 0;

        uint8_t read_idx = bmi088.accel_buf_idx ^ 1;
        BMI088_ParseAccel(bmi088.accel_rx_buf[read_idx]);
        updated = 1;
    }

    /* 输出数据 */
    if (data != NULL && updated)
    {
        memcpy(data, &bmi088.data, sizeof(BMI088_Data_t));
    }

    return updated;
}