//
// Created by Administrator on 2026/6/14.
//

#include "modules_BMI270.h"
#include "bsp_timestamp.h"
#include "modules_BMI270_config_file.h"
#include "modules_BMI270_reg.h"
#include "string.h"
#include "memory_section.h"
#include "user_math.h"
/* ===================== 全局唯一实例 ===================== */
static BMI270_Instance_t bmi270;
/* 在 .c 文件内部定义静态的 DMA 专属内存，并强制分配到非 Cache 段 */
static uint8_t bmi270_dma_tx_buf[BMI270_DMA_BUF_SIZE] DMA_BUFFER;
static uint8_t bmi270_dma_rx_buf[2][BMI270_DMA_BUF_SIZE] DMA_BUFFER;
/* 序列锁最大读取重试次数 */
#define SEQLOCK_MAX_RETRY  3

/* ===================== 私有函数声明 ===================== */

static HAL_StatusTypeDef BMI270_WriteReg(uint8_t reg, uint8_t val);
static HAL_StatusTypeDef BMI270_ReadReg(uint8_t reg, uint8_t *val);
static HAL_StatusTypeDef BMI270_BurstWrite(uint8_t reg, const uint8_t *data, uint16_t len);
static BMI270_Status_e BMI270_UploadConfigFile(void);
static void BMI270_DMA_Callback(SPIInstance *ins, SPI_Event_e event);
static void BMI270_ParseData(const uint8_t *buf, BMI270_Data_t *out_data);

/* ===================== 阻塞式底层读写 ===================== */

/**
 * @brief BMI270 写单个寄存器（阻塞）
 *
 * SPI 写协议：[reg & 0x7F, value]
 */
static HAL_StatusTypeDef BMI270_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = {reg & 0x7F, val};
    uint8_t rx[2];

    SPI_TXRX_MODE_e saved = bmi270.spi->spi_work_mode;
    SPISetMode(bmi270.spi, SPI_BLOCK_MODE);

    HAL_StatusTypeDef ret = SPITransRecv(bmi270.spi, tx, rx, 2);

    SPISetMode(bmi270.spi, saved);
    return ret;
}

/**
 * @brief BMI270 读单个寄存器（阻塞）
 *
 * SPI 读协议：发 [reg|0x80, dummy, dummy]
 *             收 [x, x, value]
 * BMI270 读操作第一个返回字节是 dummy
 */
static HAL_StatusTypeDef BMI270_ReadReg(uint8_t reg, uint8_t *val)
{
    uint8_t tx[3] = {reg | BMI270_SPI_READ, 0xFF, 0xFF};
    uint8_t rx[3] = {0};

    SPI_TXRX_MODE_e saved = bmi270.spi->spi_work_mode;
    SPISetMode(bmi270.spi, SPI_BLOCK_MODE);

    HAL_StatusTypeDef ret = SPITransRecv(bmi270.spi, tx, rx, 3);

    SPISetMode(bmi270.spi, saved);

    if (ret == HAL_OK)
    {
        *val = rx[2]; // 跳过 1 字节 dummy
    }
    return ret;
}

/**
 * @brief BMI270 burst write（阻塞）
 *        用于上传配置文件，每次最多写一个 burst
 *
 * 协议：[reg & 0x7F, data[0], data[1], ...]
 */
static HAL_StatusTypeDef BMI270_BurstWrite(uint8_t reg, const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) return HAL_ERROR;

    SPI_TXRX_MODE_e saved = bmi270.spi->spi_work_mode;
    SPISetMode(bmi270.spi, SPI_BLOCK_MODE);

    /*
     * 手动控制 CS，因为 bsp_spi 的 SPITransmit 会自动管理 CS
     * 但 burst write 需要：CS低 → 发地址 → 发数据 → CS高
     * 用 SPITransRecv 一次性完成需要临时缓冲区
     *
     * 这里用分段发送：
     * 由于 bsp_spi 每次调用会自动拉高 CS，我们需要用一个完整的缓冲区
     * 配置文件分块上传（每块 256 字节），所以用栈上 buffer
     */

    /* 分配 burst 缓冲区：1字节地址 + len字节数据 */
    /* 配置文件分块 256B，加上地址最多 257B，栈上可以承受 */
    uint8_t burst_buf[258]; // 最大 1 + 256 + 1(余量)
    uint16_t total = len + 1;

    if (total > sizeof(burst_buf))
    {
        SPISetMode(bmi270.spi, saved);
        return HAL_ERROR;
    }

    burst_buf[0] = reg & 0x7F;
    memcpy(&burst_buf[1], data, len);

    /* 使用 SPITransmit 一次性发送 */
    /* 注：我们的 bsp_spi SPITransmit 在阻塞模式下会自动管理 CS */
    HAL_StatusTypeDef ret = SPITransmit(bmi270.spi, burst_buf, total);

    SPISetMode(bmi270.spi, saved);
    return ret;
}

/* ===================== 配置文件上传 ===================== */

/**
 * @brief 上传 BMI270 配置文件
 *
 * BMI270 必须在初始化阶段上传约 8KB 的配置文件才能正常工作
 *
 * 流程：
 *   1. 写 INIT_CTRL = 0x00（准备接收配置文件）
 *   2. 分块（每块最多 256 字节）写入 INIT_DATA (0x5E)
 *      每块前需要先设置 INIT_ADDR_0/1 指定偏移量
 *   3. 写 INIT_CTRL = 0x01（完成上传，触发内部初始化）
 *   4. 等待 >= 140ms
 *   5. 读 INTERNAL_STATUS 检查结果
 */
static BMI270_Status_e BMI270_UploadConfigFile(void)
{
    /* 1. 通知芯片准备接收配置文件 */
    BMI270_WriteReg(BMI270_INIT_CTRL_REG, 0x00);
    Bsp_Delay_ms(1);

    /*
     * 2. 分块上传
     *    每次写 256 字节（最后一块可能不足 256）
     *    INIT_ADDR 的单位是 "word"（每个 word = 2 字节）
     *    所以每写 256 字节，地址增加 128
     */
    uint16_t offset = 0;
    uint16_t remaining = bmi270_config_file_size;
    uint16_t addr_word = 0; // INIT_ADDR 的值（以 2 字节为单位）

    while (remaining > 0)
    {
        uint16_t chunk = (remaining > 256) ? 256 : remaining;

        /* 设置写入地址 */
        BMI270_WriteReg(BMI270_INIT_ADDR_0_REG, (uint8_t)(addr_word & 0x0F));
        BMI270_WriteReg(BMI270_INIT_ADDR_1_REG, (uint8_t)((addr_word >> 4) & 0xFF));

        /* Burst write 数据块 */
        HAL_StatusTypeDef ret = BMI270_BurstWrite(
            BMI270_INIT_DATA_REG,
            &bmi270_config_file[offset],
            chunk);

        if (ret != HAL_OK)
        {
            return BMI270_ERR_CONFIG;
        }

        offset    += chunk;
        remaining -= chunk;
        addr_word += (chunk / 2); // 每 2 字节 = 1 word
    }

    /* 3. 通知芯片配置文件上传完成 */
    BMI270_WriteReg(BMI270_INIT_CTRL_REG, 0x01);

    /* 4. 等待内部初始化完成 */
    Bsp_Delay_ms(150); // datasheet 要求 >= 140ms

    /* 5. 校验初始化结果 */
    uint8_t status = 0;
    BMI270_ReadReg(BMI270_INTERNAL_STATUS_REG, &status);

    if ((status & BMI270_INTERNAL_STATUS_MSG_MASK) != BMI270_INTERNAL_STATUS_INIT_OK)
    {
        return BMI270_ERR_CONFIG;
    }

    return BMI270_OK;
}

/* ===================== DMA 回调 ===================== */

/**
 * @brief DMA 读取完成回调
 *
 * 工作流程：
 *   DRDY (EXTI PB7) → 启动 DMA TransRecv → 完成后进入此回调
 *     ① 切换双缓冲区索引
 *     ② 在序列锁保护下解析数据到 bmi270.data
 *     ③ 标记 DMA 空闲
 */
static void BMI270_DMA_Callback(SPIInstance *ins, SPI_Event_e event)
{
    (void)ins;

    if (event == SPI_EVENT_TXRX_CPLT)
    {
        /* 保存刚写完的缓冲区索引 */
        uint8_t completed_idx = bmi270.buf_write_idx;

        /* 切换双缓冲区（下次 DMA 写入另一个）*/
        bmi270.buf_write_idx ^= 1;

        /* 在序列锁保护下解析数据 */
        SeqLock_WriteBegin(&bmi270.data_lock);
        {
            memcpy(bmi270.raw_rx_buf, bmi270.rx_buf[completed_idx], BMI270_DMA_BUF_SIZE);
        }
        SeqLock_WriteEnd(&bmi270.data_lock);

        bmi270.dma_cplt_count++;
    }
    else if (event == SPI_EVENT_ERROR)
    {
        bmi270.dma_error_count++;
    }

    /* 无论成功还是失败，都释放 DMA 状态 */
    bmi270.dma_state = BMI270_DMA_IDLE;
}

/* ===================== 数据解析 ===================== */

/**
 * @brief 解析 DMA 接收缓冲区
 *
 * 缓冲区布局（17 字节）：
 *
 *   Index  含义
 *   ─────────────────────────────
 *   [0]    dummy（SPI 地址回显）
 *   [1]    dummy（BMI270 SPI 读 dummy）
 *   [2]    ACC_X_LSB
 *   [3]    ACC_X_MSB
 *   [4]    ACC_Y_LSB
 *   [5]    ACC_Y_MSB
 *   [6]    ACC_Z_LSB
 *   [7]    ACC_Z_MSB
 *   [8]    GYR_X_LSB
 *   [9]    GYR_X_MSB
 *   [10]   GYR_Y_LSB
 *   [11]   GYR_Y_MSB
 *   [12]   GYR_Z_LSB
 *   [13]   GYR_Z_MSB
 *   [14]   SENSORTIME_0
 *   [15]   SENSORTIME_1
 *   [16]   SENSORTIME_2
 *
 * @note 此函数在序列锁保护内调用，不要做耗时操作
 */
/**
 * @brief 解析 DMA 接收缓冲区 (现在是纯函数，可以在中断外自由调用)
 */
static void BMI270_ParseData(const uint8_t *buf, BMI270_Data_t *out_data)
{
    const uint8_t *d = &buf[2];

    /* 加速度计 */
    out_data->accel_raw[0] = (int16_t)((uint16_t)d[1]  << 8 | d[0]);
    out_data->accel_raw[1] = (int16_t)((uint16_t)d[3]  << 8 | d[2]);
    out_data->accel_raw[2] = (int16_t)((uint16_t)d[5]  << 8 | d[4]);

    out_data->accel[IMU_X] = out_data->accel_raw[0] * bmi270.accel_sensitivity;
    out_data->accel[IMU_Y] = -out_data->accel_raw[1] * bmi270.accel_sensitivity;
    out_data->accel[IMU_Z] = -out_data->accel_raw[2] * bmi270.accel_sensitivity;

    /* 陀螺仪 */
    out_data->gyro_raw[0] = (int16_t)((uint16_t)d[7]  << 8 | d[6]);
    out_data->gyro_raw[1] = (int16_t)((uint16_t)d[9]  << 8 | d[8]);
    out_data->gyro_raw[2] = (int16_t)((uint16_t)d[11] << 8 | d[10]);

    out_data->gyro[IMU_X] = out_data->gyro_raw[0] * bmi270.gyro_sensitivity;
    out_data->gyro[IMU_Y] = -out_data->gyro_raw[1] * bmi270.gyro_sensitivity;
    out_data->gyro[IMU_Z] = -out_data->gyro_raw[2] * bmi270.gyro_sensitivity;

}
/* ===================== 对外接口 ===================== */

/**
 * @brief BMI270 初始化
 */
BMI270_Status_e BMI270_Init(void)
{
    memset(&bmi270, 0, sizeof(BMI270_Instance_t));
    /* 绑定底层 DMA 缓冲区指针 */
    bmi270.tx_buf = bmi270_dma_tx_buf;
    bmi270.rx_buf = bmi270_dma_rx_buf;
    /* ---- 注册 SPI 实例 ---- */
    SPI_Init_Config_s spi_conf = {
        .spi_handle    = BMI270_SPI_HANDLE,
        .GPIOx         = BMI270_CS_PORT,
        .cs_pin        = BMI270_CS_PIN,
        .spi_work_mode = SPI_DMA_MODE,
        .callback      = BMI270_DMA_Callback,
        .id            = &bmi270,
    };
    bmi270.spi = SPIRegister(&spi_conf);

    if (bmi270.spi == NULL)
    {
        bmi270.init_status = BMI270_ERR_SPI;
        return BMI270_ERR_SPI;
    }

    /* ---- 1. 软复位 ---- */
    BMI270_WriteReg(BMI270_CMD_REG, BMI270_CMD_SOFTRESET);
    Bsp_Delay_ms(50);

    /* ---- 2. Dummy read 唤醒 SPI 接口 ---- */
    uint8_t chip_id = 0;
    BMI270_ReadReg(BMI270_CHIP_ID_REG, &chip_id);
    Bsp_Delay_ms(1);

    /* ---- 3. 读 Chip ID 校验 ---- */
    BMI270_ReadReg(BMI270_CHIP_ID_REG, &chip_id);
    if (chip_id != BMI270_CHIP_ID_VALUE)
    {
        bmi270.init_status = BMI270_ERR_ID;
        return BMI270_ERR_ID;
    }

    /* ---- 4. 关闭高级省电模式（上传配置文件前必须关闭）---- */
    BMI270_WriteReg(BMI270_PWR_CONF_REG, BMI270_PWR_CONF_ADV_PWR_SAVE_OFF);
    Bsp_Delay_ms(1);

    /* ---- 5. 上传配置文件（BMI270 独有步骤，约 8KB）---- */
    BMI270_Status_e status = BMI270_UploadConfigFile();
    if (status != BMI270_OK)
    {
        bmi270.init_status = status;
        return status;
    }

    /* ---- 6. 使能加速度计 + 陀螺仪 + 温度 ---- */
    BMI270_WriteReg(BMI270_PWR_CTRL_REG, BMI270_PWR_CTRL_ALL_EN);
    Bsp_Delay_ms(1);

    /* ---- 7. 保持关闭高级省电模式（使能传感器后需要再确认）---- */
    BMI270_WriteReg(BMI270_PWR_CONF_REG, BMI270_PWR_CONF_ADV_PWR_SAVE_OFF);
    Bsp_Delay_ms(1);

    /* ---- 8. 配置加速度计 ---- */
    /* ODR=800Hz, BWP=Normal(OSR2), Filter=High Performance */
    BMI270_WriteReg(BMI270_ACC_CONF_REG,
                    BMI270_ACC_ODR_800HZ | BMI270_ACC_BWP_NORMAL | BMI270_ACC_FILTER_HP);
    Bsp_Delay_ms(1);

    /* 量程 ±8g */
    BMI270_WriteReg(BMI270_ACC_RANGE_REG, BMI270_ACC_RANGE_8G);
    Bsp_Delay_ms(1);

    /* ---- 9. 配置陀螺仪 ---- */
    /* ODR=800Hz, BWP=Normal(OSR2), Filter=HP, Noise=HP */
    BMI270_WriteReg(BMI270_GYR_CONF_REG,
                    BMI270_GYR_ODR_800HZ | BMI270_GYR_BWP_NORMAL |
                    BMI270_GYR_FILTER_HP | BMI270_GYR_NOISE_HP);
    Bsp_Delay_ms(1);

    /* 量程 ±2000 dps */
    BMI270_WriteReg(BMI270_GYR_RANGE_REG, BMI270_GYR_RANGE_2000DPS);
    Bsp_Delay_ms(1);

    /* ---- 10. 配置 DRDY 中断 ---- */
    /* INT1: 推挽输出，高电平有效，输出使能 */
    BMI270_WriteReg(BMI270_INT1_IO_CTRL_REG,
                    BMI270_INT1_OD_PP | BMI270_INT1_LVL_HIGH | BMI270_INT1_OUTPUT_EN);
    Bsp_Delay_ms(1);

    /* 将 DRDY 映射到 INT1 */
    BMI270_WriteReg(BMI270_INT_MAP_DATA_REG, BMI270_INT_MAP_DRDY_INT1);
    Bsp_Delay_ms(1);

    /* ---- 11. 计算灵敏度系数 ---- */

    /*
     * 加速度计灵敏度：
     *   ±2g  → 16384 LSB/g
     *   ±4g  → 8192  LSB/g
     *   ±8g  → 4096  LSB/g
     *   ±16g → 2048  LSB/g
     *
     * 这里选 ±8g：1 LSB = 1/4096 g = 9.80665/4096 m/s²
     */
    bmi270.accel_sensitivity = 9.80665f / 4096.0f;

    /*
     * 陀螺仪灵敏度：
     *   ±2000dps → 16.4 LSB/dps → 1 LSB = 2000/32768 dps
     *   ±1000dps → 32.8 LSB/dps
     *   ...
     */
    bmi270.gyro_sensitivity = (2000.0f / 32768.0f) * DEG_TO_RAD;

    /* ---- 12. 预填充 DMA TX 缓冲区 ---- */

    /*
     * 从 ACC_X_LSB (0x0C) 开始连续读：
     *   0x0C~0x17: 12B (6轴数据)
     *   0x18~0x1A: 3B  (sensortime)
     *   共 15 字节数据
     *
     * SPI 帧 = 1(addr) + 1(dummy) + 15(data) = 17 字节
     */
    memset(bmi270.tx_buf, 0xFF, sizeof(bmi270.tx_buf));
    bmi270.tx_buf[0] = BMI270_ACC_X_LSB_REG | BMI270_SPI_READ;

    /* 初始化双缓冲 */
    bmi270.buf_write_idx = 0;
    bmi270.dma_state     = BMI270_DMA_IDLE;
    bmi270.data_lock.sequence = 0;

    /* ---- 等待传感器稳定 ---- */
    Bsp_Delay_ms(50);

    bmi270.init_status = BMI270_OK;
    return BMI270_OK;
}

/**
 * @brief 获取驱动实例指针
 */
BMI270_Instance_t *BMI270_GetInstance(void)
{
    return &bmi270;
}

/**
 * @brief DRDY 中断处理（PB7 EXTI 触发时调用）
 *
 * 数据流：
 *   DRDY 上升沿 → 检查 DMA 空闲 → 启动 DMA 全双工读取
 *     → DMA 完成后自动进入 BMI270_DMA_Callback
 *       → 切换缓冲区 → 序列锁保护下解析数据 → 标记空闲
 */
void BMI270_DRDY_Handler(void)
{
    bmi270.drdy_count++;

    /* 丢帧保护：上一次 DMA 未完成则跳过 */
    if (bmi270.dma_state != BMI270_DMA_IDLE)
    {
        return;
    }
    bmi270.capture_timestamp = Bsp_Timestamp_us_Get();
    bmi270.dma_state = BMI270_DMA_BUSY;

    uint8_t write_idx = bmi270.buf_write_idx;

    HAL_StatusTypeDef ret = SPITransRecv(
        bmi270.spi,
        bmi270.tx_buf,
        bmi270.rx_buf[write_idx],
        BMI270_DMA_BUF_SIZE);

    if (ret != HAL_OK)
    {
        /* 启动失败，恢复空闲 */
        bmi270.dma_state = BMI270_DMA_IDLE;
        bmi270.dma_error_count++;
    }
}

/**
 * @brief 获取最新 IMU 数据（序列锁保护）
 *
 * 使用序列锁而非关中断的优势：
 *   - 中断写数据时零等待，不影响 DRDY 响应延迟
 *   - 读者（主循环）最多重试几次，不会拿到"半新半旧"的脏数据
 *   - 整个过程无需关中断，系统实时性不受影响
 *
 * @param  data 输出指针
 * @retval 1=成功, 0=失败（无数据或超过重试次数）
 */
uint8_t BMI270_GetData(BMI270_Data_t *data)
{
    if (data == NULL)
    {
        return 0;
    }

    /* 如果还没有任何 DMA 完成过，直接返回 */
    if (bmi270.dma_cplt_count == 0)
    {
        return 0;
    }

    /*
     * 序列锁读取流程：
     *
     *   1. 读取 sequence（如果是奇数，说明写者正在写，自旋等待）
     *   2. 拷贝数据
     *   3. 再次检查 sequence 是否变化
     *   4. 如果变化了，说明读取期间被写者打断，数据不一致，重试
     *   5. 如果没变化，数据一致，返回成功
     */
    for (uint8_t retry = 0; retry < SEQLOCK_MAX_RETRY; retry++)
    {
        uint32_t seq = SeqLock_ReadBegin(&bmi270.data_lock);

        /* 拷贝数据（结构体赋值，约 50+ 字节） */
        BMI270_ParseData(bmi270.raw_rx_buf, data);
        data->Bim270_Timestamp = bmi270.capture_timestamp;

        if (!SeqLock_ReadRetry(&bmi270.data_lock, seq))
        {
            /* 读取期间没有被写者打断，数据一致 */
            return 1;
        }
        /* 被打断了，重试 */
    }

    /* 超过最大重试次数（极少发生，说明中断频率极高或主循环太慢）*/
    return 0;
}