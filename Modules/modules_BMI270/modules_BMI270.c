//
// Created by Administrator on 2026/6/14.
//

#include "modules_BMI270.h"
#include "bsp_timestamp.h"
#include "modules_BMI270_config_file.h"
#include "modules_BMI270_reg.h"
#include "string.h"
#include "bsp_memory_section.h"
#include "user_math.h"

/* ===================== 全局唯一实例 ===================== */
static BMI270_Instance_t bmi270;

/*
 * DMA 专属内存，强制分配到非 Cache 段
 * 两块 rx_buf 轮流使用，tx_buf 固定不变
 */
static uint8_t bmi270_dma_tx_buf[BMI270_DMA_BUF_SIZE] DMA_BUFFER;
static uint8_t bmi270_dma_rx_buf[2][BMI270_DMA_BUF_SIZE] DMA_BUFFER;

/*
 * BurstWrite 静态缓冲区
 * 原来放在栈上（258 字节），初始化阶段有栈溢出风险
 * 改为静态分配，仅在 BMI270_BurstWrite 内部使用
 */
static uint8_t g_burst_buf[258];

/* ===================== 私有函数声明 ===================== */
static HAL_StatusTypeDef BMI270_WriteReg(uint8_t reg, uint8_t val);
static HAL_StatusTypeDef BMI270_ReadReg(uint8_t reg, uint8_t *val);
static HAL_StatusTypeDef BMI270_BurstWrite(uint8_t reg, const uint8_t *data, uint16_t len);
static BMI270_Status_e   BMI270_UploadConfigFile(void);
static void              BMI270_DMA_Callback(SPIInstance *ins, SPI_Event_e event);
static void              BMI270_ParseData(const uint8_t *buf, BMI270_Data_t *out_data);

/* ===================== 阻塞式底层读写 ===================== */

/**
 * @brief BMI270 写单个寄存器（阻塞）
 *
 * SPI 写协议：[reg & 0x7F, value]
 *
 * 优化：原来用 SPITransRecv（全双工），接收的 rx[2] 根本没用到。
 *       改为 SPITransmit，语义更清晰，少占一份栈空间。
 */
static HAL_StatusTypeDef BMI270_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = {reg & 0x7F, val};

    SPI_TXRX_MODE_e saved = bmi270.spi->spi_work_mode;
    SPISetMode(bmi270.spi, SPI_BLOCK_MODE);

    HAL_StatusTypeDef ret = SPITransmit(bmi270.spi, tx, 2);

    SPISetMode(bmi270.spi, saved);
    return ret;
}

/**
 * @brief BMI270 读单个寄存器（阻塞）
 *
 * SPI 读协议：发 [reg|0x80, dummy, dummy]
 *             收 [x, x, value]
 * BMI270 读操作第二个返回字节才是有效数据（第一个是 dummy）
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
 *
 * 用于上传配置文件，每次写一个数据块。
 * 协议：[reg & 0x7F, data[0], data[1], ...]
 *
 * 优化：原来在栈上分配 258 字节临时缓冲区，有栈溢出风险。
 *       改为使用文件级静态缓冲区 g_burst_buf。
 *       此函数只在初始化阶段串行调用，无并发问题。
 */
static HAL_StatusTypeDef BMI270_BurstWrite(uint8_t reg, const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0)
    {
        return HAL_ERROR;
    }

    uint16_t total = len + 1; // 1 字节寄存器地址 + len 字节数据

    if (total > sizeof(g_burst_buf))
    {
        return HAL_ERROR;
    }

    SPI_TXRX_MODE_e saved = bmi270.spi->spi_work_mode;
    SPISetMode(bmi270.spi, SPI_BLOCK_MODE);

    g_burst_buf[0] = reg & 0x7F;
    memcpy(&g_burst_buf[1], data, len);

    HAL_StatusTypeDef ret = SPITransmit(bmi270.spi, g_burst_buf, total);

    SPISetMode(bmi270.spi, saved);
    return ret;
}

/* ===================== 配置文件上传 ===================== */

/**
 * @brief 上传 BMI270 配置文件（约 8KB）
 *
 * BMI270 必须在初始化阶段上传配置文件才能正常工作。
 *
 * 流程：
 * 1. 写 INIT_CTRL = 0x00（通知芯片准备接收）
 * 2. 分块写入 INIT_DATA (0x5E)，每块前先设置 INIT_ADDR_0/1
 * 3. 写 INIT_CTRL = 0x01（通知芯片上传完成）
 * 4. 等待 >= 140ms
 * 5. 读 INTERNAL_STATUS 校验
 *
 * INIT_ADDR 单位是 word（2 字节），每写 256 字节地址增加 128。
 */
static BMI270_Status_e BMI270_UploadConfigFile(void)
{
    /* 1. 准备接收 */
    BMI270_WriteReg(BMI270_INIT_CTRL_REG, 0x00);
    Bsp_Delay_ms(1);

    /* 2. 分块上传 */
    uint16_t offset    = 0;
    uint16_t remaining = bmi270_config_file_size;
    uint16_t addr_word = 0;

    while (remaining > 0)
    {
        uint16_t chunk = (remaining > 256) ? 256 : remaining;

        /* 设置写入地址（INIT_ADDR 以 word 为单位）*/
        BMI270_WriteReg(BMI270_INIT_ADDR_0_REG, (uint8_t)(addr_word & 0x0F));
        BMI270_WriteReg(BMI270_INIT_ADDR_1_REG, (uint8_t)((addr_word >> 4) & 0xFF));

        /* 写入一块数据 */
        HAL_StatusTypeDef ret = BMI270_BurstWrite(BMI270_INIT_DATA_REG, &bmi270_config_file[offset], chunk);

        if (ret != HAL_OK)
        {
            return BMI270_ERR_CONFIG;
        }

        offset += chunk;
        remaining -= chunk;

        /*
         * addr_word 单位是 word（2 字节），chunk 一定是偶数（256 或末块对齐）
         * 用右移代替除法，语义更清晰
         */
        addr_word += (chunk >> 1);
    }

    /* 3. 通知完成 */
    BMI270_WriteReg(BMI270_INIT_CTRL_REG, 0x01);

    /* 4. 等待内部初始化 */
    Bsp_Delay_ms(150);

    /* 5. 校验 */
    uint8_t status = 0;
    BMI270_ReadReg(BMI270_INTERNAL_STATUS_REG, &status);

    if ((status & BMI270_INTERNAL_STATUS_MSG_MASK) != BMI270_INTERNAL_STATUS_INIT_OK)
    {
        return BMI270_ERR_CONFIG;
    }

    return BMI270_OK;
}

/* ===================== DMA 回调（优化版）===================== */

/**
 * @brief DMA 读取完成回调（优化版）
 *
 * 与原版的区别：
 *
 * 原版：
 *   回调里做 memcpy(rx_buf[x] → raw_rx_buf)，
 *   memcpy 完成后才置 dma_state = IDLE，
 *   导致双缓冲并行优势无法发挥。
 *
 * 优化版：
 *   ① 回调里只做两件事：
 *      - 在序列锁保护下发布 latest_completed_idx 和 capture_timestamp
 *      - 立刻置 dma_state = IDLE
 *   ② 序列锁保护的内容极少（两个赋值），执行极快
 *   ③ dma_state 在 SPI 传输完成后立刻释放，不再等 memcpy
 *   ④ 真正的数据复制和解析移到主循环 BMI270_GetData 里进行
 *
 * 为什么 dma_state 可以提前释放：
 *   此时 buf_write_idx 还没切换（切换在下一次 DRDY_Handler 里做），
 *   所以下一次 DMA 如果来了，会写 buf_write_idx 指向的另一块缓冲区，
 *   不会影响 completed_idx 那块——双缓冲的天然隔离保证了这一点。
 */
static void BMI270_DMA_Callback(SPIInstance *ins, SPI_Event_e event)
{
    (void)ins;

    if (event == SPI_EVENT_TXRX_CPLT)
    {
        /*
         * 此时 buf_write_idx 是"本次 DMA 写入的索引"
         * （因为 DRDY_Handler 里先记录 write_idx，再启动 DMA，还没切换）
         * 所以 completed_idx == buf_write_idx
         */
        uint8_t completed_idx = bmi270.buf_write_idx;

        /*
         * 序列锁保护：
         * 原版保护的是"从 DMA 缓冲区 memcpy 到 raw_rx_buf"这段耗时操作
         * 优化版只保护两个轻量赋值，执行时间极短（< 10 ns 量级）
         * 这样主循环即使恰好撞上，重试代价也极低
         *
         * 被保护的内容：
         * - latest_completed_idx（让主循环知道读哪块 rx_buf）
         * - capture_timestamp（让时间戳和 idx 严格对应同一帧）
         */
        SeqLock_WriteBegin(&bmi270.data_lock);
        bmi270.latest_completed_idx = completed_idx;
        bmi270.capture_timestamp    = bmi270.capture_timestamp; // 已在 DRDY_Handler 写入，此处显式纳入保护
        SeqLock_WriteEnd(&bmi270.data_lock);

        /* 切换缓冲区，下次 DRDY 写另一块 */
        bmi270.buf_write_idx ^= 1;

        bmi270.dma_cplt_count++;
    }
    else if (event == SPI_EVENT_ERROR)
    {
        bmi270.dma_error_count++;
    }

    /*
     * 立刻释放 DMA 状态
     * 原版在 memcpy 之后才释放，优化版传输完成即释放
     * 使得双缓冲可以真正并行：
     *   - 本帧 idx=0 已完成，主循环可以从 rx_buf[0] 取数据
     *   - 下一帧 DRDY 来了，DMA 立刻可以写 rx_buf[1]
     *   - 两者操作不同内存，互不干扰
     */
    bmi270.dma_state = BMI270_DMA_IDLE;
}

/* ===================== 数据解析 ===================== */

/**
 * @brief 解析 DMA 接收缓冲区
 *
 * 缓冲区布局（17 字节）：
 *
 * Index  含义
 * ─────────────────────────────
 * [0]    dummy（SPI 地址回显）
 * [1]    dummy（BMI270 SPI 读 dummy）
 * [2]    ACC_X_LSB
 * [3]    ACC_X_MSB
 * [4]    ACC_Y_LSB
 * [5]    ACC_Y_MSB
 * [6]    ACC_Z_LSB
 * [7]    ACC_Z_MSB
 * [8]    GYR_X_LSB
 * [9]    GYR_X_MSB
 * [10]   GYR_Y_LSB
 * [11]   GYR_Y_MSB
 * [12]   GYR_Z_LSB
 * [13]   GYR_Z_MSB
 * [14]   SENSORTIME_0
 * [15]   SENSORTIME_1
 * [16]   SENSORTIME_2
 *
 * @note 此函数是纯函数，不访问任何全局状态，可在任意上下文安全调用
 */
static void BMI270_ParseData(const uint8_t *buf, BMI270_Data_t *out_data)
{
    const uint8_t *d = &buf[2]; // 跳过 2 字节 dummy

    /* 加速度计（小端，有符号）*/
    out_data->accel_raw[0] = (int16_t)((uint16_t)d[1] << 8 | d[0]);
    out_data->accel_raw[1] = (int16_t)((uint16_t)d[3] << 8 | d[2]);
    out_data->accel_raw[2] = (int16_t)((uint16_t)d[5] << 8 | d[4]);

    out_data->accel[IMU_X] = out_data->accel_raw[0] * bmi270.accel_sensitivity;
    out_data->accel[IMU_Y] = -out_data->accel_raw[1] * bmi270.accel_sensitivity;
    out_data->accel[IMU_Z] = -out_data->accel_raw[2] * bmi270.accel_sensitivity;

    /* 陀螺仪（小端，有符号）*/
    out_data->gyro_raw[0] = (int16_t)((uint16_t)d[7] << 8 | d[6]);
    out_data->gyro_raw[1] = (int16_t)((uint16_t)d[9] << 8 | d[8]);
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
    /*
     * 热复位时外部传感器可能仍在产生 DRDY。显式置为未初始化，防止 EXTI
     * 在配置文件上传和寄存器阻塞访问期间启动 SPI3 DMA。
     */
    bmi270.init_status = BMI270_NOT_INITIALIZED;

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

    /* ---- 4. 关闭高级省电模式 ---- */
    BMI270_WriteReg(BMI270_PWR_CONF_REG, BMI270_PWR_CONF_ADV_PWR_SAVE_OFF);
    Bsp_Delay_ms(1);

    /* ---- 5. 上传配置文件 ---- */
    BMI270_Status_e status = BMI270_UploadConfigFile();
    if (status != BMI270_OK)
    {
        bmi270.init_status = status;
        return status;
    }

    /* ---- 6. 使能加速度计 + 陀螺仪 + 温度 ---- */
    BMI270_WriteReg(BMI270_PWR_CTRL_REG, BMI270_PWR_CTRL_ALL_EN);
    Bsp_Delay_ms(1);

    /* ---- 7. 再次确认关闭高级省电模式 ---- */
    BMI270_WriteReg(BMI270_PWR_CONF_REG, BMI270_PWR_CONF_ADV_PWR_SAVE_OFF);
    Bsp_Delay_ms(1);

    /* ---- 8. 配置加速度计 ---- */
    BMI270_WriteReg(BMI270_ACC_CONF_REG, BMI270_ACC_ODR_800HZ | BMI270_ACC_BWP_NORMAL | BMI270_ACC_FILTER_HP);
    Bsp_Delay_ms(1);

    BMI270_WriteReg(BMI270_ACC_RANGE_REG, BMI270_ACC_RANGE_8G);
    Bsp_Delay_ms(1);

    /* ---- 9. 配置陀螺仪 ---- */
    BMI270_WriteReg(BMI270_GYR_CONF_REG,
                    BMI270_GYR_ODR_800HZ | BMI270_GYR_BWP_NORMAL | BMI270_GYR_FILTER_HP | BMI270_GYR_NOISE_HP);
    Bsp_Delay_ms(1);

    BMI270_WriteReg(BMI270_GYR_RANGE_REG, BMI270_GYR_RANGE_2000DPS);
    Bsp_Delay_ms(1);

    /* ---- 10. 配置 DRDY 中断 ---- */
    BMI270_WriteReg(BMI270_INT1_IO_CTRL_REG, BMI270_INT1_OD_PP | BMI270_INT1_LVL_HIGH | BMI270_INT1_OUTPUT_EN);
    Bsp_Delay_ms(1);

    BMI270_WriteReg(BMI270_INT_MAP_DATA_REG, BMI270_INT_MAP_DRDY_INT1);
    Bsp_Delay_ms(1);

    /* ---- 11. 计算灵敏度系数 ---- */
    bmi270.accel_sensitivity = 9.80665f / 4096.0f;                // ±8g
    bmi270.gyro_sensitivity  = (2000.0f / 32768.0f) * DEG_TO_RAD; // ±2000 dps

    /* ---- 12. 预填充 DMA TX 缓冲区 ---- */
    memset(bmi270.tx_buf, 0xFF, BMI270_DMA_BUF_SIZE);
    bmi270.tx_buf[0] = BMI270_ACC_X_LSB_REG | BMI270_SPI_READ;

    /* ---- 13. 初始化双缓冲和序列锁 ---- */
    bmi270.buf_write_idx        = 0;
    bmi270.latest_completed_idx = 0;
    bmi270.dma_state            = BMI270_DMA_IDLE;
    SeqLock_Init(&bmi270.data_lock);

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
 * DRDY 上升沿 → 检查 DMA 空闲 → 记录时间戳
 * → 启动 DMA 全双工读取 → 完成后进入 BMI270_DMA_Callback
 */
void BMI270_DRDY_Handler(void)
{
    if (bmi270.init_status != BMI270_OK)
    {
        return;
    }

    bmi270.drdy_count++;

    /* 丢帧保护：上一次 DMA 未完成则跳过 */
    if (bmi270.dma_state != BMI270_DMA_IDLE)
    {
        return;
    }

    /*
     * 记录时间戳
     * 注意：时间戳在 DMA 启动前记录，和本次 DRDY 严格对应
     * 在 DMA_Callback 里通过序列锁和 latest_completed_idx 一起发布，
     * 保证主循环拿到的时间戳和数据帧严格匹配
     */
    bmi270.capture_timestamp = Bsp_Timestamp_us_Get();
    bmi270.dma_state         = BMI270_DMA_BUSY;

    /* 本次 DMA 写入当前 buf_write_idx 指向的缓冲区 */
    uint8_t write_idx = bmi270.buf_write_idx;

    HAL_StatusTypeDef ret = SPITransRecv(bmi270.spi, bmi270.tx_buf, bmi270.rx_buf[write_idx], BMI270_DMA_BUF_SIZE);

    if (ret != HAL_OK)
    {
        /* 启动失败，恢复空闲 */
        bmi270.dma_state = BMI270_DMA_IDLE;
        bmi270.dma_error_count++;
    }
}

/**
 * @brief 获取最新 IMU 数据（序列锁保护，优化版）
 *
 * 与原版的区别：
 *
 * 原版：
 *   序列锁保护区间内直接调用 BMI270_ParseData（含浮点运算），
 *   重试代价高；且 capture_timestamp 不在锁保护范围内，
 *   可能出现时间戳与数据帧不匹配。
 *
 * 优化版：
 *   ① 序列锁只保护"读取 idx 和 timestamp"这两个轻量操作
 *   ② 拿到一致的 idx 和 timestamp 后，退出锁保护
 *   ③ 再从 rx_buf[idx] 复制一份本地副本 local_buf
 *   ④ 最后在锁外做浮点解析，重试代价低，解析不受干扰
 *
 * 为什么 memcpy(rx_buf[idx]) 不需要序列锁保护：
 *   - 双缓冲保证：下一次 DMA 写 buf_write_idx，不写 completed_idx
 *   - 再下一次才会轮回，而 17 字节 memcpy 远小于一个采样周期 1.25ms
 *   - 所以这段 memcpy 期间缓冲区一定是稳定的
 *
 * @param  data 输出指针（不可为 NULL）
 * @retval 1 = 成功，0 = 无数据或超过最大重试次数
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
     * 序列锁读取流程（优化版）：
     *
     * 只在锁保护内读取两个轻量变量：
     * - latest_completed_idx（知道该读哪块缓冲区）
     * - capture_timestamp（与本帧严格对应的时间戳）
     *
     * 浮点解析放在锁外，不参与重试。
     */
    uint8_t  idx = 0;
    uint64_t ts  = 0;

    for (uint8_t retry = 0; retry < BMI270_SEQLOCK_MAX_RETRY; retry++)
    {
        uint32_t seq = SeqLock_ReadBegin(&bmi270.data_lock);

        /* 只读两个轻量变量，执行时间极短 */
        idx = bmi270.latest_completed_idx;
        ts  = bmi270.capture_timestamp;

        if (!SeqLock_ReadRetry(&bmi270.data_lock, seq))
        {
            /* idx 和 ts 是同一帧的，一致性已确认 */
            goto parse;
        }
        /* 被打断，重试 */
    }

    /* 超过最大重试次数（极少发生）*/
    return 0;

parse:
    /*
     * 此时已退出序列锁保护区
     *
     * 从 DMA 缓冲区复制一份本地副本
     * 无需序列锁保护，原因见函数注释
     */
    uint8_t local_buf[BMI270_DMA_BUF_SIZE];
    memcpy(local_buf, bmi270.rx_buf[idx], BMI270_DMA_BUF_SIZE);

    /* 解析数据（浮点运算，完全脱离中断上下文）*/
    BMI270_ParseData(local_buf, data);

    /* 填入与本帧严格对应的时间戳 */
    data->Bim270_Timestamp = ts;

    return 1;
}
