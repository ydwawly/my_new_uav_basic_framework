/**
 * @file modules_SPL06.c
 * @brief SPL06-001 气压计初始化、DMA 采集、补偿和消息发布
 */

#include "modules_SPL06.h"

#include <math.h>
#include <string.h>

#include "bsp_RTT.h"
#include "bsp_memory_section.h"
#include "bsp_timestamp.h"
#include "main.h"
#include "user_math.h"

/* ========================== 寄存器与固定配置 ========================== */

#define SPL06_REG_PRESSURE_B2     0x00U
#define SPL06_REG_PRESSURE_CONFIG 0x06U
#define SPL06_REG_TEMP_CONFIG     0x07U
#define SPL06_REG_MEAS_CONFIG     0x08U
#define SPL06_REG_INT_CONFIG      0x09U
#define SPL06_REG_INT_STATUS      0x0AU
#define SPL06_REG_RESET           0x0CU
#define SPL06_REG_ID              0x0DU
#define SPL06_REG_COEFFICIENT     0x10U

#define SPL06_RESET_COMMAND         0x09U
#define SPL06_READY_MASK            0xC0U
#define SPL06_EXPECTED_PRODUCT_ID   0x10U
#define SPL06_PRODUCT_ID_MASK       0xF0U
#define SPL06_PRESSURE_CONFIG_VALUE 0x53U /* 32 Hz、8 倍过采样 */
#define SPL06_TEMP_CONFIG_VALUE     0xB0U /* 外部温度源、8 Hz、单次采样 */
#define SPL06_MEAS_CONFIG_VALUE     0x07U /* 后台连续测量压力和温度 */
#define SPL06_INT_ACTIVE_HIGH       0x80U
#define SPL06_INT_PRESSURE_READY    0x10U

#define SPL06_PRESSURE_SCALE_OSR8    7864320.0f
#define SPL06_TEMPERATURE_SCALE_OSR1 524288.0f
#define SPL06_RESET_DELAY_MS         20U
#define SPL06_READY_TIMEOUT_MS       100U

/* I2C2 RX DMA 不能访问 DTCM；DMA_BUFFER 将唯一 DMA 缓冲区放到非缓存 D2 SRAM。 */
static uint8_t spl06_dma_rx_buffer[SPL06_DMA_FRAME_SIZE] DMA_BUFFER;

static SPL06_Instance_t spl06_instance;
static TaskHandle_t     spl06_ready_task = NULL;

/* ========================== 私有函数声明 ========================== */

static int32_t SPL06_SignExtend(uint32_t value, uint8_t bit_count);
static int32_t SPL06_ParseSigned24(const uint8_t bytes[3]);
static void    SPL06_ParseCoefficients(const uint8_t raw[SPL06_COEFFICIENT_SIZE], SPL06_Coefficients_t *coefficients);
static HAL_StatusTypeDef SPL06_ReadRegister(uint8_t register_address, uint8_t *value);
static HAL_StatusTypeDef SPL06_WriteRegister(uint8_t register_address, uint8_t value);
static uint8_t           SPL06_DetectAddress(void);
static uint8_t           SPL06_WaitReady(void);
static void              SPL06_ConfigureInterruptPin(void);
static void              SPL06_UpdateStatistics(const SPL06_Data_t *data);
static void              SPL06_IIC_EventCallback(IICInstance *instance, IIC_Event_e event);

/* ========================== 原始数据辅助函数 ========================== */

/**
 * @brief 将任意不超过 31 位的二进制补码扩展为 int32_t
 */
static int32_t SPL06_SignExtend(uint32_t value, uint8_t bit_count)
{
    const uint32_t sign_mask  = 1UL << (bit_count - 1U);
    const uint32_t value_mask = (1UL << bit_count) - 1UL;

    value &= value_mask;
    if ((value & sign_mask) != 0U)
    {
        value |= ~value_mask;
    }
    return (int32_t)value;
}

/**
 * @brief 解析大端排列的 24 位有符号 ADC 数据
 */
static int32_t SPL06_ParseSigned24(const uint8_t bytes[3])
{
    const uint32_t raw = ((uint32_t)bytes[0] << 16U) | ((uint32_t)bytes[1] << 8U) | (uint32_t)bytes[2];
    return SPL06_SignExtend(raw, 24U);
}

/**
 * @brief 按数据手册定义解析跨字节存放的九个出厂补偿系数
 */
static void SPL06_ParseCoefficients(const uint8_t raw[SPL06_COEFFICIENT_SIZE], SPL06_Coefficients_t *coefficients)
{
    coefficients->c0 = SPL06_SignExtend(((uint32_t)raw[0] << 4U) | ((uint32_t)raw[1] >> 4U), 12U);
    coefficients->c1 = SPL06_SignExtend(((uint32_t)(raw[1] & 0x0FU) << 8U) | raw[2], 12U);

    coefficients->c00 = SPL06_SignExtend(((uint32_t)raw[3] << 12U) | ((uint32_t)raw[4] << 4U) |
                                             ((uint32_t)raw[5] >> 4U),
                                         20U);
    coefficients->c10 = SPL06_SignExtend(((uint32_t)(raw[5] & 0x0FU) << 16U) | ((uint32_t)raw[6] << 8U) | raw[7], 20U);

    coefficients->c01 = SPL06_SignExtend(((uint32_t)raw[8] << 8U) | raw[9], 16U);
    coefficients->c11 = SPL06_SignExtend(((uint32_t)raw[10] << 8U) | raw[11], 16U);
    coefficients->c20 = SPL06_SignExtend(((uint32_t)raw[12] << 8U) | raw[13], 16U);
    coefficients->c21 = SPL06_SignExtend(((uint32_t)raw[14] << 8U) | raw[15], 16U);
    coefficients->c30 = SPL06_SignExtend(((uint32_t)raw[16] << 8U) | raw[17], 16U);
}

/* ========================== 阻塞初始化访问 ========================== */

static HAL_StatusTypeDef SPL06_ReadRegister(uint8_t register_address, uint8_t *value)
{
    return IICMemRead(spl06_instance.iic, register_address, IIC_MEMORY_ADDRESS_8BIT, value, 1U);
}

static HAL_StatusTypeDef SPL06_WriteRegister(uint8_t register_address, uint8_t value)
{
    return IICMemWrite(spl06_instance.iic, register_address, IIC_MEMORY_ADDRESS_8BIT, &value, 1U);
}

/**
 * @brief 探测 0x76 和 0x77，返回实际 7 位地址
 */
static uint8_t SPL06_DetectAddress(void)
{
    static const uint8_t candidate_addresses[] = {SPL06_I2C_ADDRESS_LOW, SPL06_I2C_ADDRESS_HIGH};

    for (uint32_t index = 0U; index < (sizeof(candidate_addresses) / sizeof(candidate_addresses[0])); index++)
    {
        if ((IICSetAddress(spl06_instance.iic, candidate_addresses[index]) == HAL_OK) &&
            (IICIsDeviceReady(spl06_instance.iic, 3U, 10U) == HAL_OK))
        {
            return candidate_addresses[index];
        }
    }
    return 0U;
}

/**
 * @brief 等待内部传感器和补偿系数均准备完成
 */
static uint8_t SPL06_WaitReady(void)
{
    for (uint32_t elapsed_ms = 0U; elapsed_ms < SPL06_READY_TIMEOUT_MS; elapsed_ms++)
    {
        uint8_t status = 0U;
        if ((SPL06_ReadRegister(SPL06_REG_MEAS_CONFIG, &status) == HAL_OK) &&
            ((status & SPL06_READY_MASK) == SPL06_READY_MASK))
        {
            return 1U;
        }
        Bsp_Delay_ms(1U);
    }
    return 0U;
}

/**
 * @brief 根据 SDO 地址绑定位配置 PD0 的有效边沿与上下拉
 *
 * SDO 下拉时地址为 0x76，数据就绪必须配置为高有效；SDO 上拉/悬空时地址
 * 为 0x77，数据就绪必须配置为低有效。这里同时修改 MCU 的 EXTI 边沿，
 * 因而两种硬件焊接方式无需修改源代码。
 */
static void SPL06_ConfigureInterruptPin(void)
{
    GPIO_InitTypeDef gpio_config = {0};

    HAL_NVIC_DisableIRQ(SPL06_INT_EXTI_IRQn);
    gpio_config.Pin = SPL06_INT_Pin;

    if (spl06_instance.address_7bit == SPL06_I2C_ADDRESS_LOW)
    {
        gpio_config.Mode = GPIO_MODE_IT_RISING;
        gpio_config.Pull = GPIO_PULLDOWN;
    }
    else
    {
        gpio_config.Mode = GPIO_MODE_IT_FALLING;
        gpio_config.Pull = GPIO_PULLUP;
    }

    HAL_GPIO_Init(SPL06_INT_GPIO_Port, &gpio_config);
    __HAL_GPIO_EXTI_CLEAR_IT(SPL06_INT_Pin);
    HAL_NVIC_SetPriority(SPL06_INT_EXTI_IRQn, 7U, 0U);
    HAL_NVIC_EnableIRQ(SPL06_INT_EXTI_IRQn);
}

/**
 * @brief 用单遍 Welford 算法更新压力、温度和高度的均值/样本方差
 */
static void SPL06_UpdateStatistics(const SPL06_Data_t *data)
{
    SPL06_Statistics_t *statistics   = &spl06_instance.statistics;
    const uint32_t      sample_count = ++statistics->sample_count;

    const double pressure_delta = (double)data->pressure_pa - statistics->pressure_mean_pa;
    statistics->pressure_mean_pa += pressure_delta / (double)sample_count;
    statistics->pressure_m2_pa2 += pressure_delta * ((double)data->pressure_pa - statistics->pressure_mean_pa);

    const double temperature_delta = (double)data->temperature_c - statistics->temperature_mean_c;
    statistics->temperature_mean_c += temperature_delta / (double)sample_count;
    statistics->temperature_m2_c2 += temperature_delta * ((double)data->temperature_c - statistics->temperature_mean_c);

    const double altitude_delta = (double)data->altitude_m - statistics->altitude_mean_m;
    statistics->altitude_mean_m += altitude_delta / (double)sample_count;
    statistics->altitude_m2_m2 += altitude_delta * ((double)data->altitude_m - statistics->altitude_mean_m);

    if (sample_count > 1U)
    {
        const double denominator            = (double)(sample_count - 1U);
        statistics->pressure_variance_pa2   = statistics->pressure_m2_pa2 / denominator;
        statistics->temperature_variance_c2 = statistics->temperature_m2_c2 / denominator;
        statistics->altitude_variance_m2    = statistics->altitude_m2_m2 / denominator;
    }
}

/* ========================== 中断和 DMA 完成链路 ========================== */

static void SPL06_IIC_EventCallback(IICInstance *instance, IIC_Event_e event)
{
    if (instance != spl06_instance.iic)
    {
        return;
    }

    if (event == IIC_EVENT_MEMORY_RX_COMPLETE)
    {
        /*
         * 第一次拷贝：DMA 专用缓冲区 -> 序列锁保护的原始帧。
         * 11 字节从 0x00 连续读到 INT_STS(0x0A)，在获得同一时刻压力/温度的
         * 同时读取并清除中断状态，保证下一次数据就绪边沿能正常产生。
         */
        SeqLock_WriteBegin(&spl06_instance.data_lock);
        memcpy(spl06_instance.raw_frame, spl06_instance.dma_rx_buffer, SPL06_DMA_FRAME_SIZE);
        spl06_instance.capture_timestamp_us = spl06_instance.pending_drdy_timestamp_us;
        spl06_instance.dma_complete_count++;
        SeqLock_WriteEnd(&spl06_instance.data_lock);

        spl06_instance.dma_state = SPL06_DMA_IDLE;

        if (spl06_ready_task != NULL)
        {
            BaseType_t higher_priority_task_woken = pdFALSE;
            xTaskNotifyFromISR(spl06_ready_task, NOTIFY_BIT_SPL06, eSetBits, &higher_priority_task_woken);
            portYIELD_FROM_ISR(higher_priority_task_woken);
        }
    }
    else if (event == IIC_EVENT_ERROR)
    {
        spl06_instance.dma_error_count++;
        spl06_instance.dma_state = SPL06_DMA_IDLE;
    }
}

void SPL06_DRDY_Handler(void)
{
    if (spl06_instance.init_status != SPL06_INIT_OK)
    {
        return;
    }

    spl06_instance.drdy_count++;
    if (spl06_instance.dma_state != SPL06_DMA_IDLE)
    {
        spl06_instance.dma_busy_drop_count++;
        return;
    }

    spl06_instance.pending_drdy_timestamp_us = Bsp_Timestamp_us_Get();
    spl06_instance.dma_state                 = SPL06_DMA_READING;

    const HAL_StatusTypeDef start_status =
        IICMemRead(spl06_instance.iic, SPL06_REG_PRESSURE_B2, IIC_MEMORY_ADDRESS_8BIT,
                   spl06_instance.dma_rx_buffer, SPL06_DMA_FRAME_SIZE);

    if (start_status != HAL_OK)
    {
        spl06_instance.dma_start_error_count++;
        spl06_instance.dma_state = SPL06_DMA_IDLE;
    }
}

/* ========================== 任务级补偿和发布 ========================== */

/**
 * @brief 通过序列锁取得一帧未撕裂的 DMA 原始数据
 */
static uint8_t SPL06_ReadConsistentFrame(uint8_t local_frame[SPL06_DMA_FRAME_SIZE], uint64_t *timestamp_us)
{
    uint32_t dma_count = 0U;

    if ((local_frame == NULL) || (timestamp_us == NULL))
    {
        return 0U;
    }

    /*
     * 第二次拷贝：序列锁保护的原始帧 -> 任务栈局部数组。
     * 若复制期间新 DMA 完成打断任务，版本号会改变，本次快照立即重试。
     */
    for (uint8_t retry = 0U; retry < SPL06_SEQLOCK_MAX_RETRY; retry++)
    {
        const uint32_t start_sequence = SeqLock_ReadBegin(&spl06_instance.data_lock);

        dma_count     = spl06_instance.dma_complete_count;
        *timestamp_us = spl06_instance.capture_timestamp_us;
        memcpy(local_frame, spl06_instance.raw_frame, SPL06_DMA_FRAME_SIZE);

        if (!SeqLock_ReadRetry(&spl06_instance.data_lock, start_sequence))
        {
            if ((dma_count == 0U) || (dma_count == spl06_instance.processed_dma_count))
            {
                return 0U;
            }
            spl06_instance.processed_dma_count = dma_count;
            return 1U;
        }
    }
    return 0U;
}

/**
 * @brief 解析原始寄存器并执行 SPL06 数据手册补偿多项式
 */
static uint8_t SPL06_CompensateFrame(const uint8_t local_frame[SPL06_DMA_FRAME_SIZE], uint64_t timestamp_us,
                                     SPL06_Data_t *data)
{
    if ((local_frame == NULL) || (data == NULL))
    {
        return 0U;
    }

    memset(data, 0, sizeof(*data));
    data->timestamp_us     = timestamp_us;
    data->raw_pressure     = SPL06_ParseSigned24(&local_frame[0]);
    data->raw_temperature  = SPL06_ParseSigned24(&local_frame[3]);
    data->interrupt_status = local_frame[10];
    data->device_id        = spl06_instance.device_id;
    data->address_7bit     = spl06_instance.address_7bit;

    const float                 pressure_scaled    = (float)data->raw_pressure / spl06_instance.pressure_scale;
    const float                 temperature_scaled = (float)data->raw_temperature / spl06_instance.temperature_scale;
    const SPL06_Coefficients_t *c                  = &spl06_instance.coefficients;

    /*
     * 数据手册补偿多项式。压力系数最高到 Psc 三次项，并包含温压交叉项；
     * 所有计算在任务上下文完成，I2C/DMA 中断中不执行浮点运算。
     */
    data->pressure_pa   = (float)c->c00 +
                          pressure_scaled *
                              ((float)c->c10 + pressure_scaled * ((float)c->c20 + pressure_scaled * (float)c->c30)) +
                          temperature_scaled * (float)c->c01 +
                          temperature_scaled * pressure_scaled * ((float)c->c11 + pressure_scaled * (float)c->c21);
    data->temperature_c = (float)c->c0 * 0.5f + (float)c->c1 * temperature_scaled;
    data->altitude_m    = Math_PressureToAltitude(data->pressure_pa, SPL06_SEA_LEVEL_PRESSURE);

    data->valid = (Math_IsFinite(data->pressure_pa) && Math_IsFinite(data->temperature_c) &&
                   Math_IsFinite(data->altitude_m) && (data->pressure_pa >= 30000.0f) &&
                   (data->pressure_pa <= 110000.0f) && (data->temperature_c >= -40.0f) &&
                   (data->temperature_c <= 85.0f))
                      ? 1U
                      : 0U;
    return data->valid;
}

uint8_t SPL06_Task_Handler(void)
{
    uint8_t      local_frame[SPL06_DMA_FRAME_SIZE];
    uint64_t     timestamp_us = 0ULL;
    SPL06_Data_t data;

    if ((spl06_instance.init_status != SPL06_INIT_OK) || (spl06_instance.publisher == NULL) ||
        (SPL06_ReadConsistentFrame(local_frame, &timestamp_us) == 0U))
    {
        return 0U;
    }

    if (SPL06_CompensateFrame(local_frame, timestamp_us, &data) == 0U)
    {
        spl06_instance.parse_error_count++;
        return 0U;
    }

    spl06_instance.latest_data = data;
    SPL06_UpdateStatistics(&data);
    (void)PubPushMessage(spl06_instance.publisher, &data);
    spl06_instance.published_count++;
    return 1U;
}

/* ========================== 初始化与调试接口 ========================== */

void SPL06_RegisterReadyTask(TaskHandle_t task_handle)
{
    spl06_ready_task = task_handle;
}

const SPL06_Instance_t *SPL06_GetInstance(void)
{
    return &spl06_instance;
}

/**
 * @brief 清空运行状态并建立 DMA 缓冲区、比例因子和序列锁
 */
static void SPL06_ResetRuntime(void)
{
    memset(&spl06_instance, 0, sizeof(spl06_instance));
    memset(spl06_dma_rx_buffer, 0, sizeof(spl06_dma_rx_buffer));
    SeqLock_Init(&spl06_instance.data_lock);

    spl06_instance.dma_rx_buffer     = spl06_dma_rx_buffer;
    spl06_instance.pressure_scale    = SPL06_PRESSURE_SCALE_OSR8;
    spl06_instance.temperature_scale = SPL06_TEMPERATURE_SCALE_OSR1;
    spl06_instance.init_status       = SPL06_INIT_NOT_STARTED;
}

/**
 * @brief 注册消息发布者和 I2C2 BSP 实例，并探测实际器件地址
 */
static uint8_t SPL06_RegisterInterfaces(void)
{
    spl06_instance.publisher = PubRegister(SPL06_TOPIC_NAME, sizeof(SPL06_Data_t));
    if (spl06_instance.publisher == NULL)
    {
        spl06_instance.init_status = SPL06_INIT_IIC_REGISTER_FAILED;
        RTTERROR("[SPL06] Publisher register failed.");
        return 0U;
    }

    IIC_Init_Config_s iic_config = {0};
    iic_config.i2c_handle        = &hi2c2;
    iic_config.address_7bit      = SPL06_I2C_ADDRESS_LOW;
    iic_config.transfer_mode     = IIC_TRANSFER_BLOCKING;
    iic_config.callback          = SPL06_IIC_EventCallback;
    iic_config.id                = &spl06_instance;

    spl06_instance.iic = IICRegister(&iic_config);
    if (spl06_instance.iic == NULL)
    {
        spl06_instance.init_status = SPL06_INIT_IIC_REGISTER_FAILED;
        RTTERROR("[SPL06] I2C2 register failed.");
        return 0U;
    }

    spl06_instance.address_7bit = SPL06_DetectAddress();
    if (spl06_instance.address_7bit == 0U)
    {
        spl06_instance.init_status = SPL06_INIT_DEVICE_NOT_FOUND;
        RTTERROR("[SPL06] Device not found at 0x76/0x77.");
        return 0U;
    }
    (void)IICSetAddress(spl06_instance.iic, spl06_instance.address_7bit);
    SPL06_ConfigureInterruptPin();
    return 1U;
}

/**
 * @brief 软复位器件、校验产品 ID 并读取出厂补偿系数
 */
static uint8_t SPL06_ResetAndReadCalibration(void)
{
    if (SPL06_WriteRegister(SPL06_REG_RESET, SPL06_RESET_COMMAND) != HAL_OK)
    {
        spl06_instance.init_status = SPL06_INIT_RESET_FAILED;
        RTTERROR("[SPL06] Soft reset failed.");
        return 0U;
    }
    Bsp_Delay_ms(SPL06_RESET_DELAY_MS);

    if (SPL06_WaitReady() == 0U)
    {
        spl06_instance.init_status = SPL06_INIT_READY_TIMEOUT;
        RTTERROR("[SPL06] Sensor/coefficient ready timeout.");
        return 0U;
    }

    if ((SPL06_ReadRegister(SPL06_REG_ID, &spl06_instance.device_id) != HAL_OK) ||
        ((spl06_instance.device_id & SPL06_PRODUCT_ID_MASK) != SPL06_EXPECTED_PRODUCT_ID))
    {
        spl06_instance.init_status = SPL06_INIT_ID_INVALID;
        RTTERROR("[SPL06] Product ID invalid: 0x%02x.", spl06_instance.device_id);
        return 0U;
    }

    uint8_t coefficient_raw[SPL06_COEFFICIENT_SIZE];
    if (IICMemRead(spl06_instance.iic, SPL06_REG_COEFFICIENT, IIC_MEMORY_ADDRESS_8BIT, coefficient_raw,
                   sizeof(coefficient_raw)) != HAL_OK)
    {
        spl06_instance.init_status = SPL06_INIT_COEFFICIENT_FAILED;
        RTTERROR("[SPL06] Coefficient read failed.");
        return 0U;
    }
    SPL06_ParseCoefficients(coefficient_raw, &spl06_instance.coefficients);
    return 1U;
}

/**
 * @brief 配置连续测量、数据就绪中断，并把 I2C 传输切换为 DMA
 */
static uint8_t SPL06_ConfigureMeasurement(void)
{
    /*
     * 8 倍压力过采样不需要 P_SHIFT；只打开压力数据就绪中断。
     * 0x76 对应 SDO 下拉/高有效，0x77 对应 SDO 上拉/低有效。
     */
    const uint8_t interrupt_config = SPL06_INT_PRESSURE_READY |
                                     ((spl06_instance.address_7bit == SPL06_I2C_ADDRESS_LOW) ? SPL06_INT_ACTIVE_HIGH
                                                                                             : 0U);
    uint8_t       ignored_interrupt_status = 0U;

    if ((SPL06_WriteRegister(SPL06_REG_PRESSURE_CONFIG, SPL06_PRESSURE_CONFIG_VALUE) != HAL_OK) ||
        (SPL06_WriteRegister(SPL06_REG_TEMP_CONFIG, SPL06_TEMP_CONFIG_VALUE) != HAL_OK) ||
        (SPL06_WriteRegister(SPL06_REG_INT_CONFIG, interrupt_config) != HAL_OK) ||
        (SPL06_ReadRegister(SPL06_REG_INT_STATUS, &ignored_interrupt_status) != HAL_OK) ||
        (SPL06_WriteRegister(SPL06_REG_MEAS_CONFIG, SPL06_MEAS_CONFIG_VALUE) != HAL_OK) ||
        (IICSetMode(spl06_instance.iic, IIC_TRANSFER_DMA) != HAL_OK))
    {
        spl06_instance.init_status = SPL06_INIT_CONFIG_FAILED;
        RTTERROR("[SPL06] Measurement configuration failed.");
        return 0U;
    }
    return 1U;
}

uint8_t SPL06_Init(void)
{
    SPL06_ResetRuntime();
    if ((SPL06_RegisterInterfaces() == 0U) || (SPL06_ResetAndReadCalibration() == 0U) ||
        (SPL06_ConfigureMeasurement() == 0U))
    {
        return 0U;
    }

    spl06_instance.dma_state   = SPL06_DMA_IDLE;
    spl06_instance.init_status = SPL06_INIT_OK;
    RTTINFO("[SPL06] Init success, address=0x%02x, id=0x%02x.", spl06_instance.address_7bit, spl06_instance.device_id);
    return 1U;
}
