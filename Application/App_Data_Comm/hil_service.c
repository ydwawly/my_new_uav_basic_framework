#include "hil_service.h"
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "bsp_RTT.h"
#include "bsp_timestamp.h"
#include "modules_Message_center.h"
#include "DataRouterTask.h"
#include "mavlink_user.h"

/* ========================================================= */
/*                  Private State (私有状态)                 */
/* ========================================================= */

/* HIL 消息发布者句柄 (Publisher) */
static Publisher_t *hil_sensor_pub = NULL;
static Publisher_t *hil_gps_pub    = NULL;
static Publisher_t *hil_state_pub  = NULL;

/*
 * HIL 模式激活状态
 * 注：在 STM32 (Cortex-M) 架构上，单字节(8-bit)对齐的读写本身就是原子操作。
 */
static volatile uint8_t hil_active = 0U;

/*
 * 最新执行器控制量快照缓存。
 * 策略：只保存 Latest Value (最新值)，不使用 Queue。
 * 原因：过期的旧控制量对实时仿真而言没有重发的价值。
 */
static float    actuator_controls[HIL_SERVICE_ACTUATOR_COUNT] = {0};
static uint64_t actuator_flags                                = 0U;
static uint8_t  actuator_valid                                = 0U; // 标识缓存中是否有尚未发送的有效控制量
/* 记录最近一次发送 HIL_ACTUATOR_CONTROLS 的时间戳 */
static uint64_t last_actuator_tx_us = 0U;
/* HIL 运行状态统计信息 */
static HilService_Stats_t hil_stats = {0};
/* 模块初始化完成标志 */
static uint8_t hil_initialized = 0U;
/* ========================================================= */
/*                    RX Stats (接收统计)                    */
/* ========================================================= */

/**
 * @brief 记录接收到的 HIL 数据统计信息
 *
 * 鉴于当前架构下，所有 MAVLink 消息均由单独的 MavlinkRxTask 顺序处理，
 * 属于同一任务上下文，因此此处无需加入临界区(Critical Section)保护。
 */
static void HilService_RecordRx(uint32_t message_id, uint64_t rx_timestamp_us, uint64_t sim_timestamp_us)
{
    switch (message_id)
    {
    case MAVLINK_MSG_ID_HIL_SENSOR:
        hil_stats.sensor_messages++;
        break;
    case MAVLINK_MSG_ID_HIL_GPS:
        hil_stats.gps_messages++;
        break;
    case MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
        hil_stats.state_messages++;
        break;
    default:
        break;
    }

    hil_stats.last_rx_timestamp_us  = rx_timestamp_us;
    hil_stats.last_sim_timestamp_us = sim_timestamp_us;
}

/* ========================================================= */
/*                     HIL_SENSOR 解析处理                   */
/* ========================================================= */

static void HilService_HandleSensor(const mavlink_message_t *msg)
{
    mavlink_hil_sensor_t source;
    HIL_Sensor_Data_t    data          = {0};
    const bool           first_message = (hil_stats.sensor_messages == 0U);

    /* 解码 MAVLink 消息 */
    mavlink_msg_hil_sensor_decode(msg, &source);

    /* 同步时间戳 */
    data.timestamp_us    = source.time_usec;
    data.rx_timestamp_us = Bsp_Timestamp_us_Get();

    /* 提取 IMU 数据 (加速度与角速度) */
    data.accel[0] = source.xacc;
    data.accel[1] = source.yacc;
    data.accel[2] = source.zacc;

    data.gyro[0] = source.xgyro;
    data.gyro[1] = source.ygyro;
    data.gyro[2] = source.zgyro;

    /* 提取磁力计数据 */
    data.mag[0] = source.xmag;
    data.mag[1] = source.ymag;
    data.mag[2] = source.zmag;

    /* 提取气压计与温度数据 */
    data.abs_pressure  = source.abs_pressure;
    data.diff_pressure = source.diff_pressure;
    data.pressure_alt  = source.pressure_alt;
    data.temperature   = source.temperature;

    /* 记录字段更新掩码 */
    data.fields_updated = source.fields_updated;

    /* 发布到系统内部的消息总线 (Message Center) */
    (void)PubPushMessage(hil_sensor_pub, &data);

    /* 更新 RX 统计数据 */
    HilService_RecordRx(MAVLINK_MSG_ID_HIL_SENSOR, data.rx_timestamp_us, data.timestamp_us);

    if (first_message)
    {
        RTTINFO("[HIL] HIL_SENSOR received.");
    }
}

/* ========================================================= */
/*                       HIL_GPS 解析处理                    */
/* ========================================================= */

static void HilService_HandleGps(const mavlink_message_t *msg)
{
    mavlink_hil_gps_t source;
    HIL_GPS_Data_t    data          = {0};
    const bool        first_message = (hil_stats.gps_messages == 0U);

    mavlink_msg_hil_gps_decode(msg, &source);

    /* 同步时间戳 */
    data.timestamp_us    = source.time_usec;
    data.rx_timestamp_us = Bsp_Timestamp_us_Get();

    /*
     * 位置信息单位换算：
     * MAVLink 协议中纬经度单位为 deg * 1e7
     * 原始海拔高度单位为 mm，需转换为 m
     */
    data.latitude  = (double)source.lat * 1e-7;
    data.longitude = (double)source.lon * 1e-7;
    data.altitude  = (float)source.alt * 0.001f;

    /* 定位精度单位换算 (原始 cm -> m) */
    data.eph = (float)source.eph * 0.01f;
    data.epv = (float)source.epv * 0.01f;

    /* 速度与航向单位换算 (原始 cm/s -> m/s, 原始 cdeg -> deg) */
    data.velocity        = (float)source.vel * 0.01f;
    data.velocity_ned[0] = (float)source.vn * 0.01f;
    data.velocity_ned[1] = (float)source.ve * 0.01f;
    data.velocity_ned[2] = (float)source.vd * 0.01f;
    data.course          = (float)source.cog * 0.01f;

    data.fix_type           = source.fix_type;
    data.satellites_visible = source.satellites_visible;

    /* 发布与统计 */
    (void)PubPushMessage(hil_gps_pub, &data);
    HilService_RecordRx(MAVLINK_MSG_ID_HIL_GPS, data.rx_timestamp_us, data.timestamp_us);

    if (first_message)
    {
        RTTINFO("[HIL] HIL_GPS received.");
    }
}

/* ========================================================= */
/*            HIL_STATE_QUATERNION 解析处理                   */
/* ========================================================= */

static void HilService_HandleState(const mavlink_message_t *msg)
{
    mavlink_hil_state_quaternion_t source;
    HIL_State_Data_t               data          = {0};
    const bool                     first_message = (hil_stats.state_messages == 0U);

    mavlink_msg_hil_state_quaternion_decode(msg, &source);

    data.timestamp_us    = source.time_usec;
    data.rx_timestamp_us = Bsp_Timestamp_us_Get();

    /* 提取姿态 (四元数)与角速度 */
    memcpy(data.quaternion, source.attitude_quaternion, sizeof(data.quaternion));
    data.angular_velocity[0] = source.rollspeed;
    data.angular_velocity[1] = source.pitchspeed;
    data.angular_velocity[2] = source.yawspeed;

    /* 提取位置 */
    data.latitude  = (double)source.lat * 1e-7;
    data.longitude = (double)source.lon * 1e-7;
    data.altitude  = (float)source.alt * 0.001f;

    /* 速度换算 (原始 cm/s -> m/s) */
    data.velocity_ned[0] = (float)source.vx * 0.01f;
    data.velocity_ned[1] = (float)source.vy * 0.01f;
    data.velocity_ned[2] = (float)source.vz * 0.01f;

    /* 空速换算 (原始 cm/s -> m/s) */
    data.indicated_airspeed = (float)source.ind_airspeed * 0.01f;
    data.true_airspeed      = (float)source.true_airspeed * 0.01f;

    /*
     * 加速度换算：
     * MAVLink 协议中 xacc/yacc/zacc 的单位是 mG (毫G)。
     * 重力加速度常数 1G ≈ 9.80665 m/s²。
     * 因此转换公式为：值 * (9.80665 / 1000)
     */
    data.accel[0] = (float)source.xacc * 9.80665e-3f;
    data.accel[1] = (float)source.yacc * 9.80665e-3f;
    data.accel[2] = (float)source.zacc * 9.80665e-3f;

    /* 发布与统计 */
    (void)PubPushMessage(hil_state_pub, &data);
    HilService_RecordRx(MAVLINK_MSG_ID_HIL_STATE_QUATERNION, data.rx_timestamp_us, data.timestamp_us);

    if (first_message)
    {
        RTTINFO("[HIL] HIL_STATE_QUATERNION received.");
    }
}

/* ========================================================= */
/*                  Actuator Snapshot (快照提取)             */
/* ========================================================= */

/**
 * @brief 安全地获取最新的执行器控制量快照
 *
 * 借助 FreeRTOS 临界区保护多线程并发访问。由于仅作小尺寸 memcpy 操作，
 * 临界区时间极短，不会影响系统实时性。
 */
static bool HilService_GetActuatorSnapshot(float *controls, uint64_t *flags)
{
    uint8_t valid;

    taskENTER_CRITICAL();
    memcpy(controls, actuator_controls, sizeof(actuator_controls));
    *flags = actuator_flags;
    valid  = actuator_valid;
    taskEXIT_CRITICAL();

    return (valid != 0U);
}

/* ========================================================= */
/*              HIL Actuator TX (发送执行器控制指令)           */
/* ========================================================= */

static bool HilService_SendActuatorControls(uint64_t timestamp_us)
{
    float    controls[HIL_SERVICE_ACTUATOR_COUNT];
    uint64_t flags;

    /* 如果当前没有产生有效的控制量输出，则无需发送 */
    if (!HilService_GetActuatorSnapshot(controls, &flags))
    {
        return false;
    }

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    bool    posted = false;

    if (!Mavlink_TxTransactionBegin())
    {
        return false;
    }

    /* 将控制指令打包为 MAVLink 消息 */
    const uint16_t length = Mavlink_EncodeHilActuatorControls(buffer, sizeof(buffer), timestamp_us, controls,
                                                              MAV_MODE_FLAG_HIL_ENABLED, flags);

    if (length > 0U)
    {
        /* HIL 服务只负责组装消息，底层的物理发送交由 DataRouter 任务处理。 */
        posted = DataRouter_PostFrame(buffer, length);
    }

    Mavlink_TxTransactionEnd();
    return posted;
}

/* ========================================================= */
/*                     Public API (公共接口)                 */
/* ========================================================= */

bool HilService_Init(void)
{
    /* 容许重复调用，已初始化则直接返回成功 */
    if (hil_initialized)
    {
        return true;
    }

    /* 注册本地的消息发布者节点 */
    hil_sensor_pub = PubRegister(HIL_SENSOR_TOPIC_NAME, sizeof(HIL_Sensor_Data_t));
    hil_gps_pub    = PubRegister(HIL_GPS_TOPIC_NAME, sizeof(HIL_GPS_Data_t));
    hil_state_pub  = PubRegister(HIL_STATE_TOPIC_NAME, sizeof(HIL_State_Data_t));

    if ((hil_sensor_pub == NULL) || (hil_gps_pub == NULL) || (hil_state_pub == NULL))
    {
        RTTERROR("[HIL] Publisher register failed.");
        return false;
    }

    /* 清理全局内部状态 */
    memset(&hil_stats, 0, sizeof(hil_stats));
    memset(actuator_controls, 0, sizeof(actuator_controls));

    actuator_flags      = 0U;
    actuator_valid      = 0U;
    last_actuator_tx_us = 0U;
    hil_active          = 0U;

    hil_initialized = 1U;
    RTTINFO("[HIL] Service initialized.");

    return true;
}

void HilService_HandleMavlinkMessage(const mavlink_message_t *msg)
{
    if (!hil_initialized || msg == NULL)
    {
        return;
    }

    /* 分发解析处理 */
    switch (msg->msgid)
    {
    case MAVLINK_MSG_ID_HIL_SENSOR:
        HilService_HandleSensor(msg);
        break;
    case MAVLINK_MSG_ID_HIL_GPS:
        HilService_HandleGps(msg);
        break;
    case MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
        HilService_HandleState(msg);
        break;
    default:
        break;
    }
}

void HilService_Update(uint64_t now_us)
{
    /* 非 HIL 模式，不应向仿真器发送控制数据 */
    if (!hil_initialized || !HilService_IsActive())
    {
        return;
    }

    /* 按照预先设定的频率（500Hz / 每 2ms）周期性发送控制指令 */
    if ((last_actuator_tx_us != 0U) && ((now_us - last_actuator_tx_us) < HIL_SERVICE_ACTUATOR_INTERVAL_US))
    {
        return;
    }

    /* 确保提交成功后，才更新时间戳与统计计数 */
    if (HilService_SendActuatorControls(now_us))
    {
        last_actuator_tx_us = now_us;
        hil_stats.actuator_messages++;
        hil_stats.last_actuator_tx_us = now_us;
    }
    else
    {
        hil_stats.actuator_drops++;
    }
}

void HilService_SetActive(bool active)
{
    hil_active = active ? 1U : 0U;

    if (active)
    {
        RTTINFO("[HIL] Mode enabled.");
    }
    else
    {
        RTTINFO("[HIL] Mode disabled.");
    }
}

bool HilService_IsActive(void)
{
    return (hil_active != 0U);
}

void HilService_SetActuatorControls(const float *controls, uint8_t count, uint64_t flags)
{
    if (!hil_initialized || controls == NULL)
    {
        return;
    }

    /* 限制通道数，防止数组越界 */
    if (count > HIL_SERVICE_ACTUATOR_COUNT)
    {
        count = HIL_SERVICE_ACTUATOR_COUNT;
    }

    /*
     * 优化策略：
     * 为了最大程度地缩短临界区阻塞时间，将数据限幅（0.0 ~ 1.0）
     * 和浮点运算处理统一放在临界区外完成，暂存于局部数组 temp 中。
     */
    float temp[HIL_SERVICE_ACTUATOR_COUNT] = {0};

    for (uint8_t i = 0U; i < count; i++)
    {
        float value = controls[i];

        if (value < 0.0f)
        {
            value = 0.0f;
        }
        else if (value > 1.0f)
        {
            value = 1.0f;
        }

        temp[i] = value;
    }

    /* 临界区仅执行最快速的内存快照替换，不夹杂任何逻辑运算 */
    taskENTER_CRITICAL();

    memcpy(actuator_controls, temp, sizeof(actuator_controls));
    actuator_flags = flags;
    actuator_valid = 1U;

    taskEXIT_CRITICAL();
}

void HilService_GetStats(HilService_Stats_t *out)
{
    if (out == NULL)
    {
        return;
    }

    /*
     * 必须开启临界区保护。
     * GetStats 虽为低频的调试接口，但内部包含 64 位时间戳和多项字段，
     * 在 32 位单片机中，读 64 位数不是原子的。临界区保证数据快照的一致性。
     */
    taskENTER_CRITICAL();
    memcpy(out, &hil_stats, sizeof(*out));
    taskEXIT_CRITICAL();
}
