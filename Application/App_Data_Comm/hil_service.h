#ifndef MY_NEW_UAV_BAICE_FRAMEWORK_HIL_SERVICE_H
#define MY_NEW_UAV_BAICE_FRAMEWORK_HIL_SERVICE_H

#include <stdbool.h>
#include <stdint.h>
#include "common/mavlink.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================= Topic (主题定义) ========================= */
#define HIL_SENSOR_TOPIC_NAME "hil_sensor" // 传感器数据发布主题
#define HIL_GPS_TOPIC_NAME    "hil_gps"    // GPS数据发布主题
#define HIL_STATE_TOPIC_NAME  "hil_state"  // 飞行器完整状态发布主题

/* ========================= Config (配置参数) ========================= */
/*
 * MAVLink HIL_ACTUATOR_CONTROLS 消息配置
 * 协议规定 controls 字段固定为16个 float 类型的数据。
 */
#define HIL_SERVICE_ACTUATOR_COUNT 16U

/*
 * HIL 模式下执行器的输出频率：
 * 2000us = 500Hz (每 2ms 发送一次控制指令给仿真器)
 */
#define HIL_SERVICE_ACTUATOR_INTERVAL_US 2000ULL

/* ========================= RX Data (接收数据结构) ========================= */

/**
 * @brief HIL 模拟传感器数据 (IMU, 气压计等)
 */
typedef struct
{
    uint64_t timestamp_us;    // 仿真器生成的原始时间戳 (微秒)
    uint64_t rx_timestamp_us; // 飞控本地接收到该数据的时间戳 (微秒)

    float accel[3]; // 加速度计 (X, Y, Z)，单位：m/s^2
    float gyro[3];  // 陀螺仪 (X, Y, Z)，单位：rad/s
    float mag[3];   // 磁力计 (X, Y, Z)，单位：Gauss 或 Tesla，根据具体飞控设定

    float abs_pressure;  // 绝对气压，单位：hPa 或 mbar
    float diff_pressure; // 差压 (空速计使用)，单位：hPa
    float pressure_alt;  // 气压高度，单位：m
    float temperature;   // 传感器温度，单位：摄氏度

    uint32_t fields_updated; // 位掩码，指示哪些传感器字段在本次消息中被更新
} HIL_Sensor_Data_t;

/**
 * @brief HIL 模拟 GPS 数据
 */
typedef struct
{
    uint64_t timestamp_us;    // 仿真器生成的原始时间戳
    uint64_t rx_timestamp_us; // 飞控本地接收时间戳

    double latitude;  // 纬度，单位：度 (deg)
    double longitude; // 经度，单位：度 (deg)
    float  altitude;  // 海拔高度，单位：m

    float eph; // 水平定位精度，单位：m
    float epv; // 垂直定位精度，单位：m

    float velocity;        // 对地速度，单位：m/s
    float velocity_ned[3]; // 北东地 (NED) 坐标系下的速度分量 (北, 东, 下)，单位：m/s
    float course;          // 航向角，单位：度 (deg)

    uint8_t fix_type;           // GPS 定位状态 (0=未定位, 3=3D定位等)
    uint8_t satellites_visible; // 可见卫星数量
} HIL_GPS_Data_t;

/**
 * @brief HIL 模拟的完整系统状态 (常用于完美状态反馈)
 */
typedef struct
{
    uint64_t timestamp_us;    // 仿真器时间戳
    uint64_t rx_timestamp_us; // 飞控本地接收时间戳

    float quaternion[4];       // 姿态四元数 (w, x, y, z)
    float angular_velocity[3]; // 角速度 (滚转, 俯仰, 偏航)，单位：rad/s

    double latitude;  // 纬度，单位：度 (deg)
    double longitude; // 经度，单位：度 (deg)
    float  altitude;  // 高度，单位：m

    float velocity_ned[3];    // NED 坐标系下的速度，单位：m/s
    float indicated_airspeed; // 指示空速，单位：m/s
    float true_airspeed;      // 真实空速，单位：m/s
    float accel[3];           // 机体坐标系下的加速度 (X, Y, Z)，单位：m/s^2
} HIL_State_Data_t;

/* ========================= Stats (统计信息) ========================= */

/**
 * @brief HIL 服务运行状态与统计信息
 */
typedef struct
{
    /* RX (接收统计) */
    uint32_t sensor_messages; // 收到的 HIL_SENSOR 消息数量
    uint32_t gps_messages;    // 收到的 HIL_GPS 消息数量
    uint32_t state_messages;  // 收到的 HIL_STATE 消息数量

    /* TX (发送统计) */
    uint32_t actuator_messages; // 成功发送的控制指令消息数量
    uint32_t actuator_drops;    // 丢弃的控制指令消息数量

    /* 历史时间戳 */
    uint64_t last_rx_timestamp_us;  // 最后一次接收到任何 HIL 消息的本地时间
    uint64_t last_sim_timestamp_us; // 最后一次接收到任何 HIL 消息对应的仿真器时间
    uint64_t last_actuator_tx_us;   // 最后一次发送执行器控制指令的本地时间
} HilService_Stats_t;

/* ========================= API (接口函数) ========================= */

/**
 * @brief 初始化 HIL Service
 * @return 成功返回 true，失败返回 false
 */
bool HilService_Init(void);

/**
 * @brief 处理与 HIL 相关的 MAVLink 消息
 * @param msg 指向收到的 MAVLink 消息结构体的指针
 *
 * 当前支持解析：
 * - HIL_SENSOR
 * - HIL_GPS
 * - HIL_STATE_QUATERNION
 */
void HilService_HandleMavlinkMessage(const mavlink_message_t *msg);

/**
 * @brief HIL 服务周期性更新函数
 * @param now_us 当前的系统时间 (微秒)
 *
 * 由系统的统一 Service 任务按固定周期调用。
 * 主要负责以固定频率 (如 500Hz) 向仿真器发送 HIL_ACTUATOR_CONTROLS 消息。
 */
void HilService_Update(uint64_t now_us);

/**
 * @brief 启用或禁用 HIL (硬件在环) 模式
 * @param active true 启用, false 禁用
 */
void HilService_SetActive(bool active);

/**
 * @brief 查询当前是否处于 HIL 模式
 * @return true 处于 HIL 模式, false 未处于
 */
bool HilService_IsActive(void);

/**
 * @brief 更新最新一次的执行器 (电机/舵机) 控制量
 * @param controls 包含控制信号数组的指针 (范围通常为 0.0~1.0 或 -1.0~1.0)
 * @param count 控制通道数量 (最大支持 HIL_SERVICE_ACTUATOR_COUNT)
 * @param flags 附加标志位
 *
 * 仅保存最新计算出的控制量快照，供 Update 周期函数发送。不使用队列机制。
 */
void HilService_SetActuatorControls(const float *controls, uint8_t count, uint64_t flags);

/**
 * @brief 获取当前 HIL 服务的统计信息
 * @param out 指向统计信息结构体的指针，用于输出数据
 */
void HilService_GetStats(HilService_Stats_t *out);

#ifdef __cplusplus
}
#endif

#endif // MY_NEW_UAV_BAICE_FRAMEWORK_HIL_SERVICE_H