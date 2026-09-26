#include "system_service.h"
#include "bsp_timestamp.h"
#include "DataRouterTask.h"
#include "hil_service.h"
#include "module_pwm_motor.h"

/* 心跳包发送周期：1,000,000 微秒 (即 1Hz / 1秒) */
#define SYSTEM_HEARTBEAT_INTERVAL_US 1000000ULL

/* 记录上一次发送心跳包的本地时间戳 */
static uint64_t last_heartbeat_us = 0U;

/**
 * @brief 构造并发送系统心跳包 (Heartbeat)
 */
static void SystemService_SendHeartbeat(void)
{
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    /* 初始化心跳包的基本状态信息 */
    Mavlink_Heartbeat_Data_t heartbeat = {
        .type          = MAV_TYPE_QUADROTOR,                // 飞行器类型：四旋翼
        .autopilot     = MAV_AUTOPILOT_GENERIC,             // 飞控类型：通用飞控
        .base_mode     = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, // 基础模式：启用自定义模式支持
        .custom_mode   = 0U,                                // 自定义模式标识
        .system_status = MAV_STATE_STANDBY                  // 系统状态：待机/就绪
    };

    /* 如果当前处于 HIL (硬件在环仿真) 模式，需要向地面站广播该状态 */
    if (HilService_IsActive())
    {
        heartbeat.base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
        heartbeat.system_status = MAV_STATE_ACTIVE;
    }

    /* 将电机状态映射到标准 MAVLink 解锁位，供上位机完成安全状态闭环确认。 */
    if (Motor_GetState() == MOTOR_STATE_ARMED)
    {
        heartbeat.base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
        heartbeat.system_status = MAV_STATE_ACTIVE;
    }

    /* 编码 Heartbeat 消息 */
    if (!Mavlink_TxTransactionBegin())
    {
        return;
    }

    const uint16_t length = Mavlink_EncodeHeartbeat(buffer, sizeof(buffer), &heartbeat);

    /* 投递到数据路由任务进行发送 */
    if (length > 0U)
    {
        (void)DataRouter_PostFrame(buffer, length);
    }

    Mavlink_TxTransactionEnd();
}

/**
 * @brief 处理收到的 PING 请求
 * @param msg 接收到的 MAVLink 消息
 *
 * 收到 PING 请求后，设备必须回传相同的时间戳和序列号，用于地面站评估通信链路的延迟。
 */
static void SystemService_HandlePing(const mavlink_message_t *msg)
{
    mavlink_ping_t ping;
    mavlink_msg_ping_decode(msg, &ping);

    /*
     * MAVLink PING 协议约定：
     * 如果 target_system 和 target_component 都为 0，这代表这是一次 PING 请求（类似于广播查找）。
     * 如果它们不为 0，则说明这是某个设备对 PING 请求的“回复”。
     * 为了防止两个设备陷入无限循环地互相 PING，这里只处理请求报文，忽略回复报文。
     */
    if ((ping.target_system != 0U) || (ping.target_component != 0U))
    {
        return;
    }

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    /*
     * 编码 PING 回复消息：
     * 原封不动地塞回请求者的时间戳 (ping.time_usec) 和序列号 (ping.seq)
     * 并且将请求方的 sysid 和 compid 作为我们回复的 target 地址。
     */
    if (!Mavlink_TxTransactionBegin())
    {
        return;
    }

    const uint16_t length = Mavlink_EncodePing(buffer, sizeof(buffer), ping.time_usec, ping.seq, msg->sysid,
                                               msg->compid);

    if (length > 0U)
    {
        (void)DataRouter_PostFrame(buffer, length);
    }

    Mavlink_TxTransactionEnd();
}

/**
 * @brief 处理收到的 TIMESYNC 时间同步请求
 * @param msg 接收到的 MAVLink 消息
 */
static void SystemService_HandleTimesync(const mavlink_message_t *msg)
{
    mavlink_timesync_t timesync;
    mavlink_msg_timesync_decode(msg, &timesync);

    /*
     * MAVLink TIMESYNC 协议约定：
     * tc1 (Time Component 1) 字段如果不为 0，说明这个报文已经是一次时间同步的“响应”。
     * 本机作为响应方收到响应报文后，应当静默，不能再回复，否则会导致广播风暴。
     */
    if (timesync.tc1 != 0)
    {
        return;
    }

    /* 检查该时间同步请求是否是发给本机的 */
    if (!Mavlink_IsTarget(timesync.target_system, timesync.target_component))
    {
        return;
    }

    /* 获取本地的高精度时间，并将微秒 (us) 转换为纳秒 (ns) */
    const int64_t local_time_ns = (int64_t)(Bsp_Timestamp_us_Get() * 1000ULL);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    /* 编码响应报文：将本机的纳秒级时间戳作为 tc1 返回给请求者 */
    if (!Mavlink_TxTransactionBegin())
    {
        return;
    }

    const uint16_t length = Mavlink_EncodeTimesync(buffer, sizeof(buffer), local_time_ns, timesync.ts1, msg->sysid,
                                                   msg->compid);

    if (length > 0U)
    {
        (void)DataRouter_PostFrame(buffer, length);
    }

    Mavlink_TxTransactionEnd();
}

/* ========================================================= */
/*                     Public API (公共接口)                 */
/* ========================================================= */

bool SystemService_Init(void)
{
    last_heartbeat_us = 0U;
    return true;
}

void SystemService_HandleMavlinkMessage(const mavlink_message_t *msg)
{
    if (msg == NULL)
    {
        return;
    }

    switch (msg->msgid)
    {
    case MAVLINK_MSG_ID_PING:
        SystemService_HandlePing(msg);
        break;

    case MAVLINK_MSG_ID_TIMESYNC:
        SystemService_HandleTimesync(msg);
        break;

    default:
        break;
    }
}

void SystemService_Update(uint64_t now_us)
{
    /* 周期性触发：如果是首次运行或距上次发送已超过 1 秒，则发送一次心跳包 */
    if ((last_heartbeat_us == 0U) || ((now_us - last_heartbeat_us) >= SYSTEM_HEARTBEAT_INTERVAL_US))
    {
        last_heartbeat_us = now_us;
        SystemService_SendHeartbeat();
    }
}
