#include "log_service.h"

/**
 * @file log_service.c
 * @brief 飞行控制日志采集与 MAVLink 日志下载服务。
 *
 * 本服务把控制任务发布的 Control_Log_t 转换为稳定的 SD 卡 V4 线格式，同时把地面站
 * 的 LOG_REQUEST_* 消息转交 SD 卡任务，并将异步响应编码回 MAVLink 发送队列。
 */

#include "App_Control.h"
#include "DataRouterTask.h"
#include "modules_Message_center.h"
#include "modules_SD_Card.h"

#include <math.h>
#include <string.h>

/* 单次调度最多回传 4 条日志响应，避免日志下载长时间占用通信任务。 */
#define LOG_SERVICE_MAX_RESPONSES_PER_UPDATE 4U
/* 控制日志最小记录间隔为 5000 us，即最高 200 Hz。 */
#define LOG_SERVICE_FLIGHT_INTERVAL_US 5000ULL

/* control_log 主题订阅者：读取控制任务发布的最新一份控制快照。 */
static Subscriber_t *control_log_subscriber;
/* 上一次执行日志采样的调度时间，用于限制SD日志记录频率。 */
static uint64_t last_flight_log_us;
/* 上一次已经处理的控制样本序号，用于避免重复记录同一份快照。 */
static uint32_t last_flight_sample_sequence;
/* 标记是否已经接收过样本，避免初始 sequence=0 被误判为重复数据。 */
static uint8_t flight_sample_seen;

/**
 * @brief 将精简后的 Control_Log_t 适配到既有 Flight Control V4 线格式。
 *
 * 新控制日志不再携带姿态估计器、遥控原始值和 PID 分项。为保持现有 SD 帧版本和
 * 上位机解析兼容，这些不可获得的浮点字段明确写为 NaN，整数/状态字段写为 0；
 * 绝不把 0.0f 冒充成有效的姿态、位置或传感器测量。
 */
static void LogService_BuildFlightPayload(const Control_Log_t *snapshot, SDCard_FlightControlPayloadV4_t *payload)
{
    /*
     * 先清零完整 V4 载荷：未显式赋值的离散状态和保留字段统一保持为 0，
     * 防止把任务栈中的随机数据写入日志文件。
     */
    memset(payload, 0, sizeof(*payload));

    /*
     * V4 按“V4 包含 V3、V3 包含 V2、V2 包含 V1”的方式保持旧版前缀兼容。
     * base 指向最基础的 V1 区域，旧版解析器仍可按原偏移读取这些字段。
     */
    SDCard_FlightControlPayloadV1_t *base = &payload->v3.v2.v1;
    base->sample_sequence                 = snapshot->sequence;

    /* 新 Control_Log_t 不再提供控制周期和横滚/俯仰姿态目标，明确标记为无效值。 */
    base->dt_s                = NAN;
    base->attitude_ref_rad[0] = NAN;
    base->attitude_ref_rad[1] = NAN;

    /* 依次填写 X/Y/Z 三个轴的控制量；估计器与 PID 分项缺失时使用 NaN。 */
    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        /* 姿态、位置、速度和加速度应由各自的专用日志记录，此处不伪造为 0。 */
        base->euler_rad[axis]       = NAN;
        base->pos_ned_m[axis]       = NAN;
        base->vel_ned_mps[axis]     = NAN;
        base->gyro_rps[axis]        = snapshot->gyro_rps[axis];
        base->accel_mps2[axis]      = NAN;
        base->gyro_bias_rps[axis]   = NAN;
        base->accel_bias_mps2[axis] = NAN;

        /* 角速度环分析所需的目标、测量、误差和总输出仍由控制日志提供。 */
        base->attitude_error_rad[axis] = snapshot->attitude_error_rad[axis];
        base->rate_ref_rps[axis]       = snapshot->rate_ref_rps[axis];
        base->rate_measure_rps[axis]   = snapshot->gyro_rps[axis];
        base->pid_p[axis]              = NAN;
        base->pid_i[axis]              = NAN;
        base->pid_d[axis]              = NAN;
        base->pid_output[axis]         = snapshot->rate_output[axis];
    }

    /* 四元数当前不可用；同一个四元素循环内保存四路电机输出。 */
    for (uint8_t index = 0U; index < 4U; index++)
    {
        base->q_nb[index]         = NAN;
        base->motor_output[index] = snapshot->motor_output[index];
    }

    /* V1 公共区保存最终生效油门，以及电机和角速度闭环的实际状态。 */
    base->throttle       = snapshot->effective_throttle;
    base->motor_armed    = snapshot->motor_armed;
    base->control_active = snapshot->rate_loop_active;

    /*
     * V2 扩展区记录高度控制量。误差在这里由“目标值 - 测量值”统一计算，
     * 高度环未提供的 PID 分项和基础油门仍以 NaN 表示不可用。
     */
    SDCard_FlightControlPayloadV2_t *altitude = &payload->v3.v2;
    altitude->height_target_m                 = snapshot->height_target_m;
    altitude->height_measure_m                = snapshot->height_measure_m;
    altitude->height_error_m                  = snapshot->height_target_m - snapshot->height_measure_m;
    altitude->vertical_speed_ref_mps          = snapshot->vertical_speed_ref_mps;
    altitude->vertical_speed_measure_mps      = snapshot->vertical_speed_measure_mps;
    altitude->vertical_speed_error_mps        = snapshot->vertical_speed_ref_mps - snapshot->vertical_speed_measure_mps;
    altitude->vertical_speed_pid_p            = NAN;
    altitude->vertical_speed_pid_i            = NAN;
    altitude->altitude_base_throttle          = NAN;
    altitude->altitude_throttle_correction    = snapshot->altitude_correction;
    altitude->altitude_hold_active            = snapshot->altitude_hold_active;
    altitude->navigation_valid                = snapshot->navigation_valid;

    /* V4 新增区同时保留飞手油门、最终油门和偏航目标，便于分析控制权切换。 */
    payload->pilot_throttle     = snapshot->pilot_throttle;
    payload->effective_throttle = snapshot->effective_throttle;
    payload->yaw_target_rad     = snapshot->yaw_target_rad;
    payload->motor_armed        = snapshot->motor_armed;
    /* 该控制快照由电机输出生成路径发布，因此四路 motor_output 被视为有效。 */
    payload->motor_output_valid = 1U;
    payload->closed_loop_active = snapshot->rate_loop_active;
}

/** @brief 根据日志载荷中的离散状态生成 SD 帧头标志位。 */
static uint16_t LogService_GetFlightFlags(const SDCard_FlightControlPayloadV4_t *payload)
{
    uint16_t flags = 0U;

    /* 把常用飞行状态压缩到帧头，扫描日志时无需先解析完整的 V4 载荷。 */
    flags |= (payload->v3.v2.v1.arm_request != 0U) ? SD_CARD_FLIGHT_ARM_REQUEST : 0U;
    flags |= (payload->v3.v2.v1.failsafe != 0U) ? SD_CARD_FLIGHT_FAILSAFE : 0U;
    flags |= (payload->motor_armed != 0U) ? SD_CARD_FLIGHT_MOTOR_ARMED : 0U;
    flags |= (payload->closed_loop_active != 0U) ? SD_CARD_FLIGHT_CONTROL_ACTIVE : 0U;
    flags |= (payload->v3.v2.altitude_hold_active != 0U) ? SD_CARD_FLIGHT_ALT_HOLD_ACTIVE : 0U;
    return flags;
}

/**
 * @brief 按 200 Hz 上限记录一帧新的控制快照。
 * @note 同一 sequence 只写入一次；SD 队列满时当前帧允许丢弃，不阻塞控制/通信任务。
 */
static void LogService_RecordFlightData(uint64_t now_us)
{
    Control_Log_t snapshot;

    /*
     * 按从左到右的短路顺序跳过以下情况：
     * 1. 距离上次采样不足 5000 us；
     * 2. control_log 订阅者尚未创建；
     * 3. 主题中没有比上次更新的控制快照（0 超时表示立即返回、不阻塞）；
     * 4. 快照 sequence 与已记录样本相同。
     *
     * 限频判断放在读取主题之前，因此 5 ms 到期时读取的是控制任务发布的
     * 最新快照，而不是把期间产生的每个 1 kHz 样本逐条补写到 SD 卡。
     */
    if (((last_flight_log_us != 0ULL) && ((now_us - last_flight_log_us) < LOG_SERVICE_FLIGHT_INTERVAL_US)) ||
        (control_log_subscriber == NULL) || (SubGetMessage(control_log_subscriber, &snapshot, 0U) == 0U) ||
        ((flight_sample_seen != 0U) && (snapshot.sequence == last_flight_sample_sequence)))
    {
        return;
    }

    SDCard_FlightControlPayloadV4_t payload;
    LogService_BuildFlightPayload(&snapshot, &payload);
    const uint16_t flags = LogService_GetFlightFlags(&payload);

    /*
     * 这里只把完整日志帧提交给 SD 卡模块的内存缓冲区，实际 FatFS 写文件由
     * SDCardTask 异步完成。队列/缓冲区满时允许丢弃本帧，绝不等待磁盘而阻塞通信任务。
     */
    (void)SDCard_EnqueueFrame(SD_CARD_MSG_FLIGHT_CONTROL, 4U, flags, snapshot.timestamp_us, &payload, sizeof(payload));

    /* 使用服务调度时间做 200 Hz 限频，使用控制快照自身的 timestamp_us 写入日志帧。 */
    last_flight_log_us          = now_us;
    last_flight_sample_sequence = snapshot.sequence;
    flight_sample_seen          = 1U;
}

/** @brief 订阅控制日志主题并复位采样节流状态。 */
bool LogService_Init(void)
{
    /* 注册时校验主题元素大小，防止发布者和订阅者使用不一致的 Control_Log_t 布局。 */
    control_log_subscriber = SubRegister(CONTROL_LOG_TOPIC_NAME, sizeof(Control_Log_t));

    /* 清除上一次运行遗留的限频和去重状态，从下一份新快照重新开始记录。 */
    last_flight_log_us          = 0ULL;
    last_flight_sample_sequence = 0U;
    flight_sample_seen          = 0U;

    /* 订阅创建失败时让上层初始化流程能够发现日志服务不可用。 */
    return control_log_subscriber != NULL;
}

/**
 * @brief 解码并投递地面站日志请求。
 * @note 仅接受目标系统/组件匹配本机的请求，文件系统操作由 SD 卡任务异步执行。
 */
void LogService_HandleMavlinkMessage(const mavlink_message_t *msg)
{
    if (msg == NULL)
    {
        return;
    }

    switch (msg->msgid)
    {
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
    {
        /* 请求指定编号范围内的日志目录，SD 卡任务随后异步返回 LOG_ENTRY。 */
        mavlink_log_request_list_t request;
        mavlink_msg_log_request_list_decode(msg, &request);
        if (Mavlink_IsTarget(request.target_system, request.target_component))
        {
            (void)SDCard_RequestLogList(request.start, request.end);
        }
        break;
    }
    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
    {
        /* 请求某个日志从 ofs 开始的 count 字节，返回数据会拆分为 LOG_DATA。 */
        mavlink_log_request_data_t request;
        mavlink_msg_log_request_data_decode(msg, &request);
        if (Mavlink_IsTarget(request.target_system, request.target_component))
        {
            (void)SDCard_RequestLogData(request.id, request.ofs, request.count);
        }
        break;
    }
    case MAVLINK_MSG_ID_LOG_ERASE:
    {
        /* 擦除操作可能耗时，仅投递命令，不在 MAVLink 接收路径中直接访问文件系统。 */
        mavlink_log_erase_t request;
        mavlink_msg_log_erase_decode(msg, &request);
        if (Mavlink_IsTarget(request.target_system, request.target_component))
        {
            (void)SDCard_RequestLogErase();
        }
        break;
    }
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
    {
        /* 上位机结束下载时，通知 SD 卡任务关闭当前传输并清理会话状态。 */
        mavlink_log_request_end_t request;
        mavlink_msg_log_request_end_decode(msg, &request);
        if (Mavlink_IsTarget(request.target_system, request.target_component))
        {
            (void)SDCard_RequestLogEnd();
        }
        break;
    }
    default:
        break;
    }
}

/**
 * @brief 记录控制数据，并将有限数量的 SD 日志响应编码发送。
 * @note 单次最多处理 LOG_SERVICE_MAX_RESPONSES_PER_UPDATE 条，避免日志下载独占通信周期。
 */
void LogService_Update(uint64_t now_us)
{
    SDCard_LogResponse_t response;
    uint8_t              buffer[MAVLINK_MAX_PACKET_LEN];

    /* 每次服务调度先尝试记录一份最新控制快照，内部仍受 200 Hz 间隔限制。 */
    LogService_RecordFlightData(now_us);

    /* 限制单次处理数量，防止大日志下载连续占用 DataRouter 的整个调度周期。 */
    for (uint8_t i = 0U; i < LOG_SERVICE_MAX_RESPONSES_PER_UPDATE; i++)
    {
        /*
         * SD 模块内部 ready 队列只传递静态响应槽指针；此接口在队列临界区外
         * 把槽内容复制到 response 并归还槽位，因此不会在关中断期间复制大结构体。
         */
        if (!SDCard_PopLogResponse(&response))
        {
            /* 当前没有待发送响应，本轮更新可以立即结束。 */
            return;
        }

        if (!Mavlink_TxTransactionBegin())
        {
            return;
        }

        uint16_t length = 0U;
        if (response.type == SD_CARD_LOG_RESPONSE_ENTRY)
        {
            /* 把日志目录项编码成 MAVLink LOG_ENTRY 消息。 */
            length = Mavlink_EncodeLogEntry(buffer, sizeof(buffer), response.id, response.num_logs,
                                            response.last_log_num, response.time_utc, response.size);
        }
        else if (response.type == SD_CARD_LOG_RESPONSE_DATA)
        {
            /* 把一段文件内容编码成 MAVLink LOG_DATA 消息。 */
            length = Mavlink_EncodeLogData(buffer, sizeof(buffer), response.id, response.offset, response.count,
                                           response.data);
        }

        if (length > 0U)
        {
            /* 编码成功后交给 DataRouter，由具体通信链路异步发送到上位机。 */
            (void)DataRouter_PostFrame(buffer, length);
        }

        /* 释放后其他任务才可分配下一发送序号，保证序号顺序等于队列顺序。 */
        Mavlink_TxTransactionEnd();
    }
}
