/**
 * @file modules_SD_Card.h
 * @brief 飞控 SD 卡黑匣子的统一帧格式、运行状态与公共接口
 */

#ifndef STM32H743_UAV_FLIGHT_CONTROLLER_MODULES_SD_CARD_H
#define STM32H743_UAV_FLIGHT_CONTROLLER_MODULES_SD_CARD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ff.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================== 公共配置 ========================== */

#define SD_CARD_RING_BUFFER_SIZE  (128U * 1024U) /* DataRouter 与写卡任务之间的环形缓冲区，必须为 2 的幂 */
#define SD_CARD_WRITE_BUFFER_SIZE (8U * 1024U)   /* 单次从环形缓冲区取出的最大批量写入字节数 */
#define SD_CARD_MAX_PAYLOAD_SIZE  300U           /* 单条日志帧允许携带的最大有效载荷 */
#define SD_CARD_SYNC_PERIOD_MS    500U           /* 强制刷新 FatFS 缓存的周期，单位 ms */
#define SD_CARD_TASK_STACK_WORDS  1024U          /* SD 写卡任务栈深度，单位为 StackType_t */

#define SD_CARD_FILE_MAGIC                0x474F4C46UL /* 小端文件中显示为 ASCII“FLOG” */
#define SD_CARD_FRAME_MAGIC               0xA55AU      /* 每条日志帧的同步标记 */
#define SD_CARD_CRC_INITIAL               0xFFFFU      /* CRC16-CCITT 初始值 */
#define SD_CARD_FORMAT_VERSION            1U
#define SD_CARD_FORMAT_FLAG_LITTLE_ENDIAN (1UL << 0U)
#define SD_CARD_FORMAT_FLAG_IEEE754_FLOAT (1UL << 1U)

#define SD_CARD_LOG_DATA_CHUNK_SIZE 90U

/* ========================== 类型定义 ========================== */

/**
 * @brief 黑匣子消息类型
 *
 * 第一版记录 DataRouter 实际发送的完整 MAVLink 帧。后续增加传感器、控制器
 * 或故障消息时，应继续分配稳定 ID，已经使用的编号不得改变或复用。
 */
typedef enum
{
    SD_CARD_MSG_MAVLINK_TX           = 1U, /* DataRouter 发往地面站的完整 MAVLink 帧 */
    SD_CARD_MSG_EVENT                = 2U, /* 预留的飞控离散事件记录 */
    SD_CARD_MSG_BMI088_IMU           = 3U, /* BMI088 固定标定与 Allan 分析原始样本 */
    SD_CARD_MSG_FLIGHT_CONTROL       = 4U,
    SD_CARD_MSG_NOTCH_DIAGNOSTIC     = 5U,
    SD_CARD_MSG_ESTIMATOR_COMPARISON = 6U
} SDCard_MessageId_e;

typedef enum
{
    SD_CARD_IMU_ACCEL_VALID       = (1U << 0U),
    SD_CARD_IMU_GYRO_VALID        = (1U << 1U),
    SD_CARD_IMU_TEMPERATURE_VALID = (1U << 2U)
} SDCard_ImuFlags_e;

/**
 * @brief SD_CARD_MSG_BMI088_IMU 的版本 1 载荷
 *
 * accel/gyro 已完成芯片坐标系到飞控 FRD 坐标系的映射和 SI 单位换算，但尚未
 * 应用 LM 固定参数。保留 LSB 原始值和加速度计 sensor-time，离线工具可识别
 * 800 Hz 加速度计在 1 kHz 读取链中产生的重复样本。MCU 时间戳位于帧头。
 */
typedef struct
{
    uint32_t sample_sequence;
    uint32_t accel_sensor_time;
    int16_t  accel_raw[3];
    int16_t  gyro_raw[3];
    int16_t  temperature_raw;
    uint16_t validity_flags;
    float    accel_mps2[3];
    float    gyro_rps[3];
    float    temperature_c;
} SDCard_BMI088ImuPayloadV1_t;

typedef enum
{
    SD_CARD_FLIGHT_ARM_REQUEST     = (1U << 0U),
    SD_CARD_FLIGHT_FAILSAFE        = (1U << 1U),
    SD_CARD_FLIGHT_MOTOR_ARMED     = (1U << 2U),
    SD_CARD_FLIGHT_CONTROL_ACTIVE  = (1U << 3U),
    SD_CARD_FLIGHT_ALT_HOLD_ACTIVE = (1U << 4U)
} SDCard_FlightFlags_e;

/**
 * @brief SD_CARD_MSG_FLIGHT_CONTROL version 1 payload
 *
 * All vectors use FRD/NED coordinates and SI units. The explicit packed layout is part of the
 * on-disk protocol and must only change with a new message version.
 */
typedef struct __attribute__((packed))
{
    uint32_t sample_sequence;
    uint32_t attitude_predict_count;
    float    dt_s;
    float    q_nb[4];
    float    euler_rad[3];
    float    pos_ned_m[3];
    float    vel_ned_mps[3];
    float    gyro_rps[3];
    float    accel_mps2[3];
    float    gyro_bias_rps[3];
    float    accel_bias_mps2[3];
    float    attitude_ref_rad[2];
    float    attitude_error_rad[3];
    float    rate_ref_rps[3];
    float    rate_measure_rps[3];
    float    pid_p[3];
    float    pid_i[3];
    float    pid_d[3];
    float    pid_output[3];
    float    motor_output[4];
    float    throttle;
    uint8_t  arm_request;
    uint8_t  failsafe;
    uint8_t  motor_armed;
    uint8_t  control_active;
} SDCard_FlightControlPayloadV1_t;

_Static_assert(sizeof(SDCard_FlightControlPayloadV1_t) == 228U, "Unexpected flight log payload layout");

/**
 * @brief Flight-control version 2 payload.
 *
 * The complete V1 payload remains the prefix. Altitude position/velocity loop diagnostics are
 * appended so existing V1 logs and readers remain distinguishable by frame version.
 */
typedef struct __attribute__((packed))
{
    SDCard_FlightControlPayloadV1_t v1;
    float                           height_target_m;
    float                           height_measure_m;
    float                           height_error_m;
    float                           vertical_speed_ref_mps;
    float                           vertical_speed_measure_mps;
    float                           vertical_speed_error_mps;
    float                           vertical_speed_pid_p;
    float                           vertical_speed_pid_i;
    float                           altitude_base_throttle;
    float                           altitude_throttle_correction;
    uint8_t                         altitude_hold_request;
    uint8_t                         altitude_hold_active;
    uint8_t                         navigation_valid;
    uint8_t                         reserved;
} SDCard_FlightControlPayloadV2_t;

_Static_assert(sizeof(SDCard_FlightControlPayloadV2_t) == 272U, "Unexpected flight log V2 payload layout");
_Static_assert(offsetof(SDCard_FlightControlPayloadV2_t, v1) == 0U, "Flight log V2 must preserve the V1 prefix");

/**
 * @brief Flight-control version 3 payload.
 *
 * V2 remains the complete prefix. Raw CH3/CH5 values and the altitude-mode state make mode
 * selection, centered-throttle arming and takeoff transitions directly auditable after flight.
 */
typedef struct __attribute__((packed))
{
    SDCard_FlightControlPayloadV2_t v2;
    uint16_t                        throttle_raw;
    uint16_t                        altitude_mode_raw;
    uint8_t                         altitude_mode_state;
    uint8_t                         reserved[3];
} SDCard_FlightControlPayloadV3_t;

_Static_assert(sizeof(SDCard_FlightControlPayloadV3_t) == 280U, "Unexpected flight log V3 payload layout");
_Static_assert(offsetof(SDCard_FlightControlPayloadV3_t, v2) == 0U, "Flight log V3 must preserve the V2 prefix");

/**
 * @brief Flight-control version 4 payload.
 *
 * V3 remains the complete prefix. V4 separates pilot input, controller output and motor/control
 * states so ground-idle, closed-loop flight and disarm transitions are unambiguous.
 */
typedef struct __attribute__((packed))
{
    SDCard_FlightControlPayloadV3_t v3;
    float                           pilot_throttle;
    float                           effective_throttle;
    float                           yaw_target_rad;
    uint8_t                         motor_armed;
    uint8_t                         motor_output_valid;
    uint8_t                         closed_loop_active;
    uint8_t                         reserved;
} SDCard_FlightControlPayloadV4_t;

_Static_assert(sizeof(SDCard_FlightControlPayloadV4_t) == 296U, "Unexpected flight log V4 payload layout");
_Static_assert(sizeof(SDCard_FlightControlPayloadV4_t) <= SD_CARD_MAX_PAYLOAD_SIZE,
               "Flight log V4 exceeds the frame payload limit");
_Static_assert(offsetof(SDCard_FlightControlPayloadV4_t, v3) == 0U, "Flight log V4 must preserve the V3 prefix");

/** @brief SD_CARD_MSG_NOTCH_DIAGNOSTIC version 1 payload */
typedef struct __attribute__((packed))
{
    float    center_hz;
    float    peak_snr;
    uint32_t update_count;
    uint8_t  tracking_valid;
    uint8_t  reserved[3];
} SDCard_NotchDiagnosticPayloadV1_t;

_Static_assert(sizeof(SDCard_NotchDiagnosticPayloadV1_t) == 16U, "Unexpected notch log payload layout");

/** @brief SD_CARD_MSG_NOTCH_DIAGNOSTIC version 2 payload: Roll/Pitch/Yaw independent tracking. */
typedef struct __attribute__((packed))
{
    float    center_hz[3];
    float    peak_snr[3];
    uint32_t update_count[3];
    uint8_t  tracking_valid[3];
    uint8_t  reserved;
} SDCard_NotchDiagnosticPayloadV2_t;

_Static_assert(sizeof(SDCard_NotchDiagnosticPayloadV2_t) == 40U, "Unexpected axis notch log payload layout");

typedef enum
{
    SD_CARD_ESTIMATOR_VQF_VALID            = (1U << 0U),
    SD_CARD_ESTIMATOR_ESKF_PREDICT_VALID   = (1U << 1U),
    SD_CARD_ESTIMATOR_ESKF_GRAVITY_UPDATED = (1U << 2U),
    SD_CARD_ESTIMATOR_CONTROL_SOURCE_VQF   = (1U << 3U)
} SDCard_EstimatorFlags_e;

/** @brief SD_CARD_MSG_ESTIMATOR_COMPARISON version 1 payload. */
typedef struct __attribute__((packed))
{
    uint32_t sample_sequence;
    uint32_t core_clock_hz;
    uint32_t vqf_cycles;
    uint32_t eskf_cycles;
    uint32_t eskf_gravity_updates_accepted;
    uint32_t eskf_gravity_updates_rejected;
    uint32_t eskf_gravity_gate_rejections;
    uint32_t eskf_numerical_failures;
    float    dt_s;
    float    vqf_q_nb[4];
    float    eskf_q_nb[4];
    float    vqf_gyro_bias_rps[3];
    float    eskf_gyro_bias_rps[3];
    uint8_t  valid_flags;
    uint8_t  reserved[3];
} SDCard_EstimatorComparisonPayloadV1_t;

_Static_assert(sizeof(SDCard_EstimatorComparisonPayloadV1_t) == 96U,
               "Unexpected estimator comparison log payload layout");

/**
 * @brief SD_CARD_MSG_ESTIMATOR_COMPARISON version 2 payload.
 *
 * 前96字节与V1保持一致；追加并行ESKF位置/速度、重力门控输入及光流/测距
 * 观测诊断。该载荷仅用于离线分析，不改变VQF控制源。
 */
typedef struct __attribute__((packed))
{
    SDCard_EstimatorComparisonPayloadV1_t v1;
    float                                 eskf_pos_ned_m[3];
    float                                 eskf_vel_ned_mps[3];
    float                                 flow_vel_body_xy_mps[2];
    float                                 mtf_range_m;
    float                                 tfmini_range_m;
    float                                 gravity_accel_norm_mps2;
    float                                 gravity_gyro_norm_rps;
    float                                 gravity_speed_norm_mps;
    float                                 gravity_innovation_rad;
    float                                 last_nis;
    float                                 last_chi_square_threshold;
    uint32_t                              flow_measurement_count;
    uint32_t                              flow_fusion_count;
    uint32_t                              mtf_range_measurement_count;
    uint32_t                              mtf_range_fusion_count;
    uint32_t                              tfmini_range_measurement_count;
    uint32_t                              tfmini_range_fusion_count;
    uint32_t                              sensor_update_flags;
    uint32_t                              observation_flags;
    uint16_t                              tfmini_strength;
    uint8_t                               flow_quality;
    uint8_t                               flow_protocol;
    uint8_t                               mtf_range_quality;
    uint8_t                               mtf_range_status;
    uint8_t                               tfmini_valid;
    uint8_t                               reserved_v2;
} SDCard_EstimatorComparisonPayloadV2_t;

_Static_assert(sizeof(SDCard_EstimatorComparisonPayloadV2_t) == 200U,
               "Unexpected estimator comparison V2 log payload layout");
_Static_assert(offsetof(SDCard_EstimatorComparisonPayloadV2_t, v1) == 0U,
               "Estimator comparison V2 must preserve the V1 prefix");
_Static_assert(offsetof(SDCard_EstimatorComparisonPayloadV2_t, eskf_pos_ned_m) == 96U,
               "Estimator comparison V2 extension offset changed");

/**
 * @brief SD 卡任务运行状态
 *
 * 每个失败阶段使用独立状态，便于在 VS Code Watch 中直接判断挂载、建目录、
 * 建文件、文件头写入、回读自检或运行期写入中的具体失败位置。
 */
typedef enum
{
    SD_CARD_STATUS_NOT_STARTED = 0U,    /* 尚未初始化，或静态任务创建失败 */
    SD_CARD_STATUS_STARTING,            /* 任务已创建，正在挂载文件系统并执行自检 */
    SD_CARD_STATUS_RUNNING,             /* 自检通过，黑匣子正在正常记录 */
    SD_CARD_STATUS_MOUNT_FAILED,        /* FatFS 挂载失败 */
    SD_CARD_STATUS_DIRECTORY_FAILED,    /* BLACKBOX 目录创建或访问失败 */
    SD_CARD_STATUS_FILE_OPEN_FAILED,    /* 日志文件路径选择或打开失败 */
    SD_CARD_STATUS_HEADER_WRITE_FAILED, /* 文件头写入或首次同步失败 */
    SD_CARD_STATUS_SELF_TEST_FAILED,    /* 文件头回读内容与写入内容不一致 */
    SD_CARD_STATUS_DATA_WRITE_FAILED,   /* 运行期数据写入不完整或失败 */
    SD_CARD_STATUS_SYNC_FAILED,         /* 运行期 f_sync() 失败 */
    SD_CARD_STATUS_ERASING,             /* 正在删除 BLACKBOX 日志 */
    SD_CARD_STATUS_ERASE_FAILED         /* 日志删除或删除后的重启失败 */
} SDCard_Status_e;

/**
 * @brief 每个日志文件开头的固定文件头
 *
 * 所有多字节整数使用 STM32H7 原生小端格式；浮点 payload 使用 IEEE-754
 * 单精度格式。header_crc16 覆盖本结构体中位于它之前的全部字段。
 */
typedef struct __attribute__((packed))
{
    uint32_t magic;              /* 固定为 SD_CARD_FILE_MAGIC，用于识别日志文件 */
    uint16_t format_version;     /* 文件格式版本，结构变化时必须递增 */
    uint16_t header_size;        /* 当前文件头总字节数 */
    uint64_t start_timestamp_us; /* 创建文件时的单调时间戳，单位 us */
    uint32_t format_flags;       /* 字节序、浮点格式等解析属性 */
    uint16_t header_crc16;       /* 文件头 CRC16-CCITT 校验值 */
    uint16_t reserved;           /* 保留字段，当前固定写 0 */
} SDCard_FileHeader_t;

/**
 * @brief 环形缓冲区和 SD 文件共用的日志帧头
 *
 * 一条完整记录的布局为：
 * SDCard_FrameHeader_t + payload[payload_length] + uint16_t crc16。
 * 帧尾 CRC 覆盖帧头与 payload；sequence 使用全局递增序号并允许自然回绕。
 */
typedef struct __attribute__((packed))
{
    uint16_t magic;           /* 固定为 SD_CARD_FRAME_MAGIC，用于离线重新同步 */
    uint8_t  message_id;      /* SDCard_MessageId_e，表示 payload 的业务类型 */
    uint8_t  message_version; /* 当前 payload 字段布局版本 */
    uint16_t payload_length;  /* payload 实际字节数 */
    uint16_t flags;           /* 数据有效性、来源或降级状态标志 */
    uint32_t sequence;        /* 全局日志帧序号，用于检测离线文件中的丢帧 */
    uint64_t timestamp_us;    /* 数据产生时间而非写卡时间，单位 us */
} SDCard_FrameHeader_t;

/**
 * @brief 黑匣子运行状态与统计量
 *
 * 这些字段只用于调试观测和离线判断日志质量，不参与飞控实时控制。生产者遇到
 * 环形缓冲区空间不足时只增加丢帧计数并立即返回，不允许等待 SD 卡。
 */
typedef struct
{
    volatile SDCard_Status_e status;       /* 当前运行阶段或最后一个故障阶段 */
    volatile FRESULT         last_fresult; /* 最近一次 FatFS 返回值 */

    volatile uint8_t initialized;      /* 环形缓冲区和静态任务已创建 */
    volatile uint8_t mounted;          /* FatFS 已成功挂载 */
    volatile uint8_t file_open;        /* 当前日志文件已打开 */
    volatile uint8_t self_test_passed; /* 文件头写入、同步和回读比较均通过 */

    volatile uint32_t next_sequence;       /* 下一条日志帧将使用的序号 */
    volatile uint32_t enqueued_frames;     /* 已完整写入环形缓冲区的帧数 */
    volatile uint32_t dropped_frames;      /* 环形缓冲区空间不足而丢弃的帧数 */
    volatile uint32_t dropped_bytes;       /* 因拥塞或短写而未记录的字节数 */
    volatile uint32_t invalid_frame_count; /* 参数非法而拒绝的帧数 */

    volatile uint32_t write_count;       /* 成功完成的 FatFS 批量写调用次数 */
    volatile uint32_t sync_count;        /* 成功完成的 f_sync() 次数 */
    volatile uint32_t write_error_count; /* FatFS 写入失败或短写次数 */
    volatile uint32_t sync_error_count;  /* f_sync() 失败次数 */

    volatile uint32_t ring_used_bytes;    /* 环形缓冲区当前占用字节数 */
    volatile uint32_t ring_peak_bytes;    /* 本次运行的最大占用字节数 */
    volatile uint32_t last_write_time_us; /* 最近一次 f_write() 耗时，单位 us */
    volatile uint32_t max_write_time_us;  /* 本次运行最大 f_write() 耗时，单位 us */
    volatile uint32_t last_sync_time_us;  /* 最近一次 f_sync() 耗时，单位 us */
    volatile uint32_t max_sync_time_us;   /* 本次运行最大 f_sync() 耗时，单位 us */

    volatile uint64_t bytes_written;     /* 已交给 FatFS 并确认写入的累计字节数 */
    volatile uint64_t current_file_size; /* 当前日志文件大小，单位字节 */

    char file_path[32]; /* 当前日志文件完整路径，例如 0:/BLACKBOX/LOG00001.BIN */
} SDCard_State_t;

typedef enum
{
    SD_CARD_LOG_RESPONSE_ENTRY = 0U,
    SD_CARD_LOG_RESPONSE_DATA
} SDCard_LogResponseType_e;

typedef struct
{
    SDCard_LogResponseType_e type;
    uint16_t                 id;
    uint16_t                 num_logs;
    uint16_t                 last_log_num;
    uint32_t                 time_utc;
    uint32_t                 size;
    uint32_t                 offset;
    uint8_t                  count;
    uint8_t                  data[SD_CARD_LOG_DATA_CHUNK_SIZE];
} SDCard_LogResponse_t;

/* 位于非缓存 D2 SRAM，供 VS Code/J-Link 实时观察。 */
extern SDCard_State_t sd_card_state;

/* ========================== 接口声明 ========================== */

/**
 * @brief 初始化环形缓冲区并创建低优先级 SD 卡任务
 * @return true 表示任务创建成功；挂载、建文件和回读自检结果请查看 sd_card_state
 */
bool SDCard_Init(void);

/**
 * @brief 将一条统一格式的完整日志帧非阻塞地压入 SD 环形缓冲区
 *
 * @param message_id      SDCard_MessageId_e
 * @param message_version 当前 payload 布局版本
 * @param flags           数据有效性、来源或降级状态标志
 * @param timestamp_us    数据产生时间戳，单位 us
 * @param payload         payload 首地址
 * @param payload_length  payload 字节数，最大为 SD_CARD_MAX_PAYLOAD_SIZE
 * @return true 表示完整帧已发布；false 表示参数错误、模块未运行或空间不足
 */
bool SDCard_EnqueueFrame(uint8_t message_id, uint8_t message_version, uint16_t flags, uint64_t timestamp_us,
                         const void *payload, uint16_t payload_length);

/**
 * @brief 非阻塞记录一帧 BMI088 标定样本
 * @return true 表示样本已进入 SD 环形缓冲区；false 表示日志尚未运行或发生拥塞
 */
bool SDCard_EnqueueBMI088Imu(uint64_t timestamp_us, const SDCard_BMI088ImuPayloadV1_t *sample);

bool SDCard_RequestLogList(uint16_t start, uint16_t end);
bool SDCard_RequestLogData(uint16_t id, uint32_t offset, uint32_t count);
bool SDCard_RequestLogEnd(void);
bool SDCard_RequestLogErase(void);
bool SDCard_PopLogResponse(SDCard_LogResponse_t *response);

/**
 * @brief 低优先级 FatFS 写卡任务
 *
 * 该任务是日志文件的唯一访问者，负责挂载、自检、批量写入和定期同步。
 */
void SDCard_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* STM32H743_UAV_FLIGHT_CONTROLLER_MODULES_SD_CARD_H */
