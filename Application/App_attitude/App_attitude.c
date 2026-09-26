/**
 * @file App_attitude.c
 * @brief 姿态估计主任务：IMU滤波、VQF/HIL ESKF、并行导航 ESKF 与控制反馈
 *
 * 整体流水线（每个 IMU 样本一轮，真实硬件为 1kHz）：
 *   等待样本(任务通知阻塞) → 滤波(陷波+低通) → 准备预测(对准/dt校验)
 *   → 运行预测(VQF 姿态 + 可选并行 ESKF) → 外部观测更新(光流/测距) → 发布控制反馈
 *
 * 双估计器分工（真实硬件模式）：
 *   - VQF：输出姿态四元数与陀螺零偏，直接作为控制环姿态源（快、稳）；
 *   - 并行 ESKF(15 状态)：只负责位置/速度等导航状态，尤其 NED-Z 定高，作诊断/定高用；
 *   二者解耦，ESKF 异常不会影响 VQF 对控制环的供给。
 */

#include "App_attitude.h"
#include "App_attitude_internal.h"
#include "App_attitude_observations.h" /* 光流/测距外部观测更新 */
#include "App_Control.h"               /* 向控制任务发布 Control_Feedback */
#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "bsp_RTT.h"
#include "bsp_timestamp.h"
#include "user_math.h"

/* 按数据源选择包含的头文件：HIL 用仿真注入，真实用 BMI088 驱动 */
#if (APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_HIL)
#include "App_attitude_hil.h"
#else
#include "modules_BMI088.h"
/* 真实模式且固定+自适应陷波都使能时，才引入自适应陷波器 */
#if ((ATTITUDE_IMU_NOTCH_ENABLE == 1U) && (ATTITUDE_IMU_ADAPTIVE_NOTCH_ENABLE == 1U))
#include "App_ImuFft.h"
#endif
#endif

/* 导航 ESKF 实例的存在条件：
 *   HIL 模式：ESKF 即唯一估计器，必然存在；
 *   真实模式：仅当并行 ESKF 使能时才存在。
 * 该实例为静态常驻对象，放入 DTCM（零等待、32B 对齐）。 */
#if ((APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_HIL) ||                                                               \
     ((APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_REAL) && (ATTITUDE_PARALLEL_ESKF_ENABLE == 1U)))
static NavESKF nav_eskf_instance APP_ATTITUDE_MEMORY_ATTRIBUTE;
#endif

/* 任务控制块、栈（静态分配，单位 word）、句柄，全部放 DTCM */
static StaticTask_t attitude_task_tcb APP_ATTITUDE_MEMORY_ATTRIBUTE;
static StackType_t                    attitude_task_stack[ATTITUDE_TASK_STACK_WORDS] APP_ATTITUDE_MEMORY_ATTRIBUTE;
static TaskHandle_t                   attitude_task_handle;

/* 姿态任务运行时状态（滤波器状态、VQF 输出缓存、对准计数、时间戳等），放 DTCM */
static AppAttitudeRuntime_t attitude_runtime APP_ATTITUDE_MEMORY_ATTRIBUTE;

/* ============================== IMU 滤波 ============================== */

/**
 * @brief 初始化全部 IMU 滤波器：三轴 PT1 低通、三轴双二阶陷波、自适应陷波跟踪器。
 *        上电时调用一次；HIL 复位时也会重新调用。
 */
static bool Attitude_InitImuFilters(void)
{
    /* 由标称周期换算采样率：1/0.001 = 1000Hz，仅真实模式陷波器需要。 */
#if ((APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_REAL) && (ATTITUDE_IMU_NOTCH_ENABLE == 1U))
    const float sample_rate_hz = 1.0f / ATTITUDE_IMU_NOMINAL_DT_S;
#endif

    /* 三个轴分别初始化（陀螺、加速度各一套） */
    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        /* PT1 一阶低通：陀螺 80Hz、加速度 30Hz（截止频率来自 config） */
        PT1_Filter_Init(&attitude_runtime.gyro_lpf[axis], ATTITUDE_GYRO_LPF_CUTOFF_HZ);
        PT1_Filter_Init(&attitude_runtime.accel_lpf[axis], ATTITUDE_ACCEL_LPF_CUTOFF_HZ);
#if ((APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_REAL) && (ATTITUDE_IMU_NOTCH_ENABLE == 1U))
        /* 双二阶陷波：初始中心频率 115Hz、Q=3，陀螺/加速度各轴一个 */
        Biquad_Notch_Init(&attitude_runtime.gyro_notch[axis], sample_rate_hz, ATTITUDE_IMU_NOTCH_CENTER_HZ,
                          ATTITUDE_IMU_NOTCH_Q);
        Biquad_Notch_Init(&attitude_runtime.accel_notch[axis], sample_rate_hz, ATTITUDE_IMU_NOTCH_CENTER_HZ,
                          ATTITUDE_IMU_NOTCH_Q);
#endif
    }

#if ((APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_REAL) && (ATTITUDE_IMU_NOTCH_ENABLE == 1U) &&                         \
     (ATTITUDE_IMU_ADAPTIVE_NOTCH_ENABLE == 1U))
    /* 用 config 里的参数组装自适应陷波配置（频带、RMS/SNR 门限、限速、估计窗长） */
    const AdaptiveNotchConfig_t config = {
        .sample_rate_hz        = sample_rate_hz,
        .initial_center_hz     = ATTITUDE_IMU_NOTCH_CENTER_HZ,
        .minimum_hz            = ATTITUDE_IMU_NOTCH_MIN_HZ,
        .maximum_hz            = ATTITUDE_IMU_NOTCH_MAX_HZ,
        .minimum_rms           = ATTITUDE_IMU_NOTCH_MIN_GYRO_RMS_RPS,
        .minimum_snr           = ATTITUDE_IMU_NOTCH_MIN_SNR,
        .maximum_slew_hz_per_s = ATTITUDE_IMU_NOTCH_MAX_SLEW_HZ_PER_S,
        .update_samples        = ATTITUDE_IMU_NOTCH_UPDATE_SAMPLES,
    };
    if (!App_ImuFft_Init(&config))
    {
        RTTERROR("[Attitude] IMU Q15 FFT pipeline init failed.");
        return false;
    }
#endif
    return true;
}

/**
 * @brief 单位四元数 → 欧拉角（roll/pitch/yaw，弧度），ZYX 内旋顺序。
 * @param quaternion 输入四元数（函数内先归一化，避免数值漂移影响结果）
 * @param euler_rad  [out] 依次为 roll(x)、pitch(y)、yaw(z)
 */
static void Attitude_QuaternionToEuler(NavQuatf quaternion, float euler_rad[3])
{
    const NavQuatf q = Math_QuaternionNormalize(quaternion);

    /* roll：由 w,x,y,z 组合求 atan2 */
    const float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    const float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    /* pitch：先求 sinp，再 clamp 到 [-1,1] 防止 asin 因浮点误差越界 */
    const float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    /* yaw */
    const float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    const float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);

    euler_rad[0] = atan2f(sinr_cosp, cosr_cosp);              /* roll  */
    euler_rad[1] = asinf(Math_ClampFloat(sinp, -1.0f, 1.0f)); /* pitch */
    euler_rad[2] = atan2f(siny_cosp, cosy_cosp);              /* yaw   */
}

/* ============================== 估计器初始化 ============================== */

#if (APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_HIL)
/**
 * @brief HIL 模式 ESKF 初始化：用仿真给定的初始状态与（按 dt 缩放的）噪声建滤波器。
 * @param initial_state 初始状态（来自仿真真值，仅用于一次性初始化）
 */
static void Attitude_InitHilEstimator(const NavESKFState *initial_state)
{
    NavESKFConfig config;
    NAV_ESKF_GetDefaultConfig(&config);

    /* 连续噪声密度离散化：单步过程噪声 = 噪声密度 × sqrt(dt)（随机游走的离散化形式） */
    config.gyro_noise  = ATTITUDE_HIL_GYRO_SAMPLE_STD_RPS * sqrtf(ATTITUDE_IMU_NOMINAL_DT_S);
    config.accel_noise = ATTITUDE_HIL_ACCEL_SAMPLE_STD_MPS2 * sqrtf(ATTITUDE_IMU_NOMINAL_DT_S);
    /* 仿真 IMU 不建模零偏漂移，随机游走置 0 */
    config.gyro_bias_rw     = 0.0f;
    config.accel_bias_rw    = 0.0f;
    config.chi_square_level = NAV_ESKF_CHI2_99; /* 卡方野值拒绝门限取 99% */

    NAV_ESKF_Init(&nav_eskf_instance, &config, initial_state);
    attitude_runtime.estimator_initialized = 1U;
}
#else
#if (ATTITUDE_PARALLEL_ESKF_ENABLE == 1U)
/**
 * @brief 真实模式并行 ESKF 初始化：用当前 VQF 已收敛的姿态/零偏给 ESKF 赋初值，
 *        保证并行 ESKF 一启动就与 VQF 姿态对齐，而不是从 0 慢慢收敛。
 */
static void Attitude_InitParallelEskf(void)
{
    NavESKFConfig config;
    NavESKFState  initial_state;

    /* VQF 输出的四元数数组(w,x,y,z 在 [0..3])转成结构体四元数 */
    const NavQuatf vqf_q = {
        .w = attitude_runtime.vqf_output.quat_ned_from_body[0],
        .x = attitude_runtime.vqf_output.quat_ned_from_body[1],
        .y = attitude_runtime.vqf_output.quat_ned_from_body[2],
        .z = attitude_runtime.vqf_output.quat_ned_from_body[3],
    };
    float euler_rad[3];

    /* 真实器件噪声/零偏随机游走参数 */
    NAV_ESKF_GetDefaultConfig(&config);
    config.gyro_noise       = ATTITUDE_REAL_GYRO_NOISE;
    config.accel_noise      = ATTITUDE_REAL_ACCEL_NOISE;
    config.gyro_bias_rw     = ATTITUDE_REAL_GYRO_BIAS_RW;
    config.accel_bias_rw    = ATTITUDE_REAL_ACCEL_BIAS_RW;
    config.chi_square_level = NAV_ESKF_CHI2_99;

    memset(&initial_state, 0, sizeof(initial_state));

    /* 用 VQF 的 roll/pitch 初始化 ESKF 姿态，yaw 强制为 0（无磁，航向从 0 起） */
    Attitude_QuaternionToEuler(vqf_q, euler_rad);
    initial_state.q_nb = Math_QuaternionFromEuler(euler_rad[0], euler_rad[1], 0.0f);

    /* 陀螺零偏初值直接继承 VQF 已估计出的零偏 */
    memcpy(initial_state.gyro_bias_rps, attitude_runtime.vqf_output.gyro_bias_rps, sizeof(initial_state.gyro_bias_rps));

    NAV_ESKF_Init(&nav_eskf_instance, &config, &initial_state);
    attitude_runtime.parallel_eskf_initialized = 1U;
}
#endif
#endif

/**
 * @brief 估计器“尝试初始化/静态对准”。在估计器尚未就绪时每个样本调用，
 *        只有满足静止条件并累计够样本后才完成初始化。
 *
 * HIL  ：优先用一次性仿真真值初始化；否则用累加平均做静态对准。
 * 真实 ：对准期间持续喂 VQF 使其内部状态预热稳定，满足静止条件后 VQF 置航向并初始化并行 ESKF。
 *
 * @return true 本次调用完成了初始化；false 仍在对准中或条件不满足
 */
static bool Attitude_TryInitialize(const float gyro_rps[3], const float accel_mps2[3])
{
#if (APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_HIL)
    NavESKFState initial_state;
    memset(&initial_state, 0, sizeof(initial_state));

    /* HIL 真值只允许作为一次性初始状态，不参与后续正常融合。 */
    if (App_Attitude_Hil_TryGetInitialState(&initial_state))
    {
        Attitude_InitHilEstimator(&initial_state);
        RTTINFO("[Attitude] HIL ESKF initialized.");
        return true;
    }
#else
    /* 真实模式静态对准期间持续运行 VQF，使其内部滤波状态稳定；VQF 未就绪则继续等 */
    if (!VqfC_Update(gyro_rps, accel_mps2, &attitude_runtime.vqf_output))
        return false;
#endif

    /* 静止判据：加速度模长接近 1g、陀螺模长足够小，且都为有限数 */
    const float accel_norm = Math_VectorNorm3(accel_mps2);
    const float gyro_norm  = Math_VectorNorm3(gyro_rps);
    if ((!isfinite(accel_norm)) || (!isfinite(gyro_norm)) || (accel_norm < ATTITUDE_ALIGNMENT_MIN_ACCEL_MPS2) ||
        (accel_norm > ATTITUDE_ALIGNMENT_MAX_ACCEL_MPS2) || (gyro_norm > ATTITUDE_ALIGNMENT_MAX_GYRO_RPS))
    {
        /* 一旦检测到运动/异常，HIL 累加窗清零、整体样本计数归零重新累计 */
#if (APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_HIL)
        memset(attitude_runtime.alignment_accel_sum, 0, sizeof(attitude_runtime.alignment_accel_sum));
        memset(attitude_runtime.alignment_gyro_sum, 0, sizeof(attitude_runtime.alignment_gyro_sum));
#endif
        attitude_runtime.alignment_sample_count = 0U;
        return false;
    }

#if (APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_HIL)
    /* HIL：累加加速度/陀螺，用于求静止区间平均 */
    for (uint8_t i = 0U; i < 3U; i++)
    {
        attitude_runtime.alignment_accel_sum[i] += accel_mps2[i];
        attitude_runtime.alignment_gyro_sum[i] += gyro_rps[i];
    }
#endif

    /* 连续静止样本数不足，继续累计（真实模式约 2000 样本≈2s） */
    if (++attitude_runtime.alignment_sample_count < ATTITUDE_ALIGNMENT_SAMPLES)
        return false;

#if (APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_HIL)
    /* ---- HIL 静态对准：用区间平均加速度求初始 roll/pitch，平均陀螺作零偏 ---- */
    float accel_average[3];
    float gyro_average[3];
    for (uint8_t i = 0U; i < 3U; i++)
    {
        accel_average[i] = attitude_runtime.alignment_accel_sum[i] / (float)attitude_runtime.alignment_sample_count;
        gyro_average[i]  = attitude_runtime.alignment_gyro_sum[i] / (float)attitude_runtime.alignment_sample_count;
    }

    const float norm = Math_VectorNorm3(accel_average);
    if ((!isfinite(norm)) || (norm < 1.0e-6f))
        return false; /* 平均加速度几乎为 0，无法定姿态 */

    /* 由重力方向反解水平姿态（机体坐标下重力指向决定 roll/pitch） */
    const float pitch  = asinf(Math_ClampFloat(accel_average[0] / norm, -1.0f, 1.0f));
    const float roll   = atan2f(-accel_average[1], -accel_average[2]);
    initial_state.q_nb = Math_QuaternionFromEuler(roll, pitch, 0.0f);
    memcpy(initial_state.gyro_bias_rps, gyro_average, sizeof(initial_state.gyro_bias_rps));

    Attitude_InitHilEstimator(&initial_state);
    RTTINFO("[Attitude] HIL static alignment completed.");
#else
    /* ---- 真实对准完成：VQF 锁定零航向，再用其姿态初始化并行 ESKF ---- */
    if (!VqfC_ZeroHeading())
        return false;
#if (ATTITUDE_PARALLEL_ESKF_ENABLE == 1U)
    Attitude_InitParallelEskf();
#endif
    attitude_runtime.estimator_initialized = 1U;
    RTTINFO("[Attitude] VQF static alignment completed.");
#endif
    return true;
}

#if (APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_HIL)
/**
 * @brief HIL 仿真请求复位时：清空运行时与 ESKF、重建滤波器、通知 HIL 层复位。
 */
static void Attitude_ResetEstimator(void)
{
    memset(&attitude_runtime, 0, sizeof(attitude_runtime));
    memset(&nav_eskf_instance, 0, sizeof(nav_eskf_instance));
    (void)Attitude_InitImuFilters();
    App_Attitude_Hil_Reset();
}
#endif

/* ============================== Control 输出 ============================== */

/**
 * @brief 把本轮估计结果组装成 Control_Feedback_t 并交给控制任务。
 *        姿态/角速度：HIL 来自 ESKF，真实来自 VQF（控制源）；
 *        位置/速度(NED)：来自并行 ESKF（仅在有效且有限时附带，并打导航有效标志）。
 *
 * @param timestamp_us 本样本 64 位微秒时间戳
 * @param gyro_rps     滤波后、尚未减零偏的陀螺角速度
 * @param dt_s         本轮实测步长(秒)
 */
static void Attitude_PublishControl(uint64_t timestamp_us, const float gyro_rps[3], float dt_s)
{
    Control_Feedback_t feedback             = {0}; /* 先整体清零，避免未赋值字段带垃圾 */
    feedback.timestamp_us                   = timestamp_us;
    feedback.last_range_fusion_timestamp_us = attitude_runtime.last_range_fusion_timestamp_us;
    feedback.dt_s                           = dt_s;

#if (APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_HIL)
    /* ---------- HIL：ESKF 是唯一估计器，姿态/零偏/位置速度全取 ESKF ---------- */
    NavESKFState state;
    NAV_ESKF_GetState(&nav_eskf_instance, &state);
    const NavQuatf q = state.q_nb;
    float          euler_rad[3];
    Attitude_QuaternionToEuler(q, euler_rad);

    feedback.q_nb[0] = q.w;
    feedback.q_nb[1] = q.x;
    feedback.q_nb[2] = q.y;
    feedback.q_nb[3] = q.z;
    feedback.yaw_rad = euler_rad[2];
    /* 输出“扣除零偏后”的角速度给控制 */
    for (uint8_t i = 0U; i < 3U; i++)
        feedback.gyro_rps[i] = gyro_rps[i] - state.gyro_bias_rps[i];
    memcpy(feedback.pos_ned_m, state.pos_ned_m, sizeof(feedback.pos_ned_m));
    memcpy(feedback.vel_ned_mps, state.vel_ned_mps, sizeof(feedback.vel_ned_mps));
    /* ESKF 预测有效才置导航有效标志 */
    if (attitude_runtime.eskf_predict_valid != 0U)
        feedback.navigation_flags |= CONTROL_NAVIGATION_ESKF_VALID;
#else
    /* ---------- 真实：姿态四元数/零偏来自 VQF（控制源） ---------- */
    const NavQuatf q = {
        .w = attitude_runtime.vqf_output.quat_ned_from_body[0],
        .x = attitude_runtime.vqf_output.quat_ned_from_body[1],
        .y = attitude_runtime.vqf_output.quat_ned_from_body[2],
        .z = attitude_runtime.vqf_output.quat_ned_from_body[3],
    };
    float euler_rad[3];
    Attitude_QuaternionToEuler(q, euler_rad);

    feedback.q_nb[0] = q.w;
    feedback.q_nb[1] = q.x;
    feedback.q_nb[2] = q.y;
    feedback.q_nb[3] = q.z;
    feedback.yaw_rad = euler_rad[2];
    /* 控制用角速度 = 滤波陀螺 - VQF 估计零偏 */
    for (uint8_t i = 0U; i < 3U; i++)
        feedback.gyro_rps[i] = gyro_rps[i] - attitude_runtime.vqf_output.gyro_bias_rps[i];

#if (ATTITUDE_PARALLEL_ESKF_ENABLE == 1U)
    /* 并行 ESKF 只补充位置/速度导航量：预测有效 + 数值有限才拷贝并置有效标志 */
    if (attitude_runtime.eskf_predict_valid != 0U)
    {
        NavESKFState state;
        NAV_ESKF_GetState(&nav_eskf_instance, &state);
        if (Math_Vector3IsFinite(state.pos_ned_m) && Math_Vector3IsFinite(state.vel_ned_mps))
        {
            memcpy(feedback.pos_ned_m, state.pos_ned_m, sizeof(feedback.pos_ned_m));
            memcpy(feedback.vel_ned_mps, state.vel_ned_mps, sizeof(feedback.vel_ned_mps));
            feedback.navigation_flags |= CONTROL_NAVIGATION_ESKF_VALID;
        }
    }
    /* 曾经成功融合过测距，置“测距已融合”标志，供定高通道判断 */
    if (attitude_runtime.last_range_fusion_timestamp_us != 0ULL)
        feedback.navigation_flags |= CONTROL_NAVIGATION_RANGE_FUSED;
#endif
#endif

    /* 交给控制任务（内部为短临界区整体快照，非队列） */
    Control_SetFeedback(&feedback);
}

/* ============================== IMU 输入 ============================== */

#if (APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_HIL)
/**
 * @brief HIL 版等待并取出一帧“完整 IMU 样本”。
 *        阻塞等待 HIL 传感器数据；检测到复位请求则复位估计器；IMU 字段不齐则跳过本轮。
 * @return true 得到可用样本；false 本轮无有效样本（主循环 continue）
 */
static bool APP_ATTITUDE_HOT_CODE Attitude_WaitForImuSample(AppAttitudeImuSample_t *sample, HIL_Sensor_Data_t *sensor)
{
    if ((sample == NULL) || (sensor == NULL) || (!App_Attitude_Hil_WaitForSensor(sensor)))
        return false;

    /* 仿真端请求复位：清估计器，本轮不产出样本 */
    if ((sensor->fields_updated & HIL_SENSOR_UPDATED_RESET) != 0U)
    {
        Attitude_ResetEstimator();
        return false;
    }
    /* 陀螺/加速度字段必须都更新过，才算一帧完整 IMU */
    if (!App_Attitude_Hil_HasCompleteImu(sensor->fields_updated))
        return false;

    /* 时间戳优先用仿真时间戳，缺失时退回本机接收时间戳 */
    sample->timestamp_us = (sensor->timestamp_us != 0ULL) ? sensor->timestamp_us : sensor->rx_timestamp_us;
    memcpy(sample->gyro_rps, sensor->accel, sizeof(sample->gyro_rps)); /* 注：参数名 gyro，拷陀螺 */
    memcpy(sample->accel_mps2, sensor->accel, sizeof(sample->accel_mps2));
    return true;
}
#else
/**
 * @brief 真实版等待并取出一帧 IMU 样本。
 *        ulTaskNotifyTake 在任务通知上【阻塞】（由 BMI088 加速度 DMA 完成 ISR 唤醒），
 *        被唤醒后从双缓冲读取并解析 BMI088 数据。
 * @return true 得到可用样本；false 虚假唤醒或取数失败
 */
static bool APP_ATTITUDE_HOT_CODE Attitude_WaitForImuSample(AppAttitudeImuSample_t *sample)
{
    BMI088_Data_t imu;
    /* pdTRUE：每次取走通知值并清零；portMAX_DELAY：无限等待，不轮询、不耗 CPU */
    if ((sample == NULL) || (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == 0U))
        return false;
    /* 从双缓冲“已写完那块”解析出最新 IMU（内部 1 次结构体拷贝） */
    if (BMI088_GetData(&imu) == 0U)
        return false;

    sample->timestamp_us = imu.Bim088_Timestamp; /* DRDY 时刻锁存的 64µs 时间戳 */
    memcpy(sample->gyro_rps, imu.gyro, sizeof(sample->gyro_rps));
    memcpy(sample->accel_mps2, imu.accel, sizeof(sample->accel_mps2));
    return true;
}
#endif

/**
 * @brief 对一帧 IMU 做数字滤波：先非阻塞消费 SensorHub 给出的 FFT 结果，
 *        再把陷波前陀螺样本写入 SPSC 缓冲区，最后逐轴走“Notch → PT1”。
 *        该函数不再执行 FFT，因此 1 kHz 姿态热路径的计算量保持有界。
 */
static void APP_ATTITUDE_HOT_CODE Attitude_FilterImuSample(AppAttitudeImuSample_t *sample)
{
    /* dt 默认用标称 1ms；若能从相邻时间戳算出合法实测 dt，则用实测值（PT1 系数依赖 dt） */
    float dt_s = ATTITUDE_IMU_NOMINAL_DT_S;
    if ((sample == NULL) || (sample->timestamp_us == 0ULL))
        return;

    if ((attitude_runtime.previous_filter_timestamp_us != 0ULL) &&
        (sample->timestamp_us > attitude_runtime.previous_filter_timestamp_us))
    {
        const float measured_dt_s = (float)(sample->timestamp_us - attitude_runtime.previous_filter_timestamp_us) *
                                    1.0e-6f;
        /* 实测 dt 必须有限且落在 [0.2ms,10ms] 窗口内才采用，否则沿用标称值 */
        if (isfinite(measured_dt_s) && (measured_dt_s >= ATTITUDE_IMU_MIN_DT_S) &&
            (measured_dt_s <= ATTITUDE_IMU_MAX_DT_S))
            dt_s = measured_dt_s;
    }
    attitude_runtime.previous_filter_timestamp_us = sample->timestamp_us;

#if ((APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_REAL) && (ATTITUDE_IMU_NOTCH_ENABLE == 1U) &&                         \
     (ATTITUDE_IMU_ADAPTIVE_NOTCH_ENABLE == 1U))
    AppImuFftResult_t fft_result;
    if (App_ImuFft_TryGetResult(&fft_result) && (fft_result.notch.center_changed_mask != 0U))
    {
        const float sample_rate_hz = 1.0f / ATTITUDE_IMU_NOMINAL_DT_S;
        /* 滤波器状态只由 Attitude 修改，SensorHub 只发布不可变频率快照。 */
        for (uint8_t axis = 0U; axis < 3U; axis++)
        {
            if ((fft_result.notch.center_changed_mask & (uint8_t)(1U << axis)) != 0U)
            {
                Biquad_Notch_Update(&attitude_runtime.gyro_notch[axis], sample_rate_hz,
                                    fft_result.notch.center_hz[axis], ATTITUDE_IMU_NOTCH_Q);
                Biquad_Notch_Update(&attitude_runtime.accel_notch[axis], sample_rate_hz,
                                    fft_result.notch.center_hz[axis], ATTITUDE_IMU_NOTCH_Q);
            }
        }
    }
    /* 只做紧凑样本入队；队列满时管线会记录丢样并在 SensorHub 中重置分析窗。 */
    (void)App_ImuFft_PushSample(sample->timestamp_us, sample->gyro_rps);
#endif

    /* 逐轴滤波：先陷波去窄带振动，再低通去宽带高频噪声 */
    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
#if ((APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_REAL) && (ATTITUDE_IMU_NOTCH_ENABLE == 1U))
        sample->gyro_rps[axis]   = Biquad_Notch_Apply(&attitude_runtime.gyro_notch[axis], sample->gyro_rps[axis]);
        sample->accel_mps2[axis] = Biquad_Notch_Apply(&attitude_runtime.accel_notch[axis], sample->accel_mps2[axis]);
#endif
        sample->gyro_rps[axis]   = PT1_Filter_Apply(&attitude_runtime.gyro_lpf[axis], sample->gyro_rps[axis], dt_s);
        sample->accel_mps2[axis] = PT1_Filter_Apply(&attitude_runtime.accel_lpf[axis], sample->accel_mps2[axis], dt_s);
    }
}

/* ============================== 估计器运行 ============================== */

/**
 * @brief 预测前准备：未初始化则先尝试对准；已初始化则做时间戳单调性检查并算出预测 dt。
 * @param dt_s [out] 本轮预测步长(秒)
 * @return true 可以进入预测；false 本轮应跳过（正在对准 / 时间戳异常）
 */
static bool APP_ATTITUDE_HOT_CODE Attitude_PreparePrediction(const AppAttitudeImuSample_t *sample, float *dt_s)
{
    /* 输入与有限性保护 */
    if ((sample == NULL) || (dt_s == NULL) || (sample->timestamp_us == 0ULL) ||
        (!Math_Vector3IsFinite(sample->gyro_rps)) || (!Math_Vector3IsFinite(sample->accel_mps2)))
        return false;

    /* 估计器还没初始化：尝试静态对准，同时记录本帧时间戳，返回 false（本轮不预测） */
    if (attitude_runtime.estimator_initialized == 0U)
    {
        if (Attitude_TryInitialize(sample->gyro_rps, sample->accel_mps2))
            attitude_runtime.previous_imu_timestamp_us = sample->timestamp_us;
        return false;
    }

    /* 时间戳必须严格递增；非递增（重放/乱序）时更新基准并丢弃本轮 */
    if (sample->timestamp_us <= attitude_runtime.previous_imu_timestamp_us)
    {
        attitude_runtime.previous_imu_timestamp_us = sample->timestamp_us;
        return false;
    }

    /* 微秒时间戳差 → 秒，并做合法窗口校验 */
    *dt_s = (float)(sample->timestamp_us - attitude_runtime.previous_imu_timestamp_us) * 1.0e-6f;
    attitude_runtime.previous_imu_timestamp_us = sample->timestamp_us;
    return isfinite(*dt_s) && (*dt_s >= ATTITUDE_IMU_MIN_DT_S) && (*dt_s <= ATTITUDE_IMU_MAX_DT_S);
}

/**
 * @brief 执行一步预测。
 *   HIL ：只跑 ESKF 预测，并按本轮 dt 实时缩放过程噪声，再更新重力相关项；
 *   真实：先跑 VQF 更新（控制姿态源）；若并行 ESKF 已初始化，再对 ESKF 做同样的预测+重力更新。
 * @return true 预测有效（VQF 成功 / ESKF 预测成功）；false 本轮预测失败应跳过
 */
static bool APP_ATTITUDE_HOT_CODE Attitude_RunPrediction(const AppAttitudeImuSample_t *sample, float dt_s)
{
#if (APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_HIL)
    /* 过程噪声随实际 dt 缩放：sqrt(dt) 离散化，保证不同步长下噪声强度一致 */
    const float sqrt_dt               = sqrtf(dt_s);
    nav_eskf_instance.cfg.gyro_noise  = ATTITUDE_HIL_GYRO_SAMPLE_STD_RPS * sqrt_dt;
    nav_eskf_instance.cfg.accel_noise = ATTITUDE_HIL_ACCEL_SAMPLE_STD_MPS2 * sqrt_dt;

    const NavImuSample prediction = {
        .gyro_rps   = {sample->gyro_rps[0], sample->gyro_rps[1], sample->gyro_rps[2]},
        .accel_mps2 = {sample->accel_mps2[0], sample->accel_mps2[1], sample->accel_mps2[2]},
        .dt_s       = dt_s,
    };

    /* ESKF 一步预测（状态传播 + 协方差传播） */
    attitude_runtime.eskf_predict_valid = NAV_ESKF_Predict(&nav_eskf_instance, &prediction) ? 1U : 0U;
    if (attitude_runtime.eskf_predict_valid == 0U)
        return false;
    /* 重力/参考向量更新（为后续加速度观测做准备） */
    (void)NAV_ESKF_UpdateGravity(&nav_eskf_instance, &prediction);
    return true;
#else
    /* 真实模式：VQF 每帧更新，是控制姿态的权威来源；失败则本轮不继续 */
    if (!VqfC_Update(sample->gyro_rps, sample->accel_mps2, &attitude_runtime.vqf_output))
        return false;

    attitude_runtime.eskf_predict_valid = 0U;
#if (ATTITUDE_PARALLEL_ESKF_ENABLE == 1U)
    /* 并行 ESKF 仅在已初始化后运行，其成败不影响 VQF，故只记录有效标志 */
    if (attitude_runtime.parallel_eskf_initialized != 0U)
    {
        const NavImuSample prediction = {
            .gyro_rps   = {sample->gyro_rps[0], sample->gyro_rps[1], sample->gyro_rps[2]},
            .accel_mps2 = {sample->accel_mps2[0], sample->accel_mps2[1], sample->accel_mps2[2]},
            .dt_s       = dt_s,
        };

        const bool valid                    = NAV_ESKF_Predict(&nav_eskf_instance, &prediction);
        attitude_runtime.eskf_predict_valid = valid ? 1U : 0U;
        if (valid)
            (void)NAV_ESKF_UpdateGravity(&nav_eskf_instance, &prediction);
    }
#endif
    return true; /* VQF 成功即视为本轮预测成功 */
#endif
}

/* ============================== 主任务 ============================== */

/**
 * @brief 姿态任务主体：静态创建、无限循环。事件驱动（被 IMU DMA 完成通知唤醒），
 *        每轮严格按“取数→滤波→准备→预测→观测更新→发布”的顺序执行。
 */
static void APP_ATTITUDE_HOT_CODE App_Attitude_Task(void *argument)
{
    AppAttitudeImuSample_t sample = {0};
    float                  dt_s;
    (void)argument; /* 不使用任务入参 */

#if (APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_HIL)
    HIL_Sensor_Data_t sensor = {0};
    RTTINFO("[Attitude] HIL ESKF task started.");
#else
    RTTINFO("[Attitude] VQF + navigation ESKF task started.");
#endif

    for (;;)
    {
        /* 1) 阻塞等待并取出一帧 IMU（真实：任务通知；HIL：仿真数据） */
#if (APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_HIL)
        if (!Attitude_WaitForImuSample(&sample, &sensor))
            continue;
#else
        if (!Attitude_WaitForImuSample(&sample))
            continue;
#endif

        /* 2) 数字滤波（消费自适应频率 + 固定陷波 + PT1）；FFT 本体在 SensorHub 分轴执行。 */
        Attitude_FilterImuSample(&sample);

        /* 3)+4) 预测前准备与一步预测；任一不满足则本轮结束等下一帧 */
        const bool predict_valid = Attitude_PreparePrediction(&sample, &dt_s) && Attitude_RunPrediction(&sample, dt_s);
        if (!predict_valid)
        {
            continue;
        }

        /* 5) 外部观测更新（慢传感器融合进 ESKF） */
#if (APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_HIL)
        /* HIL：把仿真的磁/气压/GPS 等观测喂给 ESKF */
        App_Attitude_Hil_UpdateObservations(&nav_eskf_instance, &sensor, sample.timestamp_us,
                                            attitude_runtime.estimator_initialized != 0U);
#else
#if (ATTITUDE_PARALLEL_ESKF_ENABLE == 1U)
        /* 真实：仅在 ESKF 预测有效时，非阻塞消费光流/测距（内部 SubGetMessage 超时0） */
        if (attitude_runtime.eskf_predict_valid != 0U)
            App_Attitude_Observations_Update(&nav_eskf_instance, &attitude_runtime, sample.gyro_rps, dt_s);
#endif
#endif

        /* 6) 组装并发布控制反馈给控制任务 */
        Attitude_PublishControl(sample.timestamp_us, sample.gyro_rps, dt_s);
    }
}

/* ============================== 模块初始化 ============================== */

/**
 * @brief 注册数据源相关资源：HIL 初始化仿真通道；真实初始化光流/测距订阅者。
 */
static bool Attitude_RegisterSource(void)
{
#if (APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_HIL)
    return App_Attitude_Hil_Init();
#else
    return App_Attitude_Observations_Init();
#endif
}

/**
 * @brief 静态创建姿态任务，并在真实模式下把本任务句柄注册给 BMI088 驱动，
 *        使 IMU DMA 完成 ISR 能通过任务通知唤醒本任务。
 */
static bool Attitude_CreateTask(void)
{
#if (configSUPPORT_STATIC_ALLOCATION == 1)
    /* 静态任务：传入静态栈与 TCB，优先级 54，高于控制任务且不占用 FreeRTOS 堆。 */
    attitude_task_handle = xTaskCreateStatic(App_Attitude_Task, "Attitude", ATTITUDE_TASK_STACK_WORDS, NULL,
                                             ATTITUDE_TASK_PRIORITY, attitude_task_stack, &attitude_task_tcb);
#else
#error "App_Attitude requires configSUPPORT_STATIC_ALLOCATION == 1"
#endif

    if (attitude_task_handle == NULL)
    {
        RTTERROR("[Attitude] Task create failed.");
        return false;
    }

#if (APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_REAL)
    /* 关键：把姿态任务句柄交给 BMI088，DMA 完成中断里 vTaskNotifyGiveFromISR 唤醒它 */
    BMI088_RegisterReadyTask(attitude_task_handle);
#endif
    return true;
}

/**
 * @brief 姿态模块对外初始化入口（系统启动阶段调用一次）。
 *        已初始化则直接返回成功（幂等）。顺序：清零运行时 → 建滤波器 → VQF 初始化 → 注册数据源 → 建任务。
 */
bool App_Attitude_Init(void)
{
    if (attitude_task_handle != NULL)
        return true; /* 防重复初始化 */

    memset(&attitude_runtime, 0, sizeof(attitude_runtime));
    if (!Attitude_InitImuFilters())
        return false;

#if (APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_REAL)
    /* 真实模式先初始化 VQF（标称 1kHz 采样率） */
    if (!VqfC_Init(ATTITUDE_IMU_NOMINAL_DT_S))
    {
        RTTERROR("[Attitude] VQF init failed.");
        return false;
    }
#endif

    if ((!Attitude_RegisterSource()) || (!Attitude_CreateTask()))
        return false;

    RTTINFO("[Attitude] Init success.");
    return true;
}
