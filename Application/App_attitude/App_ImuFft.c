/**
 * @file App_ImuFft.c
 * @brief 使用静态 SPSC 环形缓冲区把 IMU 频谱分析从 Attitude 迁移到 SensorHub
 *
 * 数据流：
 *   Attitude --原始陀螺样本--> bsp_ringbuffer --分轴 Q15 FFT--> SensorHub
 *            <--最新陷波参数--------- Message Center <-------------+
 *
 * Attitude 只负责压入样本和读取结果，耗时较大的 FFT 不进入 1 kHz 姿态热路径。
 */

#include "App_ImuFft.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "bsp_ringbuffer.h"
#include "modules_Message_center.h"

/*
 * bsp_ringbuffer 按字节管理，容量必须是 2 的幂。
 * 2 KB 可容纳 85 个 24 字节样本，能覆盖约 85 ms 的 1 kHz IMU 数据积压。
 */
#define APP_IMU_FFT_QUEUE_BYTES       2048U
/* 限制 SensorHub 单轮最多搬运的样本数，防止一次执行时间失去上界。 */
#define APP_IMU_FFT_MAX_DRAIN_PER_RUN 16U
#define APP_IMU_FFT_RESULT_TOPIC      "imu_fft_result"
/* FFT 工作区和队列放入 DTCM，避免 D-Cache 一致性问题并降低访问抖动。 */
#define APP_IMU_FFT_MEMORY_ATTRIBUTE  __attribute__((section(".dtcm"), aligned(32)))

#if ((APP_IMU_FFT_QUEUE_BYTES & (APP_IMU_FFT_QUEUE_BYTES - 1U)) != 0U)
#error "APP_IMU_FFT_QUEUE_BYTES must be a power of two"
#endif

/** @brief Attitude 写入、SensorHub 读取的紧凑样本。 */
typedef struct
{
    uint64_t timestamp_us;                                /**< IMU 采样时刻，而不是入队时刻 */
    uint32_t sequence;                                    /**< 连续递增序号，用于发现中途丢样 */
    float    gyro_rps[ADAPTIVE_NOTCH_AXIS_COUNT];          /**< 陷波前的三轴角速度，单位 rad/s */
} AppImuFftSample_t;

/** @brief FFT 管线全部静态状态；初始化后不再申请动态内存。 */
typedef struct
{
    AdaptiveNotchTracker_t tracker;      /**< 仅由 SensorHub 访问的频谱跟踪器 */
    RingBuffer_t           sample_queue; /**< Attitude 写、SensorHub 读的 SPSC 队列 */
    /* RingBuffer 使用的静态字节池。 */
    uint8_t      sample_pool[APP_IMU_FFT_QUEUE_BYTES];
    Publisher_t  *result_publisher;  /**< SensorHub 使用的结果发布端 */
    Subscriber_t *result_subscriber; /**< Attitude 使用的结果订阅端 */

    uint32_t producer_sequence;                           /**< Attitude 侧发送序号 */
    uint32_t expected_sequence;                           /**< SensorHub 期望收到的下一序号 */
    uint64_t previous_timestamp_us;                       /**< 上一个已消费样本的采样时刻 */
    uint32_t result_generation;                           /**< 已完成的三轴分析代数 */
    uint32_t nominal_interval_us;                         /**< 按配置采样率计算的标称周期 */
    uint8_t  initialized;                                 /**< 防止重复注册消息主题 */
} AppImuFftState_t;

static AppImuFftState_t app_imu_fft APP_IMU_FFT_MEMORY_ATTRIBUTE;
static char                         app_imu_fft_topic_name[] = APP_IMU_FFT_RESULT_TOPIC;

volatile AppImuFftStatus_t app_imu_fft_status;

/**
 * @brief 判断当前样本能否与前一帧组成连续的等间隔采样序列
 *
 * 序号可发现队列满、非法输入等造成的显式丢样；时间戳可发现传感器停顿、
 * 调试器停核等没有反映在序号中的长间隔。任一条件异常都应丢弃旧 FFT 窗。
 */
static bool App_ImuFft_SampleIsContinuous(const AppImuFftSample_t *sample)
{
    bool continuous = true;
    if ((app_imu_fft.expected_sequence != 0U) && (sample->sequence != app_imu_fft.expected_sequence))
    {
        continuous = false;
    }

    if (app_imu_fft.previous_timestamp_us != 0ULL)
    {
        const uint64_t minimum_interval_us = (uint64_t)app_imu_fft.nominal_interval_us / 2ULL;
        const uint64_t maximum_interval_us = (uint64_t)app_imu_fft.nominal_interval_us * 2ULL;
        if ((sample->timestamp_us <= app_imu_fft.previous_timestamp_us) ||
            ((sample->timestamp_us - app_imu_fft.previous_timestamp_us) < minimum_interval_us) ||
            ((sample->timestamp_us - app_imu_fft.previous_timestamp_us) > maximum_interval_us))
        {
            continuous = false;
        }
    }

    app_imu_fft.expected_sequence     = sample->sequence + 1U;
    app_imu_fft.previous_timestamp_us = sample->timestamp_us;
    return continuous;
}

bool App_ImuFft_Init(const AdaptiveNotchConfig_t *config)
{
    /* 姿态估计器复位时可能再次调用初始化，已完成时直接返回。 */
    if (app_imu_fft.initialized != 0U)
    {
        return true;
    }
    if ((config == NULL) ||
        (!RingBuffer_Init(&app_imu_fft.sample_queue, app_imu_fft.sample_pool, sizeof(app_imu_fft.sample_pool))) ||
        (!Adaptive_Notch_Init(&app_imu_fft.tracker, config)))
    {
        return false;
    }

    /*
     * SensorHub 发布低频分析结果，Attitude 以 0 超时方式读取；
     * 两个任务不直接共享可变的滤波器状态。
     */
    app_imu_fft.result_publisher  = PubRegister(app_imu_fft_topic_name, (uint16_t)sizeof(AppImuFftResult_t));
    app_imu_fft.result_subscriber = SubRegister(app_imu_fft_topic_name, (uint16_t)sizeof(AppImuFftResult_t));
    if ((app_imu_fft.result_publisher == NULL) || (app_imu_fft.result_subscriber == NULL))
    {
        return false;
    }

    memset((void *)&app_imu_fft_status, 0, sizeof(app_imu_fft_status));
    app_imu_fft.nominal_interval_us = (uint32_t)(1000000.0f / config->sample_rate_hz + 0.5f);
    app_imu_fft.initialized         = 1U;
    return true;
}

bool App_ImuFft_PushSample(uint64_t timestamp_us, const float gyro_rps[3])
{
    if ((app_imu_fft.initialized == 0U) || (gyro_rps == NULL))
    {
        return false;
    }

    /* 序号在尝试入队前递增：本次失败后，消费者可从下一帧的序号跳变发现丢样。 */
    AppImuFftSample_t sample = {
        .timestamp_us = timestamp_us,
        .sequence     = ++app_imu_fft.producer_sequence,
    };
    for (uint32_t axis = 0U; axis < ADAPTIVE_NOTCH_AXIS_COUNT; axis++)
    {
        if (!isfinite(gyro_rps[axis]))
        {
            app_imu_fft_status.dropped_samples++;
            return false;
        }
        sample.gyro_rps[axis] = gyro_rps[axis];
    }

    /* RingBuffer_Push 只在整帧空间足够时写入，不会留下半个 IMU 样本。 */
    if ((timestamp_us == 0ULL) ||
        (!RingBuffer_Push(&app_imu_fft.sample_queue, &sample, (uint32_t)sizeof(sample))))
    {
        app_imu_fft_status.dropped_samples++;
        return false;
    }

    app_imu_fft_status.pushed_samples++;
    return true;
}

void App_ImuFft_ProcessOneStep(void)
{
    if (app_imu_fft.initialized == 0U)
    {
        return;
    }

    /*
     * 第一步：从环形缓冲区向 FFT 窗口搬运连续样本。
     * 分析期间暂停搬运，使 X/Y/Z 三轴始终使用完全相同的一组样本。
     */
    if (!Adaptive_Notch_IsAnalysisPending(&app_imu_fft.tracker))
    {
        for (uint32_t count = 0U; count < APP_IMU_FFT_MAX_DRAIN_PER_RUN; count++)
        {
            AppImuFftSample_t sample;
            if (RingBuffer_Pop(&app_imu_fft.sample_queue, &sample, (uint32_t)sizeof(sample)) != sizeof(sample))
            {
                break;
            }

            if (!App_ImuFft_SampleIsContinuous(&sample))
            {
                /* 不连续数据会破坏频谱含义，因此清空旧窗口，从当前样本重新收集。 */
                Adaptive_Notch_ResetSamples(&app_imu_fft.tracker);
                app_imu_fft_status.discontinuity_resets++;
            }
            (void)Adaptive_Notch_PushSample(&app_imu_fft.tracker, sample.timestamp_us, sample.gyro_rps);
            app_imu_fft_status.consumed_samples++;

            if (Adaptive_Notch_IsAnalysisPending(&app_imu_fft.tracker))
            {
                /* 窗口已满，立即停止搬运，避免覆盖本轮三轴分析所需的数据。 */
                break;
            }
        }
    }

    /*
     * 第二步：每次只计算 X/Y/Z 中的一个轴。
     * 三次 SensorHub 调度完成一个窗口，避免三轴 FFT 连续占用 CPU。
     */
    if (Adaptive_Notch_IsAnalysisPending(&app_imu_fft.tracker))
    {
        AdaptiveNotchResult_t completed_result;
        const bool            complete = Adaptive_Notch_ProcessNextAxis(&app_imu_fft.tracker, &completed_result);
        app_imu_fft_status.axis_fft_count++;

        if (complete)
        {
            /* 第三个轴完成后发布一次完整结果，Attitude 下一周期再非阻塞应用。 */
            AppImuFftResult_t result = {
                .notch      = completed_result,
                .generation = ++app_imu_fft.result_generation,
            };
            app_imu_fft_status.completed_windows++;
            if (PubPushMessage(app_imu_fft.result_publisher, &result) != 0U)
            {
                app_imu_fft_status.published_results++;
            }
        }
    }
}

bool App_ImuFft_TryGetResult(AppImuFftResult_t *result)
{
    /* 0 超时读取：没有新结果时立即返回，不能阻塞 1 kHz 姿态任务。 */
    return (app_imu_fft.initialized != 0U) && (result != NULL) &&
           (SubGetMessage(app_imu_fft.result_subscriber, result, 0U) != 0U);
}
