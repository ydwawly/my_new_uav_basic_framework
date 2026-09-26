/**
 * @file Adaptive_Notch.h
 * @brief 基于三轴陀螺 Q15 频谱的增量式自适应陷波中心频率跟踪器
 *
 * 模块只负责估计每个轴的主振动频率；实际 IIR 系数更新由调用方完成。
 */
#ifndef ADAPTIVE_NOTCH_H
#define ADAPTIVE_NOTCH_H

#include <stdbool.h>
#include <stdint.h>

#include "arm_math.h"

#define ADAPTIVE_NOTCH_FFT_SIZE         512U
#define ADAPTIVE_NOTCH_AXIS_COUNT       3U
#define ADAPTIVE_NOTCH_RFFT_OUTPUT_SIZE (2U * ADAPTIVE_NOTCH_FFT_SIZE)

/* BMI088 量程为 ±2000 dps（约 ±34.91 rad/s）。Q15 满量程留到 40 rad/s，
 * 避免满量程转动时截幅；量化分辨率约 0.00122 rad/s，远低于当前 RMS 门限。 */
#define ADAPTIVE_NOTCH_Q15_FULL_SCALE_RPS 40.0f

/** @brief 自适应频率搜索、可信度判定和变化率限制参数。 */
typedef struct
{
    float    sample_rate_hz;        /**< 输入采样率，单位 Hz */
    float    initial_center_hz;     /**< 上电初始陷波中心频率，单位 Hz */
    float    minimum_hz;            /**< 频率搜索下限，单位 Hz */
    float    maximum_hz;            /**< 频率搜索上限，单位 Hz */
    float    minimum_rms;           /**< 允许更新前的最小信号 RMS，单位 rad/s */
    float    minimum_snr;           /**< 谱峰相对背景噪声的最小信噪比 */
    float    maximum_slew_hz_per_s; /**< 中心频率最大跟踪速度，单位 Hz/s */
    uint32_t update_samples;        /**< 两次频谱分析之间至少累计的样本数 */
} AdaptiveNotchConfig_t;

/** @brief 三轴都分析完成后生成的一份不可变结果快照。 */
typedef struct
{
    uint64_t timestamp_us;                         /**< 分析窗最新样本时间戳 */
    float    center_hz[ADAPTIVE_NOTCH_AXIS_COUNT]; /**< 三轴当前陷波中心频率 */
    float    peak_snr[ADAPTIVE_NOTCH_AXIS_COUNT];  /**< 三轴本轮谱峰与背景功率比 */
    uint8_t  tracking_valid_mask;                  /**< bit0..2：本轮该轴 RMS/SNR 有效 */
    uint8_t  center_changed_mask;                  /**< bit0..2：本轮该轴中心频率变化 */
    uint8_t  reserved[2];
} AdaptiveNotchResult_t;

/**
 * @brief 三轴采样窗口、FFT 工作区及最近一次跟踪结果
 *
 * 该结构由一个任务独占，不需要互斥保护。samples 是持续更新的滑动窗；
 * fft_input、fft_output 和 spectrum_power 是三个轴依次复用的临时工作区，
 * 因而不会同时保存三份 FFT 中间结果。
 */
typedef struct
{
    arm_rfft_instance_q15 fft;    /**< CMSIS-DSP Q15 实数 FFT 实例 */
    AdaptiveNotchConfig_t config; /**< 初始化时复制的参数，运行中只读 */
    q15_t                 samples[ADAPTIVE_NOTCH_AXIS_COUNT][ADAPTIVE_NOTCH_FFT_SIZE]; /**< 三轴 Q15 滑动采样窗 */
    q15_t                 window[ADAPTIVE_NOTCH_FFT_SIZE];                             /**< 预计算的 Q15 Hann 窗 */
    q15_t                 fft_input[ADAPTIVE_NOTCH_FFT_SIZE];                          /**< 单轴加窗后的 FFT 输入 */
    q15_t                 fft_output[ADAPTIVE_NOTCH_RFFT_OUTPUT_SIZE];                 /**< 单轴 RFFT 复数输出 */
    float                 spectrum_power[ADAPTIVE_NOTCH_FFT_SIZE / 2U];                /**< 搜索频带内的功率谱 */
    float                 center_hz[ADAPTIVE_NOTCH_AXIS_COUNT];                        /**< 当前三轴中心频率 */
    float                 peak_snr[ADAPTIVE_NOTCH_AXIS_COUNT];                         /**< 最近一轮三轴谱峰 SNR */
    uint32_t              update_count[ADAPTIVE_NOTCH_AXIS_COUNT];                     /**< 三轴有效跟踪次数 */
    uint32_t              write_index;                               /**< 下一帧样本写入滑动窗的位置 */
    uint32_t              sample_count;                              /**< 当前有效样本数，最大为 FFT_SIZE */
    uint32_t              samples_since_update;                      /**< 距离上轮分析新增的样本数 */
    uint64_t              analysis_timestamp_us;                     /**< 当前分析窗最后一个样本的时间戳 */
    uint8_t               initialized;                               /**< FFT 实例与窗口已初始化 */
    uint8_t               tracking_valid[ADAPTIVE_NOTCH_AXIS_COUNT]; /**< 三轴最近一次跟踪是否可信 */
    uint8_t               analysis_pending;                          /**< 已锁定分析窗，等待完成三个轴 */
    uint8_t               next_analysis_axis;                        /**< 下一次要处理的轴：0=X、1=Y、2=Z */
    uint8_t               analysis_changed_mask;                     /**< 本轮中心频率发生变化的轴位图 */
} AdaptiveNotchTracker_t;

/**
 * @brief 校验参数、初始化 FFT 与 Hann 窗，并复位跟踪结果。
 * @return true 初始化成功
 */
bool Adaptive_Notch_Init(AdaptiveNotchTracker_t *tracker, const AdaptiveNotchConfig_t *config);

/**
 * @brief 因输入溢出或序号/时间戳不连续而废弃当前分析窗。
 * @note 保留已跟踪的中心频率，新窗重新收满前仍使用旧陷波参数。
 */
void Adaptive_Notch_ResetSamples(AdaptiveNotchTracker_t *tracker);

/**
 * @brief 向环形窗推入一帧三轴陀螺样本，仅做定点量化和 O(1) 存储。
 * @return true 样本已接收；false 参数无效或上一轮三轴分析尚未完成
 */
bool Adaptive_Notch_PushSample(AdaptiveNotchTracker_t *tracker, uint64_t timestamp_us, const float gyro_rps[3]);

/** @brief 查询是否有待计算的轴，用于暂停向分析窗灌入后续样本。 */
bool Adaptive_Notch_IsAnalysisPending(const AdaptiveNotchTracker_t *tracker);

/**
 * @brief 本次最多执行一个轴的 Hann 加窗、Q15 RFFT、谱峰/SNR 计算。
 * @param completed_result [out] 仅在三轴均完成时填充。
 * @return true 三轴本轮分析已全部完成；false 无待处理数据或仍有其他轴待处理
 */
bool Adaptive_Notch_ProcessNextAxis(AdaptiveNotchTracker_t *tracker, AdaptiveNotchResult_t *completed_result);

#endif /* ADAPTIVE_NOTCH_H */
