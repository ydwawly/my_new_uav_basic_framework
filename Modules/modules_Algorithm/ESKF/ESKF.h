/** @file ESKF.h @brief 导航误差状态卡尔曼滤波器的公共类型与接口。 */

#ifndef STM32H743_UAV_FLIGHT_CONTROLLER_ESKF_H
#define STM32H743_UAV_FLIGHT_CONTROLLER_ESKF_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "stm32h7xx.h"
#include "arm_math.h"
#include "user_math.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ==========================================================================
 *                             坐标系与基本约定
 * ==========================================================================
 * 1. 机体系 (Body Frame, B):      FRD (Front-Right-Down，前-右-下)。
 * 2. 导航系 (Navigation Frame, N): NED (North-East-Down，北-东-下)。
 * 3. 姿态四元数 (q_nb):           表示从机体系(B)旋转到导航系(N)的旋转，满足 v_n = R_nb * v_b。
 * 4. IMU 比力 (Specific Force):   加速度计测量值，单位 m/s^2。注意：比力包含抵抗重力的向上载荷，
 *                                 不能在运动学积分前提前减去重力，需转移到导航系后再做重力补偿。
 * ========================================================================== */

/* 滤波器核心维度定义 */
#define NAV_ESKF_STATE_DIM   15U      /* 误差状态总维数 */
#define NAV_ESKF_MAX_OBS_DIM 3U       /* 单次观测组的最大维数（针对三轴传感器） */
#define NAV_ESKF_G           9.80665f /* 标准重力加速度 (m/s^2)，NED系下指向 +Down */

/* 矩阵一维化索引辅助宏 (Row-Major 行主序) */
#define NAV_ESKF_IDX(r, c) ((r) * NAV_ESKF_STATE_DIM + (c))

/*
 * 预测工作区内存段属性宏
 * --------------------------------------------------------------------------
 * 稀疏协方差传播只保留一个 15x15 中间矩阵（约 0.9 KB）。当前 STM32H7
 * 工程将其放入 DTCM；DTCM 仅供 CPU 直接访问，不可用于 DMA，本工作区符合要求。
 */
#ifndef NAV_ESKF_WORKSPACE_ATTRIBUTE
#define NAV_ESKF_WORKSPACE_ATTRIBUTE __attribute__((section(".dtcm"), aligned(32)))
#endif

/* 1 kHz 预测和高频观测更新属于实时热路径，显式放入 64 KB ITCM。 */
#ifndef NAV_ESKF_HOT_CODE
#define NAV_ESKF_HOT_CODE __attribute__((section(".itcm_text")))
#endif

/* ==========================================================================
 *                             枚举与状态定义
 * ========================================================================== */

/* 15 维误差状态向量的起始切片索引：
 * [δp(0:2), δv(3:5), δθ(6:8), δbg(9:11), δba(12:14)]^T */
typedef enum
{
    NAV_ESKF_IX_POS = 0U, /* 位置误差状态 δp_ned 起始索引 */
    NAV_ESKF_IX_VEL = 3U, /* 速度误差状态 δv_ned 起始索引 */
    NAV_ESKF_IX_ATT = 6U, /* 姿态误差状态 δθ (右乘小旋转向量) 起始索引 */
    NAV_ESKF_IX_BG  = 9U, /* 陀螺仪零偏误差 δbg 起始索引 */
    NAV_ESKF_IX_BA  = 12U /* 加速度计零偏误差 δba 起始索引 */
} NavESKFStateIndex;

/* 卡方检验 (Chi-Square Test) 显著性水平
 * 用于马氏距离 (Mahalanobis Distance) / 归一化新息平方 (NIS) 门限判定 */
typedef enum
{
    NAV_ESKF_CHI2_DISABLED = 0, /* 关闭卡方拒绝（仅用于初调或调试场景） */
    NAV_ESKF_CHI2_95,           /* 95% 置信度：误拒绝概率约 5%，门限较严格 */
    NAV_ESKF_CHI2_99,           /* 99% 置信度：默认推荐，误拒绝概率约 1% */
    NAV_ESKF_CHI2_999           /* 99.9% 置信度：门限宽松，仅过滤极端野值 */
} NavESKFChiSquareLevel;

/* ==========================================================================
 *                             基础数据结构
 * ========================================================================== */

/*
     * 导航模块沿用 NavQuatf 名称，但底层类型统一由 user_math 提供。
     * 这样应用、ESKF 和其他姿态算法共享同一套四元数定义与数学实现。
     */
typedef MathQuaternionf NavQuatf;

/* ESKF 滤波器整定配置结构体 */
typedef struct
{
    /* --- 连续时间白噪声功率谱密度 (PSD) 与随机游走 --- */
    float32_t gyro_noise;    /* 陀螺仪白噪声密度，单位: rad/s/sqrt(Hz) */
    float32_t accel_noise;   /* 加速度计白噪声密度，单位: m/s^2/sqrt(Hz) */
    float32_t gyro_bias_rw;  /* 陀螺零偏随机游走密度，单位: (rad/s^2)/sqrt(Hz) */
    float32_t accel_bias_rw; /* 加计零偏随机游走密度，单位: (m/s^3)/sqrt(Hz) */

    /* --- 初始协方差参数 (初始 1 Sigma 标准差) --- */
    float32_t init_pos_std;        /* 初始位置标准差，单位: m */
    float32_t init_vel_std;        /* 初始速度标准差，单位: m/s */
    float32_t init_att_std_rad;    /* 初始姿态标准差，单位: rad */
    float32_t init_gyro_bias_std;  /* 初始陀螺零偏标准差，单位: rad/s */
    float32_t init_accel_bias_std; /* 初始加计零偏标准差，单位: m/s^2 */

    /* --- 异常检测与非线性修正逻辑阈值 --- */
    NavESKFChiSquareLevel chi_square_level;           /* 卡方检验置信度水平 */
    float32_t             att_reset_jacobian_min_rad; /* 触发姿态协方差重置 Jacobian (G_theta) 的最小姿态修正角 (rad) */
    float32_t             min_range_cos_tilt;    /* 测距仪向下更新允许的最小机体倾角余弦 (例如 0.5 对应最大倾角 60°) */
    float32_t             min_heading_cos_pitch; /* 航向更新允许的最小俯仰角余弦，避免接近垂直姿态时航向奇异 */

    /* --- 重力方向观测（只约束 Roll/Pitch，不观测 Yaw） --- */
    float32_t gravity_direction_std_rad;    /* 单次重力方向观测 1 Sigma，单位: rad */
    float32_t gravity_accel_norm_gate_mps2; /* 允许 |norm(accel)-g| 的最大偏差，单位: m/s^2 */
    float32_t gravity_gyro_gate_rps;        /* 允许融合的最大去零偏角速度模长，单位: rad/s */
    float32_t gravity_max_speed_mps;        /* 保留兼容字段；重力方向更新不再依赖惯性速度估计 */
    float32_t gravity_max_innovation_rad;   /* 重力方向最大几何新息角，单位: rad */
    float32_t gravity_min_stable_time_s;    /* 门限连续满足后开始融合的等待时间，单位: s */
    float32_t gravity_update_interval_s;    /* 重力观测最小更新间隔，单位: s */
} NavESKFConfig;

/* 滤波器名义状态向量 (Nominal State Vector) */
typedef struct
{
    float32_t pos_ned_m[3];       /* NED 导航系位置 [N, E, D]，单位: m */
    float32_t vel_ned_mps[3];     /* NED 导航系地速 [vN, vE, vD]，单位: m/s */
    NavQuatf  q_nb;               /* 机体系至导航系的姿态四元数 */
    float32_t gyro_bias_rps[3];   /* 机体系陀螺仪零偏估计，单位: rad/s */
    float32_t accel_bias_mps2[3]; /* 机体系加速度计零偏估计，单位: m/s^2 */
} NavESKFState;

/**
 * @brief 一阶状态转移矩阵中三个非单位 3x3 子块
 *
 * 完整 Fd 为 15x15，但除单位阵、δp<-δv 和 δθ<-δbg 外，只有以下三个
 * 子块非零。显式保存稀疏块可避免构造和计算大量恒为零的矩阵元素。
 */
typedef struct
{
    float32_t vel_att[9];        /* F(δv, δθ) = -R_nb[f_b]x dt */
    float32_t vel_accel_bias[9]; /* F(δv, δba) = -R_nb dt */
    float32_t att_att_delta[9];  /* F(δθ, δθ)-I = -[ω]x dt */
} NavESKFTransitionBlocks;

/* 预测阶段专用静态工作区，避免在高频姿态任务栈上分配 0.9 KB 临时矩阵。 */
typedef struct
{
    float32_t FP[NAV_ESKF_STATE_DIM * NAV_ESKF_STATE_DIM]; /* 稀疏计算得到的 Fd * P */
} NavESKFPredictWorkspace;

/* ==========================================================================
 *                             传感器观测数据格式
 * ========================================================================== */

/* IMU 同步采样输入结构体 */
typedef struct
{
    float32_t gyro_rps[3];   /* 已完成轴系映射的陀螺仪角速度 (FRD)，单位: rad/s */
    float32_t accel_mps2[3]; /* 已完成轴系映射的加速度计比力 (FRD)，单位: m/s^2 */
    float32_t dt_s;          /* 本次预测采样间隔时间，单位: s */
} NavImuSample;

/* GNSS / GPS 观测结构体 */
typedef struct
{
    bool      use_position;   /* 是否启用当前 GPS 位置更新 */
    bool      use_velocity;   /* 是否启用当前 GPS 速度更新 */
    float32_t pos_ned_m[3];   /* GNSS 天线中心 NED 位置，单位: m */
    float32_t vel_ned_mps[3]; /* GNSS 测速 NED 速度，单位: m/s */
    float32_t pos_std_m[3];   /* 位置测量 1 Sigma 标准差，单位: m */
    float32_t vel_std_mps[3]; /* 速度测量 1 Sigma 标准差，单位: m/s */
} NavGpsObservation;

/* 光流传感器 (Optical Flow) 平面测速观测 */
typedef struct
{
    float32_t vel_body_xy_mps[2]; /* 已扣除陀螺仪旋转分量并完成尺度缩放的 FRD 平面速度 [vx, vy] */
    float32_t std_mps[2];         /* 速度测量 1 Sigma 标准差，单位: m/s */
} NavFlowObservation;

/* 气压计测高观测 */
typedef struct
{
    float32_t altitude_up_m; /* 气压高度（向上为正），必须与 NED 系原点在同一起飞基准面上 */
    float32_t std_m;         /* 测高 1 Sigma 标准差，单位: m */
} NavBaroObservation;

/* 一维测距传感器（如激光或超声波下视测距） */
typedef struct
{
    float32_t range_m;       /* 沿机体 +Z 轴（机体下方）的原始测距值，单位: m */
    float32_t ground_down_m; /* 测距仪所对准地面的 NED 下向坐标 D（平坦地面通常为 0.0f） */
    float32_t std_m;         /* 测距 1 Sigma 标准差，单位: m */
} NavRangeObservation;

/* 外部视觉或动捕系统 (VIO / Mocap) 观测 */
typedef struct
{
    bool      use_position;   /* 是否融合视觉位置 */
    bool      use_velocity;   /* 是否融合视觉速度 */
    bool      use_attitude;   /* 是否融合完整视觉姿态；仅当来源真实观测到 Roll/Pitch/Yaw 时启用 */
    float32_t pos_ned_m[3];   /* 视觉系统给出的 NED 位置，单位: m */
    float32_t vel_ned_mps[3]; /* 视觉系统给出的 NED 速度，单位: m/s */
    NavQuatf  q_nb;           /* 视觉系统给出的姿态四元数 */
    float32_t pos_std_m[3];   /* 位置测量 1 Sigma 标准差，单位: m */
    float32_t vel_std_mps[3]; /* 速度测量 1 Sigma 标准差，单位: m/s */
    float32_t att_std_rad[3]; /* 姿态测量 1 Sigma 标准差（各轴等效旋转角），单位: rad */
} NavVisionObservation;

/* 绝对航向角观测，可由双天线 GNSS、外部视觉航向或已完成倾斜补偿的磁航向提供 */
typedef struct
{
    float32_t heading_rad; /* NED 航向角，指北为 0，偏东为正，范围允许超出 [-pi, pi] */
    float32_t std_rad;     /* 航向测量 1 Sigma 标准差，单位: rad */
} NavHeadingObservation;

/* 三轴磁力计航向观测
 * 输入必须已经完成零偏、软硬铁标定与传感器轴系到 FRD 的映射，本接口只负责倾斜补偿和航向融合。 */
typedef struct
{
    float32_t mag_body_gauss[3];          /* FRD 机体系磁场 [X前, Y右, Z下]，单位: gauss */
    float32_t declination_rad;            /* 当地磁偏角，东偏为正；输出航向将对齐真北 */
    float32_t heading_std_rad;            /* 倾斜补偿后磁航向 1 Sigma 标准差，单位: rad */
    float32_t field_norm_min_gauss;       /* 总磁场模长下限；<= 0 表示不启用该检查 */
    float32_t field_norm_max_gauss;       /* 总磁场模长上限；<= 0 表示不启用该检查 */
    float32_t horizontal_field_min_gauss; /* 倾斜补偿后水平磁场最小模长；<= 0 时仅执行数值安全检查 */
} NavMagObservation;

/* 滤波器运行状况与卡方检验统计数据 */
typedef struct
{
    uint32_t  accepted_observation_groups; /* 成功通过卡方检验并执行更新的观测组次数 */
    uint32_t  rejected_observation_groups; /* 因 NIS 超限被卡方检验剔除的野值观测次数 */
    uint32_t  numerical_failures;          /* 遇到矩阵非正定、计算溢出或分解失败的次数 */
    uint32_t  applied_scalar_rows;         /* 累计完成更新的标量等价观测维数 */
    float32_t last_nis;                    /* 最新一笔处理观测的归一化新息平方 (NIS) */
    float32_t last_chi_square_threshold;   /* 最新一笔观测对应自由度下的卡方理论门限 */
    uint8_t   last_dof;                    /* 最新一笔观测的自由度 (Degree of Freedom) */

    uint32_t  gravity_gate_rejections;      /* 因动态门限未满足而跳过的重力观测次数 */
    uint32_t  gravity_updates_accepted;     /* 成功融合的重力方向观测次数 */
    uint32_t  gravity_updates_rejected;     /* 因几何新息或 NIS 超限拒绝的重力观测次数 */
    float32_t last_gravity_accel_norm_mps2; /* 最近重力门控使用的加速度模长 */
    float32_t last_gravity_gyro_norm_rps;   /* 最近重力门控使用的去零偏角速度模长 */
    float32_t last_gravity_speed_norm_mps;  /* 最近一次门控时的估计速度模长，仅用于诊断 */
    float32_t last_gravity_innovation_rad;  /* 最近重力方向与预测方向夹角 */
} NavESKFStats;

/* ==========================================================================
 *                             ESKF 主滤波器句柄
 * ========================================================================== */
typedef struct
{
    NavESKFConfig cfg;   /* 滤波器配置参数 */
    NavESKFState  state; /* 当前系统名义状态估计 (Nominal State) */
    NavESKFStats  stats; /* 运行状态统计图表 */

    float32_t gravity_stable_time_s;        /* 当前连续满足重力动态门限的时间 */
    float32_t gravity_update_accumulator_s; /* 距离上次重力方向更新的累计时间 */

    /* 连续误差状态协方差矩阵 P (15x15 浮点数组，对称正定)
     * 作为实例持久化数据保存于句柄内，临时计算矩阵已移至工作区 */
    float32_t P[NAV_ESKF_STATE_DIM * NAV_ESKF_STATE_DIM];
} NavESKF;

/* ==========================================================================
 *                             导出 API 函数声明
 * ========================================================================== */

/**
 * @brief  获取参数默认配置（适用于中小微型多旋翼或飞翼）
 * @param  cfg  指向配置结构体的指针
 */
void NAV_ESKF_GetDefaultConfig(NavESKFConfig *cfg);

/**
 * @brief  初始化 ESKF 滤波器实例
 * @param  eskf           指向 ESKF 句柄的指针
 * @param  cfg            传入的配置，若为 NULL 则自动载入默认配置
 * @param  initial_state  初始名义状态，若为 NULL 则默认全 0 且四元数设为单位四元数
 */
void NAV_ESKF_Init(NavESKF *eskf, const NavESKFConfig *cfg, const NavESKFState *initial_state);

/**
 * @brief  完全重置滤波器状态与协方差矩阵 P
 * @param  eskf           指向 ESKF 句柄的指针
 * @param  initial_state  重置后的目标名义状态
 */
void NAV_ESKF_Reset(NavESKF *eskf, const NavESKFState *initial_state);

/**
 * @brief  IMU 驱动的运动学预测步 (Time Update / Prediction)
 * @note   该函数内部使用静态单例工作区，不可重入，必须由唯一的飞控导航任务单线程调用
 * @param  eskf  指向 ESKF 句柄的指针
 * @param  imu   高频同步 IMU 采样数据
 * @retval bool  预测成功返回 true，出现非法的 dt 返回 false
 */
bool NAV_ESKF_Predict(NavESKF *eskf, const NavImuSample *imu);

/**
 * @brief 以归一化加速度方向作为重力观测，严格门控后约束 Roll/Pitch
 * @note  该观测不提供 Yaw 信息；必须在 Predict 之后使用同一帧 IMU 数据调用。
 * @retval bool 本次确实完成融合返回 true，处于门控等待或拒绝时返回 false
 */
bool NAV_ESKF_UpdateGravity(NavESKF *eskf, const NavImuSample *imu);

/**
 * @brief  融合 GNSS/GPS 观测数据 (Measurement Update)
 */
bool NAV_ESKF_UpdateGps(NavESKF *eskf, const NavGpsObservation *obs);

/**
 * @brief  融合光流传感器平面测速观测数据
 */
bool NAV_ESKF_UpdateFlow(NavESKF *eskf, const NavFlowObservation *obs);

/**
 * @brief  融合气压计测高数据
 */
bool NAV_ESKF_UpdateBaro(NavESKF *eskf, const NavBaroObservation *obs);

/**
 * @brief  融如下向激光或超声波测距仪数据
 */
bool NAV_ESKF_UpdateRange(NavESKF *eskf, const NavRangeObservation *obs);

/**
 * @brief  融合一维绝对航向角观测
 * @note   使用右乘姿态误差下的精确航向 Jacobian，不可简单写成 H_yaw = [0, 0, 1]
 */
bool NAV_ESKF_UpdateHeading(NavESKF *eskf, const NavHeadingObservation *obs);

/**
 * @brief  融合校准后的三轴磁力计并更新航向角
 * @note   仅构造一维航向观测，不会把磁力计误当作完整三轴姿态观测
 */
bool NAV_ESKF_UpdateMag(NavESKF *eskf, const NavMagObservation *obs);

/**
 * @brief  融合视觉 Odom / VIO / Mocap 位姿观测数据
 */
bool NAV_ESKF_UpdateVision(NavESKF *eskf, const NavVisionObservation *obs);

/**
 * @brief  线程安全地获取当前滤波器的完整名义状态
 */
void NAV_ESKF_GetState(const NavESKF *eskf, NavESKFState *out_state);

/**
 * @brief  将当前四元数转换并输出标准的 3-2-1 航空欧拉角 (FRD 到 NED)
 * @param  roll   横滚角输出指针 (rad)，向右倾斜为正，范围 [-pi, pi]
 * @param  pitch  俯仰角输出指针 (rad)，抬头为正，范围 [-pi/2, pi/2]
 * @param  yaw    偏航角/航向角输出指针 (rad)，偏东为正，范围 [-pi, pi]
 */
void NAV_ESKF_GetEulerRad(const NavESKF *eskf, float32_t *roll, float32_t *pitch, float32_t *yaw);

#ifdef __cplusplus
}
#endif
#endif /* STM32H743_UAV_FLIGHT_CONTROLLER_ESKF_H */
