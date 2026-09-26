#ifndef APP_ATTITUDE_CONFIG_H
#define APP_ATTITUDE_CONFIG_H

#include "App_TaskPriorities.h"

/* ============================================================================
 * 文件：App_attitude_config.h
 * 作用：姿态/导航任务（App_attitude）的全部编译期可调参数集中配置。
 * 分组：数据源选择 → 任务(栈/优先级/内存段) → IMU 滤波/陷波 → 静态对准
 *       → ESKF 噪声 → 真实传感器观测门限 → HIL 仿真观测参数。
 * 约定：所有参数都用 #ifndef 包裹，允许在 CMake/编译选项里从外部覆盖而不改本文件。
 * ========================================================================== */

/* ============================== 数据源 ============================== */

/* 姿态数据来源枚举（编译期二选一，不占运行时开销） */
#define APP_ATTITUDE_SOURCE_HIL  0U   /* 硬件在环：IMU/传感器数据由 HIL 仿真注入 */
#define APP_ATTITUDE_SOURCE_REAL 1U   /* 真实硬件：数据来自板载 BMI088 等真实传感器 */

/* 默认数据源：真实硬件。可被外部 -DAPP_ATTITUDE_SOURCE=... 覆盖 */
#ifndef APP_ATTITUDE_SOURCE
#define APP_ATTITUDE_SOURCE APP_ATTITUDE_SOURCE_REAL
#endif

/* 编译期合法性校验：数据源既不是 HIL 也不是 REAL 就直接 #error，防止拼错宏 */
#if ((APP_ATTITUDE_SOURCE != APP_ATTITUDE_SOURCE_HIL) && (APP_ATTITUDE_SOURCE != APP_ATTITUDE_SOURCE_REAL))
#error "APP_ATTITUDE_SOURCE is invalid."
#endif

/* ============================== 任务配置 ============================== */

/* 姿态任务栈大小，单位是“字(word)”而非字节。
 * Cortex-M7 上 1 word = 4 字节，1536 word = 6144 字节。
 * 姿态任务同时跑 VQF + 15 状态 ESKF，局部变量/矩阵较多，故栈给得较大；
 * 使用 xTaskCreateStatic 静态分配，不占用 FreeRTOS 堆。 */
#ifndef ATTITUDE_TASK_STACK_WORDS
#define ATTITUDE_TASK_STACK_WORDS 1536U
#endif

/* 姿态任务优先级高于控制任务：每次姿态解算完成后通知控制任务运行。 */
#ifndef ATTITUDE_TASK_PRIORITY
#define ATTITUDE_TASK_PRIORITY APP_TASK_PRIORITY_ATTITUDE
#endif

/* 姿态任务的大块常驻数据（TCB、ESKF 状态、滤波器结构）放置段：
 *   section(".dtcm") -> 放 DTCM（紧耦合内存，CPU 零等待访问、不经过 D-Cache/DMA 域）；
 *   aligned(32)      -> 32 字节对齐，正好等于 Cortex-M7 一条 D-Cache Cache Line，
 *                       便于做整行 Cache 维护、避免相邻变量伪共享。 */
#ifndef APP_ATTITUDE_MEMORY_ATTRIBUTE
#define APP_ATTITUDE_MEMORY_ATTRIBUTE __attribute__((section(".dtcm"), aligned(32)))
#endif

/* 热函数放置段：section(".itcm_text") -> 代码进 ITCM，取指零等待，
 * 用于 1kHz 热路径上的函数（滤波、预测、观测更新等），压低执行耗时与抖动。 */
#ifndef APP_ATTITUDE_HOT_CODE
#define APP_ATTITUDE_HOT_CODE __attribute__((section(".itcm_text")))
#endif

/* ============================== IMU 与滤波 ============================== */

/* 相邻两帧 IMU 的时间步长 dt 合法窗口（单位：秒）。
 * 名义值 0.001s = 1kHz；小于 0.2ms 视为异常重入，大于 10ms 视为丢帧/任务延迟，
 * 超出窗口的样本不用于数值积分（防止 dt 异常导致滤波器发散）。 */
#define ATTITUDE_IMU_MIN_DT_S       0.0002f
#define ATTITUDE_IMU_MAX_DT_S       0.0100f
#define ATTITUDE_IMU_NOMINAL_DT_S   0.0010f   /* 标称采样周期 1ms（1kHz） */

/* 一阶/双二阶低通截止频率（Hz）：
 * 陀螺 80Hz 保留较高带宽以满足控制响应；加速度 30Hz 更低，因为加速度只用于
 * 估计重力方向/水平，需要更平滑以抑制机体振动和平动加速度污染。 */
#define ATTITUDE_GYRO_LPF_CUTOFF_HZ   80.0f
#define ATTITUDE_ACCEL_LPF_CUTOFF_HZ  30.0f

/* ---- 固定陷波器（Notch）：基础振动抑制，中心频率固定 ---- */
/* 固定 Notch 作为基础抑制，自适应算法只更新中心频率。 */
#define ATTITUDE_IMU_NOTCH_ENABLE     1U        /* 使能固定陷波 */
#define ATTITUDE_IMU_NOTCH_CENTER_HZ  115.0f    /* 固定陷波中心频率（Hz），对准主要电机振动频点 */
#define ATTITUDE_IMU_NOTCH_Q          3.0f      /* 品质因数 Q：越大陷波越窄、相位影响越小 */

/* ---- 自适应陷波器：在线估计振动主频并跟踪移动 ---- */
#define ATTITUDE_IMU_ADAPTIVE_NOTCH_ENABLE   1U       /* 使能自适应中心频率更新 */
#define ATTITUDE_IMU_NOTCH_UPDATE_SAMPLES    128U     /* 每 128 个新样本启动一轮三轴分轴 Q15 FFT */
#define ATTITUDE_IMU_NOTCH_MIN_HZ            90.0f    /* 自适应中心频率搜索下限（Hz） */
#define ATTITUDE_IMU_NOTCH_MAX_HZ            180.0f   /* 搜索上限（Hz），框定电机转速对应频带 */
#define ATTITUDE_IMU_NOTCH_MIN_GYRO_RMS_RPS  0.25f    /* 陀螺 RMS 超过该值(rad/s)才认为“确有振动”，避免静止时乱跟踪 */
#define ATTITUDE_IMU_NOTCH_MIN_SNR           6.0f     /* 信噪比门限：谱峰不够突出就不更新中心频率 */
#define ATTITUDE_IMU_NOTCH_MAX_SLEW_HZ_PER_S 30.0f    /* 中心频率最大变化率(Hz/s)，限速防止频率跳变造成滤波不稳 */

/* ---- 上电静态对准（粗对准/初始姿态与零偏估计）---- */
/* 静态对准要求飞机基本静止且加速度模长接近 1g。 */
#define ATTITUDE_ALIGNMENT_SAMPLES         2000U   /* 对准采样点数：1kHz 下约采集 2 秒 */
#define ATTITUDE_ALIGNMENT_MIN_ACCEL_MPS2  7.5f    /* 加速度模长下限(m/s²)，1g≈9.81，允许误差区间 */
#define ATTITUDE_ALIGNMENT_MAX_ACCEL_MPS2  12.0f   /* 加速度模长上限(m/s²)，超出说明在运动/受冲击，不满足静止 */
#define ATTITUDE_ALIGNMENT_MAX_GYRO_RPS    0.15f   /* 陀螺模长上限(rad/s)，超过视为机体在转动，不能做静态对准 */

/* ============================== ESKF ============================== */
/* 真实硬件模式下的过程噪声/传感器噪声系数（用于 ESKF 的 Q 矩阵标定）。 */

#define ATTITUDE_REAL_GYRO_NOISE      0.015f    /* 陀螺角度随机游走系数（噪声密度） */
#define ATTITUDE_REAL_ACCEL_NOISE     0.20f     /* 加速度计噪声密度 */
#define ATTITUDE_REAL_GYRO_BIAS_RW    0.0005f   /* 陀螺零偏随机游走强度（bias 随时间漂移的过程噪声） */
#define ATTITUDE_REAL_ACCEL_BIAS_RW   0.01f     /* 加速度零偏随机游走强度 */

/* HIL 模式下“虚拟 IMU”注入的采样噪声标准差（仿真用，数值比真实器件小） */
#define ATTITUDE_HIL_GYRO_SAMPLE_STD_RPS    0.0015f  /* 陀螺注入噪声标准差 rad/s */
#define ATTITUDE_HIL_ACCEL_SAMPLE_STD_MPS2  0.025f   /* 加速度注入噪声标准差 m/s² */

/* 真实模式下 VQF 负责姿态控制，ESKF 负责位置/速度与定高导航状态。
 * =1：VQF 与 15 状态 ESKF 并行运行——VQF 输出直接喂控制环（稳、快），
 *     ESKF 只做导航/诊断（水平速度、NED-Z 定高），两者解耦，互不拖累控制。 */
#define ATTITUDE_PARALLEL_ESKF_ENABLE 1U

/* ============================== 真实传感器观测 ============================== */
/* 两路外部观测总开关：光流（水平速度）与激光测距（垂直高度） */
#define ATTITUDE_ENABLE_FLOW  1U   /* 使能 MTF02 光流融合 */
#define ATTITUDE_ENABLE_RANGE 1U   /* 使能 TFmini 测距融合（NED-Z 定高） */

/* ---- 光流观测通用参数 ---- */
#define ATTITUDE_FLOW_STD_MPS  0.15f   /* 光流水平速度量测噪声标准差(m/s)，进 ESKF 的 R 矩阵 */
#define ATTITUDE_MTF02_MIN_QUALITY 30U  /* 光流画面质量下限，低于则认为纹理不足、丢弃 */
#define ATTITUDE_MTF02_CHECK_STATUS 1U  /* =1 时校验模块状态字；=0 跳过状态字检查 */
#define ATTITUDE_MTF02_VALID_STATUS 0U  /* “数据有效”对应的状态字取值 */

/* ---- MICOLINK 协议（速度型光流）轴映射与缩放 ---- */
#define ATTITUDE_MTF02_FLOW_X_SOURCE 0U    /* 机体 vx 取原始数组的第 0 路 */
#define ATTITUDE_MTF02_FLOW_Y_SOURCE 1U    /* 机体 vy 取原始数组的第 1 路 */
#define ATTITUDE_MTF02_FLOW_X_SCALE_MPS 0.01f  /* x 标定系数（原始量×高度→m/s） */
#define ATTITUDE_MTF02_FLOW_Y_SCALE_MPS 0.01f  /* y 标定系数 */

/* ---- MSP_V2 协议（角速度型光流，需陀螺旋转补偿）轴映射与缩放 ----
 * 注意这里 X/Y 源下标对调、且 X 缩放为负：是由模块安装朝向与机体坐标系定义
 * 共同决定的轴方向/轴序修正，换安装方向时需要重新标定这几个宏。 */
#define ATTITUDE_MTF02_MSP_FLOW_X_SOURCE 1U     /* MSP 机体 vx 取原始第 1 路（轴交换） */
#define ATTITUDE_MTF02_MSP_FLOW_Y_SOURCE 0U     /* MSP 机体 vy 取原始第 0 路 */
#define ATTITUDE_MTF02_MSP_FLOW_X_SCALE  (-1.0f) /* x 方向取反（安装朝向修正） */
#define ATTITUDE_MTF02_MSP_FLOW_Y_SCALE  1.0f
#define ATTITUDE_MTF02_MSP_MIN_DT_US 5000U      /* MSP 两帧间隔下限 5ms（200Hz），过短视为异常 */
#define ATTITUDE_MTF02_MSP_MAX_DT_US 100000U    /* 间隔上限 100ms（10Hz），过长则旋转补偿积分不可靠，丢弃 */

/* ---- 光流可用高度窗口与同步门限 ---- */
#define ATTITUDE_MTF02_MIN_HEIGHT_M    0.08f      /* 最低有效高度(m)：贴地时光流/测距不可用 */
#define ATTITUDE_MTF02_MAX_HEIGHT_M    6.00f      /* 最高有效高度(m)：过高光流精度恶化 */
#define ATTITUDE_MTF02_MAX_RANGE_AGE_US 100000ULL /* 光流帧与其内部测距帧最大允许时间差 100ms，超时认为高度不同步 */

/* ---- TFmini Plus 激光测距参数 ---- */
#define ATTITUDE_TFMINI_DISTANCE_SCALE_M 0.01f  /* 原始距离单位 cm → m 的换算系数（×0.01） */
#define ATTITUDE_RANGE_STD_M 0.05f              /* 测距量测噪声标准差(m)，进 R 矩阵 */
#define ATTITUDE_RANGE_GROUND_DOWN_M 0.0f       /* 传感器到机体参考点的安装偏置(m)，这里取 0 */

/* ============================== HIL 观测 ============================== */
/* 以下仅在 APP_ATTITUDE_SOURCE_HIL 时使用：500Hz 六自由度半物理仿真的观测噪声/门限。 */

#define ATTITUDE_HIL_USE_TRUTH_INITIAL_STATE 1U  /* =1：用仿真真值直接初始化 ESKF 状态，跳过粗对准 */

#define ATTITUDE_MAG_DECLINATION_RAD 0.0f  /* 当地磁偏角(弧度)，HIL 默认 0（不对磁北做修正） */

/* 磁力计量测噪声与磁场强度有效性窗口（用于判断磁数据是否可信） */
#define ATTITUDE_HIL_MAG_HEADING_STD_RAD (2.0f * 0.01745329251994329577f) /* 航向噪声 2°，×π/180 转弧度 */
#define ATTITUDE_MAG_FIELD_MIN_GAUSS 0.10f   /* 总磁场强度下限(高斯)，过低视为磁干扰 */
#define ATTITUDE_MAG_FIELD_MAX_GAUSS 1.00f   /* 总磁场强度上限(高斯) */
#define ATTITUDE_MAG_HORIZONTAL_MIN_GAUSS 0.05f /* 水平分量下限，过小无法解算航向 */

/* 气压计：先采 N 个样本求基准（初始气压高度零点），量测标准差 1m */
#define ATTITUDE_HIL_BARO_REFERENCE_SAMPLES 50U
#define ATTITUDE_HIL_BARO_STD_M 1.0f

/* GPS 各通道量测噪声标准差（仿真设定值） */
#define ATTITUDE_HIL_GPS_HORIZONTAL_STD_M 0.35f  /* 水平位置标准差(m) */
#define ATTITUDE_HIL_GPS_VERTICAL_STD_M   0.70f  /* 垂直位置标准差(m)，通常比水平差 */
#define ATTITUDE_HIL_GPS_VEL_XY_STD_MPS   0.05f  /* 水平速度标准差(m/s) */
#define ATTITUDE_HIL_GPS_VEL_Z_STD_MPS    0.08f  /* 垂直速度标准差(m/s) */

/* 各 HIL 观测的最小融合间隔（微秒），用于限频、防止同一/过密样本反复更新 */
#define ATTITUDE_HIL_MAG_MIN_INTERVAL_US  10000ULL  /* 磁力计最快 10ms 融合一次（100Hz） */
#define ATTITUDE_HIL_BARO_MIN_INTERVAL_US 20000ULL  /* 气压计最快 20ms 融合一次（50Hz） */

#endif /* APP_ATTITUDE_CONFIG_H */
