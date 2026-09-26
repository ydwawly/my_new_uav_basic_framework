/* ============================================================================
 * 文件：App_attitude_observations.c
 * 作用：姿态任务中的【外部观测更新】模块（只服务于并行运行的 15 状态 ESKF）。
 *
 * 数据来源：通过静态消息中心（Pub-Sub）订阅两路低速传感器 Topic：
 *   - MTF02 光流（含光流速度/角速度 + 模块自带测距）  -> 用于水平速度观测
 *   - TFmini Plus 激光测距                            -> 用于对地高度（NED-Z）观测
 *
 * 设计要点：
 *   1. 本模块只在「真实传感器源 + 并行 ESKF 使能」时编译，纯仿真或只跑 VQF 时整段裁掉；
 *   2. 全部用 SubGetMessage(..., 0U)【非阻塞】读取最新一帧，读不到立即返回，绝不拖慢 1kHz 主循环；
 *   3. 用时间戳去重（同一帧只融合一次），用多道门限（高度、质量、状态、时间新鲜度）把关，
 *      只有全部通过才构造观测并交给 ESKF 更新，避免脏数据污染滤波器；
 *   4. VQF 始终是控制用姿态源，这里的 ESKF 仅作导航/诊断，二者解耦。
 * ========================================================================== */

#include "App_attitude_observations.h"
#include <math.h>          /* isfinite()：浮点有限性检查，拦截 NaN / Inf */
#include "bsp_RTT.h"       /* SEGGER RTT 日志：初始化失败时打印错误 */
#include "modules_Message_center.h"  /* 静态 Pub-Sub：SubRegister / SubGetMessage */

/* 光流融合总开关：为 1 才订阅并处理 MTF02 */
#if (ATTITUDE_ENABLE_FLOW == 1U)
#include "modules_mtf02.h"
#endif

/* 测距融合总开关：为 1 才订阅并处理 TFmini Plus */
#if (ATTITUDE_ENABLE_RANGE == 1U)
#include "mudules_TFmini_Plus.h"
#endif

/* 最外层条件：必须同时满足
 *   (1) 姿态源是真实硬件（而非仿真/HIL 注入）；
 *   (2) 并行 ESKF 使能。
 * 否则本文件内所有观测逻辑都不编译（VQF 单跑时不需要这些外部观测）。 */
#if ((APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_REAL) && (ATTITUDE_PARALLEL_ESKF_ENABLE == 1U))

/* ============================ 光流（MTF02）部分 ============================ */
#if (ATTITUDE_ENABLE_FLOW == 1U)

/* 光流 Topic 的订阅者句柄（模块内静态，初始化时注册一次） */
static Subscriber_t *flow_subscriber;

/**
 * @brief 累加机体角速度（陀螺）用于光流的“旋转补偿”。
 *
 * 背景：光流传感器测到的画面移动 = 飞机平移 + 飞机自身转动 两部分叠加。
 *       MSP_V2 协议只给“角速度型光流”，其中混有机体旋转分量，必须把这部分
 *       用陀螺积分估计出来并扣除，才能得到纯平移速度。
 *       做法：在两次光流帧之间，对陀螺 x/y 轴角速度按时间做矩形积分（累加）。
 *
 * @param runtime  姿态运行时结构（保存积分结果）
 * @param gyro_rps 当前周期三轴陀螺角速度，单位 rad/s
 * @param dt_s     本周期时间步长，单位 s
 */
static void Attitude_AccumulateFlowGyro(AppAttitudeRuntime_t *runtime, const float gyro_rps[3], float dt_s)
{
    /* 入参合法性：空指针、非有限数、非正步长直接放弃本次积分 */
    if ((gyro_rps == NULL) || (!isfinite(gyro_rps[0])) || (!isfinite(gyro_rps[1])) || (!isfinite(dt_s)) || (dt_s <= 0.0f)) return;

    /* 角位移增量 = 角速度 × 步长，逐周期累加（roll/pitch 两轴即可，偏航不影响下视光流的平移估计） */
    runtime->flow_gyro_integral_rad[0] += gyro_rps[0] * dt_s;
    runtime->flow_gyro_integral_rad[1] += gyro_rps[1] * dt_s;
    /* 同时累计积分区间总时长，供后续求“区间平均角速度” */
    runtime->flow_gyro_interval_s += dt_s;
}

/**
 * @brief 一次光流融合完成后，清零陀螺积分累加器，为下一帧重新开窗。
 */
static void Attitude_ResetFlowGyro(AppAttitudeRuntime_t *runtime)
{
    runtime->flow_gyro_integral_rad[0] = 0.0f;
    runtime->flow_gyro_integral_rad[1] = 0.0f;
    runtime->flow_gyro_interval_s = 0.0f;
}

/**
 * @brief 把一帧原始光流换算成 ESKF 可用的“机体系水平速度观测”。
 *
 * 分两种协议：
 *   - MICOLINK：模块直接输出速度量（像素/单位时间），乘高度和标定系数即可，无需陀螺补偿；
 *   - MSP_V2  ：模块输出角速度型光流，需要先减去区间平均陀螺角速度做旋转补偿，再换算。
 *
 * @param runtime    运行时（取陀螺积分）
 * @param flow       本帧 MTF02 原始数据
 * @param height_m   当前对地高度（米），光流角速度 × 高度 ≈ 线速度
 * @param observation[out] 组装好的水平速度观测（含测量噪声标准差）
 * @return true 观测构造成功且数值有限；false 数据无效应丢弃
 */
static bool Attitude_BuildFlow(const AppAttitudeRuntime_t *runtime, const MTF02_Data_t *flow, float height_m,
                               NavFlowObservation *observation)
{
    /* ---- 分支一：MICOLINK 协议，直接给速度型光流 ---- */
    if (flow->flow_protocol == MTF02_PROTOCOL_MICOLINK)
    {
        const float raw[2] = {(float)flow->flow_vel_x, (float)flow->flow_vel_y};
        /* 机体系 vx/vy = 原始光流 × 对地高度 × 轴标定系数（把像素速度换算成 m/s） */
        observation->vel_body_xy_mps[0] = raw[ATTITUDE_MTF02_FLOW_X_SOURCE] * height_m * ATTITUDE_MTF02_FLOW_X_SCALE_MPS;
        observation->vel_body_xy_mps[1] = raw[ATTITUDE_MTF02_FLOW_Y_SOURCE] * height_m * ATTITUDE_MTF02_FLOW_Y_SCALE_MPS;
        return true;
    }

    /* ---- 分支二：MSP_V2 协议，角速度型光流，需要旋转补偿 ---- */
    /* 协议不符 / 帧间隔时间不在合理窗口 / 陀螺积分区间为空，则无法补偿，判无效 */
    if ((flow->flow_protocol != MTF02_PROTOCOL_MSP_V2) || (flow->flow_delta_time_us < ATTITUDE_MTF02_MSP_MIN_DT_US) ||
        (flow->flow_delta_time_us > ATTITUDE_MTF02_MSP_MAX_DT_US) || (runtime->flow_gyro_interval_s <= 0.0f)) return false;

    /* 区间平均陀螺角速度 = 累计角位移 / 累计时长 */
    const float gyro_x = runtime->flow_gyro_integral_rad[0] / runtime->flow_gyro_interval_s;
    const float gyro_y = runtime->flow_gyro_integral_rad[1] / runtime->flow_gyro_interval_s;

    /* 旋转补偿：扣除机体自身转动分量，剩下的才是平移引起的光流角速度 */
    const float compensated[2] = {flow->flow_rate_rad_s[0] - gyro_x, flow->flow_rate_rad_s[1] - gyro_y};

    /* 补偿后的角速度 × 高度 × 标定系数 = 机体系平移线速度 */
    observation->vel_body_xy_mps[0] = compensated[ATTITUDE_MTF02_MSP_FLOW_X_SOURCE] * height_m * ATTITUDE_MTF02_MSP_FLOW_X_SCALE;
    observation->vel_body_xy_mps[1] = compensated[ATTITUDE_MTF02_MSP_FLOW_Y_SOURCE] * height_m * ATTITUDE_MTF02_MSP_FLOW_Y_SCALE;

    /* 最后一道防线：结果必须是有限数，防止 NaN/Inf 进入滤波器 */
    return isfinite(observation->vel_body_xy_mps[0]) && isfinite(observation->vel_body_xy_mps[1]);
}

/**
 * @brief 光流观测更新主函数：每个姿态周期调用一次。
 *        非阻塞取最新光流帧 → 多重有效性判断 → 构造观测 → 送入 ESKF。
 *        标 HOT_CODE：属 1kHz 热路径，链接到 ITCM 以降低取指延迟。
 */
static void APP_ATTITUDE_HOT_CODE Attitude_UpdateFlow(NavESKF *eskf, AppAttitudeRuntime_t *runtime)
{
    MTF02_Data_t flow;   /* 栈上局部副本，SubGetMessage 会把 Topic 共享缓存 memcpy 到这里 */

    /* 非阻塞读取（超时 0U）：没有新数据立即返回，绝不等候，保证实时周期 */
    if (SubGetMessage(flow_subscriber, &flow, 0U) == 0U) return;

    /* 时间戳去重：从未更新过(=0)，或与上一帧时间戳相同（重复帧），都不再融合 */
    if ((flow.flow_timestamp_us == 0ULL) || (flow.flow_timestamp_us == runtime->last_mtf_flow_timestamp_us)) return;
    runtime->last_mtf_flow_timestamp_us = flow.flow_timestamp_us;

    /* MTF02 模块自带测距，取其高度（毫米 → 米） */
    const float height_m = (float)flow.distance_mm * 0.001f;

    /* 光流帧与其内部测距帧的时间差绝对值（谁新谁旧都兼容） */
    const uint64_t range_age_us = (flow.flow_timestamp_us >= flow.range_timestamp_us)
                                      ? (flow.flow_timestamp_us - flow.range_timestamp_us)
                                      : (flow.range_timestamp_us - flow.flow_timestamp_us);

    /* 门限1：测距有效——时间戳非0、光流与测距足够同步、高度有限且落在可用高度窗口内 */
    const bool range_valid = (flow.range_timestamp_us != 0ULL) && (range_age_us <= ATTITUDE_MTF02_MAX_RANGE_AGE_US) &&
                             isfinite(height_m) && (height_m >= ATTITUDE_MTF02_MIN_HEIGHT_M) &&
                             (height_m <= ATTITUDE_MTF02_MAX_HEIGHT_M);

    /* 门限2：光流画面质量达标（纹理/可信度） */
    const bool quality_valid = flow.flow_quality >= ATTITUDE_MTF02_MIN_QUALITY;

    /* 门限3：状态有效——MSP_V2 不查状态位；或配置为不查；或模块状态字等于“有效”值 */
    const bool status_valid = (flow.flow_protocol == MTF02_PROTOCOL_MSP_V2) || (ATTITUDE_MTF02_CHECK_STATUS == 0U) ||
                              (flow.flow_status == ATTITUDE_MTF02_VALID_STATUS);

    /* 构造观测，预设水平速度测量噪声标准差（R 矩阵用），其余字段由 BuildFlow 填充 */
    NavFlowObservation observation = {.std_mps = {ATTITUDE_FLOW_STD_MPS, ATTITUDE_FLOW_STD_MPS}};

    /* 三道门限全过、且速度换算成功，才真正执行一次 ESKF 光流更新 */
    if (range_valid && quality_valid && status_valid && Attitude_BuildFlow(runtime, &flow, height_m, &observation))
    {
        (void)NAV_ESKF_UpdateFlow(eskf, &observation);
    }

    /* 无论本帧是否融合，都重置陀螺积分窗口，保证下一帧的补偿区间与光流帧对齐 */
    Attitude_ResetFlowGyro(runtime);
}
#endif /* ATTITUDE_ENABLE_FLOW */

/* ============================ 测距（TFmini）部分 ============================ */
#if (ATTITUDE_ENABLE_RANGE == 1U)

/* 测距 Topic 订阅者句柄 */
static Subscriber_t *range_subscriber;

/**
 * @brief 激光测距观测更新主函数：非阻塞取 TFmini 最新帧 → 有效性判断 → 构造高度观测 → 送入 ESKF。
 *        该观测直接约束 ESKF 的 NED-Z（垂直位置/速度），是定高控制的关键。
 */
static void APP_ATTITUDE_HOT_CODE Attitude_UpdateRange(NavESKF *eskf, AppAttitudeRuntime_t *runtime)
{
    TFminiPlus_Data_t range;  /* 栈上局部副本 */

    /* 非阻塞读取，无新数据立即返回 */
    if (SubGetMessage(range_subscriber, &range, 0U) == 0U) return;

    /* 时间戳去重 */
    if ((range.timestamp_us == 0ULL) || (range.timestamp_us == runtime->last_tfmini_timestamp_us)) return;
    runtime->last_tfmini_timestamp_us = range.timestamp_us;

    /* 模块自检无效，或距离读数为 0（异常值），丢弃 */
    if ((range.is_valid == 0U) || (range.distance == 0U)) return;

    /* 组装测距观测：
     *   range_m      原始距离 × 单位换算系数 → 米
     *   ground_down_m 安装/地面参考偏置（如传感器离地安装高度）
     *   std_m        测距噪声标准差（量测协方差 R 用） */
    const NavRangeObservation observation = {
        .range_m = (float)range.distance * ATTITUDE_TFMINI_DISTANCE_SCALE_M,
        .ground_down_m = ATTITUDE_RANGE_GROUND_DOWN_M,
        .std_m = ATTITUDE_RANGE_STD_M,
    };

    /* 送入 ESKF 做 Z 轴更新；成功才记录“最近一次有效融合时间戳”，供健康监测/超时判断 */
    if (NAV_ESKF_UpdateRange(eskf, &observation)) runtime->last_range_fusion_timestamp_us = range.timestamp_us;
}
#endif /* ATTITUDE_ENABLE_RANGE */

#endif /* 真实源 && 并行 ESKF 使能 的最外层开关 */

/**
 * @brief 观测模块初始化：按编译开关注册对应 Topic 的订阅者。
 * @return true 全部需要的订阅者注册成功；false 任一失败（订阅者槽位耗尽等），上层据此报错。
 */
bool App_Attitude_Observations_Init(void)
{
/* 仅在真实源 + 并行 ESKF 下才需要真正订阅 */
#if ((APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_REAL) && (ATTITUDE_PARALLEL_ESKF_ENABLE == 1U))

#if (ATTITUDE_ENABLE_FLOW == 1U)
    /* 订阅光流 Topic，约定载荷大小必须与发布侧 MTF02_Data_t 一致（消息中心会校验尺寸） */
    flow_subscriber = SubRegister(MTF02_TOPIC_NAME, sizeof(MTF02_Data_t));
    if (flow_subscriber == NULL)
    {
        RTTERROR("[Attitude] Flow subscriber register failed.");
        return false;
    }
#endif

#if (ATTITUDE_ENABLE_RANGE == 1U)
    /* 订阅激光测距 Topic */
    range_subscriber = SubRegister(TFMINI_TOPIC_NAME, sizeof(TFminiPlus_Data_t));
    if (range_subscriber == NULL)
    {
        RTTERROR("[Attitude] Range subscriber register failed.");
        return false;
    }
#endif

#endif
    return true;
}

/**
 * @brief 观测更新统一入口，由姿态任务在每个 IMU 周期（预测之后）调用。
 *
 * @param eskf     并行 ESKF 实例指针
 * @param runtime  姿态运行时结构
 * @param gyro_rps 本周期三轴陀螺角速度（rad/s），用于光流旋转补偿积分
 * @param dt_s     本周期步长（s）
 *
 * 顺序：先累加陀螺积分 → 更新光流（内部会清零积分窗）→ 更新测距。
 */
void APP_ATTITUDE_HOT_CODE App_Attitude_Observations_Update(NavESKF *eskf, AppAttitudeRuntime_t *runtime,
                                                            const float gyro_rps[3], float dt_s)
{
    /* 空指针保护 */
    if ((eskf == NULL) || (runtime == NULL)) return;

#if ((APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_REAL) && (ATTITUDE_PARALLEL_ESKF_ENABLE == 1U))

#if (ATTITUDE_ENABLE_FLOW == 1U)
    /* 先把本周期陀螺角位移累加进补偿窗口，再尝试消费一帧光流 */
    Attitude_AccumulateFlowGyro(runtime, gyro_rps, dt_s);
    Attitude_UpdateFlow(eskf, runtime);
#else
    /* 未开光流：显式标记未使用，避免编译器“未使用参数”告警 */
    (void)gyro_rps;
    (void)dt_s;
#endif

#if (ATTITUDE_ENABLE_RANGE == 1U)
    /* 再尝试消费一帧激光测距，约束 ESKF 垂直通道 */
    Attitude_UpdateRange(eskf, runtime);
#endif

#else
    /* 非真实源或未启用并行 ESKF：观测模块整体空转，仅消除未使用参数告警 */
    (void)gyro_rps;
    (void)dt_s;
#endif
}
