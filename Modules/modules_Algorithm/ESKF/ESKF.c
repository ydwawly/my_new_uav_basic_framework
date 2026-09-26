#include "ESKF.h"

#include <math.h>
#include <string.h>

/* 内部使用简短行主序矩阵访问宏 */
#define IDX(r, c) NAV_ESKF_IDX((r), (c))

/* ==========================================================================
 *                          协方差矩阵数值维护与误差状态注入
 * ========================================================================== */

/**
 * @brief 强行对称化协方差矩阵 P 并为对角线注入正定安全极小下限
 * @note  由于微机浮点截断误差，长期迭代会导致 P 失去对称正定性质。
 *        每次更新后执行此维护操作，是工程可用 ESKF 保持长期不发散的基石。
 */
static void NAV_ESKF_HOT_CODE covariance_symmetrize_and_floor(NavESKF *e)
{
    for (uint32_t r = 0; r < NAV_ESKF_STATE_DIM; ++r)
    {
        for (uint32_t c = r + 1; c < NAV_ESKF_STATE_DIM; ++c)
        {
            const float32_t avg = 0.5f * (e->P[IDX(r, c)] + e->P[IDX(c, r)]);
            e->P[IDX(r, c)]     = avg;
            e->P[IDX(c, r)]     = avg;
        }

        /* 检查并强制确保主对角线元素大于极小下限 */
        if ((!isfinite(e->P[IDX(r, r)])) || (e->P[IDX(r, r)] < 1.0e-12f))
        {
            e->P[IDX(r, r)] = 1.0e-12f;
        }
    }
}

/**
 * @brief 构造预测所需的三个非单位状态转移子块
 */
static void NAV_ESKF_HOT_CODE build_transition_blocks(const float32_t Rnb[9], const float32_t specific_force_b[3],
                                                      const float32_t omega[3], float32_t dt,
                                                      NavESKFTransitionBlocks *blocks)
{
    float32_t skew_force[9];
    float32_t skew_omega[9];

    Math_VectorSkew3(specific_force_b, skew_force);
    Math_VectorSkew3(omega, skew_omega);

    for (uint32_t r = 0U; r < 3U; r++)
    {
        for (uint32_t c = 0U; c < 3U; c++)
        {
            float32_t rotated_skew = 0.0f;
            for (uint32_t k = 0U; k < 3U; k++)
            {
                rotated_skew += Rnb[r * 3U + k] * skew_force[k * 3U + c];
            }

            blocks->vel_att[r * 3U + c]        = -rotated_skew * dt;
            blocks->vel_accel_bias[r * 3U + c] = -Rnb[r * 3U + c] * dt;
            blocks->att_att_delta[r * 3U + c]  = -skew_omega[r * 3U + c] * dt;
        }
    }
}

/**
 * @brief 利用 Fd 的 3x3 稀疏块传播 15 维协方差
 *
 * @note 数学结果仍然是完整的 P = Fd * P * Fd^T，没有删除任何协方差项。
 *       与两次通用 15x15 乘法相比，乘加次数由约 6750 次降至约 990 次。
 */
static void NAV_ESKF_HOT_CODE propagate_covariance_sparse(NavESKF *e, float32_t dt,
                                                          const NavESKFTransitionBlocks *blocks,
                                                          NavESKFPredictWorkspace       *workspace)
{
    const float32_t *P  = e->P;
    float32_t       *FP = workspace->FP;

    memcpy(FP, P, sizeof(workspace->FP));

    /* 左乘 Fd：只更新位置、速度和姿态对应的九行。 */
    for (uint32_t c = 0U; c < NAV_ESKF_STATE_DIM; c++)
    {
        for (uint32_t r = 0U; r < 3U; r++)
        {
            FP[IDX(NAV_ESKF_IX_POS + r, c)] = P[IDX(NAV_ESKF_IX_POS + r, c)] + dt * P[IDX(NAV_ESKF_IX_VEL + r, c)];

            float32_t velocity_value = P[IDX(NAV_ESKF_IX_VEL + r, c)];
            float32_t attitude_value = P[IDX(NAV_ESKF_IX_ATT + r, c)] - dt * P[IDX(NAV_ESKF_IX_BG + r, c)];
            for (uint32_t k = 0U; k < 3U; k++)
            {
                velocity_value += blocks->vel_att[r * 3U + k] * P[IDX(NAV_ESKF_IX_ATT + k, c)];
                velocity_value += blocks->vel_accel_bias[r * 3U + k] * P[IDX(NAV_ESKF_IX_BA + k, c)];
                attitude_value += blocks->att_att_delta[r * 3U + k] * P[IDX(NAV_ESKF_IX_ATT + k, c)];
            }
            FP[IDX(NAV_ESKF_IX_VEL + r, c)] = velocity_value;
            FP[IDX(NAV_ESKF_IX_ATT + r, c)] = attitude_value;
        }
    }

    /* 右乘 Fd^T：各输出列只读取 FP，因此可以直接写回 e->P。 */
    for (uint32_t r = 0U; r < NAV_ESKF_STATE_DIM; r++)
    {
        for (uint32_t c = 0U; c < 3U; c++)
        {
            e->P[IDX(r, NAV_ESKF_IX_POS + c)] = FP[IDX(r, NAV_ESKF_IX_POS + c)] + dt * FP[IDX(r, NAV_ESKF_IX_VEL + c)];

            float32_t velocity_value = FP[IDX(r, NAV_ESKF_IX_VEL + c)];
            float32_t attitude_value = FP[IDX(r, NAV_ESKF_IX_ATT + c)] - dt * FP[IDX(r, NAV_ESKF_IX_BG + c)];
            for (uint32_t k = 0U; k < 3U; k++)
            {
                velocity_value += FP[IDX(r, NAV_ESKF_IX_ATT + k)] * blocks->vel_att[c * 3U + k];
                velocity_value += FP[IDX(r, NAV_ESKF_IX_BA + k)] * blocks->vel_accel_bias[c * 3U + k];
                attitude_value += FP[IDX(r, NAV_ESKF_IX_ATT + k)] * blocks->att_att_delta[c * 3U + k];
            }
            e->P[IDX(r, NAV_ESKF_IX_VEL + c)] = velocity_value;
            e->P[IDX(r, NAV_ESKF_IX_ATT + c)] = attitude_value;
            e->P[IDX(r, NAV_ESKF_IX_BG + c)]  = FP[IDX(r, NAV_ESKF_IX_BG + c)];
            e->P[IDX(r, NAV_ESKF_IX_BA + c)]  = FP[IDX(r, NAV_ESKF_IX_BA + c)];
        }
    }
}

/**
 * @brief 将误差状态反馈注入至名义状态，并执行误差状态重置 (Error State Reset)
 * @param dx  通过 Kalman 增益求得的 15 维误差状态向量
 *
 * @note  数学原理：
 *        在 ESKF 体系中，我们在状态注入后将误差状态重新定义为零向量 (dx = 0)。
 *        然而对于姿态误差，它采用右乘乘积更新形式：q_nb(new) = q_nb(old) ⊗ exp([δθ/2]x)。
 *        经过此次更新，原先的扰动模型 δθ_old 与新坐标下的 δθ_new 之间存在李代数坐标转换关系，
 *        一阶泰勒近似下其 Jacobian 为：G_theta ≈ I - 0.5 * [δθ]x。
 *        因此协方差相关项必须按照更新规律同步做相似变换：P <- G * P * G^T。
 */
static void NAV_ESKF_HOT_CODE inject_error_state(NavESKF *e, const float32_t dx[NAV_ESKF_STATE_DIM])
{
    /* 1. 位置、速度、传感器零偏为简单的笛卡尔加法注入 */
    for (uint32_t i = 0; i < 3; ++i)
    {
        e->state.pos_ned_m[i] += dx[NAV_ESKF_IX_POS + i];
        e->state.vel_ned_mps[i] += dx[NAV_ESKF_IX_VEL + i];
        e->state.gyro_bias_rps[i] += dx[NAV_ESKF_IX_BG + i];
        e->state.accel_bias_mps2[i] += dx[NAV_ESKF_IX_BA + i];
    }

    /* 2. 姿态通过右乘旋转四元数的方式注入 */
    const float32_t dtheta[3]   = {dx[NAV_ESKF_IX_ATT + 0], dx[NAV_ESKF_IX_ATT + 1], dx[NAV_ESKF_IX_ATT + 2]};
    const float32_t dtheta_norm = Math_VectorNorm3(dtheta);
    const NavQuatf  dq          = Math_QuaternionFromRotationVector(dtheta);
    e->state.q_nb               = Math_QuaternionNormalize(Math_QuaternionMultiply(e->state.q_nb, dq));

    /* 3. 姿态重置对协方差矩阵 P 的 Jacobian 补偿
     *    当注入的角度极其微小（如静止或观测极其平滑时），G_theta 极为接近单位阵 I，
     *    可略过此复杂的 15x3 矩阵乘法以节省 CPU 时钟周期。 */
    if (dtheta_norm >= e->cfg.att_reset_jacobian_min_rad)
    {
        float32_t skew[9];
        Math_VectorSkew3(dtheta, skew);

        const float32_t Gtheta[9] = {1.0f - 0.5f * skew[0], -0.5f * skew[1],       -0.5f * skew[2],
                                     -0.5f * skew[3],       1.0f - 0.5f * skew[4], -0.5f * skew[5],
                                     -0.5f * skew[6],       -0.5f * skew[7],       1.0f - 0.5f * skew[8]};

        /* 左乘：更新与姿态相关的三行 P[ATT, c] = G_theta * P[ATT, c] */
        float32_t rows[3][NAV_ESKF_STATE_DIM];
        for (uint32_t a = 0; a < 3; ++a)
        {
            for (uint32_t c = 0; c < NAV_ESKF_STATE_DIM; ++c)
            {
                rows[a][c] = Gtheta[a * 3 + 0] * e->P[IDX(NAV_ESKF_IX_ATT + 0, c)] +
                             Gtheta[a * 3 + 1] * e->P[IDX(NAV_ESKF_IX_ATT + 1, c)] +
                             Gtheta[a * 3 + 2] * e->P[IDX(NAV_ESKF_IX_ATT + 2, c)];
            }
        }
        for (uint32_t a = 0; a < 3; ++a)
        {
            for (uint32_t c = 0; c < NAV_ESKF_STATE_DIM; ++c)
            {
                e->P[IDX(NAV_ESKF_IX_ATT + a, c)] = rows[a][c];
            }
        }

        /* 右乘：更新与姿态相关的三列 P[r, ATT] = P[r, ATT] * G_theta^T */
        float32_t cols[NAV_ESKF_STATE_DIM][3];
        for (uint32_t r = 0; r < NAV_ESKF_STATE_DIM; ++r)
        {
            for (uint32_t a = 0; a < 3; ++a)
            {
                cols[r][a] = e->P[IDX(r, NAV_ESKF_IX_ATT + 0)] * Gtheta[a * 3 + 0] +
                             e->P[IDX(r, NAV_ESKF_IX_ATT + 1)] * Gtheta[a * 3 + 1] +
                             e->P[IDX(r, NAV_ESKF_IX_ATT + 2)] * Gtheta[a * 3 + 2];
            }
        }
        for (uint32_t r = 0; r < NAV_ESKF_STATE_DIM; ++r)
        {
            for (uint32_t a = 0; a < 3; ++a)
            {
                e->P[IDX(r, NAV_ESKF_IX_ATT + a)] = cols[r][a];
            }
        }
    }

    covariance_symmetrize_and_floor(e);
}

/* ==========================================================================
 *                     观测门控 (Gating) 与顺序标量更新引擎
 * ========================================================================== */

/**
 * @brief 根据自由度与置信度水平查询标准的卡方理论分布分布门限
 * @param level 显著性水平
 * @param dof   观测自由度 (1 ~ 3 维)
 */
static float32_t chi_square_threshold(NavESKFChiSquareLevel level, uint32_t dof)
{
    if ((dof < 1U) || (dof > NAV_ESKF_MAX_OBS_DIM))
    {
        return 0.0f;
    }

    static const float32_t threshold_95[3]  = {3.841459f, 5.991465f, 7.814728f};
    static const float32_t threshold_99[3]  = {6.634897f, 9.210340f, 11.344867f};
    static const float32_t threshold_999[3] = {10.827566f, 13.815511f, 16.266236f};

    switch (level)
    {
    case NAV_ESKF_CHI2_95:
        return threshold_95[dof - 1U];
    case NAV_ESKF_CHI2_99:
        return threshold_99[dof - 1U];
    case NAV_ESKF_CHI2_999:
        return threshold_999[dof - 1U];
    case NAV_ESKF_CHI2_DISABLED:
    default:
        return INFINITY;
    }
}

/**
 * @brief 使用 Cholesky 分解计算整组高维观测的归一化新息平方 (NIS)
 * @note  NIS = r^T * S^(-1) * r，其中 S = H * P * H^T + R 为新息协方差矩阵。
 *        直接求非对角阵 S 的逆较为危险，利用 S 的对称正定特性进行 Cholesky 分解：
 *        S = L * L^T，通过两步三角矩阵回代求解 L * y = r，则 NIS = y^T * y。
 *        如此可极大避免微小特异值导致的求逆爆炸。
 */
static bool NAV_ESKF_HOT_CODE calculate_group_nis(const NavESKF *e, const float32_t *residual, const float32_t *H_rows,
                                                  const float32_t *R_diag, uint32_t m, float32_t *out_nis)
{
    if ((e == NULL) || (residual == NULL) || (H_rows == NULL) || (R_diag == NULL) || (out_nis == NULL) || (m < 1U) ||
        (m > NAV_ESKF_MAX_OBS_DIM))
    {
        return false;
    }

    float32_t HP[NAV_ESKF_MAX_OBS_DIM][NAV_ESKF_STATE_DIM]  = {{0}};
    float32_t S[NAV_ESKF_MAX_OBS_DIM][NAV_ESKF_MAX_OBS_DIM] = {{0}};
    float32_t L[NAV_ESKF_MAX_OBS_DIM][NAV_ESKF_MAX_OBS_DIM] = {{0}};
    float32_t y[NAV_ESKF_MAX_OBS_DIM]                       = {0};

    /* 1. 构造中间矩阵 HP = H * P */
    for (uint32_t row = 0; row < m; ++row)
    {
        if ((!isfinite(residual[row])) || (!Math_IsPositiveFinite(R_diag[row])))
        {
            return false;
        }

        const float32_t *H = &H_rows[row * NAV_ESKF_STATE_DIM];
        for (uint32_t c = 0; c < NAV_ESKF_STATE_DIM; ++c)
        {
            float32_t sum = 0.0f;
            for (uint32_t k = 0; k < NAV_ESKF_STATE_DIM; ++k)
            {
                sum += H[k] * e->P[IDX(k, c)];
            }
            HP[row][c] = sum;
        }
    }

    /* 2. 构造新息协方差矩阵 S = HP * H^T + R (这里假定观测白噪声矩阵 R 为正对角阵) */
    for (uint32_t i = 0; i < m; ++i)
    {
        for (uint32_t j = 0; j < m; ++j)
        {
            const float32_t *Hj  = &H_rows[j * NAV_ESKF_STATE_DIM];
            float32_t        sum = (i == j) ? R_diag[i] : 0.0f;
            for (uint32_t k = 0; k < NAV_ESKF_STATE_DIM; ++k)
            {
                sum += HP[i][k] * Hj[k];
            }
            S[i][j] = sum;
        }
    }

    /* 3. Cholesky 分解：S = L * L^T，其中 L 为下三角阵 */
    for (uint32_t i = 0; i < m; ++i)
    {
        for (uint32_t j = 0; j <= i; ++j)
        {
            float32_t sum = S[i][j];
            for (uint32_t k = 0; k < j; ++k)
            {
                sum -= L[i][k] * L[j][k];
            }

            if (i == j)
            {
                if ((!isfinite(sum)) || (sum <= 1.0e-12f))
                {
                    return false;
                }
                if (arm_sqrt_f32(sum, &L[i][j]) != ARM_MATH_SUCCESS)
                {
                    return false;
                }
            }
            else
            {
                if (L[j][j] <= 1.0e-12f)
                {
                    return false;
                }
                L[i][j] = sum / L[j][j];
            }
        }
    }

    /* 4. 前向替换解 L * y = r */
    for (uint32_t i = 0; i < m; ++i)
    {
        float32_t sum = residual[i];
        for (uint32_t k = 0; k < i; ++k)
        {
            sum -= L[i][k] * y[k];
        }
        y[i] = sum / L[i][i];
    }

    /* 5. 组装 NIS 统计量：y^T * y */
    float32_t nis = 0.0f;
    for (uint32_t i = 0; i < m; ++i)
    {
        nis += y[i] * y[i];
    }

    if (!isfinite(nis))
    {
        return false;
    }

    *out_nis = nis;
    return true;
}

/**
 * @brief 顺序标量卡尔曼更新引擎 (Sequential Scalar Update)
 *
 * @note  数学原理与核心技巧：
 *        当观测误差矩阵 R 为非相关对角阵（无跨轴交叉协方差）时，
 *        一次高维度的批量矩阵更新可等价拆解为一系列独立且有序的一维标量更新。
 *        这种策略彻底消除了多维矩阵求逆运算，每一步只需要做一维除法 invS = 1.0 / S_scalar，
 *        不仅让算法具备极高执行速度，还能通过逐个测量维度的逻辑修正彻底杜绝数值奇异性。
 *        且所有标量处理完毕后，统一执行一次重置与注入，提升运算吞吐。
 */
static bool NAV_ESKF_HOT_CODE sequential_update_rows(NavESKF *e, const float32_t *residual, const float32_t *H_rows,
                                                     const float32_t *R_diag, uint32_t m)
{
    float32_t dx[NAV_ESKF_STATE_DIM] = {0};

    for (uint32_t row = 0; row < m; ++row)
    {
        const float32_t *H                      = &H_rows[row * NAV_ESKF_STATE_DIM];
        float32_t        HP[NAV_ESKF_STATE_DIM] = {0};
        float32_t        K[NAV_ESKF_STATE_DIM]  = {0};

        /* 在顺序迭代中，前期轴的更新已对待求 dx 贡献了偏移。
         * 根据当前增量线性展开修正后续未处理轴的残差预测 */
        float32_t corrected_residual = residual[row];
        for (uint32_t i = 0; i < NAV_ESKF_STATE_DIM; ++i)
        {
            corrected_residual -= H[i] * dx[i];
        }

        /* 预计算 1x15 的行向量 HP = H * P */
        for (uint32_t c = 0; c < NAV_ESKF_STATE_DIM; ++c)
        {
            float32_t sum = 0.0f;
            for (uint32_t k = 0; k < NAV_ESKF_STATE_DIM; ++k)
            {
                sum += H[k] * e->P[IDX(k, c)];
            }
            HP[c] = sum;
        }

        /* 计算标量新息方差 S = H * P * H^T + R */
        float32_t innovation_variance = R_diag[row];
        for (uint32_t i = 0; i < NAV_ESKF_STATE_DIM; ++i)
        {
            innovation_variance += HP[i] * H[i];
        }

        if ((!isfinite(innovation_variance)) || (innovation_variance <= 1.0e-12f))
        {
            return false;
        }

        /* 求解一维卡尔曼增益 K = (P * H^T) / S */
        const float32_t invS = 1.0f / innovation_variance;
        for (uint32_t i = 0; i < NAV_ESKF_STATE_DIM; ++i)
        {
            float32_t PHt = 0.0f;
            for (uint32_t k = 0; k < NAV_ESKF_STATE_DIM; ++k)
            {
                PHt += e->P[IDX(i, k)] * H[k];
            }
            K[i] = PHt * invS;
            dx[i] += K[i] * corrected_residual;
        }

        /* 标量形式修正协方差：P = P - K * (H * P) */
        for (uint32_t r = 0; r < NAV_ESKF_STATE_DIM; ++r)
        {
            for (uint32_t c = 0; c < NAV_ESKF_STATE_DIM; ++c)
            {
                e->P[IDX(r, c)] -= K[r] * HP[c];
            }
        }

        covariance_symmetrize_and_floor(e);
    }

    /* 将最终累计的所有标量更新统一注入至系统名义状态中 */
    inject_error_state(e, dx);
    return true;
}

/**
 * @brief 对一组 1~3 维传感观测统合执行卡方野值检验与 Kalman 融合更新
 */
static bool NAV_ESKF_HOT_CODE update_observation_group(NavESKF *e, const float32_t *residual, const float32_t *H_rows,
                                                       const float32_t *R_diag, uint32_t m)
{
    float32_t nis = 0.0f;

    /* 1. 计算观测统计特征 NIS，若分解抛出奇异直接终止 */
    if (!calculate_group_nis(e, residual, H_rows, R_diag, m, &nis))
    {
        e->stats.numerical_failures++;
        e->stats.last_nis                  = NAN;
        e->stats.last_chi_square_threshold = NAN;
        e->stats.last_dof                  = (uint8_t)m;
        return false;
    }

    /* 2. 查表获取对应的理论 Chi-square 门限 */
    const float32_t threshold          = chi_square_threshold(e->cfg.chi_square_level, m);
    e->stats.last_nis                  = nis;
    e->stats.last_chi_square_threshold = threshold;
    e->stats.last_dof                  = (uint8_t)m;

    /* 3. 门控检测：新息平方距离越界代表传感器出现尖刺或受到多径干扰，剔除之 */
    if ((e->cfg.chi_square_level != NAV_ESKF_CHI2_DISABLED) && (nis > threshold))
    {
        e->stats.rejected_observation_groups++;
        return false;
    }

    /* 4. 送入顺序标量更新内核 */
    if (!sequential_update_rows(e, residual, H_rows, R_diag, m))
    {
        e->stats.numerical_failures++;
        return false;
    }

    e->stats.accepted_observation_groups++;
    e->stats.applied_scalar_rows += m;
    return true;
}

/**
 * @brief 只直接观测一个误差状态分量时的稀疏标量更新
 *
 * @note 气压高度和下视测距都只观测 δp_D。通用更新器会为 14 个恒为零的
 *       Jacobian 元素重复执行完整点积；本函数保持相同 NIS、门控、协方差
 *       更新和误差注入公式，仅跳过确定为零的乘法。
 */
static bool NAV_ESKF_HOT_CODE update_single_state_observation(NavESKF *e, float32_t residual, uint32_t state_index,
                                                              float32_t jacobian, float32_t measurement_variance)
{
    if ((e == NULL) || (!isfinite(residual)) || (state_index >= NAV_ESKF_STATE_DIM) || (!isfinite(jacobian)) ||
        (fabsf(jacobian) < 1.0e-12f) || (!Math_IsPositiveFinite(measurement_variance)))
    {
        return false;
    }

    const float32_t innovation_variance = jacobian * jacobian * e->P[IDX(state_index, state_index)] +
                                          measurement_variance;
    if ((!isfinite(innovation_variance)) || (innovation_variance <= 1.0e-12f))
    {
        e->stats.numerical_failures++;
        return false;
    }

    const float32_t nis                = residual * residual / innovation_variance;
    const float32_t threshold          = chi_square_threshold(e->cfg.chi_square_level, 1U);
    e->stats.last_nis                  = nis;
    e->stats.last_chi_square_threshold = threshold;
    e->stats.last_dof                  = 1U;

    if ((e->cfg.chi_square_level != NAV_ESKF_CHI2_DISABLED) && (nis > threshold))
    {
        e->stats.rejected_observation_groups++;
        return false;
    }

    float32_t       gain[NAV_ESKF_STATE_DIM];
    float32_t       hp[NAV_ESKF_STATE_DIM];
    float32_t       dx[NAV_ESKF_STATE_DIM];
    const float32_t inverse_innovation = 1.0f / innovation_variance;

    for (uint32_t i = 0U; i < NAV_ESKF_STATE_DIM; i++)
    {
        gain[i] = e->P[IDX(i, state_index)] * jacobian * inverse_innovation;
        hp[i]   = jacobian * e->P[IDX(state_index, i)];
        dx[i]   = gain[i] * residual;
    }

    for (uint32_t r = 0U; r < NAV_ESKF_STATE_DIM; r++)
    {
        for (uint32_t c = 0U; c < NAV_ESKF_STATE_DIM; c++)
        {
            e->P[IDX(r, c)] -= gain[r] * hp[c];
        }
    }

    covariance_symmetrize_and_floor(e);
    inject_error_state(e, dx);
    e->stats.accepted_observation_groups++;
    e->stats.applied_scalar_rows++;
    return true;
}

/**
 * @brief 一维航向专用更新
 *
 * @note 航向测量只应改变绕 NED Down 轴的朝向，不能在倾斜状态下把航向误差错误注入 Roll/Pitch。
 *       标准 Kalman 增益的姿态三维分量会受到右乘误差坐标与协方差相关项影响，因此这里将姿态增益
 *       投影到“导航系 Down 轴在机体系中的表示”，再用 Joseph 等价形式维护协方差正定性。
 */
static bool NAV_ESKF_HOT_CODE update_heading_observation(NavESKF *e, float32_t residual,
                                                         const float32_t H[NAV_ESKF_STATE_DIM],
                                                         const float32_t yaw_axis_body[3],
                                                         float32_t       measurement_variance)
{
    if ((e == NULL) || (!isfinite(residual)) || (!Math_IsPositiveFinite(measurement_variance)))
    {
        return false;
    }

    float32_t HP[NAV_ESKF_STATE_DIM]  = {0};
    float32_t PHt[NAV_ESKF_STATE_DIM] = {0};
    float32_t K[NAV_ESKF_STATE_DIM]   = {0};
    float32_t dx[NAV_ESKF_STATE_DIM]  = {0};

    for (uint32_t c = 0; c < NAV_ESKF_STATE_DIM; ++c)
    {
        float32_t sum = 0.0f;
        for (uint32_t k = 0; k < NAV_ESKF_STATE_DIM; ++k)
        {
            sum += H[k] * e->P[IDX(k, c)];
        }
        HP[c] = sum;
    }

    for (uint32_t r = 0; r < NAV_ESKF_STATE_DIM; ++r)
    {
        float32_t sum = 0.0f;
        for (uint32_t k = 0; k < NAV_ESKF_STATE_DIM; ++k)
        {
            sum += e->P[IDX(r, k)] * H[k];
        }
        PHt[r] = sum;
    }

    float32_t innovation_variance = measurement_variance;
    for (uint32_t i = 0; i < NAV_ESKF_STATE_DIM; ++i)
    {
        innovation_variance += HP[i] * H[i];
    }

    if ((!isfinite(innovation_variance)) || (innovation_variance <= 1.0e-12f))
    {
        e->stats.numerical_failures++;
        e->stats.last_nis                  = NAN;
        e->stats.last_chi_square_threshold = NAN;
        e->stats.last_dof                  = 1U;
        return false;
    }

    const float32_t nis       = Math_SquareFloat(residual) / innovation_variance;
    const float32_t threshold = chi_square_threshold(e->cfg.chi_square_level, 1U);

    e->stats.last_nis                  = nis;
    e->stats.last_chi_square_threshold = threshold;
    e->stats.last_dof                  = 1U;

    if ((!isfinite(nis)) || ((e->cfg.chi_square_level != NAV_ESKF_CHI2_DISABLED) && (nis > threshold)))
    {
        if (!isfinite(nis))
        {
            e->stats.numerical_failures++;
        }
        else
        {
            e->stats.rejected_observation_groups++;
        }
        return false;
    }

    const float32_t invS = 1.0f / innovation_variance;
    for (uint32_t i = 0; i < NAV_ESKF_STATE_DIM; ++i)
    {
        K[i] = PHt[i] * invS;
    }

    /* 记录标准增益在航向观测方向上的闭环增益，然后把姿态修正约束为纯 NED-Yaw 旋转。 */
    float32_t heading_gain       = 0.0f;
    float32_t axis_observability = 0.0f;
    for (uint32_t i = 0; i < 3; ++i)
    {
        heading_gain += H[NAV_ESKF_IX_ATT + i] * K[NAV_ESKF_IX_ATT + i];
        axis_observability += H[NAV_ESKF_IX_ATT + i] * yaw_axis_body[i];
    }

    if ((!isfinite(axis_observability)) || (fabsf(axis_observability) < 1.0e-6f))
    {
        e->stats.numerical_failures++;
        return false;
    }

    const float32_t constrained_gain = heading_gain / axis_observability;
    for (uint32_t i = 0; i < 3; ++i)
    {
        K[NAV_ESKF_IX_ATT + i] = yaw_axis_body[i] * constrained_gain;
    }

    for (uint32_t i = 0; i < NAV_ESKF_STATE_DIM; ++i)
    {
        dx[i] = K[i] * residual;
    }

    /* 任意受约束增益下的 Joseph 等价标量形式：
     * P+ = P - KHP - PH^T K^T + KSK^T。 */
    for (uint32_t r = 0; r < NAV_ESKF_STATE_DIM; ++r)
    {
        for (uint32_t c = 0; c < NAV_ESKF_STATE_DIM; ++c)
        {
            e->P[IDX(r, c)] = e->P[IDX(r, c)] - K[r] * HP[c] - PHt[r] * K[c] + K[r] * innovation_variance * K[c];
        }
    }

    inject_error_state(e, dx);

    e->stats.accepted_observation_groups++;
    e->stats.applied_scalar_rows++;
    return true;
}

/* ==========================================================================\n *                          外部公共导出接口实现\n * ========================================================================== */

void NAV_ESKF_GetDefaultConfig(NavESKFConfig *cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    /* 典型的消费级工业 MEMS IMU 噪声谱密度 (如 BMI088 / ICM-42688P) */
    cfg->gyro_noise    = 0.015f;
    cfg->accel_noise   = 0.20f;
    cfg->gyro_bias_rw  = 0.0005f;
    cfg->accel_bias_rw = 0.01f;

    /* 初始化开机收敛协方差参数 */
    cfg->init_pos_std        = 2.0f;
    cfg->init_vel_std        = 1.0f;
    cfg->init_att_std_rad    = 10.0f * (3.14159265358979323846f / 180.0f);
    cfg->init_gyro_bias_std  = 0.05f;
    cfg->init_accel_bias_std = 0.5f;

    cfg->chi_square_level           = NAV_ESKF_CHI2_99;
    cfg->att_reset_jacobian_min_rad = 1.0e-5f;
    cfg->min_range_cos_tilt         = 0.5f;
    cfg->min_heading_cos_pitch      = 0.08715574f; /* cos(85°)，避免航向欧拉角接近奇异 */

    /* 归一化重力向量观测；仅用IMU可观测条件门控，50 Hz融合。 */
    cfg->gravity_direction_std_rad    = 0.10f;
    cfg->gravity_accel_norm_gate_mps2 = 0.10f * NAV_ESKF_G;
    cfg->gravity_gyro_gate_rps        = 0.34906585f; /* 20 deg/s */
    cfg->gravity_max_speed_mps        = 0.30f;
    cfg->gravity_max_innovation_rad   = 0.26179939f; /* 15 deg */
    cfg->gravity_min_stable_time_s    = 0.25f;
    cfg->gravity_update_interval_s    = 0.02f;
}

void NAV_ESKF_Reset(NavESKF *e, const NavESKFState *initial_state)
{
    if (e == NULL)
    {
        return;
    }

    memset(&e->stats, 0, sizeof(e->stats));
    memset(e->P, 0, sizeof(e->P));
    e->gravity_stable_time_s        = 0.0f;
    e->gravity_update_accumulator_s = 0.0f;

    if (initial_state != NULL)
    {
        e->state      = *initial_state;
        e->state.q_nb = Math_QuaternionNormalize(e->state.q_nb);
    }
    else
    {
        memset(&e->state, 0, sizeof(e->state));
        e->state.q_nb = Math_QuaternionIdentity();
    }

    /* 在对角线上按独立误差初始化 P 矩阵 */
    const float32_t p2  = Math_SquareFloat(e->cfg.init_pos_std);
    const float32_t v2  = Math_SquareFloat(e->cfg.init_vel_std);
    const float32_t th2 = Math_SquareFloat(e->cfg.init_att_std_rad);
    const float32_t bg2 = Math_SquareFloat(e->cfg.init_gyro_bias_std);
    const float32_t ba2 = Math_SquareFloat(e->cfg.init_accel_bias_std);

    for (uint32_t i = 0; i < 3; ++i)
    {
        e->P[IDX(NAV_ESKF_IX_POS + i, NAV_ESKF_IX_POS + i)] = p2;
        e->P[IDX(NAV_ESKF_IX_VEL + i, NAV_ESKF_IX_VEL + i)] = v2;
        e->P[IDX(NAV_ESKF_IX_ATT + i, NAV_ESKF_IX_ATT + i)] = th2;
        e->P[IDX(NAV_ESKF_IX_BG + i, NAV_ESKF_IX_BG + i)]   = bg2;
        e->P[IDX(NAV_ESKF_IX_BA + i, NAV_ESKF_IX_BA + i)]   = ba2;
    }
}

void NAV_ESKF_Init(NavESKF *e, const NavESKFConfig *cfg, const NavESKFState *initial_state)
{
    if (e == NULL)
    {
        return;
    }

    memset(e, 0, sizeof(*e));

    if (cfg != NULL)
    {
        e->cfg = *cfg;
    }
    else
    {
        NAV_ESKF_GetDefaultConfig(&e->cfg);
    }

    NAV_ESKF_Reset(e, initial_state);
}

/**
 * @brief 使用一帧IMU数据推进名义状态和15维误差协方差
 *
 * @note 本函数按“去零偏、坐标变换、名义积分、姿态积分、状态转移矩阵、
 *       协方差传播、过程噪声”顺序连续对应ESKF预测公式。为方便对照推导和
 *       数值审查，有意保留为一个分阶段数学函数，不按普通业务函数行数拆分。
 */
bool NAV_ESKF_HOT_CODE NAV_ESKF_Predict(NavESKF *e, const NavImuSample *imu)
{
    if ((e == NULL) || (imu == NULL))
    {
        return false;
    }

    const float32_t dt = imu->dt_s;
    /* 防护非法更新周期：限制最高频率不过高，最低步长不超过 50ms (20Hz) */
    if ((!isfinite(dt)) || (dt <= 0.0f) || (dt > 0.05f))
    {
        return false;
    }

    /* 声明为静态局部变量，利用 NAV_ESKF_WORKSPACE_ATTRIBUTE 指定的段（如 DTCM）存储，
     * 避免在嵌入式系统的高频中断堆栈中申请 2.7KB 内存堆栈空间 */
    static NavESKFPredictWorkspace workspace NAV_ESKF_WORKSPACE_ATTRIBUTE;

    /* 1. 扣除当前估计零偏，得到机体系真实角速度与真实比力 */
    float32_t omega[3];
    float32_t specific_force_b[3];
    for (uint32_t i = 0; i < 3; ++i)
    {
        omega[i]            = imu->gyro_rps[i] - e->state.gyro_bias_rps[i];
        specific_force_b[i] = imu->accel_mps2[i] - e->state.accel_bias_mps2[i];
    }

    /* 2. 求旋转矩阵，将比力从机体系映射到导航系 (FRD -> NED) */
    float32_t Rnb[9];
    Math_QuaternionToRotationMatrix(e->state.q_nb, Rnb);

    float32_t accel_n[3];
    Math_RotateVector3(Rnb, specific_force_b, accel_n);

    /* 3. 在导航系下补偿由于地球引力作用带来的垂直方向比力感应
     *    NED 坐标系下 Z 轴垂直向下，重力加速度向量为 [0, 0, +g]^T */
    accel_n[2] += NAV_ESKF_G;

    /* 4. 名义运动学积分：利用修正加速度推进导航位置与速度 (二阶欧拉积分) */
    for (uint32_t i = 0; i < 3; ++i)
    {
        e->state.pos_ned_m[i] += e->state.vel_ned_mps[i] * dt + 0.5f * accel_n[i] * dt * dt;
        e->state.vel_ned_mps[i] += accel_n[i] * dt;
    }

    /* 5. 名义姿态演进：右乘角速度引起的微小旋转增量 */
    const float32_t dtheta[3] = {omega[0] * dt, omega[1] * dt, omega[2] * dt};
    e->state.q_nb             = Math_QuaternionNormalize(
        Math_QuaternionMultiply(e->state.q_nb, Math_QuaternionFromRotationVector(dtheta)));

    /* 6. 构造 Fd ≈ I + Fc*dt 的非单位 3x3 子块，并执行完整协方差演进
     *    P = Fd * P * Fd^T。这里利用已知稀疏结构跳过零元素，不改变数学模型。 */
    NavESKFTransitionBlocks transition_blocks;
    build_transition_blocks(Rnb, specific_force_b, omega, dt, &transition_blocks);
    propagate_covariance_sparse(e, dt, &transition_blocks, &workspace);

    /* 8. 叠加离散化过程噪声协方差 Qd
     *    由于对千赫兹(1kHz)系统而言，对角形式一阶方差贡献占绝对主导地位，
     *    此处直接向主自变量误差项追加离散步长噪声 Qd = Qc * dt，忽略微小的积分交叉项。 */
    const float32_t q_vel = Math_SquareFloat(e->cfg.accel_noise) * dt;
    const float32_t q_att = Math_SquareFloat(e->cfg.gyro_noise) * dt;
    const float32_t q_bg  = Math_SquareFloat(e->cfg.gyro_bias_rw) * dt;
    const float32_t q_ba  = Math_SquareFloat(e->cfg.accel_bias_rw) * dt;

    for (uint32_t i = 0; i < 3; ++i)
    {
        e->P[IDX(NAV_ESKF_IX_VEL + i, NAV_ESKF_IX_VEL + i)] += q_vel;
        e->P[IDX(NAV_ESKF_IX_ATT + i, NAV_ESKF_IX_ATT + i)] += q_att;
        e->P[IDX(NAV_ESKF_IX_BG + i, NAV_ESKF_IX_BG + i)] += q_bg;
        e->P[IDX(NAV_ESKF_IX_BA + i, NAV_ESKF_IX_BA + i)] += q_ba;
    }

    covariance_symmetrize_and_floor(e);
    return true;
}

static bool gravity_config_valid(const NavESKFConfig *cfg)
{
    if (cfg == NULL)
    {
        return false;
    }

    const float32_t values[] = {cfg->gravity_direction_std_rad,  cfg->gravity_accel_norm_gate_mps2,
                                cfg->gravity_gyro_gate_rps,      cfg->gravity_max_speed_mps,
                                cfg->gravity_max_innovation_rad, cfg->gravity_min_stable_time_s,
                                cfg->gravity_update_interval_s};
    for (uint32_t i = 0U; i < (sizeof(values) / sizeof(values[0])); ++i)
    {
        if (!Math_IsPositiveFinite(values[i]))
        {
            return false;
        }
    }
    return true;
}

static bool gravity_sample_ready(NavESKF *e, const NavImuSample *imu, float32_t accel_corrected[3],
                                 float32_t *accel_norm_out)
{
    float32_t gyro_corrected[3];
    for (uint32_t axis = 0U; axis < 3U; ++axis)
    {
        accel_corrected[axis] = imu->accel_mps2[axis] - e->state.accel_bias_mps2[axis];
        gyro_corrected[axis]  = imu->gyro_rps[axis] - e->state.gyro_bias_rps[axis];
    }

    const float32_t accel_norm            = Math_VectorNorm3(accel_corrected);
    const float32_t gyro_norm             = Math_VectorNorm3(gyro_corrected);
    const float32_t speed_norm            = Math_VectorNorm3(e->state.vel_ned_mps);
    e->stats.last_gravity_accel_norm_mps2 = accel_norm;
    e->stats.last_gravity_gyro_norm_rps   = gyro_norm;
    e->stats.last_gravity_speed_norm_mps  = speed_norm;

    const bool motion_gate_passed = isfinite(accel_norm) && isfinite(gyro_norm) && isfinite(speed_norm) &&
                                    (accel_norm > 1.0e-6f) &&
                                    (fabsf(accel_norm - NAV_ESKF_G) <= e->cfg.gravity_accel_norm_gate_mps2) &&
                                    (gyro_norm <= e->cfg.gravity_gyro_gate_rps);
    if (!motion_gate_passed)
    {
        e->gravity_stable_time_s        = 0.0f;
        e->gravity_update_accumulator_s = 0.0f;
        e->stats.gravity_gate_rejections++;
        return false;
    }

    e->gravity_stable_time_s = fminf(e->gravity_stable_time_s + imu->dt_s, e->cfg.gravity_min_stable_time_s);
    if (e->gravity_stable_time_s < e->cfg.gravity_min_stable_time_s)
    {
        return false;
    }

    e->gravity_update_accumulator_s += imu->dt_s;
    if (e->gravity_update_accumulator_s < e->cfg.gravity_update_interval_s)
    {
        return false;
    }
    e->gravity_update_accumulator_s = fmodf(e->gravity_update_accumulator_s, e->cfg.gravity_update_interval_s);
    *accel_norm_out                 = accel_norm;
    return true;
}

static bool gravity_tangent_basis(const float32_t predicted[3], float32_t tangent_1[3], float32_t tangent_2[3])
{
    const bool      use_down_reference = fabsf(predicted[2]) < 0.9f;
    const float32_t reference[3]       = {use_down_reference ? 0.0f : 1.0f, 0.0f, use_down_reference ? 1.0f : 0.0f};
    tangent_1[0]                       = reference[1] * predicted[2] - reference[2] * predicted[1];
    tangent_1[1]                       = reference[2] * predicted[0] - reference[0] * predicted[2];
    tangent_1[2]                       = reference[0] * predicted[1] - reference[1] * predicted[0];

    const float32_t norm = Math_VectorNorm3(tangent_1);
    if ((!isfinite(norm)) || (norm < 1.0e-6f))
    {
        return false;
    }
    for (uint32_t axis = 0U; axis < 3U; ++axis)
    {
        tangent_1[axis] /= norm;
    }

    tangent_2[0] = predicted[1] * tangent_1[2] - predicted[2] * tangent_1[1];
    tangent_2[1] = predicted[2] * tangent_1[0] - predicted[0] * tangent_1[2];
    tangent_2[2] = predicted[0] * tangent_1[1] - predicted[1] * tangent_1[0];
    return true;
}

static bool gravity_build_observation(NavESKF *e, const float32_t accel_corrected[3], float32_t accel_norm,
                                      float32_t residual[2], float32_t H[2 * NAV_ESKF_STATE_DIM])
{
    float32_t Rnb[9];
    Math_QuaternionToRotationMatrix(e->state.q_nb, Rnb);

    /* 静止时FRD加速度计测得向上的比力；R_nb第三行是导航系Down轴在机体系的方向。 */
    const float32_t predicted[3]         = {-Rnb[6], -Rnb[7], -Rnb[8]};
    const float32_t measured[3]          = {accel_corrected[0] / accel_norm, accel_corrected[1] / accel_norm,
                                            accel_corrected[2] / accel_norm};
    const float32_t dot_product          = Math_ClampFloat(predicted[0] * measured[0] + predicted[1] * measured[1] +
                                                               predicted[2] * measured[2],
                                                           -1.0f, 1.0f);
    const float32_t innovation_angle     = acosf(dot_product);
    e->stats.last_gravity_innovation_rad = innovation_angle;
    if ((!isfinite(innovation_angle)) || (innovation_angle > e->cfg.gravity_max_innovation_rad))
    {
        e->stats.gravity_updates_rejected++;
        return false;
    }

    /* 在预测重力方向的切平面构造两个正交残差，严格保留Yaw不可观测方向。 */
    float32_t tangent_1[3];
    float32_t tangent_2[3];
    if (!gravity_tangent_basis(predicted, tangent_1, tangent_2))
    {
        e->stats.numerical_failures++;
        return false;
    }

    const float32_t delta[3] = {measured[0] - predicted[0], measured[1] - predicted[1], measured[2] - predicted[2]};
    const float32_t tangents[2][3] = {{tangent_1[0], tangent_1[1], tangent_1[2]},
                                      {tangent_2[0], tangent_2[1], tangent_2[2]}};
    float32_t       predicted_skew[9];
    Math_VectorSkew3(predicted, predicted_skew);

    for (uint32_t row = 0U; row < 2U; ++row)
    {
        for (uint32_t axis = 0U; axis < 3U; ++axis)
        {
            residual[row] += tangents[row][axis] * delta[axis];
            H[row * NAV_ESKF_STATE_DIM + NAV_ESKF_IX_ATT + axis] = tangents[row][0] * predicted_skew[axis] +
                                                                   tangents[row][1] * predicted_skew[3U + axis] +
                                                                   tangents[row][2] * predicted_skew[6U + axis];
        }
    }
    return true;
}

bool NAV_ESKF_HOT_CODE NAV_ESKF_UpdateGravity(NavESKF *e, const NavImuSample *imu)
{
    if ((e == NULL) || (imu == NULL) || (!Math_Vector3IsFinite(imu->gyro_rps)) ||
        (!Math_Vector3IsFinite(imu->accel_mps2)) || (!Math_IsPositiveFinite(imu->dt_s)) ||
        (!gravity_config_valid(&e->cfg)))
    {
        return false;
    }

    float32_t accel_corrected[3];
    float32_t accel_norm;
    if (!gravity_sample_ready(e, imu, accel_corrected, &accel_norm))
    {
        return false;
    }

    float32_t residual[2]               = {0.0f, 0.0f};
    float32_t H[2 * NAV_ESKF_STATE_DIM] = {0.0f};
    if (!gravity_build_observation(e, accel_corrected, accel_norm, residual, H))
    {
        return false;
    }

    const float32_t variance = Math_SquareFloat(e->cfg.gravity_direction_std_rad);
    const float32_t Rdiag[2] = {variance, variance};
    if (!update_observation_group(e, residual, H, Rdiag, 2U))
    {
        e->stats.gravity_updates_rejected++;
        return false;
    }

    e->stats.gravity_updates_accepted++;
    return true;
}

bool NAV_ESKF_HOT_CODE NAV_ESKF_UpdateGps(NavESKF *e, const NavGpsObservation *obs)
{
    if ((e == NULL) || (obs == NULL))
    {
        return false;
    }

    bool updated = false;

    /* 1. 位置观测融合 */
    if (obs->use_position)
    {
        float32_t residual[3];
        float32_t H[3 * NAV_ESKF_STATE_DIM] = {0};
        float32_t Rdiag[3];

        for (uint32_t i = 0; i < 3; ++i)
        {
            residual[i]                                     = obs->pos_ned_m[i] - e->state.pos_ned_m[i];
            H[i * NAV_ESKF_STATE_DIM + NAV_ESKF_IX_POS + i] = 1.0f;
            Rdiag[i]                                        = Math_SquareFloat(obs->pos_std_m[i]);
        }

        updated |= update_observation_group(e, residual, H, Rdiag, 3U);
    }

    /* 2. 速度观测融合
     *    在位置更新完毕并注入状态后，通过读取最新名义速度重新构造残差，
     *    可显著提升在高速机动下 GPS 多模态更新的耦合响应。 */
    if (obs->use_velocity)
    {
        float32_t residual[3];
        float32_t H[3 * NAV_ESKF_STATE_DIM] = {0};
        float32_t Rdiag[3];

        for (uint32_t i = 0; i < 3; ++i)
        {
            residual[i]                                     = obs->vel_ned_mps[i] - e->state.vel_ned_mps[i];
            H[i * NAV_ESKF_STATE_DIM + NAV_ESKF_IX_VEL + i] = 1.0f;
            Rdiag[i]                                        = Math_SquareFloat(obs->vel_std_mps[i]);
        }

        updated |= update_observation_group(e, residual, H, Rdiag, 3U);
    }

    return updated;
}

bool NAV_ESKF_HOT_CODE NAV_ESKF_UpdateFlow(NavESKF *e, const NavFlowObservation *obs)
{
    if ((e == NULL) || (obs == NULL))
    {
        return false;
    }

    float32_t Rnb[9];
    Math_QuaternionToRotationMatrix(e->state.q_nb, Rnb);

    /* 计算机体系下的当前估计地速：v_b = R_bn * v_n = R_nb^T * v_n */
    float32_t velocity_b[3];
    Math_RotateVector3Transpose(Rnb, e->state.vel_ned_mps, velocity_b);

    float32_t residual[2] = {obs->vel_body_xy_mps[0] - velocity_b[0], obs->vel_body_xy_mps[1] - velocity_b[1]};
    float32_t H[2 * NAV_ESKF_STATE_DIM] = {0};
    float32_t Rdiag[2]                  = {Math_SquareFloat(obs->std_mps[0]), Math_SquareFloat(obs->std_mps[1])};

    /* 速度测量的对速度状态 Jacobian 就是旋转矩阵 R_nb^T 的前两行 */
    for (uint32_t axis = 0; axis < 2; ++axis)
    {
        H[axis * NAV_ESKF_STATE_DIM + NAV_ESKF_IX_VEL + 0] = Rnb[0 * 3 + axis];
        H[axis * NAV_ESKF_STATE_DIM + NAV_ESKF_IX_VEL + 1] = Rnb[1 * 3 + axis];
        H[axis * NAV_ESKF_STATE_DIM + NAV_ESKF_IX_VEL + 2] = Rnb[2 * 3 + axis];
    }

    /* 对右乘姿态误差体系，通过扰动分析得到 δv_b ≈ [v_b]x * δθ */
    float32_t skew_velocity_b[9];
    Math_VectorSkew3(velocity_b, skew_velocity_b);
    for (uint32_t axis = 0; axis < 2; ++axis)
    {
        for (uint32_t j = 0; j < 3; ++j)
        {
            H[axis * NAV_ESKF_STATE_DIM + NAV_ESKF_IX_ATT + j] = skew_velocity_b[axis * 3 + j];
        }
    }

    return update_observation_group(e, residual, H, Rdiag, 2U);
}

bool NAV_ESKF_HOT_CODE NAV_ESKF_UpdateBaro(NavESKF *e, const NavBaroObservation *obs)
{
    if ((e == NULL) || (obs == NULL) || (!Math_IsPositiveFinite(obs->std_m)))
    {
        return false;
    }

    /* 气压高度向上为正，NED 坐标系位置 Z(p_D) 向下为正，故观测方程为 h(x) = -p_D */
    const float32_t residual = obs->altitude_up_m - (-e->state.pos_ned_m[2]);
    return update_single_state_observation(e, residual, NAV_ESKF_IX_POS + 2U, -1.0f, Math_SquareFloat(obs->std_m));
}

bool NAV_ESKF_HOT_CODE NAV_ESKF_UpdateRange(NavESKF *e, const NavRangeObservation *obs)
{
    if ((e == NULL) || (obs == NULL) || (!Math_IsPositiveFinite(obs->range_m)) || (!Math_IsPositiveFinite(obs->std_m)))
    {
        return false;
    }

    float32_t Rnb[9];
    Math_QuaternionToRotationMatrix(e->state.q_nb, Rnb);

    /* 提取机体系 +Z 轴（向下）在 NED 坐标系 Down 轴方向上的分量 R[2,2] */
    const float32_t cos_tilt = Rnb[8];
    if (cos_tilt < e->cfg.min_range_cos_tilt)
    {
        return false; /* 飞行器倾角太大，激光下视距离不再能线性等价于飞行高度，驳回 */
    }

    /* 将机体斜距转换并投影至垂直地面下向坐标伪观测：measured_p_D = ground_down - range * cos_tilt */
    const float32_t measured_p_down = obs->ground_down_m - obs->range_m * cos_tilt;
    const float32_t residual[1]     = {measured_p_down - e->state.pos_ned_m[2]};

    return update_single_state_observation(e, residual[0], NAV_ESKF_IX_POS + 2U, 1.0f, Math_SquareFloat(obs->std_m));
}

bool NAV_ESKF_HOT_CODE NAV_ESKF_UpdateHeading(NavESKF *e, const NavHeadingObservation *obs)
{
    if ((e == NULL) || (obs == NULL) || (!isfinite(obs->heading_rad)) || (!Math_IsPositiveFinite(obs->std_rad)))
    {
        return false;
    }

    float32_t Rnb[9];
    Math_QuaternionToRotationMatrix(e->state.q_nb, Rnb);

    /* 机体 X 轴在 NED 水平面上的投影长度就是 |cos(pitch)|。
     * 当机头接近竖直时，3-2-1 欧拉航向本身失去唯一性，必须拒绝本次更新。 */
    const float32_t horizontal_projection_sq = Math_SquareFloat(Rnb[0]) + Math_SquareFloat(Rnb[3]);
    const float32_t min_cos_pitch            = fmaxf(1.0e-3f, e->cfg.min_heading_cos_pitch);
    if ((!isfinite(horizontal_projection_sq)) || (horizontal_projection_sq < Math_SquareFloat(min_cos_pitch)))
    {
        return false;
    }

    const float32_t estimated_heading = atan2f(Rnb[3], Rnb[0]);
    const float32_t residual[1]       = {Math_WrapPi(obs->heading_rad - estimated_heading)};

    float32_t       H[NAV_ESKF_STATE_DIM]        = {0};
    const float32_t inv_horizontal_projection_sq = 1.0f / horizontal_projection_sq;

    /*
     * 右乘误差定义：R_nb(true) ≈ R_nb(est) * (I + [δθ]x)。
     * 对 ψ = atan2(R_nb[1,0], R_nb[0,0]) 求导可得：
     *
     * δψ = H_theta * δθ
     *
     * 这里不能无条件写成 H[δθ_z] = 1：只有 Roll/Pitch 接近 0 时该近似才成立。
     * 使用精确 Jacobian 可避免倾斜飞行时磁航向错误地修正姿态。 */
    H[NAV_ESKF_IX_ATT + 0] = 0.0f;
    H[NAV_ESKF_IX_ATT + 1] = (Rnb[3] * Rnb[2] - Rnb[0] * Rnb[5]) * inv_horizontal_projection_sq;
    H[NAV_ESKF_IX_ATT + 2] = (Rnb[0] * Rnb[4] - Rnb[3] * Rnb[1]) * inv_horizontal_projection_sq;

    /* 导航系 Down 轴在机体系中的表示。沿此轴右乘旋转，等价于在导航系左乘纯 Yaw 旋转，
     * 因而可以保持当前 Roll/Pitch 不变。 */
    const float32_t yaw_axis_body[3] = {Rnb[6], Rnb[7], Rnb[8]};

    return update_heading_observation(e, residual[0], H, yaw_axis_body, Math_SquareFloat(obs->std_rad));
}

bool NAV_ESKF_HOT_CODE NAV_ESKF_UpdateMag(NavESKF *e, const NavMagObservation *obs)
{
    if ((e == NULL) || (obs == NULL) || (!isfinite(obs->mag_body_gauss[0])) || (!isfinite(obs->mag_body_gauss[1])) ||
        (!isfinite(obs->mag_body_gauss[2])) || (!isfinite(obs->declination_rad)) ||
        (!Math_IsPositiveFinite(obs->heading_std_rad)))
    {
        return false;
    }

    const float32_t field_norm = Math_VectorNorm3(obs->mag_body_gauss);
    if ((!Math_IsPositiveFinite(field_norm)) ||
        ((obs->field_norm_min_gauss > 0.0f) && (field_norm < obs->field_norm_min_gauss)) ||
        ((obs->field_norm_max_gauss > 0.0f) && (field_norm > obs->field_norm_max_gauss)))
    {
        return false;
    }

    float32_t Rnb[9];
    Math_QuaternionToRotationMatrix(e->state.q_nb, Rnb);

    /* 从当前姿态中只提取 Roll/Pitch，用于把机体系磁场旋转到“航向为 0 的水平参考系”。
     * 该过程不会使用当前 Yaw 参与磁航向测量值的构造，因此不会形成把估计值反馈给自身的伪观测。 */
    const float32_t cos_pitch_sq = Math_SquareFloat(Rnb[0]) + Math_SquareFloat(Rnb[3]);
    float32_t       cos_pitch    = 0.0f;
    if ((arm_sqrt_f32(cos_pitch_sq, &cos_pitch) != ARM_MATH_SUCCESS) ||
        (cos_pitch < fmaxf(1.0e-3f, e->cfg.min_heading_cos_pitch)))
    {
        return false;
    }

    const float32_t sin_pitch = -Rnb[6];
    const float32_t sin_roll  = Rnb[7] / cos_pitch;
    const float32_t cos_roll  = Rnb[8] / cos_pitch;

    const float32_t mx = obs->mag_body_gauss[0];
    const float32_t my = obs->mag_body_gauss[1];
    const float32_t mz = obs->mag_body_gauss[2];

    /* FRD / NED 倾斜补偿：
     * level_x 指向机头水平投影方向，level_y 指向机体右侧水平投影方向。
     * 在北半球无磁偏角、机头朝东时，磁北位于机体左侧，因此航向应为 -atan2(level_y, level_x)。 */
    const float32_t level_x = cos_pitch * mx + sin_roll * sin_pitch * my + cos_roll * sin_pitch * mz;
    const float32_t level_y = cos_roll * my - sin_roll * mz;

    float32_t horizontal_field = 0.0f;
    if (arm_sqrt_f32(Math_SquareFloat(level_x) + Math_SquareFloat(level_y), &horizontal_field) != ARM_MATH_SUCCESS)
    {
        return false;
    }

    const float32_t min_horizontal_field = (obs->horizontal_field_min_gauss > 0.0f) ? obs->horizontal_field_min_gauss
                                                                                    : 1.0e-6f;

    if ((!Math_IsPositiveFinite(horizontal_field)) || (horizontal_field < min_horizontal_field))
    {
        return false;
    }

    const NavHeadingObservation heading = {.heading_rad = Math_WrapPi(obs->declination_rad - atan2f(level_y, level_x)),
                                           .std_rad     = obs->heading_std_rad};

    return NAV_ESKF_UpdateHeading(e, &heading);
}

bool NAV_ESKF_HOT_CODE NAV_ESKF_UpdateVision(NavESKF *e, const NavVisionObservation *obs)
{
    if ((e == NULL) || (obs == NULL))
    {
        return false;
    }

    bool updated = false;

    if (obs->use_position)
    {
        float32_t residual[3];
        float32_t H[3 * NAV_ESKF_STATE_DIM] = {0};
        float32_t Rdiag[3];

        for (uint32_t i = 0; i < 3; ++i)
        {
            residual[i]                                     = obs->pos_ned_m[i] - e->state.pos_ned_m[i];
            H[i * NAV_ESKF_STATE_DIM + NAV_ESKF_IX_POS + i] = 1.0f;
            Rdiag[i]                                        = Math_SquareFloat(obs->pos_std_m[i]);
        }
        updated |= update_observation_group(e, residual, H, Rdiag, 3U);
    }

    if (obs->use_velocity)
    {
        float32_t residual[3];
        float32_t H[3 * NAV_ESKF_STATE_DIM] = {0};
        float32_t Rdiag[3];

        for (uint32_t i = 0; i < 3; ++i)
        {
            residual[i]                                     = obs->vel_ned_mps[i] - e->state.vel_ned_mps[i];
            H[i * NAV_ESKF_STATE_DIM + NAV_ESKF_IX_VEL + i] = 1.0f;
            Rdiag[i]                                        = Math_SquareFloat(obs->vel_std_mps[i]);
        }
        updated |= update_observation_group(e, residual, H, Rdiag, 3U);
    }

    if (obs->use_attitude)
    {
        float32_t residual[3];
        /*
         * 仅允许真实提供完整姿态的 VIO / Mocap 进入该分支。
         * 磁力计或普通 GPS 航迹角只有一维航向信息，必须分别调用
         * NAV_ESKF_UpdateMag() 或 NAV_ESKF_UpdateHeading()，不能伪造完整四元数，
         * 否则会对 Roll/Pitch/Yaw 产生无物理依据的三轴修正。
         */
        /* 利用短弧度李代数旋转解耦求得等效姿态残差 */
        Math_QuaternionErrorRotationVector(e->state.q_nb, obs->q_nb, residual);

        float32_t H[3 * NAV_ESKF_STATE_DIM] = {0};
        float32_t Rdiag[3];
        for (uint32_t i = 0; i < 3; ++i)
        {
            H[i * NAV_ESKF_STATE_DIM + NAV_ESKF_IX_ATT + i] = 1.0f;
            Rdiag[i]                                        = Math_SquareFloat(obs->att_std_rad[i]);
        }
        updated |= update_observation_group(e, residual, H, Rdiag, 3U);
    }

    return updated;
}
