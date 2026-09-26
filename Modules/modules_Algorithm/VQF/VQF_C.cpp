/**
 * @file VQF_C.cpp
 * @brief VQF C++ 实现到飞控 C 接口的静态单例适配层
 *
 * 适配层使用 placement new 在静态存储区构造 VQF，不使用动态堆；同时负责把库的
 * ENU 四元数转换到飞控统一使用的 NED/FRD 约定，并施加上电航向归零修正。
 */
#include "VQF_C.h"

#include <cmath>
#include <new>

#include "vqf.hpp"

namespace
{
/* VQF 对象使用静态对齐存储，避免飞行期间产生 new/delete 堆操作。 */
alignas(VQF) unsigned char vqf_storage[sizeof(VQF)];
VQF     *vqf_instance              = nullptr;
uint32_t update_count              = 0U;
uint32_t invalid_output_count      = 0U;
float    heading_correction_ned[4] = {1.0F, 0.0F, 0.0F, 0.0F};

/** @brief 检查一段浮点向量是否全部为有限数。 */
bool IsFiniteVector(const float *values, size_t count)
{
    for (size_t i = 0U; i < count; ++i)
    {
        if (!std::isfinite(values[i]))
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief 把 VQF 输出的 ENU 四元数转换为 NED，并叠加当前航向修正。
 * @note 输出会归一化并统一到实部非负的等价四元数，便于连续观察和比较。
 */
void ConvertEnuToNed(const vqf_real_t quat_enu[4], float quat_ned[4])
{
    constexpr vqf_real_t kInvSqrtTwo = static_cast<vqf_real_t>(0.70710678118654752440);

    const float raw_ned[4] = {
        static_cast<float>(-kInvSqrtTwo * (quat_enu[1] + quat_enu[2])),
        static_cast<float>(kInvSqrtTwo * (quat_enu[0] + quat_enu[3])),
        static_cast<float>(kInvSqrtTwo * (quat_enu[0] - quat_enu[3])),
        static_cast<float>(kInvSqrtTwo * (quat_enu[2] - quat_enu[1])),
    };

    quat_ned[0] = heading_correction_ned[0] * raw_ned[0] - heading_correction_ned[3] * raw_ned[3];
    quat_ned[1] = heading_correction_ned[0] * raw_ned[1] - heading_correction_ned[3] * raw_ned[2];
    quat_ned[2] = heading_correction_ned[0] * raw_ned[2] + heading_correction_ned[3] * raw_ned[1];
    quat_ned[3] = heading_correction_ned[0] * raw_ned[3] + heading_correction_ned[3] * raw_ned[0];

    const float norm = std::sqrt(quat_ned[0] * quat_ned[0] + quat_ned[1] * quat_ned[1] + quat_ned[2] * quat_ned[2] +
                                 quat_ned[3] * quat_ned[3]);
    if (norm > 1.0e-6F)
    {
        const float inverse_norm = 1.0F / norm;
        for (size_t i = 0U; i < 4U; ++i)
        {
            quat_ned[i] *= inverse_norm;
        }
    }

    if (quat_ned[0] < 0.0F)
    {
        for (size_t i = 0U; i < 4U; ++i)
        {
            quat_ned[i] = -quat_ned[i];
        }
    }
}
} // namespace

/** @brief 在静态存储区构造 VQF 单例并复位统计与航向修正。 */
extern "C" bool VqfC_Init(float sample_period_s)
{
    if ((!std::isfinite(sample_period_s)) || (sample_period_s <= 0.0F))
    {
        return false;
    }

    if (vqf_instance != nullptr)
    {
        vqf_instance->~VQF();
    }

    vqf_instance              = new (vqf_storage) VQF(static_cast<vqf_real_t>(sample_period_s));
    heading_correction_ned[0] = 1.0F;
    heading_correction_ned[1] = 0.0F;
    heading_correction_ned[2] = 0.0F;
    heading_correction_ned[3] = 0.0F;
    update_count              = 0U;
    invalid_output_count      = 0U;
    return true;
}

/** @brief 复位 VQF 内部状态和本适配层统计，不释放静态对象。 */
extern "C" void VqfC_Reset(void)
{
    if (vqf_instance != nullptr)
    {
        vqf_instance->resetState();
    }
    heading_correction_ned[0] = 1.0F;
    heading_correction_ned[1] = 0.0F;
    heading_correction_ned[2] = 0.0F;
    heading_correction_ned[3] = 0.0F;
    update_count              = 0U;
    invalid_output_count      = 0U;
}

/**
 * @brief 以当前 6D 姿态的航向为零点，更新后续输出使用的 NED 航向修正。
 * @return false 表示尚未初始化或当前航向不是有限数
 */
extern "C" bool VqfC_ZeroHeading(void)
{
    if (vqf_instance == nullptr)
    {
        return false;
    }

    vqf_real_t quat_enu[4];
    float      quat_ned[4];
    vqf_instance->getQuat6D(quat_enu);

    heading_correction_ned[0] = 1.0F;
    heading_correction_ned[1] = 0.0F;
    heading_correction_ned[2] = 0.0F;
    heading_correction_ned[3] = 0.0F;
    ConvertEnuToNed(quat_enu, quat_ned);

    const float yaw = std::atan2(2.0F * (quat_ned[0] * quat_ned[3] + quat_ned[1] * quat_ned[2]),
                                 1.0F - 2.0F * (quat_ned[2] * quat_ned[2] + quat_ned[3] * quat_ned[3]));
    if (!std::isfinite(yaw))
    {
        return false;
    }

    heading_correction_ned[0] = std::cos(-0.5F * yaw);
    heading_correction_ned[3] = std::sin(-0.5F * yaw);
    return true;
}

/**
 * @brief 输入一帧机体系陀螺和加速度，输出 NED 姿态及陀螺零偏估计。
 * @note 输入单位分别为 rad/s 与 m/s²；所有边界数据都会进行有限性检查。
 */
extern "C" bool VqfC_Update(const float gyro_rps[3], const float accel_mps2[3], VqfCOutput_t *output)
{
    if ((vqf_instance == nullptr) || (gyro_rps == nullptr) || (accel_mps2 == nullptr) || (output == nullptr) ||
        !IsFiniteVector(gyro_rps, 3U) || !IsFiniteVector(accel_mps2, 3U))
    {
        ++invalid_output_count;
        return false;
    }

    vqf_real_t gyro[3];
    vqf_real_t accel[3];
    for (size_t i = 0U; i < 3U; ++i)
    {
        gyro[i]  = static_cast<vqf_real_t>(gyro_rps[i]);
        accel[i] = static_cast<vqf_real_t>(accel_mps2[i]);
    }

    vqf_instance->update(gyro, accel);

    vqf_real_t quat_enu[4];
    vqf_real_t gyro_bias[3];
    vqf_real_t rest_deviation[2];
    vqf_instance->getQuat6D(quat_enu);
    const vqf_real_t bias_sigma = vqf_instance->getBiasEstimate(gyro_bias);
    vqf_instance->getRelativeRestDeviations(rest_deviation);

    ConvertEnuToNed(quat_enu, output->quat_ned_from_body);
    for (size_t i = 0U; i < 3U; ++i)
    {
        output->gyro_bias_rps[i] = static_cast<float>(gyro_bias[i]);
    }
    output->bias_sigma_rps             = static_cast<float>(bias_sigma);
    output->relative_rest_deviation[0] = static_cast<float>(rest_deviation[0]);
    output->relative_rest_deviation[1] = static_cast<float>(rest_deviation[1]);
    output->rest_detected              = vqf_instance->getRestDetected() ? 1U : 0U;
    output->initialized                = 1U;
    output->reserved[0]                = 0U;
    output->reserved[1]                = 0U;
    output->update_count               = ++update_count;
    output->invalid_output_count       = invalid_output_count;

    const bool valid = IsFiniteVector(output->quat_ned_from_body, 4U) && IsFiniteVector(output->gyro_bias_rps, 3U) &&
                       std::isfinite(output->bias_sigma_rps);
    if (!valid)
    {
        output->initialized          = 0U;
        output->invalid_output_count = ++invalid_output_count;
    }
    return valid;
}
