/**
 * @file user_math.c
 * @brief 工程公共基础数学函数实现
 */

#include "user_math.h"

#include <math.h>
#include <string.h>

/* ========================== 标量运算 ========================== */

float Math_SquareFloat(float value)
{
    return value * value;
}

bool Math_IsFinite(float value)
{
    return isfinite(value);
}

bool Math_IsPositiveFinite(float value)
{
    return isfinite(value) && (value > 0.0f);
}

float Math_ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

float Math_Normalize(float value, float min_value, float max_value)
{
    if ((!isfinite(value)) || (!isfinite(min_value)) || (!isfinite(max_value)) || (max_value <= min_value))
    {
        return 0.0f;
    }

    return Math_ClampFloat((value - min_value) / (max_value - min_value), 0.0f, 1.0f);
}

float Math_NormalizeCentered(float value, float min_value, float max_value, float center_value, float deadband)
{
    if ((!isfinite(value)) || (!isfinite(min_value)) || (!isfinite(max_value)) || (!isfinite(center_value)) ||
        (!isfinite(deadband)) || (deadband < 0.0f) || (center_value <= min_value) || (center_value >= max_value))
    {
        return 0.0f;
    }

    if ((value >= (center_value - deadband)) && (value <= (center_value + deadband)))
    {
        return 0.0f;
    }

    if (value > center_value)
    {
        const float denominator = max_value - center_value - deadband;
        return (denominator > 0.0f) ? Math_ClampFloat((value - center_value - deadband) / denominator, 0.0f, 1.0f)
                                    : 0.0f;
    }

    const float denominator = center_value - deadband - min_value;
    return (denominator > 0.0f) ? Math_ClampFloat(-(center_value - deadband - value) / denominator, -1.0f, 0.0f) : 0.0f;
}

float Math_WrapPi(float angle_rad)
{
    if (!isfinite(angle_rad))
    {
        return angle_rad;
    }

    angle_rad = fmodf(angle_rad + MATH_PI_F, MATH_TWO_PI_F);
    if (angle_rad < 0.0f)
    {
        angle_rad += MATH_TWO_PI_F;
    }

    return angle_rad - MATH_PI_F;
}

/* ========================== 向量与矩阵 ========================== */

bool Math_Vector3IsFinite(const float vector[3])
{
    return (vector != NULL) && isfinite(vector[0]) && isfinite(vector[1]) && isfinite(vector[2]);
}

float Math_VectorNorm3(const float vector[3])
{
    if (vector == NULL)
    {
        return 0.0f;
    }

    return sqrtf(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
}

void Math_VectorSkew3(const float vector[3], float skew_matrix[9])
{
    if ((vector == NULL) || (skew_matrix == NULL))
    {
        return;
    }

    skew_matrix[0] = 0.0f;
    skew_matrix[1] = -vector[2];
    skew_matrix[2] = vector[1];
    skew_matrix[3] = vector[2];
    skew_matrix[4] = 0.0f;
    skew_matrix[5] = -vector[0];
    skew_matrix[6] = -vector[1];
    skew_matrix[7] = vector[0];
    skew_matrix[8] = 0.0f;
}

void Math_MatrixZero(float *matrix, uint32_t rows, uint32_t columns)
{
    if ((matrix == NULL) || (rows == 0U) || (columns == 0U))
    {
        return;
    }

    memset(matrix, 0, sizeof(float) * rows * columns);
}

void Math_MatrixIdentity(float *matrix, uint32_t dimension)
{
    if ((matrix == NULL) || (dimension == 0U))
    {
        return;
    }

    Math_MatrixZero(matrix, dimension, dimension);
    for (uint32_t index = 0U; index < dimension; index++)
    {
        matrix[index * dimension + index] = 1.0f;
    }
}

void Math_RotateVector3(const float rotation[9], const float vector[3], float output[3])
{
    if ((rotation == NULL) || (vector == NULL) || (output == NULL))
    {
        return;
    }

    output[0] = rotation[0] * vector[0] + rotation[1] * vector[1] + rotation[2] * vector[2];
    output[1] = rotation[3] * vector[0] + rotation[4] * vector[1] + rotation[5] * vector[2];
    output[2] = rotation[6] * vector[0] + rotation[7] * vector[1] + rotation[8] * vector[2];
}

void Math_RotateVector3Transpose(const float rotation[9], const float vector[3], float output[3])
{
    if ((rotation == NULL) || (vector == NULL) || (output == NULL))
    {
        return;
    }

    output[0] = rotation[0] * vector[0] + rotation[3] * vector[1] + rotation[6] * vector[2];
    output[1] = rotation[1] * vector[0] + rotation[4] * vector[1] + rotation[7] * vector[2];
    output[2] = rotation[2] * vector[0] + rotation[5] * vector[1] + rotation[8] * vector[2];
}

/* ========================== 四元数与姿态 ========================== */

MathQuaternionf Math_QuaternionIdentity(void)
{
    const MathQuaternionf identity = {.w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f};
    return identity;
}

MathQuaternionf Math_QuaternionConjugate(MathQuaternionf quaternion)
{
    const MathQuaternionf conjugate = {.w = quaternion.w, .x = -quaternion.x, .y = -quaternion.y, .z = -quaternion.z};
    return conjugate;
}

MathQuaternionf Math_QuaternionMultiply(MathQuaternionf left, MathQuaternionf right)
{
    MathQuaternionf output;

    output.w = left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z;
    output.x = left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y;
    output.y = left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x;
    output.z = left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w;

    return output;
}

MathQuaternionf Math_QuaternionNormalize(MathQuaternionf quaternion)
{
    const float norm_squared = quaternion.w * quaternion.w + quaternion.x * quaternion.x + quaternion.y * quaternion.y +
                               quaternion.z * quaternion.z;

    if ((!isfinite(norm_squared)) || (norm_squared <= 1.0e-12f))
    {
        return Math_QuaternionIdentity();
    }

    const float norm = sqrtf(norm_squared);
    if ((!isfinite(norm)) || (norm <= 1.0e-6f))
    {
        return Math_QuaternionIdentity();
    }

    const float inverse_norm = 1.0f / norm;
    quaternion.w *= inverse_norm;
    quaternion.x *= inverse_norm;
    quaternion.y *= inverse_norm;
    quaternion.z *= inverse_norm;
    return quaternion;
}

MathQuaternionf Math_QuaternionFromEuler(float roll_rad, float pitch_rad, float yaw_rad)
{
    const float half_roll  = 0.5f * roll_rad;
    const float half_pitch = 0.5f * pitch_rad;
    const float half_yaw   = 0.5f * yaw_rad;
    const float cos_roll   = cosf(half_roll);
    const float sin_roll   = sinf(half_roll);
    const float cos_pitch  = cosf(half_pitch);
    const float sin_pitch  = sinf(half_pitch);
    const float cos_yaw    = cosf(half_yaw);
    const float sin_yaw    = sinf(half_yaw);

    MathQuaternionf quaternion;
    quaternion.w = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw;
    quaternion.x = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw;
    quaternion.y = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw;
    quaternion.z = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw;
    return Math_QuaternionNormalize(quaternion);
}

MathQuaternionf Math_QuaternionFromRotationVector(const float rotation_vector_rad[3])
{
    if (rotation_vector_rad == NULL)
    {
        return Math_QuaternionIdentity();
    }

    const float     angle_rad = Math_VectorNorm3(rotation_vector_rad);
    MathQuaternionf quaternion;

    if (angle_rad < 1.0e-6f)
    {
        quaternion.w = 1.0f;
        quaternion.x = 0.5f * rotation_vector_rad[0];
        quaternion.y = 0.5f * rotation_vector_rad[1];
        quaternion.z = 0.5f * rotation_vector_rad[2];
        return Math_QuaternionNormalize(quaternion);
    }

    const float half_angle = 0.5f * angle_rad;
    const float scale      = sinf(half_angle) / angle_rad;
    quaternion.w           = cosf(half_angle);
    quaternion.x           = scale * rotation_vector_rad[0];
    quaternion.y           = scale * rotation_vector_rad[1];
    quaternion.z           = scale * rotation_vector_rad[2];
    return quaternion;
}

void Math_QuaternionToRotationMatrix(MathQuaternionf quaternion, float rotation[9])
{
    if (rotation == NULL)
    {
        return;
    }

    quaternion = Math_QuaternionNormalize(quaternion);

    const float ww = quaternion.w * quaternion.w;
    const float xx = quaternion.x * quaternion.x;
    const float yy = quaternion.y * quaternion.y;
    const float zz = quaternion.z * quaternion.z;

    rotation[0] = ww + xx - yy - zz;
    rotation[1] = 2.0f * (quaternion.x * quaternion.y - quaternion.w * quaternion.z);
    rotation[2] = 2.0f * (quaternion.x * quaternion.z + quaternion.w * quaternion.y);
    rotation[3] = 2.0f * (quaternion.x * quaternion.y + quaternion.w * quaternion.z);
    rotation[4] = ww - xx + yy - zz;
    rotation[5] = 2.0f * (quaternion.y * quaternion.z - quaternion.w * quaternion.x);
    rotation[6] = 2.0f * (quaternion.x * quaternion.z - quaternion.w * quaternion.y);
    rotation[7] = 2.0f * (quaternion.y * quaternion.z + quaternion.w * quaternion.x);
    rotation[8] = ww - xx - yy + zz;
}

void Math_QuaternionErrorRotationVector(MathQuaternionf estimate, MathQuaternionf measurement,
                                        float error_rotation_vector_rad[3])
{
    if (error_rotation_vector_rad == NULL)
    {
        return;
    }

    MathQuaternionf error = Math_QuaternionMultiply(Math_QuaternionConjugate(Math_QuaternionNormalize(estimate)),
                                                    Math_QuaternionNormalize(measurement));

    /* q 与 -q 表示同一旋转，固定在 w >= 0 的半球可得到最短旋转弧。 */
    if (error.w < 0.0f)
    {
        error.w = -error.w;
        error.x = -error.x;
        error.y = -error.y;
        error.z = -error.z;
    }

    const float vector_part[3] = {error.x, error.y, error.z};
    const float vector_norm    = Math_VectorNorm3(vector_part);

    if (vector_norm < 1.0e-7f)
    {
        error_rotation_vector_rad[0] = 2.0f * error.x;
        error_rotation_vector_rad[1] = 2.0f * error.y;
        error_rotation_vector_rad[2] = 2.0f * error.z;
        return;
    }

    const float angle_rad        = 2.0f * atan2f(vector_norm, error.w);
    const float scale            = angle_rad / vector_norm;
    error_rotation_vector_rad[0] = scale * error.x;
    error_rotation_vector_rad[1] = scale * error.y;
    error_rotation_vector_rad[2] = scale * error.z;
}

void Math_EulerToQuaternion(float roll_rad, float pitch_rad, float yaw_rad, float quaternion[4])
{
    if (quaternion == NULL)
    {
        return;
    }

    const MathQuaternionf result = Math_QuaternionFromEuler(roll_rad, pitch_rad, yaw_rad);
    quaternion[0]                = result.w;
    quaternion[1]                = result.x;
    quaternion[2]                = result.y;
    quaternion[3]                = result.z;
}

/* ========================== 数据完整性校验 ========================== */

/**
 * @brief 逐字节计算 CRC-16/CCITT-FALSE
 *
 * 使用多项式 0x1021、最高位优先、无输入/输出反射且无最终异或。initial_value
 * 由上层传入，因此既可以一次计算完整数据，也可以将上一段结果作为下一段初值。
 */
uint16_t Math_Crc16Ccitt(const void *data, uint32_t length, uint16_t initial_value)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint16_t       crc   = initial_value;

    if ((bytes == NULL) && (length > 0U))
    {
        return initial_value;
    }

    for (uint32_t byte_index = 0U; byte_index < length; byte_index++)
    {
        crc ^= (uint16_t)bytes[byte_index] << 8U;

        for (uint8_t bit_index = 0U; bit_index < 8U; bit_index++)
        {
            crc = ((crc & 0x8000U) != 0U) ? (uint16_t)((crc << 1U) ^ 0x1021U) : (uint16_t)(crc << 1U);
        }
    }

    return crc;
}

/* ========================== 常用物理量换算 ========================== */

float Math_PressureToAltitude(float pressure_pa, float sea_level_pressure_pa)
{
    if ((!Math_IsPositiveFinite(pressure_pa)) || (!Math_IsPositiveFinite(sea_level_pressure_pa)))
    {
        return NAN;
    }

    return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pressure_pa, 1.0f / 5.255f));
}
