/**
 * @file user_math.h
 * @brief 工程公共标量、向量、矩阵、四元数及物理量换算接口
 *
 * 应用层和传感器层只表达业务逻辑，不再各自重复实现限幅、有限值检查、
 * 坐标旋转等基础数学。所有函数均无动态内存、无静态可变状态，可在任务中重入调用。
 */

#ifndef MY_NEW_UAV_BAICE_FRAMEWORK_USER_MATH_H
#define MY_NEW_UAV_BAICE_FRAMEWORK_USER_MATH_H

#include <stdbool.h>
#include <stdint.h>

#define MATH_PI_F         3.14159265358979323846f
#define MATH_TWO_PI_F     (2.0f * MATH_PI_F)
#define MATH_DEG_TO_RAD_F 0.01745329251994329577f
#define MATH_RAD_TO_DEG_F 57.2957795130823208768f

/* 兼容原工程已有名称，新代码优先使用带 MATH_ 前缀的常量。 */
#ifndef PI
#define PI MATH_PI_F
#endif
#ifndef DEG_TO_RAD
#define DEG_TO_RAD MATH_DEG_TO_RAD_F
#endif
#define IMU_X 0
#define IMU_Y 1
#define IMU_Z 2

/**
 * @brief Hamilton 约定的单精度四元数，成员顺序为 [w, x, y, z]
 */
typedef struct
{
    float w;
    float x;
    float y;
    float z;
} MathQuaternionf;

/* ========================== 标量运算 ========================== */

float Math_SquareFloat(float value);
bool  Math_IsFinite(float value);
bool  Math_IsPositiveFinite(float value);
float Math_ClampFloat(float value, float min_value, float max_value);
float Math_Normalize(float value, float min_value, float max_value);
float Math_NormalizeCentered(float value, float min_value, float max_value, float center_value, float deadband);
float Math_WrapPi(float angle_rad);

/* ========================== 向量与矩阵 ========================== */

bool  Math_Vector3IsFinite(const float vector[3]);
float Math_VectorNorm3(const float vector[3]);
void  Math_VectorSkew3(const float vector[3], float skew_matrix[9]);
void  Math_MatrixZero(float *matrix, uint32_t rows, uint32_t columns);
void  Math_MatrixIdentity(float *matrix, uint32_t dimension);
void  Math_RotateVector3(const float rotation[9], const float vector[3], float output[3]);
void  Math_RotateVector3Transpose(const float rotation[9], const float vector[3], float output[3]);

/* ========================== 四元数与姿态 ========================== */

MathQuaternionf Math_QuaternionIdentity(void);
MathQuaternionf Math_QuaternionConjugate(MathQuaternionf quaternion);
MathQuaternionf Math_QuaternionMultiply(MathQuaternionf left, MathQuaternionf right);
MathQuaternionf Math_QuaternionNormalize(MathQuaternionf quaternion);
MathQuaternionf Math_QuaternionFromEuler(float roll_rad, float pitch_rad, float yaw_rad);
MathQuaternionf Math_QuaternionFromRotationVector(const float rotation_vector_rad[3]);
void            Math_QuaternionToRotationMatrix(MathQuaternionf quaternion, float rotation[9]);
void            Math_QuaternionErrorRotationVector(MathQuaternionf estimate, MathQuaternionf measurement,
                                                   float error_rotation_vector_rad[3]);

/**
 * @brief 兼容旧接口：将欧拉角转换为 [w, x, y, z] 数组
 */
void Math_EulerToQuaternion(float roll_rad, float pitch_rad, float yaw_rad, float quaternion[4]);

/* ========================== 数据完整性校验 ========================== */

/**
 * @brief 计算 CRC-16/CCITT-FALSE 校验值。
 * @param data          待校验字节流；length 为 0 时允许传入 NULL。
 * @param length        待校验字节数。
 * @param initial_value 初始值，标准 CRC-16/CCITT-FALSE 使用 0xFFFF。
 * @return 多项式 0x1021、无反射、无异或输出的 16 位 CRC。
 */
uint16_t Math_Crc16Ccitt(const void *data, uint32_t length, uint16_t initial_value);

/* ========================== 常用物理量换算 ========================== */

/**
 * @brief 按国际标准大气模型把绝对气压换算为海拔高度
 * @param pressure_pa        当前绝对气压，单位 Pa
 * @param sea_level_pressure 海平面参考气压，单位 Pa，通常为 101325 Pa
 * @return 海拔高度，单位 m；输入无效时返回 NAN
 */
float Math_PressureToAltitude(float pressure_pa, float sea_level_pressure_pa);

#endif /* MY_NEW_UAV_BAICE_FRAMEWORK_USER_MATH_H */
