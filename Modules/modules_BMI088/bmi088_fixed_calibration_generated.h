#ifndef BMI088_FIXED_CALIBRATION_GENERATED_H
#define BMI088_FIXED_CALIBRATION_GENERATED_H

/*
 * 默认单位矩阵。完成实机 LM 后，用 imu_calibration.py --c-header 生成的文件
 * 替换本文件；参数单位分别为 m/s^2 和 rad/s。
 */
/* LM_GUIDED_20260802_223050, with the +Z pose replaced and revalidated. */
/* Model: corrected = matrix * (measured - bias). */
static const float bmi088_accel_bias_mps2[3] = {-9.67887034e-02f, 9.27658614e-03f, -7.48942566e-02f};

static const float bmi088_accel_matrix[3][3] = {{1.01036060e+00f, 3.45574307e-04f, 5.67987154e-03f},
                                                {0.00000000e+00f, 1.00557524e+00f, 3.18033484e-04f},
                                                {0.00000000e+00f, 0.00000000e+00f, 1.00709440e+00f}};

static const float bmi088_gyro_bias_rps[3] = {-1.05562581e-03f, -5.24725821e-03f, -1.05703985e-03f};

static const float bmi088_gyro_matrix[3][3] = {{9.93627931e-01f, -1.56859653e-03f, 2.76755649e-03f},
                                               {4.17691623e-03f, 9.95489396e-01f, 2.06041459e-03f},
                                               {-1.23124572e-03f, -4.04251613e-04f, 9.85605635e-01f}};

#endif
