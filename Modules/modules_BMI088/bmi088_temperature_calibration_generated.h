#ifndef BMI088_TEMPERATURE_CALIBRATION_GENERATED_H
#define BMI088_TEMPERATURE_CALIBRATION_GENERATED_H

/*
 * BMI088 gyroscope temperature-drift model validated by the independent
 * round-3 cold-to-hot capture.
 *
 * Source captures:
 *   output/temperature_capture/BMI088_USB_20260802_123330_COM11_part001.BIN
 *   output/temperature_capture/BMI088_USB_20260802_141932_COM11_part001.BIN
 * Model:
 *   delta_bias(T) = c1 * (T - T_ref) + c2 * (T - T_ref)^2
 *
 * The round-1 coefficients are retained on Y. X and Z are shrunk by 0.591
 * and 0.771 respectively to minimize the worst temperature-bin peak-to-peak
 * drift across the first two independent cold-to-hot captures. These are
 * final values fitted with the first two runs and retained after the
 * independent round-3 validation passed the frozen acceptance criteria.
 *
 * Coefficients are expressed in the mapped, pre-LM sensor frame used by
 * gyro_uncalibrated[]. The temperature input is clamped to the observed fit
 * range.
 */
#ifndef BMI088_ENABLE_GYRO_TEMPERATURE_COMPENSATION
#define BMI088_ENABLE_GYRO_TEMPERATURE_COMPENSATION 1U
#endif

#if ((BMI088_ENABLE_GYRO_TEMPERATURE_COMPENSATION != 0U) && (BMI088_ENABLE_GYRO_TEMPERATURE_COMPENSATION != 1U))
#error "BMI088_ENABLE_GYRO_TEMPERATURE_COMPENSATION must be 0U or 1U"
#endif

static const float bmi088_gyro_temp_reference_c = 45.7043125f;
static const float bmi088_gyro_temp_minimum_c   = 27.25f;
static const float bmi088_gyro_temp_maximum_c   = 46.0f;

static const float bmi088_gyro_temp_c1_rps_per_c[3] = {
    -3.2220137e-05f,
    -2.6622598e-04f,
    -2.6712068e-05f,
};

static const float bmi088_gyro_temp_c2_rps_per_c2[3] = {
    -1.2759631e-06f,
    -3.9388435e-06f,
    -3.7307752e-06f,
};

#endif
