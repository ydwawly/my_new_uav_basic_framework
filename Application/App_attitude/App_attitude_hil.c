#include "App_attitude_hil.h"

/**
 * @file App_attitude_hil.c
 * @brief 将 MAVLink HIL 传感器与定位数据转换为导航 ESKF 可融合的观测量。
 *
 * HIL_SENSOR 提供高频 IMU、磁场和气压数据，HIL_GPS 提供低频位置/速度数据；
 * 本模块同时维护气压零点和局部 NED 原点。所有状态仅由姿态任务访问。
 */

#if (APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_HIL)

#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "bsp_RTT.h"
#include "modules_Message_center.h"
#include "user_math.h"

typedef struct
{
    double  latitude_rad;
    double  longitude_rad;
    double  altitude_m;
    double  meridian_radius_m;
    double  prime_vertical_radius_m;
    uint8_t valid;
} AppAttitudeHilOrigin_t;

/* 各订阅者由姿态任务串行消费，不在中断上下文访问。 */
static Subscriber_t *hil_sensor_subscriber;
static Subscriber_t *hil_gps_subscriber;
static Subscriber_t *hil_state_subscriber;

static uint64_t               hil_last_mag_update_us;
static uint64_t               hil_last_baro_update_us;
static double                 hil_baro_reference_sum;
static float                  hil_baro_reference_altitude_m;
static uint16_t               hil_baro_reference_count;
static uint8_t                hil_baro_reference_valid;
static AppAttitudeHilOrigin_t hil_gps_origin;

/** @brief 注册 HIL 数据主题；任一订阅失败时初始化失败。 */
bool App_Attitude_Hil_Init(void)
{
    hil_sensor_subscriber = SubRegister(HIL_SENSOR_TOPIC_NAME, sizeof(HIL_Sensor_Data_t));
    hil_gps_subscriber    = SubRegister(HIL_GPS_TOPIC_NAME, sizeof(HIL_GPS_Data_t));
    hil_state_subscriber  = SubRegister(HIL_STATE_TOPIC_NAME, sizeof(HIL_State_Data_t));

    if ((hil_sensor_subscriber == NULL) || (hil_gps_subscriber == NULL) || (hil_state_subscriber == NULL))
    {
        RTTERROR("[Attitude] HIL subscriber register failed.");
        return false;
    }
    return true;
}

/** @brief 清除本地 GPS/气压基准以及观测节流时间戳。 */
void App_Attitude_Hil_Reset(void)
{
    memset(&hil_gps_origin, 0, sizeof(hil_gps_origin));
    hil_last_mag_update_us        = 0ULL;
    hil_last_baro_update_us       = 0ULL;
    hil_baro_reference_sum        = 0.0;
    hil_baro_reference_altitude_m = 0.0f;
    hil_baro_reference_count      = 0U;
    hil_baro_reference_valid      = 0U;
}

/**
 * @brief 可选地使用 HIL 真值姿态和速度初始化 ESKF。
 * @return 真值功能已启用且收到合法状态时返回 true，否则返回 false。
 */
bool App_Attitude_Hil_TryGetInitialState(NavESKFState *initial_state)
{
#if (ATTITUDE_HIL_USE_TRUTH_INITIAL_STATE == 1U)
    HIL_State_Data_t truth;
    if ((initial_state == NULL) || (SubGetMessage(hil_state_subscriber, &truth, 0U) == 0U))
        return false;

    const float q_norm2 = truth.quaternion[0] * truth.quaternion[0] + truth.quaternion[1] * truth.quaternion[1] +
                          truth.quaternion[2] * truth.quaternion[2] + truth.quaternion[3] * truth.quaternion[3];
    if ((!isfinite(q_norm2)) || (q_norm2 < 0.25f) || (!Math_Vector3IsFinite(truth.velocity_ned)))
        return false;

    memset(initial_state, 0, sizeof(*initial_state));
    initial_state->q_nb.w = truth.quaternion[0];
    initial_state->q_nb.x = truth.quaternion[1];
    initial_state->q_nb.y = truth.quaternion[2];
    initial_state->q_nb.z = truth.quaternion[3];
    memcpy(initial_state->vel_ned_mps, truth.velocity_ned, sizeof(initial_state->vel_ned_mps));
    return true;
#else
    (void)initial_state;
    return false;
#endif
}

/** @brief 阻塞等待下一帧 HIL_SENSOR，供姿态任务作为主采样节拍。 */
bool App_Attitude_Hil_WaitForSensor(HIL_Sensor_Data_t *sensor)
{
    return (sensor != NULL) && (SubGetMessage(hil_sensor_subscriber, sensor, portMAX_DELAY) != 0U);
}

/**
 * @brief 判断 HIL_SENSOR 的更新位是否同时包含三轴加速度计和三轴陀螺仪。
 * @note fields_updated 为 0 时按 MAVLink 发送端未提供位图处理，兼容旧仿真器。
 */
bool App_Attitude_Hil_HasCompleteImu(uint32_t fields_updated)
{
    if (fields_updated == 0U)
        return true;

    const uint32_t accel_mask = HIL_SENSOR_UPDATED_XACC | HIL_SENSOR_UPDATED_YACC | HIL_SENSOR_UPDATED_ZACC;
    const uint32_t gyro_mask  = HIL_SENSOR_UPDATED_XGYRO | HIL_SENSOR_UPDATED_YGYRO | HIL_SENSOR_UPDATED_ZGYRO;
    return (fields_updated & (accel_mask | gyro_mask)) == (accel_mask | gyro_mask);
}

/** @brief 以第一帧有效 3D GPS 定位建立 WGS-84 局部切平面原点。 */
static bool Attitude_SetGpsOrigin(const HIL_GPS_Data_t *gps)
{
    const double wgs84_a      = 6378137.0;
    const double wgs84_e2     = 6.6943799901413165e-3;
    const double deg_to_rad   = 0.017453292519943295769;
    const double latitude_rad = gps->latitude * deg_to_rad;
    const double sin_latitude = sin(latitude_rad);
    const double denominator  = sqrt(1.0 - wgs84_e2 * sin_latitude * sin_latitude);

    if ((!isfinite(denominator)) || (denominator <= 0.0))
        return false;

    hil_gps_origin.latitude_rad            = latitude_rad;
    hil_gps_origin.longitude_rad           = gps->longitude * deg_to_rad;
    hil_gps_origin.altitude_m              = gps->altitude;
    hil_gps_origin.prime_vertical_radius_m = wgs84_a / denominator;
    hil_gps_origin.meridian_radius_m       = wgs84_a * (1.0 - wgs84_e2) / (denominator * denominator * denominator);
    hil_gps_origin.valid                   = 1U;
    return true;
}

/**
 * @brief 使用小范围切平面近似将经纬高转换为相对原点的 NED 坐标。
 * @param position_ned_m 输出北、东、地三个方向的位置，单位 m。
 */
static bool Attitude_GpsToNed(const HIL_GPS_Data_t *gps, float position_ned_m[3])
{
    if ((!hil_gps_origin.valid) || (gps == NULL) || (position_ned_m == NULL))
        return false;

    const double deg_to_rad      = 0.017453292519943295769;
    const double delta_latitude  = gps->latitude * deg_to_rad - hil_gps_origin.latitude_rad;
    const double delta_longitude = gps->longitude * deg_to_rad - hil_gps_origin.longitude_rad;

    position_ned_m[0] = (float)(delta_latitude * (hil_gps_origin.meridian_radius_m + hil_gps_origin.altitude_m));
    position_ned_m[1] = (float)(delta_longitude * (hil_gps_origin.prime_vertical_radius_m + hil_gps_origin.altitude_m) *
                                cos(hil_gps_origin.latitude_rad));
    position_ned_m[2] = (float)(hil_gps_origin.altitude_m - (double)gps->altitude);
    return true;
}

/** @brief 校验仿真器报告的标准差，并施加配置的可信度下限。 */
static float Attitude_ChooseGpsStd(float reported, float minimum)
{
    if ((!isfinite(reported)) || (reported <= 0.001f) || (reported > 100.0f))
        return minimum;
    return (reported > minimum) ? reported : minimum;
}

/** @brief 按最小更新间隔融合机体系磁场，磁场单位 gauss。 */
static void Attitude_UpdateMag(NavESKF *eskf, const HIL_Sensor_Data_t *sensor, uint64_t timestamp_us)
{
    if (((timestamp_us - hil_last_mag_update_us) < ATTITUDE_HIL_MAG_MIN_INTERVAL_US) ||
        (!Math_Vector3IsFinite(sensor->mag)))
        return;

    const NavMagObservation observation = {
        .mag_body_gauss             = {sensor->mag[0], sensor->mag[1], sensor->mag[2]},
        .declination_rad            = ATTITUDE_MAG_DECLINATION_RAD,
        .heading_std_rad            = ATTITUDE_HIL_MAG_HEADING_STD_RAD,
        .field_norm_min_gauss       = ATTITUDE_MAG_FIELD_MIN_GAUSS,
        .field_norm_max_gauss       = ATTITUDE_MAG_FIELD_MAX_GAUSS,
        .horizontal_field_min_gauss = ATTITUDE_MAG_HORIZONTAL_MIN_GAUSS,
    };

    hil_last_mag_update_us = timestamp_us;
    (void)NAV_ESKF_UpdateMag(eskf, &observation);
}

/**
 * @brief 建立气压高度零点后融合相对高度。
 * @note 初始化阶段先对若干 pressure_alt 样本求均值，避免把海拔绝对值直接当作局部高度。
 */
static void Attitude_UpdateBarometer(NavESKF *eskf, const HIL_Sensor_Data_t *sensor, uint64_t timestamp_us)
{
    if (((timestamp_us - hil_last_baro_update_us) < ATTITUDE_HIL_BARO_MIN_INTERVAL_US) ||
        (!isfinite(sensor->pressure_alt)))
        return;

    hil_last_baro_update_us = timestamp_us;
    if (!hil_baro_reference_valid)
    {
        hil_baro_reference_sum += (double)sensor->pressure_alt;
        hil_baro_reference_count++;
        if (hil_baro_reference_count >= ATTITUDE_HIL_BARO_REFERENCE_SAMPLES)
        {
            hil_baro_reference_altitude_m = (float)(hil_baro_reference_sum / (double)hil_baro_reference_count);
            hil_baro_reference_valid      = 1U;
        }
        return;
    }

    const NavBaroObservation observation = {
        .altitude_up_m = sensor->pressure_alt - hil_baro_reference_altitude_m,
        .std_m         = ATTITUDE_HIL_BARO_STD_M,
    };
    (void)NAV_ESKF_UpdateBaro(eskf, &observation);
}

/** @brief 非阻塞读取最新 GPS；首个有效定位仅用于建立原点，后续数据才进入 ESKF。 */
static void Attitude_UpdateGps(NavESKF *eskf)
{
    HIL_GPS_Data_t gps;
    float          position_ned_m[3];

    if ((SubGetMessage(hil_gps_subscriber, &gps, 0U) == 0U) || (gps.fix_type < 3U) ||
        (!Math_Vector3IsFinite(gps.velocity_ned)))
        return;

    if (!hil_gps_origin.valid)
    {
        (void)Attitude_SetGpsOrigin(&gps);
        return;
    }

    if (!Attitude_GpsToNed(&gps, position_ned_m))
        return;

    const float             horizontal_std = Attitude_ChooseGpsStd(gps.eph, ATTITUDE_HIL_GPS_HORIZONTAL_STD_M);
    const float             vertical_std   = Attitude_ChooseGpsStd(gps.epv, ATTITUDE_HIL_GPS_VERTICAL_STD_M);
    const NavGpsObservation observation    = {
        .use_position = true,
        .use_velocity = true,
        .pos_ned_m    = {position_ned_m[0], position_ned_m[1], position_ned_m[2]},
        .vel_ned_mps  = {gps.velocity_ned[0], gps.velocity_ned[1], gps.velocity_ned[2]},
        .pos_std_m    = {horizontal_std, horizontal_std, vertical_std},
        .vel_std_mps  = {ATTITUDE_HIL_GPS_VEL_XY_STD_MPS, ATTITUDE_HIL_GPS_VEL_XY_STD_MPS,
                         ATTITUDE_HIL_GPS_VEL_Z_STD_MPS},
    };
    (void)NAV_ESKF_UpdateGps(eskf, &observation);
}

/** @brief 在估计器完成初始化后依次融合磁场、气压和 GPS 观测。 */
void App_Attitude_Hil_UpdateObservations(NavESKF *eskf, const HIL_Sensor_Data_t *sensor, uint64_t timestamp_us,
                                         bool estimator_initialized)
{
    if ((eskf == NULL) || (sensor == NULL) || (!estimator_initialized))
        return;

    Attitude_UpdateMag(eskf, sensor, timestamp_us);
    Attitude_UpdateBarometer(eskf, sensor, timestamp_us);
    Attitude_UpdateGps(eskf);
}

#endif
