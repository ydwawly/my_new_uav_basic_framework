#include "ESKF.h"

#include <math.h>

void NAV_ESKF_GetState(const NavESKF *e, NavESKFState *out_state)
{
    if ((e == NULL) || (out_state == NULL))
    {
        return;
    }
    *out_state = e->state;
}

void NAV_ESKF_GetEulerRad(const NavESKF *e, float32_t *roll, float32_t *pitch, float32_t *yaw)
{
    if (e == NULL)
    {
        return;
    }

    const NavQuatf q = Math_QuaternionNormalize(e->state.q_nb);

    /* 经典的航空 3-2-1 欧拉角四元数转换公式 (FRD 机体系 -> NED 导航系) */
    const float32_t sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    const float32_t cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    const float32_t sinp      = 2.0f * (q.w * q.y - q.z * q.x);
    const float32_t siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    const float32_t cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);

    if (roll != NULL)
    {
        *roll = atan2f(sinr_cosp, cosr_cosp);
    }
    if (pitch != NULL)
    {
        /* 对单精度浮点做边界裁剪防非安全区间导致 asinf 产生 NaN */
        *pitch = asinf(fmaxf(-1.0f, fminf(1.0f, sinp)));
    }
    if (yaw != NULL)
    {
        *yaw = atan2f(siny_cosp, cosy_cosp);
    }
}
