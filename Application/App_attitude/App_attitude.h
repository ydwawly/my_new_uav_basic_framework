/**
 * @file App_attitude.h
 * @brief 姿态估计模块公共接口
 */

#ifndef APP_ATTITUDE_H
#define APP_ATTITUDE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 初始化姿态估计模块并创建姿态任务
 * @return true 初始化成功，false 初始化失败
 */
bool App_Attitude_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_ATTITUDE_H */
