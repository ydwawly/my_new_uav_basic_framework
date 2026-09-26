//
// Created by Administrator on 2026/7/18.
//

#ifndef MY_NEW_UAV_BAICE_FRAMEWORK_APP_DATA_COMM_H
#define MY_NEW_UAV_BAICE_FRAMEWORK_APP_DATA_COMM_H

#include <stdbool.h>

#include "DataRouterTask.h"
#include "bsp_usb.h"
#include "hil_service.h"
#include "mavlink_user.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
     * @brief 初始化整个应用层的数据通信框架
     *
     * 涵盖：消息总线、各个业务 Service (System, Command, HIL等)、
     * MAVLink 协议栈、USB 硬件接口，以及相关 FreeRTOS 任务的创建。
     *
     * @return 初始化成功返回 true，否则返回 false
     */
bool App_CommInit(void);

/**
     * @brief 获取当前通信模块使用的 USB 实例
     * @return USBInstance* 指向全局 USB 实例的指针
     */
USBInstance *App_GetUSBInstance(void);

#ifdef __cplusplus
}
#endif

#endif // MY_NEW_UAV_BAICE_FRAMEWORK_APP_DATA_COMM_H