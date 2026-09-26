/**
 * @file App_TaskPriorities.h
 * @brief 飞控任务优先级的唯一配置表。
 *
 * FreeRTOS 任务优先级数值越大越高。当前 configMAX_PRIORITIES 为 56，
 * 因此应用任务从 55 开始连续向下排列，避免不同模块各自使用零散魔数。
 */

#ifndef APP_TASK_PRIORITIES_H
#define APP_TASK_PRIORITIES_H

#define APP_TASK_PRIORITY_BOOTSTRAP     55U
#define APP_TASK_PRIORITY_ATTITUDE      54U
#define APP_TASK_PRIORITY_CONTROL       53U
#define APP_TASK_PRIORITY_SENSOR_HUB    52U
#define APP_TASK_PRIORITY_UAV_CMD       51U
#define APP_TASK_PRIORITY_MAVLINK_RX    50U
#define APP_TASK_PRIORITY_USB_TX        49U
#define APP_TASK_PRIORITY_DATA_ROUTER   48U
#define APP_TASK_PRIORITY_TIMER_SERVICE 47U
#define APP_TASK_PRIORITY_SD_CARD       46U

#endif /* APP_TASK_PRIORITIES_H */
