/**
 * @file App_SystemInit.h
 * @brief 飞控应用层统一初始化入口与可观测状态
 */

#ifndef STM32H743_UAV_FLIGHT_CONTROLLER_APP_SYSTEM_INIT_H
#define STM32H743_UAV_FLIGHT_CONTROLLER_APP_SYSTEM_INIT_H

#include <stdbool.h>
#include <stdint.h>

/* ========================== 安全配置 ========================== */

/*
 * 当前正式配置启用电机输出。
 *
 * 烧录、调试和首次硬件验证必须拆桨，并确认电机编号/旋向、IMU 坐标系、
 * Mixer 符号、SBUS 通道及解锁阈值。启动阶段保持 1000 us 锁定脉宽。
 */
#ifndef APP_ENABLE_MOTOR_OUTPUT
#define APP_ENABLE_MOTOR_OUTPUT 1U
#endif

/* USB 标定模式不启动 SD 卡任务；恢复飞行黑匣子时改为 1U。 */
#ifndef APP_ENABLE_SD_LOGGER
#define APP_ENABLE_SD_LOGGER 1U
#endif

/* ========================== 类型定义 ========================== */

/**
 * @brief 启动阶段各子系统的初始化结果
 *
 * 该结构体保留为全局只读快照，可直接在 VS Code/Ozone Watch 中观察，
 * 用于区分“任务未运行”和“某个硬件初始化失败”。
 */
typedef struct
{
    uint8_t sensor_hub_ready;     /* 当前实机要求的传感器均已完成注册 */
    uint8_t bmi088_ready;         /* 主姿态 IMU 初始化成功 */
    uint8_t bmi270_ready;         /* 备用/对照 IMU 初始化成功 */
    uint8_t communication_ready;  /* MAVLink、USB、DataRouter 初始化成功 */
    uint8_t sd_logger_started;    /* SD 黑匣子任务已创建；硬件结果查看 sd_card_state */
    uint8_t command_ready;        /* SBUS 到控制指令的转换链已启动 */
    uint8_t control_ready;        /* PID 和控制消息订阅已初始化 */
    uint8_t attitude_ready;       /* ESKF 姿态任务及其消息接口已启动 */
    uint8_t motor_output_enabled; /* 编译期是否允许创建电机/控制输出任务 */
    uint8_t motor_ready;          /* 四路 PWM 已初始化并保持锁定输出 */
    uint8_t control_task_started; /* 1 表示闭环控制任务已真正开始运行 */
    uint8_t all_required_ready;   /* 当前编译配置要求的模块是否全部就绪 */
    uint8_t reserved;
} AppSystemStatus_t;

extern volatile AppSystemStatus_t app_system_status;

/* ========================== 接口声明 ========================== */

/**
 * @brief 按依赖顺序初始化完整飞控应用
 *
 * @return true 表示当前编译配置要求的模块全部成功，false 表示至少一项失败
 *
 * @note 必须从 FreeRTOS 任务上下文调用，且只调用一次。
 */
bool App_SystemInit(void);

#endif /* STM32H743_UAV_FLIGHT_CONTROLLER_APP_SYSTEM_INIT_H */
