/**
 * @file App_Sensor.h
 * @brief 传感器汇聚模块公共接口
 */

#ifndef MY_NEW_UAV_BAICE_FRAMEWORK_APP_SENSOR_H
#define MY_NEW_UAV_BAICE_FRAMEWORK_APP_SENSOR_H

#include <stdint.h>

/*
 * 当前实机未安装 GPS，因此默认不注册 GPS 串口和解析任务。
 * 后续装入 GPS 后可在编译选项中定义 APP_SENSOR_ENABLE_GPS=1，
 * 无需改动 SensorHub_Init() 的初始化顺序。
 */
#ifndef APP_SENSOR_ENABLE_GPS
#define APP_SENSOR_ENABLE_GPS 0U
#endif

/* 每一位表示对应传感器已完成初始化，任务只处理已经就绪的设备。 */
#define SENSOR_READY_MTF02         (1UL << 0)
#define SENSOR_READY_TFMINI        (1UL << 1)
#define SENSOR_READY_SPL06         (1UL << 2)
#define SENSOR_READY_SBUS          (1UL << 3)
#define SENSOR_READY_GPS           (1UL << 4)
#define SENSOR_READY_REQUIRED_BASE (SENSOR_READY_MTF02 | SENSOR_READY_TFMINI | SENSOR_READY_SPL06 | SENSOR_READY_SBUS)

#define SENSOR_HUB_TASK_STACK_WORDS 512U
/**
 * @brief 传感器汇聚模块的初始化状态
 *
 * 该结构体可直接加入 VS Code Watch。GPS 的 enabled 与 ready 分开记录，
 * 避免“硬件未安装”与“硬件初始化失败”混为一谈。
 */
typedef struct
{
    uint8_t mtf02_ready;    /* MTF-02 光流/测距串口已启动 */
    uint8_t tfmini_ready;   /* TFmini Plus 激光测距串口已启动 */
    uint8_t spl06_ready;    /* SPL06 气压计已通过 I2C2 初始化并进入 DMA 采集 */
    uint8_t sbus_ready;     /* SBUS 遥控接收串口已启动 */
    uint8_t gps_enabled;    /* 当前编译配置是否要求 GPS */
    uint8_t gps_ready;      /* GPS 已启用且串口初始化成功 */
    uint8_t required_ready; /* 当前实机要求的传感器是否全部完成初始化 */
    uint8_t reserved;
} AppSensorStatus_t;

extern volatile AppSensorStatus_t app_sensor_status;

/**
 * @brief 创建传感器汇聚任务并按顺序初始化 UART/I2C 传感器
 *
 * @return 1 表示当前编译配置要求的传感器全部初始化成功，0 表示至少一项失败
 *
 * @note 必须在 FreeRTOS 调度器启动后调用。各串口初始化会立即打开 DMA/中断，
 *       若在调度器启动前执行，中断回调中的 xTaskNotifyFromISR() 没有安全的任务上下文。
 */
uint8_t SensorHub_Init(void);

/**
 * @brief 传感器汇聚任务入口
 *
 * UART/I2C DMA 中断只保存原始数据并置通知位；校验、单位换算和消息发布均在
 * 本任务中执行，从而缩短中断执行时间。任务末尾还会处理有界的 IMU FFT 工作，
 * 且每轮最多计算一个轴，避免一次三轴 FFT 阻塞传感器事件。
 */
void SensorHub_Task(void *pv_parameters);

#endif /* MY_NEW_UAV_BAICE_FRAMEWORK_APP_SENSOR_H */
