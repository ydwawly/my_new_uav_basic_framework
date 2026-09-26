/**
 * @file App_Sensor.c
 * @brief UART/I2C 传感器统一初始化与任务级数据处理
 */

#include "App_Sensor.h"

#include "App_TaskPriorities.h"
#include "App_ImuFft.h"
#include "FreeRTOS.h"
#include "modules_Microvoid_MG.h"
#include "modules_Sbus.h"
#include "modules_TFmini_Plus.h"
#include "modules_SPL06.h"
#include "modules_mtf02.h"
#include "task.h"

volatile AppSensorStatus_t app_sensor_status;

static StaticTask_t sensor_hub_task_control;
static StackType_t  sensor_hub_task_stack[SENSOR_HUB_TASK_STACK_WORDS];
static TaskHandle_t sensor_hub_task_handle = NULL;
static uint32_t     sensor_ready_mask      = 0U;

uint8_t SensorHub_Init(void)
{
    uint32_t required_mask = SENSOR_READY_REQUIRED_BASE;

#if (APP_SENSOR_ENABLE_GPS == 1U)
    required_mask |= SENSOR_READY_GPS;
#endif

    if (sensor_hub_task_handle != NULL)
    {
        return ((sensor_ready_mask & required_mask) == required_mask) ? 1U : 0U;
    }

    app_sensor_status             = (AppSensorStatus_t){0};
    app_sensor_status.gps_enabled = APP_SENSOR_ENABLE_GPS;

    /*
     * 静态任务栈不占用 configTOTAL_HEAP_SIZE。通信模块还会创建三个动态任务，
     * 因此传感器任务使用静态内存可避免启动阶段因 15 KB FreeRTOS 堆耗尽而失败。
     */
    sensor_hub_task_handle = xTaskCreateStatic(SensorHub_Task, "sensor_hub", SENSOR_HUB_TASK_STACK_WORDS, NULL,
                                               APP_TASK_PRIORITY_SENSOR_HUB, sensor_hub_task_stack,
                                               &sensor_hub_task_control);
    if (sensor_hub_task_handle == NULL)
    {
        return 0U;
    }

    /*
     * 先注册任务句柄，再启动 UART/I2C DMA。即使传感器在初始化完成后立即来数，
     * ISR 也能把通知投递给一个有效任务，不会丢失第一次就绪事件。
     */
    Mtf02_RegisterReadyTask(sensor_hub_task_handle);
    Tfmini_RegisterReadyTask(sensor_hub_task_handle);
    SPL06_RegisterReadyTask(sensor_hub_task_handle);
    Sbus_RegisterReadyTask(sensor_hub_task_handle);

    sensor_ready_mask = 0U;
    if (MTF02_Init() != 0U)
    {
        sensor_ready_mask |= SENSOR_READY_MTF02;
        app_sensor_status.mtf02_ready = 1U;
    }
    if (TFmini_Init() != 0U)
    {
        sensor_ready_mask |= SENSOR_READY_TFMINI;
        app_sensor_status.tfmini_ready = 1U;
    }
    if (SPL06_Init() != 0U)
    {
        sensor_ready_mask |= SENSOR_READY_SPL06;
        app_sensor_status.spl06_ready = 1U;
    }
    if (SBUS_Init() != 0U)
    {
        sensor_ready_mask |= SENSOR_READY_SBUS;
        app_sensor_status.sbus_ready = 1U;
    }

#if (APP_SENSOR_ENABLE_GPS == 1U)
    if (GPS_Init() != 0U)
    {
        sensor_ready_mask |= SENSOR_READY_GPS;
        app_sensor_status.gps_ready = 1U;
    }
#endif

    app_sensor_status.required_ready = ((sensor_ready_mask & required_mask) == required_mask) ? 1U : 0U;
    return app_sensor_status.required_ready;
}

void SensorHub_Task(void *pv_parameters)
{
    uint32_t notify_bits;

    (void)pv_parameters;

    for (;;)
    {
        /*
         * FreeRTOS 任务通知在这里充当轻量级事件位：
         * - 进入等待前不清位，避免覆盖刚到达的中断；
         * - 退出等待时一次清除已读取位；
         * - 最长等待 5 ms，满足 GPS 状态机建议的轮询周期。
         */
        notify_bits               = 0U;
        const BaseType_t notified = xTaskNotifyWait(0U, UINT32_MAX, &notify_bits, pdMS_TO_TICKS(5U));
        (void)notified;

        if (((sensor_ready_mask & SENSOR_READY_SBUS) != 0U) && ((notify_bits & NOTIFY_BIT_REMOTE) != 0U))
        {
            (void)SBUS_Task_Handler();
        }

        if (((sensor_ready_mask & SENSOR_READY_TFMINI) != 0U) && ((notify_bits & NOTIFY_BIT_TFMINI) != 0U))
        {
            (void)TFmini_Task_Handler();
        }

        if (((sensor_ready_mask & SENSOR_READY_MTF02) != 0U) && ((notify_bits & NOTIFY_BIT_MTF02) != 0U))
        {
            (void)MTF02_Task_Handler();
        }

        if (((sensor_ready_mask & SENSOR_READY_SPL06) != 0U) && ((notify_bits & NOTIFY_BIT_SPL06) != 0U))
        {
            (void)SPL06_Task_Handler();
        }

        /* GPS 为可选硬件；启用后按固定周期推进可能跨越多次 UART IDLE 事件的 UBX 状态机。 */
#if (APP_SENSOR_ENABLE_GPS == 1U)
        if ((sensor_ready_mask & SENSOR_READY_GPS) != 0U)
        {
            (void)GPS_Task_Handler();
        }
#endif

        /*
         * 慢工作放在所有外部传感器处理之后：
         * - 每轮最多搬运 16 个 IMU 样本；
         * - 如果已满足分析窗，每轮最多运行一个轴的 Q15 FFT；
         * - 不为 FFT 单独创建任务，也不因每个 IMU 样本额外唤醒本任务。
        */
        App_ImuFft_ProcessOneStep();
    }
}
