/**
 * @file App_SystemInit.c
 * @brief 飞控应用初始化顺序的唯一实现
 */

#include "App_SystemInit.h"

#include "App_Control.h"
#include "App_Data_Comm.h"
#include "App_Sensor.h"
#include "App_TaskPriorities.h"
#include "App_Uav_Cmd.h"
#include "App_attitude.h"
#include "App_attitude_config.h"
#include "FreeRTOS.h"
#include "SEGGER_SYSVIEW.h"
#include "bsp_RTT.h"
#include "module_pwm_motor.h"
#include "modules_BMI088.h"
#include "modules_BMI270.h"
#include "modules_Message_center.h"
#include "modules_SD_Card.h"
#include "stm32h7xx_it.h"
#include "task.h"

/* ========================== 任务配置 ========================== */

#define CONTROL_TASK_STACK_WORDS 512U

/* ========================== 全局初始化状态 ========================== */

volatile AppSystemStatus_t app_system_status;

#if (APP_ENABLE_MOTOR_OUTPUT == 1U)
static StaticTask_t control_task_control;
static StackType_t  control_task_stack[CONTROL_TASK_STACK_WORDS];
#endif

/* ========================== 分阶段初始化 ========================== */

/**
 * @brief 按依赖顺序初始化传感器、通信、指令和控制基础服务
 */
static void App_InitializeCoreServices(void)
{
    app_system_status.sensor_hub_ready = SensorHub_Init();
    app_system_status.bmi088_ready     = (BMI088_Init() == BMI088_OK) ? 1U : 0U;
    app_system_status.bmi270_ready     = (BMI270_Init() == BMI270_OK) ? 1U : 0U;

#if (APP_ENABLE_SD_LOGGER == 1U)
    /* SD 卡挂载和文件自检由低优先级任务异步执行，不阻塞飞控主初始化。 */
    app_system_status.sd_logger_started = SDCard_Init() ? 1U : 0U;
#endif
    app_system_status.communication_ready = App_CommInit() ? 1U : 0U;
    app_system_status.command_ready       = ((Uav_Cmd_Init() != 0U) && (Uav_Cmd_StartTask() != 0U)) ? 1U : 0U;
    app_system_status.control_ready       = Control_Init();
}

/**
 * @brief 在所选数据源就绪后创建姿态估计任务
 */
static void App_InitializeAttitude(void)
{
#if (APP_ATTITUDE_SOURCE == APP_ATTITUDE_SOURCE_REAL)
    if (app_system_status.bmi088_ready != 0U)
    {
        app_system_status.attitude_ready = App_Attitude_Init() ? 1U : 0U;
    }
#else
    if (app_system_status.communication_ready != 0U)
    {
        app_system_status.attitude_ready = App_Attitude_Init() ? 1U : 0U;
    }
#endif
}

/**
 * @brief 根据编译期安全开关初始化电机与控制任务
 */
static void App_InitializeMotorOutput(void)
{
    app_system_status.motor_output_enabled = APP_ENABLE_MOTOR_OUTPUT;

#if (APP_ENABLE_MOTOR_OUTPUT == 1U)
    /*
     * 只有显式打开安全开关后才初始化 PWM 并创建闭环控制任务。
     * Motor_Init() 成功后仍保持 DISARMED，输出固定为 1000 us。
     */
    app_system_status.motor_ready = Motor_Init();
    if ((app_system_status.motor_ready != 0U) && (app_system_status.control_ready != 0U) &&
        (app_system_status.attitude_ready != 0U))
    {
        app_system_status.control_task_started = (xTaskCreateStatic(Control_Task, "control", CONTROL_TASK_STACK_WORDS,
                                                                    NULL, APP_TASK_PRIORITY_CONTROL, control_task_stack,
                                                                    &control_task_control) != NULL)
                                                     ? 1U
                                                     : 0U;
    }
#endif
}

/**
 * @brief 汇总当前编译配置要求的全部启动条件
 */
static bool App_RequiredModulesAreReady(void)
{
    bool ready = (app_system_status.sensor_hub_ready != 0U) && (app_system_status.bmi088_ready != 0U) &&
                 (app_system_status.bmi270_ready != 0U) && (app_system_status.communication_ready != 0U) &&
                 (app_system_status.command_ready != 0U) && (app_system_status.control_ready != 0U) &&
                 (app_system_status.attitude_ready != 0U);

#if (APP_ENABLE_MOTOR_OUTPUT == 1U)
    ready = ready && (app_system_status.motor_ready != 0U) && (app_system_status.control_task_started != 0U);
#endif
    return ready;
}

static void App_ReportInitializationResult(bool ready)
{
    if (ready)
    {
        RTTINFO("[APP_INIT] All required modules are ready.");
    }
    else
    {
        RTTERROR("[APP_INIT] One or more modules failed; inspect app_system_status.");
    }
}

/* ========================== 统一初始化入口 ========================== */

bool App_SystemInit(void)
{
    /*
     * 依赖顺序：
     * 1. SensorHub 和双 IMU 建立硬件数据源；
     * 2. SD、通信、指令、控制器和姿态估计依次注册消息节点；
     * 3. 可选电机输出最后启动；
     * 4. 冻结消息中心，禁止飞行期间动态改变拓扑。
     */
    app_system_status = (AppSystemStatus_t){0};

    App_InitializeCoreServices();
    App_InitializeAttitude();
    App_InitializeMotorOutput();
    Message_Center_Freeze();

    /*
     * App_SystemInit() 运行在 bootstrap 任务中，此时调度器已经启动。
     * 延后开放 ISR 跟踪可避免早期 TIM17 中断在内核启动前访问 FreeRTOS 状态。
     */
    SystemView_ISRTraceSetReady();
#if UAV_SYSTEMVIEW_AUTOSTART
    SEGGER_SYSVIEW_Start();
#endif

    const bool required_ready            = App_RequiredModulesAreReady();
    app_system_status.all_required_ready = required_ready ? 1U : 0U;
    App_ReportInitializationResult(required_ready);
    return required_ready;
}
