/**
 * @file App_Data_Comm.c
 * @brief MAVLink业务服务、USB链路和DataRouter的统一初始化与消息分发
 */

#include "App_Data_Comm.h"

#include "App_TaskPriorities.h"
#include "FreeRTOS.h"
#include "task.h"

#include "bsp_RTT.h"
#include "mavlink_user.h"
#include "modules_Message_center.h"

#include "DataRouterTask.h"

#include "system_service.h"
#include "hil_service.h"
#include "command_service.h"
#include "log_service.h"

/* ======================== Task Config (任务配置) ======================== */

/* FreeRTOS 任务栈大小分配 (单位：字/Word) */
#define APP_MAVLINK_RX_TASK_STACK  768U
#define APP_USB_TX_TASK_STACK      512U
#define APP_DATA_ROUTER_TASK_STACK 768U

/* ======================== Private Data (私有数据) ======================== */

static USBInstance *usb_instance         = NULL; /* 绑定的 USB 硬件外设实例 */
static uint8_t      app_comm_initialized = 0U;   /* 模块初始化完成标志位 */

/* ======================== Service Interface (服务接口) ==================== */

/**
 * @brief MAVLink RX 消息处理函数类型定义
 *
 * 约定每个业务 Service 都必须实现该签名：
 * void XxxService_HandleMavlinkMessage(const mavlink_message_t *msg);
 */
typedef void (*App_MavlinkHandler_t)(const mavlink_message_t *msg);

/**
 * @brief MAVLink RX 消息处理分发路由表
 *
 * 【架构设计】：
 * App_Data_Comm 作为通信枢纽，本身不关心具体的 msgid。
 * 收到完整的 MAVLink 消息后，将遍历此表，依次交给各 Service 自行判断是否需要处理。
 * 后续如需增加新的 Service (如参数服务、航点服务)，只需在此数组中添加一行即可。
 */
static const App_MavlinkHandler_t mavlink_handlers[] = {
    SystemService_HandleMavlinkMessage,
    CommandService_HandleMavlinkMessage,
    HilService_HandleMavlinkMessage,
    LogService_HandleMavlinkMessage,
};

/**
 * @brief 需要周期性执行的 Service 更新函数表
 *
 * 【架构设计】：
 * DataRouterTask 会统一按周期循环调用这些 Update 函数。
 * 具体的发送频率由各 Service 内部自己管理时间戳来决定，例如：
 * - SystemService_Update()  -> 心跳包 Heartbeat 1Hz
 * - HilService_Update()     -> HIL_ACTUATOR_CONTROLS 500Hz
 * - ParamService_Update()   -> 参数流发布
 */
static const DataRouter_UpdateCallback_t service_updates[] = {
    SystemService_Update,
    HilService_Update,
    LogService_Update,
};

/* ======================== USB RX (接收底层中断) ======================== */

/**
 * @brief USB 数据接收回调函数 (中断/底层触发)
 *
 * 职责极简：USB 层只负责把收到的原始字节流推送给 MAVLink 协议栈。
 * 不在此处进行任何 MAVLink 业务、HIL 或 Command 的处理，以保证底层收发的实时性。
 */
static void App_USBEventCallback(USBInstance *instance, USB_EVENT_e event, uint8_t *data, uint16_t size)
{
    (void)instance;

    if (event == USB_EVENT_RX_CPLT)
    {
#if (DATA_ROUTER_ENABLE_USB_IMU_STREAM == 1U)
        DataRouter_UsbCalibrationInputFromISR(data, size);
#else
        /* 将接收到的原始字节流送入 MAVLink 解析器 */
        Mavlink_InputBytesFromISR(data, size);
#endif
    }
}

/* ======================== MAVLink Dispatch (消息分发) ======================== */

/**
 * @brief 完整 MAVLink 消息的全局分发入口
 *
 * 该函数运行在 MavlinkRxTask 的任务上下文中，脱离了中断。
 * 遍历已注册的所有 Service，将完整的消息广播给它们处理。
 */
static void App_MavlinkMessageCallback(const mavlink_message_t *msg)
{
    if (msg == NULL)
    {
        return;
    }

    const uint32_t handler_count = sizeof(mavlink_handlers) / sizeof(mavlink_handlers[0]);

    for (uint32_t i = 0U; i < handler_count; i++)
    {
        if (mavlink_handlers[i] != NULL)
        {
            mavlink_handlers[i](msg);
        }
    }
}

/* ======================== Initialization (模块初始化) ======================== */

/**
 * @brief 初始化所有依赖消息中心的MAVLink业务服务
 */
static bool App_CommInitializeServices(void)
{
    if (!SystemService_Init())
    {
        RTTERROR("[APP_COMM] SystemService init failed.");
        return false;
    }

    if (!CommandService_Init())
    {
        RTTERROR("[APP_COMM] CommandService init failed.");
        return false;
    }

    if (!HilService_Init())
    {
        RTTERROR("[APP_COMM] HilService init failed.");
        return false;
    }

    if (!LogService_Init())
    {
        RTTERROR("[APP_COMM] LogService init failed.");
        return false;
    }

    return true;
}

/**
 * @brief 初始化MAVLink协议栈、USB物理链路和DataRouter
 */
static bool App_CommInitializeTransport(void)
{
    Mavlink_Init_Config_s mavlink_config = {
        .system_id        = MAVLINK_DEFAULT_SYSTEM_ID,
        .component_id     = MAVLINK_DEFAULT_COMPONENT_ID,
        .message_callback = App_MavlinkMessageCallback,
    };

    if (!Mavlink_Init(&mavlink_config))
    {
        RTTERROR("[APP_COMM] MAVLink init failed.");
        return false;
    }

    USB_Init_Config_s usb_config = {
        .event_callback = App_USBEventCallback,
    };

    usb_instance = USBRegister(&usb_config);
    if (usb_instance == NULL)
    {
        RTTERROR("[APP_COMM] USB register failed.");
        return false;
    }

    const DataRouter_Config_t router_config = {
        .usb_instance          = usb_instance,
        .update_callbacks      = service_updates,
        .update_callback_count = (uint8_t)(sizeof(service_updates) / sizeof(service_updates[0])),
    };

    if (!DataRouter_Init(&router_config))
    {
        RTTERROR("[APP_COMM] DataRouter init failed.");
        return false;
    }
    return true;
}

/**
 * @brief 按接收解析、路由调度、USB发送的顺序创建通信任务
 */
static bool App_CommCreateTasks(void)
{
    if (xTaskCreate(MavlinkRxTask, "mav_rx", APP_MAVLINK_RX_TASK_STACK, NULL, APP_TASK_PRIORITY_MAVLINK_RX, NULL) !=
        pdPASS)
    {
        RTTERROR("[APP_COMM] MAVLink RX task create failed.");
        return false;
    }

    if (xTaskCreate(DataRouterTask, "data_router", APP_DATA_ROUTER_TASK_STACK, NULL, APP_TASK_PRIORITY_DATA_ROUTER,
                    NULL) != pdPASS)
    {
        RTTERROR("[APP_COMM] DataRouter task create failed.");
        return false;
    }

    if (xTaskCreate(USBTxTask, "usb_tx", APP_USB_TX_TASK_STACK, usb_instance, APP_TASK_PRIORITY_USB_TX, NULL) != pdPASS)
    {
        RTTERROR("[APP_COMM] USB TX task create failed.");
        return false;
    }
    return true;
}

bool App_CommInit(void)
{
    if (app_comm_initialized)
    {
        return false;
    }

    if ((!App_CommInitializeServices()) || (!App_CommInitializeTransport()) || (!App_CommCreateTasks()))
    {
        return false;
    }

    app_comm_initialized = 1U;
    RTTINFO("[APP_COMM] MAVLink communication ready.");
    return true;
}

/* ======================== Public API (公共接口) ======================== */

USBInstance *App_GetUSBInstance(void)
{
    return usb_instance;
}
