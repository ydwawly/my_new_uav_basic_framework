# 系统架构与数据流

## 分层

- `Core`：时钟、GPIO、DMA、中断、外设句柄与 FreeRTOS 启动代码。CubeMX 生成文件仅在 USER CODE 区域维护自有逻辑。
- `Bsp`：把 HAL 外设抽象为可注册、可复用的总线和设备接口，并集中处理 DMA、Cache、时间戳和回调路由。
- `Modules`：传感器协议、算法、消息中心、电机输出、SD 文件系统和 MAVLink 封装。
- `Application`：系统初始化、传感器汇聚、姿态/导航估计、控制、遥控命令和数据通信任务。

## 主飞行链路

```text
BMI088 DRDY/EXTI
  -> SPI DMA 完成
  -> 最新 IMU 快照 + 时间戳
  -> 通知 Attitude(54)
  -> LPF/Notch + VQF + ESKF 预测/观测
  -> Control_Feedback_t 快照
  -> 通知 Control(53)
  -> 姿态外环 + 角速度 PID + Mixer
  -> TIM1 PWM -> ESC
```

姿态任务是控制反馈的唯一写者，控制任务在读取完整快照时使用很短的临界区。控制任务正常路径只由姿态通知驱动；1 ms 等待超时仅检查遥控、反馈新鲜度和上锁请求，不重复运行控制律。

## 外部观测与命令

```text
UART/I2C ISR or DMA callback
  -> 原始数据槽/快照 + timestamp
  -> SensorHub(52)
  -> 协议解析与有效性检查
  -> MTF02 flow / TFmini range / SPL06 / SBUS

SBUS -> UavCmd(51) -> 消息中心 -> Control
MTF02/TFmini -> Attitude observations -> ESKF
```

中断不执行协议状态机。MTF02 使用固定静态槽池，ISR 发布完成槽，SensorHub 排空并解析，从而限制中断执行时间。

## 通信与日志

```text
USB CDC RX -> mav_rx(50) -> MAVLink services
services/telemetry -> DataRouter queue -> data_router(48)
                                  |-> USB TX ring -> usb_tx(49)
                                  `-> SD frame queue -> sd_card(46)
```

`DataRouterTask` 负责输出路由，避免高优先级飞行任务直接等待 USB 或 FATFS。SD 日志下载使用静态响应槽池和指针队列；槽的所有权在请求方与 SD 任务之间显式转移。

## 任务表

优先级数值越大越高，统一定义于 `Application/App_TaskPriorities.h`。

| 任务 | 优先级 | 栈（word） | 创建方式 | 触发方式 |
|---|---:|---:|---|---|
| bootstrap/defaultTask | 55 | 512 | CMSIS-RTOS2 | 启动一次后删除自身 |
| Attitude | 54 | 1536 | 静态 | IMU 数据就绪通知 |
| control | 53 | 512 | 静态 | 姿态通知，1 ms 安全超时 |
| sensor_hub | 52 | 512 | 静态 | 传感器通知位 |
| uav_cmd | 51 | 384 | 静态 | 遥控通知/超时 |
| mav_rx | 50 | 768 | 动态 | USB/MAVLink 接收 |
| usb_tx | 49 | 512 | 动态 | USB 发送缓冲事件 |
| data_router | 48 | 768 | 动态 | 路由队列 |
| FreeRTOS timer | 47 | 配置项 | 内核 | 软件定时器 |
| sd_card | 46 | 1024 | 静态 | 日志与下载请求 |

动态任务只在启动阶段创建。`configTOTAL_HEAP_SIZE` 为 15 KiB；姿态、控制、传感器、命令和 SD 栈使用静态分配。

## 坐标系和单位

- 机体系：FRD（前、右、下）。
- 导航系：NED（北、东、下）。高度为 `-position_ned_z`。
- 角度：弧度；角速度：rad/s；距离：m；速度：m/s；时间戳：µs。
- 四元数顺序：`[w, x, y, z]`，表示 body 到 NED 的旋转。
