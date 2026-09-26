# 代码库结构

## 1) 顶层目录图

| 路径 | 用途 | 依据 |
|---|---|---|
| `Core/` | CubeMX 生成的启动、外设配置和中断向量 | `Core/Src/main.c`、`Core/Src/stm32h7xx_it.c` |
| `Application/` | 飞行任务：传感器汇聚、姿态、指令、控制、遥测和日志 | `Application/App_SystemInit.c` |
| `Modules/` | 传感器、估计器、消息中心、电机、SD 和 MAVLink 模块 | `Modules/modules_BMI088/`、`Modules/modules_Message_center/` |
| `Bsp/` | SPI/I²C/UART/PWM/USB/时间戳/内存抽象 | `Bsp/bsp_spi/`、`Bsp/bsp_iic/` |
| `Drivers/`、`Middlewares/`、`FATFS/`、`USB_DEVICE/` | Cube/厂商生成依赖 | `CMakeLists.txt` |
| `Tests/` | 主机回归脚本与 HAL Mock BSP 检查 | `Tests/` |
| `Firmware/` | 忽略的 BIN/ELF/HEX 构建产物及受版本管理的产物清单 | `Firmware/MANIFEST.md` |

## 2) 入口点

- 主运行入口：`Core/Src/main.c`；配置 MPU、Cache 和外设后启动 FreeRTOS。
- 应用启动：`Core/Src/freertos.c:StartDefaultTask()` 在调度器启动后调用 `App_SystemInit()`，随后删除启动任务。
- 长期任务：SensorHub、Attitude、Control、Uav Cmd、DataRouter、MAVLink RX、USB TX、SD 与 Performance 任务由 `Application/` 模块创建。
- 构建选择：`CMakeLists.txt` 选择 Release、RelWithDebInfo/Ozone、HIL 等构建类型。

## 3) 模块边界

| 边界 | 应放入的内容 | 不应放入的内容 |
|---|---|---|
| `App_Sensor` | UART/I²C 传感器快照的任务级解析 | 飞行 PID 或 PWM 写入 |
| `App_attitude` | IMU 滤波、VQF 控制姿态、并行 ESKF 诊断/导航 | RC 解析或电机映射 |
| `App_Controll` | 模式状态、姿态/角速度环、混控和控制日志快照 | 传感器 DMA 解析 |
| `App_Uav_Cmd` | SBUS 到命令的归一化与安全命令转换 | 接收机 UART ISR 工作 |
| `DataRouterTask` | USB/MAVLink 外发串行化和 SD 帧生产 | 直接拥有 FatFS 文件 |
| `modules_SD_Card` | 环形缓冲消费者、FatFS 文件和日志传输请求 | 高频控制计算 |
| `Bsp` | 硬件操作及 DMA/中断所有权边界 | 飞行模式决策 |

## 4) 命名与组织规则

- 文件采用工程历史保留的 Pascal/camel 混合目录名，例如 `App_attitude`、`modules_BMI088`。
- 公共 API 通常以模块前缀命名，例如 `BMI088_GetData`、`Control_SetFeedback`、`SDCard_EnqueueFrame`。
- 私有 C 状态/函数使用 `static`；公共契约放在同名头文件。
- include 直接使用工程头文件，不配置路径别名系统。

## 5) 依据

- `Application/App_SystemInit.c`
- `Application/App_Sensor/App_Sensor.c`
- `Application/App_attitude/App_attitude.c`
- `Application/App_Controll/App_Controll.c`
