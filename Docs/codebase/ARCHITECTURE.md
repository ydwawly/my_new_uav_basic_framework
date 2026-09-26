# 架构与数据流

需要按函数逐步查看 ISR、DMA、解析、拷贝、任务通信和阻塞行为时，参见 [`DATA_FLOW_FUNCTIONS.md`](DATA_FLOW_FUNCTIONS.md)。

## 1) 架构风格

- 主体是事件驱动的 FreeRTOS 固件，控制链路以固定 1 kHz 周期运行。
- 外设 ISR/DMA 回调负责接收、快照或通知；SensorHub 与 Attitude 任务负责解析和估计；Control 独占模式/PID/混控；DataRouter 与 SD 负责外发和日志。
- 主要约束是 STM32H7 的 Cache/DMA 内存域、ISR 时长必须有界、FatFS 单任务所有权，以及 1 ms 控制周期。

## 2) 系统总体数据流

```text
BMI088 DRDY → SPI DMA 双缓冲 → 标定/坐标映射
  → 自适应陷波（若启用）→ PT1 低通 → VQF 6D → 控制姿态
                                      → 并行 ESKF → 位置/速度/导航标志

SBUS UART DMA → SensorHub → sbus_data topic → Uav Cmd → uav_cmd topic

控制姿态 + 滤波角速度 + uav_cmd → 四元数角度环 → 角速度 PID
  → X 型混控 → 四路归一化电机命令 → PWM ESC 脉宽

TFmini UART → SensorHub → TFmini topic → ESKF 测距更新 → 定高资格
MTF02 UART → SensorHub → MTF02 topic → ESKF 水平光流速度更新
SPL06 I²C/DMA → SensorHub → SPL06 topic → 当前实机仅记录/诊断

控制/估计器快照 → DataRouter 定期日志服务 → SD 环形缓冲 → SD 任务/FatFS
USB RX → MAVLink RX 任务 → 各服务；服务输出 → DataRouter → USB TX 与 MAVLink 日志帧
```

### 实机 IMU 与姿态链路

1. BMI088 陀螺 DRDY 启动 SPI DMA；陀螺 DMA 完成后立即启动加速度计 DMA；最终回调唤醒 Attitude 任务。驱动读取已完成的双缓冲半区，把原始计数转换为 SI 单位、映射到工程 IMU 坐标系，并应用固定标定与可选温度补偿。
2. `App_Attitude_Task` 根据 IMU 时间戳计算实际 `dt_s`，拒绝无效或非有限样本。随后依次执行三轴陷波与 PT1 低通；当前配置为陀螺 80 Hz、加速度 30 Hz。
3. 滤波后的陀螺/加速度输入 `VqfC_Update()`；其 6D 四元数 `q_nb` 与欧拉角是飞控唯一使用的姿态来源。
4. 同一个滤波 IMU 样本也输入并行 `NAV_ESKF_Predict()` 与重力更新。ESKF 不提供姿态控制，而是提供导航状态、陀螺/加速度零偏诊断，以及定高使用的 Z 状态。
5. 姿态任务构造 `Control_Feedback_t`，通过短临界区保护的快照交给 `Control_SetFeedback()`。

### 外部观测链路

| 原始来源 | 任务级产物 | 当前实机构建中的 ESKF 用法 | 控制用途 |
|---|---|---|---|
| TFmini Plus UART | `TFminiPlus_Data_t` | 新鲜且有效时作为测距/高度观测 | 与有效 ESKF 预测共同决定是否允许定高 |
| MTF02 UART | `MTF02_Data_t` | 经过质量、时间和高度门限后作为水平光流速度观测 | 不直接参与姿态或电机控制 |
| MTF02 自带测距 | 仅记录诊断 | 不作为当前实机测距融合源 | 无 |
| SPL06 I²C/DMA | `SPL06_Data_t` | 当前实机没有 ESKF 融合路径 | 无 |
| HIL MAVLink | `HIL_Sensor_Data_t` | 仅 HIL 构建替代实机传感器来源 | HIL ESKF 控制反馈 |

### 指令与控制链路

1. SBUS ISR 在序列锁下复制一个 25 字节帧并通知 SensorHub；SensorHub 校验后发布 `sbus_data`。
2. `Uav_Cmd_Task` 归一化摇杆：Roll/Pitch 为角度参考（Pitch 在此唯一边界反向），Yaw 为角速度参考，油门为 0…1；物理 CH6（数组 `channels[5]`）高于 1025 请求定高。丢帧/failsafe 会发布立即解锁关闭、油门为零的命令。
3. Control 每 1 ms 运行一次，要求命令和反馈均新鲜。闭环时它将 Yaw 角速度命令积分为 Yaw 角目标，构造 Roll/Pitch/Yaw 目标四元数，计算旋转向量姿态误差，滤波后变成受限角速度参考。
4. 三轴角速度 PID 将参考与滤波角速度测量比较；X 型混控将 Roll/Pitch/Yaw 项叠加到公共油门，归一化四路输出并调用 `Motor_SetOutput()`。
5. 电机模块检查输出是否有限、限幅、转换为配置好的 PWM 脉宽、写入四个 PWM 通道，并记录最后输出时间戳。

### 定高链路

定高是串级控制而不是只有速度环：高度目标与 ESKF NED-Z 的误差产生垂直速度参考；垂直速度 PID 产生油门修正；修正经斜坡限制后叠加到 `altitude_base_throttle`。只有并行 ESKF 预测有效且近期成功融合 TFmini 测距时才允许定高。VQF 仍然提供该模式下的 Roll/Pitch/Yaw 控制姿态。

### 日志与通信链路

- `Control_UpdateLogSnapshot()` 采集命令、反馈、目标/误差四元数、角速度 PID 分量、电机命令和定高状态。
- `LogService_Update()` 在 DataRouter 上下文运行：以 200 Hz 记录飞控数据，按配置周期记录估计器对比，并以 10 Hz 记录陷波诊断。
- `SDCard_EnqueueFrame()` 构造帧头/payload/CRC 后推入 128 KiB SPSC 环形缓冲；只有低优先级 SD 任务拥有 FatFS 写入与同步。
- USB 在 ISR 接收字节，`MavlinkRxTask` 在任务上下文解析 MAVLink；服务响应复制到 DataRouter 槽。DataRouter 是唯一 USB TX 所有者，并记录发出的 MAVLink 帧。

## 3) 数据搬运与并发审计

本节只描述“字节/结构体如何流动”，不展开飞控算法。文中的“拷贝”指 CPU `memcpy` 或 FreeRTOS 队列的 item copy；DMA 外设写内存会单独标注。

### 公共传输原语

| 原语 | 所有权与同步 | 拷贝与阻塞语义 | 结果 |
|---|---|---|---|
| `RingBuffer_t` | 仅限单生产者单消费者（SPSC）。生产者只写 `head`，消费者只写 `tail`；二者为 `volatile`，并通过 `__DMB()` 发布。 | `Push`/`Pop` 会复制字节，绝不阻塞；空间不足时整段输入直接拒绝。 | 只有严格一个生产者和一个消费者时才正确。 |
| Message Center topic | 每个 topic 一块静态 payload、一个序列锁、每订阅者一个读取游标；没有每订阅者队列。 | 发布者把调用者结构体复制到 topic 一次；读取者把最新一致结构体复制到本地一次，最多尝试 3 次。阻塞读取者睡眠在任务通知上，但发布者从不等待。 | 最新值传输：中间样本可能被覆盖，通知计数会合并，慢消费者不会给生产者施加背压。 |
| 驱动原始快照 | 写者为 ISR/DMA 完成回调，读者为对应延后任务；短写入和读重试由 `SeqLock_t` 保护。 | 通常 ISR 从 DMA 缓冲复制一次到驱动私有原始帧，任务再复制一次到栈局部帧。 | ISR 不使用互斥锁、不等待；持续竞争时最多重试后放弃本次快照。 |
| 临界区快照 | 用 `taskENTER_CRITICAL()` / `taskEXIT_CRITICAL()` 包围整结构体复制。 | 写者一次复制、读者一次复制；没有等待互斥锁。中断屏蔽时间与结构体大小成正比。 | 用于“只需最新一致状态”的控制/诊断快照，而不是连续帧流。 |
| FreeRTOS 队列 | 队列复制 item，或复制指向预分配槽的指针。 | 实时生产路径统一零等待；满队列时丢弃。DataRouter 归还槽是唯一刻意使用 `portMAX_DELAY` 的队列发送。 | 有界的所有权转移，区别于 Message Center 的最新值模型。 |

### DMA 内存与 Cache 所有权

| 缓冲区类别 | 位置与 Cache 规则 | 生产者 → 消费者 | 安全机制 |
|---|---|---|---|
| SPI/I²C/UART DMA 缓冲（`DMA_BUFFER`、`BSP_DMA_Malloc`） | `.dma_buffer`，D2 SRAM `0x30000000`，MPU 配为非缓存并 32 字节对齐。 | DMA 外设 → ISR/回调或任务。 | 不需要对这些缓冲做 D-Cache 维护。 |
| BMI088 RX 缓冲 | 陀螺与加速度各有两个 RX 半缓冲。 | SPI DMA → BMI088 回调 → Attitude 任务。 | 乒乓索引选择已完成半区；下一次 DMA 写另一个半区。 |
| SDMMC 传输缓冲 | D1 AXI SRAM（`SDMMC_DMA_BUFFER`），可缓存。 | FatFS/SDMMC DMA ↔ SD 任务。 | `sd_diskio.c` 在写前 clean、读后 invalidate；未对齐扇区路径还会经过 512 字节 scratch 缓冲复制。 |
| CPU 日志环形缓冲 | 普通 SRAM，不属于 DMA 所有权。 | DataRouter → SD 任务；DataRouter → USB TX 任务。 | SPSC 环形缓冲用 `__DMB()` 发布。 |

### 各链路传输追踪

| 链路 | 中断/DMA 动作 | 延后解析与拷贝 | 任务间交接 | 是否阻塞 / 丢失策略 |
|---|---|---|---|---|
| **BMI088 → Attitude → Control** | 陀螺 DRDY EXTI 记录时间戳并启动 SPI DMA 到 `gyro_rx_buf[write]`；不拷贝 payload，也不调用 FreeRTOS。陀螺完成后切换索引并启动加速度 DMA；加速度完成后切换索引、置位并 `vTaskNotifyGiveFromISR()` 唤醒 Attitude。 | `BMI088_GetData()` 直接从 `index ^ 1` 的已完成 DMA 半区解析寄存器字节；随后复制 `BMI088_Data_t` 到任务局部变量，再各复制一次三轴陀螺/加速度到姿态样本。陷波、低通、VQF、并行 ESKF 全在 Attitude 任务中执行。 | Attitude 在短临界区内复制一个 `Control_Feedback_t`；Control 每 1 ms 在另一个短临界区复制该快照。这里不是 Message Center topic。 | Attitude 无限阻塞等待 IMU 通知。若运行前累积多次通知，计数会记录漏样数量，但只使用最新双缓冲内容。SPI 链忙时的新 DRDY 被显式计数并丢弃。 |
| **SBUS UART → SensorHub → UavCmd → Control** | UART DMA 写 BSP 持有的 D2 缓冲。IDLE/TC 时 UART 回调在序列锁下复制 25 字节到 `raw_rx_buf`、记录时间戳、置一个 SensorHub 通知位，然后重启 UART DMA。 | SensorHub 最多重试 3 次把原始帧复制到栈，校验并解析 16 路通道，再发布 `SBUS_Data_t`。`Uav_Cmd_Task` 读取最新 SBUS 结构体、归一化/解释开关，再发布 `Uav_Cmd_t`。 | SBUS 发布复制一次进 topic，UavCmd 读取再复制一次。UavCmd 发布复制一次进 topic；Control 每 1 ms 非阻塞复制最新命令。 | SensorHub 位通知会合并，只保存最新原始帧。UavCmd 可在 `SubGetMessage` 最多阻塞 10 ms；Control 从不为命令阻塞。坏帧与被覆盖的中间 RC 帧会丢弃，这符合最新摇杆语义。 |
| **TFmini UART → SensorHub → ESKF** | UART DMA 写 D2 缓冲。ISR 在序列锁下复制至多 9 字节原始帧、记录时间戳、置 TFmini SensorHub 位。 | SensorHub 复制原始快照，校验 checksum 并解析为 `TFminiPlus_Data_t`。 | 一次 Message Center 发布拷贝；Attitude 每个 IMU 周期非阻塞读取最新值，并且每个新且有效时间戳只融合一次。 | 生产者不阻塞。序列锁冲突或后续 UART 帧会替换旧帧；checksum 错误帧丢弃。 |
| **SPL06 I²C DMA → SensorHub** | DRDY EXTI 启动 11 字节 I²C DMA 到 D2 非缓存内存。DMA 完成把 11 字节复制一次到受序列锁保护的 `raw_frame`，再置 SensorHub 位。 | SensorHub 复制 `raw_frame` 到栈，在任务中做压力/温度补偿并发布 `SPL06_Data_t`。 | Message Center 最新值 topic；当前实机姿态路径不订阅它做 ESKF 融合。 | I²C 传输进行时来的 DRDY 会计数并丢弃。启动后 ISR 不执行浮点运算或阻塞 I²C 调用。 |
| **MTF02 UART → SensorHub → ESKF** | UART DMA 写 D2 内存，**随后 UART 回调当前会逐字节执行 MicoLink 与 MSPv2 两个解析器**。完整合法帧写入受序列锁保护的 `latest_data`，然后才置 SensorHub 位；没有原始帧拷贝。 | SensorHub 复制已完成的 `MTF02_Data_t` 快照；若为 MSP 流数据，在任务中计算光流角速度并发布。 | 一次 Message Center 发布拷贝；Attitude 非阻塞读取最新值，并可融合新鲜且通过门限的水平光流观测。 | 没有队列、生产者不阻塞；SensorHub 未运行前的多个合法帧合并为最新状态。**这是异类：协议状态机/校验在 ISR，而非其他传感器的“ISR 快照、任务解析”。** |
| **USB RX → MAVLink 服务** | USB CDC RX hook 在中断中调用 `Mavlink_InputBytesFromISR()`；后者把字节块复制到 4096 字节 SPSC RX 环形缓冲并通知 `MavlinkRxTask`。 | `MavlinkRxTask` 从环形缓冲复制块到栈缓冲，在任务上下文解析 MAVLink 并分发完整消息；每次唤醒有处理量上限，剩余数据会自通知。 | ISR 到任务使用计数通知和 SPSC 字节环，不使用 Message Center。 | ISR 永不阻塞；RX 环无空间时整块输入计入丢弃。解析任务空闲时无限阻塞。 |
| **服务/MAVLink → DataRouter → USB TX** | 正常服务路径没有 ISR 生产者。`DataRouter_PostFrame()` 零等待取预分配空闲槽，复制完整序列化包进槽，再把槽指针入队。 | DataRouter 独占 USB 提交。`USBSend()` 把包复制到 32 KiB D2 SPSC TX 环；`USBTxTask` 至多复制 512 字节到稳定的 pending DMA 缓冲并启动 CDC 发送。TX 完成 ISR 只清 pending 状态并通知 TX 任务。 | FreeRTOS 队列只传槽指针；USB 使用 SPSC 字节环。仅 DataRouter 可以调用物理 USB 发送。 | 槽池或 USB 环满立即丢新数据。USB TX 任务空闲/忙时最多等待 20 ms；不会阻塞数据生产者。USB 断开会主动丢弃已排队旧数据。 |
| **Control/日志快照 → DataRouter → SD** | 没有日志 ISR。Control 在自身任务更新日志快照；`LogService_Update()` 在 DataRouter 上下文运行。 | 日志服务复制 Control 的临界区快照，在栈上构建完整 `帧头 + payload + CRC`；`SDCard_EnqueueFrame()` 再把整帧复制到 128 KiB SPSC SD 环。SD 任务把批次复制到 `sd_write_buffer`，然后调用 FatFS。 | DataRouter 是唯一生产者，低优先级 SD 任务是唯一消费者；环形缓冲和 FatFS 无需互斥锁，因为只有 SD 任务拥有文件。 | 入队从不阻塞；满环丢弃新帧。`f_write`、`f_sync` 与 SD DMA 完成等待可能使**低优先级 SD 任务**阻塞到 SD 超时（30 s）。不会阻塞 Control/Attitude，但会令环形缓冲填满并丢日志。 |

### ISR 优先级与 FreeRTOS API 合法性

`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` 为 5。在当前 Cortex-M 配置中，调用 `...FromISR()` 的 ISR 数值优先级必须 **大于或等于 5**。

| IRQ 链路 | 优先级 | 该路径是否调用 FreeRTOS API | 结论 |
|---|---:|---|---|
| BMI088 DRDY EXTI15_10 | 1 | 否，仅启动 DMA | 合法：刻意高于 RTOS syscall 阈值。 |
| BMI088 SPI DMA | 5 | 完成路径调用 `vTaskNotifyGiveFromISR()` | 在配置阈值上，合法。 |
| SBUS/TFmini 的 UART6/USART1 及 DMA | 6 | `xTaskNotifyFromISR()` | 合法。 |
| MTF02 UART4 | 7 | 解析后 `xTaskNotifyFromISR()` | 合法，但 ISR 工作量偏大。 |
| SPL06 EXTI/DMA | 7 | 完成时 `xTaskNotifyFromISR()` | 合法。 |
| USB OTG FS 回调 | 7 | MAVLink/USB 路径调用 `vTaskNotifyGiveFromISR()` | 合法；CubeMX 重新生成 USB 配置时必须保持数值优先级不小于 5。 |

### 本工程中“无锁”的准确含义

本工程有三类不同机制，不能混为一谈：

1. **无锁 SPSC 字节环**：生产者/消费者分别拥有不同游标；实际仍有字节复制，但没有互斥锁或等待。
2. **序列锁快照**：读者可能重试，写者永不等待；这是最新状态传递，不是无损队列。
3. **临界区快照**：严格说不属于无锁，因为短时间屏蔽中断；它用于获得紧凑且一致的控制/诊断状态。

因此系统的总体选择是“有界延迟与最新数据优先”，而不是保存每一个传感器或遥测样本。BMI088 双缓冲链路是例外：任务漏唤醒会计数，随后处理最新完整样本。

## 4) 层与模块职责

| 层或模块 | 负责什么 | 不应负责什么 | 依据 |
|---|---|---|---|
| BMI088 模块 | DRDY/DMA 状态、原始数据换算和标定 | PID 或飞行模式 | `Modules/modules_BMI088/modules_BMI088.c` |
| Attitude 任务 | 滤波状态、VQF、并行 ESKF、控制反馈快照 | PWM/RC 处理 | `Application/App_attitude/App_attitude.c` |
| 观测适配器 | TFmini/MTF02 订阅及量测门限 | VQF 输出 | `Application/App_attitude/App_attitude_observations.c` |
| Control 任务 | 命令/反馈新鲜度、PID、混控、定高模式 | DMA 解析/FatFS | `Application/App_Controll/App_Controll.c` |
| Motor 模块 | 解锁状态与 PWM 转换 | 估计器或模式状态机策略 | `Modules/modules_Motor/module_pwm_motor/module_pwm_motor.c` |
| DataRouter/SD | 外部输出串行化和持久日志 | 直接飞控决策 | `Application/DataRouterTask/DataRouterTask.c`、`Modules/modules_SD_Card/modules_SD_Card.c` |

## 5) 复用模式

| 模式 | 位置 | 原因 |
|---|---|---|
| 发布/订阅 topic | SBUS/Uav Cmd，经 Message Center | 在任务上下文间复制最新结构化命令 |
| ISR 通知 + 任务处理 | BMI088、SBUS、TFmini、SPL06 | 缩短 ISR 且使执行时间有界 |
| 双缓冲 | BMI088 SPI RX | 防止任务解析仍归 DMA 所有的缓冲区 |
| 序列锁快照 | SBUS 原始帧、估计器对比快照 | 不用阻塞互斥锁也能读取一致多字段状态 |
| SPSC 环/队列 | SD 与 DataRouter 链路 | 从高频生产者向消费者做有界且不阻塞的交接 |

## 6) 已知架构风险

- 当前构建中 `Motor_SafetyCheck()` 无调用者，因此输出超时策略没有生效；Control 任务停滞时，最后写入的 PWM 比较值可能继续保持。
- VQF 控制姿态而 ESKF 提供定高/导航，这一分工是刻意设计；日志分析必须区分 `control_*` 姿态字段与 ESKF 状态字段。
- 实机导航没有融合气压计，且只有 TFmini 是有效高度观测，因此其有效性门限决定定高是否可用。

## 7) 依据

- `Core/Src/freertos.c`
- `Application/App_SystemInit.c`
- `Application/App_attitude/App_attitude.c`
- `Application/App_attitude/App_attitude_observations.c`
- `Application/App_Controll/App_Controll.c`
- `Application/App_Data_Comm/log_service.c`
