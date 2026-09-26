# 函数级数据流、拷贝与阻塞分析

本文从数据入口开始，逐函数追踪到最终使用位置。每一步说明：运行上下文、处理内容、是否复制、同步方式和是否可能阻塞。

## 1) 先区分四种执行上下文

| 上下文 | 典型函数 | 允许做的工作 | 当前工程中的同步方式 |
|---|---|---|---|
| EXTI ISR | `HAL_GPIO_EXTI_Callback()` | 记录时间戳、检查状态、启动 DMA | 状态变量、DMA 所有权 |
| DMA/UART/USB 完成 ISR | `BMI088_Accel_DMA_Callback()`、`SBUS_UART_EventCallback()` | 结束所有权、做小快照、通知任务 | `...FromISR()`、序列锁、双缓冲 |
| 事件任务 | `App_Attitude_Task()`、`SensorHub_Task()`、`MavlinkRxTask()` | 解析、滤波、估计和协议处理 | 任务通知、Message Center、SPSC 环 |
| 周期任务 | `Control_Task()`、`DataRouterTask()` | 控制计算、通信和日志调度 | `vTaskDelayUntil()`、有超时队列等待 |

本文把结构体赋值（例如 `dst = src`）也算作一次逻辑拷贝；编译器最终可能展开为多条 load/store，而不一定调用 `memcpy()`。

## 2) BMI088 原始数据到 PWM 电机输出

### 2.1 完整函数调用链

```text
EXTI15_10_IRQHandler()
  → HAL_GPIO_EXTI_IRQHandler(GYRO_INT_Pin)
  → HAL_GPIO_EXTI_Callback()
  → BMI088_GYRO_DRDY_Handler()
  → SPITransRecv(... gyro_rx_buf[write_idx] ...)
  → HAL SPI/DMA
  → SPI_CompleteRouting()
  → BMI088_Gyro_DMA_Callback()
  → SPITransRecv(... accel_rx_buf[write_idx] ...)
  → HAL SPI/DMA
  → SPI_CompleteRouting()
  → BMI088_Accel_DMA_Callback()
  → vTaskNotifyGiveFromISR(attitude_task)
  → Attitude_WaitForImuSample()
  → BMI088_GetData()
  → Attitude_FilterImuSample()
  → Attitude_PreparePrediction()
  → Attitude_RunPrediction()
       → VqfC_Update()
       → NAV_ESKF_Predict()
       → NAV_ESKF_UpdateGravity()
  → App_Attitude_Observations_Update()
  → Attitude_PublishEstimate()
  → Control_SetFeedback()
  → Control_Task()
  → Control_GetRateReference()
  → PIDCalculate() × 3
  → Control_Mixer()
  → Motor_SetOutput()
  → PWMSetPulseWidthUs() × 4
```

### 2.2 中断和 DMA 阶段

| 顺序 | 函数与位置 | 数据处理 | 是否复制 | 阻塞/同步 |
|---:|---|---|---|---|
| 1 | `EXTI15_10_IRQHandler()`，`Core/Src/stm32h7xx_it.c:479` | HAL 分发 BMI088 加速度/陀螺 GPIO 中断 | 否 | ISR，不阻塞 |
| 2 | `HAL_GPIO_EXTI_Callback()`，`Core/Src/stm32h7xx_it.c:716` | 根据 GPIO pin 调用 BMI088 DRDY 处理器 | 否 | ISR，不阻塞 |
| 3 | `BMI088_GYRO_DRDY_Handler()`，`Modules/modules_BMI088/modules_BMI088.c:657` | 记录采样时间；若 SPI 链忙则 `dma_busy_drop_count++`；否则选择 `gyro_buf_idx` 并启动 DMA | 否，DMA 将稍后直接写 RX 半缓冲 | ISR；`SPITransRecv()` 为 DMA 启动，立即返回 |
| 4 | `SPITransRecv()` / `SPI_RunTransfer()`，`Bsp/bsp_spi/bsp_spi.c` | 声明 SPI 总线 owner、拉低 CS、调用 `HAL_SPI_TransmitReceive_DMA()` | 否 | 不等待 DMA 完成；总线已占用则返回 `HAL_BUSY` |
| 5 | `BMI088_Gyro_DMA_Callback()`，`modules_BMI088.c:310` | `gyro_buf_idx ^= 1`，置陀螺更新标志，并级联启动加速度 DMA | 否 | SPI DMA ISR；失败恢复 `DMA_IDLE` |
| 6 | `BMI088_Accel_DMA_Callback()`，`modules_BMI088.c:352` | `accel_buf_idx ^= 1`，置加速度更新标志，恢复 `DMA_IDLE` 并通知 Attitude | 否 | `vTaskNotifyGiveFromISR()`；DMA IRQ 优先级 5，符合 FreeRTOS 限制 |

双缓冲的核心不是把一份数据再复制一遍，而是交替改变 DMA 写入地址：

```c
/* 回调完成后，buf_idx 已指向下一次要写的半区。 */
bmi088.gyro_buf_idx ^= 1U;

/* 任务读取刚完成、当前不属于 DMA 的半区。 */
uint8_t read_idx = bmi088.gyro_buf_idx ^ 1U;
BMI088_ParseGyro(bmi088.gyro_rx_buf[read_idx]);
```

### 2.3 Attitude 任务中的数据处理

| 顺序 | 函数与位置 | 输入 → 输出 | 是否复制 | 阻塞/同步 |
|---:|---|---|---|---|
| 7 | `Attitude_WaitForImuSample()`，`App_attitude.c:563` | 等待 DMA 完成通知；调用 `BMI088_GetData(&imu)` | 是：驱动内部状态 → 局部 `BMI088_Data_t imu` | `ulTaskNotifyTake(pdTRUE, portMAX_DELAY)`，无数据时无限阻塞且不占 CPU |
| 8 | `BMI088_GetData()`，`modules_BMI088.c:786` | 直接解析已完成 DMA 半区；原始计数 → 坐标变换 → 标定后的 `gyro[]/accel[]` | 不复制 DMA 原始帧；最后 `memcpy()` 一次完整 `BMI088_Data_t` | 不阻塞；只处理最新完成半区 |
| 9 | `Attitude_WaitForImuSample()` | `imu.gyro/accel` → `AppAttitudeImuSample_t sample` | 两次 3-float `memcpy()` | Attitude 单任务所有权 |
| 10 | `Attitude_FilterImuSample()`，`App_attitude.c:618` | 根据时间戳计算 `dt`；三轴先自适应陷波，再 PT1 低通 | 原地修改 `sample`，无跨缓冲复制 | 不阻塞；滤波状态只属于 Attitude 任务 |
| 11 | `Attitude_PreparePrediction()`，`App_attitude.c:672` | 检查时间戳、有限值、初始化状态及 `dt` 范围 | 否 | 不阻塞；无效样本立即拒绝 |
| 12 | `Attitude_RunPrediction()`，`App_attitude.c:709` | 同一份滤波 IMU 分别送入 VQF 和并行 ESKF | 构造一次局部 `NavImuSample`，是小结构体复制 | 不阻塞；VQF/ESKF 状态均由 Attitude 单任务拥有 |

`Attitude_RunPrediction()` 的关键代码关系是：

```c
VqfC_Update(sample->gyro_rps, sample->accel_mps2,
            &attitude_runtime.vqf_output);

const NavImuSample prediction = {
    .gyro_rps   = {sample->gyro_rps[0], sample->gyro_rps[1], sample->gyro_rps[2]},
    .accel_mps2 = {sample->accel_mps2[0], sample->accel_mps2[1], sample->accel_mps2[2]},
    .dt_s       = dt_s,
};
NAV_ESKF_Predict(&nav_eskf_instance, &prediction);
NAV_ESKF_UpdateGravity(&nav_eskf_instance, &prediction);
```

因此当前事实是：**陷波和低通后的同一份数据先进入 VQF，也进入并行 ESKF；原始 BMI088 数据不会直接进入控制 PID。**

### 2.4 姿态/导航结果到 Control

| 顺序 | 函数与位置 | 数据处理 | 是否复制 | 阻塞/同步 |
|---:|---|---|---|---|
| 13 | `Attitude_PublishEstimate()`，`App_attitude.c:444` | 汇总 VQF 控制姿态、滤波 IMU、ESKF 位置/速度/零偏、观测与陷波诊断 | 多个数组复制/字段赋值；构造局部 `Control_Feedback_t` | Attitude 任务，不阻塞 |
| 14 | `Control_SetFeedback()`，`App_Controll.c:173` | 将完整反馈写入 `control_instance.feedback` | 一次完整结构体复制 | 短临界区；不是无锁，也不是 Message Center |
| 15 | `Control_FeedbackIsReady()`，`App_Controll.c:305` | 每个控制周期复制一致反馈到局部变量，并检查接收时间 | 一次完整结构体复制 | 短临界区；不等待数据 |

关键快照代码：

```c
taskENTER_CRITICAL();
control_instance.feedback = *feedback;
control_instance.feedback.received_timestamp_us = received_timestamp_us;
control_instance.feedback_valid = 1U;
taskEXIT_CRITICAL();
```

这条链不是严格无锁：它通过短时间屏蔽可调用 RTOS 的中断和任务切换来保证整结构体一致。

### 2.5 Control 到 PWM

| 顺序 | 函数与位置 | 数据处理 | 是否复制 | 阻塞/同步 |
|---:|---|---|---|---|
| 16 | `Control_Task()`，`App_Controll.c:860` | 每 1 ms 读取最新指令/反馈、检查解锁和导航状态、运行控制状态机 | 反馈复制见上；命令通过 Message Center 复制 | `vTaskDelayUntil()` 到绝对周期；计算阶段不阻塞 |
| 17 | `Control_GetRateReference()`，`App_Controll.c:644` | 遥控角度目标 + Yaw 积分目标 → 目标四元数 → 旋转向量角误差 → PT1 → 三轴角速度参考 | 局部四元数/数组赋值 | Control 单任务所有权 |
| 18 | `PIDCalculate()`，由 `Control_RunRateLoop()` 调用 | `rate_ref_rps[] - feedback->gyro_rps[]` → P/I/D → 三轴控制量 | 无跨任务复制 | Control 单任务 |
| 19 | `Control_Mixer()`，`App_Controll.c:205` | 油门和三轴控制量 → 四路 X 型电机归一化输出 | 写局部 `motor_output[4]` | 同步函数，不阻塞 |
| 20 | `Motor_SetOutput()`，`module_pwm_motor.c:401` | 检查有限值、限幅、换算四路 PWM 脉宽、写 CCR | 复制四路限幅输出和脉宽到电机状态 | `PWMSetPulseWidthUs()` 同步寄存器写，不等待电机或 ESC |

## 3) SBUS 遥控器数据到控制任务

### 3.1 完整函数调用链

```text
USART6 / DMA RX / IDLE
  → HAL_UARTEx_RxEventCallback()
  → USARTNormalRxHandler()
  → SBUS_UART_EventCallback()
  → xTaskNotifyFromISR(sensor_hub, NOTIFY_BIT_REMOTE, eSetBits)
  → SensorHub_Task()
  → SBUS_Task_Handler()
  → SBUS_ParseRawFrame()
  → PubPushMessage("sbus_data")
  → SubGetMessage() in Uav_Cmd_Task()
  → Uav_Cmd_ParseSBUS()
  → PubPushMessage("uav_cmd")
  → SubGetMessage() in Control_CommandIsReady()
```

### 3.2 每一步与复制次数

| 步骤 | 函数与位置 | 操作 | CPU 复制 | 是否阻塞 |
|---:|---|---|---:|---|
| 1 | `HAL_UARTEx_RxEventCallback()`，`Bsp/bsp_uart/bsp_uart.c:377` | 根据 UART 实例和 Normal/Circular 模式分发 | 0 | ISR，不阻塞 |
| 2 | `USARTNormalRxHandler()`，`bsp_uart.c:312` | 把 BSP DMA 缓冲指针直接传给设备回调，回调返回后重新启动 DMA | 0 | ISR，不阻塞 |
| 3 | `SBUS_UART_EventCallback()`，`mudules_Sbus.c:72` | DMA 缓冲 → `sbus_instance.raw_rx_buf[25]`，写时间戳和计数 | 1 | ISR；序列锁写入，不等待读者 |
| 4 | `SensorHub_Task()`，`App_Sensor.c:104` | 等待并读取通知位，调用 SBUS handler | 0 | 最多阻塞 5 ms；同一传感器多次通知合并成一位 |
| 5 | `SBUS_Task_Handler()`，`mudules_Sbus.c:116` | 序列锁下 raw → `local_buf[25]`，校验帧头/尾，解析 16 路通道 | 1 | 最多重试 3 次，不等待 ISR |
| 6 | `PubPushMessage()`，`modules_Message_center.c:356` | `SBUS_Data_t` → `sbus_data` topic 唯一 payload | 1 | 发布者不阻塞 |
| 7 | `SubGetMessage()` / `TryRead()`，`modules_Message_center.c:403/165` | topic payload → `Uav_Cmd_Task` 局部 `sbus_data` | 1 | UavCmd 最多等 10 ms |
| 8 | `Uav_Cmd_ParseSBUS()`，`App_Uav_Cmd.c:56` | 通道值 → Roll/Pitch 角度、Yaw 角速度、油门、解锁/定高/failsafe | 新建并写一个 `Uav_Cmd_t` | 不阻塞 |
| 9 | `PubPushMessage()` | `Uav_Cmd_t` → `uav_cmd` topic | 1 | 不阻塞 |
| 10 | `Control_CommandIsReady()`，`App_Controll.c:277` | topic → `control_instance.uav_cmd` 并检查超时/failsafe | 1 | `xWait=0`，不阻塞；没有新帧时继续用旧值，但必须通过时间检查 |

从 UART DMA 结束到 Control 获得命令，共有 **6 次明确跨缓冲复制**：ISR raw、SensorHub local、SBUS topic 写、UavCmd topic 读、命令 topic 写、Control topic 读；另有一次把通道字段转换为新 `Uav_Cmd_t`。这样做的目标是保证各任务不共享可变帧缓冲。

## 4) TFmini 测距到 ESKF

```text
USART1 DMA/IDLE
  → HAL_UARTEx_RxEventCallback()
  → USARTNormalRxHandler()
  → TFmini_UART_EventCallback()
  → SensorHub_Task()
  → TFmini_Task_Handler()
  → TFmini_ParseRawFrame()
  → PubPushMessage("tfmini_data")
  → Attitude_UpdateTfminiObservation()
  → NAV_ESKF_UpdateRange()
```

| 函数 | 处理内容 | 复制/同步/阻塞 |
|---|---|---|
| `TFmini_UART_EventCallback()`，`mudules_TFmini_Plus.c:236` | ISR 将 DMA 数据复制到 9 字节 `raw_rx_buf`，保存接收时间戳并置通知位 | 1 次 raw 拷贝；seqlock；ISR 不阻塞 |
| `TFmini_Task_Handler()`，`mudules_TFmini_Plus.c:290` | raw 快照复制到局部数组；检查长度、`0x59 0x59` 帧头与 checksum；解析距离、强度和温度 | 1 次 raw 拷贝；最多重试 3 次 |
| `PubPushMessage()` | 发布 `TFminiPlus_Data_t` | 1 次结构体拷贝；不阻塞 |
| `Attitude_UpdateTfminiObservation()`，`App_attitude_observations.c:174` | 非阻塞读取最新测距；拒绝重复时间戳/无效值；米制换算并构造 `NavRangeObservation` | 1 次 topic 读取拷贝；`xWait=0` |
| `NAV_ESKF_UpdateRange()` | 执行测距创新、NIS/门限和状态更新；成功后更新 `last_range_fusion_timestamp_us` | 原地修改 Attitude 独占的 ESKF；不跨任务 |

明确复制数：**DMA 后 4 次**（ISR raw、任务 local、topic publish、Attitude subscribe）。

## 5) MTF02 光流到 ESKF

```text
UART4 DMA/IDLE
  → HAL_UARTEx_RxEventCallback()
  → USARTNormalRxHandler()
  → MTF02_UART_EventCallback()
       → MTF02_ConsumeMicoLinkByte()
       → MTF02_ConsumeMspByte()
       → MTF02_AcceptMicoLinkFrame() / MTF02_AcceptMspFrame()
  → SensorHub_Task()
  → MTF02_Task_Handler()
  → PubPushMessage("mtf02_data")
  → Attitude_UpdateMtfObservations()
  → Attitude_UpdateMtfFlow()
  → NAV_ESKF_UpdateFlow()
```

| 函数 | 处理内容 | 复制/同步/阻塞 |
|---|---|---|
| `MTF02_UART_EventCallback()`，`modules_mtf02.c:394` | 在 UART ISR 中逐字节同时推进 MicoLink 与 MSPv2 状态机 | 不先复制原始块；**协议解析发生在 ISR** |
| `MTF02_AcceptMicoLinkFrame()` / `MTF02_AcceptMspFrame()`，`modules_mtf02.c:127/172` | 校验后的字段写入 `latest_data`，更新协议/质量/时间戳并通知 SensorHub | seqlock 写；不等待任务 |
| `MTF02_Task_Handler()`，`modules_mtf02.c:457` | `latest_data` 结构体快照到局部 `data`；判断哪些计数有更新；MSPv2 积分量换算成 rad/s | 1 次逻辑结构体复制；最多重试 3 次 |
| `PubPushMessage()` | 发布 `MTF02_Data_t` | 1 次结构体复制 |
| `Attitude_UpdateMtfObservations()`，`App_attitude_observations.c:155` | 非阻塞读取最新帧；计算高度有效性并记录 MTF 自带测距诊断 | 1 次结构体复制；不融合 MTF 测距 |
| `Attitude_UpdateMtfFlow()`，`App_attitude_observations.c:91` | 检查新时间戳、质量、状态、测距时差和高度；构造机体系速度观测 | Attitude 任务局部计算 |
| `NAV_ESKF_UpdateFlow()` | NIS/门限通过时更新水平速度/位置相关状态 | 原地更新 ESKF |

确认的问题：MTF02 是当前传感器链中唯一在 UART ISR 内运行完整协议状态机的模块。推荐结构应是 ISR 把原始字节复制到 SPSC 环或 ping-pong 块，`SensorHub_Task()` 再调用两个解析器。

## 6) SPL06 气压数据链

```text
EXTI0_IRQHandler()
  → HAL_GPIO_EXTI_Callback()
  → SPL06_DRDY_Handler()
  → IICMemRead()
  → HAL_I2C_Mem_Read_DMA()
  → HAL_I2C_MemRxCpltCallback()
  → IIC_CompleteRouting()
  → SPL06_IIC_EventCallback()
  → SensorHub_Task()
  → SPL06_Task_Handler()
  → SPL06_CompensateFrame()
  → PubPushMessage("spl06_data")
```

| 函数 | 处理内容 | 复制/同步/阻塞 |
|---|---|---|
| `SPL06_DRDY_Handler()`，`modules_SPL06.c:256` | 若 I²C 空闲，启动连续读取 11 字节压力/温度/状态寄存器 | DMA 直接写 `spl06_dma_rx_buffer`；不阻塞 |
| `SPL06_IIC_EventCallback()`，`modules_SPL06.c:220` | DMA 缓冲 → `raw_frame[11]`，配对 DRDY 时间戳并通知 SensorHub | 1 次 ISR 拷贝；seqlock |
| `SPL06_ReadConsistentFrame()` | `raw_frame` → 栈 `local_frame[11]` | 1 次任务拷贝；最多重试 3 次 |
| `SPL06_CompensateFrame()` | 24 位原始压力/温度 → 数据手册补偿 → Pa、℃、海拔 | 任务内浮点计算，不阻塞 |
| `PubPushMessage()` | 发布 `SPL06_Data_t` | 1 次结构体复制 |

当前实机构建没有订阅 `spl06_data` 进行 ESKF 融合，所以数据流到 topic 为止；它不会影响定高控制。

## 7) Message Center 的真实数据结构和函数

### 7.1 发布

`PubPushMessage()`，`Modules/modules_Message_center/modules_Message_center.c:356`：

```c
SeqLock_WriteBegin(&pub->seqlock);
memcpy(pub->data_ptr, data, pub->data_len);
SeqLock_WriteEnd(&pub->seqlock);

/* 只通知正在等待的订阅任务，不复制第二份 payload。 */
xTaskNotifyGive(task);
```

- 每个 topic 只有一个 `pub->data_ptr`。
- 发布一次只复制一份，不会给每个 subscriber 复制一份。
- 新发布会覆盖旧 payload，因此不是 FIFO。
- 发布者从不等待订阅者。

### 7.2 非阻塞读取

`TryRead()`，`modules_Message_center.c:165`：

```c
seq = SeqLock_TryReadBegin(&pub->seqlock);
memcpy(out, pub->data_ptr, pub->data_len);
if (SeqLock_ReadRetry(&pub->seqlock, seq)) {
    continue;
}
sub->last_read_seq = seq;
```

- 最多重试 `Message_center_MAX_READ_RETRY = 3`。
- 复制期间被发布者打断就丢弃这次本地副本并重试。
- 三次都冲突时返回 0，绝不无限自旋。

### 7.3 阻塞读取

`SubGetMessage()`，`modules_Message_center.c:403`：先 `TryRead()`，无新数据时登记 `waiting_task`，再检查一次以关闭丢唤醒窗口，随后 `ulTaskNotifyTake()`。因此：

- 阻塞的是 subscriber 自己，不是 publisher。
- 通知只代表“可能有新版本”，payload 仍需通过 seqlock 读取。
- 多次发布可能合并为一次唤醒，只保证最新值，不保证每帧到达。

## 8) USB 接收和 MAVLink 解析

```text
CDC_Receive_FS()
  → USB_CDC_RxHook()
  → App_USBEventCallback()
  → Mavlink_InputBytesFromISR()
  → RingBuffer_Push(rx_ringbuffer)
  → vTaskNotifyGiveFromISR(mav_rx)
  → MavlinkRxTask()
  → RingBuffer_Pop(local buffer)
  → mavlink_parse_char()
  → App_MavlinkMessageCallback()
  → System/Command/HIL/Log Service
```

| 函数 | 数据操作 | 复制/阻塞 |
|---|---|---|
| `USB_CDC_RxHook()`，`Bsp/bsp_usb/bsp_usb.c:299` | 把 USB middleware 的指针转交应用回调 | 不复制；ISR |
| `Mavlink_InputBytesFromISR()`，`mavlink_user.c:62` | USB chunk → 4096 字节 RX SPSC 环 | 1 次字节复制；满时整块丢弃；ISR 不阻塞 |
| `MavlinkRxTask()`，`mavlink_user.c:87` | RX 环 → 栈 buffer，再逐字节调用 `mavlink_parse_char()` | 1 次字节复制；空闲时 `portMAX_DELAY` 阻塞 |
| `App_MavlinkMessageCallback()`，`App_Data_Comm.c` | 完整 `mavlink_message_t` 依次广播到四个服务 | 同一只读指针传递，不为每个服务复制 |

## 9) MAVLink/业务数据到 USB 发送

```text
XxxService_Update()/HandleMavlinkMessage()
  → Mavlink_EncodeXxx(local buffer)
  → DataRouter_PostFrame()
  → free_queue 取 DataRouter_TxSlot_t*
  → memcpy(slot->data, local buffer)
  → tx_queue 发送 slot 指针
  → DataRouterTask()
  → USBSend()
  → RingBuffer_Push(usb tx ring)
  → USBTxTask()
  → RingBuffer_Pop(tx_pending_buffer)
  → CDC_Transmit_FS(tx_pending_buffer)
  → USB_CDC_TxCpltHook()
```

| 函数 | 数据操作 | 复制/阻塞 |
|---|---|---|
| `DataRouter_PostFrame()`，`DataRouterTask.c:380` | 零等待取得空闲槽；把序列化 MAVLink 帧复制到槽；队列只复制槽指针 | 1 次完整帧复制；槽池满直接失败 |
| `DataRouterTask()`，`DataRouterTask.c:494` | 最多阻塞 1 ms 等待 tx_queue；集中处理积压帧和周期 Service | 队列传指针，不复制帧 |
| `USBSend()`，`bsp_usb.c:73` | slot → 32 KiB USB TX SPSC 环 | 1 次字节复制；环满丢弃，不阻塞 |
| `USBTxTask()` / `USBTransmitHandler()`，`bsp_usb.c:151/185` | TX 环 → 512 字节 `tx_pending_buffer`；提交 USB CDC | 1 次字节复制；任务最多等待 20 ms |
| `USB_CDC_TxCpltHook()`，`bsp_usb.c:324` | 清 busy/pending，累计发送字节并通知 USB TX 任务 | 不复制；ISR 不阻塞 |

`tx_pending_buffer` 在 USB 完成回调前不能复用；这是单缓冲“所有权保持”，不是双缓冲。

## 10) Control/估计器数据到 SD 日志

```text
Control_UpdateLogSnapshot()
  → Control_GetLogSnapshot()
  → LogService_Update()
  → SDCard_EnqueueFrame()
       → 构造 header + payload + CRC 到栈 frame[]
       → RingBuffer_Push(sd_ring_buffer)
  → SDCard_Task()
  → RingBuffer_Pop(sd_write_buffer)
  → SDCard_WriteExact()
  → f_write()
  → sd_diskio / SDMMC DMA
  → f_sync()
```

| 函数 | 数据操作 | 复制/阻塞 |
|---|---|---|
| `Control_UpdateLogSnapshot()`，`App_Controll.c` | Control 任务构造最新控制快照 | 结构体/数组复制；由 Control 单任务写 |
| `Control_GetLogSnapshot()`，`App_Controll.c` | `control_instance.log_snapshot` → DataRouter 局部快照 | 1 次完整结构体复制；短临界区 |
| `LogService_Update()`，`log_service.c:286` | 把快照选择/转换为各版本日志 payload | 局部结构体构造；DataRouter 任务上下文 |
| `SDCard_EnqueueFrame()`，`modules_SD_Card.c:497` | header 和 payload 复制到栈 `frame[]`，计算 CRC，再整帧复制到 128 KiB SD 环 | payload 1 次 + 整帧 1 次；环满立即丢帧 |
| `SDCard_Task()`，`modules_SD_Card.c:570` | SD 环 → `sd_write_buffer` → FatFS | 1 次批量复制；没有数据时延时 5 ms |
| `SDCard_WriteExact()` / `f_write()`，`modules_SD_Card.c:134` | 写文件并更新耗时/错误计数 | 可能阻塞 SD 任务；不会阻塞 Control/Attitude |
| `sd_diskio.c` | 对齐缓冲直接 SDMMC DMA；未对齐时逐扇区复制到 512 字节 `scratch`；写前 clean、读后 invalidate | SD DMA 完成等待上限为 30 s，仅发生在 SD 任务 |

## 11) FreeRTOS 任务调度与阻塞点

`configMAX_PRIORITIES = 56`，数值越大优先级越高；Tick 为 1 kHz。

| 任务 | 优先级 | 激活方式 | 正常阻塞点 | 主要数据所有权 |
|---|---:|---|---|---|
| defaultTask | 55 | 调度器启动 | 初始化完成后 `vTaskDelete(NULL)` | 系统初始化顺序；运行期不存在 |
| Attitude | 54 | BMI088 DMA 完成通知 | `ulTaskNotifyTake(..., portMAX_DELAY)` | 滤波、VQF、ESKF；发布反馈后通知 Control |
| Control | 53 | Attitude 反馈通知或 1 ms 超时 | `ulTaskNotifyTake(pdTRUE, 1 ms)` | PID、模式、混控、最新命令/反馈 |
| SensorHub | 52 | 传感器事件位 | `xTaskNotifyWait(..., 5 ms)` | SBUS/TFmini/MTF02/SPL06 任务级解析 |
| Uav Cmd | 51 | SBUS topic 通知 | `SubGetMessage(..., 10 ms)` | SBUS → 控制命令转换 |
| MAVLink RX | 50 | USB RX 通知 | `ulTaskNotifyTake(..., portMAX_DELAY)` | MAVLink RX parser |
| USB TX | 49 | 新 TX 数据/完成通知 | `ulTaskNotifyTake(..., 20 ms)` | USB pending 缓冲和 CDC busy 状态 |
| DataRouter | 48 | TX queue 或 1 ms 超时 | `xQueueReceive(..., 1 ms)` | 外发槽、Service 更新、SD 唯一生产者 |
| Timer Service | 47 | 软件定时器命令/到期事件 | FreeRTOS timer queue | 软件定时器回调 |
| SD Card | 46 | 环轮询/请求 | 空闲延时 5 ms；FatFS/SD DMA 可长时间阻塞 | FatFS 文件、SD 写缓冲 |
| Performance | 45 | 周期运行 | 周期延时 | 性能统计快照 |

## 12) 汇总结论

| 数据链 | 双缓冲 | 无锁 | 会复制 | 会阻塞生产者 | 过载策略 |
|---|---|---|---|---|---|
| BMI088 DMA | 是，陀螺和加速度各两份 | 基于单生产/单消费索引 | DMA 原始帧不复制，解析后结构体复制 | 否 | SPI 忙时丢当前 DRDY；任务落后时取最新完整半区 |
| SBUS/TFmini/SPL06 raw | 否，单 raw 快照 | seqlock，写者不等待 | ISR copy + task copy | 否 | 覆盖旧快照或读者有界重试失败 |
| MTF02 | 否 | seqlock 结果快照 | 不复制 raw，复制解析结果 | 否 | 多帧合并为最新结果；当前 ISR 解析过重 |
| Message Center | 否，每 topic 单 payload | seqlock latest-value | publish 一次 + 每次 read 一次 | 否 | 覆盖旧版本，不保证逐帧 |
| Control feedback | 否 | 否，短临界区 | write 一次 + read 一次 | 不等待，但短时屏蔽调度/中断 | 超时则停控/锁定 |
| USB RX/TX | TX pending 为单持有缓冲 | SPSC 环 | 环写一次 + 环读一次 | 否 | 满环丢新数据 |
| SD 日志 | 否 | SPSC 环 | 构帧、入环、出环均复制 | 否 | SD 慢时环满丢新日志；只有 SD 任务阻塞 |

当前最需要修改的数据流问题不是“复制太多”，而是 `MTF02_UART_EventCallback()` 把两个协议解析器放进 ISR。其余主控制链的所有权基本明确：DMA/ISR 不被慢任务反向阻塞，控制只读取最新一致数据。

## 13) 主要源码依据

- `Core/Src/stm32h7xx_it.c`
- `Bsp/bsp_spi/bsp_spi.c`
- `Bsp/bsp_uart/bsp_uart.c`
- `Bsp/bsp_iic/bsp_iic.c`
- `Bsp/bsp_usb/bsp_usb.c`
- `Bsp/bsp_ringbuffer/bsp_ringbuffer.c`
- `Modules/modules_Message_center/modules_Message_center.c`
- `Modules/modules_BMI088/modules_BMI088.c`
- `Application/App_attitude/App_attitude.c`
- `Application/App_attitude/App_attitude_observations.c`
- `Application/App_Sensor/App_Sensor.c`
- `Application/App_Uav_Cmd/App_Uav_Cmd.c`
- `Application/App_Controll/App_Controll.c`
- `Application/DataRouterTask/DataRouterTask.c`
- `Modules/modules_SD_Card/modules_SD_Card.c`
