# 外部集成

## 1) 集成清单

| 系统 | 类型 | 用途 | 认证模型 | 关键程度 | 依据 |
|---|---|---|---|---|---|
| BMI088 | SPI DMA 传感器 | 姿态/控制的六轴 IMU 输入 | 仅物理总线 | 高 | `Modules/modules_BMI088/modules_BMI088.c` |
| TFmini Plus | UART 传感器 | ESKF Z 轴/定高测距观测 | 仅物理 UART | 定高时高 | `Application/App_attitude/App_attitude_observations.c` |
| MTF02 | UART 传感器 | 光流速度观测 | 仅物理 UART | 中 | `Application/App_attitude/App_attitude_observations.c` |
| SPL06 | I²C/DMA 传感器 | 气压传感器采集 | 仅物理 I²C | 当前实机融合中低 | `Application/App_Sensor/App_Sensor.c` |
| SBUS 接收机 | UART DMA 输入 | 飞手指令和 failsafe | 物理无线链路 | 高 | `Modules/mudules_Remote_sensor/mudules_Sbus/mudules_Sbus.c` |
| ESC | TIM1 PWM 输出 | 四路电机命令 | 物理 PWM | 高 | `Modules/modules_Motor/module_pwm_motor/module_pwm_motor.c` |
| USB CDC + MAVLink | USB 协议 | 地面站、HIL 和日志传输 | 未发现应用层认证 | 遥测高 | `Application/App_Data_Comm/App_Data_Comm.c` |
| SD 卡 | SDMMC/FatFS | 黑匣子日志持久化 | 可拆卸物理存储 | 中 | `Modules/modules_SD_Card/modules_SD_Card.c` |

## 2) 数据存储

| 存储 | 职责 | 访问层 | 关键风险 | 依据 |
|---|---|---|---|---|
| SD FAT 文件系统 | 持久化黑匣子文件 | 仅低优先级 `SDCard_Task` | SD 延迟或挂载失败会丢日志，不会停止控制环 | `Modules/modules_SD_Card/modules_SD_Card.c` |
| SD SPSC 环形缓冲 | 写卡前的帧缓冲 | DataRouter 生产 / SD 任务消费 | 满环会丢弃新帧并增加计数 | `Modules/modules_SD_Card/modules_SD_Card.h` |
| Message Center topic 存储 | 最新结构化任务间消息 | 发布/订阅者复制 | 消费者必须自己检查新鲜度 | `Modules/modules_Message_center/modules_Message_center.c` |

## 3) 凭据与密钥处理

- 凭据来源：扫描的固件源码中未发现。
- 硬编码检查：扫描未发现环境模板或凭据机制。
- 轮换/生命周期：当前物理外设集成不适用。

## 4) 可靠性与失效行为

- 接收机丢帧/failsafe 会形成 `Uav_Cmd_t.failsafe=1`、关闭解锁请求且油门为零。
- SensorHub 将 UART/I²C 解析延后至任务上下文；单个驱动初始化失败会使该传感器不进入 ready mask。
- TFmini 的新鲜度/测距及 ESKF 有效性共同限制定高；导航无效会拒绝或退出闭环定高。
- SD 日志对控制链路非阻塞；挂载/写入失败会在 `sd_card_state` 暴露。

## 5) 集成可观测性

- RTT 启动/错误日志：覆盖传感器、电机、SD 和通信初始化。
- 二进制黑匣子：飞控、估计器对比、陷波、IMU 与 MAVLink 帧。
- 调试快照：Ozone ELF 中可查看 control、SBUS、VQF/ESKF 与 SD 状态。
- 缺口：当前没有调度主动电机输出超时保护，见 `CONCERNS.md`。

## 6) 依据

- `Application/App_Uav_Cmd/App_Uav_Cmd.c`
- `Application/App_attitude/App_attitude_observations.c`
- `Application/App_Data_Comm/App_Data_Comm.c`
- `Modules/modules_SD_Card/modules_SD_Card.c`
