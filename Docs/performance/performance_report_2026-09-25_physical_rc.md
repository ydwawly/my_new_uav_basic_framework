# 飞控性能复测报告（物理遥控器接入）

> 本文档是 FFT 重构前的物理遥控器/通信组合满载历史报告。当前 FFT 重构版本的最新基线见
> [FFT 重构后性能复测报告](performance_report_2026-09-25_fft_q15.md)。两份报告的固件和工况不同，数据不得混合。

## 1. 测试标识

- 日期：2026-09-25
- MCU：STM32H743VI，CPU 400 MHz，HCLK 200 MHz
- 固件 ELF：`cmake-build-performance/my_new_uav_baice_framework_v1.elf`
- ELF SHA-256：`E357001A03BD368C2DE1ABB47B03E72285807FD3462F2132CCB4EFD34407874B`
- 调试器：J-Link，SWD 4 MHz，序列号 602712338
- 物理遥控链路：SBUS，约 129.6 Hz
- 遥控器状态：有效、无 failsafe，解锁开关保持上锁

本报告只记录本次“物理遥控器已经接入”的新数据，不把 2026-09-24 无物理遥控器报告中的数值混入本次统计。

## 2. 测试工况

### 2.1 物理遥控器基线

- 复位后连续运行 100.5 s；
- 不启动上位机压力注入；
- SBUS 持续输入，累计接收 13,159 帧；
- 飞控保持上锁，SD 卡正常挂载。

### 2.2 物理遥控器与通信组合满载

- 物理 SBUS 持续输入；
- MAVLink RC override：25 Hz，上锁请求；
- PING 目标注入周期：1 ms；
- TIMESYNC 目标注入周期：2 ms；
- HIL_SENSOR 目标注入周期：2 ms，HIL 模式未开启，只压测解析与分发；
- 同时下载一个 248,715,578 B 的既有日志；
- 主机压力持续 90 s，中段飞控快照位于启动后 68.551 s；
- 飞控仍保持上锁，未绕过实体遥控器安全链。

主机定时器和串口吞吐不能完全达到目标注入频率。最终飞控回包实测约为 PING 544.46 Hz、TIMESYNC 299.70 Hz、LOG_DATA 504.99 Hz。

## 3. 结论摘要

1. 物理 SBUS 稳定在约 129.6 Hz；`rx_err_count=0`、`frame_lost=0`、`failsafe=0`。启动时记录到 1 次解析不完整帧，此后未继续增长。
2. 物理遥控器基线 CPU 为 11.0%；组合满载 CPU 为 37.2%，Idle 仍有 62.8%。
3. 组合满载下全部 9 个被监测任务的诊断截止期超限计数均为 0。
4. BMI088 DRDY 到控制任务开始为 101/116.706/489 us；姿态发布到控制任务开始为 11/12.894/36 us。
5. `uav_cmd` 在物理 SBUS 与 MAVLink RC 同时存在时，`start→finish` 最大 7.883 ms；这是处理 MAVLink 后等待下一帧 SBUS 造成的阻塞墙钟时间，不是 CPU 连续执行时间。由于 SBUS 周期约 7.7 ms，本轮没有超过 10 ms 诊断阈值。
6. 飞控侧 MAVLink 接收丢字节、接收序号丢包、DataRouter 槽耗尽、DataRouter 队列投递失败、USB 入队丢弃均为 0。
7. 最长最外层 BASEPRI 临界区为 10.435 us；MAVLink TX 互斥量最长等待为 139.393 us，没有持续优先级反转。
8. Heap 和所有任务栈均有正余量；没有 HardFault、SD 错误或电机故障。

## 4. CPU 与任务资源

### 4.1 CPU 总量

| 工况 | 总 CPU | Idle | 上下文切换率 |
|---|---:|---:|---:|
| 物理遥控器基线 | 11.0% | 89.0% | 7,033 次/s |
| 物理遥控器组合满载 | 37.2% | 62.8% | 16,186 次/s |

### 4.2 组合满载任务 CPU 与栈余量

| 任务 | 优先级 | CPU | 历史最小剩余栈 |
|---|---:|---:|---:|
| Attitude | 54 | 7.3% | 4,512 B |
| control | 53 | 2.0% | 1,440 B |
| sensor_hub | 52 | 0.3% | 1,632 B |
| uav_cmd | 51 | 0.1% | 1,060 B |
| mav_rx | 50 | 3.1% | 956 B |
| usb_tx | 49 | 2.4% | 1,704 B |
| data_router | 48 | 4.0% | 1,924 B |
| Timer Service | 47 | 约 0% | 876 B |
| sd_card | 46 | 17.4% | 2,936 B |
| performance | 45 | 约 0% | 1,168 B |
| Idle | 0 | 62.8% | 388 B |

### 4.3 FreeRTOS Heap

| 指标 | 数值 |
|---|---:|
| 总量 | 15,360 B |
| 当前空闲 | 6,360 B |
| 当前使用 | 9,000 B |
| 历史最低空闲 | 4,200 B |
| 历史峰值使用 | 11,160 B |

## 5. 任务实时性

### 5.1 物理遥控器基线

| 任务 | release→start 最短/平均/最长 | start→finish 最短/平均/最长 | release→finish 最短/平均/最长 |
|---|---:|---:|---:|
| Attitude | 5/5/251 us | 20/62/415 us | 25/68/421 us |
| control | 5/7/23 us | 3/6/361 us | 10/15/33 us |
| sensor_hub | 1/11/378 us | 0/4/382 us | 2/17/390 us |
| uav_cmd | 6/9/378 us | 3/3/131 us | 9/13/382 us |

基线下 `uav_cmd` 共处理 12,970 个 SBUS 批次，没有超时唤醒、通知合并或截止期超限。

### 5.2 物理遥控器组合满载

| 任务 | release→start 最短/平均/最长 | start→finish 最短/平均/最长 | release→finish 最短/平均/最长 | 截止期超限 |
|---|---:|---:|---:|---:|
| Attitude | 5/6/255 us | 20/64/437 us | 25/71/443 us | 0 |
| control | 5/7/30 us | 3/6/417 us | 10/15/40 us | 0 |
| sensor_hub | 2/12/385 us | 0/5/381 us | 5/18/391 us | 0 |
| uav_cmd | 6/10/192 us | 3/1128/7883 us | 9/14/195 us | 0 |
| mav_rx | 3/12/387 us | 0/18/481 us | 12/31/497 us | 0 |
| usb_tx | 2/6/465 us | 0/5/438 us | 3/12/467 us | 0 |
| data_router | 3/9/956 us | 1/18/549 us | 8/36/980 us | 0 |
| sd_card | 无离散发布事件 | 3/2130/32557 us | 无离散发布事件 | 0 |
| performance | 约 1 s 周期 | 185/261/970 us | 约 1 s 周期 | 0 |

`uav_cmd` 的 `start→finish` 包含等待 SBUS 的阻塞时间。任务在等待期间处于 Blocked，不消耗对应的 1.128 ms 平均 CPU 时间；实际 CPU 占用只有约 0.1%。本轮共记录 2,577 个合并事件，说明 MAVLink RC 与 SBUS 到达存在重叠。

## 6. 周期与抖动

| 指标 | 最短 | 平均 | 最长 | 峰峰值 |
|---|---:|---:|---:|---:|
| Attitude 有效周期 | 979.477 us | 1000.189 us | 1023.308 us | 43.831 us |
| control 正常控制周期 | 631.607 us | 1000.247 us | 1383.637 us | 752.030 us |

Attitude 相对 1 ms 的最大提前约 20.523 us、最大延后约 23.308 us。control 的统计同时受姿态通知、1 Tick 安全检查和抢占位置影响，不能直接理解为控制律自身执行时间抖动；控制律探针为 5.648/6.794/29.405 us。

## 7. 关键端到端延迟

> 本节是 FFT 重构前固件的历史数据，且终点是“控制任务开始”，不包含随后完整控制业务路径。
> 最新固件已经新增并实测“BMI088 DRDY→控制业务完成”，结果见
> [最新报告第 4.6 节](performance_report_2026-09-25_fft_q15.md#46-bmi088-drdy-到控制任务完成)。

| 链路 | 样本数 | 最短 | 平均 | 最长 |
|---|---:|---:|---:|---:|
| BMI088 DRDY→姿态任务开始 | 67,000 | 39 us | 41.956 us | 64 us |
| 姿态发布→控制任务开始 | 66,012 | 11 us | 12.894 us | 36 us |
| BMI088 DRDY→控制任务开始 | 66,012 | 101 us | 116.706 us | 489 us |
| 当前控制指令时间戳→控制任务开始 | 66,012 | 109 us | 4.556 ms | 15.388 ms |

最后一项混合了物理 SBUS 和 MAVLink RC 两个独立输入源；最新指令时间戳会被实体 SBUS覆盖，因此它反映双输入更新相位及任务等待，不代表 IMU 控制链失去 1 kHz 实时性。

电机保持上锁，所以本次 `IMU/姿态/控制指令→Motor_SetOutput()` 没有有效样本，不能用旧报告的解锁数据代替。

## 8. 中断执行时间

| IRQ | 样本数 | 最短 | 平均 | 最长 |
|---|---:|---:|---:|---:|
| SPL06 EXTI | 2,148 | 3.202 us | 4.131 us | 17.120 us |
| BMI270 EXTI | 53,548 | 2.565 us | 3.878 us | 8.988 us |
| BMI088 EXTI | 68,012 | 2.572 us | 3.535 us | 4.568 us |
| SPI2 RX DMA | 271,688 | 0.422 us | 0.573 us | 5.585 us |
| SPI3 RX DMA | 107,096 | 0.425 us | 0.640 us | 5.295 us |
| SPL06 DMA | 2,148 | 0.678 us | 1.136 us | 20.335 us |
| SDMMC | 6,909 | 2.850 us | 3.759 us | 31.547 us |
| USB FS | 370,320 | 0.230 us | 3.425 us | 30.135 us |
| MTF02 UART | 3,266 | 6.720 us | 9.618 us | 25.510 us |

这些数值是 ISR C Handler 墙钟执行时间，不是外部引脚边沿到 CPU 第一条指令的物理中断响应时间。

## 9. 临界区、互斥量和缓存

| 指标 | 样本/事件数 | 最短 | 平均 | 最长 |
|---|---:|---:|---:|---:|
| 最外层 FreeRTOS BASEPRI 临界区 | 4,250,695 | 0.093 us | 0.309 us | 10.435 us |
| MAVLink TX 递归互斥量等待 | 148,150 | 0.115 us | 0.850 us | 139.393 us |
| 优先级继承 | 184 | — | — | — |

全部任务快照中 `current_priority == base_priority`，没有未释放互斥量或持续优先级反转。互斥量等待不是关中断时间。

D-Cache 已开启。8 KiB 顺序读基准为：冷缓存 6,152 cycles、热缓存 4,111 cycles、非缓存 SRAM 28,677 cycles、DTCM 4,104 cycles。Cortex-M7 当前没有可用的直接命中/未命中计数器，因此不报告伪造的命中率。

## 10. 通信、队列与 SD

### 10.1 飞控侧

| 指标 | 数值 |
|---|---:|
| MAVLink RX bytes（中段快照） | 2,290,156 B |
| MAVLink RX messages | 64,459 |
| MAVLink RX dropped bytes | 0 |
| MAVLink RX sequence drops | 0 |
| DataRouter posted/dispatched | 75,189 / 75,189 |
| DataRouter 槽耗尽丢弃 | 0 |
| DataRouter 队列投递失败 | 0 |
| USB 入队丢弃 | 0 |
| DataRouter 队列峰值 | 5 / 8 槽 |

### 10.2 上位机 90 s 压力结果

| 指标 | 数值 |
|---|---:|
| 接收字节 | 5,858,062 B |
| 发送字节 | 3,134,061 B |
| 接收 MAVLink 包 | 103,949 |
| 飞控源序号丢包 | 0 |
| 主机主动丢弃待发送请求 | 3,062 |

主机主动丢弃发生在测试脚本把注入速率推到串口/事件循环的可持续吞吐以上；飞控端 `rx_dropped_bytes` 和 DataRouter/USB 丢弃仍为 0，不能把两种计数混为飞控丢包。

SD 日志下载持续输出约 505 个 `LOG_DATA`/s，SD 任务峰值 CPU 为 17.4%，无 FatFs、写入或同步错误。由于飞控保持上锁，本轮新飞行日志只有 24 B 文件头，未形成“飞行日志持续写入 + 历史日志下载”的双向 SD 压力。

## 11. 故障与能力边界

- `fault_diagnostic.fault_type = FAULT_TYPE_NONE`；
- `motor_instance.state = MOTOR_STATE_DISARMED`；
- `motor_instance.fault = MOTOR_FAULT_NONE`；
- SBUS `frame_lost=0`、`failsafe=0`；
- SD 卡保持 `SD_CARD_STATUS_RUNNING`、`FR_OK`。

本次尝试通过 SystemView 4.10b 命令行重新录制，但命令行实例未完成保存，因此没有生成可验证的新 `.SVDat`，本报告不把旧轨迹冒充为本次物理遥控器轨迹。任务、ISR、临界区和端到端数值来自目标侧 DWT/FreeRTOS 累计器。

要补齐与旧“解锁满载”完全等价的物理遥控器工况，需要实体遥控器在最低油门下切到解锁，并保持电机电源断开；随后重新采集电机输出链路、飞行日志持续写入和 SystemView 轨迹。

## 12. 复现产物

- 上位机压力脚本：`D:/myFiles/RM/uav_basic_framework/uav_control_upper_computer/scripts/firmware-pressure-physical-rc.cjs`
- 90 s 主机结果：`D:/myFiles/RM/uav_basic_framework/uav_control_upper_computer/artifacts/pressure-test-physical-rc/pressure-result-1790320133205.json`
- 45 s 复核结果：`D:/myFiles/RM/uav_basic_framework/uav_control_upper_computer/artifacts/pressure-test-physical-rc/pressure-result-1790320295381.json`
