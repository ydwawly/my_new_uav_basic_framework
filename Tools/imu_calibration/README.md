# BMI088 USB 固定参数 LM 与 Allan 分析

## 1. 当前采集模式

固件使用 STM32 USB CDC 直接输出 BMI088 二进制 FLOG，不需要 SD 卡。为保证标定安全与带宽，当前配置为：

- `APP_ENABLE_MOTOR_OUTPUT=0U`：不创建电机输出与闭环控制任务；
- `APP_ENABLE_SD_LOGGER=0U`：不启动 SD 卡任务；
- `ATTITUDE_ENABLE_BMI088_CALIBRATION_STREAM=1U`：约 1 kHz 提交 BMI088 样本；
- `DATA_ROUTER_ENABLE_USB_IMU_STREAM=1U`：USB 输出 FLOG；
- `DATA_ROUTER_ENABLE_MAVLINK_DURING_IMU_STREAM=0U`：采集期间不混入 MAVLink 字节。

日志保存的是完成芯片轴到飞控 FRD 轴映射和 SI 单位换算、但尚未应用固定参数的数据：

```text
accel_corrected = accel_matrix @ (accel_logged - accel_bias)
gyro_corrected  = gyro_matrix  @ (gyro_logged  - gyro_bias)
```

本标定固件将 H743 从 `480 MHz / VOS0` 调整为 `400 MHz / VOS1`，以降低飞控板自热；PLL1-Q 仍为 160 MHz，所以 BMI088 的 SPI2 仍保持 10 MHz。BMI088 温度寄存器已合并进原有加速度计 DMA 连续读取，不额外发起阻塞 SPI 事务。

USB 采集必须由电脑发送 `IMUCAP1 + 16 位随机会话令牌 + \n` 握手后才开始。每次重连都会重新发送带 CRC 和相同会话令牌的 FLOG 文件头，电脑端据此丢弃旧 USB 缓存并建立新的 part 文件。

## 2. 安装电脑端依赖

在工程根目录运行一次：

```powershell
python -m pip install -r Tools/imu_calibration/requirements.txt
```

飞控 USB VID/PID 为 `0483:5740`，采集程序默认自动寻找匹配的 STM32 Virtual COM Port；COM 号变化不影响自动重连。

## 3. 启动 USB 采集

无限时长采集，按 `Ctrl+C` 停止：

```powershell
python Tools/imu_calibration/capture_usb.py -o output/usb_capture
```

固定采集 60 分钟：

```powershell
python Tools/imu_calibration/capture_usb.py -o output/usb_capture --duration-minutes 60
```

如需指定端口：

```powershell
python Tools/imu_calibration/capture_usb.py --port COM7 -o output/usb_capture
```

自动重连行为：

1. 未找到飞控时每秒重新枚举端口；
2. 端口打开后清空电脑驱动旧缓存并发送握手；
3. 从第一个合法且 CRC 正确的 FLOG 文件头开始保存；
4. 拔线、端口号变化、读取异常或连续 3 秒无数据时关闭当前文件并重连；
5. 每次重连生成新的 `partNNN.BIN`，不把中断区间伪装成连续数据；
6. 文件每 5 秒执行一次 `flush + fsync`，降低电脑异常关机造成的数据损失。

程序每秒显示文件大小和平均传输速率。正常流量约为 70～80 KiB/s。

## 4. 实机观测量

通过调试器观察：

```text
data_router_log_stats.imu_log_dropped       == 0
data_router_log_stats.usb_stream_dropped    == 0
data_router_log_stats.usb_stream_active     == 1
data_router_log_stats.usb_stream_sessions   >= 1
data_router_log_stats.usb_header_send_failures 不持续增加
USBInstance.dropped_tx_bytes                 == 0
BMI088 runtime_stats.dma_busy_drop_count     == 0
attitude_runtime.missed_imu_count            == 0
```

如果任一丢帧计数持续增长，不应采用该日志的 Allan 参数。

## 5. 推荐：使用可视化界面采集 LM 数据

拆桨并给 BMI088 上电预热。不要只按固定 5 分钟判断；界面会直接显示 BMI088 温度与变化率，并使用最近 90～120 秒数据判断热稳定。然后双击：

```text
Tools/imu_calibration/start_guided_lm.bat
```

也可以在工程根目录运行：

```powershell
python Tools/imu_calibration/guided_lm_gui.py -o output/guided_lm
```

界面会自动查找 `0483:5740` 的 STM32 USB CDC，COM 号变化后仍能自动重连。操作方法：

1. 看到“温度已稳定，可以开始 LM 姿态采集”后点击“开始 / 重新采集”；进入稳定要求原有严格门槛连续满足 10 秒。进入后采用迟滞防抖：去除极少量瞬时跳点后的温差超过 `0.75 °C`，或斜率超过 `0.15 °C/min`，并持续 30 秒才退出稳定；短时波动只提示观察，不会立即锁死采集；
2. 左侧灰色实体四旋翼显示飞控当前姿态，右侧半透明绿色四旋翼显示目标姿态；机体上方的红色箭头明确标识机头 `+X`；
3. 按界面给出的“向左/向右滚转、抬机头/压机头”提示转动，让两个四旋翼姿态一致，累计合格保持 3 秒；人手保持时不超过 `0.35` 秒的短暂抖动只暂停进度，恢复后继续累计，持续抖动才从 0 重新计时；
4. 每个姿态自动验收并切换下一个目标，不需要按确认键；转场建议在 30 秒内完成，30～90 秒仍可用于陀螺仪 LM，超过 90 秒才不计入陀螺仪矩阵标定，但对应静止姿态仍用于加速度计标定；
5. 完成 18 个姿态后查看右下角整组质量判定；全部通过后，“运行 LM 标定”按钮才会启用。

上位机保存的 USB/BIN 和导出的 CSV 始终是映射后的原始物理量。实时静止判断、零偏漂移判断、转场积分以及点击“运行 LM 标定”后的离线求解，会统一先应用已通过第三轮独立数据验证的陀螺仪二次温漂模型；生成的 JSON 会记录完整预处理模型和系数，避免重复补偿或漏补偿。

加速度计只能确定倾斜/翻转，无法观测绕重力方向的航向角；界面中的航向仅用于固定四旋翼画法，不作为 LM 验收条件。

单姿态实时验收条件：

- 目标方向误差不大于 8°；
- 0.2 秒窗口内陀螺仪 RMS 小于 `0.035 rad/s`；
- 第一个正常平放姿态建立零偏参考后，上位机会显示后续姿态的陀螺仪窗口均值偏差；该数值仅用于提示，不再作为单姿态硬门槛，避免温漂、手持微动导致保持进度反复清零；
- 0.2 秒窗口内加速度三轴合成抖动 RMS 小于 `0.12 m/s²`；
- 平均加速度模长在 `0.88 g`～`1.12 g`；
- 上述条件累计满足 3 秒；不超过 `0.35` 秒的短暂抖动暂停累计，长时间不合格才重新计时。

整组数据还会自动检查：

- 18 个姿态全部完成，球面覆盖最小特征值不小于 0.20；
- 六个轴向静止姿态间最大陀螺仪零偏变化不大于 `0.05 deg/s`，斜姿态仍参与加速度计覆盖和转场标定，但不再把手持慢转误判成传感器零偏漂移；
- 18 个姿态都必须带有效温度，整组姿态均温跨度不大于 `1.0 °C`；
- 至少 12 个可用旋转转场；
- X/Y/Z 各轴累计旋转激励不小于 180°；
- USB 数据质量会分别显示丢样、CRC、重同步和时序异常，所有字段都必须为 0（正常间隔要求 500～2000 μs），不再把同一次丢帧在多个统计项中重复相加成一个总数。

如果温度持续超出迟滞门槛 30 秒，上位机会暂停采集并清零当前姿态尚未完成的 3 秒保持计时；温度重新满足严格门槛并确认 10 秒后再从该姿态重新计时。已经验收的姿态不会被删除，但最终整组温度跨度仍必须通过 `1.0 °C` 门槛。

从第一个姿态开始，上位机会实时显示整组温差预算；达到 `0.8 °C` 时预警，超过 `1.0 °C` 后停止接受新姿态并提示在当前温度重新开始，避免完成 18 个姿态后才判定失败。

如果采集中 USB 断开，上位机会：

1. 立刻关闭当前 BIN 并生成对应的 `*.interrupted.json`；
2. 保留不完整文件用于排查，但不允许它进入 LM；
3. 自动重新枚举端口、握手并创建新的 part；
4. 从第 1 个姿态自动重新开始，避免把断线前后拼成连续数据。

一次完成的输出目录包含：

- `BMI088_USB_*.BIN`：只由完整且 CRC 正确的 FLOG 帧组成；
- `BMI088_USB_*.guided_report.json`：上位机质量判断、18 个姿态均值和转场激励；
- 点击“运行 LM 标定”后生成 `lm_capture.csv`、`bmi088_fixed_calibration.json` 和 `bmi088_fixed_calibration_generated.h`。

界面生成的 C 头文件先保存在本次输出目录，不会直接覆盖飞控中的生产参数；检查结果后再决定是否复制到 `Modules/modules_BMI088/`。

### 5.1 手工采集备用流程

拆桨。上电预热约 5 分钟后启动 USB 采集，然后：

1. 至少选择 18 个静止姿态，覆盖正负 X/Y/Z 和多个斜向姿态；
2. 每个姿态完全静止 3～5 秒；
3. 姿态之间分别绕 X/Y/Z 和斜轴明显旋转，每次最好大于 30°；
4. 完成后按 `Ctrl+C` 停止。

选择没有发生重连、覆盖完整动作的 part 文件：

```powershell
python Tools/imu_calibration/imu_calibration.py extract `
  output/usb_capture/BMI088_USB_xxx_part001.BIN -o output/lm.csv

python Tools/imu_calibration/imu_calibration.py lm output/lm.csv `
  -o output/bmi088_fixed_calibration.json `
  --c-header Modules/modules_BMI088/bmi088_fixed_calibration_generated.h `
  --gyro-temperature-compensation
```

检查：

- `selected_pose_count >= 12`，推荐至少 18；
- 加速度计 `norm_rmse_after_mps2` 明显下降；
- 陀螺仪状态为 `matrix_and_bias`；若为 `bias_only`，应增加多轴旋转后重采。

生成头文件后重新编译、烧录。

## 6. 温漂模型状态与复测

当前固件和 LM 上位机已经写入同一套陀螺仪二次温漂模型。该模型使用前两轮冷机到热稳数据拟合，并已通过第三轮独立采集的冻结判据验证。模型在 `27.25～46.0 °C` 范围内计算，范围外按端点温度钳位；USB/BIN 中仍保存未补偿的 `gyro_uncalibrated[]`，因此可以保留原始证据并在电脑端复算。

更换飞控板、BMI088、安装结构或明显改变 H743 时钟/散热后，应重新执行下面的冷机到热稳采集，不要直接沿用本板系数：

1. 飞控断电并在室温下静置至少 20 分钟；
2. 将飞控可靠固定，整个过程不能移动、吹风或晒太阳；
3. 启动下面的命令后再给飞控上电，从冷机连续记录到温度稳定后至少再保持 15 分钟，建议总长 45～60 分钟；
4. 如果 USB 重连，只保留一份从冷机到稳定且没有中断的 part，必要时重新采集。

```powershell
python Tools/imu_calibration/capture_usb.py -o output/temperature_capture --duration-minutes 60
```

把生成的 `BMI088_USB_*_part001.BIN` 交给温漂拟合流程。应先检查温度有效标志、USB 丢帧、板子是否移动，再分别验证陀螺仪和加速度计三轴零偏；新模型必须用未参与拟合的独立冷启动数据验证后才能替换现有系数。

## 7. Allan 数据采集

固定飞控、关闭电机、避免振动和气流，并在温度稳定后连续记录至少 30 分钟，推荐 60～120 分钟：

```powershell
python Tools/imu_calibration/capture_usb.py -o output/allan_capture --duration-minutes 60
```

Allan 必须使用一份完整连续、没有重连的 part。若中途重连，应重新计时，或只使用满足时长的最长 part：

```powershell
python Tools/imu_calibration/imu_calibration.py extract `
  output/allan_capture/BMI088_USB_xxx_part001.BIN -o output/allan.csv

python Tools/imu_calibration/imu_calibration.py allan output/allan.csv `
  --calibration output/bmi088_fixed_calibration.json `
  -o output/allan
```

输出：

- `allan_deviation.png`；
- `allan_results.json`；
- `eskf_conservative` 中的四个 ESKF 建议值。

必须确认 `sequence_gap_count == 0`。加速度计保持样本依据 BMI088 24 位 sensor-time 删除。温度有效标志必须为 1，并确认整段数据已经处于热稳定阶段。

注意：当前 `allan` 子命令尚未自动应用上述温漂预处理。最终 Allan 参数应在温度稳定、变化很小的连续数据上计算；后续把同一预处理接入 Allan 时，需在结果元数据中明确记录，避免对已补偿数据再次补偿。

## 8. 完成标定后

将 `ATTITUDE_ENABLE_BMI088_CALIBRATION_STREAM` 和 `DATA_ROUTER_ENABLE_USB_IMU_STREAM` 改为 `0U`，按需求恢复 MAVLink、SD 黑匣子及电机输出。不要在正式飞行固件中长期保留 USB 标定模式。
