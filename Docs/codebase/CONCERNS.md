# 代码库关注项

## 1) 最高优先级风险

| 严重度 | 关注项 | 依据 | 影响 | 建议动作 |
|---|---|---|---|---|
| 高 | `Motor_SafetyCheck()` 当前没有调用者 | `Modules/modules_Motor/module_pwm_motor/module_pwm_motor.c:Motor_SafetyCheck`，全仓调用搜索 | 若 Control 任务停滞，最后一次 PWM 比较值可能继续保持 | 增加独立调度的安全看门狗；先在无桨条件下做故障注入验证 |
| 高 | 控制输出失败路径会用紧急停机故障覆盖原始电机故障 | `Application/App_Controll/App_Controll.c`、`Motor_EmergencyStop()` | 输出失败后的首个根因会丢失，调试诊断不完整 | 保留第一个故障原因，同时仍强制最小电机输出 |
| 中 | MTF02 协议状态机运行在 UART DMA 回调中 | `Modules/mudules_optical_flow_sensor/modules_mtf02/modules_mtf02.c:MTF02_UART_EventCallback` | 较长或突发的 UART 数据块会延长 ISR 执行时间；这与 SBUS/TFmini/SPL06 的“ISR 仅快照与通知”结构不同 | ISR 仅保存原始字节快照/环形缓冲；把两个解析器移至 `SensorHub_Task`。修改前先量测 UART 块长度和 IRQ 最坏执行时间 |
| 中 | VQF 负责控制姿态，ESKF 负责导航与定高资格 | `Application/App_attitude/App_attitude.c`、`Application/App_Controll/App_Controll.c` | 日志读取者可能错误比较来自不同估计器的字段 | 继续显式区分 `control_*` 与 ESKF 字段，并按数据来源复核飞行日志 |
| 中 | 定高依赖有效且近期的 TFmini 融合；SPL06 不参与当前实机融合 | `Application/App_attitude/App_attitude_observations.c` | 测距质量或时序异常会导致定高不可用或退出 | 在最终机架上验证测距观测门限和失效处理 |

## 2) 技术债务

| 技术债务 | 成因 | 位置 | 忽略后的风险 | 建议处理 |
|---|---|---|---|---|
| 旧目录拼写 | 历史工程命名 | `App_Controll`、`mudules_*` | 搜索、导入或新代码路径容易写错 | 仅在单独的迁移任务中改名，并复核构建系统 |
| 未发现 CI 流水线 | 当前以本地桌面工作流为主 | 扫描输出 | 回归依赖人工执行 | 加入主机回归测试与格式检查 CI |
| 命令服务仍有 TODO | 功能扩展被刻意延后 | `Application/App_Data_Comm/command_service.c` | 后续需要 MAVLink 命令时缺少明确接口 | 仅在命令协议明确后实现 |

## 3) 安全关注项

| 风险 | 分类 | 依据 | 现有缓解措施 | 缺口 |
|---|---|---|---|---|
| USB MAVLink 接收应用层未认证的协议命令 | 嵌入式物理链路风险 | `Application/App_Data_Comm/App_Data_Comm.c` | 服务内存在目标系统/组件筛选 | 当前信任物理 USB 访问；若暴露外部连接器，应先定义访问策略 |
| 扫描的固件源码未发现凭据存储 | 不适用 | 扫描输出 | 未发现密钥 | [TODO] 后续接入网络遥测时重新检查 |

## 4) 性能与扩展性关注项

| 关注项 | 依据 | 当前表现 | 扩展风险 | 建议改进 |
|---|---|---|---|---|
| SD 写卡刻意使用低优先级 | `Modules/modules_SD_Card/modules_SD_Card.c` | 环形缓冲满时日志会丢失 | 高日志速率可能损失诊断数据 | 飞行中监控 `sd_card_state` 的丢帧/环形缓冲计数 |
| DataRouter 使用固定队列和固定槽池 | `Application/DataRouterTask/DataRouterTask.h` | 满载时按背压策略丢帧 | 遥测/日志突发时会丢消息 | 依据实测流量而非假设配置队列深度 |
| SPI/I²C 总线所有权依赖单核单生产者假设 | `Bsp/bsp_spi/bsp_spi.c`、`Bsp/bsp_iic/bsp_iic.c` | 当前调用者数量受控 | 新增多生产者可能引入竞争 | 每次新增 SPI/I²C 生产者时重新审计，必要时引入总线服务任务或短临界区 |

## 5) 脆弱或高变更区域

| 区域 | 脆弱原因 | 变更信号 | 安全修改策略 |
|---|---|---|---|
| `Application/App_Controll/App_Controll.c` | 模式、定高、PID 和混控相互关联 | 90 天扫描中 5 次提交 | 每次只改一种行为类别，保留日志对比 |
| `Application/App_attitude/` | VQF、ESKF 与多个观测源并存 | 核心文件各有 3–4 次提交 | 保留估计器来源标识，测试实机/HIL 两种构建 |
| SD/日志文件 | 二进制 ABI 和任务所有权边界严格 | 3–4 次提交 | 保留静态布局断言和上位机解析回归测试 |

## 6) 待确认问题

1. [ASK USER] 计划让 GPS 进入正式导航链路吗？GPS 当前是编译期可选，而文档中的实机定高路径目前依赖 TFmini 与并行 ESKF。
2. [ASK USER] 是否应把未启用的电机输出超时保护作为下一项独立安全修改，并在后续带桨测试前完成？

## 7) 依据

- `docs/codebase/.codebase-scan.txt`
- `Modules/modules_Motor/module_pwm_motor/module_pwm_motor.c`
- `Application/App_Controll/App_Controll.c`
- `Application/App_attitude/App_attitude_observations.c`
