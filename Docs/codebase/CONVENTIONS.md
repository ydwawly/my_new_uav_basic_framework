# 编码规范

## 1) 命名规则

| 项目 | 规则 | 示例 | 依据 |
|---|---|---|---|
| 文件 | 模块前缀的 C/H 成对文件；保留历史目录拼写 | `App_Controll.c`、`mudules_Sbus.c` | `Application/`、`Modules/` |
| 函数 | 模块 Pascal/camel 前缀后接动作 | `Control_SetFeedback`、`BMI088_GetData` | `Application/App_Controll/App_Controll.c` |
| 类型 | PascalCase，按类别以 `_t`、`_s` 或 `_e` 结尾 | `Control_Feedback_t`、`IIC_Init_Config_s` | `Application/App_Controll/App_Controll.h` |
| 常量 | 大写模块前缀；相关时在名称中携带单位 | `ATTITUDE_GYRO_LPF_CUTOFF_HZ` | `Application/App_attitude/App_attitude_config.h` |

## 2) 格式与静态检查

- 格式化工具：clang-format，配置文件为 `.clang-format`。
- 静态检查：`code_quality_check` CTest 门禁，同时校验已配置的源码约束。
- 相关规则：改动文件必须格式正确；生成/厂商代码不纳入维护代码重构；二进制日志布局需要静态断言。
- 运行命令：`clang-format --dry-run --Werror <files>` 与 `ctest --test-dir <build-dir> --output-on-failure`。

## 3) 引用与模块规则

- 每个源文件先 include 对应公共头文件，再 include 标准/系统/模块头文件。
- 没有路径别名；include 目录由 CMake 提供。
- 公共头文件只暴露调用者契约；模块私有状态和辅助函数应放在 C 文件并使用 `static`。

## 4) 错误与日志规则

- 按底层契约使用 `bool`、`uint8_t` 或 `HAL_StatusTypeDef` 返回错误。
- 硬件失败应更新模块诊断状态，并使用 `RTTERROR`、`RTTWARNING`、`RTTINFO` 等 RTT 宏。
- 高频路径优先使用计数器/标志，而非无限制诊断输出。
- [TODO] 扫描未发现工程级敏感数据/脱敏策略；当前固件没有凭据存储。

## 5) 测试规则

- 测试脚本位于 `Tests/`；上位机解析器测试位于 `uav_control_upper_computer/tests/`。
- 需要硬件行为覆盖的回归脚本，会直接用 HAL Mock 编译选定正式 C 源文件。
- 未发现覆盖率阈值配置。

## 6) 依据

- `.clang-format`
- `CMakeLists.txt`
- `Application/App_Controll/App_Controll.c`
- `Bsp/bsp_iic/bsp_iic.h`
