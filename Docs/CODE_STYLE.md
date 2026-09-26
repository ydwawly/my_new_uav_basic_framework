# 代码与注释规范

本规范借鉴 `hnuyuelurm/basic_framework` 的模块分层、接口命名和注释组织方式，并以仓库根目录 `.clang-format` 为唯一排版标准。

## 分层和依赖

- `Application` 可以依赖 `Modules` 与 `Bsp`；`Modules` 可以依赖 `Bsp`；`Bsp` 不依赖业务应用。
- HAL 句柄、DMA 和中断细节收敛在 `Core/Bsp`，上层使用模块接口。
- 一个模块只暴露完成业务所需的最小 API；私有状态和辅助函数使用 `static`。
- CubeMX 生成文件只在 USER CODE 区域修改；第三方代码不统一格式化。

## 命名

- 公共函数：模块前缀 + 动宾结构，如 `Control_SetFeedback`、`SDCard_RequestLogData`。
- 私有函数：同样使用模块前缀，并声明为 `static`。
- 变量和字段：`lower_snake_case`；宏和编译期配置：`UPPER_SNAKE_CASE`。
- 类型后缀：结构体 `_t`/`_s`，枚举 `_e`；已有稳定公共接口可保留原后缀，新增代码必须一致。
- 布尔或状态名称表达正向含义，如 `feedback_valid`、`motor_armed`。

## 注释

- 公共类型、宏语义和 API 在头文件使用 Doxygen 中文注释，说明参数、返回值、单位和调用约束。
- `.c` 文件重点解释设计原因：线程/中断上下文、数据所有权、单位、坐标系、DMA/Cache 限制、超时与异常回退。
- 注释必须与代码一致，不保留作者电脑、日期、用户名或“以后再说”的个人记录。
- 不做逐行翻译式注释；代码本身能表达的流程不重复描述。

## 实时与并发

- ISR 只确认硬件事件、搬运固定上界数据、记录时间戳并通知任务；不得解析可变长协议、动态分配或等待锁。
- 任务间优先使用直接通知、静态槽池和有界队列。大对象通过所有权明确的槽或指针传递。
- 临界区必须短、固定上界，并在注释中说明保护对象和写者/读者关系。
- DMA 缓冲必须说明所在内存域、对齐、Cache 维护和所有权转移。
- 所有等待必须有超时或明确说明永久等待为何安全。

## 错误处理

- 初始化失败返回状态并写入可观察状态；不得静默假定硬件存在。
- 控制输入过期、非有限或越界时进入已定义的安全路径。
- 对外协议和日志结构使用版本号、静态尺寸断言和兼容前缀。

## 格式和提交前检查

```bash
clang-format --dry-run --Werror --style=file <owned-source-files>
cmake --build build/debug
ctest --test-dir build/debug --output-on-failure
git diff --check
```

格式门禁排除自动生成的 MAVLink、上游 VQF 和 Bosch BMI270 配置字节流。
