# 测试方式

## 1) 测试栈与命令

- 主要测试框架：CTest 执行 Python 回归脚本和正式源码的 HAL Mock 测试。
- 断言/Mock 工具：Python `unittest`；为 SPI/I²C 源码测试生成 Mock 头文件。
- 命令：

```powershell
ctest --test-dir cmake-build-release-axis-notch-clean --output-on-failure
ctest --test-dir cmake-build-vqf-ozone --output-on-failure
ctest --test-dir cmake-build-hil-refactor --output-on-failure
python tests/quick-analysis-regression.py
```

## 2) 测试布局

- 固件回归模式：`Tests/*_regression.py`。
- 上位机解析器回归模式：`uav_control_upper_computer/tests/*`。
- 配置方式：CMake 注册固件测试；SPI/I²C 脚本通过测试 HAL 定义编译实际的正式 BSP 源码。

## 3) 测试范围矩阵

| 范围 | 是否覆盖 | 典型目标 | 说明 |
|---|---|---|---|
| 单元测试 | 是 | ESKF 数学、PID 配置、传感器协议 | 可在主机执行 |
| 集成测试 | 部分 | VQF 集成、USB 捕获、日志解析器、BSP 回调路由 | 使用 Mock、记录数据或协议输入 |
| 端到端测试 | 否 | 实际飞行 | 需要硬件、接收机、传感器和电机 |

## 4) Mock 与隔离策略

- SPI/I²C 测试替换 HAL 头文件/函数，直接编译 `bsp_spi.c` / `bsp_iic.c`。
- 解析器测试直接操作二进制日志/上位机数据结构，不连接飞控。
- 共同缺口：主机测试无法验证真实中断延迟、负载下 DMA Cache 一致性、ESC 响应或飞行动力学。

## 5) 覆盖率与质量信号

- 覆盖率工具/阈值：[TODO] 在 CMake 或扫描中均未发现。
- 当前已报告覆盖率：[TODO] 未发现聚合覆盖率报告。
- 已知缺口：没有自动化无桨安全测试，也没有实飞回放/等价性测试。

## 6) 依据

- `CMakeLists.txt`
- `Tests/spi_bus_regression.py`
- `Tests/iic_bus_regression.py`
- `uav_control_upper_computer/tests/quick-analysis-regression.py`
