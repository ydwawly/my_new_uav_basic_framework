# 构建、烧录与 SystemView

## 工具要求

- CMake 4.0 或更高版本
- Ninja
- Arm GNU Toolchain（已验证 `arm-none-eabi-gcc 13.3.1`）
- Python 3
- clang-format（质量门禁使用）
- 可选：OpenOCD 或 J-Link/Ozone、SEGGER SystemView

确保 `arm-none-eabi-gcc`、`cmake`、`ninja` 和 `clang-format` 位于 `PATH`。

## 构建配置

```bash
# Debug + 格式检查 + 11 项回归测试
cmake -S . -B build/debug -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DUAV_ENABLE_QUALITY_GATE=ON \
  -DUAV_SYSTEMVIEW_AUTOSTART=OFF
cmake --build build/debug

# Release
cmake -S . -B build/release -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DUAV_ENABLE_QUALITY_GATE=OFF
cmake --build build/release

# 带符号的性能分析版本，启动后自动记录 SystemView
cmake -S . -B build/systemview -G Ninja \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DUAV_ENABLE_QUALITY_GATE=OFF \
  -DUAV_SYSTEMVIEW_AUTOSTART=ON
cmake --build build/systemview
```

手工运行测试：

```bash
ctest --test-dir build/debug --output-on-failure
```

## 烧录

烧录前拆除螺旋桨。使用任一支持 STM32H743 的工具加载 ELF/HEX：

```bash
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
  -c "program build/debug/stm32h743_uav_flight_controller.elf verify reset exit"
```

J-Link/Ozone 用户应新建本机工程并选择 `STM32H743VI`、SWD 和生成的 ELF。仓库不保存探针序列号、本机绝对路径或个人 Ozone 工程。

## SystemView

固件始终保留 FreeRTOS trace facility 和 SEGGER RTT/SystemView 源码；`configGENERATE_RUN_TIME_STATS` 关闭，不使用自定义性能任务。

1. 构建 `RelWithDebInfo`，建议先用 `UAV_SYSTEMVIEW_AUTOSTART=OFF`。
2. 烧录 ELF 并运行目标。
3. 在 SystemView 中选择 J-Link、STM32H743、目标频率 400 MHz，并加载同一 ELF。
4. 连接 RTT 后手工开始记录；若使用自动启动构建，调度器完成应用初始化后会调用 `SEGGER_SYSVIEW_Start()`。
5. 至少采集 30 s 稳态窗口和一次通信/SD 满载窗口，保存分析表而不是提交原始 `.SVDat`。

重点观察：ISR 时长与嵌套、IMU→Attitude→Control 链路、任务执行时间/周期/抖动、CPU Load、阻塞时间、优先级反转和栈余量。

## 常见问题

- CMake 找不到 clang-format：安装 clang-format，或仅在临时本机构建中设置 `UAV_ENABLE_QUALITY_GATE=OFF`；提交前必须重新打开门禁。
- VQF 在 Release 下异常：不要移除 VQF 源文件上的 `-fno-fast-math`，其内部依赖 NaN 哨兵语义。
- DMA 数据偶发陈旧：检查缓冲所在内存域、32 字节对齐、Clean/Invalidate 时序和所有权。
