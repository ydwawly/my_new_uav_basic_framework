# STM32H743 UAV Flight Controller

面向四旋翼验证平台的 STM32H743 飞控固件。项目采用 FreeRTOS，包含双 IMU、VQF 姿态估计、15 状态 ESKF 导航、级联姿态/角速度控制、光流与测距融合、SBUS 遥控、SD 黑匣子、USB/MAVLink 和 SEGGER SystemView 支持。

> 安全提示：当前正式配置会初始化电机 PWM。烧录、调试和首次验证必须拆除螺旋桨。本项目不是经过适航或功能安全认证的产品固件。

## 设计重点

- 1 kHz IMU 数据就绪事件驱动姿态任务，姿态结果再通知控制任务，避免控制任务轮询和重复使用旧反馈。
- VQF 为真实传感器模式的姿态控制源；并行 ESKF 提供 NED 位置、速度和测距/光流融合状态。
- MTF02、TFmini、SBUS 等 UART 中断只搬运原始数据并记录时间戳，协议解析统一在传感器任务上下文完成。
- DMA 缓冲区、D-Cache 维护、DTCM/ITCM 热数据与热代码放置均显式管理。
- SD 日志使用固定槽池和指针队列，避免在 FreeRTOS 队列临界区复制大块日志响应。
- 自有性能探针已移除；运行时分析使用 SystemView，业务故障计数、时间戳和数据新鲜度检查继续保留。

## 快速构建

要求 CMake 4.0+、Ninja、Python 3、`arm-none-eabi-gcc` 以及 clang-format。

```bash
cmake -S . -B build/debug -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/debug
ctest --test-dir build/debug --output-on-failure
```

生成物位于构建目录：

- `stm32h743_uav_flight_controller.elf`
- `stm32h743_uav_flight_controller.hex`
- `stm32h743_uav_flight_controller.bin`

仓库不提交这些二进制文件。完整环境、烧录和 SystemView 方法见 [构建与调试](Docs/BUILD_AND_DEBUG.md)。

## 目录

```text
Application/   飞行任务、控制、姿态估计与通信服务
Bsp/           SPI/I2C/UART/USB/PWM/时间戳等板级抽象
Modules/       传感器、算法、消息中心、电机与 SD 日志
Core/          STM32CubeMX 生成的启动、外设和中断代码
Tests/         11 项主机回归与源码契约测试
Docs/          架构、硬件、规范、安全与验证资料
```

## 文档

- [系统架构与数据流](Docs/ARCHITECTURE.md)
- [硬件、接口与引脚](Docs/HARDWARE.md)
- [构建、烧录与 SystemView](Docs/BUILD_AND_DEBUG.md)
- [安全边界](Docs/SAFETY.md)
- [代码规范](Docs/CODE_STYLE.md)
- [性能与复现方法](Docs/PERFORMANCE.md)
- [发布验证记录](Docs/RELEASE_VALIDATION.md)

## 当前验证边界

当前候选版本已通过三种 ARM 构建配置、SystemView 自动启动开关构建、格式检查与 11 项主机回归测试。硬件无桨回归和实飞仍属于发布前人工验证项，详见验证记录。

## License

项目自有代码采用 [MIT License](LICENSE)。ST、Arm、FreeRTOS、FatFs、VQF、MAVLink 与 SEGGER 等第三方组件遵循其各自许可证，详见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

## English summary

This repository is a portfolio-ready STM32H743 quadrotor flight-controller firmware based on FreeRTOS. It integrates event-driven IMU-to-attitude-to-control scheduling, VQF attitude estimation, a parallel 15-state ESKF for navigation, optical-flow/range fusion, SBUS input, SD flight logging, USB/MAVLink transport, cache-aware DMA handling, and optional SEGGER SystemView tracing. The repository contains source and reproducible build/tests only; generated firmware binaries are intentionally excluded.
