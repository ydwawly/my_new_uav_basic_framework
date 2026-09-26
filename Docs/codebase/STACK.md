# 技术栈

## 1) 运行时概览

| 领域 | 内容 | 依据 |
|---|---|---|
| 主要语言 | C11；构建中启用少量 C++17 | `CMakeLists.txt` |
| 运行时与版本 | STM32H743 Cortex-M7 固件，CPU 配置为 400 MHz | `CMakeLists.txt`、`Core/Src/main.c` |
| 包管理 | 无；源码与第三方中间件直接由 CMake 构建 | `CMakeLists.txt` |
| 模块/构建系统 | CMake + ARM GNU Toolchain（`arm-none-eabi-gcc`） | `CMakeLists.txt` |

## 2) 正式框架与依赖

| 依赖 | 版本 | 系统职责 | 依据 |
|---|---|---|---|
| STM32 HAL/CMSIS | Cube 提供 | 外设、中断与 Cache 接口 | `Core/Src/main.c`、`Drivers/` |
| FreeRTOS | Cube 提供 | 任务、通知、队列和临界区 | `Core/Src/freertos.c`、`Middlewares/Third_Party/FreeRTOS/` |
| FatFS | Cube 提供 | SD 卡文件系统 | `Modules/modules_SD_Card/modules_SD_Card.c`、`FATFS/` |
| MAVLink C 库 | 生成的 C 头文件 | USB 遥测、HIL 与日志传输协议 | `Application/App_Data_Comm/`、`Modules/modules_Mavlink/` |
| VQF | 工程内置 C 实现 | 控制使用的 6D 姿态与陀螺零偏估计 | `Modules/modules_Algorithm/VQF/VQF_C.h`、`Application/App_attitude/App_attitude.c` |
| Nav ESKF | 工程模块 | 并行导航/高度诊断估计 | `Application/App_attitude/App_attitude.c`、`Modules/modules_Algorithm/` |

## 3) 开发工具链

| 工具 | 用途 | 依据 |
|---|---|---|
| CMake/Ninja | 交叉编译与测试目标 | `CMakeLists.txt`、`cmake-build-release-axis-notch-clean/` |
| clang-format | 维护 C/C++ 文件的格式门禁 | `.clang-format`、`CMakeLists.txt` |
| CTest + Python | 主机回归测试 | `CMakeLists.txt`、`Tests/` |
| SEGGER Ozone | DWARF/J-Link 变量观察 | `Firmware/MANIFEST.md` |

## 4) 常用命令

```powershell
cmake --build cmake-build-release-axis-notch-clean --parallel 8
ctest --test-dir cmake-build-release-axis-notch-clean --output-on-failure
cmake --build cmake-build-vqf-ozone --parallel 8
python tests/quick-analysis-regression.py
```

最后一条命令应在 `uav_control_upper_computer/` 下执行。

## 5) 环境与配置

- 配置来源：`Application/` 下的编译期头文件、`Core/` 下 Cube 生成外设文件，以及 `CMakeLists.txt`。
- 必需环境变量：扫描未发现。[TODO] 后续增加部署工具时重新检查。
- 运行时约束：DMA 缓冲使用 STM32H7 专用内存段；FreeRTOS 调度器启动后才执行应用初始化。

## 6) 依据

- `CMakeLists.txt`
- `Core/Src/main.c`
- `Core/Src/freertos.c`
- `Application/App_SystemInit.c`
