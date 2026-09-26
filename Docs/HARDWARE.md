# 硬件、接口与引脚

## 主控与时钟

- MCU：STM32H743VIT6，Cortex-M7。
- CPU/SYSCLK：400 MHz；HCLK：200 MHz。
- I-Cache/D-Cache：各 16 KiB；ITCM/DTCM 用于实时热代码和状态。
- 调试：SWD；SystemView 数据通过 SEGGER RTT 传输。

## 主要传感器与接口

| 设备 | 用途 | 接口 | 关键引脚 |
|---|---|---|---|
| BMI088 | 主 IMU | SPI2 + DMA | SCK PD3、MISO PC2、MOSI PC3、ACC CS PD4、GYRO CS PD5、INT PC14/PC15 |
| BMI270 | 备用/对照 IMU | SPI3 + DMA | SCK PB3、MISO PB4、MOSI PD6、CS PA15、INT PB7 |
| SPL06 | 气压计 | I2C2 + DMA | SCL PB10、SDA PB11、INT PD0 |
| MTF02 | 光流/测距 | UART DMA | 具体串口绑定以驱动注册配置和 `.ioc` 为准 |
| TFmini Plus | 激光测距 | UART DMA | 具体串口绑定以驱动注册配置和 `.ioc` 为准 |
| SBUS 接收机 | 遥控输入 | UART DMA | 具体串口绑定以驱动注册配置和 `.ioc` 为准 |
| Microvoid MG | GPS | UART DMA | 默认可编译关闭，具体绑定以驱动配置为准 |
| microSD | 黑匣子 | SDMMC1 4-bit | PC8–PC12、PD2 |
| USB CDC | MAVLink/日志下载 | USB FS | PA11、PA12 |
| CAN | 扩展总线 | FDCAN1 | PB8、PB9 |

## 电机输出

控制器中的电机顺序为左前、右前、右后、左后，实际映射如下：

| 电机索引 | 位置 | 定时器通道 | 引脚 |
|---:|---|---|---|
| 0 | 左前 | TIM1 CH4 | PE14 |
| 1 | 右前 | TIM1 CH3 | PE13 |
| 2 | 右后 | TIM1 CH2 | PE11 |
| 3 | 左后 | TIM1 CH1 | PE9 |

修改机架、ESC 接线或电机顺序后，必须重新验证 Mixer 符号和旋向。PWM 的周期、最小、怠速和最大脉宽由 `module_pwm_motor.h` 定义。

## DMA 与 Cache 约束

- DMA 缓冲必须位于 DMA 可访问的 SRAM 域，不能放入 DTCM。
- Cacheable 缓冲在 DMA 发送前 Clean，在 DMA 接收完成后 Invalidate；地址和长度按 32 字节 Cache Line 对齐处理。
- 外设回调只发布已经完成的数据，消费者不得在 DMA 仍拥有缓冲时修改它。
- 链接脚本和 MPU 配置属于硬件契约，修改内存段后必须重新审查 DMA 可达性和一致性。

权威引脚来源是 `stm32h743_uav_flight_controller.ioc`、生成的 `Core/Inc/main.h` 以及各模块注册配置；本文用于快速阅读，不替代原理图。
