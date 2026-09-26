# Third-party notices

The root MIT license applies only to project-owned code. The repository also contains the following third-party or generated components, which remain under their own licenses and copyright notices.

| Component | Location | License / notice |
|---|---|---|
| STM32H7 CMSIS device files | `Drivers/CMSIS/Device/ST/STM32H7xx` | ST license in `LICENSE.txt` |
| CMSIS core and DSP | `Drivers/CMSIS`, `Middlewares/ST/ARM/DSP` | Apache-2.0 and Arm/ST notices retained in source and `Drivers/CMSIS/LICENSE.txt` |
| STM32H7 HAL | `Drivers/STM32H7xx_HAL_Driver` | ST license in `LICENSE.txt` |
| STM32 USB Device Library | `Middlewares/ST/STM32_USB_Device_Library` | ST license in `LICENSE.txt` |
| FreeRTOS kernel/CMSIS wrapper | `Middlewares/Third_Party/FreeRTOS` | MIT; full text in `Source/LICENSE` |
| FatFs R0.12c | `Middlewares/Third_Party/FatFs` | ChaN permissive license retained in `ff.c` and related files |
| VQF | `Modules/modules_Algorithm/VQF` | MIT; `LICENSE.MIT.txt`; upstream commit recorded in `UPSTREAM.md` |
| MAVLink generated C library | `Modules/modules_Mavlink/c_library_v2-master` | Generated protocol headers and embedded upstream notices; SHA-256 helper retains its BSD-style notice |
| SEGGER RTT and SystemView target sources | `Middlewares/Third_Party/SEGGER` | SEGGER redistribution conditions retained verbatim in each source file |
| Bosch BMI270 configuration blob | `Modules/modules_BMI270/modules_BMI270_config_file.h` | Bosch copyright and source references retained in the file |

This summary is provided for convenience and does not replace the license text embedded in or shipped alongside each component. When redistributing a modified repository or binary, review the applicable upstream terms and preserve all required notices.
