cmake_minimum_required(VERSION 4.0)

# STM32H743VITX / Cortex-M7 / hard-float ARM GCC firmware.
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)

project(stm32h743_uav_flight_controller C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_definitions(
        USE_HAL_DRIVER
        USE_FLASH_ECC=0
        configUSE_QUEUE_SETS=0
        INCLUDE_xTaskGetIdleTaskHandle=0
        USBD_CLASS_BOS_ENABLED=0
        USBD_USER_REGISTER_CALLBACK=0
        STM32H743xx
        ARM_MATH_CM7
        ARM_MATH_MATRIX_CHECK
        ARM_MATH_ROUNDING
)

set(MCU_OPTIONS
        -mcpu=cortex-m7
        -mthumb
        -mfpu=fpv5-d16
        -mfloat-abi=hard
)

add_compile_options(
        ${MCU_OPTIONS}
        $<$<COMPILE_LANGUAGE:C>:-include>
        $<$<COMPILE_LANGUAGE:C>:stm32h7xx.h>
        $<$<COMPILE_LANGUAGE:CXX>:-include>
        $<$<COMPILE_LANGUAGE:CXX>:stm32h7xx.h>
        $<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>
        -ffunction-sections
        -fdata-sections
        -fno-common
        -fmessage-length=0
        -Wno-address-of-packed-member
)
add_link_options(${MCU_OPTIONS})

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "Build type: Release - Maximum optimization for speed")
    # NaN/Inf guards in sensor and control safety checks must remain effective.
    add_compile_options(-Ofast -fno-finite-math-only)
elseif (CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    message(STATUS "Build type: RelWithDebInfo - Speed optimization + debug info")
    # Ozone 3.x parses DWARF-4 reliably. Keep Ofast so debug builds retain flight code generation.
    add_compile_options(-Ofast -fno-finite-math-only -g3 -gdwarf-4)
elseif (CMAKE_BUILD_TYPE STREQUAL "MinSizeRel")
    message(STATUS "Build type: MinSizeRel - Optimize for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Build type: Debug - Minimal optimization + debug info")
    add_compile_options(-Og -g)
endif ()

# Select the first FreeRTOS GCC port present in this Cube package.
set(FREERTOS_PORT_BASE "${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC")
set(FREERTOS_PORT_CANDIDATES
        "${FREERTOS_PORT_BASE}/ARM_CM7/r0p1"
        "${FREERTOS_PORT_BASE}/ARM_CM7/r0p0"
        "${FREERTOS_PORT_BASE}/ARM_CM7"
        "${FREERTOS_PORT_BASE}/ARM_CM4F"
)
set(FREERTOS_PORT_DIR "")
foreach (PORT_DIR IN LISTS FREERTOS_PORT_CANDIDATES)
    if (EXISTS "${PORT_DIR}/port.c")
        set(FREERTOS_PORT_DIR "${PORT_DIR}")
        break()
    endif ()
endforeach ()
if (NOT FREERTOS_PORT_DIR)
    message(FATAL_ERROR "Cannot find a valid FreeRTOS GCC Cortex-M port.c")
endif ()
message(STATUS "FreeRTOS port: ${FREERTOS_PORT_DIR}")

if (EXISTS "${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Inc/arm_math.h")
    set(CMSIS_DSP_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Inc")
elseif (EXISTS "${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Include/arm_math.h")
    set(CMSIS_DSP_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Include")
else ()
    message(FATAL_ERROR "CMSIS-DSP arm_math.h not found")
endif ()
message(STATUS "CMSIS-DSP include: ${CMSIS_DSP_INCLUDE_DIR}")

include_directories(
        Core/Inc
        FATFS/Target
        FATFS/App
        USB_DEVICE/App
        USB_DEVICE/Target
        Middlewares/Third_Party/SEGGER/Config
        Middlewares/Third_Party/SEGGER/RTT
        Middlewares/Third_Party/SEGGER/SysView
        Drivers/STM32H7xx_HAL_Driver/Inc
        Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
        Middlewares/Third_Party/FreeRTOS/Source/include
        Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
        ${FREERTOS_PORT_DIR}
        Middlewares/Third_Party/FatFs/src
        Middlewares/ST/STM32_USB_Device_Library/Core/Inc
        Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
        ${CMSIS_DSP_INCLUDE_DIR}
        Drivers/CMSIS/Device/ST/STM32H7xx/Include
        Drivers/CMSIS/Include
        Bsp
        Bsp/bsp_mpu
        Bsp/bsp_timestamp
        Bsp/bsp_RTT
        Bsp/bsp_ringbuffer
        Bsp/bsp_iic
        Bsp/bsp_spi
        Bsp/bsp_uart
        Bsp/bsp_memory_section
        Bsp/bsp_utils_seqlock
        Bsp/bsp_usb
        Bsp/bsp_pwm
        Modules
        Modules/modules_BMI088
        Modules/modules_BMI270
        Modules/modules_Message_center
        Modules/modules_SD_Card
        Modules/modules_Algorithm/user_math
        Modules/modules_Algorithm/PID
        Modules/modules_Algorithm/EKF
        Modules/modules_Algorithm/ESKF
        Modules/modules_Algorithm/Biquad_Notch
        Modules/modules_Algorithm/VQF
        Modules/modules_Algorithm/ADRC
        Modules/modules_Algorithm/Filter
        Modules/modules_Algorithm/PT1_Filter
        Modules/modules_optical_flow_sensor/modules_mtf02
        Modules/modules_Remote_sensor/modules_Sbus
        Modules/modules_GPS_sensor/modules_Microvoid_MG
        Modules/modules_altitude_sensor/modules_TFmini_Plus
        Modules/modules_altitude_sensor/modules_SPL06
        Modules/modules_Mavlink/c_library_v2-master
        Modules/modules_Mavlink/mavlink_user
        Modules/modules_Motor/module_pwm_motor
        Application
        Application/DataRouterTask
        Application/App_Data_Comm
        Application/App_Control
        Application/App_Uav_Cmd
        Application/App_Sensor
        Application/App_attitude
)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
        "Core/*.c" "Core/*.cpp" "Core/*.s" "Core/*.S"
        "FATFS/*.c" "FATFS/*.cpp"
        "Middlewares/*.c" "Middlewares/*.cpp" "Middlewares/*.s" "Middlewares/*.S"
        "USB_DEVICE/*.c" "USB_DEVICE/*.cpp"
        "Drivers/*.c" "Drivers/*.cpp" "Drivers/*.s" "Drivers/*.S"
        "Bsp/*.c" "Bsp/*.cpp" "Bsp/*.s" "Bsp/*.S"
        "Modules/*.c" "Modules/*.cpp" "Modules/*.s" "Modules/*.S"
        "Application/*.c" "Application/*.cpp" "Application/*.s" "Application/*.S"
)

# GLOB finds every portable implementation; link exactly the selected FreeRTOS port.
list(FILTER SOURCES EXCLUDE REGEX
        ".*/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM[^/]*/.*port\\.c$"
)
list(APPEND SOURCES "${FREERTOS_PORT_DIR}/port.c")

# Strict warnings apply only to maintained project code, not generated/vendor sources.
file(GLOB_RECURSE PROJECT_OWN_SOURCES CONFIGURE_DEPENDS
        "Application/*.c"
        "Bsp/*.c"
        "Modules/*.c"
)
list(FILTER PROJECT_OWN_SOURCES EXCLUDE REGEX
        ".*/Modules/modules_Mavlink/c_library_v2-master/.*"
)
set_source_files_properties(${PROJECT_OWN_SOURCES} PROPERTIES
        COMPILE_OPTIONS
        "-Wall;-Wextra;-Wshadow;-Wundef;-Wformat=2;-Wdouble-promotion;-Werror=return-type;-Werror=implicit-function-declaration"
)

# VQF uses NaN sentinels internally and must keep IEEE floating-point behavior.
set(VQF_CPP_SOURCES
        "${CMAKE_SOURCE_DIR}/Modules/modules_Algorithm/VQF/vqf.cpp"
        "${CMAKE_SOURCE_DIR}/Modules/modules_Algorithm/VQF/VQF_C.cpp"
)
set_source_files_properties(${VQF_CPP_SOURCES} PROPERTIES
        COMPILE_DEFINITIONS VQF_SINGLE_PRECISION
        COMPILE_OPTIONS "-fno-exceptions;-fno-rtti;-fno-fast-math"
)

set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/STM32H743VITX_FLASH.ld")
if (NOT EXISTS "${LINKER_SCRIPT}")
    message(FATAL_ERROR "Linker script not found: ${LINKER_SCRIPT}")
endif ()
message(STATUS "Linker script: ${LINKER_SCRIPT}")

option(UAV_SYSTEMVIEW_AUTOSTART "调度器启动后自动开始 SystemView 追踪" OFF)

add_executable(${PROJECT_NAME}.elf ${SOURCES})
target_compile_definitions(${PROJECT_NAME}.elf PRIVATE
        UAV_SYSTEMVIEW_AUTOSTART=$<BOOL:${UAV_SYSTEMVIEW_AUTOSTART}>
)
target_link_options(${PROJECT_NAME}.elf PRIVATE
        -mcpu=cortex-m7
        -mthumb
        -mfpu=fpv5-d16
        -mfloat-abi=hard
        -T${LINKER_SCRIPT}
        -Wl,--gc-sections
        -Wl,--print-memory-usage
        -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map
)

find_library(CMSIS_DSP_LIBRARY
        NAMES arm_cortexM7lfsp_math
        PATHS
        "${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib"
        "${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib/GCC"
        NO_DEFAULT_PATH
)
if (NOT CMSIS_DSP_LIBRARY)
    message(FATAL_ERROR
            "CMSIS-DSP library libarm_cortexM7lfsp_math.a not found in Middlewares/ST/ARM/DSP/Lib or Lib/GCC"
    )
endif ()
message(STATUS "CMSIS-DSP library: ${CMSIS_DSP_LIBRARY}")
target_link_libraries(${PROJECT_NAME}.elf PRIVATE ${CMSIS_DSP_LIBRARY} m)

set(HEX_FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex")
set(BIN_FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin")
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}\nBuilding ${BIN_FILE}"
)

option(UAV_ENABLE_QUALITY_GATE "每次默认构建后执行格式、结构和回归测试" ON)
if (UAV_ENABLE_QUALITY_GATE)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)

    file(GLOB_RECURSE PROJECT_OWN_FORMAT_FILES CONFIGURE_DEPENDS
            "Application/*.c" "Application/*.h"
            "Bsp/*.c" "Bsp/*.h"
            "Modules/*.c" "Modules/*.h"
    )
    list(FILTER PROJECT_OWN_FORMAT_FILES EXCLUDE REGEX
            ".*/Modules/modules_Mavlink/c_library_v2-master/.*"
    )
    list(FILTER PROJECT_OWN_FORMAT_FILES EXCLUDE REGEX ".*/Modules/modules_Algorithm/VQF/.*")
    list(FILTER PROJECT_OWN_FORMAT_FILES EXCLUDE REGEX ".*/Modules/modules_BMI270/modules_BMI270_config_file\.h$")

    find_program(CLANG_FORMAT_EXECUTABLE
            NAMES clang-format clang-format-20 clang-format-19 clang-format-18 clang-format-17
    )
    # Fall back to the formatter bundled with the VS Code C/C++ extension on Windows.
    if (NOT CLANG_FORMAT_EXECUTABLE AND DEFINED ENV{USERPROFILE})
        file(GLOB VSCODE_CLANG_FORMAT_CANDIDATES
                "$ENV{USERPROFILE}/.vscode/extensions/ms-vscode.cpptools-*/LLVM/bin/clang-format.exe"
        )
        if (VSCODE_CLANG_FORMAT_CANDIDATES)
            list(SORT VSCODE_CLANG_FORMAT_CANDIDATES COMPARE NATURAL ORDER DESCENDING)
            list(GET VSCODE_CLANG_FORMAT_CANDIDATES 0 CLANG_FORMAT_EXECUTABLE)
        endif ()
    endif ()
    if (NOT CLANG_FORMAT_EXECUTABLE)
        message(FATAL_ERROR "UAV quality gate requires clang-format. Install it or set UAV_ENABLE_QUALITY_GATE=OFF.")
    endif ()

    add_custom_target(format-check
            COMMAND ${CLANG_FORMAT_EXECUTABLE} --dry-run --Werror --style=file ${PROJECT_OWN_FORMAT_FILES}
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            COMMENT "Checking custom C/C++ formatting"
            VERBATIM
    )

    enable_testing()
    set(REGRESSION_TESTS
            sensor_protocol_regression
            eskf_math_regression
            code_quality_check
            imu_calibration_regression
            vqf_integration_regression
            usb_capture_regression
            guided_lm_regression
            quadrotor_visual_regression
            control_pid_config_regression
            spi_bus_regression
            iic_bus_regression
    )
    foreach (TEST_NAME IN LISTS REGRESSION_TESTS)
        add_test(
                NAME ${TEST_NAME}
                COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Tests/${TEST_NAME}.py
        )
    endforeach ()

    add_custom_target(quality-check ALL
            COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
            DEPENDS ${PROJECT_NAME}.elf format-check
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
            COMMENT "Running source quality and regression tests"
            VERBATIM
    )
endif ()
