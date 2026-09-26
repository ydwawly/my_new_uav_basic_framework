"""Compile the production SPI bus wrapper against HAL mocks and exercise its state machine."""

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SPI_DIR = ROOT / "Bsp" / "bsp_spi"
SPI_SOURCE = SPI_DIR / "bsp_spi.c"

MOCK_SPI_H = r"""
#ifndef SPI_H
#define SPI_H

#include <stdint.h>

typedef struct
{
    uint32_t tag;
} SPI_HandleTypeDef;

typedef enum
{
    HAL_OK = 0U,
    HAL_ERROR = 1U,
    HAL_BUSY = 2U,
    HAL_TIMEOUT = 3U
} HAL_StatusTypeDef;

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *handle, uint8_t *data, uint16_t length, uint32_t timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *handle, uint8_t *data, uint16_t length, uint32_t timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *handle, uint8_t *tx_data, uint8_t *rx_data,
                                          uint16_t length, uint32_t timeout);
HAL_StatusTypeDef HAL_SPI_Transmit_IT(SPI_HandleTypeDef *handle, uint8_t *data, uint16_t length);
HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef *handle, uint8_t *data, uint16_t length);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *handle, uint8_t *tx_data, uint8_t *rx_data,
                                             uint16_t length);
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *handle, uint8_t *data, uint16_t length);
HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef *handle, uint8_t *data, uint16_t length);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *handle, uint8_t *tx_data, uint8_t *rx_data,
                                              uint16_t length);

#endif
"""

MOCK_GPIO_H = r"""
#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>

typedef struct
{
    uint32_t tag;
} GPIO_TypeDef;

typedef enum
{
    GPIO_PIN_RESET = 0U,
    GPIO_PIN_SET = 1U
} GPIO_PinState;

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);

#endif
"""

MOCK_FREERTOS_H = r"""
#ifndef FREERTOS_H
#define FREERTOS_H

#include <stddef.h>

void *pvPortMalloc(size_t size);

#endif
"""

HARNESS_C = r"""
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include "bsp_spi.h"

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *handle);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *handle);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *handle);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *handle);

typedef enum
{
    HAL_CALL_NONE = 0,
    HAL_CALL_TX_BLOCK,
    HAL_CALL_RX_BLOCK,
    HAL_CALL_TXRX_BLOCK,
    HAL_CALL_TX_IT,
    HAL_CALL_RX_IT,
    HAL_CALL_TXRX_IT,
    HAL_CALL_TX_DMA,
    HAL_CALL_RX_DMA,
    HAL_CALL_TXRX_DMA
} HalCall;

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState state;
} PinEvent;

static HAL_StatusTypeDef next_status;
static HalCall hal_call;
static PinEvent pin_events[16];
static size_t pin_event_count;
static uint32_t hal_timeout;
static SPIInstance *callback_restart_instance;
static HAL_StatusTypeDef callback_restart_status;
static SPI_Event_e callback_event;
static uint8_t callback_busy;
static uint8_t callback_should_restart;
static size_t callback_count;
static uint8_t callback_data[2] = {0x5AU, 0xA5U};

void *pvPortMalloc(size_t size)
{
    return calloc(1U, size);
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    assert(pin_event_count < (sizeof(pin_events) / sizeof(pin_events[0])));
    pin_events[pin_event_count++] = (PinEvent){port, pin, state};
}

static HAL_StatusTypeDef RecordHalCall(HalCall call, uint32_t timeout)
{
    hal_call = call;
    hal_timeout = timeout;
    return next_status;
}

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *handle, uint8_t *data, uint16_t length, uint32_t timeout)
{
    assert((handle != NULL) && (data != NULL) && (length != 0U));
    return RecordHalCall(HAL_CALL_TX_BLOCK, timeout);
}

HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *handle, uint8_t *data, uint16_t length, uint32_t timeout)
{
    assert((handle != NULL) && (data != NULL) && (length != 0U));
    return RecordHalCall(HAL_CALL_RX_BLOCK, timeout);
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *handle, uint8_t *tx_data, uint8_t *rx_data,
                                          uint16_t length, uint32_t timeout)
{
    assert((handle != NULL) && (tx_data != NULL) && (rx_data != NULL) && (length != 0U));
    return RecordHalCall(HAL_CALL_TXRX_BLOCK, timeout);
}

HAL_StatusTypeDef HAL_SPI_Transmit_IT(SPI_HandleTypeDef *handle, uint8_t *data, uint16_t length)
{
    assert((handle != NULL) && (data != NULL) && (length != 0U));
    return RecordHalCall(HAL_CALL_TX_IT, 0U);
}

HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef *handle, uint8_t *data, uint16_t length)
{
    assert((handle != NULL) && (data != NULL) && (length != 0U));
    return RecordHalCall(HAL_CALL_RX_IT, 0U);
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *handle, uint8_t *tx_data, uint8_t *rx_data,
                                             uint16_t length)
{
    assert((handle != NULL) && (tx_data != NULL) && (rx_data != NULL) && (length != 0U));
    return RecordHalCall(HAL_CALL_TXRX_IT, 0U);
}

HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *handle, uint8_t *data, uint16_t length)
{
    assert((handle != NULL) && (data != NULL) && (length != 0U));
    return RecordHalCall(HAL_CALL_TX_DMA, 0U);
}

HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef *handle, uint8_t *data, uint16_t length)
{
    assert((handle != NULL) && (data != NULL) && (length != 0U));
    return RecordHalCall(HAL_CALL_RX_DMA, 0U);
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *handle, uint8_t *tx_data, uint8_t *rx_data,
                                              uint16_t length)
{
    assert((handle != NULL) && (tx_data != NULL) && (rx_data != NULL) && (length != 0U));
    return RecordHalCall(HAL_CALL_TXRX_DMA, 0U);
}

static void TestCallback(SPIInstance *instance, SPI_Event_e event)
{
    callback_count++;
    callback_event = event;
    callback_busy = instance->is_busy;
    if (callback_should_restart != 0U)
    {
        callback_restart_status = SPITransmit(callback_restart_instance, callback_data, sizeof(callback_data));
    }
}

static void ResetTrace(void)
{
    next_status = HAL_OK;
    hal_call = HAL_CALL_NONE;
    pin_event_count = 0U;
    hal_timeout = 0U;
    callback_restart_instance = NULL;
    callback_restart_status = HAL_ERROR;
    callback_event = SPI_EVENT_ERROR;
    callback_busy = 1U;
    callback_should_restart = 0U;
    callback_count = 0U;
}

static void AssertPin(size_t index, GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    assert(index < pin_event_count);
    assert(pin_events[index].port == port);
    assert(pin_events[index].pin == pin);
    assert(pin_events[index].state == state);
}

int main(void)
{
    SPI_HandleTypeDef bus_a = {1U};
    SPI_HandleTypeDef bus_b = {2U};
    SPI_HandleTypeDef unknown_bus = {3U};
    GPIO_TypeDef port_a = {1U};
    GPIO_TypeDef port_b = {2U};
    uint8_t tx[3] = {1U, 2U, 3U};
    uint8_t rx[3] = {0U};

    const SPI_Init_Config_s config_a = {&bus_a, &port_a, 1U, SPI_DMA_MODE, TestCallback, NULL};
    const SPI_Init_Config_s config_b = {&bus_a, &port_a, 2U, SPI_DMA_MODE, NULL, NULL};
    const SPI_Init_Config_s config_c = {&bus_b, &port_b, 3U, SPI_BLOCK_MODE, TestCallback, NULL};

    assert(SPIRegister(NULL) == NULL);
    SPIInstance *device_a = SPIRegister(&config_a);
    SPIInstance *device_b = SPIRegister(&config_b);
    SPIInstance *device_c = SPIRegister(&config_c);
    assert((device_a != NULL) && (device_b != NULL) && (device_c != NULL));
    assert(pin_event_count == 3U);
    AssertPin(0U, &port_a, 1U, GPIO_PIN_SET);
    AssertPin(1U, &port_a, 2U, GPIO_PIN_SET);
    AssertPin(2U, &port_b, 3U, GPIO_PIN_SET);
    assert(SPIRegister(&config_a) == device_a);
    assert(pin_event_count == 3U);

    assert(SPITransmit(NULL, tx, sizeof(tx)) == HAL_ERROR);
    assert(SPITransmit(device_a, NULL, sizeof(tx)) == HAL_ERROR);
    assert(SPIRecv(device_a, rx, 0U) == HAL_ERROR);
    assert(SPITransRecv(device_a, tx, NULL, sizeof(tx)) == HAL_ERROR);

    ResetTrace();
    assert(SPITransmit(device_a, tx, sizeof(tx)) == HAL_OK);
    assert((hal_call == HAL_CALL_TX_DMA) && (device_a->is_busy == 1U));
    assert(pin_event_count == 1U);
    AssertPin(0U, &port_a, 1U, GPIO_PIN_RESET);
    assert(SPITransmit(device_b, tx, sizeof(tx)) == HAL_BUSY);
    assert(pin_event_count == 1U);

    callback_restart_instance = device_b;
    callback_should_restart = 1U;
    HAL_SPI_TxCpltCallback(&bus_a);
    assert((callback_count == 1U) && (callback_event == SPI_EVENT_TX_CPLT));
    assert((callback_busy == 0U) && (callback_restart_status == HAL_OK));
    assert((device_a->is_busy == 0U) && (device_b->is_busy == 1U));
    assert(pin_event_count == 3U);
    AssertPin(1U, &port_a, 1U, GPIO_PIN_SET);
    AssertPin(2U, &port_a, 2U, GPIO_PIN_RESET);
    HAL_SPI_TxCpltCallback(&bus_a);
    assert((device_b->is_busy == 0U) && (pin_event_count == 4U));
    AssertPin(3U, &port_a, 2U, GPIO_PIN_SET);

    ResetTrace();
    next_status = HAL_TIMEOUT;
    assert(SPIRecv(device_a, rx, sizeof(rx)) == HAL_TIMEOUT);
    assert((hal_call == HAL_CALL_RX_DMA) && (device_a->is_busy == 0U));
    assert(pin_event_count == 2U);
    AssertPin(0U, &port_a, 1U, GPIO_PIN_RESET);
    AssertPin(1U, &port_a, 1U, GPIO_PIN_SET);

    ResetTrace();
    assert(SPIRecv(device_b, rx, sizeof(rx)) == HAL_OK);
    assert(device_b->is_busy == 1U);
    HAL_SPI_ErrorCallback(&bus_a);
    assert((device_b->is_busy == 0U) && (pin_event_count == 2U));
    AssertPin(1U, &port_a, 2U, GPIO_PIN_SET);

    assert(SPISetMode(device_a, SPI_IT_MODE) == HAL_OK);
    ResetTrace();
    assert(SPITransRecv(device_a, tx, rx, sizeof(tx)) == HAL_OK);
    assert((hal_call == HAL_CALL_TXRX_IT) && (device_a->is_busy == 1U));
    assert(SPISetMode(device_a, SPI_BLOCK_MODE) == HAL_BUSY);
    HAL_SPI_ErrorCallback(&bus_a);
    assert((callback_count == 1U) && (callback_event == SPI_EVENT_ERROR));
    assert((callback_busy == 0U) && (device_a->is_busy == 0U));
    assert(SPISetMode(device_a, (SPI_TXRX_MODE_e)99) == HAL_ERROR);
    assert(device_a->spi_work_mode == SPI_IT_MODE);

    ResetTrace();
    next_status = HAL_TIMEOUT;
    assert(SPITransRecv(device_c, tx, rx, sizeof(tx)) == HAL_TIMEOUT);
    assert((hal_call == HAL_CALL_TXRX_BLOCK) && (hal_timeout == SPI_BLOCK_TIMEOUT));
    assert((device_c->is_busy == 0U) && (pin_event_count == 2U));
    AssertPin(0U, &port_b, 3U, GPIO_PIN_RESET);
    AssertPin(1U, &port_b, 3U, GPIO_PIN_SET);
    HAL_SPI_TxRxCpltCallback(&bus_b);
    assert(callback_count == 0U);
    HAL_SPI_RxCpltCallback(&unknown_bus);
    assert(callback_count == 0U);

    return 0;
}
"""


def find_host_gcc() -> Path | None:
    configured = os.environ.get("HOST_CC")
    if configured and Path(configured).is_file():
        return Path(configured)

    on_path = shutil.which("gcc")
    if on_path:
        return Path(on_path)

    candidates: list[Path] = []
    for root in (Path("D:/applications/Clion"), Path.home() / "AppData/Local/JetBrains/Toolbox/apps/CLion"):
        if root.exists():
            candidates.extend(root.glob("**/bin/mingw/bin/gcc.exe"))
    return sorted(candidates)[-1] if candidates else None


def find_vs_devcmd() -> Path | None:
    candidates: list[Path] = []
    for root_name in (os.environ.get("ProgramFiles"), os.environ.get("ProgramFiles(x86)")):
        if not root_name:
            continue
        visual_studio = Path(root_name) / "Microsoft Visual Studio"
        if visual_studio.exists():
            candidates.extend(visual_studio.glob("*/*/Common7/Tools/VsDevCmd.bat"))
    return sorted(candidates)[-1] if candidates else None


def verify_source_contract() -> None:
    source = SPI_SOURCE.read_text(encoding="utf-8")
    header = (SPI_DIR / "bsp_spi.h").read_text(encoding="utf-8")
    for dead_field in ("pending_event", "tx_buffer", "rx_buffer", "tx_size", "rx_size"):
        assert dead_field not in source + header
    assert source.index("SPI_ReleaseBus(bus);") < source.index("owner->callback(owner, event);")
    assert "if (status != HAL_OK)" in source
    assert "SPI_ReleaseBus(bus);" in source[source.index("if (status != HAL_OK)") :]


def main() -> None:
    verify_source_contract()
    vs_devcmd = find_vs_devcmd()
    gcc = find_host_gcc()
    if (vs_devcmd is None) and (gcc is None):
        print("SPI source contract passed; host C compiler unavailable, executable HAL-mock test skipped.")
        return

    with tempfile.TemporaryDirectory(prefix="uav-spi-regression-") as temp_name:
        temp = Path(temp_name)
        (temp / "spi.h").write_text(MOCK_SPI_H, encoding="utf-8")
        (temp / "gpio.h").write_text(MOCK_GPIO_H, encoding="utf-8")
        (temp / "FreeRTOS.h").write_text(MOCK_FREERTOS_H, encoding="utf-8")
        harness = temp / "spi_harness.c"
        harness.write_text(HARNESS_C, encoding="utf-8")
        executable = temp / "spi_harness.exe"

        if vs_devcmd is not None:
            build_script = temp / "build_spi_harness.cmd"
            build_script.write_text(
                "@echo off\n"
                f'call "{vs_devcmd}" -no_logo -arch=x64 -host_arch=x64 >nul\n'
                "if errorlevel 1 exit /b %errorlevel%\n"
                f'cl /nologo /std:c11 /utf-8 /W4 /WX /I"{temp}" /I"{SPI_DIR}" '
                f'"{SPI_SOURCE}" "{harness}" /Fe:"{executable}"\n',
                encoding="utf-8",
            )
            compile_result = subprocess.run(
                ["cmd.exe", "/d", "/c", str(build_script)], capture_output=True, text=True, cwd=temp
            )
        else:
            assert gcc is not None
            compile_result = subprocess.run(
                [
                    str(gcc),
                    "-std=c11",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    f"-I{temp}",
                    f"-I{SPI_DIR}",
                    str(SPI_SOURCE),
                    str(harness),
                    "-o",
                    str(executable),
                ],
                capture_output=True,
                text=True,
            )
        if compile_result.returncode != 0:
            raise RuntimeError(
                f"SPI HAL-mock build failed with exit code {compile_result.returncode}:\n"
                f"{compile_result.stdout}{compile_result.stderr}"
            )
        subprocess.run([str(executable)], check=True)

    print("SPI regression passed: CS, bus ownership, start failures and ISR completion ordering verified.")


if __name__ == "__main__":
    main()
