"""Compile the production I2C wrapper against HAL mocks and exercise its bus state machine."""

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
IIC_DIR = ROOT / "Bsp" / "bsp_iic"
IIC_SOURCE = IIC_DIR / "bsp_iic.c"

MOCK_I2C_H = r"""
#ifndef I2C_H
#define I2C_H

#include <stddef.h>
#include <stdint.h>

#define I2C_MEMADD_SIZE_8BIT  1U
#define I2C_MEMADD_SIZE_16BIT 2U

typedef struct
{
    uint32_t tag;
} I2C_HandleTypeDef;

typedef enum
{
    HAL_OK = 0U,
    HAL_ERROR = 1U,
    HAL_BUSY = 2U,
    HAL_TIMEOUT = 3U
} HAL_StatusTypeDef;

HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *handle, uint16_t address, uint32_t trials,
                                         uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *handle, uint16_t address, uint8_t *data,
                                          uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *handle, uint16_t address, uint8_t *data,
                                         uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *handle, uint16_t address, uint8_t *data,
                                             uint16_t size);
HAL_StatusTypeDef HAL_I2C_Master_Receive_IT(I2C_HandleTypeDef *handle, uint16_t address, uint8_t *data,
                                            uint16_t size);
HAL_StatusTypeDef HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *handle, uint16_t address, uint8_t *data,
                                              uint16_t size);
HAL_StatusTypeDef HAL_I2C_Master_Receive_DMA(I2C_HandleTypeDef *handle, uint16_t address, uint8_t *data,
                                             uint16_t size);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *handle, uint16_t address, uint16_t memory_address,
                                     uint16_t memory_address_size, uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *handle, uint16_t address, uint16_t memory_address,
                                    uint16_t memory_address_size, uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef *handle, uint16_t address, uint16_t memory_address,
                                        uint16_t memory_address_size, uint8_t *data, uint16_t size);
HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *handle, uint16_t address, uint16_t memory_address,
                                       uint16_t memory_address_size, uint8_t *data, uint16_t size);
HAL_StatusTypeDef HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *handle, uint16_t address, uint16_t memory_address,
                                         uint16_t memory_address_size, uint8_t *data, uint16_t size);
HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *handle, uint16_t address, uint16_t memory_address,
                                        uint16_t memory_address_size, uint8_t *data, uint16_t size);

#endif
"""

HARNESS_C = r"""
#include <assert.h>
#include <stddef.h>
#include <stdint.h>

#include "bsp_iic.h"

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *handle);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *handle);
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *handle);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *handle);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *handle);

typedef enum
{
    CALL_NONE = 0,
    CALL_READY,
    CALL_MASTER_TX_BLOCK,
    CALL_MASTER_RX_BLOCK,
    CALL_MASTER_TX_IT,
    CALL_MASTER_RX_IT,
    CALL_MASTER_TX_DMA,
    CALL_MASTER_RX_DMA,
    CALL_MEMORY_TX_BLOCK,
    CALL_MEMORY_RX_BLOCK,
    CALL_MEMORY_TX_IT,
    CALL_MEMORY_RX_IT,
    CALL_MEMORY_TX_DMA,
    CALL_MEMORY_RX_DMA
} HalCall;

static HAL_StatusTypeDef next_status;
static HalCall hal_call;
static uint16_t hal_address;
static uint16_t hal_memory_address;
static uint16_t hal_memory_address_size;
static uint32_t hal_trials;
static uint32_t hal_timeout;
static size_t hal_call_count;

static IICInstance *callback_restart_instance;
static HAL_StatusTypeDef callback_restart_status;
static IIC_Event_e callback_event;
static uint8_t callback_busy;
static uint8_t callback_should_restart;
static size_t callback_count;
static uint8_t callback_data[2] = {0x5AU, 0xA5U};

static HAL_StatusTypeDef RecordCall(HalCall call, uint16_t address, uint16_t memory_address,
                                    uint16_t memory_address_size, uint32_t trials, uint32_t timeout)
{
    hal_call = call;
    hal_address = address;
    hal_memory_address = memory_address;
    hal_memory_address_size = memory_address_size;
    hal_trials = trials;
    hal_timeout = timeout;
    hal_call_count++;
    return next_status;
}

HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *handle, uint16_t address, uint32_t trials,
                                         uint32_t timeout)
{
    assert(handle != NULL);
    return RecordCall(CALL_READY, address, 0U, 0U, trials, timeout);
}

#define DEFINE_MASTER_HAL(name, call_id)                                                                    \
    HAL_StatusTypeDef name(I2C_HandleTypeDef *handle, uint16_t address, uint8_t *data, uint16_t size)        \
    {                                                                                                        \
        assert((handle != NULL) && (data != NULL) && (size != 0U));                                         \
        return RecordCall(call_id, address, 0U, 0U, 0U, 0U);                                                \
    }

#define DEFINE_MASTER_BLOCKING_HAL(name, call_id)                                                           \
    HAL_StatusTypeDef name(I2C_HandleTypeDef *handle, uint16_t address, uint8_t *data, uint16_t size,        \
                           uint32_t timeout)                                                                 \
    {                                                                                                        \
        assert((handle != NULL) && (data != NULL) && (size != 0U));                                         \
        return RecordCall(call_id, address, 0U, 0U, 0U, timeout);                                           \
    }

#define DEFINE_MEMORY_HAL(name, call_id)                                                                    \
    HAL_StatusTypeDef name(I2C_HandleTypeDef *handle, uint16_t address, uint16_t memory_address,             \
                           uint16_t memory_address_size, uint8_t *data, uint16_t size)                        \
    {                                                                                                        \
        assert((handle != NULL) && (data != NULL) && (size != 0U));                                         \
        return RecordCall(call_id, address, memory_address, memory_address_size, 0U, 0U);                    \
    }

#define DEFINE_MEMORY_BLOCKING_HAL(name, call_id)                                                           \
    HAL_StatusTypeDef name(I2C_HandleTypeDef *handle, uint16_t address, uint16_t memory_address,             \
                           uint16_t memory_address_size, uint8_t *data, uint16_t size, uint32_t timeout)      \
    {                                                                                                        \
        assert((handle != NULL) && (data != NULL) && (size != 0U));                                         \
        return RecordCall(call_id, address, memory_address, memory_address_size, 0U, timeout);               \
    }

DEFINE_MASTER_BLOCKING_HAL(HAL_I2C_Master_Transmit, CALL_MASTER_TX_BLOCK)
DEFINE_MASTER_BLOCKING_HAL(HAL_I2C_Master_Receive, CALL_MASTER_RX_BLOCK)
DEFINE_MASTER_HAL(HAL_I2C_Master_Transmit_IT, CALL_MASTER_TX_IT)
DEFINE_MASTER_HAL(HAL_I2C_Master_Receive_IT, CALL_MASTER_RX_IT)
DEFINE_MASTER_HAL(HAL_I2C_Master_Transmit_DMA, CALL_MASTER_TX_DMA)
DEFINE_MASTER_HAL(HAL_I2C_Master_Receive_DMA, CALL_MASTER_RX_DMA)
DEFINE_MEMORY_BLOCKING_HAL(HAL_I2C_Mem_Write, CALL_MEMORY_TX_BLOCK)
DEFINE_MEMORY_BLOCKING_HAL(HAL_I2C_Mem_Read, CALL_MEMORY_RX_BLOCK)
DEFINE_MEMORY_HAL(HAL_I2C_Mem_Write_IT, CALL_MEMORY_TX_IT)
DEFINE_MEMORY_HAL(HAL_I2C_Mem_Read_IT, CALL_MEMORY_RX_IT)
DEFINE_MEMORY_HAL(HAL_I2C_Mem_Write_DMA, CALL_MEMORY_TX_DMA)
DEFINE_MEMORY_HAL(HAL_I2C_Mem_Read_DMA, CALL_MEMORY_RX_DMA)

static void TestCallback(IICInstance *instance, IIC_Event_e event)
{
    callback_count++;
    callback_event = event;
    callback_busy = instance->busy;
    if (callback_should_restart != 0U)
    {
        callback_restart_status = IICMemWrite(callback_restart_instance, 0x20U, IIC_MEMORY_ADDRESS_8BIT,
                                              callback_data, sizeof(callback_data));
    }
}

static void ResetTrace(void)
{
    next_status = HAL_OK;
    hal_call = CALL_NONE;
    hal_address = 0U;
    hal_memory_address = 0U;
    hal_memory_address_size = 0U;
    hal_trials = 0U;
    hal_timeout = 0U;
    hal_call_count = 0U;
    callback_restart_instance = NULL;
    callback_restart_status = HAL_ERROR;
    callback_event = IIC_EVENT_ERROR;
    callback_busy = 1U;
    callback_should_restart = 0U;
    callback_count = 0U;
}

static void CompleteAsync(I2C_HandleTypeDef *handle, IIC_Event_e event)
{
    switch (event)
    {
    case IIC_EVENT_MASTER_TX_COMPLETE:
        HAL_I2C_MasterTxCpltCallback(handle);
        break;
    case IIC_EVENT_MASTER_RX_COMPLETE:
        HAL_I2C_MasterRxCpltCallback(handle);
        break;
    case IIC_EVENT_MEMORY_TX_COMPLETE:
        HAL_I2C_MemTxCpltCallback(handle);
        break;
    case IIC_EVENT_MEMORY_RX_COMPLETE:
        HAL_I2C_MemRxCpltCallback(handle);
        break;
    case IIC_EVENT_ERROR:
        HAL_I2C_ErrorCallback(handle);
        break;
    }
}

int main(void)
{
    I2C_HandleTypeDef bus_a = {1U};
    I2C_HandleTypeDef bus_b = {2U};
    I2C_HandleTypeDef unknown_bus = {3U};
    uint8_t tx[3] = {1U, 2U, 3U};
    uint8_t rx[3] = {0U};

    const IIC_Init_Config_s config_a = {&bus_a, 0x77U, IIC_TRANSFER_DMA, TestCallback, NULL};
    const IIC_Init_Config_s config_b = {&bus_a, 0x76U, IIC_TRANSFER_DMA, NULL, NULL};
    const IIC_Init_Config_s config_c = {&bus_b, 0x50U, IIC_TRANSFER_BLOCKING, TestCallback, NULL};
    const IIC_Init_Config_s invalid_address = {&bus_a, 0x80U, IIC_TRANSFER_DMA, NULL, NULL};

    assert(IICRegister(NULL) == NULL);
    assert(IICRegister(&invalid_address) == NULL);
    IICInstance *device_a = IICRegister(&config_a);
    IICInstance *device_b = IICRegister(&config_b);
    IICInstance *device_c = IICRegister(&config_c);
    assert((device_a != NULL) && (device_b != NULL) && (device_c != NULL));
    assert(IICRegister(&config_a) == device_a);

    assert(IICTransmit(NULL, tx, sizeof(tx)) == HAL_ERROR);
    assert(IICTransmit(device_a, NULL, sizeof(tx)) == HAL_ERROR);
    assert(IICReceive(device_a, rx, 0U) == HAL_ERROR);
    assert(IICMemWrite(device_a, 0U, IIC_MEMORY_ADDRESS_8BIT, NULL, sizeof(tx)) == HAL_ERROR);
    assert(IICMemRead(device_a, 0U, IIC_MEMORY_ADDRESS_8BIT, NULL, sizeof(rx)) == HAL_ERROR);

    ResetTrace();
    assert(IICIsDeviceReady(device_a, 3U, 10U) == HAL_OK);
    assert((hal_call == CALL_READY) && (hal_address == 0xEEU));
    assert((hal_trials == 3U) && (hal_timeout == 10U));

    ResetTrace();
    next_status = HAL_TIMEOUT;
    assert(IICTransmit(device_c, tx, sizeof(tx)) == HAL_TIMEOUT);
    assert((hal_call == CALL_MASTER_TX_BLOCK) && (hal_address == 0xA0U) && (hal_timeout == 100U));
    assert(device_c->busy == 0U);
    assert(IICReceive(device_c, rx, sizeof(rx)) == HAL_TIMEOUT);
    assert(hal_call == CALL_MASTER_RX_BLOCK);
    assert(IICMemWrite(device_c, 0x1234U, IIC_MEMORY_ADDRESS_16BIT, tx, sizeof(tx)) == HAL_TIMEOUT);
    assert((hal_call == CALL_MEMORY_TX_BLOCK) && (hal_memory_address == 0x1234U));
    assert((hal_memory_address_size == I2C_MEMADD_SIZE_16BIT) && (hal_timeout == 100U));
    assert(IICMemRead(device_c, 0x42U, IIC_MEMORY_ADDRESS_8BIT, rx, sizeof(rx)) == HAL_TIMEOUT);
    assert((hal_call == CALL_MEMORY_RX_BLOCK) && (hal_memory_address == 0x42U));
    assert(hal_memory_address_size == I2C_MEMADD_SIZE_8BIT);

    ResetTrace();
    assert(IICMemRead(device_a, 0x10U, IIC_MEMORY_ADDRESS_8BIT, rx, sizeof(rx)) == HAL_OK);
    assert((hal_call == CALL_MEMORY_RX_DMA) && (device_a->busy == 1U));
    assert(IICMemWrite(device_b, 0x20U, IIC_MEMORY_ADDRESS_8BIT, tx, sizeof(tx)) == HAL_BUSY);
    assert(hal_call_count == 1U);
    assert(IICSetAddress(device_b, 0x75U) == HAL_BUSY);
    assert(IICSetMode(device_b, IIC_TRANSFER_IT) == HAL_BUSY);
    assert(IICIsDeviceReady(device_b, 1U, 1U) == HAL_BUSY);

    callback_restart_instance = device_b;
    callback_should_restart = 1U;
    HAL_I2C_MemRxCpltCallback(&bus_a);
    assert((callback_count == 1U) && (callback_event == IIC_EVENT_MEMORY_RX_COMPLETE));
    assert((callback_busy == 0U) && (callback_restart_status == HAL_OK));
    assert((device_a->busy == 0U) && (device_b->busy == 1U));
    assert(hal_call == CALL_MEMORY_TX_DMA);
    HAL_I2C_MemTxCpltCallback(&bus_a);
    assert(device_b->busy == 0U);

    ResetTrace();
    next_status = HAL_TIMEOUT;
    assert(IICMemWrite(device_a, 0x33U, IIC_MEMORY_ADDRESS_8BIT, tx, sizeof(tx)) == HAL_TIMEOUT);
    assert((hal_call == CALL_MEMORY_TX_DMA) && (device_a->busy == 0U));
    next_status = HAL_OK;
    assert(IICReceive(device_b, rx, sizeof(rx)) == HAL_OK);
    assert(device_b->busy == 1U);
    HAL_I2C_ErrorCallback(&bus_a);
    assert(device_b->busy == 0U);

    assert(IICSetMode(device_a, IIC_TRANSFER_IT) == HAL_OK);
    ResetTrace();
    assert(IICTransmit(device_a, tx, sizeof(tx)) == HAL_OK);
    assert(hal_call == CALL_MASTER_TX_IT);
    CompleteAsync(&bus_a, IIC_EVENT_MASTER_TX_COMPLETE);
    assert((callback_event == IIC_EVENT_MASTER_TX_COMPLETE) && (callback_busy == 0U));
    ResetTrace();
    assert(IICReceive(device_a, rx, sizeof(rx)) == HAL_OK);
    assert(hal_call == CALL_MASTER_RX_IT);
    CompleteAsync(&bus_a, IIC_EVENT_MASTER_RX_COMPLETE);
    ResetTrace();
    assert(IICMemWrite(device_a, 0x44U, IIC_MEMORY_ADDRESS_8BIT, tx, sizeof(tx)) == HAL_OK);
    assert(hal_call == CALL_MEMORY_TX_IT);
    CompleteAsync(&bus_a, IIC_EVENT_MEMORY_TX_COMPLETE);
    ResetTrace();
    assert(IICMemRead(device_a, 0x55U, IIC_MEMORY_ADDRESS_8BIT, rx, sizeof(rx)) == HAL_OK);
    assert(hal_call == CALL_MEMORY_RX_IT);
    CompleteAsync(&bus_a, IIC_EVENT_MEMORY_RX_COMPLETE);

    assert(IICSetMode(device_a, (IIC_TransferMode_e)99) == HAL_ERROR);
    assert(device_a->transfer_mode == IIC_TRANSFER_IT);
    assert(IICSetAddress(device_a, 0x60U) == HAL_OK);
    assert(device_a->address_7bit == 0x60U);
    assert(IICSetAddress(device_a, 0x80U) == HAL_ERROR);
    HAL_I2C_ErrorCallback(&unknown_bus);

    return 0;
}
"""


def find_vs_devcmd() -> Path | None:
    candidates: list[Path] = []
    for root_name in (os.environ.get("ProgramFiles"), os.environ.get("ProgramFiles(x86)")):
        if root_name:
            candidates.extend((Path(root_name) / "Microsoft Visual Studio").glob("*/*/Common7/Tools/VsDevCmd.bat"))
    return sorted(candidates)[-1] if candidates else None


def find_host_gcc() -> Path | None:
    configured = os.environ.get("HOST_CC")
    if configured and Path(configured).is_file():
        return Path(configured)
    if on_path := shutil.which("gcc"):
        return Path(on_path)

    candidates: list[Path] = []
    for root in (Path("D:/applications/Clion"), Path.home() / "AppData/Local/JetBrains/Toolbox/apps/CLion"):
        if root.exists():
            candidates.extend(root.glob("**/bin/mingw/bin/gcc.exe"))
    return sorted(candidates)[-1] if candidates else None


def verify_source_contract() -> None:
    source = IIC_SOURCE.read_text(encoding="utf-8")
    header = (IIC_DIR / "bsp_iic.h").read_text(encoding="utf-8")
    for dead_field in (
        "pending_event",
        "tx_buffer",
        "rx_buffer",
        "tx_size",
        "rx_size",
        "memory_address;",
        "memory_address_size;",
    ):
        assert dead_field not in header
    assert source.index("IIC_ReleaseBus(bus);") < source.index("owner->callback(owner, event);")
    assert "if (status != HAL_OK)" in source


def main() -> None:
    verify_source_contract()
    vs_devcmd = find_vs_devcmd()
    gcc = find_host_gcc()
    if (vs_devcmd is None) and (gcc is None):
        print("I2C source contract passed; host C compiler unavailable, executable HAL-mock test skipped.")
        return

    with tempfile.TemporaryDirectory(prefix="uav-iic-regression-") as temp_name:
        temp = Path(temp_name)
        (temp / "i2c.h").write_text(MOCK_I2C_H, encoding="utf-8")
        harness = temp / "iic_harness.c"
        harness.write_text(HARNESS_C, encoding="utf-8")
        executable = temp / "iic_harness.exe"

        if vs_devcmd is not None:
            build_script = temp / "build_iic_harness.cmd"
            build_script.write_text(
                "@echo off\n"
                f'call "{vs_devcmd}" -no_logo -arch=x64 -host_arch=x64 >nul\n'
                "if errorlevel 1 exit /b %errorlevel%\n"
                f'cl /nologo /std:c11 /utf-8 /W4 /WX /I"{temp}" /I"{IIC_DIR}" '
                f'"{IIC_SOURCE}" "{harness}" /Fe:"{executable}"\n',
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
                    f"-I{IIC_DIR}",
                    str(IIC_SOURCE),
                    str(harness),
                    "-o",
                    str(executable),
                ],
                capture_output=True,
                text=True,
            )

        if compile_result.returncode != 0:
            raise RuntimeError(
                f"I2C HAL-mock build failed with exit code {compile_result.returncode}:\n"
                f"{compile_result.stdout}{compile_result.stderr}"
            )
        subprocess.run([str(executable)], check=True)

    print("I2C regression passed: mode dispatch, arbitration, rollback and ISR callback ordering verified.")


if __name__ == "__main__":
    main()
