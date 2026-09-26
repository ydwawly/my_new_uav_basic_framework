"""飞控传感器协议与单位换算的主机端回归测试。

这些测试不访问 STM32 外设，目的是用确定的字节向量验证协议字段、小端转换、
校验和、SBUS 11 位打包及工程中的物理量换算约定。实机 DMA/中断测试结果另由
J-Link RTT 和 GDB 观察完成。
"""

from __future__ import annotations

import struct
import unittest
from math import isfinite
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def additive_checksum(frame_without_checksum: bytes) -> int:
    return sum(frame_without_checksum) & 0xFF


def crc8_dvb_s2(data: bytes) -> int:
    """计算 MSPv2 使用的 CRC-8/DVB-S2（多项式 0xD5）。"""
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def pack_msp_v2(command: int, payload: bytes, flags: int = 0) -> bytes:
    """按原生 MSPv2 的 $X< 格式构造传感器上行帧。"""
    body = struct.pack("<BHH", flags, command, len(payload)) + payload
    return b"$X<" + body + bytes((crc8_dvb_s2(body),))


def ubx_checksum(payload: bytes) -> tuple[int, int]:
    ck_a = 0
    ck_b = 0
    for value in payload:
        ck_a = (ck_a + value) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def pack_sbus_channels(channels: list[int]) -> bytes:
    bit_stream = 0
    for index, channel in enumerate(channels):
        bit_stream |= (channel & 0x07FF) << (index * 11)
    return bit_stream.to_bytes(22, "little")


def unpack_sbus_channels(payload: bytes) -> list[int]:
    bit_stream = int.from_bytes(payload, "little")
    return [(bit_stream >> (index * 11)) & 0x07FF for index in range(16)]


def assemble_mtf02_chunks(chunks: list[bytes]) -> list[bytes]:
    """模拟固件的按字节同步逻辑，验证 UART IDLE 任意分片不会破坏协议帧。"""
    assembly = bytearray()
    frames: list[bytes] = []

    for chunk in chunks:
        for value in chunk:
            if not assembly and value != 0xEF:
                continue
            assembly.append(value)

            if (len(assembly) == 4 and value != 0x51) or (len(assembly) == 6 and value != 20):
                assembly = bytearray((value,)) if value == 0xEF else bytearray()
                continue

            if len(assembly) == 27:
                if additive_checksum(assembly[:-1]) == assembly[-1]:
                    frames.append(bytes(assembly))
                assembly.clear()

    return frames


def assemble_msp_v2_chunks(chunks: list[bytes]) -> list[tuple[int, bytes]]:
    """模拟固件 MSPv2 字节状态机，验证 UART IDLE 任意分片均能正确拼帧。"""
    stream = b"".join(chunks)
    frames: list[tuple[int, bytes]] = []
    index = 0

    while index + 9 <= len(stream):
        header = stream.find(b"$X<", index)
        if header < 0 or header + 9 > len(stream):
            break

        payload_size = struct.unpack_from("<H", stream, header + 6)[0]
        frame_end = header + 9 + payload_size
        if frame_end > len(stream):
            break

        body = stream[header + 3 : frame_end - 1]
        if crc8_dvb_s2(body) == stream[frame_end - 1]:
            command = struct.unpack_from("<H", stream, header + 4)[0]
            frames.append((command, stream[header + 8 : frame_end - 1]))
        index = frame_end

    return frames


def sign_extend(value: int, bit_count: int) -> int:
    """把指定宽度的二进制补码扩展为 Python 有符号整数。"""
    value &= (1 << bit_count) - 1
    sign = 1 << (bit_count - 1)
    return value - (1 << bit_count) if value & sign else value


def pack_spl06_coefficients(values: tuple[int, ...]) -> bytes:
    """按 SPL06 数据手册的跨字节布局打包九个补偿系数。"""
    c0, c1, c00, c10, c01, c11, c20, c21, c30 = values
    c0 &= 0xFFF
    c1 &= 0xFFF
    c00 &= 0xFFFFF
    c10 &= 0xFFFFF
    raw = bytearray(18)
    raw[0] = c0 >> 4
    raw[1] = ((c0 & 0xF) << 4) | (c1 >> 8)
    raw[2] = c1 & 0xFF
    raw[3] = c00 >> 12
    raw[4] = (c00 >> 4) & 0xFF
    raw[5] = ((c00 & 0xF) << 4) | (c10 >> 16)
    raw[6] = (c10 >> 8) & 0xFF
    raw[7] = c10 & 0xFF
    for index, coefficient in enumerate((c01, c11, c20, c21, c30), start=4):
        struct.pack_into(">H", raw, 2 * index, coefficient & 0xFFFF)
    return bytes(raw)


def unpack_spl06_coefficients(raw: bytes) -> tuple[int, ...]:
    """复现固件的 SPL06 系数解析，用测试向量验证符号扩展与位拼接。"""
    return (
        sign_extend((raw[0] << 4) | (raw[1] >> 4), 12),
        sign_extend(((raw[1] & 0xF) << 8) | raw[2], 12),
        sign_extend((raw[3] << 12) | (raw[4] << 4) | (raw[5] >> 4), 20),
        sign_extend(((raw[5] & 0xF) << 16) | (raw[6] << 8) | raw[7], 20),
        *(sign_extend(struct.unpack_from(">H", raw, offset)[0], 16) for offset in (8, 10, 12, 14, 16)),
    )


class SensorProtocolRegression(unittest.TestCase):
    def test_mtf02_micolink_frame_and_velocity_scale(self) -> None:
        payload = bytearray(20)
        struct.pack_into("<I", payload, 0, 12_345)  # 传感器时间：ms
        struct.pack_into("<I", payload, 4, 1_250)   # 距离：mm，即 1.25 m
        payload[8:12] = bytes((180, 2, 0, 0))
        struct.pack_into("<h", payload, 12, -120)   # cm/s @ 1 m
        struct.pack_into("<h", payload, 14, 80)
        payload[16:20] = bytes((200, 0, 0, 0))

        frame = bytearray((0xEF, 1, 1, 0x51, 7, len(payload)))
        frame.extend(payload)
        frame.append(additive_checksum(frame))

        self.assertEqual(len(frame), 27)
        self.assertEqual(frame[-1], additive_checksum(frame[:-1]))
        self.assertEqual(struct.unpack_from("<I", frame, 10)[0], 1_250)
        self.assertEqual(struct.unpack_from("<h", frame, 18)[0], -120)

        height_m = struct.unpack_from("<I", frame, 10)[0] * 0.001
        velocity_x_mps = struct.unpack_from("<h", frame, 18)[0] * height_m * 0.01
        self.assertAlmostEqual(velocity_x_mps, -1.5, places=6)

        chunks = [b"\x00\xAA" + frame[:3], frame[3:9], frame[9:10], frame[10:]]
        self.assertEqual(assemble_mtf02_chunks(chunks), [bytes(frame)])

    def test_mtf02_msp_v2_range_flow_and_rate_conversion(self) -> None:
        range_payload = struct.pack("<Bi", 220, 1_250)
        flow_payload = struct.pack("<Bii", 200, 21, -42)
        range_frame = pack_msp_v2(0x1F01, range_payload)
        flow_frame = pack_msp_v2(0x1F02, flow_payload)

        self.assertTrue(range_frame.startswith(b"$X<"))
        self.assertEqual(len(range_frame), 14)
        self.assertEqual(len(flow_frame), 18)
        self.assertEqual(crc8_dvb_s2(range_frame[3:-1]), range_frame[-1])
        self.assertEqual(crc8_dvb_s2(flow_frame[3:-1]), flow_frame[-1])

        chunks = [
            b"\x00\xEF" + range_frame[:2],
            range_frame[2:7],
            range_frame[7:] + flow_frame[:1],
            flow_frame[1:10],
            flow_frame[10:],
        ]
        frames = assemble_msp_v2_chunks(chunks)
        self.assertEqual(frames, [(0x1F01, range_payload), (0x1F02, flow_payload)])
        self.assertEqual(struct.unpack("<Bi", frames[0][1]), (220, 1_250))
        self.assertEqual(struct.unpack("<Bii", frames[1][1]), (200, 21, -42))

        delta_time_us = 20_000
        rate_scale = (1_000_000.0 / delta_time_us) / 10.5 * (3.141592653589793 / 180.0)
        self.assertAlmostEqual(21 * rate_scale, 1.74532925199, places=6)
        self.assertAlmostEqual(-42 * rate_scale, -3.49065850399, places=6)

    def test_tfmini_frame_checksum_and_units(self) -> None:
        distance_cm = 234
        strength = 500
        raw_temperature = int((25.0 + 256.0) * 8.0)
        frame = bytearray((0x59, 0x59))
        frame.extend(struct.pack("<HHH", distance_cm, strength, raw_temperature))
        frame.append(additive_checksum(frame))

        self.assertEqual(len(frame), 9)
        self.assertEqual(frame[-1], additive_checksum(frame[:-1]))
        self.assertEqual(struct.unpack_from("<H", frame, 2)[0] * 0.01, 2.34)
        self.assertAlmostEqual(struct.unpack_from("<H", frame, 6)[0] / 8.0 - 256.0, 25.0)

    def test_sbus_11_bit_channel_round_trip_and_flags(self) -> None:
        channels = [172, 992, 1811, 0, 2047, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300]
        flags = (1 << 0) | (1 << 2)
        frame = bytes((0x0F,)) + pack_sbus_channels(channels) + bytes((flags, 0x00))

        self.assertEqual(len(frame), 25)
        self.assertEqual(unpack_sbus_channels(frame[1:23]), channels)
        self.assertEqual((frame[23] >> 0) & 1, 1)  # CH17
        self.assertEqual((frame[23] >> 2) & 1, 1)  # frame_lost
        self.assertEqual((frame[23] >> 3) & 1, 0)  # failsafe

    def test_ubx_nav_pvt_checksum_and_metric_conversion(self) -> None:
        payload = bytearray(92)
        struct.pack_into("<HBBBBB", payload, 4, 2026, 7, 28, 12, 34, 56)
        payload[11] = 0x07
        payload[20:24] = bytes((3, 1, 0, 18))
        struct.pack_into("<i", payload, 24, 1_164_070_000)  # lon: 116.407°
        struct.pack_into("<i", payload, 28, 399_042_000)    # lat: 39.9042°
        struct.pack_into("<i", payload, 32, 50_500)         # mm
        struct.pack_into("<i", payload, 36, 45_250)         # mm
        struct.pack_into("<I", payload, 40, 800)            # hAcc: mm
        struct.pack_into("<i", payload, 48, 1_250)          # velN: mm/s
        struct.pack_into("<i", payload, 64, 9_000_000)      # heading: 90 deg

        body = bytes((0x01, 0x07, 92, 0)) + payload
        ck_a, ck_b = ubx_checksum(body)
        frame = bytes((0xB5, 0x62)) + body + bytes((ck_a, ck_b))

        self.assertEqual(len(frame), 100)
        self.assertEqual(ubx_checksum(frame[2:-2]), tuple(frame[-2:]))
        self.assertAlmostEqual(struct.unpack_from("<i", payload, 28)[0] * 1e-7, 39.9042)
        self.assertAlmostEqual(struct.unpack_from("<i", payload, 36)[0] * 0.001, 45.25)
        self.assertAlmostEqual(struct.unpack_from("<i", payload, 48)[0] * 0.001, 1.25)
        self.assertAlmostEqual(struct.unpack_from("<i", payload, 64)[0] * 1e-5, 90.0)

    def test_uart_dma_modes_match_fixed_frame_drivers(self) -> None:
        ioc = (PROJECT_ROOT / "stm32h743_uav_flight_controller.ioc").read_text(encoding="utf-8")
        uart_bsp = (PROJECT_ROOT / "Bsp/bsp_uart/bsp_uart.c").read_text(encoding="utf-8")
        sensor_header = (PROJECT_ROOT / "Application/App_Sensor/App_Sensor.h").read_text(encoding="utf-8")
        mtf02_driver = (
            PROJECT_ROOT / "Modules/modules_optical_flow_sensor/modules_mtf02/modules_mtf02.c"
        ).read_text(encoding="utf-8")

        self.assertIn("Dma.USART1_RX.8.Mode=DMA_NORMAL", ioc)
        self.assertIn("Dma.USART6_RX.7.Mode=DMA_NORMAL", ioc)
        self.assertIn("event_type == HAL_UART_RXEVENT_HT", uart_bsp)
        self.assertIn("huart->RxState == HAL_UART_STATE_BUSY_RX", uart_bsp)
        self.assertIn("#define APP_SENSOR_ENABLE_GPS 0U", sensor_header)
        self.assertIn("MTF02_ConsumeMicoLinkByte(data_ptr[index])", mtf02_driver)
        self.assertIn("MTF02_ConsumeMspByte(data_ptr[index])", mtf02_driver)
        self.assertIn("MTF02_MSP2_SENSOR_RANGEFINDER", mtf02_driver)
        self.assertIn("MTF02_MSP2_SENSOR_OPTICAL_FLOW", mtf02_driver)

    def test_spl06_coefficients_compensation_and_dma_contract(self) -> None:
        coefficients = (-100, 850, -250_000, 180_000, -1200, 900, 30, -15, 2)
        packed = pack_spl06_coefficients(coefficients)
        self.assertEqual(len(packed), 18)
        self.assertEqual(unpack_spl06_coefficients(packed), coefficients)
        self.assertEqual(sign_extend(0xFFFFFF, 24), -1)
        self.assertEqual(sign_extend(0x800000, 24), -(1 << 23))

        raw_pressure = 1_000_000
        raw_temperature = 100_000
        pressure_scaled = raw_pressure / 7_864_320.0
        temperature_scaled = raw_temperature / 524_288.0
        c0, c1, c00, c10, c01, c11, c20, c21, c30 = coefficients
        pressure_pa = (
            c00
            + pressure_scaled * (c10 + pressure_scaled * (c20 + pressure_scaled * c30))
            + temperature_scaled * c01
            + temperature_scaled * pressure_scaled * (c11 + pressure_scaled * c21)
        )
        temperature_c = c0 * 0.5 + c1 * temperature_scaled
        self.assertTrue(isfinite(pressure_pa))
        self.assertTrue(isfinite(temperature_c))

        ioc = (PROJECT_ROOT / "stm32h743_uav_flight_controller.ioc").read_text(encoding="utf-8")
        driver = (
            PROJECT_ROOT / "Modules/modules_altitude_sensor/modules_SPL06/modules_SPL06.c"
        ).read_text(encoding="utf-8")
        header = (
            PROJECT_ROOT / "Modules/modules_altitude_sensor/modules_SPL06/modules_SPL06.h"
        ).read_text(encoding="utf-8")

        self.assertIn("Dma.I2C2_RX.6.Instance=DMA2_Stream7", ioc)
        self.assertIn("Dma.I2C2_RX.6.Mode=DMA_NORMAL", ioc)
        self.assertIn("PB10.Signal=I2C2_SCL", ioc)
        self.assertIn("PB11.Signal=I2C2_SDA", ioc)
        self.assertIn("PD0.GPIO_Label=SPL06_INT", ioc)
        self.assertIn("#define SPL06_DMA_FRAME_SIZE", header)
        self.assertIn("memcpy(spl06_instance.raw_frame", driver)
        self.assertIn("memcpy(local_frame, spl06_instance.raw_frame", driver)
        self.assertNotIn("spl06_dma_rx_buffer[2]", driver)

    def test_imu_hot_reset_blocks_drdy_until_initialization_finishes(self) -> None:
        bmi088 = (PROJECT_ROOT / "Modules/modules_BMI088/modules_BMI088.c").read_text(encoding="utf-8")
        bmi270 = (PROJECT_ROOT / "Modules/modules_BMI270/modules_BMI270.c").read_text(encoding="utf-8")

        self.assertIn("BMI088_NOT_INITIALIZED;", bmi088)
        self.assertIn("if (bmi088.init_status != BMI088_OK)", bmi088)
        self.assertLess(
            bmi088.index("BMI088_NOT_INITIALIZED;"),
            bmi088.index("BMI088_Status_e status = BMI088_Accel_Init();"),
        )

        self.assertIn("BMI270_NOT_INITIALIZED;", bmi270)
        self.assertIn("if (bmi270.init_status != BMI270_OK)", bmi270)
        self.assertLess(
            bmi270.index("BMI270_NOT_INITIALIZED;"),
            bmi270.index("BMI270_Status_e status = BMI270_UploadConfigFile();"),
        )

    def test_sd_diskio_unaligned_dma_fix_is_preserved(self) -> None:
        """CubeMX重新生成后，非对齐写分支的提前返回不能丢失。"""
        diskio = (PROJECT_ROOT / "FATFS/Target/sd_diskio.c").read_text(encoding="utf-8")
        unaligned_branch = diskio.index("if (((uint32_t)buff & 0x1FU) != 0U)")
        unaligned_dma_start = diskio.index("BSP_SD_WriteBlocks_DMA((uint32_t *)scratch", unaligned_branch)
        early_return = diskio.index("return res;", unaligned_dma_start)
        aligned_dma_start = diskio.index("BSP_SD_WriteBlocks_DMA((uint32_t *)buff", early_return)

        self.assertLess(unaligned_branch, unaligned_dma_start)
        self.assertLess(unaligned_dma_start, early_return)
        self.assertLess(early_return, aligned_dma_start)


if __name__ == "__main__":
    unittest.main(verbosity=2)
