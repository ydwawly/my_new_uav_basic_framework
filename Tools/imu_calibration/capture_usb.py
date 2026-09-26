#!/usr/bin/env python3
"""Capture the BMI088 FLOG stream from STM32 USB CDC with automatic reconnect."""

from __future__ import annotations

import argparse
import os
import secrets
import struct
import sys
import time
from pathlib import Path
from typing import Any


USB_VID = 0x0483
USB_PID = 0x5740
START_PREFIX = b"IMUCAP1"
FILE_MAGIC_BYTES = b"FLOG"
FILE_MAGIC = 0x474F4C46
FILE_HEADER = struct.Struct("<IHHQIHH")


def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    crc = initial
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def find_valid_file_header(buffer: bytes | bytearray, expected_token: int | None = None) -> tuple[int, bytes] | None:
    search_from = 0
    while True:
        offset = buffer.find(FILE_MAGIC_BYTES, search_from)
        if offset < 0:
            return None
        if len(buffer) - offset < FILE_HEADER.size:
            return None
        candidate = bytes(buffer[offset : offset + FILE_HEADER.size])
        magic, version, header_size, _, flags, header_crc, _ = FILE_HEADER.unpack(candidate)
        token = flags >> 16
        valid_flags = (flags & 0x3) == 0x3
        if (
            magic == FILE_MAGIC
            and version == 1
            and header_size == FILE_HEADER.size
            and valid_flags
            and crc16_ccitt(candidate[:20]) == header_crc
            and (expected_token is None or token == expected_token)
        ):
            return offset, candidate
        search_from = offset + 1


def load_serial_modules() -> tuple[Any, Any]:
    try:
        import serial
        from serial.tools import list_ports
    except ModuleNotFoundError as exc:
        raise RuntimeError("缺少 pyserial，请先执行: python -m pip install pyserial>=3.5") from exc
    return serial, list_ports


def find_port(list_ports: Any, requested: str | None, serial_number: str | None) -> str | None:
    ports = list(list_ports.comports())
    if requested:
        requested_upper = requested.upper()
        return next((port.device for port in ports if port.device.upper() == requested_upper), None)

    matches = [port for port in ports if port.vid == USB_VID and port.pid == USB_PID]
    if serial_number:
        matches = [port for port in matches if port.serial_number == serial_number]
    if not matches:
        return None
    matches.sort(key=lambda port: port.device)
    return matches[0].device


def create_part_file(output_directory: Path, port: str, part_number: int) -> tuple[Path, Any]:
    output_directory.mkdir(parents=True, exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    safe_port = "".join(character if character.isalnum() else "_" for character in port)
    suffix = 0
    while True:
        extra = f"_{suffix:02d}" if suffix else ""
        path = output_directory / f"BMI088_USB_{timestamp}_{safe_port}_part{part_number:03d}{extra}.BIN"
        try:
            return path, path.open("xb", buffering=1024 * 1024)
        except FileExistsError:
            suffix += 1


def wait_for_stream_header(port: Any, handshake_timeout_s: float) -> tuple[bytes, bytes]:
    buffer = bytearray()
    deadline = time.monotonic() + handshake_timeout_s
    next_request = 0.0
    session_token = secrets.randbelow(65535) + 1
    start_command = START_PREFIX + struct.pack("<H", session_token) + b"\n"
    port.reset_input_buffer()

    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_request:
            port.write(start_command)
            port.flush()
            next_request = now + 1.0

        waiting = int(getattr(port, "in_waiting", 0))
        chunk = port.read(max(1, min(waiting, 65536)))
        if chunk:
            buffer.extend(chunk)
            found = find_valid_file_header(buffer, expected_token=session_token)
            if found is not None:
                offset, header = found
                return header, bytes(buffer[offset + FILE_HEADER.size :])
            if len(buffer) > 8192:
                del buffer[:-FILE_HEADER.size]

    raise TimeoutError("飞控未在握手后返回有效 FLOG 文件头")


def capture_part(
    serial_module: Any,
    port_name: str,
    output_directory: Path,
    part_number: int,
    baudrate: int,
    silence_timeout_s: float,
    sync_period_s: float,
    end_time: float | None,
) -> Path:
    with serial_module.Serial(port_name, baudrate=baudrate, timeout=0.25, write_timeout=1.0) as port:
        port.dtr = True
        header, initial_data = wait_for_stream_header(port, handshake_timeout_s=6.0)
        path, output = create_part_file(output_directory, port_name, part_number)
        print(f"[连接] {port_name} -> {path}")

        total_bytes = 0
        started = time.monotonic()
        last_data = started
        last_status = started
        last_sync = started
        try:
            output.write(header)
            output.write(initial_data)
            total_bytes = FILE_HEADER.size + len(initial_data)

            while end_time is None or time.monotonic() < end_time:
                waiting = int(port.in_waiting)
                chunk = port.read(max(1, min(waiting, 65536)))
                now = time.monotonic()
                if chunk:
                    output.write(chunk)
                    total_bytes += len(chunk)
                    last_data = now
                elif now - last_data >= silence_timeout_s:
                    raise TimeoutError(f"连续 {silence_timeout_s:.1f} 秒没有收到 IMU 数据")

                if now - last_sync >= sync_period_s:
                    output.flush()
                    os.fsync(output.fileno())
                    last_sync = now

                if now - last_status >= 1.0:
                    elapsed = max(now - started, 1e-6)
                    print(
                        f"[采集中] part={part_number:03d}  "
                        f"{total_bytes / 1048576.0:8.2f} MiB  {total_bytes / elapsed / 1024.0:7.1f} KiB/s",
                        flush=True,
                    )
                    last_status = now
        finally:
            output.flush()
            os.fsync(output.fileno())
            output.close()
        return path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-o", "--output-directory", type=Path, default=Path("output/usb_capture"))
    parser.add_argument("--port", help="固定端口，例如 COM7；省略时按 VID/PID 自动枚举")
    parser.add_argument("--serial-number", help="多块 STM32 同时连接时用于筛选 USB 序列号")
    parser.add_argument(
        "--baudrate",
        type=int,
        default=115_200,
        help="CDC 虚拟波特率，默认 115200；不限制 USB 实际吞吐率",
    )
    parser.add_argument("--duration-minutes", type=float, default=0.0, help="总采集时长；0 表示直到 Ctrl+C")
    parser.add_argument("--reconnect-delay", type=float, default=1.0)
    parser.add_argument("--silence-timeout", type=float, default=3.0)
    parser.add_argument("--sync-period", type=float, default=5.0)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.duration_minutes < 0 or args.reconnect_delay <= 0 or args.silence_timeout <= 0 or args.sync_period <= 0:
        print("时长和超时参数无效", file=sys.stderr)
        return 2

    try:
        serial_module, list_ports = load_serial_modules()
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        return 2

    end_time = time.monotonic() + args.duration_minutes * 60.0 if args.duration_minutes > 0 else None
    part_number = 1
    print("等待 STM32 USB CDC（VID=0483, PID=5740），按 Ctrl+C 停止。")
    try:
        while end_time is None or time.monotonic() < end_time:
            port_name = find_port(list_ports, args.port, args.serial_number)
            if port_name is None:
                print("[等待] 未找到匹配端口", flush=True)
                time.sleep(args.reconnect_delay)
                continue
            try:
                capture_part(
                    serial_module,
                    port_name,
                    args.output_directory,
                    part_number,
                    args.baudrate,
                    args.silence_timeout,
                    args.sync_period,
                    end_time,
                )
                part_number += 1
            except (serial_module.SerialException, OSError, TimeoutError) as exc:
                print(f"[断开] {port_name}: {exc}；{args.reconnect_delay:.1f} 秒后自动重连", flush=True)
                part_number += 1
                time.sleep(args.reconnect_delay)
    except KeyboardInterrupt:
        print("\n已停止采集。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
