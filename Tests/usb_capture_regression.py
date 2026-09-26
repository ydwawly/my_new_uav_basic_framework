#!/usr/bin/env python3
"""Regression tests for USB FLOG resynchronization without requiring pyserial."""

from __future__ import annotations

import importlib.util
import struct
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "Tools" / "imu_calibration" / "capture_usb.py"
SPEC = importlib.util.spec_from_file_location("capture_usb", MODULE_PATH)
capture = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = capture
SPEC.loader.exec_module(capture)


def make_header(timestamp_us: int = 1234, token: int = 0) -> bytes:
    prefix = struct.pack("<IHHQI", capture.FILE_MAGIC, 1, capture.FILE_HEADER.size, timestamp_us, 3 | (token << 16))
    return prefix + struct.pack("<HH", capture.crc16_ccitt(prefix), 0)


class UsbCaptureRegression(unittest.TestCase):
    def test_finds_header_after_stale_stream_bytes(self) -> None:
        header = make_header()
        buffer = b"stale usb bytes\x5a\xa5" + header + b"frame"
        found = capture.find_valid_file_header(buffer)
        self.assertIsNotNone(found)
        offset, parsed = found
        self.assertEqual(offset, len(b"stale usb bytes\x5a\xa5"))
        self.assertEqual(parsed, header)

    def test_rejects_corrupt_header_and_uses_next_valid_one(self) -> None:
        corrupt = bytearray(make_header())
        corrupt[8] ^= 0x40
        valid = make_header(5678)
        found = capture.find_valid_file_header(bytes(corrupt) + b"noise" + valid)
        self.assertIsNotNone(found)
        offset, parsed = found
        self.assertEqual(offset, len(corrupt) + len(b"noise"))
        self.assertEqual(parsed, valid)

    def test_waits_for_complete_fragmented_header(self) -> None:
        header = make_header()
        self.assertIsNone(capture.find_valid_file_header(header[:10]))
        self.assertEqual(capture.find_valid_file_header(header), (0, header))

    def test_session_token_rejects_stale_valid_header(self) -> None:
        stale = make_header(token=10)
        current = make_header(timestamp_us=9999, token=20)
        found = capture.find_valid_file_header(stale + current, expected_token=20)
        self.assertEqual(found, (len(stale), current))


if __name__ == "__main__":
    unittest.main()
