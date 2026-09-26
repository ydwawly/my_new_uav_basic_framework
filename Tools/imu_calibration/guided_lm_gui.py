#!/usr/bin/env python3
"""Visual, self-checking BMI088 LM capture over STM32 USB CDC."""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import subprocess
import sys
import threading
import time
import traceback
from dataclasses import asdict
from pathlib import Path
from typing import Any

from capture_usb import (
    create_part_file,
    find_port,
    load_serial_modules,
    wait_for_stream_header,
)
from guided_lm_core import (
    FlogStreamParser,
    GuidedLmSession,
    ParserStats,
    TARGET_POSES,
    TemperatureMonitor,
)
from quadrotor_visual import QuadrotorView, turn_instruction


class UsbGuidedWorker(threading.Thread):
    """Own the serial port, lossless file writer and guided session state."""

    def __init__(
        self, args: argparse.Namespace, events: queue.Queue[dict[str, Any]]
    ) -> None:
        super().__init__(name="bmi088-usb-guided", daemon=True)
        self.args = args
        self.events = events
        self.commands: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.shutdown_requested = False
        self.record_requested = False
        self.record_directory: Path | None = None
        self.part_number = 1
        self.restart_after_disconnect = False
        self.thermal_monitor = TemperatureMonitor()

    def command(self, name: str, value: Any = None) -> None:
        self.commands.put((name, value))

    def run(self) -> None:
        try:
            serial_module, list_ports = load_serial_modules()
        except Exception as exc:  # noqa: BLE001 - forwarded to the UI
            self._post("fatal", message=str(exc))
            return

        while not self.shutdown_requested:
            self._drain_offline_commands()
            if self.shutdown_requested:
                break
            port_name = find_port(list_ports, self.args.port, self.args.serial_number)
            if port_name is None:
                self._post(
                    "connection",
                    connected=False,
                    message="等待 STM32 USB CDC（自动重连中）",
                )
                self._responsive_wait(self.args.reconnect_delay)
                continue
            try:
                self._connected_run(serial_module, port_name)
            except (serial_module.SerialException, OSError, TimeoutError) as exc:
                self._post(
                    "connection",
                    connected=False,
                    message=f"{port_name} 已断开：{exc}；正在自动重连",
                )
                self._responsive_wait(self.args.reconnect_delay)
            except (
                Exception
            ) as exc:  # noqa: BLE001 - keep reconnect loop alive and expose details
                self._post(
                    "fatal",
                    message=f"USB 工作线程异常：{exc}\n{traceback.format_exc()}",
                )
                return

    def _connected_run(self, serial_module: Any, port_name: str) -> None:
        output: Any | None = None
        capture_path: Path | None = None
        session: GuidedLmSession | None = None
        parser = FlogStreamParser()
        self.thermal_monitor.reset()
        total_bytes = 0
        last_sync = time.monotonic()
        last_live = 0.0
        last_data = time.monotonic()
        header = b""

        try:
            with serial_module.Serial(
                port_name,
                baudrate=self.args.baudrate,
                timeout=0.025,
                write_timeout=1.0,
            ) as port:
                port.dtr = True
                if hasattr(port, "set_buffer_size"):
                    try:
                        port.set_buffer_size(rx_size=1_048_576)
                    except (
                        AttributeError,
                        OSError,
                        ValueError,
                        serial_module.SerialException,
                    ):
                        pass
                header, initial_data = wait_for_stream_header(
                    port, handshake_timeout_s=6.0
                )
                self._post(
                    "connection",
                    connected=True,
                    port=port_name,
                    message=f"已连接 {port_name}",
                )

                if self.record_requested:
                    output, capture_path, session = self._start_part(
                        header, parser, port_name
                    )
                    total_bytes = len(header)
                    event_name = (
                        "session_restarted"
                        if self.restart_after_disconnect
                        else "session_started"
                    )
                    self._post(event_name, path=str(capture_path))
                    self.restart_after_disconnect = False

                frames = parser.feed(initial_data)
                output, session, total_bytes = self._consume_frames(
                    frames, parser, output, capture_path, session, total_bytes
                )

                while not self.shutdown_requested:
                    commands = self._drain_commands()
                    for name, value in commands:
                        if name == "shutdown":
                            self.shutdown_requested = True
                        elif name == "stop":
                            self.record_requested = False
                            if output is not None:
                                self._save_interrupted(
                                    session, parser.stats, capture_path, "用户停止采集"
                                )
                                self._close_output(output)
                                output = None
                                session = None
                                self._post("session_stopped", path=str(capture_path))
                        elif name == "start":
                            if output is not None:
                                self._save_interrupted(
                                    session, parser.stats, capture_path, "用户重新开始"
                                )
                                self._close_output(output)
                            self.record_requested = True
                            self.record_directory = Path(value)
                            output, capture_path, session = self._start_part(
                                header, parser, port_name
                            )
                            total_bytes = len(header)
                            self._post("session_started", path=str(capture_path))

                    if self.shutdown_requested:
                        break

                    # Read a block instead of polling one USB packet at a time.  The
                    # short timeout keeps commands responsive while greatly reducing
                    # Python/Win32 call overhead at the 1 kHz stream rate.
                    chunk = port.read(16_384)
                    now = time.monotonic()
                    if chunk:
                        last_data = now
                        frames = parser.feed(chunk)
                        output, session, total_bytes = self._consume_frames(
                            frames, parser, output, capture_path, session, total_bytes
                        )
                    elif now - last_data >= self.args.silence_timeout:
                        raise TimeoutError(
                            f"连续 {self.args.silence_timeout:.1f} 秒没有 IMU 数据"
                        )

                    if output is not None and now - last_sync >= self.args.sync_period:
                        output.flush()
                        os.fsync(output.fileno())
                        last_sync = now
                    if now - last_live >= 0.10:
                        self._post(
                            "live",
                            status=(
                                asdict(session.live_status)
                                if session is not None
                                else None
                            ),
                            thermal=asdict(self.thermal_monitor.status),
                            parser=asdict(parser.stats),
                            bytes=total_bytes,
                        )
                        last_live = now
        except Exception:
            if output is not None:
                self._save_interrupted(
                    session, parser.stats, capture_path, "USB 连接中断"
                )
                self._close_output(output)
                output = None
                self.restart_after_disconnect = self.record_requested
                self._post("session_interrupted", path=str(capture_path))
            raise
        finally:
            if output is not None:
                self._save_interrupted(session, parser.stats, capture_path, "程序关闭")
                self._close_output(output)

    def _consume_frames(
        self,
        frames: list[Any],
        parser: FlogStreamParser,
        output: Any | None,
        capture_path: Path | None,
        session: GuidedLmSession | None,
        total_bytes: int,
    ) -> tuple[Any | None, GuidedLmSession | None, int]:
        if output is not None and frames:
            raw_frames = b"".join(frame.raw for frame in frames)
            output.write(raw_frames)
            total_bytes += len(raw_frames)
        for frame in frames:
            thermal_status = None
            if frame.sample is not None:
                thermal_status = self.thermal_monitor.process(frame.sample)
            if session is None or frame.sample is None:
                continue
            if thermal_status is None or not thermal_status.stable:
                reason = (
                    thermal_status.reason
                    if thermal_status is not None
                    else "等待有效温度"
                )
                session.suspend_for_temperature(reason)
                continue
            accepted = session.process(frame.sample)
            if accepted is not None:
                self._post(
                    "pose_accepted",
                    pose=asdict(accepted),
                    status=asdict(session.live_status),
                )
            if session.state == "complete":
                assert output is not None and capture_path is not None
                report = session.build_report(self._copy_stats(parser.stats))
                report["capture_file"] = str(capture_path)
                report_path = capture_path.with_suffix(".guided_report.json")
                report_path.write_text(
                    json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8"
                )
                self._close_output(output)
                output = None
                session = None
                self.record_requested = False
                self._post(
                    "complete",
                    report=report,
                    report_path=str(report_path),
                    path=str(capture_path),
                )
        return output, session, total_bytes

    def _start_part(
        self, header: bytes, parser: FlogStreamParser, port_name: str
    ) -> tuple[Any, Path, GuidedLmSession]:
        if self.record_directory is None:
            raise RuntimeError("未设置采集输出目录")
        path, output = create_part_file(
            self.record_directory, port_name, self.part_number
        )
        self.part_number += 1
        output.write(header)
        output.flush()
        # Preserve an incomplete frame already buffered by the continuously running
        # parser.  Clearing it here starts in the middle of a frame and creates a
        # false resynchronization error at the beginning of every recording.
        parser.reset_statistics(clear_buffer=False)
        return output, path, GuidedLmSession()

    @staticmethod
    def _copy_stats(session_stats: ParserStats) -> ParserStats:
        return ParserStats(**asdict(session_stats))

    def _save_interrupted(
        self,
        session: GuidedLmSession | None,
        stats: ParserStats,
        capture_path: Path | None,
        reason: str,
    ) -> None:
        if capture_path is None:
            return
        artifact = {
            "format_version": 1,
            "overall_pass": False,
            "interrupted": True,
            "reason": reason,
            "capture_file": str(capture_path),
            "parser": asdict(stats),
            "completed_poses": (
                []
                if session is None
                else [asdict(item) for item in session.pose_results]
            ),
        }
        capture_path.with_suffix(".interrupted.json").write_text(
            json.dumps(artifact, indent=2, ensure_ascii=False), encoding="utf-8"
        )

    @staticmethod
    def _close_output(output: Any) -> None:
        output.flush()
        os.fsync(output.fileno())
        output.close()

    def _drain_commands(self) -> list[tuple[str, Any]]:
        commands: list[tuple[str, Any]] = []
        while True:
            try:
                commands.append(self.commands.get_nowait())
            except queue.Empty:
                return commands

    def _drain_offline_commands(self) -> None:
        for name, value in self._drain_commands():
            if name == "shutdown":
                self.shutdown_requested = True
            elif name == "stop":
                self.record_requested = False
                self.restart_after_disconnect = False
                self._post("session_stopped", path="")
            elif name == "start":
                self.record_requested = True
                self.record_directory = Path(value)
                self.restart_after_disconnect = False

    def _responsive_wait(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while not self.shutdown_requested and time.monotonic() < deadline:
            self._drain_offline_commands()
            time.sleep(0.05)

    def _post(self, event_type: str, **payload: Any) -> None:
        self.events.put({"type": event_type, **payload})


def run_lm_pipeline(capture_path: Path, events: queue.Queue[dict[str, Any]]) -> None:
    session_directory = capture_path.parent
    tool = Path(__file__).with_name("imu_calibration.py")
    guided_report_path = capture_path.with_suffix(".guided_report.json")
    csv_path = session_directory / "lm_capture.csv"
    json_path = session_directory / "bmi088_fixed_calibration.json"
    header_path = session_directory / "bmi088_fixed_calibration_generated.h"
    commands = (
        [sys.executable, str(tool), "extract", str(capture_path), "-o", str(csv_path)],
        [
            sys.executable,
            str(tool),
            "lm",
            str(csv_path),
            "-o",
            str(json_path),
            "--c-header",
            str(header_path),
            "--gyro-temperature-compensation",
            "--guided-report",
            str(guided_report_path),
        ],
    )
    try:
        for command in commands:
            subprocess.run(
                command, check=True, capture_output=True, text=True, encoding="utf-8"
            )
        events.put(
            {
                "type": "lm_complete",
                "json": str(json_path),
                "header": str(header_path),
                "csv": str(csv_path),
            }
        )
    except subprocess.CalledProcessError as exc:
        detail = (exc.stderr or exc.stdout or str(exc)).strip()
        events.put({"type": "lm_failed", "message": detail})


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-o", "--output-directory", type=Path, default=Path("output/guided_lm")
    )
    parser.add_argument("--port", help="固定 COM 端口；省略时按 VID/PID 自动枚举")
    parser.add_argument("--serial-number", help="多块 STM32 同时连接时指定 USB 序列号")
    parser.add_argument("--baudrate", type=int, default=115_200)
    parser.add_argument("--reconnect-delay", type=float, default=1.0)
    parser.add_argument("--silence-timeout", type=float, default=3.0)
    parser.add_argument("--sync-period", type=float, default=5.0)
    return parser


class GuidedLmApplication:
    def __init__(self, root: Any, args: argparse.Namespace) -> None:
        import tkinter as tk
        from tkinter import ttk

        import matplotlib

        matplotlib.rcParams["font.sans-serif"] = [
            "Microsoft YaHei",
            "SimHei",
            "DejaVu Sans",
        ]
        matplotlib.rcParams["axes.unicode_minus"] = False
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        from matplotlib.figure import Figure

        self.tk = tk
        self.ttk = ttk
        self.root = root
        self.args = args
        self.events: queue.Queue[dict[str, Any]] = queue.Queue()
        self.worker = UsbGuidedWorker(args, self.events)
        self.capture_path: Path | None = None
        self.session_directory: Path | None = None
        self.latest_status: dict[str, Any] | None = None
        self.latest_thermal: dict[str, Any] | None = None
        self.connected_at: float | None = None

        root.title("BMI088 引导式 LM 标定采集")
        root.geometry("1320x860")
        root.minsize(1120, 760)
        root.protocol("WM_DELETE_WINDOW", self.close)

        style = ttk.Style()
        style.configure("Title.TLabel", font=("Microsoft YaHei UI", 17, "bold"))
        style.configure("Target.TLabel", font=("Microsoft YaHei UI", 15, "bold"))
        style.configure(
            "Good.TLabel", foreground="#138a36", font=("Microsoft YaHei UI", 11, "bold")
        )
        style.configure(
            "Bad.TLabel", foreground="#c62f2f", font=("Microsoft YaHei UI", 11, "bold")
        )
        style.configure(
            "Info.TLabel", foreground="#1769aa", font=("Microsoft YaHei UI", 11, "bold")
        )

        outer = ttk.Frame(root, padding=10)
        outer.pack(fill="both", expand=True)
        header = ttk.Frame(outer)
        header.pack(fill="x")
        ttk.Label(header, text="BMI088 引导式 LM 数据采集", style="Title.TLabel").pack(
            side="left"
        )
        self.connection_label = ttk.Label(header, text="● 等待 USB", style="Bad.TLabel")
        self.connection_label.pack(side="right")

        warning = ttk.Label(
            outer,
            text=(
                "安全提示：拆桨、固定好 USB 线；BMI088 充分热稳定后再开始。"
                "四旋翼航向是示意值，LM 只根据重力判断倾斜与翻面。"
            ),
            foreground="#8a5600",
        )
        warning.pack(fill="x", pady=(7, 5))

        center = ttk.Panedwindow(outer, orient="horizontal")
        center.pack(fill="both", expand=True)
        plot_frame = ttk.LabelFrame(
            center, text="四旋翼姿态引导（左边当前，右边目标）", padding=4
        )
        control_frame = ttk.LabelFrame(center, text="实时判定", padding=10)
        center.add(plot_frame, weight=3)
        center.add(control_frame, weight=2)

        figure = Figure(figsize=(7.2, 5.2), dpi=100)
        self.quadrotor_view = QuadrotorView(figure)
        self.quadrotor_view.update(None, TARGET_POSES[0].direction)
        self.canvas = FigureCanvasTkAgg(figure, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        self.target_number = ttk.Label(
            control_frame, text="尚未开始", style="Target.TLabel"
        )
        self.target_number.pack(anchor="w")
        self.target_label = ttk.Label(
            control_frame, text=TARGET_POSES[0].label, wraplength=390
        )
        self.target_label.pack(anchor="w", pady=(4, 12))
        self.turn_label = ttk.Label(
            control_frame,
            text="等待实时姿态；四旋翼模型出现后再开始转动。",
            foreground="#5d4037",
            wraplength=410,
            font=("Microsoft YaHei UI", 11, "bold"),
        )
        self.turn_label.pack(anchor="w", pady=(0, 10))
        self.judgement_label = ttk.Label(
            control_frame, text="等待有效 IMU 数据", style="Info.TLabel", wraplength=390
        )
        self.judgement_label.pack(anchor="w", pady=(0, 10))

        self.hold_progress = ttk.Progressbar(control_frame, maximum=100.0)
        self.hold_progress.pack(fill="x")
        self.hold_label = ttk.Label(control_frame, text="保持 0.0 / 4.0 s")
        self.hold_label.pack(anchor="w", pady=(3, 10))

        self.metrics = {}
        for key, caption in (
            ("angle", "方向误差"),
            ("accel", "加速度模长"),
            ("accel_rms", "加速度抖动 RMS"),
            ("gyro", "陀螺仪 RMS"),
            ("gyro_bias", "零速均值偏差"),
            ("transition", "本次转场用时"),
            ("link", "USB 数据质量"),
            ("size", "本次数据量"),
            ("preheat", "温度 / 热稳定"),
            ("temp_budget", "整组温差预算"),
        ):
            row = ttk.Frame(control_frame)
            row.pack(fill="x", pady=2)
            ttk.Label(row, text=caption, width=14).pack(side="left")
            value = ttk.Label(row, text="--")
            value.pack(side="left")
            self.metrics[key] = value

        button_row = ttk.Frame(control_frame)
        button_row.pack(fill="x", pady=(16, 4))
        self.start_button = ttk.Button(
            button_row, text="开始 / 重新采集", command=self.start_session
        )
        self.start_button.pack(side="left", padx=(0, 6))
        self.stop_button = ttk.Button(
            button_row, text="停止", command=self.stop_session, state="disabled"
        )
        self.stop_button.pack(side="left", padx=6)
        self.lm_button = ttk.Button(
            button_row, text="运行 LM 标定", command=self.run_lm, state="disabled"
        )
        self.lm_button.pack(side="left", padx=6)

        self.output_label = ttk.Label(
            control_frame, text="输出目录：尚未创建", wraplength=390
        )
        self.output_label.pack(anchor="w", pady=(10, 0))

        bottom = ttk.LabelFrame(outer, text="18 个目标姿态与最终质量门槛", padding=5)
        bottom.pack(fill="x", pady=(8, 0))
        columns = ("index", "pose", "status")
        self.pose_table = ttk.Treeview(
            bottom, columns=columns, show="headings", height=7
        )
        self.pose_table.heading("index", text="#")
        self.pose_table.heading("pose", text="目标方向")
        self.pose_table.heading("status", text="状态")
        self.pose_table.column("index", width=40, anchor="center")
        self.pose_table.column("pose", width=360)
        self.pose_table.column("status", width=130, anchor="center")
        self.pose_table.pack(side="left", fill="both", expand=True)
        scrollbar = ttk.Scrollbar(
            bottom, orient="vertical", command=self.pose_table.yview
        )
        scrollbar.pack(side="left", fill="y")
        self.pose_table.configure(yscrollcommand=scrollbar.set)
        self.quality_text = tk.Text(
            bottom, width=48, height=8, state="disabled", font=("Consolas", 9)
        )
        self.quality_text.pack(side="left", fill="both", padx=(8, 0))
        self._reset_pose_table()
        self._write_quality(
            "最终会自动检查：\n姿态覆盖、温度有效性与整组温差、静态零偏漂移、\n"
            "三轴旋转激励、可用转场数量以及 USB CRC/丢帧/重同步。"
        )

        self.worker.start()
        root.after(50, self.poll_events)
        root.after(1000, self.update_clock)

    def start_session(self) -> None:
        if self.latest_thermal is None or not self.latest_thermal["stable"]:
            reason = (
                "尚未收到有效温度，请先刷入本次固件并等待。"
                if self.latest_thermal is None
                else self.latest_thermal["reason"]
            )
            self._write_quality(f"暂不开始 LM：{reason}")
            return
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        base = self.args.output_directory.resolve() / f"LM_GUIDED_{timestamp}"
        self.session_directory = base
        suffix = 1
        while self.session_directory.exists():
            self.session_directory = Path(f"{base}_{suffix:02d}")
            suffix += 1
        self.session_directory.mkdir(parents=True)
        self.capture_path = None
        self.worker.command("start", self.session_directory)
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self.lm_button.configure(state="disabled")
        self._reset_pose_table()
        self._write_quality("正在等待会话文件开始；请先保持飞控静止。")
        self.output_label.configure(text=f"输出目录：{self.session_directory}")

    def stop_session(self) -> None:
        self.worker.command("stop")
        self.stop_button.configure(state="disabled")
        self.start_button.configure(state="normal")

    def run_lm(self) -> None:
        if self.capture_path is None:
            return
        self.lm_button.configure(state="disabled")
        self._write_quality("质量门槛已通过，正在提取 CSV 并执行 LM……")
        threading.Thread(
            target=run_lm_pipeline,
            args=(self.capture_path, self.events),
            name="bmi088-lm",
            daemon=True,
        ).start()

    def poll_events(self) -> None:
        pending_live: dict[str, Any] | None = None
        try:
            while True:
                event = self.events.get_nowait()
                if event["type"] == "live":
                    pending_live = event
                    continue
                if pending_live is not None:
                    self._handle_event(pending_live)
                    pending_live = None
                self._handle_event(event)
        except queue.Empty:
            pass
        if pending_live is not None:
            self._handle_event(pending_live)
        if self.root.winfo_exists():
            self.root.after(50, self.poll_events)

    def _handle_event(self, event: dict[str, Any]) -> None:
        event_type = event["type"]
        if event_type == "connection":
            if event.get("connected"):
                self.connection_label.configure(
                    text=f"● {event['message']}", style="Good.TLabel"
                )
                self.connected_at = time.monotonic()
            else:
                self.connection_label.configure(
                    text=f"● {event['message']}", style="Bad.TLabel"
                )
                self.connected_at = None
                self.latest_thermal = None
        elif event_type in ("session_started", "session_restarted"):
            self.capture_path = Path(event["path"])
            self.start_button.configure(state="normal")
            self.stop_button.configure(state="normal")
            self._reset_pose_table()
            if event_type == "session_restarted":
                self._write_quality(
                    "检测到 USB 重连：上一份数据已标记中断，本次已自动从姿态 1 重新开始。"
                )
            else:
                self._write_quality("采集已开始。完成每个姿态后再按界面顺序转动。")
        elif event_type == "session_interrupted":
            self._write_quality(
                "USB 中断：当前 BIN 不会用于 LM；自动重连后会从第 1 个姿态重采。"
            )
        elif event_type == "session_stopped":
            self.start_button.configure(state="normal")
            self.stop_button.configure(state="disabled")
            self._write_quality("采集已停止；未完成的文件不会自动运行 LM。")
        elif event_type == "live":
            self.latest_status = event["status"]
            self.latest_thermal = event["thermal"]
            if event["status"] is not None:
                self._update_live(event["status"], event["parser"], event["bytes"])
            self._update_thermal(event["thermal"])
        elif event_type == "pose_accepted":
            self._update_pose_rows(event["status"])
        elif event_type == "complete":
            self.capture_path = Path(event["path"])
            self._show_report(event["report"])
        elif event_type == "lm_complete":
            self._write_quality(
                "LM 已完成。\nJSON: " + event["json"] + "\nC 头文件: " + event["header"]
            )
        elif event_type == "lm_failed":
            self.lm_button.configure(state="normal")
            self._write_quality("LM 执行失败：\n" + event["message"])
        elif event_type == "fatal":
            self.connection_label.configure(text="● 上位机错误", style="Bad.TLabel")
            self._write_quality(event["message"])

    def _update_live(
        self, status: dict[str, Any], parser: dict[str, Any], total_bytes: int
    ) -> None:
        completed = len(status["completed_codes"])
        if status["state"] == "complete":
            self.target_number.configure(text="18 / 18 已完成")
        else:
            self.target_number.configure(
                text=f"目标 {status['target_index'] + 1} / {status['target_count']}"
            )
        self.target_label.configure(text=status["target_label"])
        self.judgement_label.configure(
            text=status["reason"],
            style="Good.TLabel" if status["qualified"] else "Bad.TLabel",
        )
        instruction = turn_instruction(
            status["current_direction"], status["target_direction"]
        )
        transition_seconds = status["transition_seconds"]
        recommended_transition_seconds = status["transition_recommended_seconds"]
        maximum_transition_seconds = status["transition_maximum_seconds"]
        if transition_seconds > maximum_transition_seconds:
            instruction = (
                f"本次转场已超过 {maximum_transition_seconds:.0f} 秒，"
                "姿态仍用于加速度计，但该转场不计入陀螺仪矩阵标定。" + instruction
            )
        elif transition_seconds > recommended_transition_seconds:
            instruction = (
                f"已超过建议的 {recommended_transition_seconds:.0f} 秒，"
                f"但在 {maximum_transition_seconds:.0f} 秒内完成仍可用于陀螺仪 LM。"
                + instruction
            )
        if status["temperature_budget_warning"]:
            instruction = (
                f"整组温差已用 {status['pose_temperature_span_c']:.2f} / 1.00°C；"
                "请尽快完成，预算耗尽后需在当前温度重新开始。"
                + instruction
            )
        self.turn_label.configure(text=instruction)
        self.hold_progress["value"] = 100.0 * status["hold_fraction"]
        self.hold_label.configure(
            text=(
                f"累计合格保持 {status['hold_seconds']:.1f} / "
                f"{status['hold_required_seconds']:.1f} s"
            )
        )
        self.metrics["angle"].configure(
            text=self._finite_text(status["angle_error_deg"], "{:.1f}°（要求 ≤ 8°）")
        )
        self.metrics["accel"].configure(
            text=self._finite_text(status["accel_norm_mps2"], "{:.3f} m/s²")
        )
        self.metrics["accel_rms"].configure(
            text=self._finite_text(
                status["accel_rms_deviation_mps2"],
                "{:.3f} m/s²（要求 < 0.120）",
            )
        )
        self.metrics["gyro"].configure(
            text=self._finite_text(
                status["gyro_rms_rps"], "{:.4f} rad/s（要求 < 0.035）"
            )
        )
        self.metrics["gyro_bias"].configure(
            text=self._finite_text(
                math.degrees(status["gyro_bias_residual_rps"]),
                "{:.3f} deg/s（仅提示，不拦截）",
            )
        )
        budget_color = "#c66a00" if status["temperature_budget_warning"] else "#138a36"
        if status["temperature_budget_remaining_c"] <= 0.0:
            budget_color = "#c62f2f"
        self.metrics["temp_budget"].configure(
            text=(
                f"已用 {status['pose_temperature_span_c']:.2f} / 1.00°C｜"
                f"剩余 {status['temperature_budget_remaining_c']:.2f}°C"
            ),
            foreground=budget_color,
        )
        self.metrics["transition"].configure(
            text=(
                f"{transition_seconds:.1f} s"
                f"（建议 ≤ {recommended_transition_seconds:.0f}，"
                f"上限 {maximum_transition_seconds:.0f}）"
            )
        )
        link_ok = all(
            parser[name] == 0
            for name in (
                "crc_errors",
                "resync_bytes",
                "sequence_gaps",
                "duplicate_or_backward_frames",
                "sample_sequence_gaps",
                "duplicate_or_backward_samples",
                "timestamp_nonmonotonic",
                "timestamp_interval_outliers",
            )
        )
        received_samples = parser["imu_samples"]
        missing_samples = parser["sample_sequence_gaps"]
        expected_samples = received_samples + missing_samples
        loss_percent = 100.0 * missing_samples / max(expected_samples, 1)
        self.metrics["link"].configure(
            text=(
                f"{'合格' if link_ok else '不合格'}｜丢包 {loss_percent:.3f}%｜丢样 "
                f"{parser['sample_sequence_gaps']}｜CRC {parser['crc_errors']}｜"
                f"重同步 {parser['resync_bytes']}｜时序 "
                f"{parser['timestamp_interval_outliers']}"
            )
        )
        self.metrics["size"].configure(text=f"{total_bytes / 1048576.0:.2f} MiB")
        visual_changed = self.quadrotor_view.update(
            status["current_direction"], status["target_direction"]
        )
        if visual_changed:
            self.canvas.draw_idle()
        self._update_pose_rows(status)
        if completed < len(TARGET_POSES):
            self.pose_table.see(str(min(status["target_index"], len(TARGET_POSES) - 1)))

    def _reset_pose_table(self) -> None:
        for item in self.pose_table.get_children():
            self.pose_table.delete(item)
        for index, pose in enumerate(TARGET_POSES):
            self.pose_table.insert(
                "", "end", iid=str(index), values=(index + 1, pose.label, "等待")
            )

    def _update_pose_rows(self, status: dict[str, Any]) -> None:
        completed = set(status["completed_codes"])
        for index, pose in enumerate(TARGET_POSES):
            if pose.code in completed:
                state = "✓ 已记录"
            elif index == status["target_index"] and status["state"] == "running":
                state = "▶ 当前目标"
            else:
                state = "等待"
            self.pose_table.set(str(index), "status", state)

    def _show_report(self, report: dict[str, Any]) -> None:
        passed = report["overall_pass"]
        lines = ["整组判定：" + ("通过" if passed else "不通过")]
        labels = {
            "pose_count": "姿态数量",
            "coverage_min_eigenvalue": "球面覆盖",
            "gyro_bias_drift_dps": "零偏漂移",
            "temperature_valid": "温度有效姿态",
            "pose_temperature_span_c": "整组温度跨度",
            "usable_rotation_transitions": "可用转场",
            "rotation_excitation_each_axis_deg": "三轴旋转激励",
            "usb_link_errors": "USB 完整性",
        }
        for check in report["checks"]:
            mark = "✓" if check["pass"] else "✗"
            lines.append(
                f"{mark} {labels.get(check['name'], check['name'])}: {check['value']} ({check['requirement']})"
            )
        self._write_quality("\n".join(lines))
        self.start_button.configure(state="normal")
        self.stop_button.configure(state="disabled")
        self.lm_button.configure(state="normal" if passed else "disabled")

    def _write_quality(self, content: str) -> None:
        self.quality_text.configure(state="normal")
        self.quality_text.delete("1.0", "end")
        self.quality_text.insert("1.0", content)
        self.quality_text.configure(state="disabled")

    def update_clock(self) -> None:
        if self.connected_at is None:
            self.metrics["preheat"].configure(text="未连接")
        elif self.latest_thermal is None or not self.latest_thermal["valid"]:
            elapsed = time.monotonic() - self.connected_at
            self.metrics["preheat"].configure(
                text=f"等待有效温度｜已连接 {elapsed / 60.0:.1f} min"
            )
        if self.root.winfo_exists():
            self.root.after(1000, self.update_clock)

    def _update_thermal(self, thermal: dict[str, Any]) -> None:
        if not thermal["valid"]:
            self.metrics["preheat"].configure(
                text=thermal["reason"], foreground="#c62f2f"
            )
            return
        state = "稳定，可采集" if thermal["stable"] else "未稳定"
        color = "#138a36" if thermal["stable"] else "#c66a00"
        self.metrics["preheat"].configure(
            text=(
                f"{thermal['current_c']:.2f}°C｜{thermal['slope_c_per_min']:+.2f}°C/min｜"
                f"{state}"
            ),
            foreground=color,
        )
        if not thermal["stable"]:
            self.turn_label.configure(text=thermal["reason"], foreground="#c66a00")

    @staticmethod
    def _finite_text(value: float, template: str) -> str:
        return template.format(value) if math.isfinite(value) else "--"

    def close(self) -> None:
        if getattr(self, "closing", False):
            return
        self.closing = True
        self.worker.command("shutdown")
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="disabled")
        self.lm_button.configure(state="disabled")
        self.connection_label.configure(
            text="● 正在安全关闭文件……", style="Info.TLabel"
        )
        self.close_deadline = time.monotonic() + 7.0
        self._wait_for_worker_close()

    def _wait_for_worker_close(self) -> None:
        if self.worker.is_alive() and time.monotonic() < self.close_deadline:
            self.root.after(100, self._wait_for_worker_close)
            return
        self.root.destroy()


def main() -> int:
    args = build_parser().parse_args()
    if args.reconnect_delay <= 0 or args.silence_timeout <= 0 or args.sync_period <= 0:
        print("重连、静默超时和同步周期必须大于 0", file=sys.stderr)
        return 2
    try:
        import tkinter as tk
    except ModuleNotFoundError:
        print("当前 Python 未安装 Tk，请安装带 Tcl/Tk 的 Python 3。", file=sys.stderr)
        return 2
    root = tk.Tk()
    GuidedLmApplication(root, args)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
