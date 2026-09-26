#!/usr/bin/env python3
"""Build a bounded Data Analytics artifact from a guided BMI088 LM session."""

from __future__ import annotations

import argparse
import json
import math
import sqlite3
from datetime import datetime
from pathlib import Path


def source(
    source_id: str, label: str, path: str, description: str, sql: str, table: str
) -> dict:
    return {
        "id": source_id,
        "label": label,
        "path": path,
        "query": {
            "engine": "sqlite",
            "language": "sql",
            "sql": sql,
            "description": description,
            "tables_used": [table],
            "filters": [],
            "metric_definitions": {},
        },
    }


def write_sqlite_table(connection: sqlite3.Connection, name: str, rows: list[dict]) -> None:
    if not rows:
        return
    columns = list(rows[0])
    types = []
    for column in columns:
        value = rows[0][column]
        types.append("INTEGER" if isinstance(value, int) else "REAL" if isinstance(value, float) else "TEXT")
    connection.execute(f'DROP TABLE IF EXISTS "{name}"')
    definitions = ", ".join(f'"{column}" {kind}' for column, kind in zip(columns, types))
    connection.execute(f'CREATE TABLE "{name}" ({definitions})')
    placeholders = ", ".join("?" for _ in columns)
    connection.executemany(
        f'INSERT INTO "{name}" VALUES ({placeholders})',
        [[row[column] for column in columns] for row in rows],
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    session = args.session_dir.resolve()
    guided_path = session / "BMI088_USB_20260802_223050_COM11_part001.patched_plus_z.guided_report.json"
    validation_path = session / "lm_validation_patched_plus_z.json"
    calibration_path = session / "bmi088_fixed_calibration_patched_plus_z.json"
    guided = json.loads(guided_path.read_text(encoding="utf-8"))
    validation = json.loads(validation_path.read_text(encoding="utf-8"))
    calibration = json.loads(calibration_path.read_text(encoding="utf-8"))

    root = Path.cwd().resolve()
    poses = []
    for index, pose in enumerate(guided["poses"], 1):
        accel = pose["mean_accel_mps2"]
        gyro = pose["mean_gyro_rps"]
        poses.append({
            "pose_index": index,
            "pose": pose["code"],
            "angle_error_deg": pose["angle_error_deg"],
            "temperature_c": pose["mean_temperature_c"],
            "accel_norm_mps2": math.sqrt(sum(value * value for value in accel)),
            "gyro_mean_norm_dps": math.degrees(math.sqrt(sum(value * value for value in gyro))),
            "samples": pose["samples"],
        })

    lm = validation["lm"]
    accel_fit = [
        {"phase": "标定前", "rmse_mps2": lm["accel_norm_rmse_before_mps2"], "metric": "加速度模长 RMSE"},
        {"phase": "标定后", "rmse_mps2": lm["accel_norm_rmse_after_mps2"], "metric": "加速度模长 RMSE"},
    ]
    gyro_fit = [
        {"phase": "标定前", "mean_angle_deg": lm["gyro_endpoint_angle_before_deg"]["mean"], "max_angle_deg": lm["gyro_endpoint_angle_before_deg"]["max"], "metric": "陀螺终点角误差"},
        {"phase": "标定后", "mean_angle_deg": lm["gyro_endpoint_angle_after_deg"]["mean"], "max_angle_deg": lm["gyro_endpoint_angle_after_deg"]["max"], "metric": "陀螺终点角误差"},
    ]
    excitation_values = next(check["value"] for check in guided["checks"] if check["name"] == "rotation_excitation_each_axis_deg")
    excitation = [
        {"axis": axis, "excitation_deg": value, "threshold_deg": 180.0}
        for axis, value in zip(("X", "Y", "Z"), excitation_values)
    ]
    transitions = []
    for index, item in enumerate(guided["transitions"], 1):
        transitions.append({
            "transition_index": index,
            "transition": f"{item['from_code']} → {item['to_code']}",
            "duration_s": item["duration_s"],
            "endpoint_angle_deg": item["endpoint_angle_deg"],
            "integrated_angle_deg": item["integrated_angle_deg"],
            "usable": "使用" if item["usable"] else "排除",
        })

    checks = {item["name"]: item["value"] for item in guided["checks"]}
    usb_errors = sum(checks["usb_link_errors"].values())
    headline = [{
        "pose_count": checks["pose_count"],
        "gyro_bias_drift_dps": checks["gyro_bias_drift_dps"],
        "temperature_span_c": checks["pose_temperature_span_c"],
        "usable_transitions": checks["usable_rotation_transitions"],
        "usb_errors": usb_errors,
        "sample_count": guided["metrics"]["parser"]["imu_samples"],
    }]
    gyro_bias_dps = [math.degrees(value) for value in calibration["gyroscope"]["bias"]]
    calibration_rows = [
        {"sensor": "加速度计", "axis": axis, "bias": value, "unit": "m/s²"}
        for axis, value in zip(("X", "Y", "Z"), calibration["accelerometer"]["bias"])
    ] + [
        {"sensor": "陀螺仪", "axis": axis, "bias": value, "unit": "°/s"}
        for axis, value in zip(("X", "Y", "Z"), gyro_bias_dps)
    ]

    headline[0].update({"pose_target": 18, "drift_limit_dps": 0.05, "temperature_limit_c": 1.0, "transition_minimum": 12})

    datasets = {
        "headline": headline,
        "poses": poses,
        "accel_fit": accel_fit,
        "gyro_fit": gyro_fit,
        "excitation": excitation,
        "transitions": transitions,
        "calibration_bias": calibration_rows,
    }
    sqlite_path = args.output.with_suffix(".sqlite")
    sqlite_path.parent.mkdir(parents=True, exist_ok=True)
    with sqlite3.connect(sqlite_path) as connection:
        for name, rows in datasets.items():
            write_sqlite_table(connection, name, rows)
        connection.commit()
    sqlite_rel = sqlite_path.resolve().relative_to(root).as_posix()
    sources = [
        source("guided", "姿态与温度分析快照", sqlite_rel, "读取 18 个姿态的验收误差、温度和静止统计。", "SELECT * FROM poses ORDER BY pose_index", "poses"),
        source("validation", "LM 前后误差分析快照", sqlite_rel, "读取加速度与陀螺 LM 前后误差。", "SELECT a.phase, a.rmse_mps2, g.mean_angle_deg, g.max_angle_deg FROM accel_fit a JOIN gyro_fit g USING (phase) ORDER BY CASE a.phase WHEN '标定前' THEN 1 ELSE 2 END", "accel_fit, gyro_fit"),
        source("excitation", "三轴激励分析快照", sqlite_rel, "读取三轴累计旋转激励和最低门槛。", "SELECT * FROM excitation ORDER BY axis", "excitation"),
        source("transitions", "转场分析快照", sqlite_rel, "读取全部 17 个转场及其 LM 使用状态。", "SELECT * FROM transitions ORDER BY transition_index", "transitions"),
        source("calibration", "最终固定偏置分析快照", sqlite_rel, "读取最终加速度计与陀螺仪固定偏置。", "SELECT * FROM calibration_bias ORDER BY sensor, axis", "calibration_bias"),
    ]

    charts = [
        {"id": "pose_angle", "title": "18 个姿态的方向误差", "description": "所有姿态均低于 8° 验收线；最大误差来自 +Y+Z。", "type": "bar", "intent": "comparison", "question": "每个静止姿态距离目标方向有多远？", "rationale": "按姿态比较单一角误差，柱状图最直接。", "dataset": "poses", "sourceId": "guided", "encodings": {"x": {"field": "pose"}, "y": {"field": "angle_error_deg"}}, "palette": {"kind": "single", "name": "blue"}, "legend": {"show": False}, "labels": {"values": "all"}, "referenceLines": [{"axis": "y", "value": 8.0, "label": "验收上限 8°", "lineStyle": "dashed", "color": "neutral"}]},
        {"id": "pose_temperature", "title": "采集顺序中的姿态温度", "description": "18 个姿态均值为 46.63–47.36°C，整组跨度 0.73°C。", "type": "line", "intent": "trend", "question": "整组采集过程中温度是否保持在窄区间？", "rationale": "按采集顺序的连续温度序列适合折线图。", "dataset": "poses", "sourceId": "guided", "encodings": {"x": {"field": "pose_index"}, "y": {"field": "temperature_c"}}, "palette": {"kind": "single", "name": "orange"}, "legend": {"show": False}, "labels": {"values": "endpoints"}},
        {"id": "accel_fit", "title": "加速度计 LM 模长残差", "description": "RMSE 从 0.1053 降到 0.00217 m/s²，下降约 97.9%。", "type": "bar", "intent": "comparison", "question": "加速度计固定参数标定降低了多少模长误差？", "rationale": "两个阶段的同单位绝对值适合柱状比较。", "dataset": "accel_fit", "sourceId": "validation", "encodings": {"x": {"field": "phase"}, "y": {"field": "rmse_mps2"}}, "palette": {"kind": "single", "name": "blue"}, "legend": {"show": False}, "labels": {"values": "all"}},
        {"id": "gyro_fit", "title": "陀螺仪 LM 终点平均角误差", "description": "平均误差从 1.254° 降到 0.578°，最大残余误差 1.542°。", "type": "bar", "intent": "comparison", "question": "陀螺矩阵标定降低了多少终点重力方向误差？", "rationale": "标定前后两个阶段适合直接比较。", "dataset": "gyro_fit", "sourceId": "validation", "encodings": {"x": {"field": "phase"}, "y": {"field": "mean_angle_deg"}}, "palette": {"kind": "single", "name": "gold"}, "legend": {"show": False}, "labels": {"values": "all"}},
        {"id": "axis_excitation", "title": "三轴累计旋转激励", "description": "X/Y/Z 分别达到 4691°、4882°、3509°，均远高于 180° 下限。", "type": "bar", "intent": "comparison", "question": "每个轴是否获得足够旋转激励？", "rationale": "三轴同单位总量适合柱状比较并叠加阈值线。", "dataset": "excitation", "sourceId": "excitation", "encodings": {"x": {"field": "axis"}, "y": {"field": "excitation_deg"}}, "palette": {"kind": "single", "name": "olive"}, "legend": {"show": False}, "labels": {"values": "all"}, "referenceLines": [{"axis": "y", "value": 180.0, "label": "最低 180°", "lineStyle": "dashed", "color": "neutral"}]},
    ]

    tables = [
        {"id": "pose_detail", "title": "姿态级质量明细", "description": "按采集顺序列出方向误差、温度、加速度模长和静止陀螺均值。", "dataset": "poses", "sourceId": "guided", "columns": [{"field": "pose_index", "label": "#", "type": "number"}, {"field": "pose", "label": "姿态", "type": "string"}, {"field": "angle_error_deg", "label": "方向误差 (°)", "type": "number"}, {"field": "temperature_c", "label": "温度 (°C)", "type": "number"}, {"field": "accel_norm_mps2", "label": "加速度模长 (m/s²)", "type": "number"}, {"field": "gyro_mean_norm_dps", "label": "陀螺均值模长 (°/s)", "type": "number"}, {"field": "samples", "label": "样本数", "type": "number"}], "defaultSort": {"field": "pose_index", "direction": "asc"}},
        {"id": "transition_detail", "title": "转场使用明细", "description": "+X→+Z 与 +Z→+Y 因 +Z 单独补采而排除，其余 15 段用于陀螺 LM。", "dataset": "transitions", "sourceId": "transitions", "columns": [{"field": "transition_index", "label": "#", "type": "number"}, {"field": "transition", "label": "转场", "type": "string"}, {"field": "duration_s", "label": "时长 (s)", "type": "number"}, {"field": "integrated_angle_deg", "label": "积分角 (°)", "type": "number"}, {"field": "endpoint_angle_deg", "label": "端点夹角 (°)", "type": "number"}, {"field": "usable", "label": "LM 状态", "type": "string"}], "defaultSort": {"field": "transition_index", "direction": "asc"}},
        {"id": "calibration_bias", "title": "最终固定偏置", "description": "运行时按 corrected = matrix × (measured − bias) 使用。", "dataset": "calibration_bias", "sourceId": "calibration", "columns": [{"field": "sensor", "label": "传感器", "type": "string"}, {"field": "axis", "label": "轴", "type": "string"}, {"field": "bias", "label": "偏置", "type": "number"}, {"field": "unit", "label": "单位", "type": "string"}], "defaultSort": {"field": "sensor", "direction": "asc"}},
    ]

    blocks = [
        {"id": "title", "type": "markdown", "body": "# BMI088 补采 +Z 后的 LM 数据分析"},
        {"id": "summary", "type": "markdown", "sourceId": "validation", "body": "## 技术结论\n\n- **本轮数据可用于固定参数标定。** 18 个姿态全部通过，USB 错误为 0，补采后的最大零偏漂移为 0.0249 °/s。\n- **LM 改善显著。** 加速度模长 RMSE 下降约 97.9%；陀螺终点平均角误差下降约 53.9%。\n- **旋转激励充分。** 排除补采姿态两侧不连续的旧转场后仍有 15 段，三轴累计激励均超过最低要求 19 倍。\n- **主要限制是温度外推。** 本轮姿态均温 46.63–47.36°C，而当前温漂模型在 46°C 以上钳位，因此该组固定偏置最代表热稳态工作区。"},
        {"id": "pose_heading", "type": "markdown", "body": "## 18 个姿态均通过，但斜姿态误差明显更大\n\n六个主轴姿态的方向误差总体较小；误差较大的点集中在手持斜姿态，最大仍低于 8° 门槛。它们不会让本组失效，但说明下轮若追求更高重复性，应优先改善斜姿态支撑稳定度。"},
        {"id": "pose_angle_block", "type": "chart", "chartId": "pose_angle"},
        {"id": "pose_table_block", "type": "table", "tableId": "pose_detail"},
        {"id": "temperature_heading", "type": "markdown", "sourceId": "guided", "body": "## 温度跨度合格，但这是一组热稳态参数\n\n姿态均值温度跨度为 0.730°C，低于 1.0°C 上限。+Z 补采点温度最高，仍未破坏整组一致性。由于采集温度略高于温漂模型 46°C 的钳位上界，冷启动至升温阶段仍需依靠温漂补偿和在线零偏估计。"},
        {"id": "temperature_block", "type": "chart", "chartId": "pose_temperature"},
        {"id": "lm_heading", "type": "markdown", "sourceId": "validation", "body": "## LM 同时改善了加速度几何关系和陀螺积分一致性\n\n加速度计拟合后的残差已接近毫重力量级；陀螺矩阵将平均终点方向误差压低到 0.578°。这说明比例因子、非正交和交叉轴项获得了有效约束，而不是仅仅重新估计一个静止零偏。"},
        {"id": "accel_block", "type": "chart", "chartId": "accel_fit"},
        {"id": "gyro_block", "type": "chart", "chartId": "gyro_fit"},
        {"id": "rotation_heading", "type": "markdown", "sourceId": "transitions", "body": "## 15 段连续转场仍足以约束陀螺矩阵\n\n单独补采 +Z 后，原始的 +X→+Z 和 +Z→+Y 不再是同一次连续运动，因此正确做法是排除而不是拼接。保留的 15 段仍提供了远高于门槛的三轴激励。"},
        {"id": "excitation_block", "type": "chart", "chartId": "axis_excitation"},
        {"id": "transition_table_block", "type": "table", "tableId": "transition_detail"},
        {"id": "scope", "type": "markdown", "body": "## 数据范围与指标定义\n\n分析对象为补采并替换 +Z 后的 18 姿态 LM 数据集，共 470,934 个 1 kHz 样本。方向误差是每个静止段平均加速度方向与目标重力方向的夹角；零偏漂移是六个主轴姿态静止陀螺均值的最大轴向跨度；陀螺终点误差是转场角速度积分预测重力方向与下一静止姿态重力方向的夹角。"},
        {"id": "method", "type": "markdown", "body": "## 方法与稳健性处理\n\n加速度计使用 18 个静止姿态约束校准矩阵与偏置；陀螺仪使用 15 段连续转场，在 250 Hz 优化速率下拟合 3×3 矩阵与固定偏置。USB CRC、帧序号、样本序号和时间戳完整性均为零异常。补采数据只替换原 +Z 静止区间，并保留原始时间戳和序号。"},
        {"id": "limitations", "type": "markdown", "body": "## 限制与不确定性\n\n本报告证明的是这组数据足以支撑当前温度附近的固定参数标定，并不证明全温区偏置已完全解决。46°C 以上温漂补偿被钳位，低温和快速升温阶段需要后续多温度验证。手持斜姿态的姿态误差也高于主轴姿态，因此重复采集时使用泡棉支撑或简易夹具会提高重复性。"},
        {"id": "bias_heading", "type": "markdown", "body": "## 最终参数与下一步\n\n下面列出最终固定偏置。下一步应把生成头文件接入固件并做六面静止复测；通过后，再对校准后的长时间静止数据运行 Allan 分析，为 ESKF 配置白噪声和零偏随机游走参数。"},
        {"id": "bias_table_block", "type": "table", "tableId": "calibration_bias"},
        {"id": "questions", "type": "markdown", "body": "## 仍需回答的问题\n\n1. 参数写入固件后，六个主轴的校准后加速度模长和静止角速度是否仍满足门槛？\n2. 20–50°C 范围内固定偏置随温度的残余曲线是否需要扩大模型有效区间？\n3. 校准后长时间静止数据的 Allan 白噪声、零偏不稳定性和随机游走分别是多少？"},
    ]

    artifact = {
        "surface": "report",
        "manifest": {
            "version": 1,
            "surface": "report",
            "title": "BMI088 补采 +Z 后的 LM 数据分析",
            "description": "补采倒扣 +Z 后的采集质量、温度一致性、旋转激励与 LM 前后误差分析。",
            "generatedAt": datetime.now().astimezone().isoformat(),
            "blocks": blocks,
            "charts": charts,
            "tables": tables,
            "sources": sources,
        },
        "snapshot": {
            "version": 1,
            "status": "ready",
            "generatedAt": datetime.now().astimezone().isoformat(),
            "datasets": datasets,
        },
        "sources": sources,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(artifact, indent=2, ensure_ascii=False), encoding="utf-8")
    print(args.output)


if __name__ == "__main__":
    main()
