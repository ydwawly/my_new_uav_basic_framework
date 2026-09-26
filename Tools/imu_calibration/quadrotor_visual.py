#!/usr/bin/env python3
"""Gravity-based quadrotor visualization helpers for guided LM capture."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import matplotlib
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


WORLD_UP = np.array([0.0, 0.0, 1.0], dtype=np.float64)
BODY_UP = np.array([0.0, 0.0, -1.0], dtype=np.float64)
BODY_DOWN = -BODY_UP

# Windows normally provides Microsoft YaHei; SimHei is a common fallback on
# engineering workstations. DejaVu Sans remains last for non-CJK glyphs.
matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False


def normalized_direction(
    direction: list[float] | tuple[float, ...] | np.ndarray
) -> np.ndarray | None:
    vector = np.asarray(direction, dtype=np.float64)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        return None
    length = float(np.linalg.norm(vector))
    if length < 1e-9:
        return None
    return vector / length


def orientation_from_specific_force(
    direction: list[float] | tuple[float, ...] | np.ndarray
) -> np.ndarray:
    """Return a body-to-world rotation whose body gravity direction points up.

    Accelerometer-only LM data cannot observe yaw. The returned yaw is therefore a
    deterministic display convention chosen to keep the body X axis close to world X.
    """

    gravity_body = normalized_direction(direction)
    if gravity_body is None:
        raise ValueError("specific-force direction must be a finite non-zero 3-vector")

    candidates = np.eye(3, dtype=np.float64)
    reference = min(candidates, key=lambda axis: abs(float(np.dot(axis, gravity_body))))
    world_x_body = reference - float(np.dot(reference, gravity_body)) * gravity_body
    world_x_body /= np.linalg.norm(world_x_body)
    world_y_body = np.cross(gravity_body, world_x_body)
    world_to_body = np.column_stack((world_x_body, world_y_body, gravity_body))
    return world_to_body.T


def turn_instruction(
    current_direction: list[float] | tuple[float, ...] | np.ndarray,
    target_direction: list[float] | tuple[float, ...] | np.ndarray,
    tolerance_deg: float = 8.0,
) -> str:
    current = normalized_direction(current_direction)
    target = normalized_direction(target_direction)
    if current is None:
        return "等待实时姿态；四旋翼模型出现后再开始转动。"
    if target is None:
        return "目标姿态无效。"

    dot = float(np.clip(np.dot(current, target), -1.0, 1.0))
    angle_deg = math.degrees(math.acos(dot))
    if angle_deg <= tolerance_deg:
        return f"姿态已对准（误差 {angle_deg:.1f}°），放稳并保持不动。"

    # Body rotation changes the gravity vector expressed in body coordinates in
    # the opposite direction, hence the minus sign.
    body_axis = -np.cross(current, target)
    axis_length = float(np.linalg.norm(body_axis))
    if axis_length < 1e-6:
        reference = np.array([1.0, 0.0, 0.0])
        if abs(float(np.dot(reference, current))) > 0.85:
            reference = np.array([0.0, 1.0, 0.0])
        body_axis = reference - float(np.dot(reference, current)) * current
        body_axis /= np.linalg.norm(body_axis)
    else:
        body_axis /= axis_length

    dominant = int(np.argmax(np.abs(body_axis)))
    sign = 1.0 if body_axis[dominant] >= 0.0 else -1.0
    descriptions = {
        (0, 1.0): "向右滚转（右侧机臂下压）",
        (0, -1.0): "向左滚转（左侧机臂下压）",
        (1, 1.0): "抬机头",
        (1, -1.0): "压机头",
        (2, 1.0): "主要绕机体 +Z 轴按右手定则转",
        (2, -1.0): "主要绕机体 -Z 轴按右手定则转",
    }
    axis_name = "XYZ"[dominant]
    sign_text = "+" if sign > 0.0 else "-"
    return (
        f"建议：{descriptions[(dominant, sign)]}，约 {angle_deg:.0f}°；"
        f"对应绕机体 {sign_text}{axis_name} 轴。接近目标后慢慢停稳。"
    )


def _ellipsoid_faces(
    center: np.ndarray,
    radii: tuple[float, float, float],
    longitude_count: int = 14,
    latitude_count: int = 7,
) -> list[np.ndarray]:
    longitude = np.linspace(0.0, 2.0 * math.pi, longitude_count, endpoint=False)
    latitude = np.linspace(-0.5 * math.pi, 0.5 * math.pi, latitude_count)
    rings = []
    for angle in latitude:
        rings.append(
            center
            + np.column_stack(
                (
                    radii[0] * math.cos(angle) * np.cos(longitude),
                    radii[1] * math.cos(angle) * np.sin(longitude),
                    np.full(longitude_count, radii[2] * math.sin(angle)),
                )
            )
        )
    faces: list[np.ndarray] = []
    for row in range(latitude_count - 1):
        for column in range(longitude_count):
            following = (column + 1) % longitude_count
            faces.append(
                np.array(
                    [
                        rings[row][column],
                        rings[row][following],
                        rings[row + 1][following],
                        rings[row + 1][column],
                    ]
                )
            )
    return faces


def _tube_faces(
    start: np.ndarray,
    end: np.ndarray,
    start_radius: float,
    end_radius: float,
    sides: int = 10,
) -> list[np.ndarray]:
    direction = end - start
    direction /= np.linalg.norm(direction)
    reference = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(reference, direction))) > 0.9:
        reference = np.array([0.0, 1.0, 0.0])
    normal_a = np.cross(direction, reference)
    normal_a /= np.linalg.norm(normal_a)
    normal_b = np.cross(direction, normal_a)
    angles = np.linspace(0.0, 2.0 * math.pi, sides, endpoint=False)
    start_ring = np.array(
        [
            start
            + start_radius * (math.cos(angle) * normal_a + math.sin(angle) * normal_b)
            for angle in angles
        ]
    )
    end_ring = np.array(
        [
            end + end_radius * (math.cos(angle) * normal_a + math.sin(angle) * normal_b)
            for angle in angles
        ]
    )
    faces = [start_ring[::-1], end_ring]
    for index in range(sides):
        following = (index + 1) % sides
        faces.append(
            np.array(
                [
                    start_ring[index],
                    start_ring[following],
                    end_ring[following],
                    end_ring[index],
                ]
            )
        )
    return faces


def _propeller_faces(center: np.ndarray, angle: float) -> list[np.ndarray]:
    direction = np.array([math.cos(angle), math.sin(angle), 0.0])
    lateral = np.array([-direction[1], direction[0], 0.0])
    blade = np.array(
        [
            center - 0.42 * direction,
            center - 0.10 * direction - 0.075 * lateral,
            center + 0.10 * direction - 0.075 * lateral,
            center + 0.42 * direction,
            center + 0.10 * direction + 0.075 * lateral,
            center - 0.10 * direction + 0.075 * lateral,
        ]
    )
    return [blade]


def _build_quadrotor_meshes() -> dict[str, list[np.ndarray]]:
    # The flight-controller frame is FRD: +X forward, +Y right and +Z down.
    # Therefore motors/propellers use negative body Z, while landing gear uses
    # positive body Z. This makes a normal -Z-up pose render feet-down.
    motors = np.array(
        [
            [0.66, -0.66, -0.08],
            [0.66, 0.66, -0.08],
            [-0.66, -0.66, -0.08],
            [-0.66, 0.66, -0.08],
        ],
        dtype=np.float64,
    )
    shell = _ellipsoid_faces(np.array([0.0, 0.0, -0.02]), (0.47, 0.34, 0.18))
    motor_mesh: list[np.ndarray] = []
    propeller_mesh: list[np.ndarray] = []
    gear_mesh: list[np.ndarray] = []
    for index, motor in enumerate(motors):
        arm_start = np.array(
            [0.12 * np.sign(motor[0]), 0.12 * np.sign(motor[1]), -0.02]
        )
        arm_end = motor.copy()
        shell.extend(_tube_faces(arm_start, arm_end, 0.16, 0.14, sides=10))
        shell.extend(_ellipsoid_faces(motor, (0.19, 0.19, 0.16), 10, 6))

        motor_bottom = motor + 0.10 * BODY_UP
        motor_top = motor + 0.29 * BODY_UP
        motor_mesh.extend(_tube_faces(motor_bottom, motor_top, 0.095, 0.085, sides=12))
        propeller_mesh.extend(
            _propeller_faces(
                motor + 0.32 * BODY_UP,
                0.20 * math.pi if index % 2 else -0.20 * math.pi,
            )
        )

        leg_start = 0.52 * motor + 0.12 * BODY_DOWN
        leg_end = 0.80 * motor + 0.58 * BODY_DOWN
        gear_mesh.extend(_tube_faces(leg_start, leg_end, 0.035, 0.027, sides=7))
        foot_end = leg_end + np.array([0.16 * np.sign(motor[0]), 0.0, 0.0])
        gear_mesh.extend(_tube_faces(leg_end, foot_end, 0.028, 0.022, sides=7))

    def shell_decal(xy: np.ndarray) -> np.ndarray:
        radial_fraction = np.sum((xy / np.array([0.47, 0.34])) ** 2, axis=1)
        surface_z = -0.02 - 0.18 * np.sqrt(np.clip(1.0 - radial_fraction, 0.0, 1.0))
        return np.column_stack((xy, surface_z - 0.02))

    nose = [
        shell_decal(
            np.array(
                [
                    [-0.20, -0.045],
                    [0.19, -0.045],
                    [0.19, 0.045],
                    [-0.20, 0.045],
                ]
            )
        ),
        shell_decal(np.array([[0.43, 0.0], [0.16, -0.13], [0.16, 0.13]])),
    ]
    return {
        "shell": shell,
        "motors": motor_mesh,
        "propellers": propeller_mesh,
        "gear": gear_mesh,
        "nose": nose,
    }


@dataclass
class _QuadrotorArtists:
    axis: Any
    target_style: bool

    def __post_init__(self) -> None:
        self.body_meshes = _build_quadrotor_meshes()
        self._last_direction: np.ndarray | None = None
        self._visible = True
        if self.target_style:
            colors = {
                "shell": "#b7dfb9",
                "motors": "#66bb6a",
                "propellers": "#43a047",
                "gear": "#81c784",
                "nose": "#d50000",
            }
            edge_color = "#2e7d32"
            alpha = 0.58
        else:
            colors = {
                "shell": "#9aa0a8",
                "motors": "#b8d95a",
                "propellers": "#d69b39",
                "gear": "#a9ce70",
                "nose": "#d50000",
            }
            edge_color = "#30343b"
            alpha = 0.96

        self.collections: dict[str, Poly3DCollection] = {}
        for name, faces in self.body_meshes.items():
            collection = Poly3DCollection(
                faces,
                facecolor=colors[name],
                edgecolor=edge_color,
                linewidth=0.35 if name != "nose" else 0.7,
                alpha=1.0 if name == "nose" else alpha,
            )
            if name == "nose":
                # Keep the body-surface decal visible despite mplot3d sorting
                # separate collections by their average depth.
                collection.set_sort_zpos(10.0)
            else:
                collection.set_zsort("average")
            self.axis.add_collection3d(collection)
            self.collections[name] = collection

    def set_direction(
        self, direction: list[float] | tuple[float, ...] | np.ndarray | None
    ) -> bool:
        normalized = None if direction is None else normalized_direction(direction)
        if normalized is None:
            if not self._visible:
                return False
            for collection in self.collections.values():
                collection.set_visible(False)
            self._visible = False
            self._last_direction = None
            return True
        minimum_change_deg = 0.0 if self.target_style else 0.75
        if self._last_direction is not None:
            dot = float(np.clip(np.dot(normalized, self._last_direction), -1.0, 1.0))
            if dot >= math.cos(math.radians(minimum_change_deg)):
                return False
        rotation = orientation_from_specific_force(normalized)
        for name, collection in self.collections.items():
            collection.set_visible(True)
            transformed_faces = [
                (rotation @ face.T).T for face in self.body_meshes[name]
            ]
            collection.set_verts(transformed_faces)
        self._visible = True
        self._last_direction = normalized
        return True


class QuadrotorView:
    """Two synchronized 3-D panels showing current and target quadrotors."""

    def __init__(self, figure: Any) -> None:
        self.current_axis = figure.add_subplot(121, projection="3d")
        self.target_axis = figure.add_subplot(122, projection="3d")
        self._configure_axis(self.current_axis, "当前四旋翼")
        self._configure_axis(self.target_axis, "目标四旋翼")
        self.current = _QuadrotorArtists(self.current_axis, target_style=False)
        self.target = _QuadrotorArtists(self.target_axis, target_style=True)
        figure.subplots_adjust(
            left=0.01, right=0.99, bottom=0.03, top=0.92, wspace=0.02
        )

    def update(
        self,
        current_direction: list[float] | tuple[float, ...] | np.ndarray | None,
        target_direction: list[float] | tuple[float, ...] | np.ndarray,
    ) -> bool:
        current_changed = self.current.set_direction(current_direction)
        target_changed = self.target.set_direction(target_direction)
        return current_changed or target_changed

    @staticmethod
    def _configure_axis(axis: Any, title: str) -> None:
        axis.set_title(title, pad=4, fontsize=11, fontweight="bold")
        axis.set_xlim(-1.2, 1.2)
        axis.set_ylim(-1.2, 1.2)
        axis.set_zlim(-1.05, 1.25)
        axis.set_box_aspect((1.0, 1.0, 1.0))
        axis.view_init(elev=24.0, azim=-58.0)
        axis.set_xlabel("前 +X", labelpad=-2)
        axis.set_ylabel("右 +Y", labelpad=-2)
        axis.set_zlabel("上", labelpad=-2)
        axis.set_xticks([])
        axis.set_yticks([])
        axis.set_zticks([])
        axis.plot(
            [0.0, 0.0],
            [0.0, 0.0],
            [-1.0, 1.0],
            color="#b0bec5",
            linestyle=":",
            linewidth=1.0,
        )
        axis.text(0.0, 0.0, 1.08, "世界向上", color="#607d8b", ha="center", fontsize=8)
