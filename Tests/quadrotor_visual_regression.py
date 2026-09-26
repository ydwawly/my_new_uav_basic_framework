#!/usr/bin/env python3
"""Regression tests for the gravity-based quadrotor guidance view."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import matplotlib
import numpy as np


matplotlib.use("Agg")
from matplotlib.figure import Figure  # noqa: E402


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "Tools" / "imu_calibration"
sys.path.insert(0, str(TOOLS))

from guided_lm_core import TARGET_POSES  # noqa: E402
from quadrotor_visual import (  # noqa: E402
    QuadrotorView,
    _build_quadrotor_meshes,
    orientation_from_specific_force,
    turn_instruction,
)


class QuadrotorMathRegression(unittest.TestCase):
    def test_all_target_rotations_are_proper_and_point_gravity_up(self) -> None:
        for pose in TARGET_POSES:
            direction = np.asarray(pose.direction)
            rotation = orientation_from_specific_force(direction)
            np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
            np.testing.assert_allclose(
                rotation @ direction, [0.0, 0.0, 1.0], atol=1e-12
            )
            self.assertAlmostEqual(float(np.linalg.det(rotation)), 1.0, places=12)

    def test_turn_instruction_uses_intuitive_roll_and_pitch_words(self) -> None:
        self.assertIn("向右滚转", turn_instruction([0, 0, -1], [0, -1, 0]))
        self.assertIn("压机头", turn_instruction([0, 0, -1], [-1, 0, 0]))
        self.assertIn("保持不动", turn_instruction([0, 0, -1], [0, 0, -1]))

    def test_invalid_current_direction_waits_instead_of_guessing(self) -> None:
        self.assertIn("等待实时姿态", turn_instruction([0, 0, 0], [0, 0, -1]))


class QuadrotorRenderRegression(unittest.TestCase):
    def test_nose_arrow_points_along_positive_body_x(self) -> None:
        arrow = np.concatenate(_build_quadrotor_meshes()["nose"], axis=0)
        tip = arrow[int(np.argmax(arrow[:, 0]))]
        self.assertGreater(float(tip[0]), 0.4)
        self.assertAlmostEqual(float(tip[1]), 0.0)
        self.assertLessEqual(float(np.max(np.abs(arrow[:, 0]))), 0.47)
        self.assertLessEqual(float(np.max(np.abs(arrow[:, 1]))), 0.34)
        self.assertTrue(np.all((-0.22 < arrow[:, 2]) & (arrow[:, 2] < -0.08)))

    def test_normal_pose_places_propellers_above_landing_gear(self) -> None:
        meshes = _build_quadrotor_meshes()
        rotation = orientation_from_specific_force([0, 0, -1])

        def world_z(mesh_name: str) -> np.ndarray:
            vertices = np.concatenate(meshes[mesh_name], axis=0)
            return (rotation @ vertices.T).T[:, 2]

        propeller_z = world_z("propellers")
        motor_z = world_z("motors")
        gear_z = world_z("gear")
        self.assertGreater(float(np.min(propeller_z)), 0.0)
        self.assertGreater(float(np.mean(motor_z)), 0.0)
        self.assertLess(float(np.max(gear_z)), 0.0)

    def test_dual_quadrotor_view_renders_without_gui_backend(self) -> None:
        figure = Figure(figsize=(8, 4), dpi=80)
        view = QuadrotorView(figure)
        view.update([0, 0, -1], [1, 0, 0])
        figure.canvas.draw()
        self.assertEqual(len(figure.axes), 2)
        self.assertTrue(all(axis.lines for axis in figure.axes))
        self.assertTrue(all(len(axis.collections) >= 5 for axis in figure.axes))


if __name__ == "__main__":
    unittest.main()
