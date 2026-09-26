"""ESKF核心数学约定的主机端回归测试。

测试使用独立的Python参考计算验证NIS、航向环绕、Joseph协方差更新和NED静止
比力符号。它不替代实机浮点测试，但能防止重构时无意改变公式和坐标系约定。
"""

from __future__ import annotations

import math
import random
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def wrap_pi(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def cholesky_nis_2d(residual: tuple[float, float], covariance: tuple[tuple[float, float], tuple[float, float]]) -> float:
    """用Cholesky前向替换计算r^T S^-1 r，不直接求逆。"""
    s00, s01 = covariance[0]
    _, s11 = covariance[1]
    l00 = math.sqrt(s00)
    l10 = s01 / l00
    l11 = math.sqrt(s11 - l10 * l10)
    y0 = residual[0] / l00
    y1 = (residual[1] - l10 * y0) / l11
    return y0 * y0 + y1 * y1


def joseph_scalar_update_2d(
    covariance: tuple[tuple[float, float], tuple[float, float]],
    observation: tuple[float, float],
    measurement_variance: float,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """复现标量观测Joseph形式，检查更新后协方差性质。"""
    p00, p01 = covariance[0]
    p10, p11 = covariance[1]
    h0, h1 = observation
    pht = (p00 * h0 + p01 * h1, p10 * h0 + p11 * h1)
    innovation_variance = h0 * pht[0] + h1 * pht[1] + measurement_variance
    gain = (pht[0] / innovation_variance, pht[1] / innovation_variance)
    hp = (h0 * p00 + h1 * p10, h0 * p01 + h1 * p11)

    updated = [[0.0, 0.0], [0.0, 0.0]]
    original = ((p00, p01), (p10, p11))
    for row in range(2):
        for column in range(2):
            updated[row][column] = (
                original[row][column]
                - gain[row] * hp[column]
                - pht[row] * gain[column]
                + gain[row] * innovation_variance * gain[column]
            )
    return (tuple(updated[0]), tuple(updated[1]))

def multiply(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    """小规模测试矩阵乘法，避免测试依赖NumPy。"""
    return [
        [sum(left[row][k] * right[k][column] for k in range(len(right))) for column in range(len(right[0]))]
        for row in range(len(left))
    ]


def transpose(matrix: list[list[float]]) -> list[list[float]]:
    return [list(column) for column in zip(*matrix)]


def quaternion_multiply(left: tuple[float, float, float, float], right: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def quaternion_from_rotation_vector(rotation: tuple[float, float, float]) -> tuple[float, float, float, float]:
    angle = math.sqrt(sum(component * component for component in rotation))
    if angle < 1.0e-12:
        return (1.0, 0.5 * rotation[0], 0.5 * rotation[1], 0.5 * rotation[2])
    scale = math.sin(0.5 * angle) / angle
    return (math.cos(0.5 * angle), *(scale * component for component in rotation))


def quaternion_to_rotation_matrix(quaternion: tuple[float, float, float, float]) -> list[list[float]]:
    w, x, y, z = quaternion
    return [
        [w * w + x * x - y * y - z * z, 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), w * w - x * x + y * y - z * z, 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), w * w - x * x - y * y + z * z],
    ]


def predicted_specific_force_direction(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float]:
    rotation = quaternion_to_rotation_matrix(quaternion)
    return (-rotation[2][0], -rotation[2][1], -rotation[2][2])


def gravity_tangent_jacobian(predicted: tuple[float, float, float]) -> list[list[float]]:
    reference = (0.0, 0.0, 1.0) if abs(predicted[2]) < 0.9 else (1.0, 0.0, 0.0)
    tangent_1 = (
        reference[1] * predicted[2] - reference[2] * predicted[1],
        reference[2] * predicted[0] - reference[0] * predicted[2],
        reference[0] * predicted[1] - reference[1] * predicted[0],
    )
    norm = math.sqrt(sum(component * component for component in tangent_1))
    tangent_1 = tuple(component / norm for component in tangent_1)
    tangent_2 = (
        predicted[1] * tangent_1[2] - predicted[2] * tangent_1[1],
        predicted[2] * tangent_1[0] - predicted[0] * tangent_1[2],
        predicted[0] * tangent_1[1] - predicted[1] * tangent_1[0],
    )
    skew = (
        (0.0, -predicted[2], predicted[1]),
        (predicted[2], 0.0, -predicted[0]),
        (-predicted[1], predicted[0], 0.0),
    )
    return [[sum(tangent[row] * skew[row][column] for row in range(3)) for column in range(3)] for tangent in (tangent_1, tangent_2)]


def sparse_predict_covariance(
    covariance: list[list[float]],
    dt: float,
    vel_att: list[list[float]],
    vel_accel_bias: list[list[float]],
    att_att_delta: list[list[float]],
) -> list[list[float]]:
    """复现固件的5组3维误差状态稀疏协方差传播。"""
    state_dim = 15
    pos, vel, att, gyro_bias, accel_bias = 0, 3, 6, 9, 12
    fp = [row[:] for row in covariance]

    for column in range(state_dim):
        for row in range(3):
            fp[pos + row][column] = covariance[pos + row][column] + dt * covariance[vel + row][column]
            fp[vel + row][column] = covariance[vel + row][column] + sum(
                vel_att[row][k] * covariance[att + k][column]
                + vel_accel_bias[row][k] * covariance[accel_bias + k][column]
                for k in range(3)
            )
            fp[att + row][column] = (
                covariance[att + row][column]
                - dt * covariance[gyro_bias + row][column]
                + sum(att_att_delta[row][k] * covariance[att + k][column] for k in range(3))
            )

    propagated = [[0.0] * state_dim for _ in range(state_dim)]
    for row in range(state_dim):
        for column in range(3):
            propagated[row][pos + column] = fp[row][pos + column] + dt * fp[row][vel + column]
            propagated[row][vel + column] = fp[row][vel + column] + sum(
                fp[row][att + k] * vel_att[column][k]
                + fp[row][accel_bias + k] * vel_accel_bias[column][k]
                for k in range(3)
            )
            propagated[row][att + column] = (
                fp[row][att + column]
                - dt * fp[row][gyro_bias + column]
                + sum(fp[row][att + k] * att_att_delta[column][k] for k in range(3))
            )
            propagated[row][gyro_bias + column] = fp[row][gyro_bias + column]
            propagated[row][accel_bias + column] = fp[row][accel_bias + column]
    return propagated


class EskfMathRegression(unittest.TestCase):
    def test_heading_residual_wraps_across_pi(self) -> None:
        measured = math.radians(-179.0)
        predicted = math.radians(179.0)
        self.assertAlmostEqual(wrap_pi(measured - predicted), math.radians(2.0), places=7)
        self.assertGreaterEqual(wrap_pi(math.pi), -math.pi)
        self.assertLess(wrap_pi(math.pi), math.pi)

    def test_group_nis_matches_closed_form_and_gate(self) -> None:
        residual = (1.0, -0.5)
        covariance = ((2.0, 0.3), (0.3, 1.5))
        nis = cholesky_nis_2d(residual, covariance)
        determinant = covariance[0][0] * covariance[1][1] - covariance[0][1] ** 2
        closed_form = (
            covariance[1][1] * residual[0] ** 2
            - 2.0 * covariance[0][1] * residual[0] * residual[1]
            + covariance[0][0] * residual[1] ** 2
        ) / determinant
        self.assertAlmostEqual(nis, closed_form, places=7)
        self.assertLess(nis, 9.210340)  # 二自由度99%卡方阈值

    def test_joseph_update_preserves_covariance_properties(self) -> None:
        updated = joseph_scalar_update_2d(((2.0, 0.4), (0.4, 1.0)), (1.0, 0.2), 0.25)
        self.assertAlmostEqual(updated[0][1], updated[1][0], places=7)
        self.assertGreater(updated[0][0], 0.0)
        self.assertGreater(updated[1][1], 0.0)
        self.assertGreater(updated[0][0] * updated[1][1] - updated[0][1] * updated[1][0], 0.0)

    def test_static_frd_specific_force_cancels_ned_gravity(self) -> None:
        gravity = 9.80665
        specific_force_frd = (0.0, 0.0, -gravity)
        acceleration_ned = (
            specific_force_frd[0],
            specific_force_frd[1],
            specific_force_frd[2] + gravity,
        )
        self.assertEqual(acceleration_ned, (0.0, 0.0, 0.0))

    def test_gravity_tangent_jacobian_matches_right_quaternion_perturbation(self) -> None:
        nominal = quaternion_from_rotation_vector((0.31, -0.22, 0.17))
        predicted = predicted_specific_force_direction(nominal)
        jacobian = gravity_tangent_jacobian(predicted)
        epsilon = 1.0e-6

        reference = (0.0, 0.0, 1.0) if abs(predicted[2]) < 0.9 else (1.0, 0.0, 0.0)
        tangent_1_raw = (
            reference[1] * predicted[2] - reference[2] * predicted[1],
            reference[2] * predicted[0] - reference[0] * predicted[2],
            reference[0] * predicted[1] - reference[1] * predicted[0],
        )
        tangent_norm = math.sqrt(sum(component * component for component in tangent_1_raw))
        tangent_1 = tuple(component / tangent_norm for component in tangent_1_raw)
        tangent_2 = (
            predicted[1] * tangent_1[2] - predicted[2] * tangent_1[1],
            predicted[2] * tangent_1[0] - predicted[0] * tangent_1[2],
            predicted[0] * tangent_1[1] - predicted[1] * tangent_1[0],
        )

        for axis in range(3):
            perturbation = tuple(epsilon if index == axis else 0.0 for index in range(3))
            perturbed = predicted_specific_force_direction(
                quaternion_multiply(nominal, quaternion_from_rotation_vector(perturbation))
            )
            derivative = tuple((perturbed[index] - predicted[index]) / epsilon for index in range(3))
            projected = (
                sum(tangent_1[index] * derivative[index] for index in range(3)),
                sum(tangent_2[index] * derivative[index] for index in range(3)),
            )
            self.assertAlmostEqual(projected[0], jacobian[0][axis], places=5)
            self.assertAlmostEqual(projected[1], jacobian[1][axis], places=5)

        # 绕预测重力方向旋转是Yaw不可观测方向，二维Jacobian必须保持零响应。
        for row in jacobian:
            self.assertAlmostEqual(sum(row[axis] * predicted[axis] for axis in range(3)), 0.0, places=7)

    def test_sparse_covariance_prediction_matches_dense_product(self) -> None:
        random_generator = random.Random(20260729)
        state_dim = 15
        dt = 0.001

        basis = [[random_generator.uniform(-1.0, 1.0) for _ in range(state_dim)] for _ in range(state_dim)]
        covariance = multiply(basis, transpose(basis))
        vel_att = [[random_generator.uniform(-0.01, 0.01) for _ in range(3)] for _ in range(3)]
        vel_accel_bias = [[random_generator.uniform(-0.01, 0.01) for _ in range(3)] for _ in range(3)]
        att_att_delta = [[random_generator.uniform(-0.01, 0.01) for _ in range(3)] for _ in range(3)]

        transition = [[float(row == column) for column in range(state_dim)] for row in range(state_dim)]
        for axis in range(3):
            transition[axis][3 + axis] = dt
            transition[6 + axis][9 + axis] = -dt
        for row in range(3):
            for column in range(3):
                transition[3 + row][6 + column] = vel_att[row][column]
                transition[3 + row][12 + column] = vel_accel_bias[row][column]
                transition[6 + row][6 + column] += att_att_delta[row][column]

        dense = multiply(multiply(transition, covariance), transpose(transition))
        sparse = sparse_predict_covariance(covariance, dt, vel_att, vel_accel_bias, att_att_delta)
        maximum_error = max(
            abs(dense[row][column] - sparse[row][column])
            for row in range(state_dim)
            for column in range(state_dim)
        )
        self.assertLess(maximum_error, 1.0e-12)

    def test_firmware_keeps_documented_numerical_stages(self) -> None:
        source = (PROJECT_ROOT / "Modules/modules_Algorithm/ESKF/ESKF.c").read_text(encoding="utf-8")
        for marker in (
            "构造中间矩阵 HP = H * P",
            "构造新息协方差矩阵",
            "Cholesky 分解",
            "前向替换解 L * y = r",
            "Joseph 等价标量形式",
            "P = Fd * P * Fd^T",
        ):
            self.assertIn(marker, source)

    def test_gravity_gate_does_not_depend_on_integrated_speed(self) -> None:
        source = (PROJECT_ROOT / "Modules/modules_Algorithm/ESKF/ESKF.c").read_text(encoding="utf-8")
        self.assertNotIn("speed_norm <= e->cfg.gravity_max_speed_mps", source)
        self.assertIn("fabsf(accel_norm - NAV_ESKF_G) <= e->cfg.gravity_accel_norm_gate_mps2", source)
        self.assertIn("gyro_norm <= e->cfg.gravity_gyro_gate_rps", source)
        self.assertIn("last_gravity_speed_norm_mps", source)


if __name__ == "__main__":
    unittest.main(verbosity=2)
