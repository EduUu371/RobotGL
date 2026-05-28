from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, degrees, hypot, radians, sin
from typing import Any


class SemanticError(Exception):
    """Erro de significado: comando valido, mas impossivel para o robo."""


@dataclass(frozen=True)
class JointState:
    robot_type: str
    values: tuple[float, float, float]
    point: tuple[float, float, float]


def _inside(value: float, limits: list[float] | tuple[float, float]) -> bool:
    return float(limits[0]) <= value <= float(limits[1])


def inverse_kinematics(
    robot_type: str,
    point: tuple[float, float, float],
    limits: dict[str, Any],
) -> JointState:
    x, y, z = point
    cfg = limits.get(robot_type)
    if not cfg:
        raise SemanticError(f"Tipo de robo desconhecido: {robot_type!r}.")

    workspace = cfg["workspace"]
    if robot_type == "cartesiano":
        if not _inside(x, workspace["x"]):
            raise SemanticError(f"x={x:.2f} fora do limite {workspace['x']}.")
        if not _inside(y, workspace["y"]):
            raise SemanticError(f"y={y:.2f} fora do limite {workspace['y']}.")
        if not _inside(z, workspace["z"]):
            raise SemanticError(f"z={z:.2f} fora do limite {workspace['z']}.")
        return JointState(robot_type, (x, y, z), point)

    if robot_type == "cilindrico":
        radius = hypot(x, y)
        theta = degrees(atan2(y, x))
        if not _inside(theta, workspace["theta_deg"]):
            raise SemanticError(f"theta={theta:.1f} graus fora do limite {workspace['theta_deg']}.")
        if not _inside(radius, workspace["radius"]):
            raise SemanticError(f"raio={radius:.2f} fora do limite {workspace['radius']}.")
        if not _inside(z, workspace["z"]):
            raise SemanticError(f"z={z:.2f} fora do limite {workspace['z']}.")
        return JointState(robot_type, (theta, radius, z), point)

    raise SemanticError(f"Cinematica nao implementada para {robot_type!r}.")


def forward_kinematics(robot_type: str, values: tuple[float, float, float]) -> tuple[float, float, float]:
    if robot_type == "cartesiano":
        return values
    if robot_type == "cilindrico":
        theta, radius, z = values
        t = radians(theta)
        return (radius * cos(t), radius * sin(t), z)
    raise SemanticError(f"Cinematica direta nao implementada para {robot_type!r}.")


def interpolate_linear(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    steps: int,
) -> list[tuple[float, float, float]]:
    steps = max(2, steps)
    return [
        (
            start[0] + (end[0] - start[0]) * i / (steps - 1),
            start[1] + (end[1] - start[1]) * i / (steps - 1),
            start[2] + (end[2] - start[2]) * i / (steps - 1),
        )
        for i in range(steps)
    ]


def interpolate_joint(
    robot_type: str,
    start_values: tuple[float, float, float],
    end_values: tuple[float, float, float],
    steps: int,
) -> list[tuple[float, float, float]]:
    return [
        forward_kinematics(
            robot_type,
            (
                start_values[0] + (end_values[0] - start_values[0]) * i / (steps - 1),
                start_values[1] + (end_values[1] - start_values[1]) * i / (steps - 1),
                start_values[2] + (end_values[2] - start_values[2]) * i / (steps - 1),
            ),
        )
        for i in range(max(2, steps))
    ]


def interpolate_arc_xy(
    start: tuple[float, float, float],
    center: tuple[float, float, float],
    angle_deg: float,
    steps: int,
) -> list[tuple[float, float, float]]:
    steps = max(3, steps)
    sx, sy, sz = start
    cx, cy, cz = center
    radius = hypot(sx - cx, sy - cy)
    if radius <= 0.0001:
        raise SemanticError("MoveC precisa que o ponto atual esteja afastado do centro do arco.")
    start_angle = atan2(sy - cy, sx - cx)
    total = radians(angle_deg)
    return [
        (
            cx + radius * cos(start_angle + total * i / (steps - 1)),
            cy + radius * sin(start_angle + total * i / (steps - 1)),
            sz + (cz - sz) * i / (steps - 1),
        )
        for i in range(steps)
    ]
