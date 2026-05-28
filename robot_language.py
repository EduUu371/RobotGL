from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from kinematics import (
    SemanticError,
    inverse_kinematics,
    interpolate_arc_xy,
    interpolate_joint,
    interpolate_linear,
)


class LexicalError(Exception):
    pass


class SyntaxRobotError(Exception):
    pass


@dataclass(frozen=True)
class MotionCommand:
    kind: str
    points: list[tuple[float, float, float]]
    speed: float
    zone: float


@dataclass(frozen=True)
class ProgramResult:
    robot_type: str
    start: tuple[float, float, float]
    commands: list[MotionCommand]
    path: list[tuple[float, float, float]]
    final_joints: tuple[float, float, float]
    diagnostics: list[str]


DEFAULT_SOURCE = """ROBO cartesiano;
PONTO_INICIAL(0, 0, 1);
VEL(1.0);
ZONA(0.05);

MoveL(2.2, 0.0, 1.0);
MoveL(2.2, 1.4, 1.8, v=1.4);
MoveC(1.2, 1.4, 1.8, angulo=120, v=0.8);
MoveJ(-1.2, -0.8, 2.4, v=1.1);
"""


CYLINDRICAL_EXAMPLE = """ROBO cilindrico;
PONTO_INICIAL(1.2, 0, 0.8);
VEL(0.9);
ZONA(0.04);

MoveJ(2.6, 0.0, 1.0);
MoveL(2.6, 1.4, 1.8, v=1.2);
MoveC(0.0, 0.0, 1.8, angulo=95, v=0.7);
MoveL(1.3, 2.2, 2.7);
"""


ANIMATION_TEST_EXAMPLE = """ROBO cilindrico;
PONTO_INICIAL(1.2, 0.0, 0.7);
VEL(0.9);
ZONA(0.05);

MoveJ(2.5, 0.0, 1.0, v=1.0);
MoveL(2.5, 1.2, 1.5, v=1.2);
MoveC(0.0, 0.0, 1.5, angulo=110, v=0.7);
MoveL(-1.0, 2.2, 2.4, v=0.9);
MoveJ(1.4, -1.4, 1.2, v=1.1);
MoveC(0.0, 0.0, 1.2, angulo=-95, v=0.8);
"""


def strip_comments(source: str) -> str:
    clean: list[str] = []
    for line in source.splitlines():
        for marker in ("//", "#"):
            if marker in line:
                line = line.split(marker, 1)[0]
        clean.append(line)
    return "\n".join(clean)


def _check_chars(source: str) -> None:
    allowed = set("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.,;()=+- \n\r\t")
    for index, char in enumerate(source):
        if char not in allowed:
            raise LexicalError(f"Caractere invalido {char!r} na posicao {index}.")


def _split_statements(source: str) -> list[str]:
    clean = strip_comments(source).strip()
    if not clean:
        raise SyntaxRobotError("Programa vazio.")
    if not clean.endswith(";"):
        raise SyntaxRobotError("Todo comando precisa terminar com ponto e virgula ';'.")
    return [part.strip() for part in clean.split(";") if part.strip()]


def _parse_call(statement: str) -> tuple[str, list[str], dict[str, str]]:
    if "(" not in statement:
        parts = statement.split()
        if len(parts) == 2 and parts[0].upper() == "ROBO":
            return ("ROBO", [parts[1]], {})
        raise SyntaxRobotError(f"Comando sem parenteses: {statement!r}.")
    if not statement.endswith(")"):
        raise SyntaxRobotError(f"Parenteses nao fechado em {statement!r}.")
    name, raw_args = statement.split("(", 1)
    name = name.strip().upper()
    raw_args = raw_args[:-1]
    args: list[str] = []
    kwargs: dict[str, str] = {}
    for item in [p.strip() for p in raw_args.split(",") if p.strip()]:
        if "=" in item:
            key, value = item.split("=", 1)
            kwargs[key.strip().lower()] = value.strip()
        else:
            args.append(item)
    return name, args, kwargs


def _number(text: str, label: str) -> float:
    try:
        return float(text)
    except ValueError as exc:
        raise SyntaxRobotError(f"{label} deveria ser numero, recebido {text!r}.") from exc


def _point(args: list[str]) -> tuple[float, float, float]:
    if len(args) < 3:
        raise SyntaxRobotError("Ponto precisa de tres coordenadas: x, y, z.")
    return (_number(args[0], "x"), _number(args[1], "y"), _number(args[2], "z"))


def _speed(default_speed: float, kwargs: dict[str, str], cfg: dict[str, Any]) -> float:
    value = _number(kwargs.get("v", kwargs.get("vel", str(default_speed))), "velocidade")
    if value <= 0:
        raise SemanticError("Velocidade precisa ser positiva.")
    if value > float(cfg["max_speed"]):
        raise SemanticError(f"Velocidade {value:.2f} maior que o maximo {cfg['max_speed']}.")
    return value


def compile_program(source: str, limits: dict[str, Any]) -> ProgramResult:
    source = strip_comments(source)
    _check_chars(source)
    statements = _split_statements(source)
    robot_type = ""
    start = (0.0, 0.0, 0.8)
    default_speed = 1.0
    zone = 0.02
    current = start
    current_joints = (0.0, 0.0, 0.0)
    path = [current]
    commands: list[MotionCommand] = []
    diagnostics: list[str] = []

    for statement in statements:
        name, args, kwargs = _parse_call(statement)
        if name == "ROBO":
            if not args:
                raise SyntaxRobotError("ROBO precisa receber cartesiano ou cilindrico.")
            robot_type = args[0].lower()
            if robot_type not in limits:
                raise SemanticError(f"Tipo de robo invalido: {robot_type}.")
            if robot_type == "cilindrico" and current == (0.0, 0.0, 0.8):
                workspace = limits[robot_type]["workspace"]
                current = (float(workspace["radius"][0]), 0.0, current[2])
                start = current
                path = [current]
            current_joints = inverse_kinematics(robot_type, current, limits).values
            diagnostics.append(f"Robo selecionado: {limits[robot_type]['name']}.")
            continue

        if not robot_type:
            raise SyntaxRobotError("O primeiro comando deve selecionar o robo: ROBO cartesiano;")
        cfg = limits[robot_type]

        if name in {"PONTO_INICIAL", "INICIO", "ORIGEM"}:
            start = _point(args)
            current = start
            current_joints = inverse_kinematics(robot_type, current, limits).values
            path = [current]
            diagnostics.append(f"Ponto inicial validado em {current}.")
        elif name in {"VEL", "VELOCIDADE"}:
            if len(args) != 1:
                raise SyntaxRobotError("VEL recebe um unico numero.")
            default_speed = _speed(default_speed, {"v": args[0]}, cfg)
        elif name == "ZONA":
            if len(args) != 1:
                raise SyntaxRobotError("ZONA recebe um unico numero.")
            zone = _number(args[0], "zona")
            if zone < 0:
                raise SemanticError("Zona nao pode ser negativa.")
        elif name in {"MOVEL", "MOVEJ"}:
            target = _point(args)
            target_state = inverse_kinematics(robot_type, target, limits)
            speed = _speed(default_speed, kwargs, cfg)
            steps = max(12, int(28 / speed))
            if name == "MOVEL":
                segment = interpolate_linear(current, target, steps)
                for point in segment:
                    inverse_kinematics(robot_type, point, limits)
            else:
                segment = interpolate_joint(robot_type, current_joints, target_state.values, steps)
                for point in segment:
                    inverse_kinematics(robot_type, point, limits)
            commands.append(MotionCommand(name, segment, speed, zone))
            path.extend(segment[1:])
            current = target
            current_joints = target_state.values
        elif name == "MOVEC":
            center = _point(args)
            if "angulo" not in kwargs:
                raise SyntaxRobotError("MoveC precisa de angulo=graus.")
            angle = _number(kwargs["angulo"], "angulo")
            speed = _speed(default_speed, kwargs, cfg)
            segment = interpolate_arc_xy(current, center, angle, max(18, int(abs(angle) / 4)))
            for point in segment:
                inverse_kinematics(robot_type, point, limits)
            commands.append(MotionCommand(name, segment, speed, zone))
            path.extend(segment[1:])
            current = segment[-1]
            current_joints = inverse_kinematics(robot_type, current, limits).values
        else:
            raise SyntaxRobotError(f"Comando desconhecido: {name}.")

    return ProgramResult(robot_type, start, commands, path, current_joints, diagnostics)
