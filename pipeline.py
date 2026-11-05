"""Extrai, etapa por etapa, como o codigo do robo vira movimento.

Nao duplica regras do interpretador: cada comando e validado compilando o
prefixo do programa com ``compile_program``. Os detalhes (tokens, formulas da
cinematica inversa, verificacao pela cinematica direta) sao derivados do
resultado desse compilador, entao a demonstracao nunca diverge do simulador.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from math import atan2, cos, degrees, hypot, radians, sin, dist
from typing import Any

from kinematics import SemanticError, forward_kinematics, inverse_kinematics
from robot_language import (
    LexicalError,
    ProgramResult,
    SyntaxRobotError,
    _check_chars,
    _parse_call,
    _split_statements,
    compile_program,
    strip_comments,
)

Point = tuple[float, float, float]

CONFIG_COMMANDS = {"PONTO_INICIAL", "INICIO", "ORIGEM", "VEL", "VELOCIDADE", "ZONA"}
MOTION_COMMANDS = {"MOVEL", "MOVEJ", "MOVEC"}

_TOKEN_RE = re.compile(r"\s*(?:(\d+\.\d*|\.\d+|\d+)|([A-Za-z_]\w*)|(.))")


@dataclass(frozen=True)
class IKLine:
    """Uma linha da conversao cartesiano -> junta."""

    joint: int  # indice da junta (0..2) para colorir
    label: str  # ex.: "J1 theta"
    formula: str  # ex.: "atan2(y, x)"
    substituted: str  # ex.: "atan2(1.40, 2.20)"
    result: str  # ex.: "32.47 deg"
    limit: str  # ex.: "[-170, 170]"
    ok: bool


@dataclass
class Step:
    index: int
    text: str
    name: str
    kind: str  # robo | config | motion
    tokens: list[tuple[str, str]] = field(default_factory=list)
    fields: list[tuple[str, str]] = field(default_factory=list)
    target: Point | None = None
    prev_point: Point | None = None
    joints: tuple[float, float, float] | None = None
    prev_joints: tuple[float, float, float] | None = None
    ik_lines: list[IKLine] = field(default_factory=list)
    fk_text: str = ""
    fk_error: float = 0.0
    notes: list[str] = field(default_factory=list)
    samples: int = 0
    path_len: int = 0  # tamanho da trajetoria ate o fim deste comando
    segment: list[Point] = field(default_factory=list)  # pontos deste movimento
    error: tuple[str, str] | None = None  # (etapa, mensagem)


@dataclass
class PipelineReport:
    source: str
    robot_type: str
    steps: list[Step]
    result: ProgramResult | None
    error: tuple[str, str] | None  # erro global (lexico) ou o primeiro de um passo
    error_step: int = -1

    @property
    def ok(self) -> bool:
        return self.error is None and self.result is not None

    @property
    def motion_steps(self) -> list[Step]:
        return [s for s in self.steps if s.kind == "motion" or (s.name in {"PONTO_INICIAL", "INICIO", "ORIGEM"} and s.target)]


def tokenize(statement: str) -> list[tuple[str, str]]:
    """Analise lexica: devolve (categoria, texto)."""
    raw: list[tuple[str, str]] = []
    pos = 0
    while pos < len(statement):
        m = _TOKEN_RE.match(statement, pos)
        if not m or m.end() == pos:
            break
        pos = m.end()
        number, ident, punct = m.groups()
        if number is not None:
            raw.append(("numero", number))
        elif ident is not None:
            raw.append(("ident", ident))
        elif punct is not None and not punct.isspace():
            raw.append(("pontuacao", punct))
    tokens: list[tuple[str, str]] = []
    for i, (kind, text) in enumerate(raw):
        prev = tokens[-1] if tokens else None
        if kind == "numero" and prev and prev[1] in "+-" and prev[0] == "pontuacao":
            before = tokens[-2][1] if len(tokens) >= 2 else ""
            if before in {"(", ",", "="}:
                tokens[-1] = ("numero", prev[1] + text)
                continue
        if kind == "ident":
            nxt = raw[i + 1][1] if i + 1 < len(raw) else ""
            if not tokens:
                kind = "comando"
            elif nxt == "=":
                kind = "parametro"
        tokens.append((kind, text))
    return tokens


def _fmt(value: float) -> str:
    return f"{value:.2f}"


def _fmt_point(p: Point) -> str:
    return f"({_fmt(p[0])}, {_fmt(p[1])}, {_fmt(p[2])})"


def _ik_lines(robot_type: str, point: Point, limits: dict[str, Any]) -> list[IKLine]:
    ws = limits[robot_type]["workspace"]
    x, y, z = point

    def inside(v: float, lim: list[float]) -> bool:
        return float(lim[0]) <= v <= float(lim[1])

    def lim_text(lim: list[float]) -> str:
        return f"[{lim[0]:g}, {lim[1]:g}]"

    if robot_type == "cartesiano":
        return [
            IKLine(0, "J1 (eixo X)", "J1 = x", f"x = {_fmt(x)}", f"{_fmt(x)} m", lim_text(ws["x"]), inside(x, ws["x"])),
            IKLine(1, "J2 (eixo Y)", "J2 = y", f"y = {_fmt(y)}", f"{_fmt(y)} m", lim_text(ws["y"]), inside(y, ws["y"])),
            IKLine(2, "J3 (eixo Z)", "J3 = z", f"z = {_fmt(z)}", f"{_fmt(z)} m", lim_text(ws["z"]), inside(z, ws["z"])),
        ]
    theta = degrees(atan2(y, x))
    radius = hypot(x, y)
    return [
        IKLine(
            0, "J1 (base)", "theta = atan2(y, x)", f"atan2({_fmt(y)}, {_fmt(x)})", f"{theta:.2f} graus",
            lim_text(ws["theta_deg"]), inside(theta, ws["theta_deg"]),
        ),
        IKLine(
            1, "J2 (raio)", "r = sqrt(x^2 + y^2)", f"sqrt({_fmt(x)}^2 + {_fmt(y)}^2)", f"{radius:.2f} m",
            lim_text(ws["radius"]), inside(radius, ws["radius"]),
        ),
        IKLine(2, "J3 (altura)", "h = z", f"z = {_fmt(z)}", f"{_fmt(z)} m", lim_text(ws["z"]), inside(z, ws["z"])),
    ]


def _fk_check(robot_type: str, joints: tuple[float, float, float], point: Point) -> tuple[str, float]:
    back = forward_kinematics(robot_type, joints)
    err = dist(back, point)
    if robot_type == "cilindrico":
        t, r, h = joints
        text = f"FK: ({_fmt(r)}*cos {t:.1f}, {_fmt(r)}*sin {t:.1f}, {_fmt(h)}) = {_fmt_point(back)}"
    else:
        text = f"FK: (J1, J2, J3) = {_fmt_point(back)}"
    return text, err


def _classify(name: str) -> str:
    if name == "ROBO":
        return "robo"
    if name in MOTION_COMMANDS:
        return "motion"
    return "config"


def _stage_of(exc: Exception) -> str:
    if isinstance(exc, LexicalError):
        return "Lexico"
    if isinstance(exc, SyntaxRobotError):
        return "Sintatico"
    if isinstance(exc, SemanticError):
        return "Semantico"
    return "Interno"


_PRETTY = {"MOVEL": "MoveL", "MOVEJ": "MoveJ", "MOVEC": "MoveC"}


def analyze(source: str, limits: dict[str, Any]) -> PipelineReport:
    """Roda o compilador comando a comando e registra cada etapa da conversao."""
    clean = strip_comments(source)
    try:
        _check_chars(clean)
        statements = _split_statements(source)
    except (LexicalError, SyntaxRobotError) as exc:
        return PipelineReport(source, "", [], None, (_stage_of(exc), str(exc)))

    steps: list[Step] = []
    prev: ProgramResult | None = None
    robot_type = ""
    error: tuple[str, str] | None = None
    error_step = -1

    for i, text in enumerate(statements):
        step = Step(index=i, text=text, name="?", kind="config", tokens=tokenize(text))
        steps.append(step)
        try:
            name, args, kwargs = _parse_call(text)
        except SyntaxRobotError as exc:
            step.error = (_stage_of(exc), str(exc))
            error, error_step = step.error, i
            break
        step.name = name
        step.kind = _classify(name)
        step.fields = [("comando", _PRETTY.get(name, name))] + [(f"arg{n + 1}", a) for n, a in enumerate(args)] + [(k, v) for k, v in kwargs.items()]
        if name == "ROBO":
            step.fields = [("comando", "ROBO"), ("tipo", args[0] if args else "?")]

        prefix = ";".join(statements[: i + 1]) + ";"
        try:
            res = compile_program(prefix, limits)
        except (LexicalError, SyntaxRobotError, SemanticError) as exc:
            step.error = (_stage_of(exc), str(exc))
            error, error_step = step.error, i
            break

        robot_type = res.robot_type or robot_type
        before_point = prev.path[-1] if prev and prev.path else res.start
        before_joints = prev.final_joints if prev else (0.0, 0.0, 0.0)

        step.path_len = len(res.path)
        if step.kind == "robo":
            step.notes.append(limits[robot_type]["name"] + " selecionado; os limites vem de .robot_limits.json")
        elif name in {"PONTO_INICIAL", "INICIO", "ORIGEM"} or step.kind == "motion":
            target = res.path[-1] if step.kind == "motion" else res.start
            step.target = target
            step.prev_point = before_point
            step.joints = res.final_joints
            step.prev_joints = before_joints
            step.path_len = len(res.path)
            step.ik_lines = _ik_lines(robot_type, target, limits)
            step.fk_text, step.fk_error = _fk_check(robot_type, res.final_joints, target)
            cmd = res.commands[-1] if step.kind == "motion" and res.commands else None
            if cmd:
                step.samples = len(cmd.points)
                step.segment = list(cmd.points)
                if name == "MOVEL":
                    step.notes.append(f"MoveL interpola em XYZ (reta) e aplica a IK nos {step.samples} pontos")
                elif name == "MOVEJ":
                    step.notes.append(f"MoveJ interpola nas JUNTAS e aplica FK nos {step.samples} pontos")
                else:
                    ang = kwargs.get("angulo", "0")
                    step.notes.append(f"MoveC: arco de {ang} graus no plano XY, {step.samples} pontos, cada um validado pela IK")
                step.notes.append(f"velocidade {cmd.speed:g}  zona {cmd.zone:g}")
        else:
            step.path_len = len(res.path)
            if name in {"VEL", "VELOCIDADE"}:
                step.notes.append(f"velocidade padrao = {args[0]} (limite {limits[robot_type]['max_speed']})")
            elif name == "ZONA":
                step.notes.append(f"zona de aproximacao = {args[0]}")
        prev = res

    result: ProgramResult | None = None
    if error is None:
        try:
            result = compile_program(source, limits)
        except (LexicalError, SyntaxRobotError, SemanticError) as exc:
            error = (_stage_of(exc), str(exc))
    return PipelineReport(source, robot_type, steps, result, error, error_step)


def format_report(report: PipelineReport) -> str:
    """Versao em texto da demonstracao (modo --terminal, sem GPU nem janela)."""
    out: list[str] = []
    rule = "=" * 72
    out += [rule, "CODIGO -> CINEMATICA -> SIMULADOR", rule]
    for step in report.steps:
        out.append("")
        out.append(f"[{step.index + 1}] {step.text};")
        out.append("    tokens : " + "  ".join(f"{t}<{k}>" for k, t in step.tokens))
        out.append("    comando: " + ", ".join(f"{k}={v}" for k, v in step.fields))
        if step.target:
            out.append(f"    alvo   : XYZ = {_fmt_point(step.target)}")
            for line in step.ik_lines:
                mark = "ok" if line.ok else "FORA"
                out.append(f"    IK     : {line.label:<12} {line.formula}  ->  {line.substituted}  ->  {line.result}  limite {line.limit} {mark}")
            out.append(f"    juntas : {tuple(round(v, 3) for v in (step.joints or ()))}")
            out.append(f"    {step.fk_text}  (erro {step.fk_error:.1e})")
        for note in step.notes:
            out.append(f"    nota   : {note}")
        if step.error:
            out.append(f"    ERRO {step.error[0]}: {step.error[1]}")
    if report.error and report.error_step < 0:
        out += ["", f"ERRO {report.error[0]}: {report.error[1]}"]
    out.append("")
    if report.result:
        r = report.result
        out.append(f"OK: {len(r.commands)} movimentos, {len(r.path)} pontos de trajetoria, juntas finais {tuple(round(v, 3) for v in r.final_joints)}")
    else:
        out.append("Programa invalido: simulacao nao iniciada.")
    return "\n".join(out)
