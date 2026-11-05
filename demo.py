"""Linha do tempo da demonstracao: codigo -> cinematica -> simulacao."""

from __future__ import annotations

from dataclasses import dataclass

from pipeline import PipelineReport

# Fracoes do progresso (0..1) em que cada etapa da conversao aparece.
STAGES = ("Codigo", "Tokens", "Comando", "Alvo XYZ", "Cinematica inversa", "Juntas > Robo")
STAGE_START = (0.00, 0.10, 0.24, 0.38, 0.52, 0.80)

DWELL_MOTION = 5.5
DWELL_CONFIG = 1.8
DWELL_ERROR = 1e9


@dataclass(frozen=True)
class Frame:
    phase: str  # "passo" | "erro" | "simulacao"
    step: int  # indice do statement (-1 em simulacao)
    dwell: float


def stage_progress(progress: float, stage: int) -> float:
    """Quanto da etapa ``stage`` ja foi revelado (0..1)."""
    start = STAGE_START[stage]
    end = STAGE_START[stage + 1] if stage + 1 < len(STAGE_START) else 1.0
    return max(0.0, min(1.0, (progress - start) / max(1e-9, end - start)))


def active_stage(progress: float) -> int:
    stage = 0
    for i, start in enumerate(STAGE_START):
        if progress >= start:
            stage = i
    return stage


def ease(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)


class Timeline:
    def __init__(self, report: PipelineReport, speed: float = 1.0) -> None:
        self.frames: list[Frame] = []
        self.speed = speed
        for step in report.steps:
            if step.error:
                self.frames.append(Frame("erro", step.index, DWELL_ERROR))
                break
            self.frames.append(Frame("passo", step.index, DWELL_MOTION if step.target and step.kind == "motion" else DWELL_CONFIG))
        if report.ok:
            self.frames.append(Frame("simulacao", -1, DWELL_ERROR))
        elif report.error and report.error_step < 0:
            self.frames.append(Frame("erro", -1, DWELL_ERROR))
        self.index = 0
        self.t = 0.0
        self.auto = True

    @property
    def frame(self) -> Frame:
        return self.frames[self.index]

    @property
    def progress(self) -> float:
        f = self.frame
        return 1.0 if f.dwell >= DWELL_ERROR else min(1.0, self.t / f.dwell)

    @property
    def in_simulation(self) -> bool:
        return self.frame.phase == "simulacao"

    @property
    def at_end(self) -> bool:
        return self.index >= len(self.frames) - 1

    def update(self, dt: float) -> None:
        if not self.auto or self.at_end:
            self.t += dt if self.at_end else 0.0
            return
        self.t += dt * self.speed
        if self.t >= self.frame.dwell:
            self.next()

    def next(self) -> None:
        if self.at_end:
            return
        if self.progress < 0.97 and self.frame.dwell < DWELL_ERROR:
            self.t = self.frame.dwell  # primeiro completa o passo atual
            return
        self.index += 1
        self.t = 0.0

    def prev(self) -> None:
        if self.t > 0.6 or self.index == 0:
            self.t = 0.0
        else:
            self.index -= 1
            self.t = 0.0

    def goto_end(self) -> None:
        self.index = len(self.frames) - 1
        self.t = 0.0

    def restart(self) -> None:
        self.index = 0
        self.t = 0.0
        self.auto = True
