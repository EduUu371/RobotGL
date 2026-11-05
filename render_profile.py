"""Deteccao de GPU e perfis de qualidade de renderizacao.

O simulador usa OpenGL. Com GPU real (``GPU``) usa malhas detalhadas e
antialiasing; em renderizacao por software (``CPU``: llvmpipe, SwiftShader,
GDI Generic...) usa malhas leves, sem preenchimento transparente do dominio e
com taxa de quadros menor. A escolha e automatica e pode ser forcada.
"""

from __future__ import annotations

import os
from collections import deque
from dataclasses import dataclass

SOFTWARE_MARKERS = (
    "llvmpipe",
    "softpipe",
    "swrast",
    "software",
    "swiftshader",
    "gdi generic",
    "microsoft basic render",
    "mesa offscreen",
)


@dataclass(frozen=True)
class RenderProfile:
    name: str  # "GPU" | "CPU"
    sphere: tuple[int, int]
    cylinder_steps: int
    domain_steps: int
    domain_fill: bool
    multisample: int
    fps: int
    vsync: bool


GPU_PROFILE = RenderProfile("GPU", (14, 10), 28, 72, True, 4, 60, True)
CPU_PROFILE = RenderProfile("CPU", (7, 4), 12, 20, False, 0, 30, False)
PROFILES = {"gpu": GPU_PROFILE, "cpu": CPU_PROFILE}


def is_software_renderer(renderer: str) -> bool:
    low = renderer.lower()
    return any(marker in low for marker in SOFTWARE_MARKERS)


def requested_mode(argv: list[str]) -> str:
    """auto | gpu | cpu, via --gpu/--cpu/--auto ou ROBOTGL_RENDER."""
    for arg in argv:
        if arg in {"--gpu", "--cpu", "--auto"}:
            return arg[2:]
    env = os.environ.get("ROBOTGL_RENDER", "auto").strip().lower()
    return env if env in {"auto", "gpu", "cpu"} else "auto"


def choose_profile(mode: str, renderer: str) -> RenderProfile:
    if mode in PROFILES:
        return PROFILES[mode]
    return CPU_PROFILE if is_software_renderer(renderer) else GPU_PROFILE


class FpsGovernor:
    """Rebaixa para o perfil CPU se a GPU nao sustentar a taxa de quadros."""

    def __init__(self, window_s: float = 4.0, floor: float = 22.0) -> None:
        self.window_s = window_s
        self.floor = floor
        self.samples: deque[float] = deque()
        self.elapsed = 0.0
        self.warmup = 2.0

    def reset(self) -> None:
        self.samples.clear()
        self.elapsed = 0.0
        self.warmup = 2.0

    def update(self, dt: float) -> bool:
        """Devolve True quando deve rebaixar a qualidade."""
        if self.warmup > 0:
            self.warmup -= dt
            return False
        self.samples.append(dt)
        self.elapsed += dt
        while self.elapsed > self.window_s and len(self.samples) > 1:
            self.elapsed -= self.samples.popleft()
        if self.elapsed < self.window_s * 0.9:
            return False
        fps = len(self.samples) / self.elapsed
        return fps < self.floor


def probe_renderer() -> str:
    """Abre uma janela oculta so para ler o nome do renderer OpenGL."""
    import pyglet
    from pyglet import gl

    window = pyglet.window.Window(1, 1, visible=False)
    try:
        return str(gl.gl_info.get_renderer())
    finally:
        window.close()
