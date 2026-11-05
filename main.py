from __future__ import annotations

import json
from math import atan2, cos, hypot, radians, sin
import sys
from pathlib import Path
from typing import Any

import pyglet
from pyglet import gl
from pyglet.math import Mat4, Vec3

from demo import STAGE_START, Timeline, active_stage, ease, stage_progress
from kinematics import forward_kinematics, inverse_kinematics
from pipeline import PipelineReport, analyze, format_report
from render_profile import CPU_PROFILE, FpsGovernor, RenderProfile, choose_profile, probe_renderer, requested_mode
from robot_language import (
    ANIMATION_TEST_EXAMPLE,
    CYLINDRICAL_EXAMPLE,
    DEFAULT_SOURCE,
    LexicalError,
    ProgramResult,
    SemanticError,
    SyntaxRobotError,
    compile_program,
)


ROOT = Path(__file__).resolve().parent
LIMITS_PATH = ROOT / ".robot_limits.json"


def load_limits() -> dict[str, Any]:
    return json.loads(LIMITS_PATH.read_text(encoding="utf-8"))


LEFT_W = 340
BOTTOM_H = 276
TOP_H = 42

PANEL = (20, 24, 28, 238)
PANEL_EDGE = (58, 66, 74, 255)
TEXT = (232, 238, 243, 255)
MUTED = (140, 152, 164, 255)
ACCENT = (36, 170, 116, 255)
WARN = (255, 140, 120, 255)
TOKEN_COLORS = {
    "comando": (255, 176, 64, 255),
    "parametro": (206, 150, 255, 255),
    "numero": (96, 214, 255, 255),
    "ident": (126, 238, 170, 255),
    "pontuacao": (150, 162, 174, 255),
}
STAGE_TITLES = ("Codigo", "Tokens", "Comando", "Alvo XYZ", "Cinem. inversa", "Juntas > Robo")


def lerp3(a: tuple[float, float, float], b: tuple[float, float, float], t: float) -> tuple[float, float, float]:
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t)


def clamp01(v: float) -> float:
    return max(0.0, min(1.0, v))


def fit(text: str, n: int) -> str:
    return text if len(text) <= n else text[: max(1, n - 3)] + "..."


def rgb255(color: list[float] | tuple[float, ...], alpha: int = 255) -> tuple[int, int, int, int]:
    return (int(color[0] * 255), int(color[1] * 255), int(color[2] * 255), alpha)


class RobotApp(pyglet.window.Window):
    def __init__(self, mode: str, profile: RenderProfile, renderer: str, shots_dir: Path | None = None) -> None:
        self.mode = mode
        self.profile = profile
        self.renderer = renderer
        configs: list[Any] = []
        if profile.multisample:
            configs.append(gl.Config(double_buffer=True, depth_size=24, sample_buffers=1, samples=profile.multisample))
        configs.append(gl.Config(double_buffer=True, depth_size=24))
        configs.append(None)
        last_error: Exception | None = None
        for config in configs:
            try:
                super().__init__(1400, 800, "RobotGL - do codigo a cinematica ao simulador", resizable=True, vsync=profile.vsync, config=config)
                break
            except pyglet.window.NoSuchConfigException as exc:
                last_error = exc
        else:
            raise last_error  # type: ignore[misc]
        self.set_minimum_size(1060, 680)

        self.limits = load_limits()
        self.source = DEFAULT_SOURCE
        self.result: ProgramResult | None = None
        self.report: PipelineReport | None = None
        self.timeline: Timeline | None = None
        self.error = ""
        self.active_tab = "cartesiano"
        self.sim_time = 0.0
        self.camera_yaw = 42.0
        self.camera_pitch = 28.0
        self.zoom = 13.0
        self.show_domain = True
        self.show_trace = True
        self.ui_hitboxes: dict[str, tuple[int, int, int, int]] = {}
        self.workspace_mesh_cache: dict[str, tuple[list[float], list[float]]] = {}
        self.trace_vertices: list[float] = []
        self.cmd_bounds: list[int] = []
        self.cmd_steps: list[int] = []
        self.fps = 0.0
        self.governor = FpsGovernor()
        self.labels: dict[str, pyglet.text.Label] = {}
        self.rects: dict[str, pyglet.shapes.Rectangle] = {}
        self.chip_cache: tuple[Any, list[tuple[pyglet.text.Label, int, int, int, tuple[int, int, int, int]]]] = (None, [])
        self.history_cache: tuple[int, list[float]] = (-1, [])
        self.editing = False
        self.drag_in_editor = False
        self.edit_error = False
        self.edit_text = ""
        self.edit_batch: pyglet.graphics.Batch | None = None
        self.edit_doc: Any = None
        self.edit_layout: Any = None
        self.edit_caret: Any = None
        self.shots_dir = shots_dir
        self.shot_plan: list[tuple[str, int, float]] = []
        self.shot_pending: Path | None = None
        self.shot_wait = 0
        self.apply_profile(profile, announce=False)
        self.compile_now()
        pyglet.clock.schedule_interval(self.update, 1 / 60)
        if shots_dir:
            self.plan_shots(shots_dir)

    # ------------------------------------------------------------------ perfil GPU/CPU
    def apply_profile(self, profile: RenderProfile, announce: bool = True) -> None:
        self.profile = profile
        self.workspace_mesh_cache.clear()
        self.governor.reset()
        try:
            self.set_vsync(profile.vsync)
        except Exception:
            pass
        pyglet.clock.unschedule(self.draw)
        pyglet.clock.schedule_interval(self.draw, 1 / profile.fps)
        if announce:
            print(f"[render] perfil {profile.name} ({profile.fps} fps alvo) - renderer: {self.renderer}")

    def cycle_render_mode(self) -> None:
        order = ["auto", "gpu", "cpu"]
        self.mode = order[(order.index(self.mode) + 1) % 3]
        self.apply_profile(choose_profile(self.mode, self.renderer))

    # ------------------------------------------------------------------ compilacao
    def compile_now(self) -> None:
        report = analyze(self.source, self.limits)
        self.report = report
        if report.ok and report.result:
            self.result = report.result
            self.trace_vertices = flatten_points(report.result.path)
            self.active_tab = report.result.robot_type
            self.error = "OK - programa compilado: veja a conversao e depois a simulacao."
            bound = 1
            self.cmd_bounds = []
            for cmd in report.result.commands:
                bound += len(cmd.points) - 1
                self.cmd_bounds.append(bound)
            self.cmd_steps = [s.index for s in report.steps if s.kind == "motion"]
        else:
            stage, message = report.error or ("Interno", "erro desconhecido")
            where = f" (comando {report.error_step + 1})" if report.error_step >= 0 else ""
            self.error = f"Erro {stage}{where}: {message}"
            if report.robot_type:
                self.active_tab = report.robot_type
        self.timeline = Timeline(report)
        if not report.ok:
            self.timeline.goto_end()  # mostra direto o comando com erro
        self.sim_time = 0.0
        self.history_cache = (-1, [])

    def set_source(self, source: str) -> None:
        self.stop_editing()
        self.source = source
        self.compile_now()

    # ------------------------------------------------------------------ editor de codigo embutido
    def start_editing(self) -> None:
        if self.editing:
            return
        if self.edit_doc is None:
            self.edit_batch = pyglet.graphics.Batch()
            self.edit_doc = pyglet.text.document.UnformattedDocument("")
            self.edit_layout = pyglet.text.layout.IncrementalTextLayout(
                self.edit_doc, width=LEFT_W - 28, height=200, multiline=True, wrap_lines=False, batch=self.edit_batch
            )
            self.edit_layout.anchor_y = "top"
            self.edit_caret = pyglet.text.caret.Caret(self.edit_layout, batch=self.edit_batch, color=(255, 255, 255, 255))
        self.edit_doc.text = self.source.strip("\n") + "\n"
        self.edit_doc.set_style(0, len(self.edit_doc.text), {"color": TEXT, "font_size": 10})
        self.edit_caret.position = len(self.edit_doc.text)
        self.edit_caret.visible = True
        self.editing = True
        self.edit_error = False
        self.edit_text = self.edit_doc.text
        if self.timeline:
            self.timeline.auto = False

    def stop_editing(self) -> None:
        if not self.editing:
            return
        self.edit_caret.visible = False
        self.editing = False
        self.edit_error = False

    def compile_edit(self) -> None:
        self.source = self.edit_doc.text
        self.compile_now()
        if self.report and self.report.ok:
            self.stop_editing()
        else:
            self.edit_error = True
            self.edit_text = self.edit_doc.text

    # ------------------------------------------------------------------ laco principal
    def update(self, dt: float) -> None:
        if self.editing and self.edit_error and self.edit_doc.text != self.edit_text:
            self.edit_error = False  # voltou a editar: botao volta a verde
        if self.timeline:
            was_sim = self.timeline.in_simulation
            self.timeline.update(dt)
            if self.timeline.in_simulation:
                self.sim_time = 0.0 if not was_sim else self.sim_time + dt
        if dt > 0:
            instant = 1.0 / dt
            self.fps = instant if self.fps == 0 else self.fps * 0.92 + instant * 0.08
            if self.mode == "auto" and self.profile.name == "GPU" and self.governor.update(dt):
                print("[render] taxa de quadros baixa: trocando automaticamente para o perfil CPU (leve).")
                self.apply_profile(CPU_PROFILE)
        self.step_shots()

    def on_draw(self) -> None:
        self.clear()
        self.draw_scene()
        self.draw_ui()
        if self.shot_pending:
            pyglet.image.get_buffer_manager().get_color_buffer().save(str(self.shot_pending))
            print(f"[shot] {self.shot_pending}")
            self.shot_pending = None

    # ------------------------------------------------------------------ estado da vista
    def scene_type(self) -> str:
        if self.report and self.report.robot_type:
            return self.report.robot_type
        return self.active_tab

    def home_joints(self, rt: str) -> tuple[float, float, float]:
        ws = self.limits[rt]["workspace"]
        return (0.0, 0.0, 0.8) if rt == "cartesiano" else (0.0, float(ws["radius"][0]), 0.8)

    def last_joints_before(self, index: int) -> tuple[float, float, float] | None:
        if not self.report:
            return None
        for s in reversed(self.report.steps[:index]):
            if s.joints:
                return s.joints
        return None

    def sim_index(self) -> int:
        if not self.result or not self.result.path:
            return 0
        return int(self.sim_time * 18) % len(self.result.path)

    def sim_point(self) -> tuple[float, float, float]:
        if not self.result or not self.result.path:
            return (0, 0, 0)
        return self.result.path[self.sim_index()]

    def sim_command(self) -> int:
        idx = self.sim_index()
        for k, bound in enumerate(self.cmd_bounds):
            if idx < bound:
                return k
        return max(0, len(self.cmd_bounds) - 1)

    def view_state(self) -> dict[str, Any]:
        """Passo mostrado nos paineis, progresso 0..1 e juntas atuais."""
        rt = self.scene_type()
        tl, rep = self.timeline, self.report
        if not tl or not rep:
            return {"step": None, "progress": 0.0, "joints": None, "sim": False}
        frame = tl.frame
        if frame.phase == "simulacao" and self.result:
            k = self.sim_command()
            step = rep.steps[self.cmd_steps[k]] if k < len(self.cmd_steps) else None
            joints = inverse_kinematics(rt, self.sim_point(), self.limits).values
            return {"step": step, "progress": 1.0, "joints": joints, "sim": True}
        if frame.step < 0:
            return {"step": None, "progress": 1.0, "joints": None, "sim": False}
        step = rep.steps[frame.step]
        prog = tl.progress
        prev = self.last_joints_before(step.index) or self.home_joints(rt)
        if step.joints:
            joints = lerp3(step.prev_joints or prev, step.joints, ease(stage_progress(prog, 5)))
        else:
            joints = prev
        return {"step": step, "progress": prog, "joints": joints, "sim": False}

    # ------------------------------------------------------------------ cena 3D
    def scene_rect(self) -> tuple[int, int, int, int]:
        return (LEFT_W, BOTTOM_H, max(1, self.width - LEFT_W), max(1, self.height - TOP_H - BOTTOM_H))

    def draw_scene(self) -> None:
        ratio = self.get_pixel_ratio()
        vx, vy, vw, vh = self.scene_rect()
        px, py, pw, ph = int(vx * ratio), int(vy * ratio), int(vw * ratio), int(vh * ratio)
        gl.glViewport(px, py, pw, ph)
        gl.glEnable(gl.GL_SCISSOR_TEST)
        gl.glScissor(px, py, pw, ph)
        gl.glClearColor(0.10, 0.12, 0.13, 1.0)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gl.glDisable(gl.GL_SCISSOR_TEST)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
        if self.profile.multisample:
            gl.glEnable(gl.GL_MULTISAMPLE)

        self.projection = Mat4.perspective_projection(vw / vh, 0.1, 100.0, 50)
        yaw = radians(self.camera_yaw)
        pitch = radians(self.camera_pitch)
        eye = Vec3(self.zoom * cos(pitch) * cos(yaw), self.zoom * cos(pitch) * sin(yaw), self.zoom * sin(pitch))
        self.view = Mat4.look_at(eye, Vec3(0, 0, 1.2), Vec3(0, 0, 1))

        rt = self.scene_type()
        self.draw_grid()
        self.draw_axes()
        if not self.report or not self.timeline:
            return
        if self.show_domain:
            self.draw_workspace_domain(rt)
        view = self.view_state()
        step, prog = view["step"], view["progress"]
        if view["sim"]:
            point = self.sim_point()
            if self.show_trace:
                self.draw_trace()
        else:
            joints = view["joints"] or self.home_joints(rt)
            point = forward_kinematics(rt, joints)
            self.draw_history(self.timeline.frame.step)
            if step is not None and step.target:
                self.draw_step_guides(step, prog, rt)
        if rt == "cilindrico":
            self.draw_cylindrical_robot(point)
        else:
            self.draw_cartesian_robot(point)

    def draw_history(self, upto: int) -> None:
        """Movimentos ja convertidos (passos anteriores) em azul esmaecido."""
        if not self.report or not self.show_trace:
            return
        if self.history_cache[0] != upto:
            pts: list[float] = []
            for s in self.report.steps[: max(0, upto)]:
                if s.segment:
                    pts.extend(flatten_points(s.segment))
            self.history_cache = (upto, pts)
        gl.glLineWidth(3)
        draw_vertices(gl.GL_LINE_STRIP, self.history_cache[1], (0.0, 0.7, 0.8, 0.8))

    def draw_step_guides(self, step: Any, prog: float, rt: str) -> None:
        cols = [tuple(j["color"]) for j in self.limits[rt]["joints"]]
        x, y, z = step.target
        reveal = clamp01(stage_progress(prog, 3) * 1.4)
        if reveal > 0:
            if step.segment:
                gl.glLineWidth(4)
                draw_vertices(gl.GL_LINE_STRIP, flatten_points(step.segment), (1.0, 0.82, 0.15, 0.95))
            pulse = 0.13 + 0.03 * sin(prog * 40)
            draw_sphere((x, y, z), pulse * reveal, (1.0, 1.0, 1.0), *self.profile.sphere)
            gl.glLineWidth(2)
            c = 0.32 * reveal
            draw_vertices(gl.GL_LINES, [x - c, y, z, x + c, y, z, x, y - c, z, x, y + c, z, x, y, z - c, x, y, z + c], (1, 1, 1, 0.9))
        q = stage_progress(prog, 4)
        if q <= 0:
            return
        f0, f1, f2 = clamp01(q * 3), clamp01(q * 3 - 1), clamp01(q * 3 - 2)
        gl.glLineWidth(4)
        if rt == "cilindrico":
            theta = atan2(y, x)
            r = hypot(x, y)
            arc_r = max(0.45, min(0.9, r * 0.45))
            n = 28
            arc: list[float] = []
            for i in range(n + 1):
                a = theta * f0 * i / n
                arc.extend([arc_r * cos(a), arc_r * sin(a), 0.03])
            if f0 > 0:
                draw_vertices(gl.GL_LINE_STRIP, arc, cols[0])
            if f1 > 0:
                draw_vertices(gl.GL_LINES, [0, 0, 0.03, x * f1, y * f1, 0.03], cols[1])
            if f2 > 0:
                draw_vertices(gl.GL_LINES, [x, y, 0.03, x, y, 0.03 + (z - 0.03) * f2], cols[2])
        else:
            if f0 > 0:
                draw_vertices(gl.GL_LINES, [0, 0, 0.03, x * f0, 0, 0.03], cols[0])
            if f1 > 0:
                draw_vertices(gl.GL_LINES, [x, 0, 0.03, x, y * f1, 0.03], cols[1])
            if f2 > 0:
                draw_vertices(gl.GL_LINES, [x, y, 0.03, x, y, 0.03 + (z - 0.03) * f2], cols[2])

    def draw_grid(self) -> None:
        gl.glLineWidth(1)
        verts: list[float] = []
        for i in range(-6, 7):
            verts.extend([i, -6, 0, i, 6, 0, -6, i, 0, 6, i, 0])
        draw_vertices(gl.GL_LINES, verts, (0.38, 0.40, 0.42))

    def draw_axes(self) -> None:
        gl.glLineWidth(3)
        draw_vertices(gl.GL_LINES, [0, 0, 0, 1.2, 0, 0], (1, 0.12, 0.08))
        draw_vertices(gl.GL_LINES, [0, 0, 0, 0, 1.2, 0], (0.1, 0.9, 0.25))
        draw_vertices(gl.GL_LINES, [0, 0, 0, 0, 0, 1.2], (0.1, 0.25, 1))

    # ------------------------------------------------------------------ helpers 2D (cache de labels/retangulos)
    def text(
        self, key: str, s: str, x: float, y: float, size: float = 10, color: tuple[int, int, int, int] = TEXT,
        bold: bool = False, anchor: str = "left", width: int | None = None,
    ) -> pyglet.text.Label:
        label = self.labels.get(key)
        if label is None:
            label = pyglet.text.Label(
                s, x=int(x), y=int(y), font_size=size, weight="bold" if bold else "normal", color=color, anchor_x=anchor,
                width=width, multiline=width is not None,
            )
            self.labels[key] = label
        else:
            if label.text != s:
                label.text = s
            if (label.x, label.y) != (int(x), int(y)):
                label.position = (int(x), int(y), 0)
            if tuple(label.color) != tuple(color):
                label.color = color
        label.draw()
        return label

    def rect(self, key: str, x: float, y: float, w: float, h: float, color: tuple[int, int, int, int]) -> None:
        r = self.rects.get(key)
        if r is None:
            r = pyglet.shapes.Rectangle(x, y, w, h, color=color[:3])
            self.rects[key] = r
        r.position = (x, y)
        r.width, r.height = w, h
        r.color = color
        r.draw()

    def frame_box(self, key: str, x: float, y: float, w: float, h: float, fill: tuple[int, int, int, int], edge: tuple[int, int, int, int]) -> None:
        self.rect(key + "e", x - 1, y - 1, w + 2, h + 2, edge)
        self.rect(key + "f", x, y, w, h, fill)

    # ------------------------------------------------------------------ interface 2D
    def draw_ui(self) -> None:
        gl.glDisable(gl.GL_DEPTH_TEST)
        gl.glViewport(0, 0, int(self.width * self.get_pixel_ratio()), int(self.height * self.get_pixel_ratio()))
        self.projection = Mat4.orthogonal_projection(0, self.width, 0, self.height, -255, 255)
        self.view = Mat4()
        self.draw_top_bar()
        self.draw_code_panel()
        self.draw_bottom_panel()
        self.draw_scene_caption()

    def draw_top_bar(self) -> None:
        self.refresh_ui_hitboxes()
        h = self.height
        self.rect("top", 0, h - TOP_H, self.width, TOP_H, (42, 45, 48, 255))
        self.toolbar_checkbox("toggle_domain", "Dominio", self.show_domain)
        self.toolbar_checkbox("toggle_trace", "Rastro", self.show_trace)
        suffix = " (auto)" if self.mode == "auto" else " (fixo)"
        self.toolbar_button("render", f"Render: {self.profile.name}{suffix}", True, accent=(60, 90, 150, 255))
        self.toolbar_button("auto", "Pausar demo" if self.timeline and self.timeline.auto else "Retomar demo", False)
        self.toolbar_button("restart", "Reiniciar demo", False)
        self.toolbar_button("skip", "Ir p/ simulacao", False)
        self.text("t_fps", f"{self.fps:04.1f} fps", self.width - 12, h - 27, 10, (210, 218, 224, 255), anchor="right")

    def refresh_ui_hitboxes(self) -> None:
        yb = self.height - 34
        yc = self.height - 35
        top = self.height - TOP_H
        self.ui_hitboxes = {
            "toggle_domain": (14, yc, 80, 22),
            "toggle_trace": (102, yc, 74, 22),
            "render": (190, yb, 150, 24),
            "auto": (354, yb, 104, 24),
            "restart": (466, yb, 110, 24),
            "skip": (584, yb, 118, 24),
            "ex_cart": (12, top - 84, 74, 28),
            "ex_cyl": (92, top - 84, 74, 28),
            "ex_anim": (172, top - 84, 74, 28),
            "edit": (252, top - 84, 76, 28),
            "compile": (12, 152, LEFT_W - 24, 30),
        }

    def toolbar_button(self, action: str, label: str, active: bool, accent: tuple[int, int, int, int] = (30, 128, 96, 255)) -> None:
        x, y, w, h = self.ui_hitboxes[action]
        self.rect("b_" + action, x, y, w, h, accent if active else (70, 74, 78, 255))
        self.text("bl_" + action, label, x + 8, y + 7, 9, (245, 248, 250, 255))

    def toolbar_checkbox(self, action: str, label: str, checked: bool) -> None:
        x, y, w, h = self.ui_hitboxes[action]
        self.rect("c_" + action, x, y + 3, 16, 16, (235, 238, 240, 255) if checked else (92, 96, 100, 255))
        if checked:
            self.rect("cc_" + action, x + 4, y + 7, 8, 8, ACCENT)
        else:
            self.rect("cc_" + action, 0, 0, 0, 0, (0, 0, 0, 0))
        self.text("cl_" + action, label, x + 22, y + 5, 9, (238, 242, 245, 255))

    def draw_scene_caption(self) -> None:
        vx, vy, vw, vh = self.scene_rect()
        tl = self.timeline
        if not tl:
            return
        rt_name = self.limits[self.scene_type()]["name"] if self.scene_type() in self.limits else ""
        if tl.in_simulation:
            title, color = "SIMULACAO - o robo executa a trajetoria ja convertida em juntas", ACCENT
        elif tl.frame.phase == "erro":
            title, color = "PROGRAMA COM ERRO - a conversao parou aqui", WARN
        else:
            title, color = f"CONVERSAO {tl.index + 1}/{max(1, len(tl.frames) - (1 if self.report and self.report.ok else 0))} - do codigo ate as juntas", (255, 200, 90, 255)
        self.text("cap1", title, vx + 14, vy + vh - 26, 12, color, bold=True)
        self.text("cap2", f"{rt_name}   |   arraste: girar   roda: zoom   ESPACO: avancar   S: simulacao   R: reiniciar", vx + 14, vy + vh - 46, 9, MUTED)

    # ------------------------------------------------------------------ painel do codigo
    def draw_code_panel(self) -> None:
        h = self.height
        top = h - TOP_H
        self.frame_box("lp", 0, 0, LEFT_W, top, PANEL, PANEL_EDGE)
        self.text("lp_t", "PROGRAMA", 14, top - 28, 12, TEXT, bold=True)
        self.text("lp_s", "Ex. 1/2/3: exemplos   Editar: escrever codigo", 14, top - 46, 8, MUTED)
        for key, label in (("ex_cart", "Ex. 1"), ("ex_cyl", "Ex. 2"), ("ex_anim", "Ex. 3"), ("edit", "Editar")):
            x, y, w, bh = self.ui_hitboxes[key]
            self.rect("p_" + key, x, y, w, bh, (70, 74, 78, 255))
            self.text("pl_" + key, label, x + 12, y + 9, 9, (245, 248, 250, 255))
        rep = self.report
        view = self.view_state()
        current = view["step"].index if view["step"] is not None else -1
        if self.editing:
            area_h = self.editor_rect()[3] - 8
            self.rect("ed_bg", 8, 190, LEFT_W - 16, area_h + 8, (12, 15, 18, 255))
            x, y, w, bh = self.ui_hitboxes["compile"]
            fill, label = ((178, 48, 40, 255), "Erro - corrija e compile de novo") if self.edit_error else ((30, 128, 96, 255), "Compilar")
            self.rect("p_compile", x, y, w, bh, fill)
            self.text("pl_compile", label, x + 14, y + 10, 10, (245, 248, 250, 255), bold=True)
            layout = self.edit_layout
            if (layout.width, layout.height) != (LEFT_W - 28, area_h):
                layout.width, layout.height = LEFT_W - 28, area_h
            layout.position = (14, 194 + area_h, 0)
            self.edit_batch.draw()
            self.rect("lp_hl", 0, 0, 0, 0, (0, 0, 0, 0))
        elif rep and rep.steps:
            row_h = 24
            max_rows = max(1, (top - 130 - 140) // row_h)
            first = max(0, min(current - max_rows // 2, len(rep.steps) - max_rows)) if current >= 0 else 0
            for n, step in enumerate(rep.steps[first : first + max_rows]):
                y = top - 134 - n * row_h
                idx = step.index
                if step.error:
                    state = WARN
                elif idx == current:
                    state = ACCENT
                elif view["sim"] or (self.timeline and idx < current):
                    state = (60, 120, 150, 255)
                else:
                    state = (70, 76, 82, 255)
                if idx == current:
                    self.rect("lp_hl", 6, y - 6, LEFT_W - 12, row_h - 2, (40, 56, 52, 255))
                self.rect(f"lp_m{n}", 12, y + 1, 5, 12, state)
                color = TEXT if idx == current else MUTED
                self.text(f"lp_n{n}", f"{idx + 1:02d}", 24, y, 9, (110, 120, 130, 255))
                self.text(f"lp_c{n}", fit(step.text + ";", 38), 50, y, 10, color)
            if current < 0:
                self.rect("lp_hl", 0, 0, 0, 0, (0, 0, 0, 0))
        elif rep:
            self.text("lp_raw", fit(self.source.strip().replace("\n", " "), 40), 14, top - 134, 10, MUTED)
        color = ACCENT if self.error.startswith("OK") else WARN
        self.text("lp_st", self.error, 14, 118, 9, color, width=LEFT_W - 28)
        self.text(
            "lp_help", "ESPACO/Seta dir: proximo   Seta esq: voltar\nP: pausa   S: simulacao   R: reiniciar\nG: render auto/GPU/CPU   D: dominio   T: rastro\nF1/F2/F3: exemplos   F5: recompilar\nEditando: Ctrl+Enter compila, Esc cancela", 14, 58, 8, MUTED, width=LEFT_W - 28,
        )

    # ------------------------------------------------------------------ painel inferior (a conversao)
    def draw_bottom_panel(self) -> None:
        x0 = LEFT_W
        w = self.width - LEFT_W
        self.frame_box("bp", x0, 0, w, BOTTOM_H - 2, PANEL, PANEL_EDGE)
        view = self.view_state()
        step, prog = view["step"], view["progress"]
        rt = self.scene_type()
        if step is None:
            msg = self.error if self.report and not self.report.ok else "Sem comando para mostrar."
            self.text("bp_none", msg, x0 + 18, BOTTOM_H // 2, 11, WARN if self.report and not self.report.ok else MUTED, width=w - 40)
            return
        self.draw_stage_strip(step, prog, x0, w, view)
        col_w = (w - 48) / 3
        cx = [x0 + 16, x0 + 24 + col_w, x0 + 32 + 2 * col_w]
        top = BOTTOM_H - 82
        self.draw_col_tokens(step, prog, cx[0], top, col_w)
        self.draw_col_ik(step, prog, cx[1], top, col_w, rt)
        self.draw_col_joints(step, prog, cx[2], top, col_w, rt, view)

    def draw_stage_strip(self, step: Any, prog: float, x0: int, w: int, view: dict[str, Any]) -> None:
        n = len(STAGE_TITLES)
        gap = 22
        bw = (w - 32 - gap * (n - 1)) / n
        cur = active_stage(prog)
        rt = self.scene_type()
        j = view["joints"]
        contents = [
            fit(step.text, 26),
            f"{len(step.tokens)} tokens",
            f"{step.fields[0][1]} + {max(0, len(step.fields) - 1)} campos" if step.fields else "-",
            f"({step.target[0]:.2f}, {step.target[1]:.2f}, {step.target[2]:.2f})" if step.target else "sem movimento",
            ("theta, r, z" if rt == "cilindrico" else "J1=x J2=y J3=z") if step.target else "-",
            (f"({j[0]:.1f}, {j[1]:.2f}, {j[2]:.2f})" if j else "-"),
        ]
        y = BOTTOM_H - 66
        for i in range(n):
            x = x0 + 16 + i * (bw + gap)
            reached = prog >= STAGE_START[i] - 1e-9
            if i == cur and not view["sim"]:
                fill, edge = (30, 110, 82, 255), (90, 230, 170, 255)
            elif reached:
                fill, edge = (34, 52, 60, 255), (70, 110, 130, 255)
            else:
                fill, edge = (30, 34, 38, 255), (52, 58, 64, 255)
            self.frame_box(f"st{i}", x, y, bw, 52, fill, edge)
            tcol = TEXT if reached else (90, 98, 106, 255)
            self.text(f"stt{i}", f"{i + 1}. {STAGE_TITLES[i]}", x + 8, y + 33, 9, tcol, bold=True)
            self.text(f"stc{i}", contents[i] if reached else "...", x + 8, y + 12, 9, (170, 220, 255, 255) if reached else (80, 88, 96, 255))
            if i < n - 1:
                self.text(f"sta{i}", ">", x + bw + 5, y + 18, 14, ACCENT if reached else (80, 88, 96, 255))

    def token_chips(self, step: Any, col_w: float) -> list[tuple[pyglet.text.Label, int, int, int, tuple[int, int, int, int]]]:
        key = (step.index, step.text, int(col_w))
        if self.chip_cache[0] == key:
            return self.chip_cache[1]
        chips = []
        x = y = 0
        for kind, tok in step.tokens:
            lab = pyglet.text.Label(tok, font_size=10, color=TOKEN_COLORS.get(kind, TEXT), x=0, y=0)
            wch = int(lab.content_width) + 12
            if x + wch > col_w:
                x, y = 0, y + 26
            chips.append((lab, x, y, wch, TOKEN_COLORS.get(kind, TEXT)))
            x += wch + 4
        self.chip_cache = (key, chips)
        return chips

    def draw_col_tokens(self, step: Any, prog: float, cx: float, top: float, col_w: float) -> None:
        self.text("c1_t", "CODIGO > TOKENS > COMANDO", cx, top + 4, 9, ACCENT, bold=True)
        chips = self.token_chips(step, col_w)
        shown = int(round(len(chips) * stage_progress(prog, 1)))
        for i, (lab, ox, oy, wch, color) in enumerate(chips):
            if i >= shown:
                break
            y = top - 26 - oy
            self.rect(f"chip{i}", cx + ox, y - 5, wch, 22, (color[0] // 6, color[1] // 6, color[2] // 6, 255))
            lab.position = (int(cx + ox + 6), int(y), 0)
            lab.draw()
        for i in range(shown, 40):
            if f"chip{i}" in self.rects:
                self.rects[f"chip{i}"].width = 0
        rows = (chips[-1][2] // 26 + 1) if chips else 1
        fy = top - 26 - rows * 26 - 10
        reveal = stage_progress(prog, 2)
        if reveal > 0:
            self.text("c1_h", "analise sintatica (comando estruturado):", cx, fy + 8, 8, MUTED)
            n_fields = int(round(len(step.fields) * reveal + 0.49))
            for i, (k, v) in enumerate(step.fields[:n_fields]):
                if fy - i * 17 < 14:
                    break
                self.text(f"c1_k{i}", k, cx + 4, fy - 12 - i * 17, 9, TOKEN_COLORS["parametro"] if k not in {"comando"} else TOKEN_COLORS["comando"])
                self.text(f"c1_v{i}", "= " + v, cx + 74, fy - 12 - i * 17, 9, TEXT)

    def draw_col_ik(self, step: Any, prog: float, cx: float, top: float, col_w: float, rt: str) -> None:
        self.text("c2_t", "XYZ > JUNTAS (CINEMATICA INVERSA)", cx, top + 4, 9, ACCENT, bold=True)
        if step.error:
            self.text("c2_n", f"ERRO {step.error[0].upper()}\n{step.error[1]}\n\nA simulacao nao inicia ate corrigir este comando.", cx, top - 24, 10, WARN, width=int(col_w))
            return
        if not step.target:
            self.text("c2_n", "\n".join(step.notes) or "Este comando nao move o robo.", cx, top - 24, 10, MUTED, width=int(col_w))
            return
        if stage_progress(prog, 3) > 0:
            x, y, z = step.target
            self.text("c2_xyz", f"alvo  x={x:.2f}  y={y:.2f}  z={z:.2f}", cx, top - 24, 11, TEXT, bold=True)
        q = stage_progress(prog, 4)
        shown = int(q * 3 + 0.999) if q > 0 else 0
        cols = [rgb255(j["color"]) for j in self.limits[rt]["joints"]]
        for i, line in enumerate(step.ik_lines[:shown]):
            y = top - 52 - i * 48
            self.rect(f"ikc{i}", cx, y - 24, 5, 40, cols[line.joint])
            self.text(f"ik1_{i}", f"{line.label}:  {line.formula}", cx + 12, y, 9, cols[line.joint], bold=True)
            tail = "ok" if line.ok else "FORA DO LIMITE"
            self.text(f"ik2_{i}", f"= {line.substituted} = {line.result}", cx + 12, y - 16, 10, TEXT)
            self.text(f"ik3_{i}", f"limite {line.limit}  {tail}", cx + 12, y - 30, 8, ACCENT if line.ok else WARN)
        for i in range(shown, 3):
            if f"ikc{i}" in self.rects:
                self.rects[f"ikc{i}"].width = 0

    def joint_ranges(self, rt: str) -> list[tuple[str, tuple[float, float], str]]:
        ws = self.limits[rt]["workspace"]
        names = [j["name"] for j in self.limits[rt]["joints"]]
        if rt == "cartesiano":
            return [(names[0], tuple(ws["x"]), "m"), (names[1], tuple(ws["y"]), "m"), (names[2], tuple(ws["z"]), "m")]
        return [(names[0], tuple(ws["theta_deg"]), "deg"), (names[1], tuple(ws["radius"]), "m"), (names[2], tuple(ws["z"]), "m")]

    def draw_col_joints(self, step: Any, prog: float, cx: float, top: float, col_w: float, rt: str, view: dict[str, Any]) -> None:
        title = "SIMULACAO: JUNTAS AO VIVO" if view["sim"] else "JUNTAS > ROBO (CIN. DIRETA)"
        self.text("c3_t", title, cx, top + 4, 9, ACCENT, bold=True)
        joints = view["joints"]
        if joints is None:
            return
        cols = [rgb255(j["color"]) for j in self.limits[rt]["joints"]]
        bar_w = col_w - 6
        for i, (name, (lo, hi), unit) in enumerate(self.joint_ranges(rt)):
            y = top - 28 - i * 38
            v = joints[i]
            self.text(f"c3n{i}", name, cx, y, 9, cols[i], bold=True)
            self.text(f"c3v{i}", f"{v:7.2f} {unit}", cx + bar_w, y, 10, TEXT, anchor="right")
            self.rect(f"c3b{i}", cx, y - 12, bar_w, 6, (46, 52, 58, 255))
            frac = clamp01((v - lo) / (hi - lo)) if hi > lo else 0.0
            self.rect(f"c3f{i}", cx, y - 12, max(2, bar_w * frac), 6, cols[i])
        y = top - 28 - 3 * 38 + 4
        if view["sim"]:
            k = self.sim_command()
            total = max(1, len(self.cmd_bounds))
            self.text("c3_f", f"executando comando {k + 1}/{total}: {fit(step.text, 24) if step else ''}", cx, y, 9, TEXT)
            self.text("c3_f2", "cada quadro: ponto XYZ > IK > juntas", cx, y - 16, 8, MUTED)
        elif stage_progress(prog, 5) > 0 and step.target:
            ok = step.fk_error < 1e-6
            self.text("c3_f", step.fk_text, cx, y, 8, TEXT, width=int(col_w))
            self.text("c3_f2", f"erro FK vs alvo = {step.fk_error:.1e}  {'(consistente)' if ok else '(INCONSISTENTE)'}", cx, y - 28, 9, ACCENT if ok else WARN)
            for i, note in enumerate(step.notes[:2]):
                self.text(f"c3_n{i}", fit(note, 52), cx, y - 46 - i * 14, 8, MUTED)
        else:
            self.text("c3_f", "", cx, y, 8, TEXT)

    # ------------------------------------------------------------------ entrada
    def editor_rect(self) -> tuple[int, int, int, int]:
        top = self.height - TOP_H
        return (8, 190, LEFT_W - 16, max(60, top - 130 - 196) + 8)

    def in_editor(self, x: float, y: float) -> bool:
        ex, ey, ew, eh = self.editor_rect()
        return self.editing and ex <= x <= ex + ew and ey <= y <= ey + eh

    # O Caret do pyglet captura todos os eventos se for empilhado na janela; por isso
    # encaminhamos so teclado e os cliques que caem dentro da caixa de codigo.
    def on_text(self, text: str) -> None:
        if self.editing and text not in ("\x1b",):
            self.edit_caret.on_text(text)

    def on_text_motion(self, motion: int) -> None:
        if self.editing:
            self.edit_caret.on_text_motion(motion)

    def on_text_motion_select(self, motion: int) -> None:
        if self.editing:
            self.edit_caret.on_text_motion_select(motion)

    def on_key_press(self, symbol: int, modifiers: int) -> None:
        key = pyglet.window.key
        tl = self.timeline
        if self.editing:
            if symbol == key.F5 or (symbol in (key.ENTER, key.RETURN) and modifiers & key.MOD_CTRL):
                self.compile_edit()
            elif symbol == key.ESCAPE:
                self.stop_editing()
            return
        if symbol == key.ESCAPE:
            self.close()
        elif symbol == key.F5:
            self.compile_now()
        elif symbol == key.F1:
            self.set_source(DEFAULT_SOURCE)
        elif symbol == key.F2:
            self.set_source(CYLINDRICAL_EXAMPLE)
        elif symbol == key.F3:
            self.set_source(ANIMATION_TEST_EXAMPLE)
        elif symbol in (key.SPACE, key.RIGHT, key.ENTER) and tl:
            tl.next()
        elif symbol in (key.LEFT, key.BACKSPACE) and tl:
            tl.prev()
        elif symbol == key.P and tl:
            tl.auto = not tl.auto
        elif symbol == key.S and tl and self.report and (self.report.ok):
            tl.goto_end()
        elif symbol == key.R and tl:
            tl.restart()
        elif symbol == key.G:
            self.cycle_render_mode()
        elif symbol == key.D:
            self.show_domain = not self.show_domain
        elif symbol == key.T:
            self.show_trace = not self.show_trace

    def on_mouse_press(self, x: int, y: int, button: int, modifiers: int) -> None:
        self.refresh_ui_hitboxes()
        for action, (bx, by, bw, bh) in self.ui_hitboxes.items():
            if bx <= x <= bx + bw and by <= y <= by + bh:
                tl = self.timeline
                if action == "ex_cart":
                    self.set_source(DEFAULT_SOURCE)
                elif action == "ex_cyl":
                    self.set_source(CYLINDRICAL_EXAMPLE)
                elif action == "ex_anim":
                    self.set_source(ANIMATION_TEST_EXAMPLE)
                elif action == "edit":
                    self.start_editing()
                elif action == "compile":
                    if self.editing:
                        self.compile_edit()
                elif action == "toggle_domain":
                    self.show_domain = not self.show_domain
                elif action == "toggle_trace":
                    self.show_trace = not self.show_trace
                elif action == "render":
                    self.cycle_render_mode()
                elif action == "auto" and tl:
                    tl.auto = not tl.auto
                elif action == "restart" and tl:
                    tl.restart()
                elif action == "skip" and tl and self.report and self.report.ok:
                    tl.goto_end()
                return
        self.drag_in_editor = self.in_editor(x, y)
        if self.drag_in_editor:
            self.edit_caret.on_mouse_press(x, y, button, modifiers)

    def on_mouse_drag(self, x: int, y: int, dx: int, dy: int, buttons: int, modifiers: int) -> None:
        if self.drag_in_editor:
            self.edit_caret.on_mouse_drag(x, y, dx, dy, buttons, modifiers)
            return
        vx, vy, vw, vh = self.scene_rect()
        if buttons & pyglet.window.mouse.LEFT and vx <= x <= vx + vw and vy <= y <= vy + vh:
            self.camera_yaw += dx * 0.4
            self.camera_pitch = max(5, min(80, self.camera_pitch + dy * 0.3))

    def on_mouse_scroll(self, x: int, y: int, scroll_x: float, scroll_y: float) -> None:
        if self.in_editor(x, y):
            self.edit_caret.on_mouse_scroll(x, y, scroll_x, scroll_y)
            return
        self.zoom = max(4.0, min(20.0, self.zoom - scroll_y * 0.5))

    # ------------------------------------------------------------------ capturas (README / testes visuais)
    def plan_shots(self, directory: Path) -> None:
        directory.mkdir(parents=True, exist_ok=True)
        tl, rep = self.timeline, self.report
        if not tl or not rep:
            return
        motion = [f for f in tl.frames if f.phase == "passo" and rep.steps[f.step].kind == "motion"]
        picks: list[tuple[str, int, float]] = []
        if tl.frames:
            picks.append(("01_codigo_tokens", 1 if len(tl.frames) > 1 else 0, 0.22))
        if motion:
            first = tl.frames.index(motion[0])
            picks.append(("02_alvo_xyz", first, 0.48))
            picks.append(("03_cinematica_inversa", first, 0.74))
            picks.append(("04_juntas_robo", first, 1.0))
            if len(motion) > 2:
                picks.append(("05_arco_movec", tl.frames.index(motion[2]), 0.9))
        if rep.ok:
            picks.append(("06_simulacao", len(tl.frames) - 1, 0.0))
        else:
            picks.append(("07_erro", len(tl.frames) - 1, 1.0))
        self.shot_plan = picks

    def step_shots(self) -> None:
        if not self.shots_dir or not self.timeline:
            return
        if self.shot_wait > 0:
            self.shot_wait -= 1
            return
        if not self.shot_plan:
            if not self.shot_pending:
                pyglet.app.exit()
            return
        name, index, progress = self.shot_plan.pop(0)
        tl = self.timeline
        tl.auto = False
        tl.index = min(index, len(tl.frames) - 1)
        tl.t = progress * min(tl.frame.dwell, 1e6)
        if tl.in_simulation:
            self.sim_time = 4.2
        self.shot_pending = self.shots_dir / f"{name}.png"
        self.shot_wait = 4


    def draw_trace(self) -> None:
        gl.glLineWidth(4)
        draw_vertices(gl.GL_LINE_STRIP, self.trace_vertices, (0.0, 0.95, 1.0))

    def draw_workspace_domain(self, robot_type: str) -> None:
        gl.glDepthMask(False)
        cfg = self.limits[robot_type]["workspace"]
        if robot_type == "cartesiano":
            xmin, xmax = cfg["x"]
            ymin, ymax = cfg["y"]
            zmin, zmax = cfg["z"]
            center = ((xmin + xmax) / 2, (ymin + ymax) / 2, (zmin + zmax) / 2)
            size = (xmax - xmin, ymax - ymin, zmax - zmin)
            if self.profile.domain_fill:
                draw_box(center, size, (0.20, 0.75, 1.0, 0.12))
            draw_box_edges(center, size, (0.25, 0.88, 1.0, 0.62))
        elif robot_type == "cilindrico":
            theta_min, theta_max = cfg["theta_deg"]
            radius_min, radius_max = cfg["radius"]
            zmin, zmax = cfg["z"]
            key = f"cilindrico:{radius_min}:{radius_max}:{zmin}:{zmax}:{theta_min}:{theta_max}:{self.profile.name}"
            if key not in self.workspace_mesh_cache:
                steps = self.profile.domain_steps
                self.workspace_mesh_cache[key] = build_cylindrical_workspace_mesh(radius_min, radius_max, zmin, zmax, theta_min, theta_max, steps)
            verts, edge_verts = self.workspace_mesh_cache[key]
            if self.profile.domain_fill:
                draw_vertices(gl.GL_TRIANGLES, verts, (0.20, 0.75, 1.0, 0.13))
            gl.glLineWidth(2)
            draw_vertices(gl.GL_LINES, edge_verts, (0.25, 0.88, 1.0, 0.65))
        gl.glDepthMask(True)

    def draw_cartesian_robot(self, point: tuple[float, float, float]) -> None:
        x, y, z = point
        cfg = self.limits["cartesiano"]["workspace"]
        xmin, xmax = cfg["x"]
        ymin, ymax = cfg["y"]
        zmin, zmax = cfg["z"]
        rail_z = zmax + 0.45
        y_front = ymin - 0.45
        y_back = ymax + 0.45
        x_left = xmin - 0.45
        x_right = xmax + 0.45
        table_center = ((xmin + xmax) / 2, (ymin + ymax) / 2, zmin - 0.12)
        table_size = (xmax - xmin + 0.9, ymax - ymin + 0.9, 0.18)
        draw_box(table_center, table_size, (0.34, 0.36, 0.37))
        for px in (x_left, x_right):
            for py in (y_front, y_back):
                draw_box((px, py, rail_z / 2), (0.18, 0.18, rail_z), (0.62, 0.65, 0.66))
        draw_box(((xmin + xmax) / 2, y_front, rail_z), (xmax - xmin + 1.1, 0.16, 0.16), (0.78, 0.80, 0.82))
        draw_box(((xmin + xmax) / 2, y_back, rail_z), (xmax - xmin + 1.1, 0.16, 0.16), (0.78, 0.80, 0.82))
        draw_box((x, (y_front + y_back) / 2, rail_z - 0.1), (0.22, ymax - ymin + 0.9, 0.20), (0.95, 0.82, 0.12))
        draw_box((x, y, rail_z - 0.18), (0.62, 0.38, 0.28), (0.12, 0.55, 1.0))
        draw_box((x, y, (rail_z + z) / 2 - 0.08), (0.18, 0.18, max(0.1, rail_z - z)), (0.10, 0.85, 0.45))
        draw_box((x, y, z - 0.18), (0.10, 0.10, 0.36), (0.08, 0.09, 0.10))
        quality = self.profile.sphere
        draw_sphere((x, y_front, rail_z), 0.16, (0.12, 0.55, 1.0), *quality)
        draw_sphere((x, y, rail_z - 0.16), 0.17, (0.95, 0.85, 0.10), *quality)
        draw_sphere((x, y, z), 0.15, (0.10, 0.85, 0.45), *quality)

    def draw_cylindrical_robot(self, point: tuple[float, float, float]) -> None:
        x, y, z = point
        cfg = self.limits["cilindrico"]["workspace"]
        radius_min = cfg["radius"][0]
        zmax = cfg["z"][1]
        radius = max(0.001, (x * x + y * y) ** 0.5)
        ux, uy = x / radius, y / radius
        arm_start = (ux * radius_min, uy * radius_min, z)
        arm_length = max(0.1, radius - radius_min)
        arm_center = ((arm_start[0] + x) / 2, (arm_start[1] + y) / 2, z)
        draw_cylinder((0, 0, 0.08), 0.58, 0.16, (0.72, 0.73, 0.74), self.profile.cylinder_steps)
        draw_cylinder((0, 0, (zmax + 0.35) / 2), 0.22, zmax + 0.35, (0.60, 0.63, 0.65), self.profile.cylinder_steps)
        draw_cylinder((0, 0, z), 0.34, 0.28, (1.0, 0.25, 0.10), self.profile.cylinder_steps)
        draw_box(arm_center, (arm_length, 0.16, 0.16), (0.10, 0.60, 1.0), angle_deg=degrees2d(x, y))
        draw_box((x, y, z - 0.22), (0.14, 0.14, 0.44), (0.08, 0.09, 0.10))
        quality = self.profile.sphere
        draw_sphere((0, 0, z), 0.22, (1.0, 0.25, 0.10), *quality)
        draw_sphere(arm_start, 0.16, (0.15, 0.90, 0.35), *quality)
        draw_sphere((x, y, z), 0.20, (0.10, 0.60, 1.0), *quality)


def degrees2d(x: float, y: float) -> float:
    from math import atan2, degrees

    return degrees(atan2(y, x))


def draw_rect(x: float, y: float, w: float, h: float, color: tuple[int, int, int]) -> None:
    pyglet.shapes.Rectangle(x, y, w, h, color=color).draw()


def flatten_points(points: list[tuple[float, float, float]]) -> list[float]:
    verts: list[float] = []
    for x, y, z in points:
        verts.extend([x, y, z])
    return verts


def draw_vertices(mode: int, verts: list[float], color: tuple[float, ...]) -> None:
    count = len(verts) // 3
    if count == 0:
        return
    alpha = color[3] if len(color) > 3 else 1.0
    rgba = [int(color[0] * 255), int(color[1] * 255), int(color[2] * 255), int(alpha * 255)] * count
    tex = [0.0, 0.0, 0.0] * count
    pyglet.graphics.draw(count, mode, position=("f", verts), colors=("Bn", rgba), tex_coords=("f", tex))


def draw_box(center: tuple[float, float, float], size: tuple[float, float, float], color: tuple[float, ...], angle_deg: float = 0.0) -> None:
    cx, cy, cz = center
    sx, sy, sz = (size[0] / 2, size[1] / 2, size[2] / 2)
    verts = [(-sx, -sy, -sz), (sx, -sy, -sz), (sx, sy, -sz), (-sx, sy, -sz), (-sx, -sy, sz), (sx, -sy, sz), (sx, sy, sz), (-sx, sy, sz)]
    a = radians(angle_deg)
    rotated = []
    for x, y, z in verts:
        rotated.append((cx + x * cos(a) - y * sin(a), cy + x * sin(a) + y * cos(a), cz + z))
    faces = [(0, 1, 2, 0, 2, 3), (4, 7, 6, 4, 6, 5), (0, 4, 5, 0, 5, 1), (1, 5, 6, 1, 6, 2), (2, 6, 7, 2, 7, 3), (3, 7, 4, 3, 4, 0)]
    out: list[float] = []
    for face in faces:
        for index in face:
            out.extend(rotated[index])
    draw_vertices(gl.GL_TRIANGLES, out, color)


def draw_box_edges(center: tuple[float, float, float], size: tuple[float, float, float], color: tuple[float, ...]) -> None:
    cx, cy, cz = center
    sx, sy, sz = (size[0] / 2, size[1] / 2, size[2] / 2)
    points = [
        (cx - sx, cy - sy, cz - sz),
        (cx + sx, cy - sy, cz - sz),
        (cx + sx, cy + sy, cz - sz),
        (cx - sx, cy + sy, cz - sz),
        (cx - sx, cy - sy, cz + sz),
        (cx + sx, cy - sy, cz + sz),
        (cx + sx, cy + sy, cz + sz),
        (cx - sx, cy + sy, cz + sz),
    ]
    edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]
    verts: list[float] = []
    for a, b in edges:
        verts.extend(points[a])
        verts.extend(points[b])
    gl.glLineWidth(2)
    draw_vertices(gl.GL_LINES, verts, color)


def build_cylindrical_workspace_mesh(
    radius_min: float,
    radius_max: float,
    zmin: float,
    zmax: float,
    theta_min: float,
    theta_max: float,
    steps: int,
) -> tuple[list[float], list[float]]:
    verts: list[float] = []
    edge_verts: list[float] = []
    for i in range(steps):
        t0 = radians(theta_min + (theta_max - theta_min) * i / steps)
        t1 = radians(theta_min + (theta_max - theta_min) * (i + 1) / steps)
        outer0_low = (radius_max * cos(t0), radius_max * sin(t0), zmin)
        outer0_high = (radius_max * cos(t0), radius_max * sin(t0), zmax)
        outer1_low = (radius_max * cos(t1), radius_max * sin(t1), zmin)
        outer1_high = (radius_max * cos(t1), radius_max * sin(t1), zmax)
        inner0_low = (radius_min * cos(t0), radius_min * sin(t0), zmin)
        inner0_high = (radius_min * cos(t0), radius_min * sin(t0), zmax)
        inner1_low = (radius_min * cos(t1), radius_min * sin(t1), zmin)
        inner1_high = (radius_min * cos(t1), radius_min * sin(t1), zmax)

        for point in (outer0_low, outer1_low, outer1_high, outer0_low, outer1_high, outer0_high):
            verts.extend(point)
        for point in (inner0_low, inner0_high, inner1_high, inner0_low, inner1_high, inner1_low):
            verts.extend(point)
        for point in (inner0_high, outer0_high, outer1_high, inner0_high, outer1_high, inner1_high):
            verts.extend(point)
        for point in (inner0_low, inner1_low, outer1_low, inner0_low, outer1_low, outer0_low):
            verts.extend(point)

        if i % 6 == 0:
            edge_verts.extend(outer0_low)
            edge_verts.extend(outer0_high)
            edge_verts.extend(inner0_low)
            edge_verts.extend(inner0_high)

    for theta in (radians(theta_min), radians(theta_max)):
        for radius in (radius_min, radius_max):
            edge_verts.extend((radius * cos(theta), radius * sin(theta), zmin))
            edge_verts.extend((radius * cos(theta), radius * sin(theta), zmax))
    for z in (zmin, zmax):
        for radius in (radius_min, radius_max):
            for i in range(steps):
                t0 = radians(theta_min + (theta_max - theta_min) * i / steps)
                t1 = radians(theta_min + (theta_max - theta_min) * (i + 1) / steps)
                edge_verts.extend((radius * cos(t0), radius * sin(t0), z))
                edge_verts.extend((radius * cos(t1), radius * sin(t1), z))

    return verts, edge_verts


def draw_cylinder(center: tuple[float, float, float], radius: float, height: float, color: tuple[float, ...], steps: int = 24) -> None:
    cx, cy, cz = center
    z_low = cz - height / 2
    z_high = cz + height / 2
    verts: list[float] = []
    for i in range(steps):
        t0 = radians(360 * i / steps)
        t1 = radians(360 * (i + 1) / steps)
        p0_low = (cx + radius * cos(t0), cy + radius * sin(t0), z_low)
        p1_low = (cx + radius * cos(t1), cy + radius * sin(t1), z_low)
        p0_high = (cx + radius * cos(t0), cy + radius * sin(t0), z_high)
        p1_high = (cx + radius * cos(t1), cy + radius * sin(t1), z_high)
        top = (cx, cy, z_high)
        bottom = (cx, cy, z_low)
        for point in (p0_low, p1_low, p1_high, p0_low, p1_high, p0_high):
            verts.extend(point)
        for point in (top, p0_high, p1_high):
            verts.extend(point)
        for point in (bottom, p1_low, p0_low):
            verts.extend(point)
    draw_vertices(gl.GL_TRIANGLES, verts, color)


def draw_sphere(center: tuple[float, float, float], radius: float, color: tuple[float, ...], slices: int = 12, stacks: int = 8) -> None:
    cx, cy, cz = center
    verts: list[float] = []
    for i in range(stacks):
        lat0 = radians(-90 + 180 * i / stacks)
        lat1 = radians(-90 + 180 * (i + 1) / stacks)
        for j in range(slices + 1):
            lng = radians(360 * j / slices)
            lng_next = radians(360 * (j + 1) / slices)
            p1 = (cx + radius * cos(lat0) * cos(lng), cy + radius * cos(lat0) * sin(lng), cz + radius * sin(lat0))
            p2 = (cx + radius * cos(lat1) * cos(lng), cy + radius * cos(lat1) * sin(lng), cz + radius * sin(lat1))
            p3 = (cx + radius * cos(lat1) * cos(lng_next), cy + radius * cos(lat1) * sin(lng_next), cz + radius * sin(lat1))
            p4 = (cx + radius * cos(lat0) * cos(lng_next), cy + radius * cos(lat0) * sin(lng_next), cz + radius * sin(lat0))
            for point in (p1, p2, p3, p1, p3, p4):
                verts.extend(point)
    draw_vertices(gl.GL_TRIANGLES, verts, color)


USAGE = """RobotGL - do codigo ao simulador de cinematica

uso: python main.py [programa.robot] [opcoes]

  --auto | --gpu | --cpu   perfil de render (padrao: auto = detecta GPU)
  --terminal               mostra a conversao em texto, sem janela/GPU
  --shots DIR              salva capturas da demonstracao e sai
  --help                   esta ajuda
Variavel de ambiente: ROBOTGL_RENDER=auto|gpu|cpu
"""


def main(argv: list[str]) -> int:
    if "--help" in argv or "-h" in argv:
        print(USAGE)
        return 0
    mode = requested_mode(argv)
    shots: Path | None = None
    if "--shots" in argv:
        shots = Path(argv[argv.index("--shots") + 1])
    files = [a for a in argv if a.endswith(".robot")]
    source = Path(files[0]).read_text(encoding="utf-8") if files else None

    def terminal_report() -> int:
        report = analyze(source or CYLINDRICAL_EXAMPLE, load_limits())
        print(format_report(report))
        return 0 if report.ok else 1

    if "--terminal" in argv:
        return terminal_report()
    try:
        renderer = probe_renderer()
    except Exception as exc:  # sem display / sem OpenGL
        print(f"[render] janela OpenGL indisponivel ({exc.__class__.__name__}: {exc}).")
        print("[render] executando a demonstracao em modo terminal.\n")
        return terminal_report()
    profile = choose_profile(mode, renderer)
    print(f"[render] {renderer} -> perfil {profile.name} (modo {mode})")
    app = RobotApp(mode, profile, renderer, shots)
    if source is not None:
        app.set_source(source)
        if shots:
            app.plan_shots(shots)
    pyglet.app.run(interval=None)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
