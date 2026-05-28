from __future__ import annotations

import json
from math import cos, radians, sin
import queue
import threading
import tkinter as tk
from tkinter import scrolledtext
from pathlib import Path
from typing import Any

import pyglet
from pyglet import gl
from pyglet.math import Mat4, Vec3

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


def run_code_editor(
    initial_source: str,
    editor_events: queue.Queue[tuple[str, str]],
    editor_commands: queue.Queue[tuple[str, str]],
) -> None:
    root = tk.Tk()
    root.title("Editor de codigo do robo")
    root.geometry("620x620")
    root.minsize(500, 420)

    header = tk.Frame(root, bg="#252a2f", padx=14, pady=10)
    header.pack(fill=tk.X)
    tk.Label(
        header,
        text="Codigo do robo",
        fg="#f5f7fa",
        bg="#252a2f",
        font=("Segoe UI", 12, "bold"),
    ).pack(anchor="w")
    tk.Label(
        header,
        text="Edite aqui. Quando terminar, clique em Compilar para atualizar a visualizacao 3D.",
        fg="#d8dee6",
        bg="#252a2f",
        font=("Segoe UI", 9),
    ).pack(anchor="w", pady=(3, 0))

    text = scrolledtext.ScrolledText(root, wrap=tk.NONE, undo=True, font=("Consolas", 11))
    text.pack(fill=tk.BOTH, expand=True, padx=14, pady=(14, 8))
    text.insert("1.0", initial_source)

    footer = tk.Frame(root, padx=14, pady=10)
    footer.pack(fill=tk.X)
    status_var = tk.StringVar(value="Edicao pronta. Clique em Compilar para enviar ao 3D.")
    status = tk.Label(footer, textvariable=status_var, anchor="w", justify="left", font=("Segoe UI", 9))
    status.pack(side=tk.LEFT, fill=tk.X, expand=True)

    def submit_compile() -> None:
        editor_events.put(("compile", text.get("1.0", "end-1c")))

    tk.Button(
        footer,
        text="Compilar",
        command=submit_compile,
        bg="#188f63",
        fg="#ffffff",
        activebackground="#126b4a",
        activeforeground="#ffffff",
        padx=18,
        pady=6,
    ).pack(side=tk.RIGHT, padx=(12, 0))

    def poll_commands() -> None:
        while True:
            try:
                action, payload = editor_commands.get_nowait()
            except queue.Empty:
                break
            if action == "status":
                status_var.set(payload)
                status.configure(fg="#16703f" if payload.startswith("OK") else "#b3261e")
            elif action == "set_source":
                text.delete("1.0", tk.END)
                text.insert("1.0", payload)
        root.after(100, poll_commands)

    poll_commands()
    root.mainloop()


class RobotApp(pyglet.window.Window):
    def __init__(self) -> None:
        super().__init__(960, 720, "Cinematica de Robos - OpenGL", resizable=True, vsync=True)
        self.limits = load_limits()
        self.source = DEFAULT_SOURCE
        self.result: ProgramResult | None = None
        self.error = ""
        self.active_tab = "cartesiano"
        self.play_time = 0.0
        self.camera_yaw = 42.0
        self.camera_pitch = 28.0
        self.zoom = 9.0
        self.show_domain = True
        self.show_trace = True
        self.gpu_render = True
        self.ui_hitboxes: dict[str, tuple[int, int, int, int]] = {}
        self.workspace_mesh_cache: dict[str, tuple[list[float], list[float]]] = {}
        self.trace_vertices: list[float] = []
        self.fps = 0.0
        self.editor_events: queue.Queue[tuple[str, str]] = queue.Queue()
        self.editor_commands: queue.Queue[tuple[str, str]] = queue.Queue()
        self.compile_now()
        self.start_code_editor()
        pyglet.clock.schedule_interval(self.update, 1 / 60)

    def compile_now(self) -> None:
        try:
            self.result = compile_program(self.source, self.limits)
            self.trace_vertices = flatten_points(self.result.path)
            self.error = "OK - programa compilado pelo botao Compilar."
            self.active_tab = self.result.robot_type
        except (LexicalError, SyntaxRobotError, SemanticError) as exc:
            self.error = f"{exc.__class__.__name__}: {exc}"
        self.editor_commands.put(("status", self.error))

    def start_code_editor(self) -> None:
        thread = threading.Thread(
            target=run_code_editor,
            args=(self.source, self.editor_events, self.editor_commands),
            daemon=True,
        )
        thread.start()

    def update(self, dt: float) -> None:
        self.play_time += dt
        self.process_editor_events()
        if dt > 0:
            instant = 1.0 / dt
            self.fps = instant if self.fps == 0 else self.fps * 0.92 + instant * 0.08

    def process_editor_events(self) -> None:
        while True:
            try:
                action, payload = self.editor_events.get_nowait()
            except queue.Empty:
                break
            if action == "compile":
                self.source = payload
                self.compile_now()

    def on_draw(self) -> None:
        self.clear()
        self.draw_viewport()

    def draw_viewport(self) -> None:
        gl.glViewport(0, 0, self.width, self.height)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
        gl.glClearColor(0.10, 0.12, 0.13, 1.0)
        gl.glClear(gl.GL_DEPTH_BUFFER_BIT)

        self.projection = Mat4.perspective_projection(max(1, self.width) / max(1, self.height), 0.1, 100.0, 50)
        yaw = radians(self.camera_yaw)
        pitch = radians(self.camera_pitch)
        eye = Vec3(self.zoom * cos(pitch) * cos(yaw), self.zoom * cos(pitch) * sin(yaw), self.zoom * sin(pitch))
        self.view = Mat4.look_at(eye, Vec3(0, 0, 1.0), Vec3(0, 0, 1))

        self.draw_grid()
        self.draw_axes()
        if self.result:
            if self.show_domain:
                self.draw_workspace_domain(self.result.robot_type)
            point = self.animated_point()
            if self.result.robot_type == "cilindrico":
                self.draw_cylindrical_robot(point)
            else:
                self.draw_cartesian_robot(point)
            if self.show_trace:
                self.draw_trace()

        gl.glDisable(gl.GL_DEPTH_TEST)
        gl.glViewport(0, 0, self.width, self.height)
        self.set_2d()
        self.draw_top_bar()

    def set_2d(self) -> None:
        self.projection = Mat4.orthogonal_projection(0, self.width, 0, self.height, -255, 255)
        self.view = Mat4()

    def animated_point(self) -> tuple[float, float, float]:
        if not self.result or not self.result.path:
            return (0, 0, 0)
        idx = int(self.play_time * 18) % len(self.result.path)
        return self.result.path[idx]

    def draw_top_bar(self) -> None:
        self.refresh_ui_hitboxes()
        draw_rect(0, self.height - 42, self.width, 42, (42, 45, 48))
        pyglet.text.Label("Robo:", x=12, y=self.height - 27, color=(240, 240, 240, 255), font_size=11).draw()
        self.draw_toolbar_button("cartesiano", "Cartesiano", 58, self.height - 34, self.active_tab == "cartesiano")
        self.draw_toolbar_button("cilindrico", "Cilindrico", 156, self.height - 34, self.active_tab == "cilindrico")
        self.draw_toolbar_checkbox("toggle_domain", "Dominio", 262, self.height - 32, self.show_domain)
        self.draw_toolbar_checkbox("toggle_trace", "Rastro", 365, self.height - 32, self.show_trace)
        self.draw_toolbar_checkbox("toggle_gpu", "GPU", 452, self.height - 32, self.gpu_render)
        pyglet.text.Label(f"{self.fps:04.1f} fps", x=524, y=self.height - 27, color=(210, 218, 224, 255), font_size=10).draw()
        status_color = (120, 235, 165, 255) if self.error.startswith("OK") else (255, 145, 135, 255)
        pyglet.text.Label(
            self.error,
            x=610,
            y=self.height - 27,
            color=status_color,
            font_size=10,
            width=max(120, self.width - 625),
            multiline=False,
        ).draw()

    def refresh_ui_hitboxes(self) -> None:
        y_button = self.height - 34
        y_check = self.height - 32
        self.ui_hitboxes = {
            "cartesiano": (58, y_button, 88, 24),
            "cilindrico": (156, y_button, 88, 24),
            "toggle_domain": (262, y_check - 3, 78, 22),
            "toggle_trace": (365, y_check - 3, 78, 22),
            "toggle_gpu": (452, y_check - 3, 78, 22),
        }

    def draw_toolbar_button(self, action: str, text: str, x: int, y: int, active: bool) -> None:
        w = 88
        color = (30, 128, 96) if active else (70, 74, 78)
        draw_rect(x, y, w, 24, color)
        pyglet.text.Label(text, x=x + 10, y=y + 7, color=(245, 248, 250, 255), font_size=9).draw()

    def draw_toolbar_checkbox(self, action: str, text: str, x: int, y: int, checked: bool) -> None:
        draw_rect(x, y, 16, 16, (235, 238, 240) if checked else (92, 96, 100))
        if checked:
            draw_rect(x + 4, y + 4, 8, 8, (24, 150, 102))
        pyglet.text.Label(text, x=x + 22, y=y + 3, color=(238, 242, 245, 255), font_size=9).draw()

    def on_text(self, text: str) -> None:
        return

    def on_key_press(self, symbol: int, modifiers: int) -> None:
        key = pyglet.window.key
        if symbol == key.F5:
            self.compile_now()
        elif symbol == key.F1:
            self.set_robot_type("cartesiano")
        elif symbol == key.F2:
            self.set_robot_type("cilindrico")
        elif symbol == key.F3:
            self.source = ANIMATION_TEST_EXAMPLE
            self.compile_now()
            self.editor_commands.put(("set_source", self.source))

    def set_robot_type(self, robot_type: str) -> None:
        self.source = CYLINDRICAL_EXAMPLE if robot_type == "cilindrico" else DEFAULT_SOURCE
        self.compile_now()
        self.editor_commands.put(("set_source", self.source))

    def on_mouse_press(self, x: int, y: int, button: int, modifiers: int) -> None:
        self.refresh_ui_hitboxes()
        for action, (bx, by, bw, bh) in self.ui_hitboxes.items():
            if bx <= x <= bx + bw and by <= y <= by + bh:
                if action in {"cartesiano", "cilindrico"}:
                    self.set_robot_type(action)
                elif action == "toggle_domain":
                    self.show_domain = not self.show_domain
                elif action == "toggle_trace":
                    self.show_trace = not self.show_trace
                elif action == "toggle_gpu":
                    self.gpu_render = not self.gpu_render
                return

    def on_mouse_drag(self, x: int, y: int, dx: int, dy: int, buttons: int, modifiers: int) -> None:
        if buttons & pyglet.window.mouse.LEFT:
            self.camera_yaw += dx * 0.4
            self.camera_pitch = max(5, min(80, self.camera_pitch + dy * 0.3))

    def on_mouse_scroll(self, x: int, y: int, scroll_x: float, scroll_y: float) -> None:
        self.zoom = max(4.0, min(18.0, self.zoom - scroll_y * 0.5))

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
            draw_box(center, size, (0.20, 0.75, 1.0, 0.12))
            draw_box_edges(center, size, (0.25, 0.88, 1.0, 0.62))
        elif robot_type == "cilindrico":
            theta_min, theta_max = cfg["theta_deg"]
            radius_min, radius_max = cfg["radius"]
            zmin, zmax = cfg["z"]
            key = f"cilindrico:{radius_min}:{radius_max}:{zmin}:{zmax}:{theta_min}:{theta_max}:{self.gpu_render}"
            if key not in self.workspace_mesh_cache:
                steps = 72 if self.gpu_render else 24
                self.workspace_mesh_cache[key] = build_cylindrical_workspace_mesh(radius_min, radius_max, zmin, zmax, theta_min, theta_max, steps)
            verts, edge_verts = self.workspace_mesh_cache[key]
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
        quality = (12, 8) if self.gpu_render else (8, 5)
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
        draw_cylinder((0, 0, 0.08), 0.58, 0.16, (0.72, 0.73, 0.74), 24 if self.gpu_render else 14)
        draw_cylinder((0, 0, (zmax + 0.35) / 2), 0.22, zmax + 0.35, (0.60, 0.63, 0.65), 24 if self.gpu_render else 14)
        draw_cylinder((0, 0, z), 0.34, 0.28, (1.0, 0.25, 0.10), 24 if self.gpu_render else 14)
        draw_box(arm_center, (arm_length, 0.16, 0.16), (0.10, 0.60, 1.0), angle_deg=degrees2d(x, y))
        draw_box((x, y, z - 0.22), (0.14, 0.14, 0.44), (0.08, 0.09, 0.10))
        quality = (12, 8) if self.gpu_render else (8, 5)
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


if __name__ == "__main__":
    RobotApp()
    pyglet.app.run()
