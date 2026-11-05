"""Gera docs/demo.gif (e capturas) renderizando a demonstracao quadro a quadro.

uso: python tools/make_demo_gif.py [programa.robot] [--gpu|--cpu] [--domain] [--out docs]

Padrao: robo cartesiano (programa de exemplo) e sem o dominio transparente.
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import pyglet  # noqa: E402
from PIL import Image  # noqa: E402

import main as app_module  # noqa: E402
from render_profile import choose_profile, probe_renderer, requested_mode  # noqa: E402


def grab(app: app_module.RobotApp) -> Image.Image:
    app.switch_to()
    app.dispatch_events()
    app.dispatch_event("on_draw")
    buf = pyglet.image.get_buffer_manager().get_color_buffer().get_image_data()
    img = Image.frombytes("RGBA", (buf.width, buf.height), buf.get_data("RGBA", buf.width * 4))
    return img.transpose(Image.FLIP_TOP_BOTTOM).convert("RGB")


def main() -> None:
    argv = sys.argv[1:]
    out = Path(argv[argv.index("--out") + 1]) if "--out" in argv else ROOT / "docs"
    files = [a for a in argv if a.endswith(".robot")]
    out.mkdir(parents=True, exist_ok=True)
    mode = requested_mode(argv)
    renderer = probe_renderer()
    app = app_module.RobotApp(mode, choose_profile(mode, renderer), renderer)
    app.show_domain = "--domain" in argv
    if files:
        app.set_source(Path(files[0]).read_text(encoding="utf-8"))
    for _ in range(5):  # deixa a janela abrir/dimensionar
        pyglet.clock.tick()
        app.dispatch_events()
    tl = app.timeline
    tl.auto = False
    frames: list[Image.Image] = []
    durations: list[int] = []
    for i, frame in enumerate(tl.frames):
        tl.index = i
        if frame.phase == "simulacao":
            for n in range(24):
                app.sim_time = n * 0.14
                frames.append(grab(app))
                durations.append(110)
            continue
        samples = 6 if frame.dwell > 3 else 2
        for n in range(samples):
            tl.t = (n + 1) / samples * frame.dwell
            frames.append(grab(app))
            durations.append(260 if frame.dwell > 3 else 420)
    width = 820
    scaled = [f.resize((width, int(f.height * width / f.width)), Image.LANCZOS) for f in frames]
    pal = [f.quantize(colors=48, method=Image.MEDIANCUT, dither=Image.NONE) for f in scaled]
    pal[0].save(out / "demo.gif", save_all=True, append_images=pal[1:], duration=durations, loop=0, optimize=True)
    print(f"{len(frames)} quadros -> {out / 'demo.gif'}")
    # capturas estaticas em alta resolucao
    picks = {"conversao": max(0, len(tl.frames) // 2), "simulacao": len(tl.frames) - 1}
    for name, idx in picks.items():
        tl.index = idx
        tl.t = 0.9 * min(tl.frame.dwell, 6.0) if tl.frame.dwell < 100 else 0.0
        app.sim_time = 3.5
        grab(app).save(out / f"{name}.png")
    print("capturas:", ", ".join(f"{n}.png" for n in picks))


if __name__ == "__main__":
    main()
