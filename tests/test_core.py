import json
import unittest
from pathlib import Path

from demo import STAGE_START, Timeline, active_stage, stage_progress
from kinematics import forward_kinematics, inverse_kinematics
from pipeline import analyze, format_report, tokenize
from render_profile import (
    CPU_PROFILE,
    GPU_PROFILE,
    FpsGovernor,
    choose_profile,
    is_software_renderer,
)
from robot_language import ANIMATION_TEST_EXAMPLE, CYLINDRICAL_EXAMPLE, DEFAULT_SOURCE

LIMITS = json.loads((Path(__file__).resolve().parent.parent / ".robot_limits.json").read_text())


class KinematicsTests(unittest.TestCase):
    def test_roundtrip_cylindrical(self):
        for p in [(2.0, 1.0, 1.0), (-1.5, 2.0, 0.5), (1.0, -1.2, 3.0)]:
            joints = inverse_kinematics("cilindrico", p, LIMITS).values
            back = forward_kinematics("cilindrico", joints)
            self.assertTrue(all(abs(a - b) < 1e-9 for a, b in zip(p, back)))

    def test_cartesian_is_identity(self):
        self.assertEqual(inverse_kinematics("cartesiano", (1, 2, 3), LIMITS).values, (1, 2, 3))


class PipelineTests(unittest.TestCase):
    def test_examples_compile(self):
        for src in (DEFAULT_SOURCE, CYLINDRICAL_EXAMPLE, ANIMATION_TEST_EXAMPLE):
            report = analyze(src, LIMITS)
            self.assertTrue(report.ok, report.error)
            self.assertEqual(len(report.steps), len([s for s in src.split(";") if s.strip()]))
            for step in report.steps:
                if step.target:
                    self.assertLess(step.fk_error, 1e-9)

    def test_error_points_at_statement(self):
        report = analyze("ROBO cartesiano;\nMoveL(1,1,1);\nMoveL(9,0,1);", LIMITS)
        self.assertFalse(report.ok)
        self.assertEqual(report.error_step, 2)
        self.assertEqual(report.error[0], "Semantico")

    def test_lexical_error(self):
        report = analyze("ROBO cartesiano; MoveL(1,1,1) @;", LIMITS)
        self.assertEqual(report.error[0], "Lexico")

    def test_tokenizer_signed_numbers(self):
        toks = tokenize("MoveJ(-1.2, 0.8, 2, v=1.1)")
        self.assertIn(("numero", "-1.2"), toks)
        self.assertIn(("parametro", "v"), toks)
        self.assertEqual(toks[0], ("comando", "MoveJ"))

    def test_terminal_report(self):
        text = format_report(analyze(CYLINDRICAL_EXAMPLE, LIMITS))
        self.assertIn("atan2", text)
        self.assertIn("OK:", text)


class TimelineTests(unittest.TestCase):
    def test_advances_and_ends_in_simulation(self):
        tl = Timeline(analyze(DEFAULT_SOURCE, LIMITS))
        for _ in range(10000):
            tl.update(0.05)
        self.assertTrue(tl.in_simulation)

    def test_error_timeline_never_simulates(self):
        tl = Timeline(analyze("ROBO cartesiano;MoveL(9,0,1);", LIMITS))
        for _ in range(10000):
            tl.update(0.05)
        self.assertFalse(tl.in_simulation)
        self.assertEqual(tl.frame.phase, "erro")

    def test_stage_helpers(self):
        self.assertEqual(active_stage(0.0), 0)
        self.assertEqual(active_stage(1.0), len(STAGE_START) - 1)
        self.assertEqual(stage_progress(0.0, 3), 0.0)
        self.assertEqual(stage_progress(1.0, 3), 1.0)


class RenderProfileTests(unittest.TestCase):
    def test_software_detection(self):
        self.assertTrue(is_software_renderer("llvmpipe (LLVM 15.0.7, 256 bits)"))
        self.assertTrue(is_software_renderer("Microsoft Basic Render Driver"))
        self.assertFalse(is_software_renderer("NVIDIA GeForce RTX 3060/PCIe/SSE2"))
        self.assertFalse(is_software_renderer("Mesa Intel(R) UHD Graphics 630"))

    def test_auto_choice(self):
        self.assertIs(choose_profile("auto", "llvmpipe"), CPU_PROFILE)
        self.assertIs(choose_profile("auto", "NVIDIA GeForce"), GPU_PROFILE)
        self.assertIs(choose_profile("cpu", "NVIDIA GeForce"), CPU_PROFILE)
        self.assertIs(choose_profile("gpu", "llvmpipe"), GPU_PROFILE)

    def test_governor_downgrades_only_when_slow(self):
        slow = FpsGovernor()
        self.assertTrue(any(slow.update(0.1) for _ in range(200)))  # 10 fps
        fast = FpsGovernor()
        self.assertFalse(any(fast.update(1 / 60) for _ in range(1000)))


if __name__ == "__main__":
    unittest.main()
