"""Tests for op-core CLI subcommands (demo, qa, build, lint, diff) and example recipes."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from op_core.cli import main
from op_core.recipe import OpRecipe

REPO_ROOT = Path(__file__).resolve().parents[1]


class TestCLIAndExamples(unittest.TestCase):
    def test_example_recipes_validate_and_diff(self) -> None:
        r1 = OpRecipe.load(REPO_ROOT / "examples" / "tt_socket.oprecipe.json")
        r2 = OpRecipe.load(REPO_ROOT / "examples" / "tt_socket_v2.oprecipe.json")
        afo = OpRecipe.load(REPO_ROOT / "examples" / "afo_leaf.oprecipe.json")
        self.assertEqual(r1.device_type, "transtibial_socket")
        self.assertEqual(afo.device_type, "afo")
        self.assertEqual(
            main(["diff", str(REPO_ROOT / "examples" / "tt_socket.oprecipe.json"), str(REPO_ROOT / "examples" / "tt_socket_v2.oprecipe.json")]),
            0,
        )
        self.assertNotEqual(r1.operations, r2.operations)

    def test_demo_tt_and_afo_and_lint_cli(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            tt_dir = Path(tmp) / "demo_tt"
            afo_dir = Path(tmp) / "demo_afo"
            self.assertEqual(main(["demo", "--out-dir", str(tt_dir), "--device", "tt", "--target", "orcaslicer"]), 0)
            self.assertEqual(main(["demo", "--out-dir", str(afo_dir), "--device", "afo", "--target", "prusaslicer"]), 0)

            self.assertTrue((tt_dir / "device_shell.stl").exists())
            self.assertTrue((tt_dir / "device_orcaslicer.3mf").exists())
            self.assertTrue((tt_dir / "carver_positive.stl").exists())
            self.assertTrue((tt_dir / "device_dossier.md").exists())
            self.assertTrue((afo_dir / "device_prusaslicer.3mf").exists())

            self.assertEqual(main(["qa", str(tt_dir / "sample_scan.stl"), "--out-stl", str(tt_dir / "aligned.stl")]), 0)
            self.assertEqual(main(["lint", str(tt_dir / "device_shell.stl"), "--min-wall-mm", "2.0"]), 0)


if __name__ == "__main__":
    unittest.main()
