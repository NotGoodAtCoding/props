"""Tests for accelerated backend probe (`op-core doctor`) and standalone Blender bridge adapter."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from adapters.blender_op_bridge import build_cli_command, run_op_core_subprocess, workspaces_to_hide
from op_core.accel import probe_backends
from op_core.cli import main
from op_core.mesh import write_binary_stl
from op_core.synthetic import generate_transtibial_limb

REPO_ROOT = Path(__file__).resolve().parents[1]


class TestAdaptersAndAccel(unittest.TestCase):
    def test_probe_backends_and_doctor_cli(self) -> None:
        st = probe_backends()
        self.assertTrue(st.core_ready)
        self.assertIn("manifold3d", st.python_packages)
        self.assertIn("ccx", st.external_binaries)
        self.assertEqual(main(["doctor"]), 0)

    def test_blender_bridge_workspace_filter_and_subprocess_build(self) -> None:
        hidden = workspaces_to_hide(["Layout", "Animation", "UV Editing", "Sculpting", "Compositing"])
        self.assertEqual(hidden, ["Animation", "UV Editing", "Compositing"])

        case = generate_transtibial_limb(open_cuff=True, seed=0)
        with tempfile.TemporaryDirectory() as tmp:
            scan_p = Path(tmp) / "scan.stl"
            out_d = Path(tmp) / "out"
            write_binary_stl(case.mesh, scan_p)
            cmd = build_cli_command(
                "build",
                scan_p,
                recipe_path=REPO_ROOT / "examples" / "tt_socket.oprecipe.json",
                out_dir=out_d,
                target="orcaslicer",
            )
            res = run_op_core_subprocess(cmd, repo_src=REPO_ROOT / "src")
            self.assertTrue(res["lint_passed"])
            self.assertTrue((out_d / "device_orcaslicer.3mf").exists())


if __name__ == "__main__":
    unittest.main()
