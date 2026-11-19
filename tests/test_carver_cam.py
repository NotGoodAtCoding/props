"""Unit tests for 4-axis rotary foam-carver G-code generation and ball-nose stock simulation (EXP-5B)."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from op_core.carver_cam import generate_4axis_gcode_and_simulate
from op_core.cli import main
from op_core.mesh import write_binary_stl
from op_core.shell import export_carver_positive
from op_core.synthetic import generate_transtibial_limb


class TestCarverCam(unittest.TestCase):
    def test_4axis_gcode_matches_positive_within_rms_and_zero_gouge(self) -> None:
        case = generate_transtibial_limb(open_cuff=True, seed=0)
        carver = export_carver_positive(case.mesh, draft_angle_deg=12.0)

        with tempfile.TemporaryDirectory() as tmp:
            gcode_p = Path(tmp) / "carver.gcode"
            res = generate_4axis_gcode_and_simulate(
                carver.positive_mesh,
                ball_radius_mm=4.0,
                output_path=gcode_p,
            )
            self.assertTrue(gcode_p.exists())
            txt = gcode_p.read_text(encoding="utf-8")
            self.assertIn("G21", txt)
            self.assertIn("M30", txt)
            self.assertEqual(res.max_gouge_mm, 0.0)
            self.assertEqual(res.mandrel_collisions, 0)
            self.assertLessEqual(res.rms_error_mm, 0.5)

    def test_cli_cam_subcommand(self) -> None:
        case = generate_transtibial_limb(open_cuff=True, seed=1)
        carver = export_carver_positive(case.mesh)
        with tempfile.TemporaryDirectory() as tmp:
            stl_p = Path(tmp) / "carver.stl"
            nc_p = Path(tmp) / "carver.gcode"
            write_binary_stl(carver.positive_mesh, stl_p)
            rc = main(["cam", str(stl_p), "--out-gcode", str(nc_p)])
            self.assertEqual(rc, 0)
            self.assertTrue(nc_p.exists())


if __name__ == "__main__":
    unittest.main()
