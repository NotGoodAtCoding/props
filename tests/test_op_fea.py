"""Unit tests for isolated op_fea CalculiX subprocess wrapper and .inp/.dat handling."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from op_core.shell import build_tt_socket_shell
from op_core.synthetic import generate_transtibial_limb
from op_fea import generate_iso10328_inp, parse_calculix_dat, run_calculix_precheck


class TestOpFea(unittest.TestCase):
    def test_generate_iso10328_inp_and_subprocess_fallback(self) -> None:
        case = generate_transtibial_limb(open_cuff=True, seed=0)
        sh = build_tt_socket_shell(case.mesh)

        with tempfile.TemporaryDirectory() as tmp:
            inp_p = generate_iso10328_inp(sh.shell_mesh, Path(tmp) / "check.inp")
            self.assertTrue(inp_p.exists())
            txt = inp_p.read_text(encoding="utf-8")
            self.assertIn("*NODE", txt)
            self.assertIn("*ELEMENT, TYPE=S3", txt)
            self.assertIn("*BOUNDARY", txt)
            self.assertIn("*CLOAD", txt)

            res = run_calculix_precheck(sh.shell_mesh, Path(tmp) / "run")
            self.assertTrue(Path(res.inp_path).exists())
            # When ccx binary is absent, executed is False with graceful diagnostic
            if not res.executed:
                self.assertIn("not installed", res.notes[0])

    def test_parse_calculix_dat_stress_extraction(self) -> None:
        sample_dat = """
 stresses (elem, integ.pnt,sxx,syy,szz,sxy,sxz,syz) for set EALL and time  0.1000000E+01
         1   1   5.000000E+00  4.000000E+00  1.450000E+01  1.000000E+00  5.000000E-01  2.000000E-01
         2   1   2.000000E+00  1.000000E+00 -8.000000E+00  0.000000E+00  0.000000E+00  0.000000E+00
        """
        vm, sz = parse_calculix_dat(sample_dat)
        self.assertAlmostEqual(sz, 14.5, places=3)
        self.assertGreater(vm, 9.0)


if __name__ == "__main__":
    unittest.main()
