"""Unit tests for Project 3 & 4 / EXP-4: Dual-schema .3mf export, safety linter, and fabrication dossier."""

from __future__ import annotations

import json
import tempfile
import unittest
import zipfile
from pathlib import Path

from op_core.export_3mf import export_3mf
from op_core.linter import export_fabrication_dossier, lint_device
from op_core.recipe import OpRecipe
from op_core.scan_qa import run_scan_qa
from op_core.shell import build_tt_socket_shell
from op_core.synthetic import generate_transtibial_limb


class TestExportAndLint(unittest.TestCase):
    def test_dual_schema_3mf_targets(self) -> None:
        case = generate_transtibial_limb(open_cuff=True, seed=12)
        res = build_tt_socket_shell(case.mesh)
        with tempfile.TemporaryDirectory() as tmp:
            p_prusa = Path(tmp) / "prusa.3mf"
            p_orca = Path(tmp) / "orca.3mf"
            export_3mf(res.shell_mesh, res.modifier_mesh, p_prusa, target="prusaslicer")
            export_3mf(res.shell_mesh, res.modifier_mesh, p_orca, target="orcaslicer")

            with zipfile.ZipFile(p_prusa) as zf:
                names = set(zf.namelist())
                self.assertIn("3D/3dmodel.model", names)
                self.assertIn("Metadata/Slic3r_PE_model.config", names)
                self.assertIn('key="modifier" value="1"', zf.read("Metadata/Slic3r_PE_model.config").decode())

            with zipfile.ZipFile(p_orca) as zf:
                names = set(zf.namelist())
                self.assertIn("3D/3dmodel.model", names)
                self.assertIn("Metadata/model_settings.config", names)
                self.assertIn('subtype="modifier_part"', zf.read("Metadata/model_settings.config").decode())

    def test_linter_and_dossier_flags_unsafe_and_passes_valid(self) -> None:
        case = generate_transtibial_limb(open_cuff=True, seed=14)
        clean, qa = run_scan_qa(case.mesh, mpt_point=case.landmark_coords["mpt"], distal_point=case.landmark_coords["distal_end"])
        res = build_tt_socket_shell(clean)

        ok_lint = lint_device(res.shell_mesh, measured_wall_override_mm=2.5)
        self.assertTrue(ok_lint.passed)

        bad_lint = lint_device(
            res.shell_mesh,
            measured_wall_override_mm=1.8,
            trimline_points=[(0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (2.0, 2.0, 0.0)],
            load_axis=(0.0, 0.0, 1.0),
            print_z_axis=(0.0, 0.707, 0.707),
        )
        self.assertFalse(bad_lint.passed)
        self.assertGreaterEqual(len(bad_lint.failures), 2)
        self.assertGreaterEqual(len(bad_lint.warnings), 1)

        with tempfile.TemporaryDirectory() as tmp:
            recipe = OpRecipe(device_type="transtibial_socket", side="left")
            j_path, m_path = export_fabrication_dossier(recipe, qa, ok_lint, tmp)
            p_path = Path(tmp) / "device_dossier.pdf"
            self.assertTrue(j_path.exists() and m_path.exists() and p_path.exists())
            pdf_bytes = p_path.read_bytes()
            self.assertTrue(pdf_bytes.startswith(b"%PDF-1.4"))
            self.assertIn(b"%%EOF", pdf_bytes)
            parsed = json.loads(j_path.read_text())
            self.assertEqual(parsed["scan_qa"]["status"], "PASS")


if __name__ == "__main__":
    unittest.main()
