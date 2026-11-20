"""Unit tests for OpenCap / OpenSim .mot gait parsing and O&P alignment/load recommendations."""

from __future__ import annotations

import unittest
from pathlib import Path

from op_core.cli import main
from op_core.gait_bridge import analyze_gait_mot, parse_opensim_mot

SAMPLE_MOT = Path(__file__).resolve().parents[1] / "examples" / "sample_gait.mot"


class TestGaitBridge(unittest.TestCase):
    def test_parse_opensim_mot_and_clinical_recommendations(self) -> None:
        tbl = parse_opensim_mot(SAMPLE_MOT)
        self.assertEqual(len(tbl.rows), 9)
        self.assertIn("ankle_angle_l", tbl.columns)

        summary = analyze_gait_mot(SAMPLE_MOT, affected_side="left", body_mass_kg=80.0)
        self.assertAlmostEqual(summary.duration_s, 1.20, places=2)
        self.assertAlmostEqual(summary.min_ankle_angle_affected_deg, -14.0, places=1)
        self.assertGreater(summary.knee_flexion_asymmetry_pct, 30.0)
        self.assertGreater(summary.peak_vertical_grf_bw, 1.2)
        self.assertGreaterEqual(len(summary.recommendations), 2)

    def test_cli_gait_subcommand(self) -> None:
        rc = main(["gait", str(SAMPLE_MOT), "--side", "left", "--body-mass-kg", "85.0"])
        self.assertEqual(rc, 0)


if __name__ == "__main__":
    unittest.main()
