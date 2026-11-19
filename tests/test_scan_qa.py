"""Unit tests for Project 1 / EXP-1: Scan QA, artifact detection, MPT alignment, and taut-tape slicing."""

from __future__ import annotations

import unittest

from op_core.scan_qa import measure_circumferences, run_scan_qa
from op_core.synthetic import corrupt_scan, generate_transtibial_limb


class TestScanQA(unittest.TestCase):
    def test_clean_open_cuff_passes_qa(self) -> None:
        case = generate_transtibial_limb(open_cuff=True, seed=1)
        aligned, report = run_scan_qa(
            case.mesh,
            mpt_point=case.landmark_coords["mpt"],
            distal_point=case.landmark_coords["distal_end"],
        )
        self.assertEqual(report.status, "PASS")
        self.assertEqual(report.proximal_cuff_loops, 1)
        self.assertEqual(report.internal_tear_loops, 0)
        self.assertGreaterEqual(len(report.slices), 7)
        self.assertTrue(aligned.slices if hasattr(aligned, "slices") else len(report.slices) > 0)

    def test_corrupted_scan_detects_floaters_tears_double_wall_and_scale(self) -> None:
        case = generate_transtibial_limb(open_cuff=True, seed=2)
        dirty = corrupt_scan(
            case.mesh,
            tilt_deg=15.0,
            scale_factor=1.03,
            jitter_mm=0.5,
            seam_step_mm=3.0,
            double_wall=True,
            floaters=3,
            tear_hole=True,
        )
        _, report = run_scan_qa(
            dirty,
            calibration_measured_mm=103.0,
            calibration_expected_mm=100.0,
        )
        self.assertEqual(report.status, "FAIL")
        self.assertEqual(report.floater_components_removed, 3)
        self.assertGreaterEqual(report.internal_tear_loops, 1)
        self.assertGreater(report.double_wall_vertices, 15)
        self.assertGreater(report.seam_step_score_mm, 1.0)
        self.assertAlmostEqual(report.detected_scale_ratio or 0.0, 1.03, places=3)

    def test_tilted_and_noisy_circumference_accuracy(self) -> None:
        case = generate_transtibial_limb(open_cuff=True, seed=3)
        _, gt_rep = run_scan_qa(
            case.mesh,
            mpt_point=case.landmark_coords["mpt"],
            distal_point=case.landmark_coords["distal_end"],
        )
        gt_slices = gt_rep.slices

        tilted = corrupt_scan(case.mesh, tilt_deg=15.0, jitter_mm=0.3, seed=9)
        from op_core.math3d import rodrigues_rotate
        import math

        rad = math.radians(15.0)
        mpt_rot = rodrigues_rotate(case.landmark_coords["mpt"], (1.0, 0.25, 0.0), rad)
        dist_rot = rodrigues_rotate(case.landmark_coords["distal_end"], (1.0, 0.25, 0.0), rad)

        _, report = run_scan_qa(tilted, mpt_point=mpt_rot, distal_point=dist_rot)
        by_depth = {round(s["depth_from_anchor_mm"]): s["taut_circumference_mm"] for s in report.slices}
        errors = [
            abs(by_depth[round(gt["depth_from_anchor_mm"])] - gt["taut_circumference_mm"])
            for gt in gt_slices
            if round(gt["depth_from_anchor_mm"]) in by_depth
        ]
        self.assertTrue(errors)
        self.assertLessEqual(sum(errors) / len(errors), 1.5)


if __name__ == "__main__":
    unittest.main()
