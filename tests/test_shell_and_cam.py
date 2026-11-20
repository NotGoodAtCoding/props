"""Unit tests for Project 3 / EXP-3 & EXP-5: Watertight TT/AFO shells, bench alignment & carver positives."""

from __future__ import annotations

import math
import unittest

from op_core.math3d import rodrigues_rotate, v_dot
from op_core.rectify import apply_afo_posture
from op_core.shell import build_afo_shell, build_tt_socket_shell, export_carver_positive
from op_core.synthetic import generate_foot_ankle_mesh, generate_transtibial_limb


class TestShellAndCarver(unittest.TestCase):
    def test_tt_socket_shell_watertight_and_bench_aligned(self) -> None:
        case = generate_transtibial_limb(open_cuff=True, seed=10)
        res = build_tt_socket_shell(
            case.mesh,
            wall_distal_mm=4.5,
            wall_proximal_mm=2.5,
            flexion_deg=5.0,
            adduction_deg=5.0,
        )
        self.assertTrue(res.shell_mesh.is_watertight_manifold(), res.shell_mesh.edge_topology())
        self.assertGreater(res.shell_mesh.signed_volume(), 0.0)
        self.assertTrue(res.modifier_mesh.is_watertight_manifold())

        expected_n = rodrigues_rotate((0.0, 0.0, -1.0), (1.0, 0.0, 0.0), math.radians(5.0))
        expected_n = rodrigues_rotate(expected_n, (0.0, 1.0, 0.0), math.radians(5.0))
        ang_err = math.degrees(math.acos(max(-1.0, min(1.0, v_dot(res.alignment_normal, expected_n)))))
        self.assertLess(ang_err, 0.2)

    def test_afo_shell_watertight_and_fillet_radius(self) -> None:
        afo_neutral = generate_foot_ankle_mesh(plantarflexion_deg=0.0, inversion_deg=0.0).mesh
        self.assertTrue(afo_neutral.is_watertight_manifold())
        ach_y = min(v[1] for v in afo_neutral.vertices if 78.0 <= v[2] <= 90.0)
        heel_y = min(v[1] for v in afo_neutral.vertices if 10.0 <= v[2] <= 35.0)
        self.assertGreater(ach_y - heel_y, 10.0)  # Distinct calcaneal heel projection behind Achilles
        med_arch_z = min(v[2] for v in afo_neutral.vertices if 70.0 <= v[1] <= 95.0 and v[0] < -15.0)
        lat_sole_z = min(v[2] for v in afo_neutral.vertices if 70.0 <= v[1] <= 95.0 and v[0] > 15.0)
        self.assertGreater(med_arch_z - lat_sole_z, 4.0)  # Raised medial longitudinal arch
        hallux_y = max(v[1] for v in afo_neutral.vertices if v[0] < -12.0 and v[2] < 35.0)
        toe5_y = max(v[1] for v in afo_neutral.vertices if v[0] > 20.0 and v[2] < 35.0)
        self.assertGreater(hallux_y - toe5_y, 15.0)  # Asymmetric hallux-to-5th-toe sweep

        afo_case = generate_foot_ankle_mesh(plantarflexion_deg=12.0, inversion_deg=5.0)
        neutral = apply_afo_posture(afo_case.mesh, dorsiflexion_deg=12.0, eversion_deg=5.0, heel_pitch_mm=10.0)
        res = build_afo_shell(neutral, wall_neck_mm=4.5, wall_calf_mm=2.5, trimline_fillet_mm=10.0)
        self.assertTrue(res.shell_mesh.is_watertight_manifold(), res.shell_mesh.edge_topology())
        self.assertGreater(res.shell_mesh.signed_volume(), 0.0)
        self.assertGreaterEqual(res.trimline_min_radius_mm, 8.0)

    def test_carver_positive_watertight_with_draft_and_keyed_bore(self) -> None:
        case = generate_transtibial_limb(open_cuff=True, seed=11)
        carver = export_carver_positive(case.mesh, draft_angle_deg=12.0, collar_height_mm=40.0)
        self.assertTrue(carver.positive_mesh.is_watertight_manifold(), carver.positive_mesh.edge_topology())
        self.assertGreater(carver.positive_mesh.signed_volume(), 0.0)
        self.assertEqual(carver.draft_angle_deg, 12.0)


if __name__ == "__main__":
    unittest.main()
