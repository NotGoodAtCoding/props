"""Unit tests for 3D ICP scan registration, signed deviation heatmaps, and limb volume tracking."""

from __future__ import annotations

import math
import unittest

from op_core.math3d import rodrigues_rotate
from op_core.metrology import compare_scans, horn_rigid_transform, icp_align_scans
from op_core.synthetic import generate_transtibial_limb


class TestMetrology(unittest.TestCase):
    def test_horn_rigid_transform_recovers_known_pose(self) -> None:
        pts = [(10.0, 0.0, 0.0), (0.0, 25.0, 5.0), (-12.0, 4.0, 30.0), (5.0, -18.0, -40.0)]
        moved = [
            (
                p[0] + 8.0,
                p[1] - 5.0,
                p[2] + 12.0,
            )
            for p in (rodrigues_rotate(v, (0.3, 0.8, 0.5), math.radians(15.0)) for v in pts)
        ]
        rot, t = horn_rigid_transform(moved, pts)
        from op_core.math3d import mat3_mul_vec, v_add, v_dist

        recovered = [v_add(mat3_mul_vec(rot, m), t) for m in moved]
        errs = [v_dist(r, p) for r, p in zip(recovered, pts)]
        self.assertLess(max(errs), 1e-6)

    def test_icp_align_scans_recovers_tilted_shifted_limb(self) -> None:
        case = generate_transtibial_limb(open_cuff=True, seed=0)
        moved = case.mesh.transformed(
            lambda p: (
                q[0] + 6.0,
                q[1] - 4.0,
                q[2] + 10.0,
            )
            if (q := rodrigues_rotate(p, (1.0, 0.2, 0.0), math.radians(10.0)))
            else p
        )
        aligned, iters = icp_align_scans(moved, case.mesh, max_iters=25)
        self.assertGreaterEqual(iters, 1)
        cmp_res = compare_scans(case.mesh, aligned, align_icp=False)
        self.assertLess(cmp_res.rmse_mm, 0.45)

    def test_compare_scans_detects_limb_volume_atrophy(self) -> None:
        base = generate_transtibial_limb(open_cuff=True, seed=1).mesh
        # Simulate 4% circumferential atrophy (radial shrink in XY -> ~8% volume drop)
        shrunk = base.transformed(lambda p: (p[0] * 0.96, p[1] * 0.96, p[2]))
        res = compare_scans(base, shrunk, align_icp=True)
        self.assertLess(res.mean_signed_dev_mm, -1.0)
        self.assertLess(res.volume_change_pct, -5.0)
        self.assertEqual(len(res.per_vertex_dev_mm), len(base.vertices))


if __name__ == "__main__":
    unittest.main()
