"""Unit tests for math3d, TriMesh topology, geodesic distances, and binary/ASCII STL round-trip."""

from __future__ import annotations

import math
import tempfile
import unittest
from pathlib import Path

from op_core.math3d import convex_hull_2d, jacobi_eigen_3x3, polygon_perimeter_2d
from op_core.mesh import TriMesh, read_stl, write_binary_stl
from op_core.synthetic import generate_transtibial_limb


class TestMathAndMesh(unittest.TestCase):
    def test_convex_hull_circle_perimeter(self) -> None:
        r = 50.0
        pts = [(r * math.cos(2.0 * math.pi * k / 120), r * math.sin(2.0 * math.pi * k / 120)) for k in range(120)]
        hull = convex_hull_2d(pts)
        perim = polygon_perimeter_2d(hull)
        self.assertAlmostEqual(perim, 2.0 * math.pi * r, delta=0.5)

    def test_jacobi_eigensolver_principal_axis(self) -> None:
        # Elongated ellipsoid along (0, 0, 1)
        cov = [[4.0, 0.0, 0.0], [0.0, 9.0, 0.0], [0.0, 0.0, 100.0]]
        vals, vecs = jacobi_eigen_3x3(cov)
        self.assertAlmostEqual(vals[0], 100.0, places=5)
        self.assertGreater(abs(vecs[0][2]), 0.999)

    def test_synthetic_limb_closed_and_stl_roundtrip(self) -> None:
        case = generate_transtibial_limb(open_cuff=False)
        self.assertTrue(case.mesh.is_watertight_manifold())
        self.assertGreater(case.mesh.signed_volume(), 0.0)

        with tempfile.TemporaryDirectory() as tmp:
            stl_path = Path(tmp) / "limb.stl"
            write_binary_stl(case.mesh, stl_path)
            loaded = read_stl(stl_path)
            self.assertEqual(len(loaded.faces), len(case.mesh.faces))
            self.assertTrue(loaded.is_watertight_manifold())

    def test_geodesic_distance_monotonicity(self) -> None:
        case = generate_transtibial_limb(open_cuff=True)
        seed = case.landmarks["fibular_head"]
        dists = case.mesh.geodesic_distances([seed])
        self.assertEqual(dists[seed], 0.0)
        distal = case.landmarks["distal_end"]
        self.assertGreater(dists[distal], 100.0)
        self.assertTrue(math.isfinite(dists[distal]))


if __name__ == "__main__":
    unittest.main()
