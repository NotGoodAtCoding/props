"""Unit tests for real-world mesh repair: welding, winding correction, hole filling, and Z-clipping."""

from __future__ import annotations

import random
import unittest

from op_core.mesh import TriMesh
from op_core.repair import (
    clip_mesh_z,
    fill_small_holes,
    fix_consistent_winding,
    repair_scan_mesh,
    weld_and_clean,
)
from op_core.synthetic import corrupt_scan, generate_transtibial_limb


class TestRepair(unittest.TestCase):
    def test_weld_removes_duplicate_vertices_and_degenerate_faces(self) -> None:
        case = generate_transtibial_limb(open_cuff=False, seed=0)
        # Split all vertices into triangle soup (3x vertices) + add 2 degenerate faces
        soup_verts = []
        soup_faces = []
        for a, b, c in case.mesh.faces:
            base = len(soup_verts)
            soup_verts.extend([case.mesh.vertices[a], case.mesh.vertices[b], case.mesh.vertices[c]])
            soup_faces.append((base, base + 1, base + 2))
        soup_faces.append((0, 0, 1))
        soup_faces.append((0, 1, 2))  # duplicate face

        soup = TriMesh(vertices=soup_verts, faces=soup_faces)
        cleaned, welded_rm, degen_rm = weld_and_clean(soup)
        self.assertGreater(welded_rm, 5000)
        self.assertEqual(degen_rm, 2)
        self.assertTrue(cleaned.is_watertight_manifold())

    def test_fix_consistent_winding_repairs_randomly_flipped_triangles(self) -> None:
        case = generate_transtibial_limb(open_cuff=False, seed=1)
        rng = random.Random(7)
        scrambled_faces = [
            (a, c, b) if rng.random() < 0.4 else (a, b, c)
            for a, b, c in case.mesh.faces
        ]
        broken = TriMesh(vertices=list(case.mesh.vertices), faces=scrambled_faces)
        self.assertFalse(broken.is_watertight_manifold())

        fixed, flipped = fix_consistent_winding(broken)
        self.assertGreater(flipped, 100)
        self.assertTrue(fixed.is_watertight_manifold(), fixed.edge_topology())
        self.assertGreater(fixed.signed_volume(), 0.0)

    def test_fill_small_holes_preserves_open_cuff_and_closes_internal_tear(self) -> None:
        case = generate_transtibial_limb(open_cuff=True, seed=2)
        torn = corrupt_scan(case.mesh, tear_hole=True)
        self.assertEqual(len(torn.boundary_loops()), 2)

        healed, filled = fill_small_holes(torn, keep_proximal_cuff=True)
        self.assertEqual(filled, 1)
        self.assertEqual(len(healed.boundary_loops()), 1)
        self.assertEqual(healed.edge_topology()["winding_mismatches"], 0)

    def test_clip_mesh_z_turns_closed_limb_into_clean_open_cuff(self) -> None:
        closed = generate_transtibial_limb(open_cuff=False, seed=3).mesh
        self.assertEqual(len(closed.boundary_loops()), 0)

        repaired, rep = repair_scan_mesh(closed, ensure_open_cuff=True)
        self.assertTrue(rep.clipped_proximal_cuff)
        self.assertEqual(len(repaired.boundary_loops()), 1)
        self.assertEqual(repaired.edge_topology()["non_manifold_edges"], 0)
        self.assertEqual(repaired.edge_topology()["winding_mismatches"], 0)

        # Also test direct keep_above clipping
        upper = clip_mesh_z(closed, z_cut=-50.0, keep_above=True)
        self.assertEqual(len(upper.boundary_loops()), 1)
        self.assertGreaterEqual(upper.bounds()[0][2], -50.001)


if __name__ == "__main__":
    unittest.main()
