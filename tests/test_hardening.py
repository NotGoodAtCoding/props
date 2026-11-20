"""Hardening tests for arbitrary/unstructured/closed/torn scans, TF sockets, and STL normalization."""

from __future__ import annotations

import random
import tempfile
import threading
import unittest
import urllib.request
from pathlib import Path

from op_core.linter import lint_device
from op_core.mesh import TriMesh, read_stl, write_binary_stl
from op_core.recipe import OpRecipe
from op_core.rectify import apply_recipe
from op_core.repair import repair_scan_mesh
from op_core.shell import build_tf_socket_shell, build_tt_socket_shell, export_carver_positive
from op_core.synthetic import corrupt_scan, generate_transtibial_limb
from op_core.ui_server import create_server


class TestHardening(unittest.TestCase):
    def test_closed_mesh_builds_watertight_shell_and_carver_positive(self) -> None:
        # Simulate an external closed STL (like OpenLimbTT) with zero open boundary loops
        closed = generate_transtibial_limb(open_cuff=False, seed=4).mesh
        self.assertEqual(len(closed.boundary_loops()), 0)

        shell_res = build_tt_socket_shell(closed, flexion_deg=5.0, adduction_deg=5.0)
        self.assertTrue(shell_res.shell_mesh.is_watertight_manifold(), shell_res.shell_mesh.edge_topology())
        self.assertGreater(shell_res.shell_mesh.signed_volume(), 0.0)

        carver = export_carver_positive(closed, draft_angle_deg=12.0)
        self.assertTrue(carver.positive_mesh.is_watertight_manifold(), carver.positive_mesh.edge_topology())
        self.assertGreater(carver.positive_mesh.signed_volume(), 0.0)

    def test_unstructured_perturbed_torn_flipped_soup_recovers_to_watertight_shell(self) -> None:
        # Take an open limb, add a tear hole, scramble winding, and split into un-welded triangle soup
        base = generate_transtibial_limb(open_cuff=True, seed=8).mesh
        torn = corrupt_scan(base, tear_hole=True, jitter_mm=0.25, seed=19)
        rng = random.Random(21)
        soup_verts = []
        soup_faces = []
        for a, b, c in torn.faces:
            idx = len(soup_verts)
            soup_verts.extend([torn.vertices[a], torn.vertices[b], torn.vertices[c]])
            if rng.random() < 0.35:
                soup_faces.append((idx, idx + 2, idx + 1))
            else:
                soup_faces.append((idx, idx + 1, idx + 2))
        messy = TriMesh(vertices=soup_verts, faces=soup_faces)

        repaired, rep = repair_scan_mesh(messy, ensure_open_cuff=True)
        self.assertGreater(rep.welded_vertices_removed, 1000)
        self.assertGreater(rep.flipped_faces, 50)
        self.assertEqual(rep.holes_filled, 1)

        shell_res = build_tt_socket_shell(repaired)
        self.assertTrue(shell_res.shell_mesh.is_watertight_manifold(), shell_res.shell_mesh.edge_topology())
        self.assertTrue(lint_device(shell_res.shell_mesh, measured_wall_override_mm=2.5).passed)

    def test_transfemoral_socket_end_to_end_and_ui_sample(self) -> None:
        tf_case = generate_transtibial_limb(length_mm=240.0, proximal_radius=68.0, distal_radius=44.0, open_cuff=True, seed=7)
        recipe = OpRecipe(
            device_type="transfemoral_socket",
            side="left",
            liner_thickness_mm=3.0,
            alignment={"flexion_deg": 5.0, "adduction_deg": 8.0, "mpt_offset_mm": [0.0, 0.0]},
            landmarks={"scarpa": {"xyz": list(tf_case.landmark_coords["mpt"])}},
            operations=[
                {"op": "global_compression", "id": "g", "z_range_mm": [-260.0, 60.0], "percent": -4.0, "transition_mm": 0.0},
                {"op": "landmark_relief", "id": "sc", "landmark": "scarpa", "offset_mm": -5.0, "radius_mm": 26.0},
            ],
            shell={"wall_distal_mm": 5.5, "wall_proximal_mm": 3.0, "trimline_fillet_mm": 12.0, "hardware": "pyramid_50_8_m6_captive"},
        )
        rect = apply_recipe(tf_case.mesh, recipe)
        sh = build_tf_socket_shell(rect.rectified_mesh)
        self.assertTrue(sh.shell_mesh.is_watertight_manifold(), sh.shell_mesh.edge_topology())
        self.assertEqual(sh.hardware_metrics["alignment_reference"], "tf_tka_plumb_line")

        # Verify TF sample endpoint in UI server
        server, store = create_server("127.0.0.1", 0)
        port = server.server_address[1]
        th = threading.Thread(target=server.serve_forever, daemon=True)
        th.start()
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        try:
            import json
            resp = json.loads(opener.open(f"http://127.0.0.1:{port}/api/sample?device=tf").read().decode("utf-8"))
            self.assertEqual(resp["recipe"]["device_type"], "transfemoral_socket")
            self.assertEqual(resp["qa"]["status"], "PASS")
            self.assertGreater(resp["mesh"]["bounds"]["max"][2], 80.0)
        finally:
            server.shutdown()
            server.server_close()
            store.close_all()

    def test_full_closed_limb_scans_trim_cleanly_to_socket_shells(self) -> None:
        from op_core.scan_qa import run_scan_qa
        from op_core.synthetic import generate_transfemoral_limb

        tt_full = generate_transtibial_limb(open_cuff=False, include_knee_thigh=True, seed=0)
        self.assertTrue(tt_full.mesh.is_watertight_manifold())
        self.assertGreater(tt_full.mesh.bounds()[1][2], 100.0)
        pat_y = max(v[1] for v in tt_full.mesh.vertices if 28.0 <= v[2] <= 38.0 and abs(v[0]) < 8.0)
        pat_side_y = max(v[1] for v in tt_full.mesh.vertices if 28.0 <= v[2] <= 38.0 and 14.0 <= abs(v[0]) <= 22.0)
        supra_y = max(v[1] for v in tt_full.mesh.vertices if 56.0 <= v[2] <= 63.0 and abs(v[0]) < 8.0)
        mpt_y = max(v[1] for v in tt_full.mesh.vertices if -2.0 <= v[2] <= 6.0 and abs(v[0]) < 8.0)
        self.assertGreater(pat_y - supra_y, 5.0)    # Clear patella contour below suprapatellar dip
        self.assertGreater(pat_y - mpt_y, 3.5)      # Clear patellar tendon shelf below patella apex
        self.assertLess(pat_y - pat_side_y, 4.0)    # Broad, realistically flat anterior patellar face
        aligned_tt, qa_tt = run_scan_qa(
            tt_full.mesh,
            mpt_point=tt_full.landmark_coords["mpt"],
            distal_point=tt_full.landmark_coords["distal_end"],
        )
        self.assertEqual(qa_tt.status, "PASS", qa_tt.errors)
        tt_sh = build_tt_socket_shell(aligned_tt, proximal_trim_z_mm=38.0)
        self.assertTrue(tt_sh.shell_mesh.is_watertight_manifold(), tt_sh.shell_mesh.edge_topology())
        self.assertLessEqual(tt_sh.shell_mesh.bounds()[1][2], 45.0)

        tf_full = generate_transfemoral_limb(open_cuff=False, seed=7)
        self.assertTrue(tf_full.mesh.is_watertight_manifold())
        aligned_tf, qa_tf = run_scan_qa(
            tf_full.mesh,
            mpt_point=tf_full.landmark_coords["mpt"],
            distal_point=tf_full.landmark_coords["distal_end"],
        )
        self.assertEqual(qa_tf.status, "PASS", qa_tf.errors)
        tf_sh = build_tf_socket_shell(aligned_tf, proximal_trim_z_mm=45.0)
        self.assertTrue(tf_sh.shell_mesh.is_watertight_manifold(), tf_sh.shell_mesh.edge_topology())
        self.assertLessEqual(tf_sh.shell_mesh.bounds()[1][2], 52.0)

        # Contoured PTB-SC & custom clicked 3D trimlines build watertight shells + carver positives
        from op_core.shell import TRIMLINE_PRESETS
        sc_tl = dict(TRIMLINE_PRESETS["ptb_sc"], preset="ptb_sc")
        sc_sh = build_tt_socket_shell(aligned_tt, trimline=sc_tl)
        self.assertTrue(sc_sh.shell_mesh.is_watertight_manifold(), sc_sh.shell_mesh.edge_topology())
        self.assertGreater(sc_sh.shell_mesh.bounds()[1][2], 44.0)
        sc_carver = export_carver_positive(aligned_tt, trimline=sc_tl)
        self.assertTrue(sc_carver.positive_mesh.is_watertight_manifold(), sc_carver.positive_mesh.edge_topology())

        custom_tl = {
            "preset": "custom",
            "points": [[0.0, 0.0, 34.0], [50.0, -54.0, 50.0], [0.0, -108.0, -2.0], [-50.0, -54.0, 48.0]],
        }
        cust_sh = build_tt_socket_shell(aligned_tt, trimline=custom_tl)
        self.assertTrue(cust_sh.shell_mesh.is_watertight_manifold(), cust_sh.shell_mesh.edge_topology())

    def test_fetch_openlimb_normalizer_scales_fractional_mesh(self) -> None:
        from scripts.fetch_openlimb import normalize_openlimb_stl

        # Create a normalized fractional mesh (~0..1 unit height)
        case = generate_transtibial_limb(length_mm=180.0, open_cuff=False, seed=0)
        frac = case.mesh.transformed(lambda p: (p[0] / 215.0, p[1] / 215.0, p[2] / 215.0))
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / "frac.stl"
            dst = Path(tmp) / "norm.stl"
            write_binary_stl(frac, src)
            normalize_openlimb_stl(src, dst, target_length_mm=190.0)
            loaded = read_stl(dst)
            bmin, bmax = loaded.bounds()
            self.assertGreater(bmax[2] - bmin[2], 150.0)
            self.assertEqual(len(loaded.boundary_loops()), 1)


if __name__ == "__main__":
    unittest.main()
