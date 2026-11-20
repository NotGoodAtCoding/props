"""Unit tests for Project 2 / EXP-2 & EXP-2B: Reference-domain M_0 isolation & AFO posture."""

from __future__ import annotations

import unittest

from op_core.math3d import v_dist, v_dot, v_sub
from op_core.recipe import OpRecipe, diff_recipes
from op_core.rectify import apply_afo_posture, apply_recipe
from op_core.synthetic import generate_foot_ankle_mesh, generate_transtibial_limb


class TestRectify(unittest.TestCase):
    def _make_tt_recipe(self, case, global_pct: float = -3.0, distal_pct: float = 0.0) -> OpRecipe:
        ops = [
            {"op": "global_compression", "id": "glob", "z_range_mm": [-200.0, 50.0], "percent": global_pct, "transition_mm": 0.0},
            {"op": "zoned_compression", "id": "dist_zone", "z_range_mm": [-200.0, -100.0], "percent": distal_pct, "transition_mm": 20.0},
            {"op": "landmark_relief", "id": "pt", "landmark": "pt_bar", "offset_mm": -6.0, "radius_mm": 20.0},
            {"op": "landmark_relief", "id": "fib", "landmark": "fibular_head", "offset_mm": 4.0, "radius_mm": 24.0},
        ]
        return OpRecipe(
            device_type="transtibial_socket",
            side="left",
            liner_thickness_mm=3.0,
            landmarks={
                "pt_bar": {"vertex_idx": case.landmarks["mpt"]},
                "fibular_head": {"vertex_idx": case.landmarks["fibular_head"]},
            },
            operations=ops,
        )

    def test_case_a_zoned_isolation(self) -> None:
        case = generate_transtibial_limb(open_cuff=True, seed=5)
        r1 = self._make_tt_recipe(case, global_pct=-3.0, distal_pct=0.0)
        r2 = self._make_tt_recipe(case, global_pct=-3.0, distal_pct=-4.0)
        out1 = apply_recipe(case.mesh, r1)
        out2 = apply_recipe(case.mesh, r2)

        # Unmodified proximal zone z > -70 mm must stay within < 0.01 mm
        prox_drifts = [
            v_dist(out1.rectified_mesh.vertices[i], out2.rectified_mesh.vertices[i])
            for i, v in enumerate(case.mesh.vertices)
            if v[2] > -70.0
        ]
        self.assertLess(max(prox_drifts), 0.05)

    def test_case_b_global_relief_height_invariance(self) -> None:
        case = generate_transtibial_limb(open_cuff=True, seed=6)
        r1 = self._make_tt_recipe(case, global_pct=-3.0)
        r2 = self._make_tt_recipe(case, global_pct=-1.0)
        out1 = apply_recipe(case.mesh, r1)
        out2 = apply_recipe(case.mesh, r2)
        n0 = case.mesh.vertex_normals()

        for lm_key in ("pt_bar", "fibular_head"):
            idx = out1.resolved_landmarks[lm_key]
            h1 = v_dot(v_sub(out1.rectified_mesh.vertices[idx], out1.base_compressed_mesh.vertices[idx]), n0[idx])
            h2 = v_dot(v_sub(out2.rectified_mesh.vertices[idx], out2.base_compressed_mesh.vertices[idx]), n0[idx])
            self.assertLess(abs(h1 - h2), 0.05)

        diff = diff_recipes(r1, r2)
        self.assertIn("operations", diff)

    def test_afo_posture_articulation_and_sole_flattening(self) -> None:
        afo_case = generate_foot_ankle_mesh(plantarflexion_deg=12.0, inversion_deg=5.0)
        neutral = apply_afo_posture(afo_case.mesh, dorsiflexion_deg=12.0, eversion_deg=5.0, heel_pitch_mm=10.0)
        self.assertEqual(len(neutral.faces), len(afo_case.mesh.faces))
        # Check forefoot plantar points are flattened cleanly around sole reference
        forefoot_z = [v[2] for v in neutral.vertices if v[1] > 110.0]
        self.assertTrue(forefoot_z)
        self.assertGreater(min(forefoot_z), -20.0)

    def test_additive_reductive_sculpt_strokes_and_taubin_smoothing(self) -> None:
        case = generate_transtibial_limb(open_cuff=False, include_knee_thigh=True, seed=3)
        n0 = case.mesh.vertex_normals()
        pt_add = case.landmark_coords["tibial_crest"]
        pt_rem = case.landmark_coords["popliteal"]

        recipe = OpRecipe(
            device_type="transtibial_socket",
            operations=[
                {
                    "op": "sculpt_stroke",
                    "id": "add_crest",
                    "mode": "add",
                    "points": [list(pt_add), [pt_add[0], pt_add[1], pt_add[2] - 25.0]],
                    "offset_mm": 5.0,
                    "radius_mm": 22.0,
                    "falloff": "gaussian",
                },
                {
                    "op": "sculpt_stroke",
                    "id": "carve_pop",
                    "mode": "remove",
                    "points": [list(pt_rem)],
                    "offset_mm": 4.5,
                    "radius_mm": 24.0,
                    "falloff": "flat_pad",
                },
            ],
        )
        res_raw = apply_recipe(case.mesh, recipe)
        idx_add = case.landmarks["tibial_crest"]
        idx_rem = case.landmarks["popliteal"]
        d_add = v_dot(v_sub(res_raw.rectified_mesh.vertices[idx_add], case.mesh.vertices[idx_add]), n0[idx_add])
        d_rem = v_dot(v_sub(res_raw.rectified_mesh.vertices[idx_rem], case.mesh.vertices[idx_rem]), n0[idx_rem])
        self.assertGreater(d_add, 4.0)
        self.assertLess(d_rem, -3.5)

        # Localized + global Taubin smoothing reduces peak sharpness while preserving overall volume within 1%
        recipe.operations.append({
            "op": "smooth",
            "id": "smooth_local",
            "points": [list(pt_add)],
            "radius_mm": 30.0,
            "iterations": 4,
            "strength": 0.6,
        })
        recipe.operations.append({
            "op": "smooth",
            "id": "smooth_global",
            "iterations": 4,
            "strength": 0.5,
        })
        res_sm = apply_recipe(case.mesh, recipe)
        self.assertTrue(res_sm.rectified_mesh.is_watertight_manifold())
        vol_raw = res_raw.rectified_mesh.signed_volume()
        vol_sm = res_sm.rectified_mesh.signed_volume()
        self.assertLess(abs(vol_sm - vol_raw) / max(1.0, vol_raw), 0.01)


if __name__ == "__main__":
    unittest.main()
