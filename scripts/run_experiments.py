"""Reproducible benchmark runner for EXP-1 through EXP-5."""

from __future__ import annotations

import math
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from op_core.export_3mf import export_3mf
from op_core.linter import lint_device
from op_core.math3d import rodrigues_rotate, v_dist, v_dot, v_sub
from op_core.recipe import OpRecipe
from op_core.rectify import apply_afo_posture, apply_recipe
from op_core.repair import repair_scan_mesh
from op_core.scan_qa import measure_circumferences, run_scan_qa
from op_core.shell import build_afo_shell, build_tf_socket_shell, build_tt_socket_shell, export_carver_positive
from op_core.synthetic import corrupt_scan, generate_foot_ankle_mesh, generate_transtibial_limb


def run_exp1() -> dict:
    t0 = time.perf_counter()
    errors: list[float] = []
    detected_corruptions = 0
    for seed in range(5):
        case = generate_transtibial_limb(open_cuff=True, seed=seed)
        _, gt_rep = run_scan_qa(
            case.mesh,
            mpt_point=case.landmark_coords["mpt"],
            distal_point=case.landmark_coords["distal_end"],
        )
        gt = {round(s["depth_from_anchor_mm"]): s["taut_circumference_mm"] for s in gt_rep.slices}

        tilted = corrupt_scan(case.mesh, tilt_deg=15.0, jitter_mm=0.3, seed=100 + seed)
        rad = math.radians(15.0)
        mpt_r = rodrigues_rotate(case.landmark_coords["mpt"], (1.0, 0.25, 0.0), rad)
        dst_r = rodrigues_rotate(case.landmark_coords["distal_end"], (1.0, 0.25, 0.0), rad)
        _, rep = run_scan_qa(tilted, mpt_point=mpt_r, distal_point=dst_r)
        for s in rep.slices:
            d = round(s["depth_from_anchor_mm"])
            if d in gt:
                errors.append(abs(s["taut_circumference_mm"] - gt[d]))

        bad = corrupt_scan(case.mesh, scale_factor=1.03, seam_step_mm=3.0, double_wall=True, floaters=2, tear_hole=True)
        _, bad_rep = run_scan_qa(bad, calibration_measured_mm=103.0)
        if bad_rep.status == "FAIL" and bad_rep.internal_tear_loops >= 1 and bad_rep.double_wall_vertices > 15:
            detected_corruptions += 1

    dt = time.perf_counter() - t0
    mean_err = sum(errors) / len(errors)
    return {
        "mean_circumference_err_mm": round(mean_err, 3),
        "max_circumference_err_mm": round(max(errors), 3),
        "corrupted_detection_rate": f"{detected_corruptions}/5",
        "elapsed_s": round(dt, 3),
        "pass": mean_err <= 1.5 and detected_corruptions == 5 and dt < 25.0,
    }


def run_exp2() -> dict:
    t0 = time.perf_counter()
    max_zoned_drift = 0.0
    max_relief_drift = 0.0
    for seed in range(5):
        case = generate_transtibial_limb(open_cuff=True, seed=seed)
        r_base = OpRecipe(
            device_type="transtibial_socket",
            liner_thickness_mm=3.0,
            landmarks={
                "pt_bar": {"vertex_idx": case.landmarks["mpt"]},
                "fibular_head": {"vertex_idx": case.landmarks["fibular_head"]},
            },
            operations=[
                {"op": "global_compression", "id": "glob", "z_range_mm": [-200.0, 50.0], "percent": -3.0, "transition_mm": 0.0},
                {"op": "zoned_compression", "id": "dist", "z_range_mm": [-200.0, -100.0], "percent": 0.0, "transition_mm": 20.0},
                {"op": "landmark_relief", "id": "pt", "landmark": "pt_bar", "offset_mm": -6.0, "radius_mm": 20.0},
                {"op": "landmark_relief", "id": "fib", "landmark": "fibular_head", "offset_mm": 4.0, "radius_mm": 24.0},
            ],
        )
        r_zoned = OpRecipe.from_dict({**r_base.to_dict(), "operations": [
            r_base.operations[0],
            {"op": "zoned_compression", "id": "dist", "z_range_mm": [-200.0, -100.0], "percent": -4.0, "transition_mm": 20.0},
            r_base.operations[2],
            r_base.operations[3],
        ]})
        r_glob = OpRecipe.from_dict({**r_base.to_dict(), "operations": [
            {"op": "global_compression", "id": "glob", "z_range_mm": [-200.0, 50.0], "percent": -1.0, "transition_mm": 0.0},
            r_base.operations[1],
            r_base.operations[2],
            r_base.operations[3],
        ]})

        o1 = apply_recipe(case.mesh, r_base)
        o2 = apply_recipe(case.mesh, r_zoned)
        o3 = apply_recipe(case.mesh, r_glob)

        for i, v in enumerate(case.mesh.vertices):
            if v[2] > -70.0:
                max_zoned_drift = max(max_zoned_drift, v_dist(o1.rectified_mesh.vertices[i], o2.rectified_mesh.vertices[i]))

        n0 = case.mesh.vertex_normals()
        for lm in ("pt_bar", "fibular_head"):
            idx = o1.resolved_landmarks[lm]
            h1 = v_dot(v_sub(o1.rectified_mesh.vertices[idx], o1.base_compressed_mesh.vertices[idx]), n0[idx])
            h3 = v_dot(v_sub(o3.rectified_mesh.vertices[idx], o3.base_compressed_mesh.vertices[idx]), n0[idx])
            max_relief_drift = max(max_relief_drift, abs(h1 - h3))

    dt = time.perf_counter() - t0
    return {
        "max_zoned_drift_mm": round(max_zoned_drift, 5),
        "max_relief_height_drift_mm": round(max_relief_drift, 5),
        "elapsed_s": round(dt, 3),
        "pass": max_zoned_drift < 0.2 and max_relief_drift < 0.2,
    }


def run_exp2b_and_exp3() -> dict:
    t0 = time.perf_counter()
    tt_watertight = 0
    afo_watertight = 0
    max_align_err_deg = 0.0

    expected_n = rodrigues_rotate((0.0, 0.0, -1.0), (1.0, 0.0, 0.0), math.radians(5.0))
    expected_n = rodrigues_rotate(expected_n, (0.0, 1.0, 0.0), math.radians(5.0))

    for seed in range(10):
        case = generate_transtibial_limb(open_cuff=True, seed=seed)
        res = build_tt_socket_shell(case.mesh, flexion_deg=5.0, adduction_deg=5.0)
        if res.shell_mesh.is_watertight_manifold() and res.shell_mesh.signed_volume() > 0:
            tt_watertight += 1
        err = math.degrees(math.acos(max(-1.0, min(1.0, v_dot(res.alignment_normal, expected_n)))))
        max_align_err_deg = max(max_align_err_deg, err)

    for pf in (8.0, 10.0, 12.0, 14.0, 15.0):
        afo_case = generate_foot_ankle_mesh(plantarflexion_deg=pf, inversion_deg=5.0)
        neutral = apply_afo_posture(afo_case.mesh, dorsiflexion_deg=pf, eversion_deg=5.0, heel_pitch_mm=10.0)
        res = build_afo_shell(neutral, trimline_fillet_mm=10.0)
        if res.shell_mesh.is_watertight_manifold() and res.shell_mesh.signed_volume() > 0:
            afo_watertight += 1

    # Hardening check: closed + torn unstructured limbs + transfemoral shell
    hardened_ok = 0
    for seed in range(3):
        closed_raw = generate_transtibial_limb(open_cuff=False, seed=seed).mesh
        torn = corrupt_scan(closed_raw, tear_hole=True, jitter_mm=0.2, seed=30 + seed)
        repaired, _ = repair_scan_mesh(torn, ensure_open_cuff=True)
        sh_tt = build_tt_socket_shell(repaired)
        sh_tf = build_tf_socket_shell(repaired)
        if (
            sh_tt.shell_mesh.is_watertight_manifold()
            and sh_tt.shell_mesh.signed_volume() > 0
            and sh_tf.shell_mesh.is_watertight_manifold()
            and sh_tf.shell_mesh.signed_volume() > 0
        ):
            hardened_ok += 1

    dt = time.perf_counter() - t0
    return {
        "tt_watertight": f"{tt_watertight}/10",
        "afo_watertight": f"{afo_watertight}/5",
        "hardened_closed_torn_and_tf": f"{hardened_ok}/3",
        "max_alignment_err_deg": round(max_align_err_deg, 4),
        "elapsed_s": round(dt, 3),
        "pass": tt_watertight == 10 and afo_watertight == 5 and hardened_ok == 3 and max_align_err_deg <= 0.2,
    }


def run_exp4_and_exp5() -> dict:
    t0 = time.perf_counter()
    case = generate_transtibial_limb(open_cuff=True, seed=0)
    shell_res = build_tt_socket_shell(case.mesh)
    with tempfile.TemporaryDirectory() as tmp:
        p1 = export_3mf(shell_res.shell_mesh, shell_res.modifier_mesh, Path(tmp) / "prusa.3mf", target="prusaslicer")
        p2 = export_3mf(shell_res.shell_mesh, shell_res.modifier_mesh, Path(tmp) / "orca.3mf", target="orcaslicer")
        both_3mf = p1.exists() and p2.exists()

    lint_ok = lint_device(shell_res.shell_mesh, measured_wall_override_mm=2.5).passed
    lint_bad = not lint_device(
        shell_res.shell_mesh,
        measured_wall_override_mm=1.8,
        trimline_points=[(0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (2.0, 2.0, 0.0)],
    ).passed

    carver_ok = 0
    for seed in range(5):
        c = generate_transtibial_limb(open_cuff=True, seed=seed)
        carv = export_carver_positive(c.mesh, draft_angle_deg=12.0)
        if carv.positive_mesh.is_watertight_manifold() and carv.positive_mesh.signed_volume() > 0:
            carver_ok += 1

    dt = time.perf_counter() - t0
    return {
        "3mf_dual_schema": both_3mf,
        "linter_discriminates": lint_ok and lint_bad,
        "carver_watertight": f"{carver_ok}/5",
        "elapsed_s": round(dt, 3),
        "pass": both_3mf and lint_ok and lint_bad and carver_ok == 5,
    }


if __name__ == "__main__":
    import argparse
    import json

    parser = argparse.ArgumentParser(
        prog="run_experiments.py",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=(
            "Run reproducible op-core validation benchmarks (EXP-1 through EXP-5).\n"
            "See experiments/high_value_experiments.md for hypothesis and pass/fail criteria."
        ),
        epilog=(
            "Examples:\n"
            "  PYTHONPATH=src python3 scripts/run_experiments.py\n"
            "  PYTHONPATH=src python3 scripts/run_experiments.py --exp EXP-1"
        ),
    )
    parser.add_argument(
        "--exp",
        choices=["all", "EXP-1", "EXP-2", "EXP-2B_and_EXP-3", "EXP-4_and_EXP-5"],
        default="all",
        help="Run all experiments or a specific benchmark group (default: all)",
    )
    args = parser.parse_args()

    runners = {
        "EXP-1": run_exp1,
        "EXP-2": run_exp2,
        "EXP-2B_and_EXP-3": run_exp2b_and_exp3,
        "EXP-4_and_EXP-5": run_exp4_and_exp5,
    }
    selected = runners if args.exp == "all" else {args.exp: runners[args.exp]}
    summary = {name: fn() for name, fn in selected.items()}
    print(json.dumps(summary, indent=2))
    all_passed = all(v["pass"] for v in summary.values())
    sys.exit(0 if all_passed else 1)
