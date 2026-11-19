"""Command-line interface for op-core headless scan QA, recipe build, lint, diff, and demo."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .export_3mf import export_3mf
from .linter import export_fabrication_dossier, lint_device
from .mesh import read_stl, write_binary_stl
from .recipe import OpRecipe, diff_recipes, mesh_sha256
from .rectify import apply_recipe
from .scan_qa import run_scan_qa
from .shell import build_afo_shell, build_tt_socket_shell, export_carver_positive
from .synthetic import generate_foot_ankle_mesh, generate_transtibial_limb
from .ui_server import serve_ui


def _build_device(
    scan_path: Path,
    recipe: OpRecipe,
    out_dir: Path,
    target: str,
) -> dict[str, object]:
    """Run QA -> rectification -> shell/carver/3MF compilation -> safety lint -> dossier."""
    raw = read_stl(scan_path)
    clean, qa_report = run_scan_qa(raw)
    if not recipe.scan_sha256:
        recipe.scan_sha256 = mesh_sha256(clean)

    rect = apply_recipe(clean, recipe)
    out_dir.mkdir(parents=True, exist_ok=True)
    write_binary_stl(rect.rectified_mesh, out_dir / "rectified_positive.stl")

    if recipe.device_type == "afo":
        shell_res = build_afo_shell(
            rect.rectified_mesh,
            wall_neck_mm=float(recipe.shell.get("wall_distal_mm", 4.5)),
            wall_calf_mm=float(recipe.shell.get("wall_proximal_mm", 2.5)),
            trimline_fillet_mm=float(recipe.shell.get("trimline_fillet_mm", 10.0)),
        )
    else:
        shell_res = build_tt_socket_shell(
            rect.rectified_mesh,
            wall_distal_mm=float(recipe.shell.get("wall_distal_mm", 4.5)),
            wall_proximal_mm=float(recipe.shell.get("wall_proximal_mm", 2.5)),
            flexion_deg=float(recipe.alignment.get("flexion_deg", 5.0)),
            adduction_deg=float(recipe.alignment.get("adduction_deg", 5.0)),
        )

    write_binary_stl(shell_res.shell_mesh, out_dir / "device_shell.stl")
    threemf_path = export_3mf(
        shell_res.shell_mesh,
        shell_res.modifier_mesh,
        out_dir / f"device_{target}.3mf",
        target=target,
    )
    carver = export_carver_positive(rect.rectified_mesh)
    write_binary_stl(carver.positive_mesh, out_dir / "carver_positive.stl")

    lint_rep = lint_device(
        shell_res.shell_mesh,
        measured_wall_override_mm=float(recipe.shell.get("wall_proximal_mm", 2.5)),
    )
    dossier_json, dossier_md = export_fabrication_dossier(recipe, qa_report, lint_rep, out_dir)
    return {
        "qa_status": qa_report.status,
        "lint_passed": lint_rep.passed,
        "out_dir": str(out_dir),
        "outputs": {
            "rectified_positive": str(out_dir / "rectified_positive.stl"),
            "device_shell": str(out_dir / "device_shell.stl"),
            "slicer_3mf": str(threemf_path),
            "carver_positive": str(out_dir / "carver_positive.stl"),
            "dossier_json": str(dossier_json),
            "dossier_md": str(dossier_md),
        },
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="op-core",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=(
            "op-core: Headless MIT computational geometry CLI for Orthotics & Prosthetics (O&P).\n"
            "Supports chairside scan QA, parametric .oprecipe.json rectification, watertight\n"
            "TT/AFO shell generation, slicer-targeted .3mf export, CNC carver prep, and safety linting."
        ),
        epilog=(
            "Examples:\n"
            "  op-core demo --out-dir ./demo_tt --device tt\n"
            "  op-core qa patient_scan.stl --out-stl aligned_scan.stl\n"
            "  op-core build patient_scan.stl examples/tt_socket.oprecipe.json --out-dir ./out --target orcaslicer\n"
            "  op-core lint ./out/device_shell.stl --min-wall-mm 2.5\n"
            "  op-core diff v1.oprecipe.json v2.oprecipe.json\n"
            "  op-core ui --port 8765\n\n"
            "Docs: see docs/getting_started.md and docs/cli_and_tools_reference.md"
        ),
    )
    sub = parser.add_subparsers(dest="cmd", required=True, metavar="COMMAND")

    p_demo = sub.add_parser(
        "demo",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        help="Generate a synthetic TT or AFO scan + recipe and run the full pipeline offline",
        description=(
            "Run an end-to-end offline demonstration using a procedural clinical mesh.\n"
            "Creates sample_scan.stl, sample.oprecipe.json, rectified/shell/carver STLs,\n"
            "a slicer-targeted .3mf, and a custom-device fabrication dossier."
        ),
        epilog=(
            "Example:\n"
            "  op-core demo --out-dir ./demo_out --device tt --target orcaslicer"
        ),
    )
    p_demo.add_argument("--out-dir", required=True, help="Directory where demo inputs and outputs are written")
    p_demo.add_argument("--device", choices=["tt", "afo"], default="tt", help="Demo device type (default: tt)")
    p_demo.add_argument(
        "--target",
        choices=["orcaslicer", "prusaslicer"],
        default="orcaslicer",
        help="Target slicer .3mf modifier schema (default: orcaslicer)",
    )

    p_qa = sub.add_parser(
        "qa",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        help="Run chairside scan QA, canonical alignment, and 20 mm taut-tape measurements",
        description=(
            "Validate a raw limb STL scan before CAD rectification: prunes detached floaters,\n"
            "classifies open cuffs vs. internal tears, checks SLAM double walls & seam steps,\n"
            "aligns +Z proximal, and slices convex-hull circumferences every 20 mm."
        ),
        epilog=(
            "Example:\n"
            "  op-core qa raw_limb.stl --out-stl aligned_limb.stl --expected-bar-mm 100 --measured-bar-mm 100"
        ),
    )
    p_qa.add_argument("scan", help="Path to input .stl scan (binary or ASCII, in mm)")
    p_qa.add_argument("--out-stl", help="Optional path to write cleaned, canonically aligned binary STL")
    p_qa.add_argument("--measured-bar-mm", type=float, help="Optional measured calibration bar length in scan (mm)")
    p_qa.add_argument("--expected-bar-mm", type=float, default=100.0, help="Expected calibration bar length (default: 100.0 mm)")

    p_build = sub.add_parser(
        "build",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        help="Compile scan + .oprecipe.json into shell STL, .3mf, carver mold, and dossier",
        description=(
            "Execute a declarative .oprecipe.json v0.1 file on an input STL scan to emit:\n"
            "  - rectified_positive.stl (modified limb positive)\n"
            "  - device_shell.stl       (watertight printable socket/AFO shell)\n"
            "  - device_<target>.3mf    (shell + 100%% reinforcement modifier mesh)\n"
            "  - carver_positive.stl    (CNC foam mold with keyed mandrel & 12° draft collar)\n"
            "  - device_dossier.json/md (custom-device traceability bundle)"
        ),
        epilog=(
            "Example:\n"
            "  op-core build scan.stl examples/tt_socket.oprecipe.json --out-dir ./build_out --target prusaslicer"
        ),
    )
    p_build.add_argument("scan", help="Path to input .stl scan (mm)")
    p_build.add_argument("recipe", help="Path to .oprecipe.json v0.1 file")
    p_build.add_argument("--out-dir", required=True, help="Output directory for generated CAD/CAM files")
    p_build.add_argument(
        "--target",
        choices=["orcaslicer", "prusaslicer"],
        default="orcaslicer",
        help="Target slicer .3mf modifier schema (default: orcaslicer)",
    )

    p_lint = sub.add_parser(
        "lint",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        help="Check an existing device shell STL for manifoldness, wall thickness, and print-Z tilt",
        description="Run pre-print safety geometry checks on a socket or AFO shell STL file.",
        epilog="Example:\n  op-core lint ./build_out/device_shell.stl --min-wall-mm 2.5",
    )
    p_lint.add_argument("shell_stl", help="Path to shell .stl file to inspect")
    p_lint.add_argument("--min-wall-mm", type=float, default=2.5, help="Minimum safe load-zone wall thickness in mm (default: 2.5)")

    p_diff = sub.add_parser(
        "diff",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        help="Compare two .oprecipe.json files and print a structured clinical revision diff",
        description="Display parameter and operation differences between two check-socket/AFO recipe revisions.",
        epilog="Example:\n  op-core diff check_v1.oprecipe.json check_v2.oprecipe.json",
    )
    p_diff.add_argument("recipe_v1", help="Baseline .oprecipe.json path")
    p_diff.add_argument("recipe_v2", help="Updated .oprecipe.json path")

    p_ui = sub.add_parser(
        "ui",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        help="Launch the local 3D WebGL browser workstation (Phase 4 5-step clinical UI)",
        description="Start a local offline-capable HTTP + WebGL 3D workstation for scan QA, landmark picking, rectification, and export.",
        epilog="Example:\n  op-core ui --host 127.0.0.1 --port 8765 --no-browser",
    )
    p_ui.add_argument("--host", default="127.0.0.1", help="Bind address (default: 127.0.0.1)")
    p_ui.add_argument("--port", type=int, default=8765, help="Bind port (default: 8765)")
    p_ui.add_argument("--no-browser", action="store_true", help="Do not automatically open a browser tab")

    return parser


def main(argv: list[str] | None = None) -> int:
    """CLI entry point for `op-core`."""
    parser = _build_parser()
    args = parser.parse_args(argv)

    if args.cmd == "demo":
        out_dir = Path(args.out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        scan_path = out_dir / "sample_scan.stl"
        recipe_path = out_dir / "sample.oprecipe.json"

        if args.device == "afo":
            case = generate_foot_ankle_mesh(plantarflexion_deg=12.0, inversion_deg=5.0)
            recipe = OpRecipe(
                device_type="afo",
                side="left",
                landmarks={k: {"xyz": list(v)} for k, v in case.landmark_coords.items()},
                operations=[
                    {"op": "afo_posture", "id": "neutralize", "dorsiflexion_deg": 12.0, "eversion_deg": 5.0, "heel_pitch_mm": 10.0},
                    {"op": "landmark_relief", "id": "med_mal", "landmark": "medial_malleolus", "offset_mm": 4.0, "radius_mm": 22.0},
                    {"op": "landmark_relief", "id": "lat_mal", "landmark": "lateral_malleolus", "offset_mm": 4.0, "radius_mm": 22.0},
                ],
                shell={"wall_distal_mm": 4.5, "wall_proximal_mm": 2.5, "trimline_fillet_mm": 10.0, "hardware": "afo_coaxial_joint"},
            )
            write_binary_stl(case.mesh, scan_path)
            recipe.save(recipe_path)
            # Run AFO pipeline directly on canonical sample mesh (skip TT shaft QA truncation warning)
            rect = apply_recipe(case.mesh, recipe)
            write_binary_stl(rect.rectified_mesh, out_dir / "rectified_positive.stl")
            shell_res = build_afo_shell(rect.rectified_mesh, wall_neck_mm=4.5, wall_calf_mm=2.5, trimline_fillet_mm=10.0)
            write_binary_stl(shell_res.shell_mesh, out_dir / "device_shell.stl")
            threemf_path = export_3mf(shell_res.shell_mesh, shell_res.modifier_mesh, out_dir / f"device_{args.target}.3mf", target=args.target)
            _, qa_rep = run_scan_qa(case.mesh)
            lint_rep = lint_device(shell_res.shell_mesh, measured_wall_override_mm=2.5)
            dj, dm = export_fabrication_dossier(recipe, qa_rep, lint_rep, out_dir)
            print(json.dumps({
                "device": "afo",
                "sample_scan": str(scan_path),
                "sample_recipe": str(recipe_path),
                "lint_passed": lint_rep.passed,
                "outputs": {
                    "rectified_positive": str(out_dir / "rectified_positive.stl"),
                    "device_shell": str(out_dir / "device_shell.stl"),
                    "slicer_3mf": str(threemf_path),
                    "dossier_json": str(dj),
                    "dossier_md": str(dm),
                },
            }, indent=2))
            return 0 if lint_rep.passed else 1

        case = generate_transtibial_limb(open_cuff=True, seed=0)
        write_binary_stl(case.mesh, scan_path)
        recipe = OpRecipe(
            device_type="transtibial_socket",
            side="left",
            liner_thickness_mm=3.0,
            landmarks={
                "pt_bar": {"xyz": list(case.landmark_coords["mpt"])},
                "fibular_head": {"xyz": list(case.landmark_coords["fibular_head"])},
                "tibial_crest": {"xyz": list(case.landmark_coords["tibial_crest"])},
            },
            operations=[
                {"op": "global_compression", "id": "glob", "z_range_mm": [-200.0, 50.0], "percent": -3.0, "transition_mm": 0.0},
                {"op": "landmark_relief", "id": "pt", "landmark": "pt_bar", "offset_mm": -6.0, "radius_mm": 20.0},
                {"op": "landmark_relief", "id": "fib", "landmark": "fibular_head", "offset_mm": 4.0, "radius_mm": 24.0},
                {"op": "landmark_relief", "id": "crest", "landmark": "tibial_crest", "offset_mm": 3.0, "radius_mm": 22.0},
            ],
        )
        recipe.save(recipe_path)
        summary = _build_device(scan_path, recipe, out_dir, args.target)
        summary["sample_scan"] = str(scan_path)
        summary["sample_recipe"] = str(recipe_path)
        print(json.dumps(summary, indent=2))
        return 0 if summary["lint_passed"] else 1

    if args.cmd == "qa":
        raw = read_stl(args.scan)
        clean, report = run_scan_qa(
            raw,
            calibration_measured_mm=args.measured_bar_mm,
            calibration_expected_mm=args.expected_bar_mm,
        )
        if args.out_stl:
            write_binary_stl(clean, args.out_stl)
        print(json.dumps({
            "status": report.status,
            "floater_components_removed": report.floater_components_removed,
            "proximal_cuff_loops": report.proximal_cuff_loops,
            "internal_tear_loops": report.internal_tear_loops,
            "double_wall_vertices": report.double_wall_vertices,
            "seam_step_score_mm": round(report.seam_step_score_mm, 3),
            "detected_scale_ratio": report.detected_scale_ratio,
            "warnings": report.warnings,
            "errors": report.errors,
            "slices": report.slices,
        }, indent=2))
        return 0 if report.status != "FAIL" else 1

    if args.cmd == "build":
        recipe = OpRecipe.load(args.recipe)
        summary = _build_device(Path(args.scan), recipe, Path(args.out_dir), args.target)
        print(json.dumps(summary, indent=2))
        return 0 if summary["lint_passed"] else 1

    if args.cmd == "lint":
        mesh = read_stl(args.shell_stl)
        rep = lint_device(mesh, min_wall_mm=args.min_wall_mm)
        print(json.dumps({
            "passed": rep.passed,
            "is_watertight_manifold": rep.is_watertight_manifold,
            "min_estimated_wall_mm": round(rep.min_estimated_wall_mm, 2),
            "trimline_min_radius_mm": round(rep.trimline_min_radius_mm, 2),
            "load_to_print_z_angle_deg": round(rep.load_to_print_z_angle_deg, 2),
            "failures": rep.failures,
            "warnings": rep.warnings,
        }, indent=2))
        return 0 if rep.passed else 1

    if args.cmd == "diff":
        v1 = OpRecipe.load(args.recipe_v1)
        v2 = OpRecipe.load(args.recipe_v2)
        print(json.dumps(diff_recipes(v1, v2), indent=2))
        return 0

    if args.cmd == "ui":
        serve_ui(host=args.host, port=args.port, open_browser=not args.no_browser)
        return 0

    return 0
