"""Command-line interface for op-core headless scan QA, recipe execution, 3MF & carver export."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .export_3mf import export_3mf
from .linter import export_fabrication_dossier, lint_device
from .mesh import read_stl, write_binary_stl
from .recipe import OpRecipe, mesh_sha256
from .rectify import apply_recipe
from .scan_qa import run_scan_qa
from .shell import build_afo_shell, build_tt_socket_shell, export_carver_positive


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="op-core", description="Headless O&P CAD/CAM kernel CLI")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_qa = sub.add_parser("qa", help="Run chairside scan QA and 20 mm circumference extraction")
    p_qa.add_argument("scan", help="Path to input .stl scan")
    p_qa.add_argument("--out-stl", help="Optional path to save cleaned/aligned canonical STL")

    p_build = sub.add_parser("build", help="Compile scan + .oprecipe.json to shell STL/3MF, carver mold, and dossier")
    p_build.add_argument("scan", help="Path to input .stl scan")
    p_build.add_argument("recipe", help="Path to .oprecipe.json")
    p_build.add_argument("--out-dir", required=True, help="Output directory")
    p_build.add_argument("--target", choices=["orcaslicer", "prusaslicer"], default="orcaslicer")

    args = parser.parse_args(argv)

    if args.cmd == "qa":
        raw = read_stl(args.scan)
        clean, report = run_scan_qa(raw)
        if args.out_stl:
            write_binary_stl(clean, args.out_stl)
        print(json.dumps({
            "status": report.status,
            "warnings": report.warnings,
            "errors": report.errors,
            "slices": report.slices,
        }, indent=2))
        return 0 if report.status != "FAIL" else 1

    if args.cmd == "build":
        raw = read_stl(args.scan)
        clean, qa_report = run_scan_qa(raw)
        recipe = OpRecipe.load(args.recipe)
        if not recipe.scan_sha256:
            recipe.scan_sha256 = mesh_sha256(clean)
        rect = apply_recipe(clean, recipe)
        out_dir = Path(args.out_dir)
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
        export_3mf(shell_res.shell_mesh, shell_res.modifier_mesh, out_dir / f"device_{args.target}.3mf", target=args.target)
        carver = export_carver_positive(rect.rectified_mesh)
        write_binary_stl(carver.positive_mesh, out_dir / "carver_positive.stl")

        lint_rep = lint_device(
            shell_res.shell_mesh,
            measured_wall_override_mm=float(recipe.shell.get("wall_proximal_mm", 2.5)),
        )
        export_fabrication_dossier(recipe, qa_report, lint_rep, out_dir)
        print(json.dumps({"lint_passed": lint_rep.passed, "out_dir": str(out_dir)}, indent=2))
        return 0 if lint_rep.passed else 1

    return 0
