"""Headless MIT computational geometry kernel for open-source O&P CAD/CAM."""

from .export_3mf import export_3mf
from .linter import LintReport, export_fabrication_dossier, lint_device
from .mesh import TriMesh, read_stl, write_binary_stl
from .recipe import OpRecipe, diff_recipes, mesh_sha256
from .rectify import RectificationResult, apply_afo_posture, apply_recipe
from .repair import (
    RepairReport,
    clip_mesh_z,
    fill_small_holes,
    fix_consistent_winding,
    repair_scan_mesh,
    weld_and_clean,
)
from .scan_qa import ScanQAReport, align_canonical_limb, measure_circumferences, run_scan_qa
from .shell import (
    CarverPositiveResult,
    ShellBuildResult,
    build_afo_shell,
    build_tt_socket_shell,
    export_carver_positive,
)

__all__ = [
    "CarverPositiveResult",
    "LintReport",
    "OpRecipe",
    "RectificationResult",
    "RepairReport",
    "ScanQAReport",
    "ShellBuildResult",
    "TriMesh",
    "align_canonical_limb",
    "apply_afo_posture",
    "apply_recipe",
    "build_afo_shell",
    "build_tt_socket_shell",
    "clip_mesh_z",
    "diff_recipes",
    "export_3mf",
    "export_carver_positive",
    "export_fabrication_dossier",
    "fill_small_holes",
    "fix_consistent_winding",
    "lint_device",
    "measure_circumferences",
    "mesh_sha256",
    "read_stl",
    "repair_scan_mesh",
    "run_scan_qa",
    "weld_and_clean",
    "write_binary_stl",
]
