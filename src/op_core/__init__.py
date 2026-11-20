"""Headless MIT computational geometry kernel for open-source O&P CAD/CAM."""

from .accel import BackendStatus, probe_backends
from .carver_cam import CarverCamResult, generate_4axis_gcode_and_simulate
from .export_3mf import export_3mf
from .gait_bridge import GaitClinicalSummary, GaitTable, analyze_gait_mot, parse_opensim_mot
from .linter import LintReport, export_fabrication_dossier, export_fabrication_dossier_pdf, lint_device
from .mesh import TriMesh, read_stl, write_binary_stl
from .metrology import ScanComparisonResult, compare_scans, has_ampscan, horn_rigid_transform, icp_align_scans
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
    build_tf_socket_shell,
    build_tt_socket_shell,
    export_carver_positive,
    has_manifold3d,
)

__all__ = [
    "BackendStatus",
    "CarverCamResult",
    "CarverPositiveResult",
    "GaitClinicalSummary",
    "GaitTable",
    "LintReport",
    "OpRecipe",
    "RectificationResult",
    "RepairReport",
    "ScanComparisonResult",
    "ScanQAReport",
    "ShellBuildResult",
    "TriMesh",
    "align_canonical_limb",
    "analyze_gait_mot",
    "apply_afo_posture",
    "apply_recipe",
    "build_afo_shell",
    "build_tf_socket_shell",
    "build_tt_socket_shell",
    "clip_mesh_z",
    "compare_scans",
    "diff_recipes",
    "export_3mf",
    "export_carver_positive",
    "export_fabrication_dossier",
    "export_fabrication_dossier_pdf",
    "fill_small_holes",
    "fix_consistent_winding",
    "generate_4axis_gcode_and_simulate",
    "has_ampscan",
    "has_manifold3d",
    "horn_rigid_transform",
    "icp_align_scans",
    "lint_device",
    "measure_circumferences",
    "mesh_sha256",
    "parse_opensim_mot",
    "probe_backends",
    "read_stl",
    "repair_scan_mesh",
    "run_scan_qa",
    "weld_and_clean",
    "write_binary_stl",
]
