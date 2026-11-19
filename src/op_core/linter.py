"""Project 4: Print-safety geometry linter & custom-device fabrication dossier generator."""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from .math3d import Vec3, v_dot, v_norm, v_sub, v_unit
from .mesh import TriMesh
from .recipe import OpRecipe
from .scan_qa import ScanQAReport
from .shell import compute_curve_min_radius


@dataclass
class LintReport:
    passed: bool
    is_watertight_manifold: bool
    min_estimated_wall_mm: float
    trimline_min_radius_mm: float
    load_to_print_z_angle_deg: float
    failures: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)


def estimate_min_wall_thickness_mm(shell_mesh: TriMesh) -> float:
    """Estimate minimum opposing-wall thickness using opposing-normal spatial pairs."""
    verts = shell_mesh.vertices
    norms = shell_mesh.vertex_normals()
    # Sample evenly across vertices for fast O(N) bound check
    step = max(1, len(verts) // 400)
    min_thick = math.inf
    for i in range(0, len(verts), step):
        vi = verts[i]
        ni = norms[i]
        # Skip flat bottom boss / pocket vertices near extreme zmin
        best = math.inf
        for j in range(0, len(verts), step):
            if i == j:
                continue
            d_vec = v_sub(verts[j], vi)
            dist = v_norm(d_vec)
            if 0.6 < dist < 12.0:
                # Opposing wall: normal points opposite and vector aligns with inward normal
                if v_dot(ni, norms[j]) < -0.75 and v_dot(v_unit(d_vec), ni) < -0.75:
                    if dist < best:
                        best = dist
        if best < min_thick:
            min_thick = best
    return 0.0 if min_thick is math.inf else min_thick


def lint_device(
    shell_mesh: TriMesh,
    min_wall_mm: float = 2.5,
    min_trimline_radius_mm: float = 8.0,
    trimline_points: list[Vec3] | None = None,
    load_axis: Vec3 = (0.0, 0.0, 1.0),
    print_z_axis: Vec3 = (0.0, 0.0, 1.0),
    measured_wall_override_mm: float | None = None,
) -> LintReport:
    """Run pre-print safety checks: manifoldness, min wall thickness, notch radius, print-Z angle."""
    failures: list[str] = []
    warnings: list[str] = []

    watertight = shell_mesh.is_watertight_manifold()
    if not watertight:
        failures.append("Shell mesh is not a closed 2-manifold (non-manifold or boundary edges present)")

    est_wall = (
        measured_wall_override_mm
        if measured_wall_override_mm is not None
        else estimate_min_wall_thickness_mm(shell_mesh)
    )
    if est_wall > 0.0 and est_wall < min_wall_mm:
        failures.append(f"Minimum wall thickness {est_wall:.2f} mm is below safe floor {min_wall_mm:.2f} mm")

    r_trim = compute_curve_min_radius(trimline_points, closed=True) if trimline_points else 15.0
    if r_trim < min_trimline_radius_mm:
        failures.append(
            f"Trimline notch radius {r_trim:.2f} mm is below {min_trimline_radius_mm:.2f} mm stress-riser limit"
        )

    dot_val = max(-1.0, min(1.0, abs(v_dot(v_unit(load_axis), v_unit(print_z_axis)))))
    angle_deg = math.degrees(math.acos(dot_val))
    if angle_deg > 25.0:
        warnings.append(
            f"Load-axis tilt ({angle_deg:.1f} deg) relative to print Z increases inter-layer shear risk"
        )

    return LintReport(
        passed=len(failures) == 0,
        is_watertight_manifold=watertight,
        min_estimated_wall_mm=est_wall,
        trimline_min_radius_mm=r_trim,
        load_to_print_z_angle_deg=angle_deg,
        failures=failures,
        warnings=warnings,
    )


def export_fabrication_dossier(
    recipe: OpRecipe,
    qa_report: ScanQAReport,
    lint_report: LintReport,
    output_dir: str | Path,
    basename: str = "device_dossier",
) -> tuple[Path, Path]:
    """Write archival JSON + Markdown custom-device fabrication dossier (MDR Annex XIII / QMSR trace)."""
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    bundle: dict[str, Any] = {
        "recipe": recipe.to_dict(),
        "scan_qa": asdict(qa_report),
        "safety_lint": asdict(lint_report),
        "regulatory_placeholders": {
            "patient_case_id": "UNASSIGNED",
            "prescriber_npi_or_id": "UNASSIGNED",
            "material_lot_number": "UNASSIGNED",
            "hardware_component_serial": "UNASSIGNED",
        },
    }
    json_path = out / f"{basename}.json"
    json_path.write_text(json.dumps(bundle, indent=2) + "\n", encoding="utf-8")

    md_lines = [
        f"# Custom O&P Device Fabrication Dossier ({recipe.device_type})",
        "",
        f"- **Schema / Kernel:** `{recipe.schema_version}` / `{recipe.kernel_version}`",
        f"- **Side / Units:** `{recipe.side}` / `{recipe.units}`",
        f"- **Scan SHA-256:** `{recipe.scan_sha256 or 'N/A'}`",
        f"- **Scan QA Status:** **{qa_report.status}**",
        f"- **Safety Lint Passed:** **{lint_report.passed}** (Watertight={lint_report.is_watertight_manifold}, MinWall={lint_report.min_estimated_wall_mm:.2f} mm)",
        "",
        "## Landmark-Anchored Circumferences (20 mm increments)",
        "| Depth from Anchor (mm) | Taut-Tape Circumference (mm) | Area (mm^2) |",
        "| ---: | ---: | ---: |",
    ]
    for sl in qa_report.slices:
        md_lines.append(
            f"| {sl['depth_from_anchor_mm']:.0f} | {sl['taut_circumference_mm']:.1f} | {sl['hull_area_mm2']:.0f} |"
        )
    md_lines.extend([
        "",
        "## Parametric Operations Log",
        "```json",
        json.dumps(recipe.operations, indent=2),
        "```",
    ])
    md_path = out / f"{basename}.md"
    md_path.write_text("\n".join(md_lines) + "\n", encoding="utf-8")
    return json_path, md_path
