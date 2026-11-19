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
    """Pre-print structural and geometric safety check results."""

    passed: bool
    is_watertight_manifold: bool
    min_estimated_wall_mm: float
    trimline_min_radius_mm: float
    load_to_print_z_angle_deg: float
    failures: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)


def estimate_min_wall_thickness_mm(shell_mesh: TriMesh) -> float:
    """Estimate minimum opposing-wall thickness using a spatial hash over opposing normal pairs."""
    from collections import defaultdict

    verts = shell_mesh.vertices
    norms = shell_mesh.vertex_normals()
    if not verts:
        return 0.0

    cell = 8.0
    grid: dict[tuple[int, int, int], list[int]] = defaultdict(list)
    for idx, (x, y, z) in enumerate(verts):
        grid[(math.floor(x / cell), math.floor(y / cell), math.floor(z / cell))].append(idx)

    step = max(1, len(verts) // 400)
    min_thick = math.inf
    for i in range(0, len(verts), step):
        vi = verts[i]
        ni = norms[i]
        gx, gy, gz = math.floor(vi[0] / cell), math.floor(vi[1] / cell), math.floor(vi[2] / cell)
        best = math.inf
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    for j in grid.get((gx + dx, gy + dy, gz + dz), ()):
                        if i == j:
                            continue
                        d_vec = v_sub(verts[j], vi)
                        dist = v_norm(d_vec)
                        if 0.6 < dist < 12.0:
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


def _pdf_escape(text: str) -> str:
    return text.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")


def export_fabrication_dossier_pdf(
    recipe: OpRecipe,
    qa_report: ScanQAReport,
    lint_report: LintReport,
    output_path: str | Path,
) -> Path:
    """Write a standalone pure-Python PDF 1.4 custom-device fabrication dossier (no external deps)."""
    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)

    lines: list[tuple[str, int, str]] = [
        ("F2", 14, f"Custom O&P Device Fabrication Dossier ({recipe.device_type})"),
        ("F1", 9, f"Schema: v{recipe.schema_version} | Kernel: op-core {recipe.kernel_version} | Side: {recipe.side} | Units: {recipe.units}"),
        ("F1", 9, f"Scan SHA-256: {(recipe.scan_sha256 or 'N/A')[:48]}"),
        ("F2", 11, "1. Quality Assurance & Pre-Print Safety Gate"),
        ("F1", 9, f"Scan QA Status: {qa_report.status} (Floaters={qa_report.floater_components_removed}, Tears={qa_report.internal_tear_loops}, DoubleWall={qa_report.double_wall_vertices})"),
        ("F1", 9, f"Safety Lint: {'PASS' if lint_report.passed else 'FAIL'} | Watertight={lint_report.is_watertight_manifold} | MinWall={lint_report.min_estimated_wall_mm:.2f} mm | TrimR={lint_report.trimline_min_radius_mm:.1f} mm"),
        ("F2", 11, "2. Landmark-Anchored Circumferences (20 mm increments)"),
        ("F1", 9, "Depth (mm)      Taut Circumference (mm)      Cross-Section Area (mm^2)"),
    ]
    for sl in qa_report.slices[:10]:
        lines.append((
            "F1",
            9,
            f"  -{sl['depth_from_anchor_mm']:5.0f}            {sl['taut_circumference_mm']:8.1f}                    {sl['hull_area_mm2']:9.0f}",
        ))
    lines.append(("F2", 11, "3. Parametric Rectification Operations (.oprecipe.json)"))
    lines.append(("F1", 9, f"Liner Offset: {recipe.liner_thickness_mm:.1f} mm | Alignment: {json.dumps(recipe.alignment)}"))
    for op in recipe.operations[:10]:
        lines.append(("F1", 8, f"  - {json.dumps(op)[:92]}"))
    lines.extend([
        ("F2", 11, "4. Regulatory Traceability (EU MDR Annex XIII / FDA QMSR)"),
        ("F1", 9, "Patient Case ID: _______________   Prescriber NPI/ID: _______________"),
        ("F1", 9, "Material Lot #:  _______________   Component Serial #: ______________"),
    ])

    stream_cmds = ["BT"]
    y = 758
    for font_key, size, txt in lines:
        if y < 48:
            break
        stream_cmds.append(f"/{font_key} {size} Tf")
        stream_cmds.append(f"1 0 0 1 44 {y} Tm ({_pdf_escape(txt)}) Tj")
        y -= (size + 6)
    stream_cmds.append("ET")
    stream_bytes = ("\n".join(stream_cmds) + "\n").encode("latin-1", errors="replace")

    objs: list[bytes] = [
        b"1 0 obj\n<< /Type /Catalog /Pages 2 0 R >>\nendobj\n",
        b"2 0 obj\n<< /Type /Pages /Kids [3 0 R] /Count 1 >>\nendobj\n",
        b"3 0 obj\n<< /Type /Page /Parent 2 0 R /MediaBox [0 0 612 792] /Resources << /Font << /F1 4 0 R /F2 5 0 R >> >> /Contents 6 0 R >>\nendobj\n",
        b"4 0 obj\n<< /Type /Font /Subtype /Type1 /BaseFont /Courier >>\nendobj\n",
        b"5 0 obj\n<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica-Bold >>\nendobj\n",
        f"6 0 obj\n<< /Length {len(stream_bytes)} >>\nstream\n".encode("ascii") + stream_bytes + b"endstream\nendobj\n",
    ]

    pdf = bytearray(b"%PDF-1.4\n")
    offsets: list[int] = [0]
    for obj in objs:
        offsets.append(len(pdf))
        pdf.extend(obj)
    xref_start = len(pdf)
    pdf.extend(f"xref\n0 {len(objs) + 1}\n0000000000 65535 f \n".encode("ascii"))
    for off in offsets[1:]:
        pdf.extend(f"{off:010d} 00000 n \n".encode("ascii"))
    pdf.extend(
        f"trailer\n<< /Size {len(objs) + 1} /Root 1 0 R >>\nstartxref\n{xref_start}\n%%EOF\n".encode("ascii")
    )
    out.write_bytes(bytes(pdf))
    return out


def export_fabrication_dossier(
    recipe: OpRecipe,
    qa_report: ScanQAReport,
    lint_report: LintReport,
    output_dir: str | Path,
    basename: str = "device_dossier",
) -> tuple[Path, Path]:
    """Write archival JSON, Markdown, and PDF custom-device fabrication dossier (MDR Annex XIII / QMSR trace)."""
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
    export_fabrication_dossier_pdf(recipe, qa_report, lint_report, out / f"{basename}.pdf")
    return json_path, md_path
