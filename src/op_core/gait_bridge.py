"""OpenCap / OpenSim `.mot` kinematics bridge for O&P alignment and ISO 10328 load sizing."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class GaitTable:
    """Parsed OpenSim `.mot` or `.sto` time-series table."""

    columns: list[str]
    rows: list[list[float]]
    in_degrees: bool = True

    def column(self, name: str) -> list[float]:
        idx = self.columns.index(name)
        return [r[idx] for r in self.rows]

    def find_column(self, candidates: tuple[str, ...]) -> str | None:
        lower_map = {c.lower(): c for c in self.columns}
        for cand in candidates:
            if cand.lower() in lower_map:
                return lower_map[cand.lower()]
        for cand in candidates:
            for col in self.columns:
                if cand.lower() in col.lower():
                    return col
        return None


@dataclass
class GaitClinicalSummary:
    """Actionable O&P gait symmetry, foot-drop, and ISO 10328 load-sizing metrics."""

    affected_side: str
    duration_s: float
    sample_count: int
    peak_knee_flexion_affected_deg: float
    peak_knee_flexion_contralateral_deg: float
    knee_flexion_asymmetry_pct: float
    min_ankle_angle_affected_deg: float
    peak_ankle_dorsiflexion_affected_deg: float
    peak_vertical_grf_bw: float
    recommended_iso10328_force_n: float
    recommendations: list[str] = field(default_factory=list)


def parse_opensim_mot(path: str | Path) -> GaitTable:
    """Parse an OpenSim / OpenCap `.mot` or `.sto` kinematics/kinetics file."""
    text = Path(path).read_text(encoding="utf-8", errors="ignore")
    lines = text.splitlines()
    in_degrees = True
    header_end_idx = -1

    for i, line in enumerate(lines):
        low = line.strip().lower()
        if low.startswith("indegrees="):
            in_degrees = low.split("=", 1)[1].strip() in ("yes", "true", "1")
        if low == "endheader":
            header_end_idx = i
            break

    if header_end_idx == -1:
        raise ValueError("Invalid .mot/.sto file: missing 'endheader'")

    col_idx = header_end_idx + 1
    while col_idx < len(lines) and not lines[col_idx].strip():
        col_idx += 1
    if col_idx >= len(lines):
        raise ValueError("Missing column header row after endheader")

    columns = lines[col_idx].strip().split()
    rows: list[list[float]] = []
    for raw in lines[col_idx + 1 :]:
        s = raw.strip()
        if not s:
            continue
        vals = [float(tok) for tok in s.split()]
        if len(vals) == len(columns):
            rows.append(vals)

    if not rows:
        raise ValueError("No numeric data rows found in .mot/.sto file")
    return GaitTable(columns=columns, rows=rows, in_degrees=in_degrees)


def analyze_gait_mot(
    path: str | Path,
    affected_side: str = "left",
    body_mass_kg: float = 80.0,
) -> GaitClinicalSummary:
    """Extract O&P alignment diagnostics and ISO 10328 peak load sizing from OpenCap/OpenSim `.mot`."""
    if affected_side not in ("left", "right"):
        raise ValueError("affected_side must be 'left' or 'right'")

    table = parse_opensim_mot(path)
    aff_s = "l" if affected_side == "left" else "r"
    con_s = "r" if affected_side == "left" else "l"
    deg_scale = 1.0 if table.in_degrees else (180.0 / math.pi)

    time_col = table.find_column(("time",))
    times = table.column(time_col) if time_col else [float(i) for i in range(len(table.rows))]
    duration = max(0.0, times[-1] - times[0])

    def _series(cands: tuple[str, ...]) -> list[float]:
        name = table.find_column(cands)
        if not name:
            return []
        return [v * deg_scale for v in table.column(name)]

    knee_aff = _series((f"knee_angle_{aff_s}", f"knee_flexion_{aff_s}"))
    knee_con = _series((f"knee_angle_{con_s}", f"knee_flexion_{con_s}"))
    ankle_aff = _series((f"ankle_angle_{aff_s}", f"ankle_dorsiflexion_{aff_s}"))

    pk_knee_aff = max(knee_aff) if knee_aff else 0.0
    pk_knee_con = max(knee_con) if knee_con else pk_knee_aff
    mean_k = max(1.0, 0.5 * (abs(pk_knee_aff) + abs(pk_knee_con)))
    asym = abs(pk_knee_aff - pk_knee_con) / mean_k * 100.0

    min_ank = min(ankle_aff) if ankle_aff else 0.0
    max_ank = max(ankle_aff) if ankle_aff else 0.0

    grf_col = table.find_column((
        f"ground_force_{aff_s}_vy",
        f"grf_{aff_s}_y",
        "ground_force_vy",
        "1_ground_force_vy",
    ))
    if grf_col:
        raw_grf = table.column(grf_col)
        peak_n = max(raw_grf)
        bw_n = max(1.0, body_mass_kg * 9.80665)
        peak_bw = peak_n / bw_n if peak_n > 10.0 else peak_n
    else:
        peak_bw = 1.20

    # Scale ISO 10328 P5 (4025 N at 100 kg & 1.2 BW) proportionally to patient mass and measured GRF
    rec_force_n = max(body_mass_kg, 50.0) * (peak_bw / 1.20) * 40.25

    recs: list[str] = []
    if ankle_aff and min_ank < -8.0:
        recs.append(
            f"Swing/loading plantarflexion drop ({min_ank:.1f} deg): reinforce AFO ankle neck >= 4.5 mm and add +2 deg dorsiflexion bias."
        )
    if knee_aff and knee_con and (pk_knee_con - pk_knee_aff) > 8.0:
        recs.append(
            f"Reduced affected knee flexion ({pk_knee_aff:.1f} vs {pk_knee_con:.1f} deg, {asym:.1f}% asym): review socket flexion (+1 to +2 deg) or heel stiffness."
        )
    if asym > 15.0 and not recs:
        recs.append(f"Notable sagittal knee asymmetry ({asym:.1f}%): inspect prosthetic alignment and dynamic rollover.")
    if rec_force_n > 4025.0:
        recs.append(
            f"Dynamic load sizing ({rec_force_n:.0f} N) exceeds ISO 10328 P5 (4025 N): enforce >= 5.0 mm distal wall and 100% infill modifier."
        )
    if not recs:
        recs.append("Sagittal kinematics within nominal symmetry bounds; standard P5 (4025 N) sizing adequate.")

    return GaitClinicalSummary(
        affected_side=affected_side,
        duration_s=duration,
        sample_count=len(table.rows),
        peak_knee_flexion_affected_deg=pk_knee_aff,
        peak_knee_flexion_contralateral_deg=pk_knee_con,
        knee_flexion_asymmetry_pct=asym,
        min_ankle_angle_affected_deg=min_ank,
        peak_ankle_dorsiflexion_affected_deg=max_ank,
        peak_vertical_grf_bw=peak_bw,
        recommended_iso10328_force_n=rec_force_n,
        recommendations=recs,
    )
