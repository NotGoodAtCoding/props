"""Project 1: Chairside mobile scan QA, artifact triage, canonical alignment, and landmark-anchored tape check."""

from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import dataclass, field

from .math3d import (
    Vec3,
    jacobi_eigen_3x3,
    polygon_perimeter_2d,
    convex_hull_2d,
    v_dot,
    v_norm,
    v_sub,
    v_unit,
)
from .mesh import TriMesh


@dataclass
class ScanQAReport:
    status: str  # "PASS" | "WARN" | "FAIL"
    floater_components_removed: int
    proximal_cuff_loops: int
    internal_tear_loops: int
    double_wall_vertices: int
    seam_step_score_mm: float
    detected_scale_ratio: float | None
    warnings: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)
    slices: list[dict[str, float]] = field(default_factory=list)


def prune_floaters(mesh: TriMesh, distance_margin_mm: float = 10.0) -> tuple[TriMesh, int]:
    """Remove detached floater components outside `distance_margin_mm` of the main mesh bbox.

    Keeps near-surface overlapping patches (e.g. SLAM double walls) intact so `detect_double_walls`
    can flag them explicitly instead of silently hiding tracking corruption.
    """
    comps = mesh.connected_components()
    if len(comps) <= 1:
        return mesh.copy(), 0

    main_verts = [mesh.vertices[i] for i in comps[0]]
    mx0 = min(v[0] for v in main_verts) - distance_margin_mm
    mx1 = max(v[0] for v in main_verts) + distance_margin_mm
    my0 = min(v[1] for v in main_verts) - distance_margin_mm
    my1 = max(v[1] for v in main_verts) + distance_margin_mm
    mz0 = min(v[2] for v in main_verts) - distance_margin_mm
    mz1 = max(v[2] for v in main_verts) + distance_margin_mm

    keep: list[int] = list(comps[0])
    removed = 0
    for comp in comps[1:]:
        cx = sum(mesh.vertices[i][0] for i in comp) / len(comp)
        cy = sum(mesh.vertices[i][1] for i in comp) / len(comp)
        cz = sum(mesh.vertices[i][2] for i in comp) / len(comp)
        inside_main = (mx0 <= cx <= mx1) and (my0 <= cy <= my1) and (mz0 <= cz <= mz1)
        if inside_main and len(comp) >= 20:
            keep.extend(comp)
        else:
            removed += 1
    return mesh.submesh_from_vertices(keep), removed


def classify_boundary_loops(mesh: TriMesh) -> tuple[list[list[int]], list[list[int]]]:
    """Classify boundary loops into intended proximal open cuff(s) vs. internal scan tears."""
    loops = mesh.boundary_loops()
    if not loops:
        return [], []

    zmin, zmax = mesh.bounds()[0][2], mesh.bounds()[1][2]
    z_span = max(1e-6, zmax - zmin)

    scored = []
    for lp in loops:
        mean_z = sum(mesh.vertices[i][2] for i in lp) / len(lp)
        perim = sum(
            v_norm(v_sub(mesh.vertices[lp[(k + 1) % len(lp)]], mesh.vertices[lp[k]]))
            for k in range(len(lp))
        )
        scored.append((lp, mean_z, perim))

    # Largest loop in top 25% of Z span is the intended proximal open cuff
    cuffs: list[list[int]] = []
    tears: list[list[int]] = []
    top_threshold = zmin + 0.75 * z_span
    scored.sort(key=lambda item: item[2], reverse=True)
    cuff_assigned = False
    for lp, mean_z, _ in scored:
        if not cuff_assigned and mean_z >= top_threshold:
            cuffs.append(lp)
            cuff_assigned = True
        else:
            tears.append(lp)
    return cuffs, tears


def detect_double_walls(mesh: TriMesh, distance_threshold_mm: float = 1.4) -> int:
    """Detect non-adjacent surface patches stacked within `distance_threshold_mm` (SLAM double walls)."""
    adj: list[set[int]] = [set() for _ in mesh.vertices]
    for a, b, c in mesh.faces:
        adj[a].update((b, c))
        adj[b].update((a, c))
        adj[c].update((a, b))

    # Build 2-ring neighborhood exclusion so legitimate tight mesh edges are not flagged
    near_ring: list[set[int]] = [set(s) | {i} for i, s in enumerate(adj)]
    for i, s in enumerate(adj):
        for n1 in s:
            near_ring[i].update(adj[n1])

    cell = max(2.0, distance_threshold_mm * 1.5)
    grid: dict[tuple[int, int, int], list[int]] = defaultdict(list)
    for i, (x, y, z) in enumerate(mesh.vertices):
        grid[(math.floor(x / cell), math.floor(y / cell), math.floor(z / cell))].append(i)

    d2_thresh = distance_threshold_mm * distance_threshold_mm
    flagged: set[int] = set()
    verts = mesh.vertices
    for i, (x, y, z) in enumerate(verts):
        gx, gy, gz = math.floor(x / cell), math.floor(y / cell), math.floor(z / cell)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    for j in grid.get((gx + dx, gy + dy, gz + dz), ()):
                        if j <= i or j in near_ring[i]:
                            continue
                        px, py, pz = verts[j]
                        d2 = (x - px) ** 2 + (y - py) ** 2 + (z - pz) ** 2
                        if d2 < d2_thresh:
                            flagged.add(i)
                            flagged.add(j)
    return len(flagged)


def detect_seam_step(mesh: TriMesh, dihedral_threshold_deg: float = 42.0) -> float:
    """Measure anomalous sharp dihedral-angle ridges indicating loop-closure seam drift."""
    fn = mesh.face_normals()
    zmin = mesh.bounds()[0][2]
    verts = mesh.vertices
    edges: dict[tuple[int, int], int] = {}
    max_excess = 0.0
    ridge_count = 0
    for fi, (a, b, c) in enumerate(mesh.faces):
        for u, v in ((a, b), (b, c), (c, a)):
            key = (u, v) if u < v else (v, u)
            if key not in edges:
                edges[key] = fi
            else:
                # Ignore extreme distal apex cap triangles where radial convergence raises dihedral angle
                if min(verts[u][2], verts[v][2]) < zmin + 12.0:
                    continue
                fj = edges[key]
                dot_val = max(-1.0, min(1.0, v_dot(fn[fi], fn[fj])))
                ang = math.degrees(math.acos(dot_val))
                if ang > dihedral_threshold_deg:
                    ridge_count += 1
                    max_excess = max(max_excess, (ang - dihedral_threshold_deg) * 0.25)
    return max_excess if ridge_count >= 3 else 0.0


def align_canonical_limb(
    mesh: TriMesh,
    mpt_point: Vec3 | None = None,
    distal_point: Vec3 | None = None,
) -> TriMesh:
    """Align limb to canonical frame (+Z proximal, +Y anterior, z=0 at MPT plane).

    Longitudinal axis `z_axis` is estimated from PCA so anterior surface landmarks never tilt slices;
    `mpt_point` + `distal_point` disambiguate +Z proximal, orient +Y anterior, and anchor z=0.
    """
    verts = mesh.vertices
    n = len(verts)
    if n == 0:
        return mesh.copy()

    cx = sum(v[0] for v in verts) / n
    cy = sum(v[1] for v in verts) / n
    cz = sum(v[2] for v in verts) / n

    cov = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    for x, y, z in verts:
        dx, dy, dz = x - cx, y - cy, z - cz
        cov[0][0] += dx * dx
        cov[0][1] += dx * dy
        cov[0][2] += dx * dz
        cov[1][1] += dy * dy
        cov[1][2] += dy * dz
        cov[2][2] += dz * dz
    cov[1][0] = cov[0][1]
    cov[2][0] = cov[0][2]
    cov[2][1] = cov[1][2]
    _, eigvecs = jacobi_eigen_3x3(cov)
    z_axis = eigvecs[0]

    # Refine longitudinal axis via cross-sectional centroid line over central shaft (25%-85%)
    # so asymmetric posterior calf bulk does not tilt the slice planes.
    for _ in range(2):
        projs_s = [v_dot(v_sub(v, (cx, cy, cz)), z_axis) for v in verts]
        p_min, p_max = min(projs_s), max(projs_s)
        span = max(1e-6, p_max - p_min)
        low_c = [0.0, 0.0, 0.0]
        high_c = [0.0, 0.0, 0.0]
        n_low = n_high = 0
        for p, s in zip(verts, projs_s):
            t = (s - p_min) / span
            if 0.25 <= t <= 0.45:
                low_c[0] += p[0]; low_c[1] += p[1]; low_c[2] += p[2]; n_low += 1
            elif 0.70 <= t <= 0.90:
                high_c[0] += p[0]; high_c[1] += p[1]; high_c[2] += p[2]; n_high += 1
        if n_low >= 10 and n_high >= 10:
            p_lo = (low_c[0] / n_low, low_c[1] / n_low, low_c[2] / n_low)
            p_hi = (high_c[0] / n_high, high_c[1] / n_high, high_c[2] / n_high)
            cand = v_unit(v_sub(p_hi, p_lo), fallback=z_axis)
            if v_dot(cand, z_axis) < 0.0:
                cand = (-cand[0], -cand[1], -cand[2])
            z_axis = cand

    if mpt_point is not None and distal_point is not None:
        if v_dot(v_sub(mpt_point, distal_point), z_axis) < 0.0:
            z_axis = (-z_axis[0], -z_axis[1], -z_axis[2])
        origin = mpt_point
    else:
        projs = sorted((v_dot(v_sub(v, (cx, cy, cz)), z_axis), v) for v in verts)
        q_n = max(10, n // 6)
        low_pts = [p[1] for p in projs[:q_n]]
        high_pts = [p[1] for p in projs[-q_n:]]

        def radial_rms(pts: list[Vec3]) -> float:
            mc = (
                sum(p[0] for p in pts) / len(pts),
                sum(p[1] for p in pts) / len(pts),
                sum(p[2] for p in pts) / len(pts),
            )
            return sum(v_dot(v_sub(p, mc), v_sub(p, mc)) for p in pts) / len(pts)

        if radial_rms(low_pts) > radial_rms(high_pts):
            z_axis = (-z_axis[0], -z_axis[1], -z_axis[2])
        origin = (cx, cy, cz)

    from .math3d import v_cross

    if mpt_point is not None:
        # Project (mpt_point - centroid) perpendicular to z_axis to define +Y anterior
        d_mpt = v_sub(mpt_point, (cx, cy, cz))
        proj_z = v_dot(d_mpt, z_axis)
        y_cand = v_sub(d_mpt, (z_axis[0] * proj_z, z_axis[1] * proj_z, z_axis[2] * proj_z))
        if v_norm(y_cand) > 1e-3:
            y_axis = v_unit(y_cand)
        else:
            ref_x = (1.0, 0.0, 0.0) if abs(z_axis[0]) < 0.85 else (0.0, 1.0, 0.0)
            y_axis = v_unit(v_cross(z_axis, ref_x))
    else:
        ref_x = (1.0, 0.0, 0.0) if abs(z_axis[0]) < 0.85 else (0.0, 1.0, 0.0)
        y_axis = v_unit(v_cross(z_axis, ref_x))

    x_axis = v_unit(v_cross(y_axis, z_axis))

    def to_canonical(p: Vec3) -> Vec3:
        d = v_sub(p, origin)
        return (v_dot(d, x_axis), v_dot(d, y_axis), v_dot(d, z_axis))

    aligned = mesh.transformed(to_canonical)
    if mpt_point is None:
        zmin, zmax = aligned.bounds()[0][2], aligned.bounds()[1][2]
        z_mpt_est = zmin + 0.82 * (zmax - zmin)
        aligned = aligned.transformed(lambda p: (p[0], p[1], p[2] - z_mpt_est))
    return aligned


def measure_circumferences(
    mesh: TriMesh,
    anchor_z_mm: float = 0.0,
    step_mm: float = 20.0,
    max_depth_mm: float = 160.0,
) -> list[dict[str, float]]:
    """Measure taut-tape (convex hull) circumferences downward from an anatomical anchor plane."""
    zmin = mesh.bounds()[0][2]
    results: list[dict[str, float]] = []
    depth = 0.0
    while depth <= max_depth_mm:
        z = anchor_z_mm - depth
        if z <= zmin + 2.0:
            break
        sl = mesh.slice_z(z)
        if sl["points"] >= 6:
            sl["depth_from_anchor_mm"] = depth
            results.append(sl)
        depth += step_mm
    return results


def run_scan_qa(
    raw_mesh: TriMesh,
    mpt_point: Vec3 | None = None,
    distal_point: Vec3 | None = None,
    clinical_tape_mm: dict[float, float] | None = None,
    calibration_measured_mm: float | None = None,
    calibration_expected_mm: float = 100.0,
) -> tuple[TriMesh, ScanQAReport]:
    """Execute full chairside Scan QA pipeline and return (clean_aligned_mesh, report)."""
    warnings: list[str] = []
    errors: list[str] = []

    clean, removed_floaters = prune_floaters(raw_mesh)
    if removed_floaters > 0:
        warnings.append(f"Pruned {removed_floaters} disconnected floater component(s)")

    aligned = align_canonical_limb(clean, mpt_point=mpt_point, distal_point=distal_point)
    cuffs, tears = classify_boundary_loops(aligned)
    if tears:
        errors.append(f"Detected {len(tears)} internal surface tear hole(s)")

    double_wall_verts = detect_double_walls(aligned)
    if double_wall_verts > 15:
        errors.append(f"Detected SLAM double-walled overlap ({double_wall_verts} vertices)")

    seam_score = detect_seam_step(aligned)
    if seam_score > 1.0:
        errors.append(f"Detected potential loop-closure seam step ({seam_score:.2f} mm excess)")

    scale_ratio: float | None = None
    if calibration_measured_mm is not None and calibration_expected_mm > 0.0:
        scale_ratio = calibration_measured_mm / calibration_expected_mm
        if abs(scale_ratio - 1.0) > 0.02:
            errors.append(
                f"Calibration scale drift {(scale_ratio - 1.0) * 100.0:+.2f}% exceeds ±2.0% threshold"
            )

    slices = measure_circumferences(aligned, anchor_z_mm=0.0, step_mm=20.0)
    if clinical_tape_mm:
        by_depth = {round(s["depth_from_anchor_mm"], 1): s["taut_circumference_mm"] for s in slices}
        for depth, tape_val in clinical_tape_mm.items():
            scan_val = by_depth.get(round(depth, 1))
            if scan_val is not None and abs(scan_val - tape_val) > 2.5:
                warnings.append(
                    f"Tape mismatch at -{depth:.0f} mm: scan={scan_val:.1f} mm vs tape={tape_val:.1f} mm"
                )

    status = "FAIL" if errors else ("WARN" if warnings else "PASS")
    return aligned, ScanQAReport(
        status=status,
        floater_components_removed=removed_floaters,
        proximal_cuff_loops=len(cuffs),
        internal_tear_loops=len(tears),
        double_wall_vertices=double_wall_verts,
        seam_step_score_mm=seam_score,
        detected_scale_ratio=scale_ratio,
        warnings=warnings,
        errors=errors,
        slices=slices,
    )
