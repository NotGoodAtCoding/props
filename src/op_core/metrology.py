"""Pure-Python 3D ICP scan registration, signed deviation heatmaps, volume tracking, and optional ampscan bridge."""

from __future__ import annotations

import importlib.util
import math
from collections import defaultdict
from dataclasses import dataclass, field

from .math3d import Vec3, jacobi_eigen_sym, mat3_mul_vec, v_add, v_dot, v_sub
from .mesh import TriMesh


@dataclass
class ScanComparisonResult:
    """Quantitative comparison between a baseline scan and an aligned follow-up/rectified scan."""

    aligned_followup: TriMesh
    rmse_mm: float
    mean_signed_dev_mm: float
    max_abs_dev_mm: float
    baseline_volume_ml: float
    followup_volume_ml: float
    volume_change_pct: float
    per_vertex_dev_mm: list[float] = field(default_factory=list)
    icp_iterations: int = 0


def has_ampscan() -> bool:
    """Return True if optional `ampscan` package is installed."""
    return importlib.util.find_spec("ampscan") is not None


def _quat_to_mat3(q: list[float]) -> tuple[Vec3, Vec3, Vec3]:
    w, x, y, z = q
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)),
        (2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)),
        (2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)),
    )


def horn_rigid_transform(src_pts: list[Vec3], dst_pts: list[Vec3]) -> tuple[tuple[Vec3, Vec3, Vec3], Vec3]:
    """Compute exact least-squares rigid transform (R, t) mapping `src_pts` onto `dst_pts` via Horn's quaternion method."""
    n = len(src_pts)
    if n == 0 or n != len(dst_pts):
        raise ValueError("src_pts and dst_pts must be non-empty and equal length")

    cs = (sum(p[0] for p in src_pts) / n, sum(p[1] for p in src_pts) / n, sum(p[2] for p in src_pts) / n)
    cd = (sum(p[0] for p in dst_pts) / n, sum(p[1] for p in dst_pts) / n, sum(p[2] for p in dst_pts) / n)

    sxx = sxy = sxz = syx = syy = syz = szx = szy = szz = 0.0
    for p, q in zip(src_pts, dst_pts):
        px, py, pz = p[0] - cs[0], p[1] - cs[1], p[2] - cs[2]
        qx, qy, qz = q[0] - cd[0], q[1] - cd[1], q[2] - cd[2]
        sxx += px * qx; sxy += px * qy; sxz += px * qz
        syx += py * qx; syy += py * qy; syz += py * qz
        szx += pz * qx; szy += pz * qy; szz += pz * qz

    n_mat = [
        [sxx + syy + szz, syz - szy, szx - sxz, sxy - syx],
        [syz - szy, sxx - syy - szz, sxy + syx, szx + sxz],
        [szx - sxz, sxy + syx, -sxx + syy - szz, syz + szy],
        [sxy - syx, szx + sxz, syz + szy, -sxx - syy + szz],
    ]
    _, eigvecs = jacobi_eigen_sym(n_mat)
    rot = _quat_to_mat3(eigvecs[0])
    rc = mat3_mul_vec(rot, cs)
    t = v_sub(cd, rc)
    return rot, t


def _build_spatial_hash(points: list[Vec3], cell_mm: float = 8.0) -> dict[tuple[int, int, int], list[int]]:
    grid: dict[tuple[int, int, int], list[int]] = defaultdict(list)
    inv = 1.0 / cell_mm
    for i, (x, y, z) in enumerate(points):
        grid[(math.floor(x * inv), math.floor(y * inv), math.floor(z * inv))].append(i)
    return grid


def _nearest_index(pt: Vec3, target: list[Vec3], grid: dict[tuple[int, int, int], list[int]], cell_mm: float = 8.0) -> int:
    inv = 1.0 / cell_mm
    gx, gy, gz = math.floor(pt[0] * inv), math.floor(pt[1] * inv), math.floor(pt[2] * inv)
    best_i = -1
    best_d2 = math.inf
    for r in (1, 2):
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                for dz in range(-r, r + 1):
                    if r == 2 and max(abs(dx), abs(dy), abs(dz)) < 2:
                        continue
                    for j in grid.get((gx + dx, gy + dy, gz + dz), ()):
                        q = target[j]
                        d2 = (pt[0] - q[0]) ** 2 + (pt[1] - q[1]) ** 2 + (pt[2] - q[2]) ** 2
                        if d2 < best_d2:
                            best_d2 = d2
                            best_i = j
        if best_i != -1:
            return best_i
    # Rare global fallback if meshes start far apart
    for j, q in enumerate(target):
        d2 = (pt[0] - q[0]) ** 2 + (pt[1] - q[1]) ** 2 + (pt[2] - q[2]) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_i = j
    return best_i


def icp_align_scans(
    source_mesh: TriMesh,
    target_mesh: TriMesh,
    max_iters: int = 20,
    sample_count: int = 320,
    tol_mm: float = 1e-3,
) -> tuple[TriMesh, int]:
    """Rigidly align `source_mesh` onto `target_mesh` using point-to-point ICP."""
    if not source_mesh.vertices or not target_mesh.vertices:
        return source_mesh.copy(), 0

    src_verts = list(source_mesh.vertices)
    dst_verts = target_mesh.vertices
    grid = _build_spatial_hash(dst_verts, cell_mm=8.0)

    # Initial centroid shift so far-apart scans enter spatial hash radius cleanly
    cs = source_mesh.centroid()
    cd = target_mesh.centroid()
    shift = v_sub(cd, cs)
    cur_verts = [v_add(v, shift) for v in src_verts]

    step = max(1, len(cur_verts) // sample_count)
    sample_indices = list(range(0, len(cur_verts), step))
    prev_rmse = math.inf
    iters_done = 0

    for it in range(1, max_iters + 1):
        iters_done = it
        src_sample = [cur_verts[i] for i in sample_indices]
        dst_indices = [_nearest_index(p, dst_verts, grid, cell_mm=8.0) for p in src_sample]
        dst_sample = [dst_verts[j] for j in dst_indices]

        # Reject top 15% outlier distances so local atrophy/relief does not pull rigid fit
        dists = [
            ((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 + (p[2] - q[2]) ** 2, idx)
            for idx, (p, q) in enumerate(zip(src_sample, dst_sample))
        ]
        dists.sort(key=lambda x: x[0])
        keep_n = max(12, int(0.85 * len(dists)))
        inlier_src = [src_sample[idx] for _, idx in dists[:keep_n]]
        inlier_dst = [dst_sample[idx] for _, idx in dists[:keep_n]]

        rot, t = horn_rigid_transform(inlier_src, inlier_dst)
        cur_verts = [v_add(mat3_mul_vec(rot, v), t) for v in cur_verts]
        rmse = math.sqrt(sum(d for d, _ in dists[:keep_n]) / keep_n)
        if abs(prev_rmse - rmse) < tol_mm:
            break
        prev_rmse = rmse

    return TriMesh(vertices=cur_verts, faces=list(source_mesh.faces)), iters_done


def _slice_volume_ml(mesh: TriMesh, z_min: float, z_max: float, step_mm: float = 10.0) -> float:
    """Integrate cross-sectional convex-hull areas over [z_min, z_max] into milliliters (cm^3)."""
    total_mm3 = 0.0
    z = z_min + 0.5 * step_mm
    while z < z_max:
        sl = mesh.slice_z(z)
        if sl["points"] >= 6:
            total_mm3 += sl["hull_area_mm2"] * step_mm
        z += step_mm
    return total_mm3 / 1000.0


def compare_scans(
    baseline_mesh: TriMesh,
    followup_mesh: TriMesh,
    align_icp: bool = True,
) -> ScanComparisonResult:
    """Align follow-up scan onto baseline and compute signed deviation & limb volume change."""
    if align_icp:
        aligned, iters = icp_align_scans(followup_mesh, baseline_mesh)
    else:
        aligned, iters = followup_mesh.copy(), 0

    b_norms = baseline_mesh.vertex_normals()
    a_grid = _build_spatial_hash(aligned.vertices, cell_mm=8.0)

    devs: list[float] = []
    for i, bv in enumerate(baseline_mesh.vertices):
        j = _nearest_index(bv, aligned.vertices, a_grid, cell_mm=8.0)
        av = aligned.vertices[j]
        signed_d = v_dot(v_sub(av, bv), b_norms[i])
        devs.append(signed_d)

    n = max(1, len(devs))
    rmse = math.sqrt(sum(d * d for d in devs) / n)
    mean_dev = sum(devs) / n
    max_abs = max((abs(d) for d in devs), default=0.0)

    b0, b1 = baseline_mesh.bounds()
    f0, f1 = aligned.bounds()
    z_lo = max(b0[2], f0[2])
    z_hi = min(b1[2], f1[2])
    vol_base = _slice_volume_ml(baseline_mesh, z_lo, z_hi)
    vol_fol = _slice_volume_ml(aligned, z_lo, z_hi)
    vol_pct = ((vol_fol - vol_base) / max(1e-6, vol_base)) * 100.0

    return ScanComparisonResult(
        aligned_followup=aligned,
        rmse_mm=rmse,
        mean_signed_dev_mm=mean_dev,
        max_abs_dev_mm=max_abs,
        baseline_volume_ml=vol_base,
        followup_volume_ml=vol_fol,
        volume_change_pct=vol_pct,
        per_vertex_dev_mm=devs,
        icp_iterations=iters,
    )
