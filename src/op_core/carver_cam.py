"""4-axis rotary CNC foam-carver G-code generator and ball-nose stock-removal simulator (EXP-5B)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

from .math3d import Vec3, v_cross, v_dot, v_sub
from .mesh import TriMesh


@dataclass
class CarverCamResult:
    """Generated 4-axis rotary G-code and ball-nose stock-removal verification metrics."""

    gcode: str
    line_count: int
    rms_error_mm: float
    max_gouge_mm: float
    max_excess_mm: float
    mandrel_collisions: int
    stock_radius_mm: float
    ball_radius_mm: float
    warnings: list[str] = field(default_factory=list)


def _ray_tri_x_dist(origin: Vec3, direction: Vec3, v0: Vec3, v1: Vec3, v2: Vec3) -> float | None:
    """Moller-Trumbore ray-triangle intersection returning positive distance t."""
    e1 = v_sub(v1, v0)
    e2 = v_sub(v2, v0)
    pvec = v_cross(direction, e2)
    det = v_dot(e1, pvec)
    if abs(det) < 1e-9:
        return None
    inv_det = 1.0 / det
    tvec = v_sub(origin, v0)
    u = v_dot(tvec, pvec) * inv_det
    if u < -1e-5 or u > 1.00001:
        return None
    qvec = v_cross(tvec, e1)
    v = v_dot(direction, qvec) * inv_det
    if v < -1e-5 or u + v > 1.00001:
        return None
    t = v_dot(e2, qvec) * inv_det
    return t if t > 1e-4 else None


def sample_cylindrical_heightmap(
    positive_mesh: TriMesh,
    z_step_mm: float = 2.5,
    n_theta: int = 60,
    center_xy: tuple[float, float] | None = None,
) -> tuple[list[float], list[float], list[list[float]], tuple[float, float]]:
    """Sample outer radial surface `R(z, theta)` via horizontal ray casting binned by Z."""
    bmin, bmax = positive_mesh.bounds()
    z0, z1 = bmin[2] + 1.0, bmax[2] - 1.0
    if center_xy is None:
        c = positive_mesh.centroid()
        center_xy = (c[0], c[1])
    cx, cy = center_xy

    n_z = max(8, int((z1 - z0) / z_step_mm) + 1)
    z_vals = [z0 + i * (z1 - z0) / (n_z - 1) for i in range(n_z)]
    thetas = [2.0 * math.pi * k / n_theta for k in range(n_theta)]

    # Bin triangles by Z interval for O(N) ray queries
    bucket_size = max(2.0, z_step_mm * 2.0)
    buckets: dict[int, list[int]] = {}
    verts = positive_mesh.vertices
    for fi, (a, b, c_idx) in enumerate(positive_mesh.faces):
        za, zb, zc = verts[a][2], verts[b][2], verts[c_idx][2]
        b_lo = math.floor(min(za, zb, zc) / bucket_size)
        b_hi = math.floor(max(za, zb, zc) / bucket_size)
        for bk in range(b_lo, b_hi + 1):
            buckets.setdefault(bk, []).append(fi)

    grid: list[list[float]] = []
    for z in z_vals:
        bk = math.floor(z / bucket_size)
        cand_faces = buckets.get(bk, ())
        row: list[float] = []
        origin = (cx, cy, z)
        for th in thetas:
            direction = (math.cos(th), math.sin(th), 0.0)
            max_r = 0.0
            for fi in cand_faces:
                a, b, c_idx = positive_mesh.faces[fi]
                t = _ray_tri_x_dist(origin, direction, verts[a], verts[b], verts[c_idx])
                if t is not None and t > max_r:
                    max_r = t
            row.append(max_r)
        grid.append(row)

    # Fill any occasional ray misses along edges by neighbor max interpolation
    for iz in range(n_z):
        for it in range(n_theta):
            if grid[iz][it] <= 1e-3:
                nbrs = [
                    grid[max(0, min(n_z - 1, iz + dz))][(it + dt) % n_theta]
                    for dz in (-1, 0, 1)
                    for dt in (-1, 0, 1)
                ]
                pos_n = [v for v in nbrs if v > 1e-3]
                grid[iz][it] = sum(pos_n) / len(pos_n) if pos_n else 15.0

    return z_vals, thetas, grid, center_xy


def generate_4axis_gcode_and_simulate(
    positive_mesh: TriMesh,
    ball_radius_mm: float = 4.0,
    z_step_mm: float = 2.0,
    n_theta: int = 60,
    rough_allowance_mm: float = 3.0,
    feed_rough_mm_min: float = 3200.0,
    feed_finish_mm_min: float = 2000.0,
    output_path: str | Path | None = None,
) -> CarverCamResult:
    """Generate gouge-free 4-axis rotary foam-carver G-code (X, Z, A) and verify stock removal.

    Convention:
      - `Z`: axial coordinate along mandrel axis (mm)
      - `A`: rotary table angle (degrees, continuous helical winding)
      - `X`: radial tool-tip distance from rotary axis (`r_ball_center - ball_radius_mm`)
    """
    z_vals, thetas, r_target, _ = sample_cylindrical_heightmap(
        positive_mesh,
        z_step_mm=z_step_mm,
        n_theta=n_theta,
    )
    n_z = len(z_vals)
    max_r = max(max(row) for row in r_target)
    stock_r = max_r + 8.0
    dz_step = (z_vals[-1] - z_vals[0]) / max(1, n_z - 1)
    max_dz_idx = max(1, int(math.ceil(ball_radius_mm / max(0.5, dz_step))))
    dtheta = 2.0 * math.pi / n_theta

    # Exact spherical ball-nose dilation: r_center(z, theta) clears all neighboring surface points
    r_center: list[list[float]] = [[0.0] * n_theta for _ in range(n_z)]
    for iz in range(n_z):
        z_cur = z_vals[iz]
        for it in range(n_theta):
            best_rc = r_target[iz][it] + ball_radius_mm
            for jz in range(max(0, iz - max_dz_idx), min(n_z, iz + max_dz_idx + 1)):
                dz = z_vals[jz] - z_cur
                if abs(dz) >= ball_radius_mm:
                    continue
                for dt_i in range(-6, 7):
                    jt = (it + dt_i) % n_theta
                    rs = r_target[jz][jt]
                    d_arc = rs * (dt_i * dtheta)
                    rad2 = ball_radius_mm * ball_radius_mm - dz * dz - d_arc * d_arc
                    if rad2 > 0.0:
                        rc = rs + math.sqrt(rad2)
                        if rc > best_rc:
                            best_rc = rc
            r_center[iz][it] = best_rc

    # Emit GRBL/LinuxCNC-compatible 4-axis rotary G-code
    retract_x = stock_r + 15.0
    lines: list[str] = [
        "; op-core 4-Axis Rotary Foam Carver G-Code",
        f"; StockRadius={stock_r:.2f}mm BallRadius={ball_radius_mm:.2f}mm ZStep={dz_step:.2f}mm",
        "G21 ; Millimeters",
        "G90 ; Absolute positioning",
        "M3 S14000 ; Spindle ON",
        f"G0 X{retract_x:.3f} Z{z_vals[0]:.3f} A0.000",
    ]

    # Roughing pass (coarse 2x Z step with +rough_allowance_mm radial stock)
    lines.append("; --- Roughing Pass ---")
    lines.append(f"G1 F{feed_rough_mm_min:.0f}")
    a_cum = 0.0
    for iz in range(0, n_z, 2):
        z = z_vals[iz]
        for k in range(n_theta):
            x_tip = (r_center[iz][k] + rough_allowance_mm) - ball_radius_mm
            a_deg = a_cum + k * (360.0 / n_theta)
            lines.append(f"G1 X{x_tip:.3f} Z{z:.3f} A{a_deg:.2f}")
        a_cum += 360.0

    # Finishing pass (continuous helical rotary pass at exact ball-center offset)
    lines.append("; --- Finishing Pass ---")
    lines.append(f"G1 F{feed_finish_mm_min:.0f}")
    finish_centers: list[list[float]] = [[0.0] * n_theta for _ in range(n_z)]
    for iz in range(n_z):
        z0_ring = z_vals[iz]
        z1_ring = z_vals[min(n_z - 1, iz + 1)]
        for k in range(n_theta):
            frac = k / n_theta
            z_cur = z0_ring * (1.0 - frac) + z1_ring * frac
            rc = r_center[iz][k]
            finish_centers[iz][k] = rc
            x_tip = rc - ball_radius_mm
            a_deg = a_cum + k * (360.0 / n_theta)
            lines.append(f"G1 X{x_tip:.3f} Z{z_cur:.3f} A{a_deg:.2f}")
        a_cum += 360.0

    lines.extend([
        f"G0 X{retract_x:.3f}",
        "M5 ; Spindle OFF",
        "M30 ; Program End",
    ])
    gcode = "\n".join(lines) + "\n"

    if output_path is not None:
        out_p = Path(output_path)
        out_p.parent.mkdir(parents=True, exist_ok=True)
        out_p.write_text(gcode, encoding="utf-8")

    # Simulate exact spherical ball-nose erosion on finishing pass to verify RMS error & gouges
    sq_err_sum = 0.0
    count = 0
    max_gouge = 0.0
    max_excess = 0.0
    mandrel_collisions = 0
    z_top_limit = z_vals[-1] + 1.0

    for iz in range(n_z):
        z_s = z_vals[iz]
        for it in range(n_theta):
            rs = r_target[iz][it]
            r_carved = stock_r
            for jz in range(max(0, iz - max_dz_idx), min(n_z, iz + max_dz_idx + 1)):
                dz = z_vals[jz] - z_s
                if abs(dz) >= ball_radius_mm:
                    continue
                for dt_i in range(-6, 7):
                    jt = (it + dt_i) % n_theta
                    d_arc = rs * (dt_i * dtheta)
                    rad2 = ball_radius_mm * ball_radius_mm - dz * dz - d_arc * d_arc
                    if rad2 > 0.0:
                        rem = finish_centers[jz][jt] - math.sqrt(rad2)
                        if rem < r_carved:
                            r_carved = rem
            if r_carved < 12.0 and z_s > z_top_limit - 10.0:
                mandrel_collisions += 1
            diff = r_carved - rs
            sq_err_sum += diff * diff
            count += 1
            if diff < -1e-6:
                max_gouge = max(max_gouge, -diff)
            elif diff > 0.0:
                max_excess = max(max_excess, diff)

    rms = math.sqrt(sq_err_sum / max(1, count))
    warnings: list[str] = []
    if max_excess > 1.5:
        warnings.append(f"Ball cutter radius ({ball_radius_mm:.1f} mm) leaves {max_excess:.2f} mm scallop in tight concavity")

    return CarverCamResult(
        gcode=gcode,
        line_count=len(lines),
        rms_error_mm=rms,
        max_gouge_mm=max_gouge,
        max_excess_mm=max_excess,
        mandrel_collisions=mandrel_collisions,
        stock_radius_mm=stock_r,
        ball_radius_mm=ball_radius_mm,
        warnings=warnings,
    )
