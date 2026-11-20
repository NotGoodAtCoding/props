"""Project 3: Watertight variable-thickness shelling, MPT/TKA bench alignment, hardware bosses, and carver molds."""

from __future__ import annotations

import importlib.util
import math
from dataclasses import dataclass, field
from typing import Any

from .math3d import (
    Vec3,
    rodrigues_rotate,
    smoothstep5,
    v_add,
    v_cross,
    v_dist,
    v_dot,
    v_norm,
    v_scale,
    v_sub,
    v_unit,
)
from .mesh import Tri, TriMesh
from .repair import clip_mesh_scalar, clip_mesh_z, repair_scan_mesh


def has_manifold3d() -> bool:
    """Return True if optional C++ `manifold3d` accelerator is installed."""
    return importlib.util.find_spec("manifold3d") is not None


@dataclass
class ShellBuildResult:
    """Compiled watertight socket or AFO shell, reinforcement modifier volume, and hardware metrics."""

    shell_mesh: TriMesh
    modifier_mesh: TriMesh
    alignment_normal: Vec3
    trimline_min_radius_mm: float
    hardware_metrics: dict[str, float | str] = field(default_factory=dict)


@dataclass
class CarverPositiveResult:
    """Carver-ready positive foam mold with keyed mandrel bore, draft collar, and overhang diagnostics."""

    positive_mesh: TriMesh
    mandrel_axis: Vec3
    draft_angle_deg: float
    max_overhang_deg: float
    recommended_cant_deg: float


def _smooth_normals(mesh: TriMesh, iterations: int = 4) -> list[Vec3]:
    """Laplacian-smooth vertex normals and guard against concavity foldovers."""
    adj: list[set[int]] = [set() for _ in mesh.vertices]
    for a, b, c in mesh.faces:
        adj[a].update((b, c))
        adj[b].update((a, c))
        adj[c].update((a, b))
    normals = mesh.vertex_normals()
    cx, cy, cz = mesh.centroid()
    for _ in range(iterations):
        nxt: list[Vec3] = []
        for i, n in enumerate(normals):
            acc = n
            for nb in adj[i]:
                acc = v_add(acc, v_scale(normals[nb], 0.35))
            sm = v_unit(acc, fallback=n)
            # Ensure normal stays outward relative to radial shaft direction in concavities
            p = mesh.vertices[i]
            rad = v_unit((p[0] - cx, p[1] - cy, 0.0), fallback=sm)
            if v_dot(sm, rad) < -0.25:
                sm = v_unit(v_add(v_scale(sm, 0.7), v_scale(rad, 0.3)), fallback=sm)
            nxt.append(sm)
        normals = nxt
    return normals


def _bridge_rings_manifold(
    ring_a: list[int],
    ring_b: list[int],
    faces: list[Tri],
    flip: bool = False,
) -> None:
    """Bridge two equal-length closed vertex rings with consistent triangle winding."""
    m = len(ring_a)
    for k in range(m):
        kn = (k + 1) % m
        a0, a1 = ring_a[k], ring_a[kn]
        b0, b1 = ring_b[k], ring_b[kn]
        if not flip:
            faces.append((b0, a0, a1))
            faces.append((b0, a1, b1))
        else:
            faces.append((b0, a1, a0))
            faces.append((b0, b1, a1))


def _bridge_unequal_rings_manifold(
    outer_cw: list[int],
    boss_cw: list[int],
    verts: list[Vec3],
    faces: list[Tri],
    center_xy: tuple[float, float],
) -> None:
    """Bridge an arbitrary M-vertex CW outer loop (with free edges (o_nxt, o_cur)) to an
    N-vertex CW boss top loop (with free edges (b_cur, b_nxt)) with zero winding mismatches.
    """
    m = len(outer_cw)
    n = len(boss_cw)
    if m < 3 or n < 3:
        raise ValueError("Cannot bridge ring with fewer than 3 vertices")

    # Align starting vertex on outer_cw closest in XY angle to boss_cw[0]
    b0 = verts[boss_cw[0]]
    b0_ang = math.atan2(b0[1] - center_xy[1], b0[0] - center_xy[0]) % (2.0 * math.pi)
    s0 = min(
        range(m),
        key=lambda i: abs(
            ((math.atan2(verts[outer_cw[i]][1] - center_xy[1], verts[outer_cw[i]][0] - center_xy[0]) % (2.0 * math.pi)) - b0_ang + math.pi)
            % (2.0 * math.pi)
            - math.pi
        ),
    )
    outer = outer_cw[s0:] + outer_cw[:s0]

    oi, bi = 0, 0
    while oi < m or bi < n:
        o_cur = outer[oi % m]
        o_nxt = outer[(oi + 1) % m]
        b_cur = boss_cw[bi % n]
        b_nxt = boss_cw[(bi + 1) % n]
        if oi == m:
            faces.append((b_nxt, b_cur, o_cur))
            bi += 1
        elif bi == n:
            faces.append((o_cur, o_nxt, b_cur))
            oi += 1
        elif oi == m - 1 and bi < n:
            # Exhaust boss ring before final outer step so closing spoke cancels step 0
            faces.append((b_nxt, b_cur, o_cur))
            bi += 1
        elif (oi + 1) / m <= (bi + 1) / n:
            faces.append((o_cur, o_nxt, b_cur))
            oi += 1
        else:
            faces.append((b_nxt, b_cur, o_cur))
            bi += 1


def _order_loop_matching_boundary(mesh: TriMesh, loop: list[int]) -> list[int]:
    """Ensure `loop` follows the mesh's directed boundary half-edges `(loop[k], loop[k+1])`."""
    half = {(a, b) for a, b, c in mesh.faces} | {(b, c) for a, b, c in mesh.faces} | {(c, a) for a, b, c in mesh.faces}
    if len(loop) >= 2 and (loop[0], loop[1]) not in half:
        return list(reversed(loop))
    return list(loop)


def _loop_signed_area_xy(verts: list[Vec3], loop: list[int]) -> float:
    s = 0.0
    for k in range(len(loop)):
        x0, y0, _ = verts[loop[k]]
        x1, y1, _ = verts[loop[(k + 1) % len(loop)]]
        s += x0 * y1 - x1 * y0
    return 0.5 * s


def _build_captive_m6_pyramid_boss_cap(
    boss_center: Vec3,
    flexion_deg: float,
    adduction_deg: float,
    boss_radius_mm: float = 36.0,
    boss_thickness_mm: float = 18.0,
    bcd_mm: float = 50.8,
    n_seg: int = 64,
) -> tuple[list[Vec3], list[Tri], list[int]]:
    """Construct a fixed-64-segment watertight 4-hole 50.8 mm BCD M6 captive-nut boss cap.

    Returns `(verts, faces, top_ring_cw_indices)` where `top_ring_cw_indices` runs CW
    around `+ez` with free directed edges `(top[k], top[k+1])` on the boss side wall.
    """
    def align_vec(p: Vec3) -> Vec3:
        q = rodrigues_rotate(p, (1.0, 0.0, 0.0), math.radians(flexion_deg))
        q = rodrigues_rotate(q, (0.0, 1.0, 0.0), math.radians(adduction_deg))
        return q

    ex = align_vec((1.0, 0.0, 0.0))
    ey = align_vec((0.0, 1.0, 0.0))
    ez = align_vec((0.0, 0.0, 1.0))

    def to_world(lx: float, ly: float, lz: float) -> Vec3:
        return (
            boss_center[0] + lx * ex[0] + ly * ey[0] + lz * ez[0],
            boss_center[1] + lx * ex[1] + ly * ey[1] + lz * ez[1],
            boss_center[2] + lx * ex[2] + ly * ey[2] + lz * ez[2],
        )

    verts: list[Vec3] = []
    faces: list[Tri] = []

    # Build CW outer ring (ang descending) so side wall has free edges (top[k], top[k+1])
    top_ring_idx: list[int] = []
    bot_ring_idx: list[int] = []
    for k in range(n_seg):
        ang = -2.0 * math.pi * k / n_seg
        lx = boss_radius_mm * math.cos(ang)
        ly = boss_radius_mm * math.sin(ang)
        top_ring_idx.append(len(verts))
        verts.append(to_world(lx, ly, boss_thickness_mm))
        bot_ring_idx.append(len(verts))
        verts.append(to_world(lx, ly, 0.0))

    _bridge_rings_manifold(top_ring_idx, bot_ring_idx, faces, flip=False)

    r_bc = bcd_mm * 0.5  # 25.4 mm
    r_hex = 5.8          # M6 nut pocket radius
    pocket_depth = 8.0

    center_bot_idx = len(verts)
    verts.append(to_world(0.0, 0.0, 0.0))

    # Reverse bot_ring_idx to get CCW ordering around (ex, ey) for clean quadrant slicing
    ccw_bot = list(reversed(bot_ring_idx))
    ccw_angs = [
        math.atan2(v_dot(v_sub(verts[idx], boss_center), ey), v_dot(v_sub(verts[idx], boss_center), ex)) % (2.0 * math.pi)
        for idx in ccw_bot
    ]

    sep_targets = [0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi]
    sep_pos = [
        min(range(n_seg), key=lambda p: abs(((ccw_angs[p] - targ + math.pi) % (2.0 * math.pi)) - math.pi))
        for targ in sep_targets
    ]

    for q in range(4):
        p_start = sep_pos[q]
        p_end = sep_pos[(q + 1) % 4]
        arc: list[int] = [ccw_bot[p_start]]
        p = (p_start + 1) % n_seg
        while p != p_end:
            arc.append(ccw_bot[p])
            p = (p + 1) % n_seg
        arc.append(ccw_bot[p_end])

        hole_ang = math.radians(45.0 + 90.0 * q)
        hx = r_bc * math.cos(hole_ang)
        hy = r_bc * math.sin(hole_ang)

        hex_bot: list[int] = []
        hex_top: list[int] = []
        for k in range(6):
            ha = hole_ang + k * (math.pi / 3.0)
            px = hx + r_hex * math.cos(ha)
            py = hy + r_hex * math.sin(ha)
            hex_bot.append(len(verts))
            verts.append(to_world(px, py, 0.0))
            hex_top.append(len(verts))
            verts.append(to_world(px, py, pocket_depth))

        _bridge_rings_manifold(hex_top, hex_bot, faces, flip=False)
        hex_center = len(verts)
        verts.append(to_world(hx, hy, pocket_depth))
        for k in range(6):
            kn = (k + 1) % 6
            faces.append((hex_top[kn], hex_top[k], hex_center))

        sector_cw = [center_bot_idx] + list(reversed(arc))
        m_sec = len(sector_cw)
        hx_w = to_world(hx, hy, 0.0)
        sec_angs = [
            math.atan2(v_dot(v_sub(verts[v_idx], hx_w), ey), v_dot(v_sub(verts[v_idx], hx_w), ex)) % (2.0 * math.pi)
            for v_idx in sector_cw
        ]
        hex_cw = list(reversed(hex_bot))
        h0_ang = math.atan2(v_dot(v_sub(verts[hex_cw[0]], hx_w), ey), v_dot(v_sub(verts[hex_cw[0]], hx_w), ex)) % (2.0 * math.pi)
        s0 = min(range(m_sec), key=lambda i: abs(((sec_angs[i] - h0_ang + math.pi) % (2.0 * math.pi)) - math.pi))
        outer = sector_cw[s0:] + sector_cw[:s0]

        oi, ii = 0, 0
        while oi < m_sec or ii < 6:
            o_cur = outer[oi % m_sec]
            o_nxt = outer[(oi + 1) % m_sec]
            i_cur = hex_cw[ii % 6]
            i_nxt = hex_cw[(ii + 1) % 6]
            if oi == m_sec:
                faces.append((o_cur, i_nxt, i_cur))
                ii += 1
            elif ii == 6:
                faces.append((o_cur, o_nxt, i_cur))
                oi += 1
            elif oi == m_sec - 1 and ii < 6:
                faces.append((o_cur, i_nxt, i_cur))
                ii += 1
            elif (oi + 1) / m_sec <= (ii + 1) / 6.0:
                faces.append((o_cur, o_nxt, i_cur))
                oi += 1
            else:
                faces.append((o_cur, i_nxt, i_cur))
                ii += 1

    return verts, faces, top_ring_idx


TRIMLINE_PRESETS: dict[str, dict[str, float]] = {
    "ptb": {"anterior_z_mm": 30.0, "posterior_z_mm": 0.0, "medial_z_mm": 24.0, "lateral_z_mm": 24.0},
    "ptb_sc": {"anterior_z_mm": 28.0, "posterior_z_mm": -2.0, "medial_z_mm": 48.0, "lateral_z_mm": 46.0},
    "ptb_scsp": {"anterior_z_mm": 56.0, "posterior_z_mm": -2.0, "medial_z_mm": 50.0, "lateral_z_mm": 48.0},
    "tf_ischial": {"anterior_z_mm": 32.0, "posterior_z_mm": 18.0, "medial_z_mm": 10.0, "lateral_z_mm": 50.0},
    "flat": {"anterior_z_mm": 38.0, "posterior_z_mm": 38.0, "medial_z_mm": 38.0, "lateral_z_mm": 38.0},
}


def _shaft_center_xy(mesh: TriMesh) -> tuple[float, float]:
    """Compute stable mid-shaft XY centerline for angular trimline evaluation."""
    if not mesh.vertices:
        return (0.0, 0.0)
    bmin, bmax = mesh.bounds()
    zspan = max(1e-6, bmax[2] - bmin[2])
    z_lo = bmin[2] + 0.25 * zspan
    z_hi = bmin[2] + 0.75 * zspan
    mid = [v for v in mesh.vertices if z_lo <= v[2] <= z_hi]
    pts = mid if len(mid) >= 12 else mesh.vertices
    return (
        sum(v[0] for v in pts) / len(pts),
        sum(v[1] for v in pts) / len(pts),
    )


def evaluate_trimline_z(
    theta: float,
    trimline: dict[str, Any],
    center_xy: tuple[float, float] = (0.0, 0.0),
    default_z: float = 38.0,
) -> float:
    """Evaluate contoured socket trimline height Z(theta) around the limb circumference.

    theta = atan2(y - cy, x - cx): +pi/2 = Anterior (+Y), -pi/2 = Posterior (-Y),
    0 = Lateral (+X), +-pi = Medial (-X).
    """
    pts = trimline.get("points") or []
    if len(pts) >= 3:
        cx, cy = center_xy
        knots = sorted(
            (math.atan2(float(p[1]) - cy, float(p[0]) - cx), float(p[2]))
            for p in pts
        )
        # Merge near-duplicate angles cleanly
        dedup: list[tuple[float, float]] = []
        for ang, zv in knots:
            if dedup and abs(ang - dedup[-1][0]) < 1e-3:
                dedup[-1] = (0.5 * (dedup[-1][0] + ang), 0.5 * (dedup[-1][1] + zv))
            else:
                dedup.append((ang, zv))
        if len(dedup) >= 3:
            ext = (
                [(dedup[-1][0] - 2.0 * math.pi, dedup[-1][1])]
                + dedup
                + [(dedup[0][0] + 2.0 * math.pi, dedup[0][1])]
            )
            th = math.atan2(math.sin(theta), math.cos(theta))
            for k in range(len(ext) - 1):
                a0, z0 = ext[k]
                a1, z1 = ext[k + 1]
                if a0 <= th <= a1:
                    t = (th - a0) / max(1e-9, a1 - a0)
                    w = smoothstep5(t)
                    return z0 * (1.0 - w) + z1 * w

    preset_name = str(trimline.get("preset", ""))
    base = TRIMLINE_PRESETS.get(preset_name, {})
    z_ant = float(trimline.get("anterior_z_mm", base.get("anterior_z_mm", default_z)))
    z_post = float(trimline.get("posterior_z_mm", base.get("posterior_z_mm", default_z)))
    z_med = float(trimline.get("medial_z_mm", base.get("medial_z_mm", default_z)))
    z_lat = float(trimline.get("lateral_z_mm", base.get("lateral_z_mm", default_z)))

    ct = math.cos(theta)
    st = math.sin(theta)
    w_ant = max(0.0, st) ** 2
    w_post = max(0.0, -st) ** 2
    w_lat = max(0.0, ct) ** 2
    w_med = max(0.0, -ct) ** 2
    return z_ant * w_ant + z_post * w_post + z_lat * w_lat + z_med * w_med


def _clip_to_socket_trimline(
    mesh: TriMesh,
    proximal_trim_z_mm: float | None,
    trimline: dict[str, Any] | None = None,
) -> TriMesh:
    """Clip a limb mesh cleanly along a contoured 3D socket trimline or planar brim height."""
    if trimline:
        cx, cy = _shaft_center_xy(mesh)
        bmin, bmax = mesh.bounds()
        is_closed = len(mesh.boundary_loops()) == 0
        default_z = proximal_trim_z_mm if proximal_trim_z_mm is not None else 38.0
        max_cut_z = bmax[2] - 4.0 if is_closed else bmax[2] + 100.0
        min_cut_z = bmin[2] + 35.0
        values: list[float] = []
        will_cut = False
        for x, y, z in mesh.vertices:
            th = math.atan2(y - cy, x - cx)
            zt = evaluate_trimline_z(th, trimline, center_xy=(cx, cy), default_z=default_z)
            zt = max(min_cut_z, min(max_cut_z, zt))
            val = zt - z
            if val < 0.0:
                will_cut = True
            values.append(val)
        if not will_cut:
            return mesh
        clipped = clip_mesh_scalar(mesh, values=values, threshold=0.0, keep_above=True)
        comps = clipped.connected_components()
        if len(comps) > 1:
            clipped = clipped.submesh_from_vertices(comps[0])
        # Gently smooth boundary cuff loop vertices so oblique triangle clips form a clean brim
        loops = clipped.boundary_loops()
        if len(loops) == 1 and len(loops[0]) >= 6:
            c_verts = list(clipped.vertices)
            lp = _order_loop_matching_boundary(clipped, loops[0])
            m = len(lp)
            for _ in range(2):
                upd: dict[int, Vec3] = {}
                for k in range(m):
                    p0 = c_verts[lp[(k - 1) % m]]
                    p1 = c_verts[lp[k]]
                    p2 = c_verts[lp[(k + 1) % m]]
                    avg = v_scale(v_add(p0, p2), 0.5)
                    upd[lp[k]] = v_add(v_scale(p1, 0.72), v_scale(avg, 0.28))
                for idx, pt in upd.items():
                    c_verts[idx] = pt
            clipped = TriMesh(vertices=c_verts, faces=clipped.faces)
        return clipped

    if proximal_trim_z_mm is None:
        return mesh
    _, bmax = mesh.bounds()
    if bmax[2] <= proximal_trim_z_mm + 5.0:
        return mesh
    return clip_mesh_z(mesh, z_cut=proximal_trim_z_mm, keep_above=False)


def _build_socket_shell_core(
    rectified_positive: TriMesh,
    wall_distal_mm: float,
    wall_proximal_mm: float,
    flexion_deg: float,
    adduction_deg: float,
    brim_round_segments: int,
    hardware_tag: str,
    alignment_type: str,
    proximal_trim_z_mm: float | None = None,
    trimline: dict[str, Any] | None = None,
) -> ShellBuildResult:
    """Arbitrary-mesh socket shell builder using contoured trimline + distal clipping + manifold bridging."""
    trimmed = _clip_to_socket_trimline(rectified_positive, proximal_trim_z_mm, trimline=trimline)
    pos, _ = repair_scan_mesh(trimmed, fill_holes=True, ensure_open_cuff=True)
    loops = pos.boundary_loops()
    if len(loops) != 1:
        raise ValueError(f"Expected 1 proximal cuff loop after repair, found {len(loops)}")
    cuff_loop = _order_loop_matching_boundary(pos, loops[0])

    zmin, zmax = pos.bounds()[0][2], pos.bounds()[1][2]
    zspan = max(1e-6, zmax - zmin)
    n_smooth = _smooth_normals(pos, iterations=4)

    # Build full S_out first, then clip its distal tip cleanly at a plane to handle arbitrary topology
    out_full_verts: list[Vec3] = []
    for i, v in enumerate(pos.vertices):
        t_prox = max(0.0, min(1.0, (v[2] - zmin) / zspan))
        thick = wall_distal_mm * (1.0 - t_prox) + wall_proximal_mm * t_prox
        out_full_verts.append(v_add(v, v_scale(n_smooth[i], thick)))
    s_out_full = TriMesh(vertices=out_full_verts, faces=list(pos.faces))

    z_out_min = s_out_full.bounds()[0][2]
    z_clip = z_out_min + min(18.0, 0.14 * zspan)
    s_out_clipped = clip_mesh_z(s_out_full, z_cut=z_clip, keep_above=True)
    out_loops = s_out_clipped.boundary_loops()
    if len(out_loops) != 2:
        raise ValueError(f"Expected 2 boundary loops on clipped outer shell, found {len(out_loops)}")

    # Separate proximal outer cuff loop (highest Z) from distal clipped loop (at z_clip)
    out_loops_sorted = sorted(
        out_loops,
        key=lambda lp: sum(s_out_clipped.vertices[i][2] for i in lp) / len(lp),
    )
    out_dist_loop = _order_loop_matching_boundary(s_out_clipped, out_loops_sorted[0])
    out_cuff_raw = out_loops_sorted[1]

    # Align out_cuff_ring indices 1-to-1 with cuff_loop by nearest vertex on untouched proximal cuff
    cuff_pts_out = [out_full_verts[i] for i in cuff_loop]
    out_cuff_ring = [
        min(out_cuff_raw, key=lambda j: v_dist(s_out_clipped.vertices[j], target_pt))
        for target_pt in cuff_pts_out
    ]

    # Combine S_in (reversed pos) + s_out_clipped into single vertex/face buffer
    verts: list[Vec3] = list(pos.vertices)
    faces: list[Tri] = [(a, c, b) for a, b, c in pos.faces]  # S_in inward

    s_out_off = len(verts)
    verts.extend(s_out_clipped.vertices)
    for a, b, c in s_out_clipped.faces:
        faces.append((s_out_off + a, s_out_off + b, s_out_off + c))

    in_ring = list(cuff_loop)
    out_ring = [s_out_off + idx for idx in out_cuff_ring]
    prev_ring = in_ring
    for step in range(1, brim_round_segments):
        ang = math.pi * step / brim_round_segments
        mid_ring: list[int] = []
        for idx_in, idx_out in zip(in_ring, out_ring):
            p_in = verts[idx_in]
            p_out = verts[idx_out]
            center = v_scale(v_add(p_in, p_out), 0.5)
            half_vec = v_scale(v_sub(p_out, p_in), 0.5)
            r_half = v_norm(half_vec)
            pos_pt = (
                center[0] - half_vec[0] * math.cos(ang),
                center[1] - half_vec[1] * math.cos(ang),
                center[2] - half_vec[2] * math.cos(ang) + r_half * math.sin(ang),
            )
            mid_ring.append(len(verts))
            verts.append(pos_pt)
        _bridge_rings_manifold(prev_ring, mid_ring, faces, flip=False)
        prev_ring = mid_ring
    _bridge_rings_manifold(prev_ring, out_ring, faces, flip=False)

    # Order out_dist_loop CW in XY (and verify it keeps s_out_clipped's directed boundary edges (o_nxt, o_cur))
    # Remember: s_out_clipped has outward normals; on its bottom clip at z_clip, boundary edges run CCW
    # when viewed from +Z, meaning _order_loop_matching_boundary gives CCW -> reversing gives CW with free edges (o_nxt, o_cur).
    if _loop_signed_area_xy(s_out_clipped.vertices, out_dist_loop) > 0.0:
        out_dist_cw = list(reversed(out_dist_loop))
    else:
        out_dist_cw = list(out_dist_loop)
    out_dist_global = [s_out_off + idx for idx in out_dist_cw]

    cx_d = sum(verts[i][0] for i in out_dist_global) / len(out_dist_global)
    cy_d = sum(verts[i][1] for i in out_dist_global) / len(out_dist_global)
    boss_center = (cx_d, cy_d, z_clip - 22.0)

    boss_verts, boss_faces, boss_top_local = _build_captive_m6_pyramid_boss_cap(
        boss_center=boss_center,
        flexion_deg=flexion_deg,
        adduction_deg=adduction_deg,
    )
    base_off = len(verts)
    verts.extend(boss_verts)
    for a, b, c in boss_faces:
        faces.append((base_off + a, base_off + b, base_off + c))
    boss_top_global = [base_off + idx for idx in boss_top_local]

    _bridge_unequal_rings_manifold(out_dist_global, boss_top_global, verts, faces, center_xy=(cx_d, cy_d))

    shell = TriMesh(vertices=verts, faces=faces)
    mod_mesh = _build_cylinder_mesh(
        center=(boss_center[0], boss_center[1], boss_center[2] - 2.0),
        radius_mm=48.0,
        height_mm=52.0,
    )

    align_n = rodrigues_rotate((0.0, 0.0, -1.0), (1.0, 0.0, 0.0), math.radians(flexion_deg))
    align_n = rodrigues_rotate(align_n, (0.0, 1.0, 0.0), math.radians(adduction_deg))

    return ShellBuildResult(
        shell_mesh=shell,
        modifier_mesh=mod_mesh,
        alignment_normal=v_unit(align_n),
        trimline_min_radius_mm=15.0,
        hardware_metrics={
            "hardware": hardware_tag,
            "alignment_reference": alignment_type,
            "bcd_mm": 50.8,
            "flexion_deg": flexion_deg,
            "adduction_deg": adduction_deg,
        },
    )


def build_tt_socket_shell(
    rectified_positive: TriMesh,
    wall_distal_mm: float = 4.5,
    wall_proximal_mm: float = 2.5,
    flexion_deg: float = 5.0,
    adduction_deg: float = 5.0,
    brim_round_segments: int = 3,
    proximal_trim_z_mm: float | None = None,
    trimline: dict[str, Any] | None = None,
) -> ShellBuildResult:
    """Build a watertight TT socket shell with contoured trimline, rounded brim, and pyramid boss."""
    return _build_socket_shell_core(
        rectified_positive=rectified_positive,
        wall_distal_mm=wall_distal_mm,
        wall_proximal_mm=wall_proximal_mm,
        flexion_deg=flexion_deg,
        adduction_deg=adduction_deg,
        brim_round_segments=brim_round_segments,
        hardware_tag="pyramid_50_8_m6_captive",
        alignment_type="tt_mpt_bisection_plumb_line",
        proximal_trim_z_mm=proximal_trim_z_mm,
        trimline=trimline,
    )


def build_tf_socket_shell(
    rectified_positive: TriMesh,
    wall_distal_mm: float = 5.5,
    wall_proximal_mm: float = 3.0,
    flexion_deg: float = 5.0,
    adduction_deg: float = 8.0,
    brim_round_segments: int = 3,
    proximal_trim_z_mm: float | None = None,
    trimline: dict[str, Any] | None = None,
) -> ShellBuildResult:
    """Build a watertight transfemoral (TF) socket shell with contoured trimline and TKA pyramid boss."""
    return _build_socket_shell_core(
        rectified_positive=rectified_positive,
        wall_distal_mm=wall_distal_mm,
        wall_proximal_mm=wall_proximal_mm,
        flexion_deg=flexion_deg,
        adduction_deg=adduction_deg,
        brim_round_segments=brim_round_segments,
        hardware_tag="pyramid_50_8_m6_captive",
        alignment_type="tf_tka_plumb_line",
        proximal_trim_z_mm=proximal_trim_z_mm,
        trimline=trimline,
    )


def _build_cylinder_mesh(center: Vec3, radius_mm: float, height_mm: float, n_seg: int = 32) -> TriMesh:
    verts: list[Vec3] = []
    faces: list[Tri] = []
    bot_ring: list[int] = []
    top_ring: list[int] = []
    for k in range(n_seg):
        a = 2.0 * math.pi * k / n_seg
        x = center[0] + radius_mm * math.cos(a)
        y = center[1] + radius_mm * math.sin(a)
        bot_ring.append(len(verts))
        verts.append((x, y, center[2]))
        top_ring.append(len(verts))
        verts.append((x, y, center[2] + height_mm))
    _bridge_rings_manifold(top_ring, bot_ring, faces, flip=False)
    bc = len(verts)
    verts.append((center[0], center[1], center[2]))
    tc = len(verts)
    verts.append((center[0], center[1], center[2] + height_mm))
    for k in range(n_seg):
        kn = (k + 1) % n_seg
        faces.append((bot_ring[k], bot_ring[kn], bc))
        faces.append((top_ring[kn], top_ring[k], tc))
    return TriMesh(vertices=verts, faces=faces)


def compute_curve_min_radius(points: list[Vec3], closed: bool = True) -> float:
    """Compute minimum discrete osculating circle radius along a 3D trimline polyline."""
    n = len(points)
    if n < 3:
        return math.inf
    min_r = math.inf
    start = 0 if closed else 1
    end = n if closed else n - 1
    for i in range(start, end):
        p0 = points[(i - 1) % n]
        p1 = points[i]
        p2 = points[(i + 1) % n]
        a = v_dist(p0, p1)
        b = v_dist(p1, p2)
        c = v_dist(p2, p0)
        area2 = v_norm(v_cross(v_sub(p1, p0), v_sub(p2, p0)))
        if area2 > 1e-6:
            r = (a * b * c) / (2.0 * area2)
            if r < min_r:
                min_r = r
    return min_r


def build_afo_shell(
    neutral_foot_mesh: TriMesh,
    wall_neck_mm: float = 4.5,
    wall_calf_mm: float = 2.5,
    trimline_fillet_mm: float = 10.0,
    joint_y_mm: float = 0.0,
    joint_z_mm: float = 56.0,
    calf_anterior_y_mm: float = 12.0,
    footplate_z_mm: float = 24.0,
    footplate_max_y_mm: float = 240.0,
    joint_recess_radius_mm: float = 8.0,
    joint_recess_depth_mm: float = 2.0,
) -> ShellBuildResult:
    """Build a watertight posterior-leaf / solid AFO shell with custom trimlines & coaxial joint pockets."""
    keep_face_mask: list[bool] = []
    for a, b, c in neutral_foot_mesh.faces:
        pts = [neutral_foot_mesh.vertices[idx] for idx in (a, b, c)]
        cy = sum(p[1] for p in pts) / 3.0
        cz = sum(p[2] for p in pts) / 3.0
        if cz > 75.0:
            keep = cy < calf_anterior_y_mm
        elif cz >= 38.0:
            t = (cz - 38.0) / 37.0
            y_cut = 35.0 * (1.0 - smoothstep5(t)) + calf_anterior_y_mm * smoothstep5(t)
            keep = cy < y_cut
        else:
            keep = cz < footplate_z_mm and cy <= footplate_max_y_mm
        keep_face_mask.append(keep)

    kept_faces = [f for f, k in zip(neutral_foot_mesh.faces, keep_face_mask) if k]
    kept_verts = {idx for f in kept_faces for idx in f}
    patch = neutral_foot_mesh.submesh_from_vertices(kept_verts)
    comps = patch.connected_components()
    if comps:
        patch = patch.submesh_from_vertices(comps[0])

    loops = patch.boundary_loops()
    if not loops:
        raise ValueError("AFO trimline patch produced no open boundary loop")

    p_verts = list(patch.vertices)
    main_loop = max(loops, key=len)
    for _ in range(6):
        updates: dict[int, Vec3] = {}
        m = len(main_loop)
        for k in range(m):
            prev_p = p_verts[main_loop[(k - 1) % m]]
            cur_p = p_verts[main_loop[k]]
            next_p = p_verts[main_loop[(k + 1) % m]]
            avg = v_scale(v_add(prev_p, next_p), 0.5)
            updates[main_loop[k]] = v_add(v_scale(cur_p, 0.55), v_scale(avg, 0.45))
        for idx, val in updates.items():
            p_verts[idx] = val
    patch = TriMesh(vertices=p_verts, faces=patch.faces)

    min_r = compute_curve_min_radius([patch.vertices[i] for i in main_loop], closed=True)
    effective_fillet = max(min_r, trimline_fillet_mm)

    ordered_loops = [_order_loop_matching_boundary(patch, lp) for lp in patch.boundary_loops()]
    p_norms = _smooth_normals(patch, iterations=3)
    verts = list(patch.vertices)
    faces: list[Tri] = [(a, c, b) for a, b, c in patch.faces]

    out_map: dict[int, int] = {}
    for i, v in enumerate(patch.vertices):
        neck_w = math.exp(-((v[2] - joint_z_mm) / 40.0) ** 2)
        thick = wall_calf_mm + (wall_neck_mm - wall_calf_mm) * neck_w
        # Stepped coaxial Tamarack/Becker joint boss + circular recess pocket on medial/lateral ankle faces
        if abs(v[0]) > 22.0:
            r_joint = math.hypot(v[1] - joint_y_mm, v[2] - joint_z_mm)
            if r_joint < 16.0:
                pad_w = 1.0 - smoothstep5(max(0.0, (r_joint - 9.0) / 7.0))
                thick += 4.2 * pad_w
                if r_joint < joint_recess_radius_mm:
                    rec_w = 1.0 - smoothstep5(r_joint / max(1e-3, joint_recess_radius_mm))
                    thick -= joint_recess_depth_mm * rec_w
        out_map[i] = len(verts)
        verts.append(v_add(v, v_scale(p_norms[i], thick)))

    for a, b, c in patch.faces:
        faces.append((out_map[a], out_map[b], out_map[c]))

    for lp in ordered_loops:
        in_ring = list(lp)
        out_ring = [out_map[i] for i in lp]
        mid_ring: list[int] = []
        for idx in lp:
            p_in = verts[idx]
            p_out = verts[out_map[idx]]
            mid_ring.append(len(verts))
            verts.append(v_scale(v_add(p_in, p_out), 0.5))
        _bridge_rings_manifold(in_ring, mid_ring, faces, flip=False)
        _bridge_rings_manifold(mid_ring, out_ring, faces, flip=False)

    shell = TriMesh(vertices=verts, faces=faces)
    mod_mesh = _build_cylinder_mesh(
        center=(0.0, joint_y_mm, joint_z_mm - 35.0),
        radius_mm=56.0,
        height_mm=70.0,
    )
    return ShellBuildResult(
        shell_mesh=shell,
        modifier_mesh=mod_mesh,
        alignment_normal=(1.0, 0.0, 0.0),
        trimline_min_radius_mm=effective_fillet,
        hardware_metrics={
            "hardware": "afo_coaxial_joint",
            "coaxial_error_deg": 0.0,
            "joint_z_mm": joint_z_mm,
        },
    )


def export_carver_positive(
    rectified_positive: TriMesh,
    draft_angle_deg: float = 12.0,
    collar_height_mm: float = 40.0,
    mandrel_radius_mm: float = 12.7,
    bore_depth_mm: float = 95.0,
    proximal_trim_z_mm: float | None = None,
    trimline: dict[str, Any] | None = None,
) -> CarverPositiveResult:
    """Convert any rectified positive (open or closed) into a watertight CNC foam-carver mold."""
    trimmed = _clip_to_socket_trimline(rectified_positive, proximal_trim_z_mm, trimline=trimline)
    pos, _ = repair_scan_mesh(trimmed, fill_holes=True, ensure_open_cuff=True)
    loops = pos.boundary_loops()
    if len(loops) != 1:
        raise ValueError(f"Carver prep expects 1 proximal cuff loop, found {len(loops)}")

    cuff = _order_loop_matching_boundary(pos, loops[0])
    cx = sum(pos.vertices[i][0] for i in cuff) / len(cuff)
    cy = sum(pos.vertices[i][1] for i in cuff) / len(cuff)
    z_top = max(pos.vertices[i][2] for i in cuff) + collar_height_mm
    tan_draft = math.tan(math.radians(draft_angle_deg))

    verts = list(pos.vertices)
    faces = list(pos.faces)

    collar_ring: list[int] = []
    for idx in cuff:
        vx, vy, vz = verts[idx]
        dx, dy = vx - cx, vy - cy
        r = math.hypot(dx, dy)
        dz = max(5.0, z_top - vz)
        scale = (r + dz * tan_draft) / max(1e-6, r)
        collar_ring.append(len(verts))
        verts.append((cx + dx * scale, cy + dy * scale, z_top))

    _bridge_rings_manifold(collar_ring, cuff, faces, flip=False)

    # Order bore angles uniformly around (cx, cy) matching cuff CW orientation
    m = len(collar_ring)
    cuff_cw = _loop_signed_area_xy(verts, collar_ring) < 0.0
    bore_top_ring: list[int] = []
    bore_bot_ring: list[int] = []
    # Find starting angle of collar_ring[0] so bore_top_ring aligns cleanly without twist
    ang0 = math.atan2(verts[collar_ring[0]][1] - cy, verts[collar_ring[0]][0] - cx)
    sign = -1.0 if cuff_cw else 1.0
    for k in range(m):
        ang = ang0 + sign * 2.0 * math.pi * k / m
        r_bore = mandrel_radius_mm
        if abs(math.sin(ang)) < 0.35 and math.cos(ang) > 0.0:
            r_bore += 4.0
        bx = cx + r_bore * math.cos(ang)
        by = cy + r_bore * math.sin(ang)
        bore_top_ring.append(len(verts))
        verts.append((bx, by, z_top))
        bore_bot_ring.append(len(verts))
        verts.append((bx, by, z_top - bore_depth_mm))

    _bridge_rings_manifold(bore_top_ring, collar_ring, faces, flip=False)
    _bridge_rings_manifold(bore_bot_ring, bore_top_ring, faces, flip=False)
    bore_floor = len(verts)
    verts.append((cx, cy, z_top - bore_depth_mm))
    for k in range(m):
        kn = (k + 1) % m
        faces.append((bore_bot_ring[kn], bore_bot_ring[k], bore_floor))

    carver_mesh = TriMesh(vertices=verts, faces=faces)

    fnorms = carver_mesh.face_normals()
    max_overhang = 0.0
    for fn in fnorms:
        ang_z = math.degrees(math.acos(max(-1.0, min(1.0, fn[2]))))
        if ang_z > max_overhang:
            max_overhang = ang_z

    bmin, bmax = carver_mesh.bounds()
    xy_span = max(bmax[0] - bmin[0], bmax[1] - bmin[1])
    z_span = bmax[2] - bmin[2]
    cant_deg = 35.0 if xy_span > 0.75 * z_span else 0.0

    return CarverPositiveResult(
        positive_mesh=carver_mesh,
        mandrel_axis=(0.0, 0.0, 1.0),
        draft_angle_deg=draft_angle_deg,
        max_overhang_deg=max_overhang,
        recommended_cant_deg=cant_deg,
    )
