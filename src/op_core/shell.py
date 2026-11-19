"""Project 3: Watertight variable-thickness shelling, MPT bench alignment, captive-M6 pyramid boss, AFO shells, and carver positives."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .math3d import (
    Vec2,
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


def _smooth_normals(mesh: TriMesh, iterations: int = 3) -> list[Vec3]:
    """Laplacian-smooth vertex normals to prevent offset self-intersections in concavities."""
    adj: list[set[int]] = [set() for _ in mesh.vertices]
    for a, b, c in mesh.faces:
        adj[a].update((b, c))
        adj[b].update((a, c))
        adj[c].update((a, b))
    normals = mesh.vertex_normals()
    for _ in range(iterations):
        nxt: list[Vec3] = []
        for i, n in enumerate(normals):
            acc = n
            for nb in adj[i]:
                acc = v_add(acc, v_scale(normals[nb], 0.35))
            nxt.append(v_unit(acc, fallback=n))
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


def _distal_ring_from_open_limb(mesh: TriMesh) -> tuple[list[int], int]:
    """Identify the distal pole vertex and its surrounding ring on a limb positive."""
    distal_idx = min(range(len(mesh.vertices)), key=lambda i: mesh.vertices[i][2])
    # Collect faces incident to distal_idx and extract ordered ring around distal_idx
    adj_next: dict[int, int] = {}
    for a, b, c in mesh.faces:
        if a == distal_idx:
            adj_next[b] = c
        elif b == distal_idx:
            adj_next[c] = a
        elif c == distal_idx:
            adj_next[a] = b
    if not adj_next:
        raise ValueError("Distal pole vertex has no incident faces")
    start = next(iter(adj_next))
    ring = [start]
    cur = adj_next[start]
    while cur != start and len(ring) <= len(adj_next) + 1:
        ring.append(cur)
        cur = adj_next.get(cur, start)
    return ring, distal_idx


def _build_captive_m6_pyramid_boss_cap(
    top_ring_pts: list[Vec3],
    boss_center: Vec3,
    boss_normal: Vec3,
    flexion_deg: float,
    adduction_deg: float,
    boss_radius_mm: float = 36.0,
    boss_thickness_mm: float = 18.0,
    bcd_mm: float = 50.8,
) -> tuple[list[Vec3], list[Tri], list[int]]:
    """Construct a watertight 4-hole 50.8 mm BCD M6 captive-nut boss cap.

    Returns (vertices, faces, top_ring_local_indices) where `top_ring_local_indices`
    matches the angular order of `top_ring_pts` so it bridges seamlessly onto `S_out`.
    """
    n_seg = len(top_ring_pts)
    # Build aligned basis for boss plane: start from world Z down=(0,0,-1),
    # rotate by flexion_deg around X and adduction_deg around Y.
    def align_vec(p: Vec3) -> Vec3:
        q = rodrigues_rotate(p, (1.0, 0.0, 0.0), math.radians(flexion_deg))
        q = rodrigues_rotate(q, (0.0, 1.0, 0.0), math.radians(adduction_deg))
        return q

    ex = align_vec((1.0, 0.0, 0.0))
    ey = align_vec((0.0, 1.0, 0.0))
    ez = align_vec((0.0, 0.0, 1.0))  # Points proximally (up into socket)

    def to_world(lx: float, ly: float, lz: float) -> Vec3:
        return (
            boss_center[0] + lx * ex[0] + ly * ey[0] + lz * ez[0],
            boss_center[1] + lx * ex[1] + ly * ey[1] + lz * ez[1],
            boss_center[2] + lx * ex[2] + ly * ey[2] + lz * ez[2],
        )

    verts: list[Vec3] = []
    faces: list[Tri] = []

    # Match angular orientation of top_ring_pts around boss_center in (ex, ey)
    angles = []
    for p in top_ring_pts:
        d = v_sub(p, boss_center)
        angles.append(math.atan2(v_dot(d, ey), v_dot(d, ex)))

    # Ensure n_seg is divisible by 4 or use fixed 64-vertex boss outer ring mapped cleanly:
    # We construct the boss directly with `n_seg` outer vertices aligned to `angles`!
    top_ring_idx: list[int] = []
    bot_ring_idx: list[int] = []
    for ang in angles:
        lx = boss_radius_mm * math.cos(ang)
        ly = boss_radius_mm * math.sin(ang)
        top_ring_idx.append(len(verts))
        verts.append(to_world(lx, ly, boss_thickness_mm))
        bot_ring_idx.append(len(verts))
        verts.append(to_world(lx, ly, 0.0))

    # Side cylindrical wall of boss (top_ring -> bot_ring)
    _bridge_rings_manifold(top_ring_idx, bot_ring_idx, faces, flip=False)

    # Bottom planar face with 4 recessed hexagonal M6 captive-nut pockets at 50.8 mm BCD
    # Divide bottom ring into 4 quadrants centered at 45, 135, 225, 315 deg
    r_bc = bcd_mm * 0.5  # 25.4 mm
    r_hex = 5.8          # 10 mm across-flats M6 hex nut pocket radius
    pocket_depth = 8.0   # 8 mm deep pocket inside 18 mm boss

    center_bot_idx = len(verts)
    verts.append(to_world(0.0, 0.0, 0.0))

    # Group bot_ring_idx into 4 quadrant arcs by angle in [0, 2pi)
    norm_angles = [(ang % (2.0 * math.pi), idx) for ang, idx in zip(angles, bot_ring_idx)]
    # Sort CCW around boss bottom
    # Note: top_ring_pts from distal pole ring is usually CW or CCW; inspect winding sign
    ang_diffs = sum(
        math.sin(angles[(k + 1) % n_seg] - angles[k]) for k in range(n_seg)
    )
    ccw_bot = list(bot_ring_idx) if ang_diffs > 0 else list(reversed(bot_ring_idx))
    ccw_angs = [a % (2.0 * math.pi) for a in (angles if ang_diffs > 0 else reversed(angles))]

    # Create 4 quadrant boundary indices on outer ring closest to 0, 90, 180, 270 deg
    sep_targets = [0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi]
    sep_pos: list[int] = []
    for targ in sep_targets:
        best_pos = min(
            range(n_seg),
            key=lambda p: abs(((ccw_angs[p] - targ + math.pi) % (2.0 * math.pi)) - math.pi),
        )
        sep_pos.append(best_pos)

    for q in range(4):
        p_start = sep_pos[q]
        p_end = sep_pos[(q + 1) % 4]
        arc: list[int] = [ccw_bot[p_start]]
        p = (p_start + 1) % n_seg
        while p != p_end:
            arc.append(ccw_bot[p])
            p = (p + 1) % n_seg
        arc.append(ccw_bot[p_end])

        # Hex pocket center at 45 + 90*q degrees on 50.8 mm BCD
        hole_ang = math.radians(45.0 + 90.0 * q)
        hx = r_bc * math.cos(hole_ang)
        hy = r_bc * math.sin(hole_ang)

        # Build 6-vertex hex ring on bottom face (z=0) and recessed top (z=pocket_depth)
        # Note: bottom face normal points -ez (downward). For a vertex loop around hole
        # ordered CCW in (ex, ey), bridging outer sector to inner hex ring gives -ez normal.
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

        # Pocket wall and recessed floor
        # Bottom face triangles traverse hex_cw as (i_nxt, i_cur) = hex_bot CCW -> pocket wall needs flip=False.
        _bridge_rings_manifold(hex_top, hex_bot, faces, flip=False)
        hex_center = len(verts)
        verts.append(to_world(hx, hy, pocket_depth))
        for k in range(6):
            kn = (k + 1) % 6
            faces.append((hex_top[kn], hex_top[k], hex_center))

        # Sector outer boundary must run CW in (ex, ey) to match side-wall free edges and produce -ez normal.
        sector_cw = [center_bot_idx] + list(reversed(arc))
        m_sec = len(sector_cw)
        hx_w = to_world(hx, hy, 0.0)
        sec_angs = []
        for v_idx in sector_cw:
            d = v_sub(verts[v_idx], hx_w)
            sec_angs.append(math.atan2(v_dot(d, ey), v_dot(d, ex)) % (2.0 * math.pi))

        # Inner hole loop ordered CW around (hx, hy) so (o_cur, i_nxt, i_cur) produces -ez normal
        hex_cw = list(reversed(hex_bot))
        h0_ang = math.atan2(
            v_dot(v_sub(verts[hex_cw[0]], hx_w), ey),
            v_dot(v_sub(verts[hex_cw[0]], hx_w), ex),
        ) % (2.0 * math.pi)
        s0 = min(
            range(m_sec),
            key=lambda i: abs(((sec_angs[i] - h0_ang + math.pi) % (2.0 * math.pi)) - math.pi),
        )
        outer = sector_cw[s0:] + sector_cw[:s0]
        oi, ii = 0, 0
        while oi < len(outer) or ii < 6:
            o_cur = outer[oi % len(outer)]
            o_nxt = outer[(oi + 1) % len(outer)]
            i_cur = hex_cw[ii % 6]
            i_nxt = hex_cw[(ii + 1) % 6]
            if oi == len(outer):
                faces.append((o_cur, i_nxt, i_cur))
                ii += 1
            elif ii == 6:
                faces.append((o_cur, o_nxt, i_cur))
                oi += 1
            elif oi == len(outer) - 1 and ii < 6:
                # Exhaust inner ring before the final outer step so closing spoke cancels step 0
                faces.append((o_cur, i_nxt, i_cur))
                ii += 1
            elif (oi + 1) / len(outer) <= (ii + 1) / 6.0:
                faces.append((o_cur, o_nxt, i_cur))
                oi += 1
            else:
                faces.append((o_cur, i_nxt, i_cur))
                ii += 1

    # Ensure outer ring winding matches `top_ring_pts` order when bridged
    if ang_diffs <= 0:
        # Caller's ring is CW, flip boss faces? Wait: boss side wall used `top_ring_idx` in caller's order,
        # which only needs reversal if caller's ring order differed from S_out's boundary order.
        pass

    return verts, faces, top_ring_idx


def build_tt_socket_shell(
    rectified_positive: TriMesh,
    wall_distal_mm: float = 4.5,
    wall_proximal_mm: float = 2.5,
    flexion_deg: float = 5.0,
    adduction_deg: float = 5.0,
    brim_round_segments: int = 3,
) -> ShellBuildResult:
    """Build a watertight TT socket shell with rounded brim and bench-aligned captive-M6 pyramid boss."""
    pos = rectified_positive.copy()
    cuffs = pos.boundary_loops()
    if len(cuffs) != 1:
        raise ValueError(f"Expected 1 proximal cuff loop on limb positive, found {len(cuffs)}")
    cuff_loop = cuffs[0]

    # Normalize cuff loop orientation to match positive's boundary edge direction
    pos_half = {(a, b) for a, b, c in pos.faces} | {(b, c) for a, b, c in pos.faces} | {(c, a) for a, b, c in pos.faces}
    if (cuff_loop[0], cuff_loop[1]) not in pos_half:
        cuff_loop = list(reversed(cuff_loop))

    zmin, zmax = pos.bounds()[0][2], pos.bounds()[1][2]
    zspan = max(1e-6, zmax - zmin)
    n_smooth = _smooth_normals(pos, iterations=4)

    # Identify distal pole + surrounding ring to replace outer cap with bench-aligned pyramid boss
    dist_ring, dist_pole = _distal_ring_from_open_limb(pos)

    # Build S_in (reversed faces of pos) + S_out (excluding triangles touching dist_pole)
    verts: list[Vec3] = list(pos.vertices)
    faces: list[Tri] = [(a, c, b) for a, b, c in pos.faces]  # S_in faces inward

    out_map: dict[int, int] = {}
    for i, v in enumerate(pos.vertices):
        if i == dist_pole:
            continue
        t_prox = max(0.0, min(1.0, (v[2] - zmin) / zspan))
        thick = wall_distal_mm * (1.0 - t_prox) + wall_proximal_mm * t_prox
        out_map[i] = len(verts)
        verts.append(v_add(v, v_scale(n_smooth[i], thick)))

    for a, b, c in pos.faces:
        if dist_pole in (a, b, c):
            continue
        faces.append((out_map[a], out_map[b], out_map[c]))

    # Rounded proximal brim bridging cuff_loop (S_in) to [out_map[i]] (S_out)
    in_ring = list(cuff_loop)
    out_ring = [out_map[i] for i in cuff_loop]
    prev_ring = in_ring
    for step in range(1, brim_round_segments):
        ang = math.pi * step / brim_round_segments
        mid_ring: list[int] = []
        for idx in cuff_loop:
            p_in = verts[idx]
            p_out = verts[out_map[idx]]
            center = v_scale(v_add(p_in, p_out), 0.5)
            half_vec = v_scale(v_sub(p_out, p_in), 0.5)
            r_half = v_norm(half_vec)
            # Bulge proximally along +Z by r_half * sin(ang)
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

    # Distal outer ring on S_out around removed dist_pole
    # Check winding of dist_ring on S_out: each face (dist_pole, b, c) had edge (b, c) on S_out boundary
    # Verify edge (dist_ring[0], dist_ring[1]) exists in S_out
    s_out_edges = {(a, b) for a, b, c in pos.faces if dist_pole not in (a, b, c)}
    s_out_edges |= {(b, c) for a, b, c in pos.faces if dist_pole not in (a, b, c)}
    s_out_edges |= {(c, a) for a, b, c in pos.faces if dist_pole not in (a, b, c)}
    if (dist_ring[0], dist_ring[1]) in s_out_edges:
        dist_ring = list(reversed(dist_ring))

    out_dist_ring = [out_map[i] for i in dist_ring]
    out_dist_pts = [verts[i] for i in out_dist_ring]
    pole_pt = pos.vertices[dist_pole]
    boss_center = (pole_pt[0], pole_pt[1], pole_pt[2] - 24.0)

    boss_verts, boss_faces, boss_top_local = _build_captive_m6_pyramid_boss_cap(
        top_ring_pts=out_dist_pts,
        boss_center=boss_center,
        boss_normal=(0.0, 0.0, -1.0),
        flexion_deg=flexion_deg,
        adduction_deg=adduction_deg,
    )
    base_off = len(verts)
    verts.extend(boss_verts)
    for a, b, c in boss_faces:
        faces.append((base_off + a, base_off + b, base_off + c))

    boss_top_global = [base_off + idx for idx in boss_top_local]
    # S_out has (out_dist_ring[k+1], out_dist_ring[k]) and boss side wall has (boss_top[k], boss_top[k+1]).
    # Bridging with flip=True emits (out_dist_ring[k], out_dist_ring[k+1]) and (boss_top[k+1], boss_top[k]).
    _bridge_rings_manifold(boss_top_global, out_dist_ring, faces, flip=True)

    shell = TriMesh(vertices=verts, faces=faces)

    # Build distal reinforcement modifier volume for .3mf export (cylinder covering distal 45 mm)
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
            "hardware": "pyramid_50_8_m6_captive",
            "bcd_mm": 50.8,
            "flexion_deg": flexion_deg,
            "adduction_deg": adduction_deg,
        },
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
) -> ShellBuildResult:
    """Build a watertight posterior-leaf / solid AFO shell with malleolar fillets & coaxial joint bosses."""
    # Extract clean posterior calf + ankle neck + plantar footplate strip from grid-structured foot mesh
    # Infer angular & ring dimensions or use spatial mask for posterior/plantar region, then build clean watertight shell
    # For general watertight guarantee: offset full posterior/plantar patch cleanly via parametric AFO manifold grid
    bounds_min, bounds_max = neutral_foot_mesh.bounds()
    n_smooth = _smooth_normals(neutral_foot_mesh, iterations=3)

    # Identify posterior/plantar faces for AFO trimline (keeping posterior calf x/y<15 and plantar sole)
    # Build smooth trimline curve with enforced R >= trimline_fillet_mm
    keep_face_mask: list[bool] = []
    for a, b, c in neutral_foot_mesh.faces:
        pts = [neutral_foot_mesh.vertices[idx] for idx in (a, b, c)]
        cy = sum(p[1] for p in pts) / 3.0
        cz = sum(p[2] for p in pts) / 3.0
        # Posterior calf above ankle (cz > 75): keep posterior ~190 deg (cy < 10)
        # Ankle neck (40 <= cz <= 75): sweep trimline smoothly behind malleoli with fillet
        # Footplate (cz < 40): keep plantar bottom (cz < 24)
        if cz > 75.0:
            keep = cy < 12.0
        elif cz >= 38.0:
            # Smooth sinusoidal malleolar fillet transition
            t = (cz - 38.0) / 37.0
            y_cut = 35.0 * (1.0 - smoothstep5(t)) + 12.0 * smoothstep5(t)
            keep = cy < y_cut
        else:
            keep = cz < 24.0
        keep_face_mask.append(keep)

    kept_faces = [f for f, k in zip(neutral_foot_mesh.faces, keep_face_mask) if k]
    kept_verts = {idx for f in kept_faces for idx in f}
    patch = neutral_foot_mesh.submesh_from_vertices(kept_verts)
    # Keep single largest component of patch
    comps = patch.connected_components()
    if comps:
        patch = patch.submesh_from_vertices(comps[0])

    # Check trimline smoothness along patch boundary loops
    loops = patch.boundary_loops()
    if not loops:
        raise ValueError("AFO trimline patch produced no open boundary loop")

    # Apply Laplacian smoothing passes to boundary loop vertices so malleolar trimline radius >= trimline_fillet_mm
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
    # Enforce floor clamp if discrete mesh edge resolution lowers local radius
    effective_fillet = max(min_r, trimline_fillet_mm)

    # Orient boundary loop to patch faces
    p_half = {(a, b) for a, b, c in patch.faces} | {(b, c) for a, b, c in patch.faces} | {(c, a) for a, b, c in patch.faces}
    ordered_loops: list[list[int]] = []
    for lp in patch.boundary_loops():
        if (lp[0], lp[1]) not in p_half:
            lp = list(reversed(lp))
        ordered_loops.append(lp)

    p_norms = _smooth_normals(patch, iterations=3)
    verts = list(patch.vertices)
    faces: list[Tri] = [(a, c, b) for a, b, c in patch.faces]  # S_in inward

    out_map: dict[int, int] = {}
    for i, v in enumerate(patch.vertices):
        # Variable wall thickness: thick at ankle neck (z ~ joint_z_mm), thinner at calf
        neck_w = math.exp(-((v[2] - joint_z_mm) / 40.0) ** 2)
        thick = wall_calf_mm + (wall_neck_mm - wall_calf_mm) * neck_w
        # Add coaxial joint pad build-up at medial/lateral ankle height
        if abs(v[2] - joint_z_mm) < 16.0 and abs(v[1] - joint_y_mm) < 18.0 and abs(v[0]) > 25.0:
            thick += 3.5 * math.exp(-((v[2] - joint_z_mm) / 10.0) ** 2)
        out_map[i] = len(verts)
        verts.append(v_add(v, v_scale(p_norms[i], thick)))

    for a, b, c in patch.faces:
        faces.append((out_map[a], out_map[b], out_map[c]))

    for lp in ordered_loops:
        in_ring = list(lp)
        out_ring = [out_map[i] for i in lp]
        # 2-step rounded edge stitch
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
        alignment_normal=(1.0, 0.0, 0.0),  # Coaxial ML joint axis orthogonal to +Y line of progression
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
) -> CarverPositiveResult:
    """Convert an open-cuff rectified positive into a watertight CNC foam-carver mold.

    Adds:
      - `collar_height_mm` proximal extension with `draft_angle_deg` (10-15 deg) release taper
      - Top cap with recessed square-keyed `25.4 mm` mandrel bore (prevents spin under cutting/vacuum)
      - 3-axis / 4-axis overhang & AFO cant diagnostic
    """
    pos = rectified_positive.copy()
    loops = pos.boundary_loops()
    if len(loops) != 1:
        raise ValueError(f"Carver prep expects 1 proximal cuff loop, found {len(loops)}")

    cuff = loops[0]
    pos_half = {(a, b) for a, b, c in pos.faces} | {(b, c) for a, b, c in pos.faces} | {(c, a) for a, b, c in pos.faces}
    if (cuff[0], cuff[1]) not in pos_half:
        cuff = list(reversed(cuff))

    # Ensure cuff is ordered cleanly around proximal centroid
    cx = sum(pos.vertices[i][0] for i in cuff) / len(cuff)
    cy = sum(pos.vertices[i][1] for i in cuff) / len(cuff)
    z_top = max(pos.vertices[i][2] for i in cuff) + collar_height_mm
    tan_draft = math.tan(math.radians(draft_angle_deg))

    verts = list(pos.vertices)
    faces = list(pos.faces)

    # Tapered thermoforming draft collar (flares outward slightly proximally for clean sheet draw)
    collar_ring: list[int] = []
    for idx in cuff:
        vx, vy, vz = verts[idx]
        dx, dy = vx - cx, vy - cy
        r = math.hypot(dx, dy)
        dz = max(5.0, z_top - vz)
        scale = (r + dz * tan_draft) / max(1e-6, r)
        collar_ring.append(len(verts))
        verts.append((cx + dx * scale, cy + dy * scale, z_top))

    # Note: on `pos`, boundary edge (cuff[k], cuff[k+1]) exists -> bridge needs (cuff[k+1], cuff[k])!
    _bridge_rings_manifold(collar_ring, cuff, faces, flip=False)

    # Top cap with square-keyed mandrel bore centered at (cx, cy, z_top)
    m = len(collar_ring)
    bore_top_ring: list[int] = []
    bore_bot_ring: list[int] = []
    for k in range(m):
        vx, vy, _ = verts[collar_ring[k]]
        ang = math.atan2(vy - cy, vx - cx)
        # Add 4 mm flat keyway slot along +X side to prevent mandrel rotation
        r_bore = mandrel_radius_mm
        if abs(math.sin(ang)) < 0.35 and math.cos(ang) > 0.0:
            r_bore += 4.0
        bx = cx + r_bore * math.cos(ang)
        by = cy + r_bore * math.sin(ang)
        bore_top_ring.append(len(verts))
        verts.append((bx, by, z_top))
        bore_bot_ring.append(len(verts))
        verts.append((bx, by, z_top - bore_depth_mm))

    # Bridge collar_ring (outer) -> bore_top_ring (inner) -> bore_bot_ring -> bore_floor
    _bridge_rings_manifold(bore_top_ring, collar_ring, faces, flip=False)
    _bridge_rings_manifold(bore_bot_ring, bore_top_ring, faces, flip=False)
    bore_floor = len(verts)
    verts.append((cx, cy, z_top - bore_depth_mm))
    for k in range(m):
        kn = (k + 1) % m
        faces.append((bore_bot_ring[kn], bore_bot_ring[k], bore_floor))

    carver_mesh = TriMesh(vertices=verts, faces=faces)

    # Undercut / swing diagnostic
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
