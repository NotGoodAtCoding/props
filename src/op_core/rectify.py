"""Project 2: Reference-domain (M_0) geodesic displacement stack & AFO oblique ankle articulation."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .math3d import Vec3, rodrigues_rotate, smoothstep5, v_add, v_dot, v_scale, v_sub, v_unit
from .mesh import TriMesh
from .recipe import OpRecipe


@dataclass
class RectificationResult:
    """Outputs of applying an `.oprecipe.json` stack in reference domain M_0."""

    rectified_mesh: TriMesh
    base_compressed_mesh: TriMesh
    relief_field_mm: list[float]
    resolved_landmarks: dict[str, int]


def _zone_weight(z: float, z_min: float, z_max: float, transition_mm: float) -> float:
    """C^2 smooth window equal to 1 inside [z_min, z_max] and 0 outside transition."""
    if z < z_min - transition_mm or z > z_max + transition_mm:
        return 0.0
    w = 1.0
    if transition_mm > 1e-6:
        if z < z_min:
            w *= smoothstep5((z - (z_min - transition_mm)) / transition_mm)
        elif z > z_max:
            w *= smoothstep5(((z_max + transition_mm) - z) / transition_mm)
    return w


def _slice_centerline_xy(mesh: TriMesh, z_bins: int = 24) -> tuple[float, float, list[tuple[float, float, float]]]:
    zmin, zmax = mesh.bounds()[0][2], mesh.bounds()[1][2]
    span = max(1e-6, zmax - zmin)
    buckets: list[list[Vec3]] = [[] for _ in range(z_bins)]
    for v in mesh.vertices:
        idx = min(z_bins - 1, max(0, int((v[2] - zmin) / span * z_bins)))
        buckets[idx].append(v)
    centers: list[tuple[float, float, float]] = []
    last_cx, last_cy = 0.0, 0.0
    for b in range(z_bins):
        zb = zmin + (b + 0.5) / z_bins * span
        if buckets[b]:
            last_cx = sum(p[0] for p in buckets[b]) / len(buckets[b])
            last_cy = sum(p[1] for p in buckets[b]) / len(buckets[b])
        centers.append((zb, last_cx, last_cy))
    return zmin, span, centers


def _interp_center_xy(z: float, zmin: float, span: float, centers: list[tuple[float, float, float]]) -> tuple[float, float]:
    t = max(0.0, min(len(centers) - 1.0, (z - zmin) / span * len(centers) - 0.5))
    i0 = int(math.floor(t))
    i1 = min(len(centers) - 1, i0 + 1)
    frac = t - i0
    cx = centers[i0][1] * (1.0 - frac) + centers[i1][1] * frac
    cy = centers[i0][2] * (1.0 - frac) + centers[i1][2] * frac
    return cx, cy


def apply_afo_posture(
    mesh: TriMesh,
    dorsiflexion_deg: float = 12.0,
    eversion_deg: float = 5.0,
    heel_pitch_mm: float = 10.0,
    ankle_origin: Vec3 = (0.0, 0.0, 58.0),
) -> TriMesh:
    """Smooth biharmonic/quintic ankle articulation + plantar sole flattening at footwear heel pitch."""
    # Oblique talocrural axis (~8 deg distal-lateral inclination)
    talocrural_axis = v_unit((1.0, -0.05, -0.14))
    subtalar_axis = v_unit((0.0, 1.0, 0.15))

    verts_rot: list[Vec3] = []
    for p in mesh.vertices:
        # Smooth blend across z in [35, 85] mm (and y-distance below ankle)
        w_z = 1.0 - smoothstep5((p[2] - 35.0) / 50.0)
        pt = p
        if w_z > 1e-6:
            if dorsiflexion_deg != 0.0:
                pt = rodrigues_rotate(
                    pt,
                    axis=talocrural_axis,
                    angle_rad=math.radians(dorsiflexion_deg * w_z),
                    origin=ankle_origin,
                )
            if eversion_deg != 0.0:
                pt = rodrigues_rotate(
                    pt,
                    axis=subtalar_axis,
                    angle_rad=math.radians(-eversion_deg * w_z),
                    origin=ankle_origin,
                )
        verts_rot.append(pt)

    # Plantar sole flattening at prescribed footwear heel-height pitch:
    # Heel sits at z_heel = heel_pitch_mm above forefoot contact plane z_fore = 0
    z_vals = [v[2] for v in verts_rot if v[1] > -35.0]
    z_floor_ref = min(z_vals) if z_vals else 0.0
    y_heel = -15.0
    y_met = 125.0

    verts_flat: list[Vec3] = []
    for x, y, z in verts_rot:
        t_y = max(0.0, min(1.0, (y - y_heel) / max(1.0, y_met - y_heel)))
        # Footwear sole plane drops linearly from heel_pitch_mm to 0 at met heads
        sole_target_z = z_floor_ref + heel_pitch_mm * (1.0 - t_y)
        if z < sole_target_z + 2.5 and y > -40.0:
            # Smoothly project lowest plantar points onto sole plane without collapsing dorsal arch
            if z < sole_target_z:
                z = sole_target_z
            else:
                blend = 1.0 - smoothstep5((z - sole_target_z) / 2.5)
                z = z * (1.0 - 0.35 * blend) + sole_target_z * (0.35 * blend)
        verts_flat.append((x, y, z))

    return TriMesh(vertices=verts_flat, faces=list(mesh.faces))


def apply_recipe(canonical_mesh: TriMesh, recipe: OpRecipe) -> RectificationResult:
    """Apply .oprecipe.json deterministically in the undeformed reference domain M_0.

    1. Apply optional pre-rectification AFO posture correction & scale.
    2. Compute fixed smoothed normals N_0 and landmark geodesic distances d_k(v) on M_0.
    3. Compute slice-wise base radial compression + liner offset -> base_compressed_mesh.
    4. Add localized geodesic relief kernels along fixed N_0 -> rectified_mesh.
    """
    recipe.validate()
    m0 = canonical_mesh.copy()
    if recipe.scale_factor != 1.0:
        sf = recipe.scale_factor
        m0 = m0.transformed(lambda p: (p[0] * sf, p[1] * sf, p[2] * sf))

    for op in recipe.operations:
        if op["op"] == "afo_posture":
            m0 = apply_afo_posture(
                m0,
                dorsiflexion_deg=float(op.get("dorsiflexion_deg", 12.0)),
                eversion_deg=float(op.get("eversion_deg", 5.0)),
                heel_pitch_mm=float(op.get("heel_pitch_mm", 10.0)),
            )

    resolved_landmarks = recipe.resolve_landmarks(m0) if recipe.landmarks else {}
    n0 = m0.vertex_normals()
    zmin, span, centers = _slice_centerline_xy(m0)
    n_verts = len(m0.vertices)

    # Step A: Evaluate radial scale percentage per vertex in reference M_0 Z coordinates
    pct_field = [0.0] * n_verts
    for op in recipe.operations:
        kind = op["op"]
        if kind in ("global_compression", "zoned_compression"):
            z_lo, z_hi = op.get("z_range_mm", [-1e6, 1e6])
            trans = float(op.get("transition_mm", 20.0))
            pct = float(op["percent"])
            for i, v in enumerate(m0.vertices):
                w = _zone_weight(v[2], z_lo, z_hi, trans)
                if w > 0.0:
                    pct_field[i] += pct * w

    base_verts: list[Vec3] = []
    liner = float(recipe.liner_thickness_mm)
    for i, (x, y, z) in enumerate(m0.vertices):
        cx, cy = _interp_center_xy(z, zmin, span, centers)
        scale = 1.0 + pct_field[i] / 100.0
        bx = cx + (x - cx) * scale
        by = cy + (y - cy) * scale
        bz = z
        if liner != 0.0:
            # Subtract or offset liner thickness radially along reference normal
            nx, ny, nz = n0[i]
            bx -= liner * nx
            by -= liner * ny
            bz -= liner * nz
        base_verts.append((bx, by, bz))

    base_mesh = TriMesh(vertices=base_verts, faces=list(m0.faces))

    # Step B: Evaluate localized geodesic reliefs strictly in M_0 domain along fixed N_0
    relief_mm = [0.0] * n_verts
    geodesic_cache: dict[int, list[float]] = {}
    for op in recipe.operations:
        if op["op"] == "landmark_relief":
            lm_name = op["landmark"]
            if lm_name not in resolved_landmarks:
                raise KeyError(f"Landmark {lm_name!r} not found in recipe landmarks")
            seed_idx = resolved_landmarks[lm_name]
            if seed_idx not in geodesic_cache:
                geodesic_cache[seed_idx] = m0.geodesic_distances([seed_idx])
            dists = geodesic_cache[seed_idx]
            offset = float(op["offset_mm"])
            radius = max(1.0, float(op.get("radius_mm", 22.0)))
            sigma = radius * 0.48
            inv_2s2 = 0.5 / (sigma * sigma)
            for i, d in enumerate(dists):
                if d < radius * 2.2:
                    relief_mm[i] += offset * math.exp(-d * d * inv_2s2)

    final_verts = [
        v_add(base_verts[i], v_scale(n0[i], relief_mm[i]))
        for i in range(n_verts)
    ]
    return RectificationResult(
        rectified_mesh=TriMesh(vertices=final_verts, faces=list(m0.faces)),
        base_compressed_mesh=base_mesh,
        relief_field_mm=relief_mm,
        resolved_landmarks=resolved_landmarks,
    )
