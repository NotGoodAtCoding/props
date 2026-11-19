"""Procedural clinical benchmark meshes (TT/TF residual limbs, corrupted mobile scans, foot-ankle AFOs)."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from .math3d import Vec3, rodrigues_rotate, v_add
from .mesh import Tri, TriMesh


@dataclass
class SyntheticLimbCase:
    """Procedural limb or foot-ankle test mesh paired with ground-truth landmark indices."""

    mesh: TriMesh
    landmarks: dict[str, int]
    landmark_coords: dict[str, Vec3]
    metadata: dict[str, float | str]


def generate_transtibial_limb(
    length_mm: float = 180.0,
    proximal_radius: float = 54.0,
    distal_radius: float = 34.0,
    cuff_height_mm: float = 35.0,
    open_cuff: bool = True,
    n_rings: int = 44,
    n_seg: int = 64,
    seed: int = 0,
) -> SyntheticLimbCase:
    """Generate an anatomically contoured transtibial residual limb in canonical frame.

    Canonical coordinates:
      z = 0.0 -> Mid-Patellar Tendon (MPT) plane
      z = -length_mm -> Distal limb apex
      z = +cuff_height_mm -> Proximal supracondylar open cuff
      +Y -> Anterior, +X -> Lateral
    """
    rng = random.Random(seed)
    scale_var = 1.0 + rng.uniform(-0.08, 0.08)
    pr = proximal_radius * scale_var
    dr = distal_radius * scale_var

    vertices: list[Vec3] = []
    faces: list[Tri] = []

    z_top = cuff_height_mm
    z_bot = -length_mm
    # Rings run from proximal (ring 0) down to near-distal pole (ring n_rings-1)
    for r in range(n_rings):
        u = r / (n_rings - 1)  # 0 at proximal cuff, 1 near distal tip
        z = z_top + u * (z_bot + 10.0 - z_top)

        # Base longitudinal taper + hemispherical distal closure
        t_len = max(0.0, min(1.0, (z_top - z) / (z_top - z_bot)))
        base_r = pr * (1.0 - 0.35 * t_len)
        if z < z_bot + 28.0:
            cap_u = max(0.0, min(1.0, (z_bot + 28.0 - z) / 28.0))
            base_r = max(dr * math.sqrt(max(0.02, 1.0 - 0.88 * cap_u * cap_u)), 8.0)

        for s in range(n_seg):
            theta = 2.0 * math.pi * s / n_seg  # 0=+X lateral, pi/2=+Y anterior
            # Anatomical contour harmonics:
            # Anterior tibial crest ridge (+Y, theta~pi/2)
            tibial_crest = 3.2 * math.exp(-((math.sin(theta) - 1.0) ** 2) / 0.14) * math.exp(-((z + 75.0) / 55.0) ** 2)
            # Posterior gastrocnemius calf belly (-Y, theta~3pi/2)
            calf_belly = 6.5 * math.exp(-((math.sin(theta) + 1.0) ** 2) / 0.35) * math.exp(-((z + 65.0) / 50.0) ** 2)
            # Lateral fibular head prominence (+X, theta~0, z~-25mm)
            fib_head = 3.5 * math.exp(-((math.cos(theta) - 1.0) ** 2) / 0.10) * math.exp(-((z + 25.0) / 18.0) ** 2)
            # Slight anterior patellar tendon contour near z=0
            pt_ridge = 2.0 * math.exp(-((math.sin(theta) - 1.0) ** 2) / 0.18) * math.exp(-(z / 16.0) ** 2)
            rad = base_r + tibial_crest + calf_belly + fib_head + pt_ridge
            # Mild sagittal flexion drift distally (~3 deg)
            y_shift = -0.05 * max(0.0, -z)
            vertices.append((rad * math.cos(theta), rad * math.sin(theta) + y_shift, z))

    for r in range(n_rings - 1):
        for s in range(n_seg):
            sn = (s + 1) % n_seg
            a = r * n_seg + s
            b = r * n_seg + sn
            c = (r + 1) * n_seg + sn
            d = (r + 1) * n_seg + s
            # Outward winding (top-to-bottom ring order)
            faces.append((a, d, c))
            faces.append((a, c, b))

    # Close distal apex pole cleanly
    distal_pole_idx = len(vertices)
    vertices.append((0.0, -0.05 * length_mm, z_bot))
    last_ring = (n_rings - 1) * n_seg
    for s in range(n_seg):
        sn = (s + 1) % n_seg
        faces.append((last_ring + s, distal_pole_idx, last_ring + sn))

    if not open_cuff:
        prox_pole_idx = len(vertices)
        vertices.append((0.0, 0.0, z_top))
        for s in range(n_seg):
            sn = (s + 1) % n_seg
            faces.append((s, sn, prox_pole_idx))

    mesh = TriMesh(vertices=vertices, faces=faces)
    target_coords: dict[str, Vec3] = {
        "mpt": (0.0, pr, 0.0),
        "popliteal": (0.0, -pr, -5.0),
        "fibular_head": (pr + 3.0, 0.0, -25.0),
        "tibial_crest": (0.0, pr * 0.85, -75.0),
        "distal_end": (0.0, -0.05 * length_mm, z_bot),
    }
    landmarks = {name: mesh.nearest_vertex(pt) for name, pt in target_coords.items()}
    return SyntheticLimbCase(
        mesh=mesh,
        landmarks=landmarks,
        landmark_coords={k: mesh.vertices[idx] for k, idx in landmarks.items()},
        metadata={"device_type": "transtibial_socket", "length_mm": length_mm, "seed": float(seed)},
    )


def generate_foot_ankle_mesh(
    plantarflexion_deg: float = 12.0,
    inversion_deg: float = 5.0,
    calf_height_mm: float = 240.0,
    foot_length_mm: float = 230.0,
    n_rings: int = 38,
    n_seg: int = 48,
) -> SyntheticLimbCase:
    """Generate an L-shaped lower-leg + ankle + foot mesh with configurable plantarflexion/inversion."""
    vertices: list[Vec3] = []
    faces: list[Tri] = []

    # Build continuous tube from proximal calf (z = calf_height_mm) down into ankle (z=50),
    # curving smoothly through calcaneus (z=0, y=0) forward to metatarsals/toes (z=18, y=+foot_length_mm*0.75).
    # Parameter u in [0, 1]: 0..0.55 is shank, 0.55..1.0 is heel-to-toe forefoot.
    for r in range(n_rings):
        u = r / (n_rings - 1)
        if u <= 0.55:
            t = u / 0.55
            cx, cy, cz = 0.0, 0.0, calf_height_mm * (1.0 - t) + 52.0 * t
            rx = 48.0 * (1.0 - 0.30 * t)
            ry = 52.0 * (1.0 - 0.32 * t)
            axis_dir = (0.0, 0.0, -1.0)
        else:
            t = (u - 0.55) / 0.45
            # Smooth quarter-turn from ankle (0,0,52) through heel (0,-12,26) to forefoot (0,+165,24)
            ang = t * (math.pi * 0.5)
            cy = -10.0 * math.sin(math.pi * t) + (foot_length_mm * 0.72) * (t ** 1.15)
            cz = 52.0 * math.cos(ang) + 22.0 * (1.0 - math.cos(ang))
            cx = 0.0
            rx = 34.0 + 11.0 * math.sin(math.pi * 0.7 * t)
            ry = 26.0 - 8.0 * t
            # Local tangent tilts from -Z toward +Y
            axis_dir = (0.0, math.sin(ang), -math.cos(ang))

        # Malleolar prominences around u ~ 0.55..0.62
        malleolar = 5.5 * math.exp(-((u - 0.57) / 0.05) ** 2)

        # Build local orthonormal ring frame perpendicular to axis_dir
        # Since axis_dir lies in YZ plane, X=(1,0,0) is always lateral/medial
        e_x = (1.0, 0.0, 0.0)
        # e_y = cross(axis_dir, e_x)
        e_y = (0.0, -axis_dir[2], axis_dir[1])

        for s in range(n_seg):
            theta = 2.0 * math.pi * s / n_seg
            ct, st = math.cos(theta), math.sin(theta)
            mal_boost = malleolar * (ct ** 4)
            # Rounded anatomical plantar arch vs dorsal instep
            px = cx + (rx + mal_boost) * ct * e_x[0] + ry * st * e_y[0]
            py = cy + (rx + mal_boost) * ct * e_x[1] + ry * st * e_y[1]
            pz = cz + (rx + mal_boost) * ct * e_x[2] + ry * st * e_y[2]
            pt = (px, py, pz)

            # Apply plantarflexion + inversion smoothly below ankle joint (z < 70)
            if u > 0.45:
                w = min(1.0, (u - 0.45) / 0.25)
                if plantarflexion_deg != 0.0:
                    pt = rodrigues_rotate(
                        pt,
                        axis=(1.0, 0.0, 0.0),
                        angle_rad=math.radians(-plantarflexion_deg * w),
                        origin=(0.0, 0.0, 58.0),
                    )
                if inversion_deg != 0.0:
                    pt = rodrigues_rotate(
                        pt,
                        axis=(0.0, 1.0, 0.0),
                        angle_rad=math.radians(inversion_deg * w),
                        origin=(0.0, 0.0, 58.0),
                    )
            vertices.append(pt)

    for r in range(n_rings - 1):
        for s in range(n_seg):
            sn = (s + 1) % n_seg
            a = r * n_seg + s
            b = r * n_seg + sn
            c = (r + 1) * n_seg + sn
            d = (r + 1) * n_seg + s
            faces.append((a, d, c))
            faces.append((a, c, b))

    # Close toe end
    toe_center = (
        sum(vertices[(n_rings - 1) * n_seg + s][0] for s in range(n_seg)) / n_seg,
        sum(vertices[(n_rings - 1) * n_seg + s][1] for s in range(n_seg)) / n_seg + 8.0,
        sum(vertices[(n_rings - 1) * n_seg + s][2] for s in range(n_seg)) / n_seg,
    )
    toe_pole = len(vertices)
    vertices.append(toe_center)
    last_ring = (n_rings - 1) * n_seg
    for s in range(n_seg):
        sn = (s + 1) % n_seg
        faces.append((last_ring + s, toe_pole, last_ring + sn))

    mesh = TriMesh(vertices=vertices, faces=faces)
    target_coords: dict[str, Vec3] = {
        "medial_malleolus": (-38.0, 0.0, 58.0),
        "lateral_malleolus": (38.0, 0.0, 54.0),
        "heel": (0.0, -25.0, 18.0),
        "metatarsal_head": (0.0, foot_length_mm * 0.55, 10.0),
        "proximal_calf": (0.0, -45.0, calf_height_mm - 20.0),
    }
    landmarks = {name: mesh.nearest_vertex(pt) for name, pt in target_coords.items()}
    return SyntheticLimbCase(
        mesh=mesh,
        landmarks=landmarks,
        landmark_coords={k: mesh.vertices[idx] for k, idx in landmarks.items()},
        metadata={
            "device_type": "afo",
            "plantarflexion_deg": plantarflexion_deg,
            "inversion_deg": inversion_deg,
        },
    )


def corrupt_scan(
    mesh: TriMesh,
    tilt_deg: float = 0.0,
    scale_factor: float = 1.0,
    jitter_mm: float = 0.0,
    seam_step_mm: float = 0.0,
    double_wall: bool = False,
    floaters: int = 0,
    tear_hole: bool = False,
    seed: int = 42,
) -> TriMesh:
    """Inject realistic mobile-scan artifacts for Scan QA testing."""
    rng = random.Random(seed)
    verts = list(mesh.vertices)
    faces = list(mesh.faces)

    if tear_hole and len(faces) > 60:
        # Remove a localized patch around mid-lateral shaft to create an internal tear
        mid_z = -80.0
        faces = [
            f
            for f in faces
            if not all(
                verts[idx][0] > 20.0 and abs(verts[idx][2] - mid_z) < 16.0
                for idx in f
            )
        ]

    if seam_step_mm > 0.0:
        for i, v in enumerate(verts):
            if v[0] > 0.0 and v[1] > 0.0 and v[2] < -10.0:
                r = math.hypot(v[0], v[1])
                if r > 1e-6:
                    verts[i] = (
                        v[0] + seam_step_mm * (v[0] / r),
                        v[1] + seam_step_mm * (v[1] / r),
                        v[2],
                    )

    if jitter_mm > 0.0:
        verts = [
            (
                v[0] + rng.gauss(0.0, jitter_mm * 0.4),
                v[1] + rng.gauss(0.0, jitter_mm * 0.4),
                v[2] + rng.gauss(0.0, jitter_mm * 0.4),
            )
            for v in verts
        ]

    if double_wall:
        # Duplicate a 120-face lateral patch offset 0.8 mm outward to mimic SLAM overlap
        patch_faces = [f for f in faces if verts[f[0]][0] > 18.0 and -110.0 < verts[f[0]][2] < -40.0][:120]
        base_idx = len(verts)
        remap: dict[int, int] = {}
        for a, b, c in patch_faces:
            new_tri = []
            for idx in (a, b, c):
                if idx not in remap:
                    remap[idx] = len(verts)
                    p = verts[idx]
                    verts.append((p[0] + 0.8, p[1], p[2]))
                new_tri.append(remap[idx])
            faces.append((new_tri[0], new_tri[1], new_tri[2]))

    if floaters > 0:
        for k in range(floaters):
            ox, oy, oz = 140.0 + k * 25.0, -80.0, -60.0 + k * 20.0
            b = len(verts)
            verts.extend([
                (ox, oy, oz),
                (ox + 8.0, oy, oz),
                (ox, oy + 8.0, oz),
                (ox, oy, oz + 8.0),
            ])
            faces.extend([
                (b, b + 2, b + 1),
                (b, b + 1, b + 3),
                (b + 1, b + 2, b + 3),
                (b + 2, b, b + 3),
            ])

    if scale_factor != 1.0:
        verts = [(v[0] * scale_factor, v[1] * scale_factor, v[2] * scale_factor) for v in verts]

    if tilt_deg != 0.0:
        rad = math.radians(tilt_deg)
        verts = [rodrigues_rotate(v, axis=(1.0, 0.25, 0.0), angle_rad=rad) for v in verts]

    return TriMesh(vertices=verts, faces=faces)
