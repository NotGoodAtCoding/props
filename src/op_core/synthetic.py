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


def _close_proximal_dome(
    vertices: list[Vec3],
    faces: list[Tri],
    n_seg: int,
    height_mm: float = 22.0,
    dome_rings: int = 4,
) -> None:
    """Close proximal ring (indices 0..n_seg-1) with a smooth rounded ellipsoidal dome."""
    cx = sum(vertices[s][0] for s in range(n_seg)) / n_seg
    cy = sum(vertices[s][1] for s in range(n_seg)) / n_seg
    cz = sum(vertices[s][2] for s in range(n_seg)) / n_seg
    base_offsets = [
        (vertices[s][0] - cx, vertices[s][1] - cy, vertices[s][2] - cz)
        for s in range(n_seg)
    ]
    prev_ring = list(range(n_seg))
    for k in range(1, dome_rings):
        phi = (k / dome_rings) * (math.pi * 0.5)
        r_scale = math.cos(phi)
        z_lift = height_mm * math.sin(phi)
        next_ring: list[int] = []
        for s in range(n_seg):
            dx, dy, dz = base_offsets[s]
            next_ring.append(len(vertices))
            vertices.append((cx + dx * r_scale, cy + dy * r_scale, cz + dz * r_scale + z_lift))
        for s in range(n_seg):
            sn = (s + 1) % n_seg
            a, b = prev_ring[s], prev_ring[sn]
            c, d = next_ring[sn], next_ring[s]
            faces.append((a, b, c))
            faces.append((a, c, d))
        prev_ring = next_ring

    prox_pole_idx = len(vertices)
    vertices.append((cx, cy, cz + height_mm))
    for s in range(n_seg):
        sn = (s + 1) % n_seg
        faces.append((prev_ring[s], prev_ring[sn], prox_pole_idx))


def generate_transtibial_limb(
    length_mm: float = 180.0,
    proximal_radius: float = 54.0,
    distal_radius: float = 34.0,
    cuff_height_mm: float = 35.0,
    open_cuff: bool = True,
    include_knee_thigh: bool = False,
    n_rings: int = 44,
    n_seg: int = 64,
    seed: int = 0,
) -> SyntheticLimbCase:
    """Generate an anatomically contoured transtibial residual limb in canonical frame.

    Canonical coordinates:
      z = 0.0 -> Mid-Patellar Tendon (MPT) plane
      z = -length_mm -> Distal limb apex
      z = +cuff_height_mm (or +125 mm when include_knee_thigh=True) -> Proximal extent
      +Y -> Anterior, +X -> Lateral
    """
    rng = random.Random(seed)
    scale_var = 1.0 + rng.uniform(-0.08, 0.08)
    pr = proximal_radius * scale_var
    dr = distal_radius * scale_var

    vertices: list[Vec3] = []
    faces: list[Tri] = []

    if include_knee_thigh and cuff_height_mm <= 40.0:
        z_top = 125.0
    else:
        z_top = cuff_height_mm
    eff_rings = max(n_rings, 58) if z_top > 60.0 else n_rings
    z_bot = -length_mm

    # Rings run from proximal (ring 0) down to near-distal pole (ring eff_rings-1)
    for r in range(eff_rings):
        u = r / (eff_rings - 1)  # 0 at proximal top, 1 near distal tip
        z_start = z_top - (22.0 if not open_cuff else 0.0)
        z = z_start + u * (z_bot + 10.0 - z_start)

        # Base longitudinal radius profile:
        # - Above knee (z > 0): gradual quadriceps/thigh flare + slight supracondylar waist at z~42
        # - Below MPT (z <= 0): taper from proximal brim down to rounded soft-tissue distal end
        if z > 0.0:
            t_thigh = max(0.0, (z - 15.0) / 110.0)
            waist = -2.8 * math.exp(-((z - 42.0) / 14.0) ** 2)
            base_r = pr * (1.0 + 0.30 * (t_thigh ** 0.92)) + waist
        else:
            t_below = max(0.0, min(1.0, -z / max(1e-6, length_mm)))
            base_r = pr * (1.0 - 0.36 * t_below)
            if z < z_bot + 28.0:
                cap_u = max(0.0, min(1.0, (z_bot + 28.0 - z) / 28.0))
                base_r = max(dr * math.sqrt(max(0.02, 1.0 - 0.88 * cap_u * cap_u)), 8.0)

        for s in range(n_seg):
            theta = 2.0 * math.pi * s / n_seg  # 0=+X lateral, pi/2=+Y anterior, pi=-X medial, 3pi/2=-Y posterior
            ct = math.cos(theta)
            st = math.sin(theta)

            # Subtle triangular tibia cross-section in proximal-mid shank (z ~ -15..-135)
            tri_w = math.exp(-((z + 65.0) / 60.0) ** 2) if z <= 5.0 else 0.0
            tri_shape = -1.6 * tri_w * math.cos(3.0 * theta)

            # Patella prominence (+Y anterior, centered z ~ +28 mm)
            patella = 7.5 * math.exp(-((st - 1.0) ** 2) / 0.14) * math.exp(-((z - 28.0) / 18.0) ** 2)

            # Patellar tendon slight concavity at MPT (z ~ 0..+8 mm) between patella and tibial tubercle
            pt_dip = -2.2 * math.exp(-((st - 1.0) ** 2) / 0.10) * math.exp(-((z - 2.0) / 9.0) ** 2)

            # Tibial tubercle prominence (+Y anterior, z ~ -16 mm)
            tib_tub = 3.2 * math.exp(-((st - 1.0) ** 2) / 0.08) * math.exp(-((z + 16.0) / 11.0) ** 2)

            # Sharp anterior tibial crest ridge (+Y, theta ~ pi/2, z ~ -25..-130 mm)
            crest_fade = 1.0 if z > z_bot + 30.0 else max(0.0, (z - z_bot) / 30.0)
            tibial_crest = (
                3.6
                * crest_fade
                * math.exp(-((st - 1.0) ** 2) / 0.06)
                * math.exp(-((z + 75.0) / 52.0) ** 2)
            )

            # Medial & lateral femoral epicondylar flare around knee joint (z ~ +22 mm)
            med_condyle = 5.2 * math.exp(-((ct + 1.0) ** 2) / 0.16) * math.exp(-((z - 22.0) / 20.0) ** 2)
            lat_condyle = 4.6 * math.exp(-((ct - 1.0) ** 2) / 0.16) * math.exp(-((z - 22.0) / 20.0) ** 2)

            # Popliteal fossa posterior dip behind the knee (-Y, z ~ +4..-10 mm)
            pop_fossa = -4.2 * math.exp(-((st + 1.0) ** 2) / 0.18) * math.exp(-(z / 16.0) ** 2)

            # Bi-lobed posterior gastrocnemius calf belly (-Y, z ~ -45..-110 mm)
            post_env = math.exp(-((z + 72.0) / 46.0) ** 2)
            calf_med = 5.8 * math.exp(-((st + 0.92) ** 2 + (ct + 0.38) ** 2) / 0.24) * post_env
            calf_lat = 5.4 * math.exp(-((st + 0.92) ** 2 + (ct - 0.38) ** 2) / 0.24) * post_env

            # Quadriceps vastus lateralis & medialis fullness in distal thigh (z ~ +45..+120 mm)
            quad_env = math.exp(-((z - 88.0) / 48.0) ** 2) if z > 15.0 else 0.0
            quad_vl = 4.8 * math.exp(-((st - 0.72) ** 2 + (ct - 0.68) ** 2) / 0.32) * quad_env
            quad_vm = 4.2 * math.exp(-((st - 0.72) ** 2 + (ct + 0.68) ** 2) / 0.32) * quad_env

            # Lateral-posterolateral fibular head prominence (+X slightly posterior, z ~ -24 mm)
            fib_head = (
                4.2
                * math.exp(-((ct - 0.96) ** 2 + (st + 0.26) ** 2) / 0.12)
                * math.exp(-((z + 24.0) / 15.0) ** 2)
            )

            rad = max(
                7.0,
                base_r
                + tri_shape
                + patella
                + pt_dip
                + tib_tub
                + tibial_crest
                + med_condyle
                + lat_condyle
                + pop_fossa
                + calf_med
                + calf_lat
                + quad_vl
                + quad_vm
                + fib_head,
            )

            # Physiological flexion curve: slight posterior thigh sweep proximally, mild shank flexion distally
            if z > 10.0:
                y_shift = -0.04 * (z - 10.0)
            else:
                y_shift = -0.05 * max(0.0, -z)
            vertices.append((rad * ct, rad * st + y_shift, z))

    for r in range(eff_rings - 1):
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
    last_ring = (eff_rings - 1) * n_seg
    for s in range(n_seg):
        sn = (s + 1) % n_seg
        faces.append((last_ring + s, distal_pole_idx, last_ring + sn))

    if not open_cuff:
        _close_proximal_dome(vertices, faces, n_seg, height_mm=22.0, dome_rings=5)

    mesh = TriMesh(vertices=vertices, faces=faces)
    target_coords: dict[str, Vec3] = {
        "mpt": (0.0, pr, 0.0),
        "popliteal": (0.0, -pr, -5.0),
        "fibular_head": (pr + 3.5, -6.0, -24.0),
        "tibial_crest": (0.0, pr * 0.85, -75.0),
        "patella": (0.0, pr + 8.0, 28.0),
        "distal_end": (0.0, -0.05 * length_mm, z_bot),
    }
    landmarks = {name: mesh.nearest_vertex(pt) for name, pt in target_coords.items()}
    return SyntheticLimbCase(
        mesh=mesh,
        landmarks=landmarks,
        landmark_coords={k: mesh.vertices[idx] for k, idx in landmarks.items()},
        metadata={
            "device_type": "transtibial_socket",
            "length_mm": length_mm,
            "seed": float(seed),
            "include_knee_thigh": float(1 if z_top > 60.0 else 0),
        },
    )


def generate_transfemoral_limb(
    length_mm: float = 235.0,
    proximal_radius: float = 68.0,
    distal_radius: float = 40.0,
    hip_height_mm: float = 95.0,
    open_cuff: bool = False,
    n_rings: int = 58,
    n_seg: int = 64,
    seed: int = 7,
) -> SyntheticLimbCase:
    """Generate an anatomically contoured transfemoral (TF) residual limb with proximal thigh/gluteal flare.

    Canonical coordinates:
      z = 0.0 -> Ischial/inguinal brim reference plane
      z = +hip_height_mm -> Proximal hip/gluteal continuation
      z = -length_mm -> Distal soft-tissue limb end
      +Y -> Anterior, +X -> Lateral
    """
    rng = random.Random(seed)
    scale_var = 1.0 + rng.uniform(-0.06, 0.06)
    pr = proximal_radius * scale_var
    dr = distal_radius * scale_var

    vertices: list[Vec3] = []
    faces: list[Tri] = []
    z_top = hip_height_mm
    z_bot = -length_mm
    z_start = z_top - (34.0 if not open_cuff else 0.0)

    for r in range(n_rings):
        u = r / (n_rings - 1)
        z = z_start + u * (z_bot + 10.0 - z_start)

        if z >= 0.0:
            t_hip = z / max(1e-6, hip_height_mm)
            base_r = pr * (1.0 + 0.19 * (t_hip ** 1.05))
        else:
            t_len = min(1.0, -z / max(1e-6, length_mm))
            base_r = pr * (1.0 - 0.36 * (t_len ** 0.92))
            if z < z_bot + 32.0:
                cap_u = max(0.0, min(1.0, (z_bot + 32.0 - z) / 32.0))
                base_r = max(base_r * math.sqrt(max(0.03, 1.0 - 0.86 * cap_u * cap_u)), 9.0)

        for s in range(n_seg):
            theta = 2.0 * math.pi * s / n_seg
            ct = math.cos(theta)
            st = math.sin(theta)

            # Gluteal prominence posterolaterally (-Y, +X) proximally (z ~ +10..+85)
            glute = (
                12.5
                * math.exp(-((st + 0.85) ** 2 + (ct - 0.25) ** 2) / 0.45)
                * math.exp(-((z - 48.0) / 42.0) ** 2)
            )
            # Greater trochanter prominence laterally (+X, z ~ +12 mm)
            troch = 6.8 * math.exp(-((ct - 1.0) ** 2) / 0.14) * math.exp(-((z - 12.0) / 24.0) ** 2)
            # Ischial tuberosity / posteromedial shelf (-X/-Y, z ~ +4 mm)
            ischial = 5.2 * math.exp(-((st + 0.78) ** 2 + (ct + 0.62) ** 2) / 0.22) * math.exp(-((z - 4.0) / 18.0) ** 2)
            # Adductor / Scarpa's triangle anterior flattening (+Y/+X medial, z ~ -15 mm)
            scarpa = -4.0 * math.exp(-((st - 0.92) ** 2 + (ct + 0.25) ** 2) / 0.22) * math.exp(-((z + 15.0) / 24.0) ** 2)
            # Quadriceps rectus femoris / vastus lateralis contour along mid-thigh (z ~ -30..-175 mm)
            quad = (
                4.2
                * math.exp(-((st - 0.82) ** 2) / 0.35)
                * math.exp(-((z + 95.0) / 68.0) ** 2)
            )
            # Hamstring posterior fullness (z ~ -40..-165 mm)
            ham = 4.6 * math.exp(-((st + 0.95) ** 2) / 0.38) * math.exp(-((z + 95.0) / 62.0) ** 2)

            rad = max(8.0, base_r + glute + troch + ischial + scarpa + quad + ham)
            # Natural physiological femoral adduction (~3.5 deg inward) + slight anterior hip tilt
            x_shift = -0.04 * max(0.0, -z)
            y_shift = 0.03 * max(0.0, z) - 0.04 * max(0.0, -z)
            vertices.append((rad * ct + x_shift, rad * st + y_shift, z))

    for r in range(n_rings - 1):
        for s in range(n_seg):
            sn = (s + 1) % n_seg
            a = r * n_seg + s
            b = r * n_seg + sn
            c = (r + 1) * n_seg + sn
            d = (r + 1) * n_seg + s
            faces.append((a, d, c))
            faces.append((a, c, b))

    dist_pole_idx = len(vertices)
    vertices.append((-0.04 * length_mm, -0.04 * length_mm, z_bot))
    last_ring_start = (n_rings - 1) * n_seg
    for s in range(n_seg):
        sn = (s + 1) % n_seg
        faces.append((last_ring_start + s, dist_pole_idx, last_ring_start + sn))

    if not open_cuff:
        _close_proximal_dome(vertices, faces, n_seg, height_mm=34.0, dome_rings=6)

    mesh = TriMesh(vertices=vertices, faces=faces)
    target_coords: dict[str, Vec3] = {
        "mpt": (0.0, pr, 0.0),
        "popliteal": (-pr * 0.45, -pr * 0.82, 4.0),
        "fibular_head": (pr + 6.0, 0.0, 12.0),
        "ischial_shelf": (-pr * 0.45, -pr * 0.82, 4.0),
        "scarpas_triangle": (-12.0, pr * 0.88, -15.0),
        "trochanter": (pr + 6.0, 0.0, 12.0),
        "distal_end": (-0.04 * length_mm, -0.04 * length_mm, z_bot),
    }
    landmarks = {name: mesh.nearest_vertex(pt) for name, pt in target_coords.items()}
    return SyntheticLimbCase(
        mesh=mesh,
        landmarks=landmarks,
        landmark_coords={k: mesh.vertices[idx] for k, idx in landmarks.items()},
        metadata={"device_type": "transfemoral_socket", "length_mm": length_mm, "seed": float(seed)},
    )


def generate_foot_ankle_mesh(
    plantarflexion_deg: float = 12.0,
    inversion_deg: float = 5.0,
    calf_height_mm: float = 240.0,
    foot_length_mm: float = 230.0,
    n_rings: int = 48,
    n_seg: int = 56,
) -> SyntheticLimbCase:
    """Generate an anatomically contoured lower-leg + ankle + foot mesh with configurable plantarflexion/inversion."""
    vertices: list[Vec3] = []
    faces: list[Tri] = []

    # Parameter u in [0, 1]: 0..0.54 covers gastrocnemius shank to supramalleolar ankle;
    # 0.54..1.0 curves around calcaneal heel pad through medial arch & metatarsal flare to toe box.
    for r in range(n_rings):
        u = r / (n_rings - 1)
        if u <= 0.54:
            t = u / 0.54
            cx, cy, cz = 0.0, 0.0, calf_height_mm * (1.0 - t) + 52.0 * t
            calf_swell = 7.5 * math.exp(-((t - 0.28) / 0.24) ** 2)
            ankle_narrow = -6.0 * math.exp(-((t - 0.86) / 0.18) ** 2)
            rx = 43.0 * (1.0 - 0.24 * t) + 0.55 * calf_swell + 0.65 * ankle_narrow
            ry = 46.0 * (1.0 - 0.26 * t) + calf_swell + ankle_narrow
            axis_dir = (0.0, 0.0, -1.0)
        else:
            t = (u - 0.54) / 0.46
            ang = min(math.pi * 0.5, t * (math.pi * 0.68))
            cy = -16.0 * math.sin(math.pi * min(1.0, t * 1.15)) + (foot_length_mm * 0.72) * (t ** 1.18)
            cz = 52.0 * math.cos(ang) + 21.0 * (1.0 - math.cos(ang))
            cx = 2.0 * math.sin(math.pi * t)
            heel_pad = 3.5 * math.exp(-((t - 0.16) / 0.12) ** 2)
            met_flare = 10.5 * math.exp(-((t - 0.74) / 0.16) ** 2)
            toe_taper = 1.0 - 0.42 * max(0.0, (t - 0.82) / 0.18) ** 1.4
            rx = (34.0 + heel_pad + met_flare) * toe_taper
            ry = max(9.5, 29.0 * (1.0 - 0.42 * t) - 7.5 * max(0.0, (t - 0.78) / 0.22))
            axis_dir = (0.0, math.sin(ang), -math.cos(ang))

        # Offset medial (-X, higher & slightly anterior) vs lateral (+X, lower & posterolateral) malleoli
        med_mal = 6.2 * math.exp(-((u - 0.51) / 0.038) ** 2)
        lat_mal = 6.0 * math.exp(-((u - 0.545) / 0.038) ** 2)

        e_x = (1.0, 0.0, 0.0)
        e_y = (0.0, -axis_dir[2], axis_dir[1])

        for s in range(n_seg):
            theta = 2.0 * math.pi * s / n_seg
            ct, st = math.cos(theta), math.sin(theta)

            mal_boost = (
                med_mal * math.exp(-((ct + 0.96) ** 2 + (st - 0.14) ** 2) / 0.16)
                + lat_mal * math.exp(-((ct - 0.96) ** 2 + (st + 0.18) ** 2) / 0.16)
            )

            achilles_w = math.exp(-((u - 0.48) / 0.08) ** 2)
            perimalleolar_pinch = -5.8 * achilles_w * math.exp(-((abs(ct) - 0.70) ** 2 + (st + 0.70) ** 2) / 0.14)
            achilles_ridge = 3.2 * achilles_w * math.exp(-((st + 1.0) ** 2) / 0.06)

            shin_ridge = (
                2.8
                * math.exp(-((st - 1.0) ** 2) / 0.06)
                * math.exp(-((u - 0.25) / 0.20) ** 2)
                if u < 0.52
                else 0.0
            )
            calf_post = (
                5.5 * math.exp(-((st + 1.0) ** 2) / 0.30) * math.exp(-((u - 0.16) / 0.13) ** 2)
                if u < 0.45
                else 0.0
            )

            dx = (rx + mal_boost + perimalleolar_pinch) * ct
            dy = (ry + shin_ridge + calf_post + achilles_ridge) * st

            if u > 0.58:
                t_f = (u - 0.58) / 0.42
                if st < 0.0:
                    dy *= 0.72
                    arch_u = math.exp(-((t_f - 0.52) / 0.22) ** 2)
                    medial_w = max(0.0, -ct) ** 1.1
                    dy += 9.5 * arch_u * medial_w * (-st)
                else:
                    dy += 3.5 * math.exp(-((t_f - 0.36) / 0.25) ** 2) * (st ** 1.5) * max(0.0, 0.6 - 0.4 * ct)

            px = cx + dx * e_x[0] + dy * e_y[0]
            py = cy + dx * e_x[1] + dy * e_y[1]
            pz = cz + dx * e_x[2] + dy * e_y[2]

            if 0.53 < u < 0.67 and st < 0.15:
                py -= 9.0 * math.exp(-((u - 0.59) / 0.045) ** 2) * max(0.0, -st + 0.15) * 0.75

            if u > 0.80:
                t_toe = (u - 0.80) / 0.20
                py += t_toe * (-7.5 * ct + 2.5 * max(0.0, 1.0 - abs(ct)))

            pt = (px, py, pz)

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

    # Close proximal calf dome and rounded distal toe cap cleanly
    _close_proximal_dome(vertices, faces, n_seg, height_mm=18.0, dome_rings=4)

    last_ring = (n_rings - 1) * n_seg
    toe_center = (
        sum(vertices[last_ring + s][0] for s in range(n_seg)) / n_seg - 4.0,
        sum(vertices[last_ring + s][1] for s in range(n_seg)) / n_seg + 10.0,
        sum(vertices[last_ring + s][2] for s in range(n_seg)) / n_seg - 1.5,
    )
    toe_pole = len(vertices)
    vertices.append(toe_center)
    for s in range(n_seg):
        sn = (s + 1) % n_seg
        faces.append((last_ring + s, toe_pole, last_ring + sn))

    mesh = TriMesh(vertices=vertices, faces=faces)
    target_coords: dict[str, Vec3] = {
        "medial_malleolus": (-37.0, 4.0, 62.0),
        "lateral_malleolus": (37.0, -5.0, 53.0),
        "heel": (0.0, -28.0, 16.0),
        "metatarsal_head": (0.0, foot_length_mm * 0.56, 8.0),
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
