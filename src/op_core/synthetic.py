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
        z_top = 135.0
    else:
        z_top = cuff_height_mm
    is_full_knee = z_top > 60.0
    eff_rings = max(n_rings, 76) if is_full_knee else n_rings
    eff_seg = max(n_seg, 72) if is_full_knee else n_seg
    z_bot = -length_mm

    # Rings run from proximal (ring 0) down to near-distal pole (ring eff_rings-1)
    for r in range(eff_rings):
        u = r / (eff_rings - 1)  # 0 at proximal top, 1 near distal tip
        z_start = z_top - (22.0 if not open_cuff else 0.0)
        z = z_start + u * (z_bot + 10.0 - z_start)

        # Base longitudinal radius profile:
        # - Above knee (z > 0): supracondylar narrowing above condyles then quadriceps/thigh flare
        # - Below MPT (z <= 0): taper from proximal brim down to rounded soft-tissue distal end
        if z > 0.0:
            t_thigh = max(0.0, (z - 22.0) / 115.0)
            waist = -4.2 * math.exp(-((z - 52.0) / 16.0) ** 2)
            base_r = pr * (1.0 + 0.28 * (t_thigh ** 0.92)) + waist
        else:
            t_below = max(0.0, min(1.0, -z / max(1e-6, length_mm)))
            base_r = pr * (1.0 - 0.36 * t_below)
            if z < z_bot + 28.0:
                cap_u = max(0.0, min(1.0, (z_bot + 28.0 - z) / 28.0))
                base_r = max(dr * math.sqrt(max(0.02, 1.0 - 0.88 * cap_u * cap_u)), 8.0)

        for s in range(eff_seg):
            theta = 2.0 * math.pi * s / eff_seg  # 0=+X lateral, pi/2=+Y anterior, pi=-X medial, 3pi/2=-Y posterior
            ct = math.cos(theta)
            st = math.sin(theta)
            # Signed angular offset from anterior midline (+Y, theta = pi/2)
            d_ant = math.atan2(-ct, st)

            # Subtle triangular tibia cross-section in proximal-mid shank (z ~ -15..-135)
            tri_w = math.exp(-((z + 65.0) / 60.0) ** 2) if z <= 5.0 else 0.0
            tri_shape = -1.6 * tri_w * math.cos(3.0 * theta)

            # --- Sculpted Anatomical Knee & Patella ---
            # Broad, flatter shield-shaped patella (~46-50 mm wide, wider superiorly, tapering to apex)
            pat_z_rel = (z - 33.0) / 18.5
            pat_ang_w = 0.44 + 0.12 * max(-1.0, min(1.0, (z - 28.0) / 18.0))
            pat_u = math.hypot(d_ant / max(0.20, pat_ang_w), pat_z_rel)
            pat_w = math.exp(-(pat_u ** 3.6) * 1.1) if st > 0.25 else 0.0
            # Low-profile anterior projection compensated across d_ant so the patellar face reads broad & flat
            pat_center_mm = 5.8 if is_full_knee else 4.8
            pat_flat_comp = (12.0 if is_full_knee else 8.0) * (1.0 - math.cos(d_ant))
            patella = (pat_center_mm + pat_flat_comp) * pat_w

            # Peripatellar & infrapatellar gutters framing the broader patella & patellar tendon
            gutter_ang = (
                math.exp(-((d_ant - 0.66) / 0.20) ** 2)
                + math.exp(-((d_ant + 0.66) / 0.20) ** 2)
            )
            gutter_z = math.exp(-((z - 12.0) / 24.0) ** 2)
            knee_gutters = (-3.8 if is_full_knee else -2.2) * gutter_ang * gutter_z

            # Gentle suprapatellar dip above superior pole of patella (z ~ +60 mm, anterior)
            supra_dip = (
                -2.8
                * math.exp(-((d_ant / 0.48) ** 2))
                * math.exp(-((z - 60.0) / 12.0) ** 2)
                if is_full_knee
                else 0.0
            )

            # Patellar tendon cord at MPT (z ~ -12..+14 mm): concave shelf vs patella, narrow midline band
            pt_shelf = (-3.2 if is_full_knee else -2.2) * math.exp(-((d_ant / 0.42) ** 2)) * math.exp(-((z - 2.0) / 9.5) ** 2)
            pt_cord = (2.4 if is_full_knee else 1.0) * math.exp(-((d_ant / 0.14) ** 2)) * math.exp(-((z - 1.0) / 12.0) ** 2)

            # Tibial tubercle prominence (+Y anterior, z ~ -18 mm) right where patellar tendon inserts
            tib_tub = (4.6 if is_full_knee else 3.2) * math.exp(-((d_ant / 0.20) ** 2)) * math.exp(-((z + 18.0) / 11.0) ** 2)

            # Sharp anterior tibial crest ridge (+Y, z ~ -25..-130 mm)
            crest_fade = 1.0 if z > z_bot + 30.0 else max(0.0, (z - z_bot) / 30.0)
            tibial_crest = (
                3.8
                * crest_fade
                * math.exp(-((st - 1.0) ** 2) / 0.055)
                * math.exp(-((z + 75.0) / 52.0) ** 2)
            )

            # Medial & lateral femoral epicondylar flare flanking knee joint (z ~ +24..+28 mm)
            med_condyle = (7.2 if is_full_knee else 5.2) * math.exp(-((ct + 0.98) ** 2 + (st + 0.05) ** 2) / 0.16) * math.exp(-((z - 25.0) / 19.0) ** 2)
            lat_condyle = (6.2 if is_full_knee else 4.6) * math.exp(-((ct - 0.98) ** 2 + (st + 0.05) ** 2) / 0.16) * math.exp(-((z - 27.0) / 19.0) ** 2)

            # Deep popliteal fossa behind the knee (-Y, z ~ +4..-6 mm) framed by hamstring tendons above
            pop_fossa = (-6.8 if is_full_knee else -4.2) * math.exp(-((st + 1.0) ** 2) / 0.16) * math.exp(-((z - 4.0) / 16.0) ** 2)
            ham_tendons = (
                4.2
                * (
                    math.exp(-((st + 0.86) ** 2 + (ct + 0.50) ** 2) / 0.14)
                    + math.exp(-((st + 0.86) ** 2 + (ct - 0.50) ** 2) / 0.14)
                )
                * math.exp(-((z - 44.0) / 34.0) ** 2)
                if is_full_knee
                else 0.0
            )

            # Bi-lobed posterior gastrocnemius calf belly (-Y, z ~ -45..-110 mm)
            post_env = math.exp(-((z + 72.0) / 46.0) ** 2)
            calf_med = 5.8 * math.exp(-((st + 0.92) ** 2 + (ct + 0.38) ** 2) / 0.24) * post_env
            calf_lat = 5.4 * math.exp(-((st + 0.92) ** 2 + (ct - 0.38) ** 2) / 0.24) * post_env

            # Quadriceps VMO teardrop (medial, lower) & vastus lateralis fullness in distal thigh
            vmo = (
                6.2
                * math.exp(-((st - 0.72) ** 2 + (ct + 0.68) ** 2) / 0.22)
                * math.exp(-((z - 76.0) / 34.0) ** 2)
                if is_full_knee
                else 0.0
            )
            quad_vl = (
                5.8
                * math.exp(-((st - 0.68) ** 2 + (ct - 0.72) ** 2) / 0.28)
                * math.exp(-((z - 94.0) / 44.0) ** 2)
                if is_full_knee
                else 0.0
            )

            # Lateral-posterolateral fibular head prominence (+X slightly posterior, z ~ -24 mm)
            fib_head = (
                4.6
                * math.exp(-((ct - 0.96) ** 2 + (st + 0.26) ** 2) / 0.11)
                * math.exp(-((z + 24.0) / 14.0) ** 2)
            )

            rad = max(
                7.0,
                base_r
                + tri_shape
                + patella
                + knee_gutters
                + supra_dip
                + pt_shelf
                + pt_cord
                + tib_tub
                + tibial_crest
                + med_condyle
                + lat_condyle
                + pop_fossa
                + ham_tendons
                + calf_med
                + calf_lat
                + vmo
                + quad_vl
                + fib_head,
            )

            # Physiological knee flexion silhouette: when full knee+thigh is shown, give a clear ~10 deg
            # resting clinical bend so the knee joint and patella project forward naturally.
            if is_full_knee:
                if z > 20.0:
                    y_shift = -0.15 * (z - 20.0)
                else:
                    y_shift = -0.07 * max(0.0, 20.0 - z)
            else:
                if z > 10.0:
                    y_shift = -0.04 * (z - 10.0)
                else:
                    y_shift = -0.05 * max(0.0, -z)
            vertices.append((rad * ct, rad * st + y_shift, z))

    for r in range(eff_rings - 1):
        for s in range(eff_seg):
            sn = (s + 1) % eff_seg
            a = r * eff_seg + s
            b = r * eff_seg + sn
            c = (r + 1) * eff_seg + sn
            d = (r + 1) * eff_seg + s
            # Outward winding (top-to-bottom ring order)
            faces.append((a, d, c))
            faces.append((a, c, b))

    # Close distal apex pole cleanly
    dist_y_shift = -0.07 * (20.0 + length_mm) if is_full_knee else -0.05 * length_mm
    distal_pole_idx = len(vertices)
    vertices.append((0.0, dist_y_shift, z_bot))
    last_ring = (eff_rings - 1) * eff_seg
    for s in range(eff_seg):
        sn = (s + 1) % eff_seg
        faces.append((last_ring + s, distal_pole_idx, last_ring + sn))

    if not open_cuff:
        _close_proximal_dome(vertices, faces, eff_seg, height_mm=22.0, dome_rings=5)

    mesh = TriMesh(vertices=vertices, faces=faces)
    target_coords: dict[str, Vec3] = {
        "mpt": (0.0, pr, 0.0),
        "popliteal": (0.0, -pr, -5.0),
        "fibular_head": (pr + 3.5, -6.0, -24.0),
        "tibial_crest": (0.0, pr * 0.85, -75.0),
        "patella": (0.0, pr + 14.0, 33.0),
        "distal_end": (0.0, dist_y_shift, z_bot),
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
    n_rings: int = 56,
    n_seg: int = 60,
) -> SyntheticLimbCase:
    """Generate an anatomically shaped lower-leg, heel, ankle, and foot mesh.

    Uses a non-self-intersecting external-pivot ankle-heel fan sweep bridging horizontal
    lower-leg rings cleanly into coronal foot cross-sections with a projecting calcaneus,
    Achilles concavity, offset malleoli, flat weight-bearing sole, medial longitudinal arch,
    oblique metatarsal flare, and asymmetrical rounded toe box.
    """
    vertices: list[Vec3] = []
    faces: list[Tri] = []

    def _smooth5(x: float) -> float:
        x = max(0.0, min(1.0, x))
        return x * x * x * (x * (x * 6.0 - 15.0) + 10.0)

    def _spow(val: float, exp: float) -> float:
        return math.copysign(abs(val) ** exp, val)

    scale_f = foot_length_mm / 230.0
    y_piv = 46.0
    z_piv = 84.0
    y_toe_end = 186.0 * scale_f

    n_shank = max(14, n_rings // 3)
    n_fan = max(14, n_rings // 3)
    n_foot = max(18, n_rings - n_shank - n_fan + 2)

    def _apply_deform(pt: Vec3, deform_w: float) -> Vec3:
        if deform_w <= 1e-6:
            return pt
        out = pt
        if plantarflexion_deg != 0.0:
            out = rodrigues_rotate(
                out,
                axis=(1.0, 0.0, 0.0),
                angle_rad=math.radians(-plantarflexion_deg * deform_w),
                origin=(0.0, 0.0, 58.0),
            )
        if inversion_deg != 0.0:
            out = rodrigues_rotate(
                out,
                axis=(0.0, 1.0, 0.0),
                angle_rad=math.radians(inversion_deg * deform_w),
                origin=(0.0, 0.0, 58.0),
            )
        return out

    ring_count = 0

    # --- Section 1: Vertical lower leg (calf belly down to supramalleolar ankle z = z_piv) ---
    for r in range(n_shank):
        t = r / (n_shank - 1)  # 0 at proximal calf top, 1 at z = z_piv (84 mm)
        z = calf_height_mm * (1.0 - t) + z_piv * t
        calf_w = math.exp(-((t - 0.28) / 0.26) ** 2)
        end_fade = 1.0 - _smooth5((t - 0.72) / 0.28)
        y_front = 29.0 + 17.0 * (1.0 - t)
        y_back = -(27.0 + 18.0 * (1.0 - t) + 12.0 * calf_w * end_fade)
        half_w = 31.0 + 14.0 * (1.0 - t) + 6.5 * calf_w * end_fade

        # Smoothly match ankle center/radii at t = 1: y in [-27, +29], x in [-31, +31]
        cy = 0.5 * (y_front + y_back)
        ry = 0.5 * (y_front - y_back)
        rx = half_w

        achilles_w = math.exp(-((t - 0.94) / 0.16) ** 2)
        deform_w = _smooth5((t - 0.85) / 0.15) * 0.15

        for s in range(n_seg):
            theta = 2.0 * math.pi * s / n_seg
            ct, st = math.cos(theta), math.sin(theta)
            # Anterior tibial crest ridge down front of lower shank
            shin_ridge = 3.0 * end_fade * math.exp(-((st - 1.0) ** 2) / 0.06) * math.exp(-((t - 0.42) / 0.34) ** 2)
            # Subtle bi-lobed gastrocnemius calf contour posteriorly
            calf_lobes = (
                4.5
                * calf_w
                * end_fade
                * (
                    math.exp(-((st + 0.92) ** 2 + (ct + 0.38) ** 2) / 0.24)
                    + math.exp(-((st + 0.92) ** 2 + (ct - 0.38) ** 2) / 0.24)
                )
            )
            # Retromalleolar hollows flanking the Achilles tendon near distal shank
            pinch = -4.5 * achilles_w * math.exp(-((abs(ct) - 0.68) ** 2 + (st + 0.72) ** 2) / 0.14)
            ach_ridge = 2.6 * achilles_w * math.exp(-((st + 1.0) ** 2) / 0.06)

            px = (rx + pinch) * ct
            py = cy + (ry + shin_ridge + calf_lobes + ach_ridge) * st
            vertices.append(_apply_deform((px, py, z), deform_w))
        ring_count += 1

    # --- Section 2: External-pivot ankle & heel fan sweep (alpha: 0 -> pi/2 around (0, y_piv, z_piv)) ---
    for r in range(1, n_fan):
        sf = r / (n_fan - 1)  # (0, 1]
        alpha = sf * (math.pi * 0.5)
        ca, sa = math.cos(alpha), math.sin(alpha)
        # e_y points anterior/dorsal (+Y at alpha=0, +Z at alpha=pi/2)
        ey_y, ey_z = ca, sa

        # Distance from external pivot (y_piv, z_piv) to anterior/dorsal skin (always >= 17 mm -> zero self-intersection)
        d_front = 17.0 + 7.0 * _smooth5(sf) + 1.2 * math.sin(math.pi * sf)

        # Posterior Achilles curves out to calcaneal tuberosity (y = -46) and rounds smoothly onto flat sole z = 0
        y_post_wall = -27.0 - 19.0 * _smooth5(sf / 0.44)
        d_wall = (y_piv - y_post_wall) / max(0.04, ca)
        d_floor = z_piv / max(0.04, sa)
        p_norm = 7.5
        d_back = (d_wall ** (-p_norm) + d_floor ** (-p_norm)) ** (-1.0 / p_norm)

        d_mid = 0.5 * (d_front + d_back)
        r_dp = 0.5 * (d_back - d_front)

        # Smooth mediolateral transition from ankle (-31..+31) through heel pad (-32..+33) to midfoot (-31.5..+35.5)
        x_med = -(31.0 + 1.5 * math.sin(math.pi * sf) + 0.5 * _smooth5(sf))
        x_lat = 31.0 + 2.0 * math.sin(math.pi * sf) + 4.5 * _smooth5(sf)
        cx = 0.5 * (x_med + x_lat)
        rx = 0.5 * (x_lat - x_med)

        # Offset anatomical malleoli: medial (-X) higher/anterior; lateral (+X) lower/posterior
        med_mal = 6.2 * math.exp(-((sf - 0.18) / 0.09) ** 2)
        lat_mal = 6.0 * math.exp(-((sf - 0.30) / 0.09) ** 2)
        ach_w = math.exp(-(sf / 0.22) ** 2)
        sole_flat_w = _smooth5((sf - 0.36) / 0.50)
        arch_start_w = _smooth5((sf - 0.72) / 0.28)
        deform_w = 0.15 + 0.85 * _smooth5(sf / 0.50)

        for s in range(n_seg):
            theta = 2.0 * math.pi * s / n_seg
            ct, st = math.cos(theta), math.sin(theta)

            mal_boost = (
                med_mal * math.exp(-((ct + 0.96) ** 2 + (st - 0.16) ** 2) / 0.16)
                + lat_mal * math.exp(-((ct - 0.96) ** 2 + (st + 0.18) ** 2) / 0.16)
            )
            pinch = -4.5 * ach_w * math.exp(-((abs(ct) - 0.68) ** 2 + (st + 0.72) ** 2) / 0.14)
            ach_ridge = 2.6 * ach_w * math.exp(-((st + 1.0) ** 2) / 0.06)

            # Superellipse plantar flattening on heel contact pad & midfoot transition
            st_shaped = _spow(st, 1.0 - 0.28 * sole_flat_w) if st < 0.0 else st
            ct_shaped = _spow(ct, 1.0 - 0.12 * sole_flat_w)

            # Medial longitudinal arch begins rising smoothly near y ~ 46 (sf -> 1.0)
            arch_lift = 0.0
            if st < 0.0 and arch_start_w > 0.0:
                arch_lift = 5.5 * arch_start_w * (max(0.0, -ct) ** 1.15) * (-st)

            # Gentle lateral dorsal slope matching coronal instep at sf = 1
            dorsal_slope = 0.0
            if st > 0.0:
                dorsal_slope = -4.0 * _smooth5((sf - 0.60) / 0.40) * max(0.0, ct) * st

            d_ray = d_mid - (r_dp + ach_ridge) * st_shaped - arch_lift - dorsal_slope
            px = cx + (rx + mal_boost + pinch) * ct_shaped
            py = y_piv - d_ray * ey_y
            pz = z_piv - d_ray * ey_z
            vertices.append(_apply_deform((px, py, pz), deform_w))
        ring_count += 1

    # --- Section 3: Coronal foot slices (midfoot arch -> metatarsal flare -> asymmetrical toe box) ---
    for r in range(1, n_foot):
        tf = r / (n_foot - 1)  # (0, 1] from y = y_piv (46) to y_toe_end
        y_base = y_piv + tf * (y_toe_end - y_piv)

        # Anatomical foot planform (medial -X, lateral +X)
        # Medial border curves slightly inward at arch waist (tf~0.25), flares to 1st MPJ (tf~0.64), full hallux
        medial_waist = -3.0 * math.exp(-((tf - 0.24) / 0.15) ** 2)
        mpj1_flare = 15.0 * math.exp(-((tf - 0.66) / 0.24) ** 2)
        x_med_raw = -(31.5 + medial_waist + mpj1_flare)
        # Lateral border flares at 5th met base/head (tf~0.52), sweeps earlier inward toward 5th toe
        mpj5_flare = 10.5 * math.exp(-((tf - 0.52) / 0.26) ** 2)
        x_lat_raw = 35.5 + mpj5_flare

        # Smooth rounded toe planform taper at distal forefoot (tf > 0.72)
        if tf > 0.72:
            tt = (tf - 0.72) / 0.28
            med_taper = math.sqrt(max(0.06, 1.0 - 0.82 * (tt ** 2.1)))
            lat_taper = math.sqrt(max(0.05, 1.0 - 0.90 * (tt ** 1.75)))
            # Shift toe center slightly medially toward hallux/2nd toe apex
            toe_cx_shift = -8.5 * _smooth5(tt)
            x_med = toe_cx_shift + (x_med_raw - toe_cx_shift) * med_taper
            x_lat = toe_cx_shift + (x_lat_raw - toe_cx_shift) * lat_taper
        else:
            x_med, x_lat = x_med_raw, x_lat_raw

        cx = 0.5 * (x_med + x_lat)
        rx = max(6.0, 0.5 * (x_lat - x_med))

        # Dorsal height slopes from 60 mm at ankle/instep (tf=0) down to 22 mm at met heads and rounded toes
        z_top_base = 60.0 * ((1.0 - tf) ** 1.35) + 21.0 * (1.0 - (1.0 - tf) ** 1.35)
        if tf > 0.80:
            tt_z = (tf - 0.80) / 0.20
            z_top_base *= math.sqrt(max(0.14, 1.0 - 0.78 * tt_z * tt_z))

        # Subtle toe-spring rocker lift at distal toe tips (tf > 0.80)
        z_bot_base = 3.5 * _smooth5((tf - 0.80) / 0.20)
        cz = 0.5 * (z_top_base + z_bot_base)
        rz = max(4.5, 0.5 * (z_top_base - z_bot_base))

        # Medial longitudinal arch cavity peaks around tf ~ 0.24 (y ~ 80 mm), fading cleanly to 0 at 1st MPJ
        arch_env = math.exp(-((tf - 0.22) / 0.20) ** 2)
        arch_amp = 5.5 * (1.0 - _smooth5(tf / 0.14)) + 17.5 * arch_env * _smooth5(tf / 0.12)
        # Subtle lateral longitudinal arch (~2.5 mm)
        lat_arch = 2.5 * math.exp(-((tf - 0.26) / 0.18) ** 2)

        for s in range(n_seg):
            theta = 2.0 * math.pi * s / n_seg
            ct, st = math.cos(theta), math.sin(theta)

            # Superellipse cross-section: flat stable plantar tread, smoothly vaulted dorsum
            st_shaped = _spow(st, 0.72) if st < 0.0 else _spow(st, 0.92)
            ct_shaped = _spow(ct, 0.88)

            px = cx + rx * ct_shaped
            pz = cz + rz * st_shaped

            if st < 0.0:
                # Carve medial longitudinal arch upward on medial-plantar quarter (-X, st < 0)
                med_w = max(0.0, -ct) ** 1.15
                lat_w = max(0.0, ct) ** 1.2
                pz += (arch_amp * med_w + lat_arch * lat_w) * (-st)
            else:
                # Realistic dorsal foot slope: slightly fuller medially over 1st-2nd ray, sloping down laterally
                slope_w = 1.0 - 0.55 * tf
                pz -= 4.0 * slope_w * max(0.0, ct) * st

            # Asymmetrical oblique toe sweep: medial hallux (-X) extends farther forward than lateral 5th toe (+X)
            py = y_base
            if tf > 0.62:
                t_ob = _smooth5((tf - 0.62) / 0.38)
                py += t_ob * scale_f * (-11.0 * max(0.0, ct) ** 1.15 + 3.5 * max(0.0, -ct))

            vertices.append(_apply_deform((px, py, pz), 1.0))
        ring_count += 1

    # Stitch all consecutive rings with consistent outward winding
    for r in range(ring_count - 1):
        for s in range(n_seg):
            sn = (s + 1) % n_seg
            a = r * n_seg + s
            b = r * n_seg + sn
            c = (r + 1) * n_seg + sn
            d = (r + 1) * n_seg + s
            faces.append((a, d, c))
            faces.append((a, c, b))

    # Smooth proximal calf dome closure
    _close_proximal_dome(vertices, faces, n_seg, height_mm=18.0, dome_rings=4)

    # Smooth rounded distal toe-tip dome closure (3 smooth rings -> rounded anatomical toe end)
    last_ring = list(range((ring_count - 1) * n_seg, ring_count * n_seg))
    tcx = sum(vertices[idx][0] for idx in last_ring) / n_seg
    tcy = sum(vertices[idx][1] for idx in last_ring) / n_seg
    tcz = sum(vertices[idx][2] for idx in last_ring) / n_seg
    toe_offsets = [
        (vertices[idx][0] - tcx, vertices[idx][1] - tcy, vertices[idx][2] - tcz)
        for idx in last_ring
    ]
    prev_toe = last_ring
    toe_dome_len = 11.0 * scale_f
    toe_dir_p1 = _apply_deform((0.0, 1.0, 58.0), 1.0)
    toe_dir_p0 = _apply_deform((0.0, 0.0, 58.0), 1.0)
    tdx = toe_dir_p1[0] - toe_dir_p0[0]
    tdy = toe_dir_p1[1] - toe_dir_p0[1]
    tdz = toe_dir_p1[2] - toe_dir_p0[2]
    for k in range(1, 3):
        phi = (k / 3.0) * (math.pi * 0.5)
        r_sc = math.cos(phi)
        ext = toe_dome_len * math.sin(phi)
        next_toe: list[int] = []
        for s in range(n_seg):
            dx, dy, dz = toe_offsets[s]
            pt_raw = (
                tcx + dx * r_sc + tdx * ext,
                tcy + dy * r_sc + tdy * ext,
                tcz + dz * r_sc + tdz * ext,
            )
            next_toe.append(len(vertices))
            vertices.append(pt_raw)
        for s in range(n_seg):
            sn = (s + 1) % n_seg
            a, b = prev_toe[s], prev_toe[sn]
            c, d = next_toe[sn], next_toe[s]
            faces.append((a, d, c))
            faces.append((a, c, b))
        prev_toe = next_toe

    toe_pole = len(vertices)
    vertices.append((tcx + tdx * toe_dome_len, tcy + tdy * toe_dome_len, tcz + tdz * toe_dome_len))
    for s in range(n_seg):
        sn = (s + 1) % n_seg
        faces.append((prev_toe[s], toe_pole, prev_toe[sn]))

    mesh = TriMesh(vertices=vertices, faces=faces)
    target_coords: dict[str, Vec3] = {
        "medial_malleolus": (-36.0, 2.0, 72.0),
        "lateral_malleolus": (36.0, -7.0, 60.0),
        "heel": (0.0, -45.0, 18.0),
        "metatarsal_head": (-10.0, 136.0 * scale_f, 4.0),
        "proximal_calf": (0.0, -52.0, calf_height_mm - 45.0),
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
