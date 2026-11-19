"""Pure-Python 3D vector, matrix, eigensolver, and 2D convex-hull primitives."""

from __future__ import annotations

import math
from typing import Iterable

Vec3 = tuple[float, float, float]
Vec2 = tuple[float, float]


def v_add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def v_sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def v_scale(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def v_dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def v_cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def v_norm(a: Vec3) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def v_dist(a: Vec3, b: Vec3) -> float:
    return v_norm(v_sub(a, b))


def v_unit(a: Vec3, fallback: Vec3 = (0.0, 0.0, 1.0)) -> Vec3:
    n = v_norm(a)
    if n < 1e-12:
        return fallback
    inv = 1.0 / n
    return (a[0] * inv, a[1] * inv, a[2] * inv)


def mat3_mul_vec(m: tuple[Vec3, Vec3, Vec3], v: Vec3) -> Vec3:
    return (v_dot(m[0], v), v_dot(m[1], v), v_dot(m[2], v))


def rodrigues_rotate(v: Vec3, axis: Vec3, angle_rad: float, origin: Vec3 = (0.0, 0.0, 0.0)) -> Vec3:
    """Rotate point/vector around unit `axis` passing through `origin`."""
    k = v_unit(axis)
    p = v_sub(v, origin)
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    k_dot_p = v_dot(k, p)
    k_cross_p = v_cross(k, p)
    rotated = (
        p[0] * c + k_cross_p[0] * s + k[0] * k_dot_p * (1.0 - c),
        p[1] * c + k_cross_p[1] * s + k[1] * k_dot_p * (1.0 - c),
        p[2] * c + k_cross_p[2] * s + k[2] * k_dot_p * (1.0 - c),
    )
    return v_add(rotated, origin)


def jacobi_eigen_3x3(cov: list[list[float]], max_iter: int = 50) -> tuple[list[float], list[Vec3]]:
    """Compute eigenvalues and eigenvectors of a symmetric 3x3 matrix via Jacobi rotations.

    Returns eigenvalues sorted descending alongside their unit eigenvectors.
    """
    a = [[cov[i][j] for j in range(3)] for i in range(3)]
    v = [[1.0 if i == j else 0.0 for j in range(3)] for i in range(3)]

    for _ in range(max_iter):
        p, q = 0, 1
        max_off = abs(a[0][1])
        for i, j in ((0, 2), (1, 2)):
            if abs(a[i][j]) > max_off:
                max_off = abs(a[i][j])
                p, q = i, j
        if max_off < 1e-12:
            break

        if abs(a[p][p] - a[q][q]) < 1e-15:
            theta = math.pi / 4.0
        else:
            theta = 0.5 * math.atan2(2.0 * a[p][q], a[q][q] - a[p][p])
        c = math.cos(theta)
        s = math.sin(theta)

        app, aqq, apq = a[p][p], a[q][q], a[p][q]
        a[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq
        a[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq
        a[p][q] = a[q][p] = 0.0

        for r in range(3):
            if r != p and r != q:
                arp, arq = a[r][p], a[r][q]
                a[r][p] = a[p][r] = c * arp - s * arq
                a[r][q] = a[q][r] = s * arp + c * arq

        for r in range(3):
            vrp, vrq = v[r][p], v[r][q]
            v[r][p] = c * vrp - s * vrq
            v[r][q] = s * vrp + c * vrq

    pairs = [
        (a[i][i], v_unit((v[0][i], v[1][i], v[2][i])))
        for i in range(3)
    ]
    pairs.sort(key=lambda item: item[0], reverse=True)
    return [p[0] for p in pairs], [p[1] for p in pairs]


def smoothstep5(t: float) -> float:
    """C^2-continuous quintic smoothstep on [0, 1]."""
    t = max(0.0, min(1.0, t))
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)


def convex_hull_2d(points: Iterable[Vec2]) -> list[Vec2]:
    """Return 2D convex hull vertices in CCW order using Andrew's monotone chain."""
    pts = sorted(set((round(x, 6), round(y, 6)) for x, y in points))
    if len(pts) <= 2:
        return pts

    def cross(o: Vec2, a: Vec2, b: Vec2) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[Vec2] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0.0:
            lower.pop()
        lower.append(p)

    upper: list[Vec2] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0.0:
            upper.pop()
        upper.append(p)

    return lower[:-1] + upper[:-1]


def polygon_perimeter_2d(poly: list[Vec2], closed: bool = True) -> float:
    """Compute perimeter of a 2D polygon."""
    if len(poly) < 2:
        return 0.0
    total = 0.0
    limit = len(poly) if closed else len(poly) - 1
    for i in range(limit):
        p0 = poly[i]
        p1 = poly[(i + 1) % len(poly)]
        total += math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    return total


def polygon_area_2d(poly: list[Vec2]) -> float:
    """Compute unsigned area of a closed 2D polygon via shoelace formula."""
    if len(poly) < 3:
        return 0.0
    s = 0.0
    for i in range(len(poly)):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % len(poly)]
        s += x0 * y1 - x1 * y0
    return 0.5 * abs(s)
