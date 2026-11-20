"""Mesh repair utilities for real-world O&P scans: welding, winding fix, hole filling, and plane clipping."""

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass, field

from .math3d import Vec3, v_add, v_cross, v_dot, v_norm, v_scale, v_sub
from .mesh import Tri, TriMesh


@dataclass
class RepairReport:
    """Summary of automated mesh cleanup and topological repairs."""

    welded_vertices_removed: int
    degenerate_faces_removed: int
    flipped_faces: int
    holes_filled: int
    clipped_proximal_cuff: bool
    warnings: list[str] = field(default_factory=list)


def weld_and_clean(mesh: TriMesh, tol: float = 1e-4) -> tuple[TriMesh, int, int]:
    """Weld near-coincident vertices and drop degenerate or duplicate triangles."""
    if not mesh.vertices or not mesh.faces:
        return mesh.copy(), 0, 0

    inv = 1.0 / max(1e-9, tol)
    vmap: dict[tuple[int, int, int], int] = {}
    new_verts: list[Vec3] = []
    old_to_new: list[int] = []
    for x, y, z in mesh.vertices:
        key = (round(x * inv), round(y * inv), round(z * inv))
        if key not in vmap:
            vmap[key] = len(new_verts)
            new_verts.append((x, y, z))
        old_to_new.append(vmap[key])

    welded_removed = len(mesh.vertices) - len(new_verts)

    seen_tris: set[frozenset[int]] = set()
    clean_faces: list[Tri] = []
    degen_removed = 0
    for a, b, c in mesh.faces:
        na, nb, nc = old_to_new[a], old_to_new[b], old_to_new[c]
        if na == nb or nb == nc or nc == na:
            degen_removed += 1
            continue
        area2 = v_norm(v_cross(v_sub(new_verts[nb], new_verts[na]), v_sub(new_verts[nc], new_verts[na])))
        if area2 < 1e-10:
            degen_removed += 1
            continue
        fkey = frozenset((na, nb, nc))
        if fkey in seen_tris:
            degen_removed += 1
            continue
        seen_tris.add(fkey)
        clean_faces.append((na, nb, nc))

    # Strip unreferenced vertices
    used = sorted({idx for f in clean_faces for idx in f})
    if len(used) < len(new_verts):
        compact_map = {old: idx for idx, old in enumerate(used)}
        final_verts = [new_verts[old] for old in used]
        final_faces = [(compact_map[a], compact_map[b], compact_map[c]) for a, b, c in clean_faces]
        return TriMesh(vertices=final_verts, faces=final_faces), welded_removed, degen_removed

    return TriMesh(vertices=new_verts, faces=clean_faces), welded_removed, degen_removed


def fix_consistent_winding(mesh: TriMesh) -> tuple[TriMesh, int]:
    """Enforce consistent outward face winding across every connected component via BFS."""
    if not mesh.faces:
        return mesh.copy(), 0

    faces = list(mesh.faces)
    verts = mesh.vertices
    n_faces = len(faces)

    edge_to_faces: dict[tuple[int, int], list[int]] = defaultdict(list)
    for fi, (a, b, c) in enumerate(faces):
        for u, v in ((a, b), (b, c), (c, a)):
            k = (u, v) if u < v else (v, u)
            edge_to_faces[k].append(fi)

    visited = [False] * n_faces
    flipped_total = 0

    for start in range(n_faces):
        if visited[start]:
            continue
        comp_faces: list[int] = []
        visited[start] = True
        q = deque([start])
        while q:
            fi = q.popleft()
            comp_faces.append(fi)
            a, b, c = faces[fi]
            for u, v in ((a, b), (b, c), (c, a)):
                k = (u, v) if u < v else (v, u)
                for nb in edge_to_faces[k]:
                    if visited[nb]:
                        continue
                    na, nb_v, nc = faces[nb]
                    # Neighbor shares undirected edge {u,v}. For consistent winding,
                    # neighbor must contain directed edge (v, u). If it has (u, v), flip it.
                    nb_edges = ((na, nb_v), (nb_v, nc), (nc, na))
                    if (u, v) in nb_edges:
                        faces[nb] = (na, nc, nb_v)
                        flipped_total += 1
                    visited[nb] = True
                    q.append(nb)

        # Orient component outward using signed volume if closed, else radial centroid dot normal
        comp_half = set()
        comp_undir: dict[tuple[int, int], int] = defaultdict(int)
        for fi in comp_faces:
            a, b, c = faces[fi]
            for u, v in ((a, b), (b, c), (c, a)):
                comp_half.add((u, v))
                comp_undir[(u, v) if u < v else (v, u)] += 1

        has_boundary = any(cnt == 1 for cnt in comp_undir.values())
        flip_comp = False
        if not has_boundary:
            vol = 0.0
            for fi in comp_faces:
                a, b, c = faces[fi]
                vol += v_dot(verts[a], v_cross(verts[b], verts[c]))
            flip_comp = vol < 0.0
        else:
            comp_v_ids = {idx for fi in comp_faces for idx in faces[fi]}
            cx = sum(verts[i][0] for i in comp_v_ids) / len(comp_v_ids)
            cy = sum(verts[i][1] for i in comp_v_ids) / len(comp_v_ids)
            cz = sum(verts[i][2] for i in comp_v_ids) / len(comp_v_ids)
            center = (cx, cy, cz)
            score = 0.0
            for fi in comp_faces:
                a, b, c = faces[fi]
                fc = v_scale(v_add(verts[a], v_add(verts[b], verts[c])), 1.0 / 3.0)
                fn = v_cross(v_sub(verts[b], verts[a]), v_sub(verts[c], verts[a]))
                score += v_dot(v_sub(fc, center), fn)
            flip_comp = score < 0.0

        if flip_comp:
            for fi in comp_faces:
                a, b, c = faces[fi]
                faces[fi] = (a, c, b)
            flipped_total += len(comp_faces)

    return TriMesh(vertices=list(verts), faces=faces), flipped_total


def fill_small_holes(
    mesh: TriMesh,
    max_loop_edges: int = 80,
    keep_proximal_cuff: bool = True,
) -> tuple[TriMesh, int]:
    """Fill internal tear holes cleanly while optionally keeping the primary proximal cuff open."""
    loops = mesh.boundary_loops()
    if not loops:
        return mesh.copy(), 0

    zmin, zmax = mesh.bounds()[0][2], mesh.bounds()[1][2]
    zspan = max(1e-6, zmax - zmin)
    top_thresh = zmin + 0.72 * zspan

    # Identify proximal cuff loop (largest perimeter in upper Z band)
    cuff_idx: int | None = None
    if keep_proximal_cuff:
        best_perim = -1.0
        for idx, lp in enumerate(loops):
            mean_z = sum(mesh.vertices[v][2] for v in lp) / len(lp)
            if mean_z >= top_thresh:
                perim = sum(
                    v_norm(v_sub(mesh.vertices[lp[(k + 1) % len(lp)]], mesh.vertices[lp[k]]))
                    for k in range(len(lp))
                )
                if perim > best_perim:
                    best_perim = perim
                    cuff_idx = idx

    half_edges = {(a, b) for a, b, c in mesh.faces} | {(b, c) for a, b, c in mesh.faces} | {(c, a) for a, b, c in mesh.faces}
    verts = list(mesh.vertices)
    faces = list(mesh.faces)
    filled = 0

    for idx, lp in enumerate(loops):
        if idx == cuff_idx or len(lp) > max_loop_edges or len(lp) < 3:
            continue
        ordered = list(lp)
        if (ordered[0], ordered[1]) not in half_edges:
            ordered.reverse()

        cx = sum(verts[v][0] for v in ordered) / len(ordered)
        cy = sum(verts[v][1] for v in ordered) / len(ordered)
        cz = sum(verts[v][2] for v in ordered) / len(ordered)
        c_idx = len(verts)
        verts.append((cx, cy, cz))

        m = len(ordered)
        for k in range(m):
            u = ordered[k]
            v = ordered[(k + 1) % m]
            # Mesh has boundary edge (u, v); cap emits (v, u, c_idx) to cancel cleanly
            faces.append((v, u, c_idx))
            half_edges.add((v, u))
        filled += 1

    return TriMesh(vertices=verts, faces=faces), filled


def clip_mesh_scalar(
    mesh: TriMesh,
    values: list[float],
    threshold: float = 0.0,
    keep_above: bool = True,
) -> TriMesh:
    """Clip a TriMesh cleanly against an arbitrary per-vertex scalar iso-surface with zero T-junctions."""
    if len(values) != len(mesh.vertices):
        raise ValueError("values length must match mesh.vertices length")

    verts = list(mesh.vertices)
    # Nudge exact-threshold values slightly to avoid zero-length edges or degenerate boundary vertices
    signed = [
        (v - threshold) if abs(v - threshold) >= 1e-7 else (1e-7 if keep_above else -1e-7)
        for v in values
    ]
    new_faces: list[Tri] = []
    edge_cut_cache: dict[tuple[int, int], int] = {}

    def is_kept(idx: int) -> bool:
        return signed[idx] >= 0.0 if keep_above else signed[idx] <= 0.0

    def cut_vertex(u: int, v: int) -> int:
        key = (u, v) if u < v else (v, u)
        if key in edge_cut_cache:
            return edge_cut_cache[key]
        pu, pv = verts[u], verts[v]
        su, sv = signed[u], signed[v]
        denom = sv - su
        t = 0.5 if abs(denom) < 1e-12 else max(1e-5, min(1.0 - 1e-5, -su / denom))
        pt = (
            pu[0] + t * (pv[0] - pu[0]),
            pu[1] + t * (pv[1] - pu[1]),
            pu[2] + t * (pv[2] - pu[2]),
        )
        idx = len(verts)
        verts.append(pt)
        edge_cut_cache[key] = idx
        return idx

    for a, b, c in mesh.faces:
        ka, kb, kc = is_kept(a), is_kept(b), is_kept(c)
        n_kept = int(ka) + int(kb) + int(kc)
        if n_kept == 0:
            continue
        if n_kept == 3:
            new_faces.append((a, b, c))
            continue

        tri = [a, b, c]
        flags = [ka, kb, kc]
        while not (flags[0] and (n_kept == 1 or flags[1])):
            tri = [tri[1], tri[2], tri[0]]
            flags = [flags[1], flags[2], flags[0]]

        v0, v1, v2 = tri
        if n_kept == 1:
            p01 = cut_vertex(v0, v1)
            p02 = cut_vertex(v0, v2)
            new_faces.append((v0, p01, p02))
        else:
            p12 = cut_vertex(v1, v2)
            p20 = cut_vertex(v2, v0)
            new_faces.append((v0, v1, p12))
            new_faces.append((v0, p12, p20))

    used = sorted({idx for f in new_faces for idx in f})
    remap = {old: idx for idx, old in enumerate(used)}
    return TriMesh(
        vertices=[verts[old] for old in used],
        faces=[(remap[a], remap[b], remap[c]) for a, b, c in new_faces],
    )


def clip_mesh_z(mesh: TriMesh, z_cut: float, keep_above: bool = True) -> TriMesh:
    """Clip a TriMesh cleanly against plane z = z_cut with shared edge vertices (zero T-junctions)."""
    verts = list(mesh.vertices)
    new_faces: list[Tri] = []
    edge_cut_cache: dict[tuple[int, int], int] = {}

    def is_kept(z: float) -> bool:
        return z >= z_cut if keep_above else z <= z_cut

    def cut_vertex(u: int, v: int) -> int:
        key = (u, v) if u < v else (v, u)
        if key in edge_cut_cache:
            return edge_cut_cache[key]
        pu, pv = verts[u], verts[v]
        dz = pv[2] - pu[2]
        t = 0.5 if abs(dz) < 1e-12 else max(0.0, min(1.0, (z_cut - pu[2]) / dz))
        pt = (
            pu[0] + t * (pv[0] - pu[0]),
            pu[1] + t * (pv[1] - pu[1]),
            z_cut,
        )
        idx = len(verts)
        verts.append(pt)
        edge_cut_cache[key] = idx
        return idx

    for a, b, c in mesh.faces:
        ka, kb, kc = is_kept(verts[a][2]), is_kept(verts[b][2]), is_kept(verts[c][2])
        n_kept = int(ka) + int(kb) + int(kc)
        if n_kept == 0:
            continue
        if n_kept == 3:
            new_faces.append((a, b, c))
            continue

        # Rotate triangle so kept vertices come first while preserving cyclic winding
        tri = [a, b, c]
        flags = [ka, kb, kc]
        while not (flags[0] and (n_kept == 1 or flags[1])):
            tri = [tri[1], tri[2], tri[0]]
            flags = [flags[1], flags[2], flags[0]]

        v0, v1, v2 = tri
        if n_kept == 1:
            p01 = cut_vertex(v0, v1)
            p02 = cut_vertex(v0, v2)
            new_faces.append((v0, p01, p02))
        else:  # n_kept == 2: v0, v1 kept; v2 clipped
            p12 = cut_vertex(v1, v2)
            p20 = cut_vertex(v2, v0)
            new_faces.append((v0, v1, p12))
            new_faces.append((v0, p12, p20))

    # Compact unused vertices
    used = sorted({idx for f in new_faces for idx in f})
    remap = {old: idx for idx, old in enumerate(used)}
    return TriMesh(
        vertices=[verts[old] for old in used],
        faces=[(remap[a], remap[b], remap[c]) for a, b, c in new_faces],
    )


def repair_scan_mesh(
    mesh: TriMesh,
    weld_tol: float = 1e-4,
    fill_holes: bool = True,
    ensure_open_cuff: bool = False,
    cuff_crop_fraction: float = 0.06,
) -> tuple[TriMesh, RepairReport]:
    """Run full scan repair: weld/clean -> fix winding -> fill internal tears -> optional cuff crop."""
    cleaned, welded_rm, degen_rm = weld_and_clean(mesh, tol=weld_tol)
    wound_ok, flipped = fix_consistent_winding(cleaned)
    holes_filled = 0
    if fill_holes:
        wound_ok, holes_filled = fill_small_holes(wound_ok, keep_proximal_cuff=True)

    clipped_cuff = False
    if ensure_open_cuff and len(wound_ok.boundary_loops()) == 0 and wound_ok.vertices:
        zmin, zmax = wound_ok.bounds()[0][2], wound_ok.bounds()[1][2]
        z_cut = zmax - max(2.0, cuff_crop_fraction * (zmax - zmin))
        wound_ok = clip_mesh_z(wound_ok, z_cut=z_cut, keep_above=False)
        clipped_cuff = True

    warnings: list[str] = []
    if welded_rm > 0:
        warnings.append(f"Welded {welded_rm} duplicate/coincident vertices")
    if flipped > 0:
        warnings.append(f"Corrected winding on {flipped} faces")
    if holes_filled > 0:
        warnings.append(f"Filled {holes_filled} internal scan tear(s)")

    return wound_ok, RepairReport(
        welded_vertices_removed=welded_rm,
        degenerate_faces_removed=degen_rm,
        flipped_faces=flipped,
        holes_filled=holes_filled,
        clipped_proximal_cuff=clipped_cuff,
        warnings=warnings,
    )
