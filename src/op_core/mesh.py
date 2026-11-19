"""Core triangle mesh representation, STL I/O, topology checks, and geodesic/slicing utilities."""

from __future__ import annotations

import heapq
import math
import struct
from collections import defaultdict, deque
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

from .math3d import (
    Vec2,
    Vec3,
    convex_hull_2d,
    polygon_area_2d,
    polygon_perimeter_2d,
    v_add,
    v_cross,
    v_dist,
    v_dot,
    v_norm,
    v_scale,
    v_sub,
    v_unit,
)

Tri = tuple[int, int, int]


@dataclass
class TriMesh:
    """Lightweight triangle mesh container with pure-Python geometric queries."""

    vertices: list[Vec3]
    faces: list[Tri]

    def copy(self) -> "TriMesh":
        return TriMesh(vertices=list(self.vertices), faces=list(self.faces))

    def bounds(self) -> tuple[Vec3, Vec3]:
        if not self.vertices:
            return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        xs = [v[0] for v in self.vertices]
        ys = [v[1] for v in self.vertices]
        zs = [v[2] for v in self.vertices]
        return ((min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs)))

    def centroid(self) -> Vec3:
        n = len(self.vertices)
        if n == 0:
            return (0.0, 0.0, 0.0)
        return (
            sum(v[0] for v in self.vertices) / n,
            sum(v[1] for v in self.vertices) / n,
            sum(v[2] for v in self.vertices) / n,
        )

    def face_normals(self) -> list[Vec3]:
        normals: list[Vec3] = []
        verts = self.vertices
        for a, b, c in self.faces:
            n = v_cross(v_sub(verts[b], verts[a]), v_sub(verts[c], verts[a]))
            normals.append(v_unit(n))
        return normals

    def vertex_normals(self) -> list[Vec3]:
        acc: list[list[float]] = [[0.0, 0.0, 0.0] for _ in self.vertices]
        verts = self.vertices
        for a, b, c in self.faces:
            fn = v_cross(v_sub(verts[b], verts[a]), v_sub(verts[c], verts[a]))
            for idx in (a, b, c):
                acc[idx][0] += fn[0]
                acc[idx][1] += fn[1]
                acc[idx][2] += fn[2]
        return [v_unit((x, y, z)) for x, y, z in acc]

    def signed_volume(self) -> float:
        verts = self.vertices
        total = 0.0
        for a, b, c in self.faces:
            total += v_dot(verts[a], v_cross(verts[b], verts[c]))
        return total / 6.0

    def edge_topology(self) -> dict[str, int]:
        """Inspect half-edge consistency and boundary/non-manifold edge counts."""
        half_edges: dict[tuple[int, int], int] = defaultdict(int)
        undirected: dict[tuple[int, int], int] = defaultdict(int)
        degenerate = 0
        verts = self.vertices
        for a, b, c in self.faces:
            if a == b or b == c or c == a:
                degenerate += 1
            elif v_norm(v_cross(v_sub(verts[b], verts[a]), v_sub(verts[c], verts[a]))) < 1e-12:
                degenerate += 1
            for u, v in ((a, b), (b, c), (c, a)):
                half_edges[(u, v)] += 1
                key = (u, v) if u < v else (v, u)
                undirected[key] += 1

        boundary_edges = sum(1 for (u, v), cnt in half_edges.items() if cnt == 1 and half_edges.get((v, u), 0) == 0)
        non_manifold_edges = sum(1 for cnt in undirected.values() if cnt > 2)
        winding_mismatches = sum(1 for cnt in half_edges.values() if cnt > 1)
        return {
            "boundary_edges": boundary_edges,
            "non_manifold_edges": non_manifold_edges,
            "winding_mismatches": winding_mismatches,
            "degenerate_faces": degenerate,
        }

    def is_watertight_manifold(self) -> bool:
        if not self.faces:
            return False
        topo = self.edge_topology()
        return (
            topo["boundary_edges"] == 0
            and topo["non_manifold_edges"] == 0
            and topo["winding_mismatches"] == 0
            and topo["degenerate_faces"] == 0
        )

    def boundary_loops(self) -> list[list[int]]:
        """Extract ordered vertex loops along mesh boundaries."""
        half_edges = set()
        for a, b, c in self.faces:
            half_edges.add((a, b))
            half_edges.add((b, c))
            half_edges.add((c, a))

        next_map: dict[int, list[int]] = defaultdict(list)
        for u, v in half_edges:
            if (v, u) not in half_edges:
                next_map[u].append(v)

        visited_edges: set[tuple[int, int]] = set()
        loops: list[list[int]] = []
        for start, targets in next_map.items():
            for nxt in targets:
                if (start, nxt) in visited_edges:
                    continue
                loop = [start]
                u, v = start, nxt
                while (u, v) not in visited_edges:
                    visited_edges.add((u, v))
                    if v == start:
                        break
                    loop.append(v)
                    candidates = [w for w in next_map.get(v, ()) if (v, w) not in visited_edges]
                    if not candidates:
                        break
                    u, v = v, candidates[0]
                if len(loop) >= 3:
                    loops.append(loop)
        return loops

    def connected_components(self) -> list[list[int]]:
        """Return vertex lists for each face-connected component sorted largest-first."""
        adj: list[list[int]] = [[] for _ in self.vertices]
        for a, b, c in self.faces:
            adj[a].extend((b, c))
            adj[b].extend((a, c))
            adj[c].extend((a, b))

        seen = [False] * len(self.vertices)
        comps: list[list[int]] = []
        for i in range(len(self.vertices)):
            if seen[i] or not adj[i]:
                continue
            q = deque([i])
            seen[i] = True
            comp: list[int] = []
            while q:
                u = q.popleft()
                comp.append(u)
                for w in adj[u]:
                    if not seen[w]:
                        seen[w] = True
                        q.append(w)
            comps.append(comp)
        comps.sort(key=len, reverse=True)
        return comps

    def submesh_from_vertices(self, keep_vertices: Iterable[int]) -> "TriMesh":
        keep_set = set(keep_vertices)
        remap: dict[int, int] = {}
        new_verts: list[Vec3] = []
        new_faces: list[Tri] = []
        for a, b, c in self.faces:
            if a in keep_set and b in keep_set and c in keep_set:
                tri = []
                for idx in (a, b, c):
                    if idx not in remap:
                        remap[idx] = len(new_verts)
                        new_verts.append(self.vertices[idx])
                    tri.append(remap[idx])
                new_faces.append((tri[0], tri[1], tri[2]))
        return TriMesh(vertices=new_verts, faces=new_faces)

    def geodesic_distances(self, seeds: Iterable[int]) -> list[float]:
        """Compute multi-source Dijkstra geodesic edge distances on this mesh."""
        n = len(self.vertices)
        adj: list[list[tuple[int, float]]] = [[] for _ in range(n)]
        verts = self.vertices
        seen_edge: set[tuple[int, int]] = set()
        for a, b, c in self.faces:
            for u, v in ((a, b), (b, c), (c, a)):
                key = (u, v) if u < v else (v, u)
                if key not in seen_edge:
                    seen_edge.add(key)
                    w = v_dist(verts[u], verts[v])
                    adj[u].append((v, w))
                    adj[v].append((u, w))

        dist = [math.inf] * n
        pq: list[tuple[float, int]] = []
        for s in seeds:
            if 0 <= s < n:
                dist[s] = 0.0
                heapq.heappush(pq, (0.0, s))

        while pq:
            d, u = heapq.heappop(pq)
            if d > dist[u]:
                continue
            for v, w in adj[u]:
                nd = d + w
                if nd < dist[v]:
                    dist[v] = nd
                    heapq.heappush(pq, (nd, v))
        return dist

    def nearest_vertex(self, point: Vec3) -> int:
        best_i = 0
        best_d2 = math.inf
        for i, v in enumerate(self.vertices):
            dx = v[0] - point[0]
            dy = v[1] - point[1]
            dz = v[2] - point[2]
            d2 = dx * dx + dy * dy + dz * dz
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        return best_i

    def slice_z(self, z_height: float) -> dict[str, float]:
        """Slice mesh with plane z = z_height and return taut-tape hull perimeter & area."""
        pts: list[Vec2] = []
        verts = self.vertices
        for a, b, c in self.faces:
            va, vb, vc = verts[a], verts[b], verts[c]
            za, zb, zc = va[2], vb[2], vc[2]
            zmin = min(za, zb, zc)
            zmax = max(za, zb, zc)
            if zmax < z_height or zmin > z_height:
                continue
            for p0, p1 in ((va, vb), (vb, vc), (vc, va)):
                dz0 = p0[2] - z_height
                dz1 = p1[2] - z_height
                if (dz0 <= 0.0 <= dz1) or (dz1 <= 0.0 <= dz0):
                    denom = p1[2] - p0[2]
                    if abs(denom) < 1e-12:
                        continue
                    t = (z_height - p0[2]) / denom
                    if 0.0 <= t <= 1.0:
                        pts.append((p0[0] + t * (p1[0] - p0[0]), p0[1] + t * (p1[1] - p0[1])))

        if len(pts) < 3:
            return {"z_mm": z_height, "taut_circumference_mm": 0.0, "hull_area_mm2": 0.0, "points": 0}
        hull = convex_hull_2d(pts)
        return {
            "z_mm": z_height,
            "taut_circumference_mm": polygon_perimeter_2d(hull, closed=True),
            "hull_area_mm2": polygon_area_2d(hull),
            "points": len(pts),
        }

    def transformed(self, fn) -> "TriMesh":
        return TriMesh(vertices=[fn(v) for v in self.vertices], faces=list(self.faces))


def write_binary_stl(mesh: TriMesh, path: str | Path, header_text: str = "op-core binary STL") -> Path:
    """Write a TriMesh to binary STL format."""
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    header = header_text.encode("ascii", errors="ignore")[:80].ljust(80, b" ")
    verts = mesh.vertices
    with out.open("wb") as fh:
        fh.write(header)
        fh.write(struct.pack("<I", len(mesh.faces)))
        for a, b, c in mesh.faces:
            va, vb, vc = verts[a], verts[b], verts[c]
            fn = v_unit(v_cross(v_sub(vb, va), v_sub(vc, va)))
            fh.write(struct.pack("<12fH", *fn, *va, *vb, *vc, 0))
    return out


def read_stl(path: str | Path, merge_tol: float = 1e-5) -> TriMesh:
    """Read binary or ASCII STL and deduplicate coincident vertices."""
    data = Path(path).read_bytes()
    if len(data) < 84:
        raise ValueError("Invalid STL file: too short")

    tri_count = struct.unpack("<I", data[80:84])[0]
    expected_bin_len = 84 + tri_count * 50
    raw_tris: list[tuple[Vec3, Vec3, Vec3]] = []

    if len(data) == expected_bin_len:
        offset = 84
        for _ in range(tri_count):
            vals = struct.unpack_from("<12fH", data, offset)
            offset += 50
            raw_tris.append((
                (vals[3], vals[4], vals[5]),
                (vals[6], vals[7], vals[8]),
                (vals[9], vals[10], vals[11]),
            ))
    else:
        text = data.decode("utf-8", errors="ignore")
        buf: list[Vec3] = []
        for line in text.splitlines():
            parts = line.strip().split()
            if len(parts) == 4 and parts[0].lower() == "vertex":
                buf.append((float(parts[1]), float(parts[2]), float(parts[3])))
                if len(buf) == 3:
                    raw_tris.append((buf[0], buf[1], buf[2]))
                    buf.clear()

    inv = 1.0 / merge_tol
    vmap: dict[tuple[int, int, int], int] = {}
    vertices: list[Vec3] = []
    faces: list[Tri] = []
    for tri in raw_tris:
        idxs: list[int] = []
        for p in tri:
            key = (round(p[0] * inv), round(p[1] * inv), round(p[2] * inv))
            if key not in vmap:
                vmap[key] = len(vertices)
                vertices.append(p)
            idxs.append(vmap[key])
        if idxs[0] != idxs[1] and idxs[1] != idxs[2] and idxs[2] != idxs[0]:
            faces.append((idxs[0], idxs[1], idxs[2]))
    return TriMesh(vertices=vertices, faces=faces)
