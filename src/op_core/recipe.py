"""Declarative .oprecipe.json v0.1 contract, validation, provenance, and revision diffing."""

from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from .math3d import Vec3
from .mesh import TriMesh

SCHEMA_VERSION = "0.1"
KERNEL_VERSION = "0.1.0"


@dataclass
class LandmarkRef:
    """Mesh-anchored or canonical landmark specification."""

    vertex_idx: int | None = None
    face_idx: int | None = None
    bary: tuple[float, float, float] | None = None
    xyz: Vec3 | None = None

    def resolve_vertex(self, mesh: TriMesh) -> int:
        if self.vertex_idx is not None and 0 <= self.vertex_idx < len(mesh.vertices):
            return self.vertex_idx
        if self.face_idx is not None and 0 <= self.face_idx < len(mesh.faces):
            a, b, c = mesh.faces[self.face_idx]
            if self.bary is not None:
                u, v, w = self.bary
                va, vb, vc = mesh.vertices[a], mesh.vertices[b], mesh.vertices[c]
                pt = (
                    u * va[0] + v * vb[0] + w * vc[0],
                    u * va[1] + v * vb[1] + w * vc[1],
                    u * va[2] + v * vb[2] + w * vc[2],
                )
                return mesh.nearest_vertex(pt)
            return a
        if self.xyz is not None:
            return mesh.nearest_vertex(self.xyz)
        raise ValueError("LandmarkRef has neither valid vertex_idx, face_idx, nor xyz")


@dataclass
class OpRecipe:
    """Portable, deterministic O&P modification recipe (.oprecipe.json v0.1)."""

    device_type: str  # "transtibial_socket" | "transfemoral_socket" | "afo"
    side: str = "left"  # "left" | "right"
    units: str = "mm"
    schema_version: str = SCHEMA_VERSION
    kernel_version: str = KERNEL_VERSION
    scan_sha256: str = ""
    scale_factor: float = 1.0
    liner_thickness_mm: float = 0.0
    alignment: dict[str, Any] = field(
        default_factory=lambda: {
            "flexion_deg": 5.0,
            "adduction_deg": 5.0,
            "mpt_offset_mm": [0.0, 0.0],
        }
    )
    landmarks: dict[str, dict[str, Any]] = field(default_factory=dict)
    operations: list[dict[str, Any]] = field(default_factory=list)
    shell: dict[str, Any] = field(
        default_factory=lambda: {
            "wall_distal_mm": 4.5,
            "wall_proximal_mm": 2.5,
            "trimline_fillet_mm": 10.0,
            "hardware": "pyramid_50_8_m6_captive",
        }
    )

    def validate(self) -> None:
        if self.units != "mm":
            raise ValueError(f"Unsupported units {self.units!r}; .oprecipe.json v0.1 requires 'mm'")
        if self.side not in ("left", "right"):
            raise ValueError(f"Invalid side {self.side!r}; expected 'left' or 'right'")
        if self.device_type not in ("transtibial_socket", "transfemoral_socket", "afo"):
            raise ValueError(f"Unsupported device_type {self.device_type!r}")
        if self.scale_factor <= 0.0:
            raise ValueError("scale_factor must be positive")
        if self.liner_thickness_mm < 0.0 or self.liner_thickness_mm > 12.0:
            raise ValueError("liner_thickness_mm out of clinical bounds [0, 12]")
        for i, op in enumerate(self.operations):
            if "op" not in op:
                raise ValueError(f"Operation index {i} missing required 'op' key")

    def to_dict(self) -> dict[str, Any]:
        self.validate()
        return asdict(self)

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)

    def save(self, path: str | Path) -> Path:
        out = Path(path)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(self.to_json() + "\n", encoding="utf-8")
        return out

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "OpRecipe":
        recipe = cls(
            device_type=data["device_type"],
            side=data.get("side", "left"),
            units=data.get("units", "mm"),
            schema_version=data.get("schema_version", SCHEMA_VERSION),
            kernel_version=data.get("kernel_version", KERNEL_VERSION),
            scan_sha256=data.get("scan_sha256", ""),
            scale_factor=float(data.get("scale_factor", 1.0)),
            liner_thickness_mm=float(data.get("liner_thickness_mm", 0.0)),
            alignment=dict(data.get("alignment", {})),
            landmarks=dict(data.get("landmarks", {})),
            operations=list(data.get("operations", [])),
            shell=dict(data.get("shell", {})),
        )
        recipe.validate()
        return recipe

    @classmethod
    def load(cls, path: str | Path) -> "OpRecipe":
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))

    def resolve_landmarks(self, mesh: TriMesh) -> dict[str, int]:
        resolved: dict[str, int] = {}
        for name, spec in self.landmarks.items():
            ref = LandmarkRef(
                vertex_idx=spec.get("vertex_idx"),
                face_idx=spec.get("face_idx"),
                bary=tuple(spec["bary"]) if "bary" in spec else None,
                xyz=tuple(spec["xyz"]) if "xyz" in spec else None,
            )
            resolved[name] = ref.resolve_vertex(mesh)
        return resolved


def mesh_sha256(mesh: TriMesh) -> str:
    """Compute a deterministic SHA-256 checksum for a TriMesh's canonical geometry."""
    h = hashlib.sha256()
    for x, y, z in mesh.vertices:
        h.update(f"{x:.4f},{y:.4f},{z:.4f};".encode("ascii"))
    for a, b, c in mesh.faces:
        h.update(f"{a},{b},{c};".encode("ascii"))
    return h.hexdigest()


def diff_recipes(v1: OpRecipe, v2: OpRecipe) -> dict[str, Any]:
    """Compute a structured clinical revision diff between two .oprecipe.json specs."""
    changes: dict[str, Any] = {}
    for key in ("liner_thickness_mm", "scale_factor", "alignment", "shell"):
        a = getattr(v1, key)
        b = getattr(v2, key)
        if a != b:
            changes[key] = {"from": a, "to": b}

    ops1 = {op.get("id", f"idx_{i}"): op for i, op in enumerate(v1.operations)}
    ops2 = {op.get("id", f"idx_{i}"): op for i, op in enumerate(v2.operations)}
    op_diffs: list[dict[str, Any]] = []
    for op_id in sorted(set(ops1) | set(ops2)):
        if op_id not in ops1:
            op_diffs.append({"id": op_id, "change": "added", "to": ops2[op_id]})
        elif op_id not in ops2:
            op_diffs.append({"id": op_id, "change": "removed", "from": ops1[op_id]})
        elif ops1[op_id] != ops2[op_id]:
            op_diffs.append({"id": op_id, "change": "modified", "from": ops1[op_id], "to": ops2[op_id]})
    if op_diffs:
        changes["operations"] = op_diffs
    return changes
