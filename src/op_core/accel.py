"""Environment capability probe for optional [accel] libraries and external CAD/CAM/FEA binaries."""

from __future__ import annotations

import importlib.util
import shutil
from dataclasses import asdict, dataclass


@dataclass
class BackendStatus:
    """Availability report for optional Python accelerators and external subprocess binaries."""

    python_packages: dict[str, bool]
    external_binaries: dict[str, str | None]
    core_ready: bool = True

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def probe_backends() -> BackendStatus:
    """Inspect installed optional packages and external binaries without importing GPL modules."""
    pkgs = {
        pkg: importlib.util.find_spec(pkg) is not None
        for pkg in ("numpy", "scipy", "trimesh", "manifold3d", "ampscan")
    }
    bins = {
        "ccx": shutil.which("ccx"),
        "gmsh": shutil.which("gmsh"),
        "orca_slicer": shutil.which("orca-slicer") or shutil.which("orcaslicer"),
        "prusa_slicer": shutil.which("prusa-slicer") or shutil.which("prusaslicer"),
        "blender": shutil.which("blender"),
    }
    return BackendStatus(python_packages=pkgs, external_binaries=bins, core_ready=True)
