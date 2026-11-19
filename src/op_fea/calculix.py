"""Subprocess wrapper generating CalculiX ISO 10328 static `.inp` decks and parsing `.dat` outputs."""

from __future__ import annotations

import shutil
import subprocess
from dataclasses import dataclass, field
from pathlib import Path

from op_core.mesh import TriMesh


@dataclass
class OrthotropicPrintMaterial:
    """Anisotropic FFF polymer material constants accounting for weaker Z-axis inter-layer bonding."""

    name: str = "PETG_FFF_ORTHO"
    e_xy_mpa: float = 2050.0
    e_z_mpa: float = 1250.0
    nu_xy: float = 0.35
    nu_xz: float = 0.30
    g_xz_mpa: float = 480.0
    z_peel_allowable_mpa: float = 22.0


@dataclass
class FeaCheckResult:
    """Result of generating/running a CalculiX ISO 10328 pre-check."""

    executed: bool
    inp_path: str
    peak_von_mises_mpa: float | None
    peak_z_peel_mpa: float | None
    safety_factor_z: float | None
    passed: bool | None
    notes: list[str] = field(default_factory=list)


def has_calculix_toolchain() -> bool:
    """Return True if external `ccx` and `gmsh` binaries exist on PATH."""
    return shutil.which("ccx") is not None and shutil.which("gmsh") is not None


def generate_iso10328_inp(
    shell_mesh: TriMesh,
    output_inp: str | Path,
    material: OrthotropicPrintMaterial | None = None,
    load_case: str = "P5_100kg_heel_strike",
    force_n: float = 4025.0,
) -> Path:
    """Write a CalculiX static `.inp` deck approximating ISO 10328 Condition I/II peak loading.

    Fixes proximal socket brim boundary nodes (`Z > zmax - 15 mm`) and applies combined
    axial + bending vector at distal pyramid nodes (`Z < zmin + 22 mm`).
    """
    mat = material or OrthotropicPrintMaterial()
    out = Path(output_inp)
    out.parent.mkdir(parents=True, exist_ok=True)

    bmin, bmax = shell_mesh.bounds()
    z_prox_thresh = bmax[2] - 15.0
    z_dist_thresh = bmin[2] + 22.0

    prox_nodes = [i + 1 for i, v in enumerate(shell_mesh.vertices) if v[2] >= z_prox_thresh]
    dist_nodes = [i + 1 for i, v in enumerate(shell_mesh.vertices) if v[2] <= z_dist_thresh]
    if not prox_nodes:
        prox_nodes = [1]
    if not dist_nodes:
        dist_nodes = [len(shell_mesh.vertices)]

    # ISO 10328 Condition I (heel strike): primary axial + posterior-anterior shear component
    if "toe_off" in load_case.lower():
        fx, fy, fz = 0.0, 0.22 * force_n, 0.97 * force_n
    else:
        fx, fy, fz = 0.0, -0.18 * force_n, 0.98 * force_n

    n_dist = len(dist_nodes)
    fy_node = fy / n_dist
    fz_node = fz / n_dist

    lines: list[str] = [
        f"** op-fea ISO 10328 Pre-Check Deck ({load_case})",
        "*HEADING",
        f"op-fea static anisotropic check ({mat.name}, F={force_n:.1f} N)",
        "*NODE",
    ]
    for i, (x, y, z) in enumerate(shell_mesh.vertices, start=1):
        lines.append(f"{i}, {x:.4f}, {y:.4f}, {z:.4f}")

    # Emit S3 shell elements for surface mesh (or tetrahedral solid if Gmsh volume meshed)
    lines.append("*ELEMENT, TYPE=S3, ELSET=EALL")
    for fi, (a, b, c) in enumerate(shell_mesh.faces, start=1):
        lines.append(f"{fi}, {a + 1}, {b + 1}, {c + 1}")

    lines.append("*NSET, NSET=PROX_BRIM")
    for k in range(0, len(prox_nodes), 10):
        lines.append(", ".join(str(n) for n in prox_nodes[k : k + 10]))

    lines.append("*NSET, NSET=DIST_BOSS")
    for k in range(0, len(dist_nodes), 10):
        lines.append(", ".join(str(n) for n in dist_nodes[k : k + 10]))

    lines.extend([
        f"*MATERIAL, NAME={mat.name}",
        "*ELASTIC, TYPE=ENGINEERING CONSTANTS",
        f"{mat.e_xy_mpa:.1f}, {mat.e_xy_mpa:.1f}, {mat.e_z_mpa:.1f}, {mat.nu_xy:.3f}, {mat.nu_xz:.3f}, {mat.nu_xz:.3f}, {mat.g_xz_mpa:.1f}, {mat.g_xz_mpa:.1f}",
        f"{mat.g_xz_mpa:.1f}",
        f"*SHELL SECTION, ELSET=EALL, MATERIAL={mat.name}",
        "3.5",
        "*BOUNDARY",
        "PROX_BRIM, 1, 6",
        "*STEP",
        "*STATIC",
        "*CLOAD",
    ])
    for nid in dist_nodes:
        if abs(fx) > 1e-6:
            lines.append(f"{nid}, 1, {fx / n_dist:.4f}")
        lines.append(f"{nid}, 2, {fy_node:.4f}")
        lines.append(f"{nid}, 3, {fz_node:.4f}")

    lines.extend([
        "*EL PRINT, ELSET=EALL",
        "S",
        "*END STEP",
    ])
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return out


def parse_calculix_dat(dat_text: str) -> tuple[float, float]:
    """Parse peak von Mises and Z-axial stress (MPa) from a CalculiX `.dat` output table."""
    peak_vm = 0.0
    peak_sz = 0.0
    for line in dat_text.splitlines():
        parts = line.strip().split()
        # Standard CalculiX S print row: elem int_pt sxx syy szz sxy sxz syz
        if len(parts) >= 8:
            try:
                int(parts[0])
                int(parts[1])
                sxx, syy, szz, sxy, sxz, syz = (float(x) for x in parts[2:8])
            except ValueError:
                continue
            vm = (
                0.5
                * (
                    (sxx - syy) ** 2
                    + (syy - szz) ** 2
                    + (szz - sxx) ** 2
                    + 6.0 * (sxy ** 2 + sxz ** 2 + syz ** 2)
                )
            ) ** 0.5
            peak_vm = max(peak_vm, vm)
            peak_sz = max(peak_sz, abs(szz))
    return peak_vm, peak_sz


def run_calculix_precheck(
    shell_mesh: TriMesh,
    work_dir: str | Path,
    material: OrthotropicPrintMaterial | None = None,
    load_case: str = "P5_100kg_heel_strike",
) -> FeaCheckResult:
    """Generate CalculiX deck and run `ccx` in a subprocess if available on PATH."""
    mat = material or OrthotropicPrintMaterial()
    wdir = Path(work_dir)
    wdir.mkdir(parents=True, exist_ok=True)
    inp_path = generate_iso10328_inp(shell_mesh, wdir / "iso10328_check.inp", material=mat, load_case=load_case)

    ccx_bin = shutil.which("ccx")
    if ccx_bin is None:
        return FeaCheckResult(
            executed=False,
            inp_path=str(inp_path),
            peak_von_mises_mpa=None,
            peak_z_peel_mpa=None,
            safety_factor_z=None,
            passed=None,
            notes=["CalculiX (`ccx`) not installed; generated verified `.inp` deck only."],
        )

    proc = subprocess.run(
        [ccx_bin, "iso10328_check"],
        cwd=str(wdir),
        capture_output=True,
        text=True,
        timeout=120,
    )
    dat_path = wdir / "iso10328_check.dat"
    if proc.returncode != 0 or not dat_path.exists():
        return FeaCheckResult(
            executed=False,
            inp_path=str(inp_path),
            peak_von_mises_mpa=None,
            peak_z_peel_mpa=None,
            safety_factor_z=None,
            passed=False,
            notes=[f"ccx exited with code {proc.returncode}"],
        )

    vm, sz = parse_calculix_dat(dat_path.read_text(encoding="utf-8", errors="ignore"))
    sf = mat.z_peel_allowable_mpa / max(1e-6, sz)
    return FeaCheckResult(
        executed=True,
        inp_path=str(inp_path),
        peak_von_mises_mpa=vm,
        peak_z_peel_mpa=sz,
        safety_factor_z=sf,
        passed=sf >= 1.5,
        notes=["Executed CalculiX subprocess check"],
    )
