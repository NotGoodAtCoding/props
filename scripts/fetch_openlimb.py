"""Fetch and normalize external OpenLimbTT / OpenLimbTF anatomical benchmarks to clinical millimeters.

Note: OpenLimbTT and OpenLimbTF are licensed under CC-BY-SA 4.0 / CC-BY 4.0 by the
University of Southampton ABEL group and are NOT vendored into this MIT repository.
Run this script in an online environment to populate a local benchmark directory.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from op_core.mesh import read_stl, write_binary_stl
from op_core.repair import repair_scan_mesh

OPENLIMB_REPOS = {
    "tt": "https://github.com/abel-research/OpenLimbTT.git",
    "tf": "https://github.com/abel-research/OpenLimbTF.git",
}


def normalize_openlimb_stl(src_stl: Path, dst_stl: Path, target_length_mm: float = 180.0) -> Path:
    """Scale a normalized fractional OpenLimb STL to clinical millimeters and crop proximal cuff."""
    raw = read_stl(src_stl)
    bmin, bmax = raw.bounds()
    z_span = max(1e-6, bmax[2] - bmin[2])
    scale = target_length_mm / z_span if z_span < 10.0 else 1.0
    scaled = raw.transformed(lambda p: (p[0] * scale, p[1] * scale, (p[2] - bmax[2]) * scale))
    repaired, _ = repair_scan_mesh(scaled, ensure_open_cuff=True)
    return write_binary_stl(repaired, dst_stl)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="fetch_openlimb.py",
        description="Clone OpenLimbTT / OpenLimbTF benchmark datasets and normalize STLs to clinical mm.",
    )
    parser.add_argument("--dataset", choices=["tt", "tf", "both"], default="tt")
    parser.add_argument("--dest", default="./data_external", help="Local destination folder (default: ./data_external)")
    parser.add_argument("--target-length-mm", type=float, default=180.0, help="Target clinical limb length in mm")
    args = parser.parse_args(argv)

    dest = Path(args.dest)
    dest.mkdir(parents=True, exist_ok=True)
    targets = ["tt", "tf"] if args.dataset == "both" else [args.dataset]
    for key in targets:
        repo_dir = dest / f"OpenLimb{key.upper()}"
        if not repo_dir.exists():
            subprocess.run(["git", "clone", "--depth", "1", OPENLIMB_REPOS[key], str(repo_dir)], check=True)
        norm_dir = dest / f"normalized_{key}"
        norm_dir.mkdir(exist_ok=True)
        for stl_file in sorted(repo_dir.rglob("*.stl"))[:10]:
            out_file = norm_dir / stl_file.name
            normalize_openlimb_stl(stl_file, out_file, target_length_mm=args.target_length_mm)
            print(f"Normalized {stl_file.name} -> {out_file}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
