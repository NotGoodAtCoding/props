"""Optional subprocess FEA wrapper (CalculiX / Gmsh boundary) kept strictly separate from MIT op_core."""

from .calculix import (
    FeaCheckResult,
    OrthotropicPrintMaterial,
    generate_iso10328_inp,
    has_calculix_toolchain,
    parse_calculix_dat,
    run_calculix_precheck,
)

__all__ = [
    "FeaCheckResult",
    "OrthotropicPrintMaterial",
    "generate_iso10328_inp",
    "has_calculix_toolchain",
    "parse_calculix_dat",
    "run_calculix_precheck",
]
