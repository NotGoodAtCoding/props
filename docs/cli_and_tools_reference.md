[Home (README)](../README.md) | [Getting Started](./getting_started.md) | **CLI & Tools Reference** | [`.oprecipe.json` Schema](./oprecipe_schema.md) | [Action Plan](../o_p_technology_action_plan.md) | [Research](../research/practitioner_software_needs.md) | [Experiments](../experiments/high_value_experiments.md)

---

# CLI, Scripts & Python Tools Reference

> **Who this page is for:** Technicians, clinical engineers, and contributors looking up exact command flags, script options, or Python module APIs inside [`src/op_core/`](../src/op_core/__init__.py).

---

## 1. Command-Line Tools (`op-core`)

Run via `PYTHONPATH=src python3 -m op_core.cli <command>` (or `op-core <command>` if installed via `pip install -e .`).

### `op-core ui`
Launches the zero-dependency local 3D WebGL browser workstation for interactive scan QA, click-to-place 3D landmarking, `M_0` rectification heatmaps, shell/lint preview, and artifact downloads.

| Flag | Default | Description |
| :--- | :--- | :--- |
| `--host` | `127.0.0.1` | Bind address for local HTTP server. |
| `--port` | `8765` | Bind port for local HTTP server. |
| `--no-browser` | `False` | Start server without automatically opening a browser tab. |

### `op-core demo`
Generates a synthetic clinical mesh + `.oprecipe.json`, executes the full pipeline, and writes all CAD/CAM outputs to `--out-dir`.

| Flag | Default | Description |
| :--- | :--- | :--- |
| `--out-dir DIR` | *(required)* | Output directory for sample inputs and compiled outputs. |
| `--device {tt,afo}` | `tt` | `tt` = transtibial socket; `afo` = plantarflexed ankle-foot orthosis. |
| `--target {orcaslicer,prusaslicer}` | `orcaslicer` | Slicer modifier schema embedded into the output `.3mf`. |

### `op-core qa`
Validates a raw `.stl` scan, prunes background floaters, aligns `+Z` proximal, and computes `20 mm` taut-tape circumferences. Exit code is `0` on `PASS`/`WARN` and `1` on `FAIL`.

| Argument / Flag | Default | Description |
| :--- | :--- | :--- |
| `scan` | *(required)* | Path to input binary or ASCII `.stl` mesh (in millimeters). |
| `--out-stl PATH` | `None` | Optional path to save the cleaned, canonically aligned binary `.stl`. |
| `--measured-bar-mm FLOAT` | `None` | Measured length of a reference calibration bar in the scan (mm). |
| `--expected-bar-mm FLOAT` | `100.0` | True physical length of the calibration bar (mm). Drift `> ±2%` triggers `FAIL`. |

### `op-core build`
Compiles an input `.stl` scan + `.oprecipe.json` v0.1 file into:
1. `rectified_positive.stl`
2. `device_shell.stl`
3. `device_<target>.3mf`
4. `carver_positive.stl`
5. `device_dossier.json` & `device_dossier.md`

| Argument / Flag | Default | Description |
| :--- | :--- | :--- |
| `scan` | *(required)* | Input `.stl` limb scan (mm). |
| `recipe` | *(required)* | Input `.oprecipe.json` v0.1 specification. |
| `--out-dir DIR` | *(required)* | Directory to write generated STL, `.3mf`, and dossier files. |
| `--target {orcaslicer,prusaslicer}` | `orcaslicer` | Slicer modifier schema (`model_settings.config` vs `Slic3r_PE_model.config`). |

### `op-core lint`
Inspects any finished shell `.stl` for watertight 2-manifoldness, minimum wall thickness, trimline smoothness, and load-axis vs. print-Z tilt.

| Argument / Flag | Default | Description |
| :--- | :--- | :--- |
| `shell_stl` | *(required)* | Path to shell `.stl` to audit. |
| `--min-wall-mm FLOAT` | `2.5` | Minimum allowable load-zone wall thickness (mm). |

### `op-core diff`
Prints a structured JSON diff between two `.oprecipe.json` files (e.g., check socket v1 vs. v2).

```bash
PYTHONPATH=src python3 -m op_core.cli diff examples/tt_socket.oprecipe.json examples/tt_socket_v2.oprecipe.json
```

---

## 2. Experiment & Benchmark Runner (`scripts/run_experiments.py`)

Reproducibly runs the quantitative validation suite defined in [`experiments/high_value_experiments.md`](../experiments/high_value_experiments.md):

```bash
# Run all MVP experiments (EXP-1, EXP-2, EXP-2B+3, EXP-4+5)
PYTHONPATH=src python3 scripts/run_experiments.py

# Run a single experiment group
PYTHONPATH=src python3 scripts/run_experiments.py --exp EXP-1
```

| `--exp` Value | Benchmarks Executed |
| :--- | :--- |
| `all` | Runs every group below and exits `0` only if all pass. |
| `EXP-1` | 5-seed tilted/noisy `20 mm` circumference error + 5-seed corrupted-scan detection. |
| `EXP-2` | Reference-domain `M_0` zoned-isolation drift + global-scale relief-height invariance. |
| `EXP-2B_and_EXP-3` | 10 watertight MPT-aligned TT shells + 5 neutralized sole-flattened AFO shells. |
| `EXP-4_and_EXP-5` | Dual-schema `.3mf` checks, safety linter discrimination, and 5 keyed carver positives. |

---

## 3. Python Package Modules (`src/op_core/`)

| Module | Purpose | Primary Entry Points |
| :--- | :--- | :--- |
| [`ui_server.py`](../src/op_core/ui_server.py) | Local WebGL 3D workstation & REST API (`SessionStore`, Moller-Trumbore picking) | `serve_ui()`, `create_server()` |
| [`repair.py`](../src/op_core/repair.py) | Real-scan vertex welding, winding consistency repair, hole filling, Z-clipping | `repair_scan_mesh()`, `weld_and_clean()`, `fix_consistent_winding()`, `fill_small_holes()`, `clip_mesh_z()` |
| [`scan_qa.py`](../src/op_core/scan_qa.py) | Chairside scan triage, canonical alignment, landmark-anchored tape check | `run_scan_qa()`, `align_canonical_limb()`, `measure_circumferences()`, `prune_floaters()`, `classify_boundary_loops()`, `detect_double_walls()`, `detect_seam_step()` |
| [`recipe.py`](../src/op_core/recipe.py) | `.oprecipe.json` v0.1 validation, landmark resolution, diffing | `OpRecipe`, `LandmarkRef`, `diff_recipes()`, `mesh_sha256()` |
| [`rectify.py`](../src/op_core/rectify.py) | Non-destructive `M_0` geodesic rectification & AFO posture neutralization | `apply_recipe()`, `apply_afo_posture()`, `RectificationResult` |
| [`shell.py`](../src/op_core/shell.py) | Variable shelling (TT, TF, AFO), bench alignment, hardware bosses, CNC carver molds | `build_tt_socket_shell()`, `build_tf_socket_shell()`, `build_afo_shell()`, `export_carver_positive()`, `has_manifold3d()` |
| [`export_3mf.py`](../src/op_core/export_3mf.py) | Slicer-targeted `.3mf` writer embedding reinforcement modifier meshes | `export_3mf()` (`target="orcaslicer"` or `"prusaslicer"`) |
| [`linter.py`](../src/op_core/linter.py) | Pre-print structural safety checks & archival MDR/QMSR dossier export | `lint_device()`, `estimate_min_wall_thickness_mm()`, `export_fabrication_dossier()` |
| [`mesh.py`](../src/op_core/mesh.py) | Pure-Python triangle mesh, manifold audit, Dijkstra geodesics, STL I/O | `TriMesh`, `read_stl()`, `write_binary_stl()` |
| [`math3d.py`](../src/op_core/math3d.py) | 3D vectors, Rodrigues rotation, 3×3 Jacobi PCA, 2D convex hull | `rodrigues_rotate()`, `jacobi_eigen_3x3()`, `convex_hull_2d()`, `smoothstep5()` |
| [`synthetic.py`](../src/op_core/synthetic.py) | Procedural TT residual limbs, plantarflexed feet, and artifact injection | `generate_transtibial_limb()`, `generate_foot_ankle_mesh()`, `corrupt_scan()` |

### Minimal Python API Example

```python
from op_core import OpRecipe, apply_recipe, build_tt_socket_shell, export_3mf, run_scan_qa
from op_core.synthetic import generate_transtibial_limb

case = generate_transtibial_limb(open_cuff=True, seed=0)
aligned, qa = run_scan_qa(case.mesh, mpt_point=case.landmark_coords["mpt"], distal_point=case.landmark_coords["distal_end"])
recipe = OpRecipe.load("examples/tt_socket.oprecipe.json")

rect = apply_recipe(aligned, recipe)
shell_res = build_tt_socket_shell(rect.rectified_mesh, flexion_deg=5.0, adduction_deg=5.0)
export_3mf(shell_res.shell_mesh, shell_res.modifier_mesh, "out/socket.3mf", target="orcaslicer")
```

---

**Next:** [`.oprecipe.json` Schema Guide](./oprecipe_schema.md) · [Getting Started](./getting_started.md)
