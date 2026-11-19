[Home (README)](../README.md) | **Getting Started** | [CLI & Tools Reference](./cli_and_tools_reference.md) | [`.oprecipe.json` Schema](./oprecipe_schema.md) | [Action Plan](../o_p_technology_action_plan.md) | [Research](../research/practitioner_software_needs.md) | [Experiments](../experiments/high_value_experiments.md)

---

# Getting Started with `op-core`

> **Who this page is for:** Practicing O&P clinicians (CPOs), fabrication technicians, and developers running `op-core` for the first time.  
> **What you will learn:** How to run the offline demo, inspect a raw `.stl` limb scan with chairside QA, compile a `.oprecipe.json` recipe into printable/carvable files, and compare two check-socket revisions.

---

## 1. Prerequisites & Setup

`op-core` is intentionally **zero-dependency** in its MVP core—it runs directly on **Python 3.11 / 3.12+** using only the Python standard library (no Blender, VTK, or C++ compilation required to test).

From the repository root:

```bash
# Option A: Run directly from source without installing
PYTHONPATH=src python3 -m op_core.cli --help

# Option B: Install editable in an environment with pip
pip install -e .
op-core --help
```

*(All commands below use `PYTHONPATH=src python3 -m op_core.cli`, which works everywhere out of the box.)*

---

## 2. Walkthrough 1: Run the 30-Second Offline Demo

If you don't have a patient `.stl` scan handy, use `demo` to generate a synthetic transtibial (TT) residual limb or Ankle-Foot Orthosis (AFO) scan and run the full pipeline:

```bash
# Transtibial socket demo (OrcaSlicer .3mf + CNC carver mold + dossier)
PYTHONPATH=src python3 -m op_core.cli demo --out-dir ./demo_tt --device tt --target orcaslicer

# Ankle-Foot Orthosis (AFO) demo (ankle neutralization + sole flattening + coaxial joints)
PYTHONPATH=src python3 -m op_core.cli demo --out-dir ./demo_afo --device afo --target prusaslicer
```

### What Gets Created in `./demo_tt/`

| Output File | Who Uses It | Purpose |
| :--- | :--- | :--- |
| `sample_scan.stl` | Clinician / QA | Synthetic raw limb scan (mm units) used as input. |
| `sample.oprecipe.json` | Clinician & Lab | Portable, human-readable record of all reductions, reliefs, and alignment angles. |
| `rectified_positive.stl` | Technician | Modified limb positive after liner subtraction, compression, and landmark reliefs. |
| `device_shell.stl` | 3D Print Tech | Watertight printable shell with rounded proximal brim, `5°/5°` MPT alignment, and captive M6 nut pockets. |
| `device_orcaslicer.3mf` | 3D Print Tech | Slicer-ready `.3mf` embedding a `100%` infill / `6`-perimeter modifier volume around the distal pyramid zone. |
| `carver_positive.stl` | Central Lab / Carver | Positive mold extended with a `12°` tapered thermoforming draft collar and square-keyed `25.4 mm` mandrel bore. |
| `device_dossier.json` / `.md` | Clinician / QA | Traceability bundle (scan SHA-256, 20 mm circumferences, lint result, and operations log). |

---

## 3. Walkthrough 2: Chairside Scan QA (`op-core qa`)

Before a patient leaves the clinic—or before a technician spends time sculpting a broken file—run `qa` on any binary or ASCII `.stl` scan:

```bash
PYTHONPATH=src python3 -m op_core.cli qa ./demo_tt/sample_scan.stl --out-stl ./demo_tt/aligned.stl
```

What `qa` checks and outputs (as JSON):
1. **Status (`PASS` / `WARN` / `FAIL`):**
   * `floater_components_removed`: Detached background/table fragments automatically pruned.
   * `proximal_cuff_loops` vs. `internal_tear_loops`: Verifies the open proximal cuff is intact and flags unintended scanning holes (`FAIL`).
   * `double_wall_vertices`: Flags overlapping SLAM tracking surfaces (`FAIL` if `> 15` stacked vertices).
   * `seam_step_score_mm`: Flags sharp loop-closure ridge steps (`FAIL` if `> 1.0 mm`).
   * `detected_scale_ratio`: If you include `--measured-bar-mm` and `--expected-bar-mm`, flags scale drift exceeding `±2.0%`.
2. **`slices` (20 mm Taut-Tape Table):** Convex-hull circumferences and cross-sectional areas measured every `20 mm` downward from the Mid-Patellar Tendon (MPT) anchor plane—ready to compare directly against a physical tape measure.

---

## 4. Walkthrough 3: Compile & Revise a Check-Socket Recipe (`build` + `diff`)

To run `build` with your own scan and one of the provided example recipes in [`examples/`](../examples/tt_socket.oprecipe.json):

```bash
PYTHONPATH=src python3 -m op_core.cli build \
  ./demo_tt/sample_scan.stl \
  examples/tt_socket.oprecipe.json \
  --out-dir ./build_v1 \
  --target orcaslicer
```

If a patient returns from a check-socket fitting needing less global compression (`-3.0% -> -1.5%`) and extra fibular head relief (`+4.0 mm -> +5.5 mm`), update the recipe (see [`examples/tt_socket_v2.oprecipe.json`](../examples/tt_socket_v2.oprecipe.json)) and inspect the exact clinical diff before rebuilding:

```bash
PYTHONPATH=src python3 -m op_core.cli diff \
  examples/tt_socket.oprecipe.json \
  examples/tt_socket_v2.oprecipe.json
```

Because `op-core` evaluates every relief in the **undeformed reference mesh domain (`M_0`)**, relaxing the global compression in `v2` will **not** distort or shift the surrounding anatomical reliefs.

---

## 5. Verify Installation & Benchmarks

Run the unit tests and reproducible experiment suite (`EXP-1` through `EXP-5`) anytime:

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -v
PYTHONPATH=src python3 scripts/run_experiments.py
```

---

**Next:** [CLI & Tools Reference](./cli_and_tools_reference.md) · [`.oprecipe.json` Schema Guide](./oprecipe_schema.md)
