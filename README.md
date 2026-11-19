**Home (README)** | [Getting Started](./docs/getting_started.md) | [CLI & Tools Reference](./docs/cli_and_tools_reference.md) | [`.oprecipe.json` Schema](./docs/oprecipe_schema.md) | [Action Plan](./o_p_technology_action_plan.md) | [Research](./research/practitioner_software_needs.md) | [Experiments](./experiments/high_value_experiments.md)

---

# props (`op-core`) — Open-Source O&P Digital CAD/CAM Kernel

**`props`** is an MIT-licensed research repository and headless Python toolkit (**`op-core`**) built to remove proprietary lock-in and destructive mesh editing from **Orthotics & Prosthetics (O&P)** digital workflows.

### Who This Is For & What It Solves
* **Practicing Clinicians (CPOs):** Catch bad mobile scans chairside in seconds (`qa`), verify `20 mm` landmark-anchored circumferences against tape measurements, and record unambiguous check-socket revisions in portable `.oprecipe.json` files.
* **O&P Technicians & Central Labs:** Apply non-destructive socket/AFO rectifications without topology drift, export watertight shells (`STL` + slicer-targeted `.3mf` reinforcement modifiers) *or* CNC foam-carver molds (`export_carver_positive`), and auto-generate custom-device traceability dossiers.
* **Biomedical / CAD Engineers:** Run deterministic, zero-dependency Python geometry experiments (`EXP-1`–`EXP-5`) without Blender (`bpy`), VTK, or OpenVDB installation friction.

---

## End-to-End Pipeline

```
  Raw Scan (.stl)
        │
        ▼
  [ 1. op-core qa ] ────► Prune floaters, flag double walls/seam steps/tears,
        │                 align +Z proximal, slice 20 mm MPT circumferences
        ▼
  [ 2. .oprecipe.json ] ► Non-destructive liner offset, compression, reliefs,
        │                 & AFO ankle posture evaluated in reference domain M_0
        ▼
  [ 3. op-core build ] ──┬─► device_shell.stl      (watertight, MPT/AFO aligned)
                         ├─► device_<slicer>.3mf   (100% distal/neck modifiers)
                         ├─► carver_positive.stl   (keyed mandrel + 12° collar)
                         └─► device_dossier.json/md (safety lint + audit log)
```

---

## 60-Second Quickstart (Zero External Dependencies)

Works immediately with **Python 3.11 / 3.12+** using only the standard library:

```bash
# 1. Run an end-to-end offline demo (generates sample scan, recipe, STL, .3mf, carver mold, dossier)
PYTHONPATH=src python3 -m op_core.cli demo --out-dir ./demo_tt --device tt --target orcaslicer

# 2. Try the Ankle-Foot Orthosis (AFO) demo
PYTHONPATH=src python3 -m op_core.cli demo --out-dir ./demo_afo --device afo --target prusaslicer

# 3. Inspect CLI help for all commands (demo, qa, build, lint, diff)
PYTHONPATH=src python3 -m op_core.cli --help
```

*(Optional editable install: `pip install -e .` enables `op-core ...` directly on your `PATH`.)*

---

## Navigation Map

### User Guides & Specifications
| Page | What You Will Find |
| :--- | :--- |
| [**Getting Started**](./docs/getting_started.md) | Step-by-step walkthroughs for `demo`, `qa`, `build`, `diff`, and inspecting every output file. |
| [**CLI & Tools Reference**](./docs/cli_and_tools_reference.md) | Complete flags, defaults, and Python module API guide for `op-core` and `scripts/run_experiments.py`. |
| [**`.oprecipe.json` Schema**](./docs/oprecipe_schema.md) | Canonical coordinate frame (`+Z` proximal, `+Y` anterior) and field-by-field authoring cookbook. |
| [**Examples (`examples/`)**](./examples/tt_socket.oprecipe.json) | Ready-to-run [`tt_socket.oprecipe.json`](./examples/tt_socket.oprecipe.json), [`tt_socket_v2.oprecipe.json`](./examples/tt_socket_v2.oprecipe.json), and [`afo_leaf.oprecipe.json`](./examples/afo_leaf.oprecipe.json). |

### Strategy, Research & Benchmarks
| Page | What You Will Find |
| :--- | :--- |
| [**Practitioner Needs Research**](./research/practitioner_software_needs.md) | Clinical & technician pain points, audit of `ufit-blender`, `orthopen`, `ampscan`, and `Meshmixer`, plus priority rankings. |
| [**Action Plan (v2.1)**](./o_p_technology_action_plan.md) | Architecture blueprint (`op-core` MIT kernel + thin UI adapters), hardware specs, licensing boundaries, and 16-week roadmap. |
| [**Validation Experiments**](./experiments/high_value_experiments.md) | Hypotheses, datasets, protocols, and quantitative pass/fail gates for `EXP-1` through `EXP-6`. |

### Code & Verification
* **Core Package:** [`src/op_core/`](./src/op_core/__init__.py) (`cli`, `scan_qa`, `recipe`, `rectify`, `shell`, `export_3mf`, `linter`, `mesh`, `math3d`, `synthetic`).
* **Unit Tests (`15` tests):** `PYTHONPATH=src python3 -m unittest discover -s tests -v`
* **Experiment Runner (`EXP-1`–`EXP-5`):** `PYTHONPATH=src python3 scripts/run_experiments.py`
* **License:** [`LICENSE`](./LICENSE) (MIT).
