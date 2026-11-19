# Open-Source Orthotics & Prosthetics (O&P) CAD/CAM Action Plan (v2.1 — Reviewer-Refined)

> **Context:** Refined from initial v1/v2 drafts using practitioner research in [`research/practitioner_software_needs.md`](./research/practitioner_software_needs.md) and technical/clinical review. Validation protocols are defined in [`experiments/high_value_experiments.md`](./experiments/high_value_experiments.md).

## Executive Summary
The orthotics and prosthetics (O&P) industry is transitioning away from manual plaster-casting toward digital scan-to-fabrication pipelines, yet practicing clinicians (CPOs) and technicians remain bottlenecked between costly proprietary suites (Canfit/Vorum, Rodin4D, WillowWood) and fragmented or discontinued free tools (Autodesk Meshmixer, which is unsupported, or Blender-coupled scripts that fail on imperfect mobile scans).

Auditing existing open-source tools—specifically **`ortigital/ufit-blender`** (socket design), **`OTA3D/orthopen`** (orthoses), **`abel-research/ampscan`** (scan deviation/measurement), and **`OpenLimbTT`/`OpenLimbTF`**—reveals that **rebuilding another sequential Blender socket wizard is low-leverage**. The highest-value open-source gaps for practicing clinicians and technicians are:

1. **Lean, Pip-Installable MIT Core (`op-core` + `.oprecipe.json`):** Decoupling a headless Python geometry kernel (`numpy`, `scipy`, `trimesh`, `manifold3d`) from GPL-3.0 Blender (`bpy`) and heavy VTK/OpenVDB dependencies so rectifications are strictly parametric, deterministic, CI-testable, and portable across CLI, thin web UIs, and central-fab portals.
2. **Anatomically Accurate Prosthetic & Orthotic Primitives:**
   * **Transtibial (TT) & Transfemoral (TF) Sockets:** Distinct TT **socket bisection / Mid-Patellar Tendon (MPT) plumb-line alignment** (~5° flexion, ~5° adduction) and **liner thickness offsets (0–6 mm)** vs. TF **ischial containment + Trochanter–Knee–Ankle (TKA)** alignment; **captive M6 nut / heat-set insert pockets** inside 4-hole `50.8 mm` BCD pyramid mounts.
   * **High-Volume Orthotics (AFOs):** Smooth biharmonic ankle posture correction, **plantar sole flattening at footwear heel-height pitch**, malleolar relief build-ups, `R >= 8–10 mm` trimline fillets against cyclic "frog mouthing" fatigue, and **coaxial medial/lateral joint cavities** (no pyramid bosses).
3. **Dual CAM Output (Slicer-Targeted `.3mf` + Hybrid Foam-Carver Positives):** Supporting both 3D printing (explicit `--target prusaslicer | orcaslicer` `.3mf` reinforcement modifiers + geometry/notch linters) and subtractive CNC foam carving (`export_carver_positive` with **keyed mandrel bores** and **10–15° tapered thermoforming draft collars**).

---

## 1. Strategic Architecture: Lean MIT Kernel + Strict Boundaries

```
      [ Single Thin Browser Review UI (Three.js — Phase 4) ]   [ Optional Blender Adapter (v0.2) ]
                                    \                                       /
                                     +--------->  .oprecipe.json  <--------+
                                           (v0.1 Schema: Units, Side, Landmarks,
                                            Pinned Kernel Version, Edit Stack)
                                                        |
                                                        v
                         [ op-core (MIT-Licensed, Pip-Installable Python Kernel) ]
                         - Deps: numpy, scipy, trimesh, manifold3d (NO VTK, NO OpenVDB)
                         - Scan QA & Landmark-Anchored Tape Cross-Check
                         - Reference-Domain Geodesic Displacement Stack (M_0)
                         - Narrow-Band Shelling + Brim Rounding + Hardware Bosses
                                                        |
                        +-------------------------------+-------------------------------+
                        v                               v                               v
            [ Slicer-Targeted .3MF ]       [ Foam-Carver Positive .STL ]     [ Geometry Linter & Dossier ]
          (--target prusaslicer vs         (Keyed mandrel bore, 10-15°      (Wall/notch/orientation checks,
           orcaslicer modifier XML)         draft collar, undercut check)    MDR Annex XIII / L-code PDF)
                                                                                        |
                                                                      [ Optional Subprocess: op-fea (v0.2) ]
                                                                      (CalculiX GPL-2 / Gmsh GPL-2+ CLI)
```

---

## 2. Prioritized High-Value Projects

### Project 0 (Foundation): `op-core` & `.oprecipe.json` v0.1 Contract (P0)
* **Problem:** Mesh edits in Meshmixer/Blender are opaque binary blobs. Re-aligning a limb orphans spatial edits, and library upgrades alter output geometry silently.
* **Solution:** Define `.oprecipe.json` v0.1 with strict clinical and reproducibility metadata:
  * `schema_version`, pinned `kernel_version`, `units: "mm"`, `side: "left" | "right"`.
  * Input scan SHA-256 checksum + detected/calibrated scale factor.
  * Canonical anatomical coordinate frame (`+Z` proximal, `+Y` anterior) established via explicit alignment transform.
  * **Mesh-anchored landmarks** (barycentric face index + UV weights on `M_0`, or canonical post-alignment coordinates) so re-alignment never orphans landmarks.
  * Ordered, reversible parametric operation list (supporting clean JSON diffs for check-socket v1 $\rightarrow$ v2 revisions).
* **Target Output:** Pure-MIT `op-core` Python package & CLI compiling `scan + .oprecipe.json -> positive/shell + manifest`.

### Project 1: Chairside Scan QA, Auto-Alignment & Landmark-Anchored Tape Gate (P0)
* **Problem:** Mobile scans (TrueDepth/LiDAR) exhibit non-rigid loop-closure seam steps, double-walled surfaces, background table floaters, posture tilt, and scale drift—discovered only after the patient leaves.
* **Solution:** Fast (`<5 s`) pure-NumPy/SciPy/`trimesh` intake gate:
  * **Artifact & Topology Triage:** Distinguish single intended proximal open cuff from surface tears, detect double-walled overlapping shells, and prune disconnected floaters.
  * **Anatomical Alignment & Landmark-Anchored Slicing:** Align canonical limb axis and slice at `20 mm` increments **anchored distally from an anatomical landmark plane** (e.g., Mid-Patellar Tendon / knee joint line, *never* from the variable distal apex) so visit-to-visit volume tracking stays invariant.
  * **Taut-Tape Cross-Check:** Compute convex-hull perimeter (matching physical tape bridging concavities) vs. clinical intake tape measurements + optional `ampscan` bridge export.
* **Target Output:** Chairside Pass/Warn/Fail QA report, canonical aligned mesh `M_0`, and PDF measurement sheet.

### Project 2: Reference-Domain Parametric Rectification & Posture Stack (P0)
* **Problem:** Sequential sculpting on deformed meshes shifts surface normals and breaks early global edits; global cylindrical `(z_norm, theta)` coordinates fail on flexed knees and L-shaped foot/ankle scans.
* **Solution:**
  * **Reference-Domain (`M_0`) Geodesic Patch Math:** Compute smoothed reference normals `N_0` and geodesic patch falloffs (Dijkstra / heat-method distance from mesh-anchored landmarks) on the undeformed canonical mesh `M_0`, combined with slice-wise cross-sectional scaling for global `%` reductions and explicit **liner thickness offsets (`0–6 mm`)**.
  * **TT & TF Prosthetics:** TT PTB/TSB primitives (PT bar, popliteal depression, fibular head/tibial crest reliefs, distal end pad) and TF primitives (ischial containment/ramus shelf, Scarpa's triangle, adductor channel, trochanteric relief).
  * **AFO Posture & Sole Contouring (Phase 3):** Smooth biharmonic/skinning ankle articulation across the oblique talocrural axis, **plantar sole flattening at prescribed footwear heel-height pitch**, metatarsal-head trimlines, and malleolar relief build-ups.
* **Target Output:** Isolated parameter mutations (`<0.2 mm` unintended drift in unedited zones) and clean v1 $\rightarrow$ v2 check-socket diffs.

### Project 3: Manifold Shelling, Bench Alignment & Dual-CAM Compiler (P1)
* **Problem:** `manifold3d` is a CSG engine (not an automatic variable-thickness SDF offsetter), `OpenVDB` lacks PyPI wheels, sharp Boolean trimlines cause skin irritation and AFO fatigue cracks, and bare printed pyramid bosses strip under load.
* **Solution:**
  * **Pip-Native Shelling & Edge Rounding:** Build a fast narrow-band distance grid / surface-offset solid cleaned with `manifold3d` (`Apache-2.0`) for variable thickness (`4.5–5.5 mm` structural zones tapering to `2.5–3.0 mm` flexible brims), geodesic trimline smoothing (`R >= 8–10 mm` malleolar fillet guard), and cross-sectional brim edge rounding.
  * **Device-Specific Alignment & Hardware Bossing:**
    * *TT Sockets:* Apply **MPT / socket bisection plumb-line alignment** (~5° flexion, ~5° adduction) and union a 4-hole `50.8 mm` BCD boss with **captive M6 nut / brass heat-set insert pockets** (and optional pin-lock/valve bore).
    * *TF Sockets:* Apply **TKA plumb-line alignment** and variable hip-flexion-contracture accommodation.
    * *AFOs:* Apply **coaxial medial/lateral joint cavities** (e.g., Tamarack/Becker) recessed into malleolar build-ups orthogonal to line of progression (no pyramid boss).
  * **Dual CAM Outputs:**
    1. **Slicer-Targeted `.3mf`:** Explicit `--target prusaslicer` (`Metadata/Slic3r_PE_model.config`) and `--target orcaslicer` (`Metadata/model_settings.config`) archives embedding `100%` infill / high wall-loop modifier meshes around distal bosses and AFO ankle necks.
    2. **Foam-Carver Positive (`export_carver_positive`):** Axis-locked positive with **keyed mandrel bore** (square/flat keyway preventing cutting/vacuum spin), **10–15° tapered thermoforming draft collars**, and 4-axis undercut/AFO-canting diagnostics.
* **Target Output:** 100% watertight meshes (`0` slicer repair flags) ready for either 3D printing or CNC foam carving.

### Project 4: Print-Safety Geometry Linter & Custom-Device Audit Dossier (P1/P2)
* **Problem:** Clinicians need verifiable safety checks and custom-made device documentation (EU MDR Annex XIII, ISO 22523, FDA QMSR/ISO 13485, US L-code custom-fab proof), while linear-elastic FEA without physical Z-peel coupon calibration risks false confidence.
* **Solution:**
  * **MVP Geometry & Printability Linter:** Fast deterministic checks for minimum structural wall thickness, trimline notch radius (`R >= 8 mm`), captive-nut boss wall clearance, and load-axis vs. print-Z peel angle.
  * **Fabrication Dossier Export:** Auto-generates PDF + JSON archival bundle recording scan checksum, tape cross-checks, pinned `kernel_version`, `.oprecipe.json` log, and material/component lot placeholders.
  * **Post-MVP (`op-fea` v0.2 Subprocess):** Separate `PyCCX`/`Gmsh`/`CalculiX` subprocess wrapper calibrated against physical ISO 10328 printed coupons.

### Project 5: Single Thin Browser Clinician Review UI (P2)
* **Problem:** Maintaining both a Blender workspace (`bpy`) and a Web UI in v0.1 duplicates effort and keeps clinicians tied to Blender's modal interface.
* **Solution:** Build one lightweight local web UI (`Three.js` + FastAPI/static local server) driving `op-core` via `.oprecipe.json` in 5 constrained steps: **Scan QA $\rightarrow$ Landmark Pin $\rightarrow$ Rectification Sliders $\rightarrow$ Alignment & Trimline Preview $\rightarrow$ Dual-CAM + Dossier Export**. Keep a Blender Geometry Nodes / `ufit-blender` bridge as a v0.2 technician plugin.

---

## 3. Refined Technology Stack & Licensing Boundaries

| Layer | Primary Tech | License | Boundary / Justification |
| :--- | :--- | :--- | :--- |
| **Core Kernel (`op-core`)** | **Python 3.12, `NumPy`, `SciPy`, `trimesh`, `manifold3d`** | MIT / BSD / Apache-2.0 | Strictly `pip install`-able; headless CI-testable; no `OpenVDB` wheel headaches or `VTK`/`PyQt` bloat. |
| **Metrology & Datasets** | **Native NumPy/SciPy ICP** + optional **`ampscan`** bridge; **`OpenLimbTT`/`OpenLimbTF`** | MIT / CC-BY-SA 4.0 | Native `op-core` slicing/ICP; datasets fetched via test-fixture scripts (not vendored). |
| **Slicer & Carver Targets** | **`.3mf` (`--target prusaslicer \| orcaslicer`)**, **keyed carver `.stl`** | Outputs (Data) / Slicers AGPL-3.0 | Explicit slicer-specific XML schemas; open `FreeCAD CAM` / `CAMotics` G-code automation deferred to v0.2. |
| **Structural FEA (v0.2)** | **`PyCCX` + `Gmsh` + `CalculiX`** | BSD-2 / GPL-2+ / GPL-2 | Isolated in separate `op-fea` **subprocess** package; never imported by MIT `op-core`. |
| **Clinical UI (Phase 4)** | **Local Browser UI (`Three.js`)** | MIT | Single thin MVP UI driving `.oprecipe.json`; Blender workspace deferred to v0.2. |

---

## 4. Revised Phased Implementation Roadmap (16 Weeks — TT-First MVP)

```
Phase 1: Core Contract, Scan QA & Benchmarks (Weeks 1–4)
├── Define .oprecipe.json v0.1 schema and bootstrap lean MIT op-core package
├── Build test-fixture fetcher for OpenLimbTT / OpenLimbTF (scaled to mm + cropped cuffs)
├── Implement Project 1: Scan triage (seam/double-wall/floater), MPT-anchored 20 mm slicing & tape check
└── Run EXP-1 (Scan-to-Tape & Corrupt-Scan QA Benchmark)

Phase 2: TT Rectification Stack, Shelling & Dual-CAM MVP (Weeks 5–9)
├── Implement reference-domain (M_0) geodesic displacement stack + liner offset (Project 2)
├── Build narrow-band shelling + manifold3d brim rounding, MPT bench alignment & captive-M6 boss (Project 3)
├── Implement --target prusaslicer | orcaslicer .3mf modifier export + export_carver_positive (keyed bore + draft collar)
└── Run EXP-2 (Isolation Drift Test) & EXP-3 (TT Shelling/Alignment Stress Test)

Phase 3: AFO Module, Safety Linter & Audit Dossier (Weeks 10–13)
├── Build procedural/open foot-ankle AFO test corpus; implement biharmonic ankle articulation & sole flattening
├── Add malleolar R>=8–10 mm trimline fillet guard & coaxial joint cavities (EXP-2B / EXP-3B)
├── Ship print-safety geometry linter & slicer .3mf verification (EXP-4 MVP) + carver prep check (EXP-5 MVP)
└── Auto-generate custom-device fabrication dossier (PDF + JSON audit bundle)

Phase 4: Single Thin Clinician Web UI & Pilot (Weeks 14–16)
├── Build 5-step Three.js local clinician review UI backed by op-core + .oprecipe.json
└── Run EXP-6 (Timed usability crossover against ufit-blender baseline)
   [v0.2 Post-MVP Backlog: CalculiX op-fea calibrated on physical coupons, Blender Workspace adapter, 4-axis G-code CAM, OpenCap gait bridge]
```

---

## 5. Key Reference Ecosystem & Standards
* **Open-Source O&P Projects:** [`ortigital/ufit-blender`](https://github.com/ortigital/ufit-blender) (GPL-3.0), [`OTA3D/orthopen`](https://github.com/OTA3D/orthopen) (GPL-3.0), [`abel-research/ampscan`](https://github.com/abel-research/ampscan) (MIT), [`SemaphoreTO/socketmixer`](https://github.com/SemaphoreTO/socketmixer) (BSD-3-Clause), `neurobionics/opensourceleg`.
* **Open Anatomical Datasets:** [`abel-research/OpenLimbTT`](https://github.com/abel-research/OpenLimbTT) & `OpenLimbTF` (CC-BY-SA 4.0 / CC-BY 4.0).
* **Standards & Regulatory Frameworks:** **ISO 10328** (lower-limb prosthetic structural testing), **ISO 22523** (external limb prostheses and orthoses requirements), **EU MDR 2017/745 Annex XIII** (custom-made device documentation), **FDA QMSR / ISO 13485 & Section 520(b)**.
