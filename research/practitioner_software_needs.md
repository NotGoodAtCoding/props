[Home (README)](../README.md) | [Getting Started](../docs/getting_started.md) | [CLI & Tools Reference](../docs/cli_and_tools_reference.md) | [`.oprecipe.json` Schema](../docs/oprecipe_schema.md) | [Action Plan](../o_p_technology_action_plan.md) | **Research** | [Experiments](../experiments/high_value_experiments.md)

---

# Research Synthesis: Top-Priority Open-Source Software Needs in Prosthetics & Orthotics (O&P)

> **Who this page is for:** Clinical stakeholders, O&P educators, and developers seeking the evidence behind why `op-core` prioritizes decoupled `.oprecipe.json` workflows, orthotics (AFO) parity, scan QA, and dual `.3mf`/carver outputs.  
> **Looking to run the software?** See [Getting Started](../docs/getting_started.md).  
> **Purpose:** Audit the assumptions in the initial [`o_p_technology_action_plan.md`](../o_p_technology_action_plan.md) against real-world clinical and fabrication workflows, existing open-source tooling, and regulatory/mechanical constraints faced by practicing Certified Prosthetist Orthotists (CPOs) and O&P technicians.

---

## 1. Executive Findings: Where the Initial Draft Diverged from Practice

The initial action plan correctly identified high proprietary CAD licensing costs (e.g., Vorum Canfit, Rodin4D, WillowWood) and mesh-processing fragility as key blockers. Cross-referencing industry surveys, clinical practice analyses, existing open-source repositories, and senior O&P engineering review reveals **six major blind spots** that change project priorities:

1. **Redundancy with `ortigital/ufit-blender` without targeting its real failure modes:** `ufit-blender` already implements a guided Blender wizard (for Blender 3.6/4.0+) with scan import/cleanup, automated circumferences, pressure-point sculpting, trimlines, variable wall thickness, connector alignment, and STL export. Simply rebuilding a Blender socket wizard duplicates effort. The actual unmet needs are **resilience on messy mobile scans (seam steps, double walls, floaters), true non-destructive parameter replay, decoupled geometry logic, and orthotics support**.
2. **Orthotics volume dwarfs prosthetics volume in everyday clinics:** Industry practice analyses (ABC Practice Analysis / AOPA) consistently show general O&P practitioners spend a greater share of clinical volume on **orthotics** than prosthetics. Lower-extremity orthoses account for roughly **60%** of orthotics focus, led by **Ankle-Foot Orthoses (AFOs) at ~28–30%** of lower-extremity volume (alongside spinal 15–21%, scoliosis ~10%, foot orthoses ~9%, and upper-extremity ~9%). Yet the initial plan was ~90% transtibial socket-centric.
3. **Device-specific alignment, liner offsets, and hardware interfaces matter:**
   * **Transtibial (TT) prosthetics:** Aligns the **socket bisection / Mid-Patellar Tendon (MPT) plumb line** (typically ~5° flexion, ~5° adduction) relative to the foot keel / pyramid center (note: *Trochanter–Knee–Ankle [TKA]* alignment applies to **transfemoral [TF]** prostheses with a prosthetic knee joint, not TT). Rectification must also account for **liner thickness offsets (0–6 mm)** and distinguish PTB vs. Total Surface Bearing (TSB) logic.
   * **Transfemoral (TF) prosthetics:** Requires distinct anatomy (ischial containment/ramus shelf, Scarpa's triangle, adductor channel, trochanteric relief) and TKA alignment.
   * **Ankle-Foot Orthoses (AFOs):** Never use a 4-hole `50.8 mm` pyramid boss. AFOs require smooth biharmonic/skinning ankle posture correction, **plantar sole flattening at footwear heel-height pitch**, malleolar relief build-ups, and **coaxial medial/lateral joint cavities** (e.g., Tamarack/Becker) recessed into malleolar zones—plus `R >= 8–10 mm` trimline fillets to prevent cyclic "frog mouthing" cracks.
   * **3D-printed distal mounts:** Bare printed bosses strip under ISO 10328 loads; printable distal mounts need **captive M6 nut pockets / threaded brass heat-set insert bosses** or lamination ring recesses.
4. **Hybrid CNC foam carving & thermoforming still dominate clinical production:** Most clinics and central fabrication labs do not exclusively 3D-print definitive sockets. Because 3D-printed polymers remain harder to spot-adjust chairside with a heat gun and carry anisotropic Z-layer fatigue liability, **hybrid workflows** (3D scan $\rightarrow$ CAD rectification $\rightarrow$ CNC foam positive carving $\rightarrow$ thermoformed PETG check socket or carbon-fiber lamination) are widespread. Open-source workflows need **carver-ready positives** featuring **keyed mandrel bores** (so foam blocks do not spin under cutting torque or vacuum draw) and **10–15° tapered thermoforming draft collars** (so cooled PETG releases cleanly).
5. **A "watertight STL" is not a safe load-bearing device:** Under **ISO 10328** and **ISO 22523**, FFF/FDM 3D-printed sockets routinely fail via **inter-layer Z-axis delamination** near the distal pyramid interface, while 3D-printed AFOs fail from cyclic fatigue around sharp malleolar trimlines. Technicians need **slicer-specific `.3mf` reinforcement modifier schemas** (`Slic3r_PE_model.config` for PrusaSlicer vs. `model_settings.config` for OrcaSlicer) and deterministic geometry/wall-thickness/notch linters.
6. **Clinician–Technician handoff, L-code documentation, and scan QA cause daily remakes:** Both roles lose substantial time to bad mobile scans discovered after the patient leaves, "black-box" central-lab order handoffs, untraceable check-socket revisions, and payer/regulatory documentation (US HCPCS L-code custom-fabrication justification, WOPD/QMSR; EU MDR 2017/745 Annex XIII custom-made device dossiers).

---

## 2. Role-Separated Pain Point Matrix

### 2.1 Practicing Clinicians (CPOs / Orthotists / Prosthetists)

| Workflow Stage | Daily Pain Point | Root Software Gap |
| :--- | :--- | :--- |
| **Shape Capture** | Mobile scanning (iPhone TrueDepth / LiDAR) suffers loop-closure seam steps, double-walled surfaces, posture tilt, and background floaters; missing soft-tissue "feel." | No chairside **Scan QA & Landmark-Anchored Tape Cross-Check Gate** that validates mesh completeness and landmark-referenced circumferences in <5 seconds before the patient leaves. |
| **Posture & Alignment** | Foot/ankle scans captured plantarflexed/inverted; TT flexion/adduction or TF TKA alignment eyeballed in world space; liner thickness ignored. | Lack of **pre-rectification anatomical alignment, liner offset, and smooth ankle articulation + sole flattening**. |
| **CAD Literacy** | Desktop mesh tools (Blender, FreeCAD) force modal hotkeys and 3D-artist concepts onto time-pressed clinicians; Autodesk discontinued **Meshmixer**. | Lack of a simplified, clinical-vocabulary review & adjustment UI (and clean replacement for Meshmixer's "digital clay" triage). |
| **Iteration & Remakes** | Explaining check-socket adjustments ("relieve lateral fibular head 3 mm, drop posterior trimline 5 mm") to central fab or techs leads to miscommunication. | No standardized, diffable **`.oprecipe.json` Rectification Manifest** linking scan v1, check socket v1, and fit delta v2. |
| **Documentation & Compliance** | Heavy burden proving medical necessity (K-levels, L-codes, custom-fabricated vs. custom-fitted) and generating custom-device safety dossiers (EU MDR Annex XIII / ISO 22523 / FDA QMSR). | CAD modifications are completely disconnected from clinical measurement forms and structured audit exports. |

### 2.2 O&P Technicians & Central Fabrication Labs

| Workflow Stage | Daily Pain Point | Root Software Gap |
| :--- | :--- | :--- |
| **Mesh Triage** | Hours wasted manually patching non-manifold edges, double walls, floating scan islands, and open proximal cuffs on incoming STL/OBJ/PLY files. | Fragile sequential wizards fail silently or crash on dirty real-world scans. |
| **Re-work on Rectifications** | Destructive sculpting forces technicians to restart or distort surrounding reliefs when a clinician requests a localized change late in the design. | Lack of a **re-executable, reference-domain parametric displacement stack** decoupled from deformed mesh normals. |
| **Distal Hardware & Trimlines** | Brittle Boolean failures when fusing 4-hole pyramid adapters (captive nut/insert bosses) or Tamarack/Becker AFO joints; jagged trimlines creating stress risers. | Lack of **narrow-band SDF / `manifold3d` shelling, smooth trimline filleting + brim rounding, and parametric hardware bossing**. |
| **3D Print Structural Safety** | Manually painting modifier meshes in PrusaSlicer/OrcaSlicer to enforce 100% infill / extra wall loops around pyramid adapters and AFO ankle necks; Z-delamination risk. | No automated **slicer-targeted `.3mf` modifier compiler** or pre-print thickness/notch linter. |
| **Hybrid Carver Support** | Labs using 3/4-axis CNC foam carvers get locked into Vorum/Rodin formats or must manually add keyed mandrel bores and tapered thermoforming collars. | No one-click **"Export Carver Positive"** (`export_carver_positive`) with keyed mandrels, tapered draft collars, and AFO swing-radius canting checks. |

---

## 3. Audit of Existing Open-Source & Free Ecosystem

| Project / Tool | License | Primary Domain | Verified Strengths | Critical Gaps for Practicing Clinics |
| :--- | :--- | :--- | :--- | :--- |
| **`ortigital/ufit-blender`** | GPL-3.0 | TT & TF Prosthetic Sockets (Blender 3.6/4.0+) | Guided step-by-step Blender add-on; scan cleanup, circumferences, scaling/pressure points, trimlines, variable thickness, connector alignment, STL export. | Tightly coupled to Blender UI/sequential operators; fails on severely flawed mobile scans; limited orthotics (AFO/spinal) workflow; no `.3mf` modifier automation or foam-carver prep. |
| **`OTA3D/orthopen`** | GPL-3.0 | Orthopedic Aids / Orthoses (Blender 3.X+) | Novice-oriented UI; Git LFS asset library; `MeasureIt` integration. | Early/narrow feature coverage; Blender-coupled; lacks automated AFO posture correction, sole flattening, malleolar stress-riser guards, and `.3mf`/carver outputs. |
| **`abel-research/ampscan`** | MIT | Scan Analysis & Shape Comparison (Python + VTK/PyQt) | Rigorous ICP registration, volumetric metrics, cross-sectional slices, deviation heatmaps; `ampscan_webapp` companion. | Analysis-only; heavy **VTK + PyQt + Matplotlib** dependency makes it unsuitable as a hard dependency inside a lightweight headless `op-core` package. |
| **`abel-research/OpenLimbTT` & `OpenLimbTF`** | CC-BY-SA 4.0 / CC-BY 4.0 | Synthetic Residual Limb Datasets | PCA-based statistical shape models with internal bone geometry (`.stl`) for GDPR-safe benchmarking & FEA. | Normalized fractional scale & closed meshes (must be scaled to clinical mm and cropped for cuff tests); **contains no ankle/foot meshes for AFO testing**. |
| **`SemaphoreTO/socketmixer`** | BSD-3-Clause | TT Sockets via Meshmixer API | Streamlined Python/web automation over Meshmixer for low-resource settings. | Relies on **Autodesk Meshmixer**, which is officially **discontinued and unsupported** by Autodesk. |
| **Autodesk Meshmixer** | Freeware (Closed) | General Mesh Editing | Grassroots O&P favorite for digital clay sculpting and hole repair. | **Sunsetted/unmaintained**; OS/Apple Silicon instability; zero clinical logic or audit trail. |
| **FreeCAD / OpenSCAD** | LGPL-2+ / GPL-2+ | Mechanical Components & Jigs | True parametric B-rep CAD (`STEP`), `Curves` & `CAM` workbenches. | Poor ergonomics for organic limb sculpting and fast freeform rectification. |
| **`CalculiX` / `Gmsh` / `PyCCX`** | GPL-2 / GPL-2+ / BSD-2 | FEA Meshing & Structural Solver | `PyCCX` + `Gmsh` + `CalculiX` automate anisotropic shell/solid static checks. | `Gmsh` (`GPL-2+`) and `CalculiX` (`GPL-2`) require a **strict subprocess boundary (`op-fea`)** away from MIT `op-core`, and require physical coupon calibration before clinical reliance. |
| **`OpenCap` / `MediaPipe`** | Apache-2.0 | Markerless Gait Kinematics | Smartphone-based 3D kinematics (`OpenCap` + `OpenSim`) and real-time pose tracking (`MediaPipe`). | Clinical outcome tracking / K-level assessment aid; complementary downstream module rather than CAD blocker. |

---

## 4. Architectural & Licensing Strategy (Lean MIT Core + Strict Boundaries)

Because this repository is **MIT-licensed**, tightly coupling computational geometry inside Blender (`bpy`) or importing GPL/heavy visualization libraries creates avoidable packaging and legal traps:
1. **Keep `op-core` strictly `pip install`-able and MIT/Apache/BSD pure:** Depend only on `numpy`, `scipy`, `trimesh`, and `manifold3d` (with pure-NumPy/C++ narrow-band distance evaluation rather than `OpenVDB`, which lacks official PyPI wheels, and without requiring `ampscan`/`VTK`/`PyQt` at runtime).
2. **Enforce GPL/AGPL isolation:** Any future `CalculiX`/`Gmsh` FEA (`op-fea`, GPL-2/GPL-2+) or Blender workspace adapter (GPL-3.0) runs across a clean CLI/subprocess or `.oprecipe.json` boundary—never imported inside `op-core`.
3. **Pick one thin UI first:** Instead of maintaining both a Blender workspace and a web app in MVP, keep Blender optional and target a single lightweight local browser UI (`Three.js` + `.oprecipe.json`) once the headless kernel passes EXP-1–3.

---

## 5. Prioritized Need Ranking (MVP-Sequenced)

| Priority | Need | Primary Beneficiary | Why Now |
| :--- | :--- | :--- | :--- |
| **P0 (Critical)** | **`.oprecipe.json` v0.1 Contract + Lean `op-core` Headless Python Kernel** | Technicians & Clinicians | Eliminates restart penalty, enables deterministic CLI/CI replay, and keeps core MIT & `pip`-installable. |
| **P0 (Critical)** | **Chairside Scan QA, Auto-Alignment & Landmark-Anchored Tape Cross-Check** | Clinicians & Techs | Catches seam steps, double walls, floaters, and cuff errors chairside in `<5 s` before patient departure. |
| **P0 (Critical)** | **Reference-Domain Parametric Rectification Stack (TT PTB/TSB + Liner Offset)** | Technicians | Fixes topology drift when earlier global compression or liner offsets change late in the workflow. |
| **P1 (High)** | **Manifold Shelling, Brim Rounding, TT MPT Alignment + Captive-Nut Boss & Dual-CAM Export** | Technicians & Labs | Guarantees watertight shells, print-safe `.3mf` (`prusaslicer`/`orcaslicer` schemas), and keyed foam-carver positives. |
| **P1 (High)** | **AFO Module (Foot/Ankle Test Corpus, Smooth Articulation, Sole Flattening, Malleolar Fillets, Coaxial Joint Bosses)** | Orthotists & Techs | Addresses the highest-volume O&P clinical category with biomechanically valid geometry. |
| **P2 (Medium)** | **Print-Safety Geometry Linter & Custom-Device Audit Dossier (PDF/JSON)** | Clinicians & QA | Bridges open-source CAD into EU MDR Annex XIII / FDA QMSR / L-code custom-fab documentation. |
| **P2 (Medium)** | **Single Thin Browser Clinician Review UI (`Three.js`)** | Clinicians | Lowers training barrier without duplicating effort across dual Blender + Web UIs. |
