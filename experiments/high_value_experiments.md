# High-Value Validation Experiments for Open-Source O&P Software (v2 — Reviewer-Refined)

> **Purpose:** De-risk the highest-leverage technical and clinical-workflow assumptions in [`o_p_technology_action_plan.md`](../o_p_technology_action_plan.md) through fast, falsifiable experiments before investing in full UI or post-MVP integrations.

---

## Experiment Portfolio Summary (MVP Scope)

| ID | Experiment | Target Project | Primary Risk Tested | Scope / Status |
| :--- | :--- | :--- | :--- | :--- |
| **EXP-1** | Scan QA & Landmark-Anchored Tape Cross-Check | Project 1 | Can pure `op-core` triage seam steps/double-walls/tilts & match landmark tape measures in `<5 s`? | **MVP (Weeks 2–4)** |
| **EXP-2** | Reference-Domain (`M_0`) Displacement Stack & Isolation | Project 0 & 2 | Does evaluating geodesic patches on `M_0` keep zoned drift & relief-height invariance `<0.2 mm`? | **MVP (Weeks 5–6)** |
| **EXP-2B** | AFO Biharmonic Ankle Articulation & Sole Flattening | Project 2 | Can a plantarflexed foot-ankle mesh be neutralized & sole-flattened at heel pitch without pinching? | **MVP (Weeks 10–11)** |
| **EXP-3** | `manifold3d` Shelling, Brim Rounding & Bench Alignment | Project 3 | Does pip-native shelling + MPT alignment + captive-nut pyramid / AFO joint bossing hit 100% watertightness? | **MVP (Weeks 7–12)** |
| **EXP-4** | Dual-Schema `.3mf` Modifiers & Geometry Safety Linter | Project 3 & 4 | Do PrusaSlicer & OrcaSlicer parse separate `.3mf` modifier schemas + catch thin/sharp notches? *(FEA deferred to v0.2 EXP-4B)* | **MVP (Weeks 12–13)** |
| **EXP-5** | Hybrid Carver Positive Prep (`export_carver_positive`) | Project 3 | Can rectified limbs get keyed mandrel bores, 10–15° tapered collars & undercut diagnostics cleanly? *(G-code/CAMotics deferred to v0.2 EXP-5B)* | **MVP (Week 13)** |
| **EXP-6** | Single 5-Step Web UI vs. `ufit-blender` Pilot | Project 5 | Does a stripped browser UI cut CPO/tech task time by ≥40% vs. `ufit-blender`? | **MVP (Weeks 15–16)** |

---

## EXP-1: Chairside Scan QA & Landmark-Anchored Tape Cross-Check

### Hypothesis
A lean Python pipeline (`numpy` + `scipy` + `trimesh`, without VTK/`ampscan` at runtime) can detect mobile scan artifacts (seam steps, double-walled overlaps, floaters, open cuffs) and compute **landmark-anchored** `20 mm` interval circumferences within **±1.5 mm** of physical tape measurements in **<5 seconds** on CPU.

### Datasets & Inputs
* **Normalized-to-clinical baseline:** 10 transtibial (`OpenLimbTT`) and 5 transfemoral (`OpenLimbTF`) meshes scaled from normalized fractional length to realistic clinical dimensions (e.g., `140–220 mm` TT residual limb length) and cropped proximally to form realistic open cuffs.
* **Realistic mobile-scan corruption suite:**
  1. Non-rigid **loop-closure seam step** (`2–4 mm` shear offset along one vertical strip).
  2. **Double-walled overlapping surface** patch + **2–3 detached background/table floaters**.
  3. Capture posture tilt (`15°` flexion/abduction) + `0.5 mm` Gaussian vertex jitter + `±3%` scale perturbation.
* **Ground truth:** Convex-hull cross-section perimeter (modeling taut tape bridging concavities) sliced every `20 mm` **distally from the Mid-Patellar Tendon (MPT) / proximal landmark plane** (NOT from the variable distal apex).

### Protocol
1. Implement floater pruning (keep primary connected component) and boundary-loop classifier (single proximal cuff cap vs. internal surface tears).
2. Implement double-wall / self-proximity detector and high-gradient seam-step detector.
3. Implement canonical alignment (`+Z` proximal via distal-to-proximal inertia/cuff centroid + 2-landmark MPT anchor) and `20 mm` convex-hull slicing downward from the MPT plane.
4. Benchmark runtime and circumference error across clean and corrupted cohorts.

### Pass / Fail Acceptance Criteria
* **Accuracy:** Mean circumference error `≤ 1.5 mm` (`max ≤ 2.5 mm`) across tilted and noisy scans anchored at the MPT landmark plane.
* **QA Detection:** `100%` separation of proximal cuff opening vs. internal tears; `≥ 90%` detection of injected `>2 mm` seam steps, double walls, and floaters.
* **Speed:** `< 5.0 s` per `100k`-triangle mesh on standard CPU.

---

## EXP-2: Reference-Domain (`M_0`) Parametric Displacement Stack & Isolation

### Hypothesis
Evaluating smooth geodesic displacement patches (Dijkstra / heat-method distance from `.oprecipe.json` mesh-anchored landmarks) along fixed smoothed reference normals `N_0` on the **undeformed canonical mesh `M_0`**—combined with slice-wise cross-sectional scaling for global `%` reductions and explicit liner thickness offset—keeps both zoned drift and relief-height invariance **<0.2 mm**.

### Datasets & Inputs
* 5 distinct `OpenLimbTT` shapes scaled to clinical mm.
* Standard TT sequence (`v1`): `3 mm` liner subtraction $\rightarrow$ `-3%` global compression $\rightarrow$ `-6 mm` PT bar $\rightarrow$ `-4 mm` popliteal depression $\rightarrow$ `+4 mm` fibular head relief & `+3 mm` tibial crest relief.
* Retroactive mutation (`v2`): Change global compression from `-3%` to `-1%` (or modify distal-only reduction) without re-editing localized reliefs.

### Protocol
1. Implement `op-core` `.oprecipe.json` displacement compiler operating strictly in the `M_0` domain.
2. **Case A (Zoned Isolation):** Mutate a distal-zone parameter (`0–40%` length) and measure Hausdorff deviation across unmodified proximal zones (`>60%` length).
3. **Case B (Additive Relief Invariance):** Mutate global compression (`-3% -> -1%`) and measure whether the relative peak height (`delta_h = relief_surface - base_compressed_surface`) of the PT bar and fibular relief stays invariant.
4. Verify deterministic replay across two independent CLI runs.

### Pass / Fail Acceptance Criteria
* **Case A (Zoned Isolation):** Unintended drift in unmodified zones `< 0.2 mm`.
* **Case B (Relief Invariance):** Relative relief peak height (`delta_h`) varies by `< 0.2 mm` under global `%` mutation.
* **Speed & Determinism:** Re-evaluation `< 3.0 s` on CPU; `< 0.01 mm` variance across repeated headless runs.

---

## EXP-2B: AFO Biharmonic Ankle Articulation & Plantar Sole Flattening

### Hypothesis
A smooth skinning / biharmonic blend across the oblique anatomical talocrural zone (paired with plantar ground-plane projection at a specified footwear heel-height pitch) can correct a `12°` plantarflexed foot-ankle scan to neutral without anterior ankle self-intersection or heel-cord collapse.

### Datasets & Inputs
* Procedural / open-source lower-leg + foot test generator producing 5 articulated foot-ankle `.stl` meshes (`0°` neutral and `10–15°` plantarflexed + `5°` inverted variants), because `OpenLimbTT`/`OpenLimbTF` lack feet.

### Protocol
1. Define smooth blend weights across the tibial shaft, oblique ankle hinge zone, and rigid calcaneal-forefoot frame.
2. Apply `12°` dorsiflexion + `5°` eversion correction, followed by plantar sole flattening at `10 mm` footwear heel-height pitch and `+4 mm` medial/lateral malleolar relief build-ups.
3. Inspect anterior ankle crease for self-intersections and measure sole flatness error.

### Pass / Fail Acceptance Criteria
* **Clean Articulation:** `0` self-intersecting faces in the anterior ankle crease; smooth normal continuity.
* **Sole & Pitch Accuracy:** Plantar contact zone flat within `±0.2 mm` at prescribed heel-height pitch.

---

## EXP-3: `manifold3d` Shelling, Brim Rounding & Device-Specific Alignment

### Hypothesis
A pip-installable narrow-band offset + `manifold3d` CSG pipeline generates **100% watertight, slicer-ready shells** across tight concavities while accurately executing **TT MPT bisection alignment + captive M6 nut pyramid bossing** and **AFO malleolar filleting + coaxial joint cavities**.

### Datasets & Inputs
* **TT Cohort (10 meshes):** Rectified `OpenLimbTT` positives from EXP-2 (plus 3 representative tight-concavity meshes to log baseline `Solidify`/normal-extrusion failures).
* **AFO Cohort (5 meshes):** Neutralized foot-ankle positives from EXP-2B.

### Protocol
1. **TT Pipeline:** Apply `manifold3d` variable-thickness shelling (`4.5 mm` distal / `2.5 mm` proximal brim), round proximal brim edges, apply **MPT bisection bench alignment** (`5°` flexion, `5°` adduction), and Boolean-union a `50.8 mm` BCD 4-hole boss incorporating **captive M6 nut pockets / brass insert bores**.
2. **AFO Pipeline:** Generate posterior/ankle-neck variable shell (`4.5 mm` neck / `2.5 mm` calf), enforce geodesic trimline fillet radius `R >= 10 mm` around malleoli, and cut/boss **coaxial medial/lateral Tamarack-style joint cavities** orthogonal to line of progression (**no pyramid boss**).
3. Validate every shell in `trimesh` (`is_watertight`, `is_winding_consistent`, `0` degenerate faces) and headless `PrusaSlicer` / `OrcaSlicer` CLI checks.

### Pass / Fail Acceptance Criteria
* **Manifold Integrity:** `15 / 15` (`100%`) watertight with `0` auto-repair errors in slicer CLI.
* **Alignment & Hardware:** TT pyramid plane within `±0.2°` of `5°` flexion / `5°` adduction; AFO joint axes coaxial within `±0.3°`; trimline minimum radius `R >= 8 mm`.
* **Speed:** `< 15 s` per compiled device.

---

## EXP-4: Dual-Schema `.3mf` Modifiers & Print-Safety Geometry Linter *(MVP)*

### Hypothesis
Emitting slicer-specific `.3mf` archives (`--target prusaslicer` via `Metadata/Slic3r_PE_model.config` and `--target orcaslicer` via `Metadata/model_settings.config`) paired with a fast deterministic geometry linter catches thin walls, sharp malleolar notches, and unsafe print orientations before slicing. *(Note: `CalculiX`/`Gmsh` FEA [`EXP-4B`] is deferred to v0.2 pending physical printed coupon Z-peel calibration.)*

### Protocol
1. Generate `.3mf` packages embedding an internal distal `40 mm` socket cup / AFO ankle-neck modifier mesh set to `100%` infill / `6` wall loops for both `--target prusaslicer` and `--target orcaslicer`.
2. Run geometry/printability linter on intentionally flawed test cases (`1.8 mm` thin wall, `3 mm` sharp malleolar notch, `45°` tilted socket print orientation).
3. Load `.3mf` files in PrusaSlicer and OrcaSlicer to verify modifier overrides apply automatically only to target sub-volumes.

### Pass / Fail Acceptance Criteria
* **Slicer Interop:** Both slicers parse modifier volumes and apply local `100%` infill/wall overrides without manual repositioning.
* **Linter Accuracy:** `100%` detection of `<2.5 mm` load-zone thickness and `<8 mm` trimline notch radii in `<2 s`.

---

## EXP-5: Hybrid Foam-Carver Positive Prep (`export_carver_positive`) *(MVP)*

### Hypothesis
`op-core` can convert any rectified TT/TF/AFO positive into a carver-ready foam mold featuring a **keyed mandrel bore** (preventing spin), **10–15° tapered thermoforming draft collars**, and an **axis/swing-radius undercut diagnostic** in `<10 s`. *(Note: Automated 4-axis G-code CAM & `CAMotics` round-trip [`EXP-5B`] is deferred to v0.2.)*

### Protocol
1. Implement `export_carver_positive` on 5 TT sockets and 2 AFOs (adding a `25.4 mm` square-keyed mandrel bore, `12°` tapered proximal/distal draft collars, and an AFO canting/undercut report checking 3-axis vs. 4-axis rotary clearance).
2. Validate output `.stl` watertightness and verify positive draft angles (`>= 10°`) on thermoforming release collars.

### Pass / Fail Acceptance Criteria
* **Integrity & Geometry:** `7 / 7` watertight carver positives; `0` undercut traps on draft collars; keyed bore aligns coaxially with carve axis within `±0.1°`.
* **Speed:** `< 10 s` per device.

---

## EXP-6: Single 5-Step Browser UI vs. `ufit-blender` Timed Pilot

### Hypothesis
A constrained 5-step local browser UI (`Three.js` + `.oprecipe.json`) reduces check-socket / AFO review and revision time by **≥40%** and eliminates fatal workflow dead-ends compared to **`ortigital/ufit-blender`** for practicing O&P clinicians/technicians.

### Protocol
1. Recruit `3–5` practicing CPOs, technicians, or O&P residents in a randomized within-subject crossover across 2 standardized tasks (1 TT check-socket revision; 1 AFO trimline & thickness spec) comparing **`ufit-blender`** vs. **5-step browser UI**.
2. Measure task completion time, unrecoverable errors, System Usability Scale (SUS), and technician reproduction accuracy from the exported PDF + `.oprecipe.json` bundle.

### Pass / Fail Acceptance Criteria
* **Efficiency:** `≥ 40%` reduction in median task time vs. `ufit-blender`.
* **Usability & Handoff:** Mean SUS `≥ 75`; `0` corrupted exports; `100%` technician parameter comprehension without clarifying calls.
