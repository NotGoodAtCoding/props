[Home (README)](../README.md) | [Getting Started](./getting_started.md) | [CLI & Tools Reference](./cli_and_tools_reference.md) | **`.oprecipe.json` Schema** | [Action Plan](../o_p_technology_action_plan.md) | [Research](../research/practitioner_software_needs.md) | [Experiments](../experiments/high_value_experiments.md)

---

# `.oprecipe.json` v0.1 Schema & Authoring Guide

> **Who this page is for:** Clinicians, technicians, and UI integrators creating or inspecting portable rectification recipes.  
> **Ready-to-run examples:** [`examples/tt_socket.oprecipe.json`](../examples/tt_socket.oprecipe.json), [`examples/tt_socket_v2.oprecipe.json`](../examples/tt_socket_v2.oprecipe.json), [`examples/tf_socket.oprecipe.json`](../examples/tf_socket.oprecipe.json), and [`examples/afo_leaf.oprecipe.json`](../examples/afo_leaf.oprecipe.json).

---

## 1. Canonical Coordinate System

All `.oprecipe.json` v0.1 coordinates assume the scan has passed through [`align_canonical_limb()`](../src/op_core/scan_qa.py):
* **Units:** Millimeters (`"units": "mm"` required).
* **`+Z` axis:** Proximal (up toward knee/hip).
* **`z = 0.0` (TT sockets):** Mid-Patellar Tendon (MPT) reference plane; distal limb extends into `-Z`.
* **`+Y` axis:** Anterior (toward patella / toes).
* **`+X` axis:** Lateral (right-handed frame with `+Y`, `+Z`).

---

## 2. Top-Level Fields

| Field | Type | Required | Description |
| :--- | :--- | :--- | :--- |
| `schema_version` | `string` | Yes | `"0.1"` |
| `kernel_version` | `string` | Yes | Pinned `op-core` kernel version (e.g., `"0.1.0"`) for deterministic replay. |
| `units` | `string` | Yes | Must be `"mm"`. |
| `side` | `string` | Yes | `"left"` or `"right"`. |
| `device_type` | `string` | Yes | `"transtibial_socket"`, `"transfemoral_socket"`, or `"afo"`. |
| `scan_sha256` | `string` | No | SHA-256 checksum of canonical input scan (auto-populated if blank during `build`). |
| `scale_factor` | `float` | No | Uniform scale multiplier (`1.0` default; must be `> 0`). |
| `liner_thickness_mm` | `float` | No | Radial liner thickness (`0.0–12.0 mm`) subtracted inward along `N_0` before reliefs. |
| `alignment` | `object` | No | `flexion_deg`, `adduction_deg`, and `mpt_offset_mm` (`[dx, dy]`) for TT/TF bench alignment. |
| `landmarks` | `object` | No | Map of landmark name $\rightarrow$ `{"vertex_idx": int}`, `{"face_idx": int, "bary": [u,v,w]}`, or `{"xyz": [x,y,z]}`. |
| `operations` | `array` | Yes | Ordered list of parametric operations (see Section 3). |
| `shell` | `object` | No | `wall_distal_mm`, `wall_proximal_mm`, `trimline_fillet_mm`, and `hardware`. |

---

## 3. Supported `operations`

### `global_compression` / `zoned_compression`
Applies slice-wise XY radial percentage scaling around the limb's anatomical centerline with `C^2` quintic transitions so Z-heights remain undistorted.

```json
{
  "id": "distal_zone",
  "op": "zoned_compression",
  "z_range_mm": [-200.0, -100.0],
  "percent": -2.5,
  "transition_mm": 20.0
}
```
* Negative `percent` compresses (reduces circumference); positive expands.
* `transition_mm` feathers smoothly into adjacent zones (prevents steps and isolates unedited regions).

### `landmark_relief`
Adds a smooth Gaussian geodesic build-up (`offset_mm > 0`) or indentation (`offset_mm < 0`) centered at a named entry in `landmarks`, evaluated in the undeformed reference domain `M_0`.

```json
{
  "id": "fibular_relief",
  "op": "landmark_relief",
  "landmark": "fibular_head",
  "offset_mm": 4.0,
  "radius_mm": 24.0
}
```

### `afo_posture` *(AFO only)*
Smoothly neutralizes plantarflexed/inverted foot-ankle scans around the oblique talocrural axis and flattens the plantar sole at footwear heel-height pitch before shelling.

```json
{
  "id": "neutralize_ankle",
  "op": "afo_posture",
  "dorsiflexion_deg": 12.0,
  "eversion_deg": 5.0,
  "heel_pitch_mm": 10.0
}
```

---

## 4. `shell.hardware` Options

| Value | Device | Geometry Generated |
| :--- | :--- | :--- |
| `"pyramid_50_8_m6_captive"` | TT / TF Sockets | Bench-aligned `50.8 mm` (2-inch) BCD 4-hole flange with recessed hexagonal captive M6 nut pockets. |
| `"afo_coaxial_joint"` | AFO | Thickened medial/lateral ankle pads aligned orthogonally to line of progression (no pyramid). |
| `"none"` | Any | Variable-thickness shell without hardware bosses. |

---

**Next:** [Getting Started](./getting_started.md) · [CLI & Tools Reference](./cli_and_tools_reference.md)
