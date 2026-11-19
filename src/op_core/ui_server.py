"""Local zero-dependency 3D WebGL browser UI and REST server for op-core (Phase 4)."""

from __future__ import annotations

import base64
import json
import re
import tempfile
import threading
import uuid
import webbrowser
from dataclasses import asdict
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

from .carver_cam import generate_4axis_gcode_and_simulate
from .export_3mf import export_3mf
from .linter import export_fabrication_dossier, lint_device
from .math3d import v_dot, v_sub
from .mesh import TriMesh, read_stl, write_binary_stl
from .recipe import OpRecipe, diff_recipes, mesh_sha256
from .rectify import apply_recipe
from .scan_qa import ScanQAReport, measure_circumferences, run_scan_qa
from .shell import build_afo_shell, build_tf_socket_shell, build_tt_socket_shell, export_carver_positive
from .synthetic import generate_foot_ankle_mesh, generate_transtibial_limb

_SAFE_NAME_RE = re.compile(r"^[A-Za-z0-9_.-]+$")


def _mesh_payload(mesh: TriMesh, deltas: list[float] | None = None) -> dict[str, Any]:
    """Serialize a TriMesh into flat vertex/normal/triangle arrays for WebGL."""
    norms = mesh.vertex_normals()
    bmin, bmax = mesh.bounds()
    center = [
        0.5 * (bmin[0] + bmax[0]),
        0.5 * (bmin[1] + bmax[1]),
        0.5 * (bmin[2] + bmax[2]),
    ]
    payload: dict[str, Any] = {
        "vertices": [round(c, 3) for v in mesh.vertices for c in v],
        "normals": [round(c, 4) for n in norms for c in n],
        "indices": [idx for f in mesh.faces for idx in f],
        "vertex_count": len(mesh.vertices),
        "face_count": len(mesh.faces),
        "center": [round(c, 3) for c in center],
        "bounds": {"min": list(bmin), "max": list(bmax)},
    }
    if deltas is not None:
        payload["deltas"] = [round(d, 3) for d in deltas]
    return payload


def _default_recipe_for_device(device: str, landmarks_xyz: dict[str, tuple[float, float, float]]) -> OpRecipe:
    if device == "afo":
        return OpRecipe(
            device_type="afo",
            side="left",
            landmarks={k: {"xyz": list(v)} for k, v in landmarks_xyz.items() if "malleolus" in k},
            operations=[
                {"op": "afo_posture", "id": "neutralize", "dorsiflexion_deg": 12.0, "eversion_deg": 5.0, "heel_pitch_mm": 10.0},
                {"op": "landmark_relief", "id": "med_mal", "landmark": "medial_malleolus", "offset_mm": 4.0, "radius_mm": 22.0},
                {"op": "landmark_relief", "id": "lat_mal", "landmark": "lateral_malleolus", "offset_mm": 4.0, "radius_mm": 22.0},
            ],
            shell={"wall_distal_mm": 4.5, "wall_proximal_mm": 2.5, "trimline_fillet_mm": 10.0, "hardware": "afo_coaxial_joint"},
        )
    if device == "tf":
        return OpRecipe(
            device_type="transfemoral_socket",
            side="left",
            liner_thickness_mm=3.0,
            alignment={"flexion_deg": 5.0, "adduction_deg": 8.0, "mpt_offset_mm": [0.0, 0.0]},
            landmarks={
                "ischial_shelf": {"xyz": list(landmarks_xyz.get("popliteal", (0.0, -58.0, 10.0)))},
                "scarpas_triangle": {"xyz": list(landmarks_xyz.get("mpt", (0.0, 56.0, -10.0)))},
                "trochanter": {"xyz": list(landmarks_xyz.get("fibular_head", (58.0, 0.0, -15.0)))},
            },
            operations=[
                {"op": "global_compression", "id": "glob", "z_range_mm": [-260.0, 60.0], "percent": -4.0, "transition_mm": 0.0},
                {"op": "landmark_relief", "id": "scarpa", "landmark": "scarpas_triangle", "offset_mm": -5.0, "radius_mm": 28.0},
                {"op": "landmark_relief", "id": "troch", "landmark": "trochanter", "offset_mm": 4.0, "radius_mm": 26.0},
            ],
            shell={"wall_distal_mm": 5.5, "wall_proximal_mm": 3.0, "trimline_fillet_mm": 12.0, "hardware": "pyramid_50_8_m6_captive"},
        )
    return OpRecipe(
        device_type="transtibial_socket",
        side="left",
        liner_thickness_mm=3.0,
        landmarks={
            "pt_bar": {"xyz": list(landmarks_xyz.get("mpt", (0.0, 54.0, 0.0)))},
            "fibular_head": {"xyz": list(landmarks_xyz.get("fibular_head", (52.0, 0.0, -25.0)))},
            "tibial_crest": {"xyz": list(landmarks_xyz.get("tibial_crest", (0.0, 46.0, -75.0)))},
            "popliteal": {"xyz": list(landmarks_xyz.get("popliteal", (0.0, -54.0, -5.0)))},
        },
        operations=[
            {"op": "global_compression", "id": "glob", "z_range_mm": [-220.0, 60.0], "percent": -3.0, "transition_mm": 0.0},
            {"op": "zoned_compression", "id": "distal_zone", "z_range_mm": [-220.0, -100.0], "percent": 0.0, "transition_mm": 20.0},
            {"op": "landmark_relief", "id": "pt", "landmark": "pt_bar", "offset_mm": -6.0, "radius_mm": 20.0},
            {"op": "landmark_relief", "id": "fib", "landmark": "fibular_head", "offset_mm": 4.0, "radius_mm": 24.0},
            {"op": "landmark_relief", "id": "crest", "landmark": "tibial_crest", "offset_mm": 3.0, "radius_mm": 22.0},
            {"op": "landmark_relief", "id": "pop", "landmark": "popliteal", "offset_mm": -4.0, "radius_mm": 24.0},
        ],
    )


class _SessionRecord:
    def __init__(self, canonical_mesh: TriMesh, qa_report: ScanQAReport, recipe: OpRecipe) -> None:
        self.canonical_mesh = canonical_mesh
        self.qa_report = qa_report
        self.baseline_recipe = OpRecipe.from_dict(recipe.to_dict())
        self.recipe = recipe
        self.tmp_dir = tempfile.TemporaryDirectory(prefix="op_ui_")

    @property
    def out_path(self) -> Path:
        return Path(self.tmp_dir.name)

    def close(self) -> None:
        self.tmp_dir.cleanup()


class SessionStore:
    """Thread-safe session cache keeping canonical meshes and exported artifacts per browser tab."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._sessions: dict[str, _SessionRecord] = {}

    def create(self, canonical_mesh: TriMesh, qa_report: ScanQAReport, recipe: OpRecipe) -> str:
        sid = uuid.uuid4().hex[:16]
        with self._lock:
            self._sessions[sid] = _SessionRecord(canonical_mesh, qa_report, recipe)
        return sid

    def get(self, sid: str) -> _SessionRecord | None:
        with self._lock:
            return self._sessions.get(sid)

    def close_all(self) -> None:
        with self._lock:
            for rec in self._sessions.values():
                rec.close()
            self._sessions.clear()


_HTML_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>op-core 3D Clinical Workstation</title>
<style>
  :root {
    --bg: #0f172a;
    --panel: #1e293b;
    --panel-alt: #263349;
    --border: #334155;
    --text: #e2e8f0;
    --muted: #94a3b8;
    --accent: #38bdf8;
    --ok: #22c55e;
    --warn: #f59e0b;
    --bad: #ef4444;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; font-family: system-ui, -apple-system, Segoe UI, Roboto, sans-serif;
    background: var(--bg); color: var(--text); display: flex; flex-direction: column; height: 100vh; overflow: hidden;
  }
  header {
    display: flex; align-items: center; justify-content: space-between;
    padding: 10px 16px; background: var(--panel); border-bottom: 1px solid var(--border);
  }
  header h1 { margin: 0; font-size: 16px; font-weight: 600; letter-spacing: 0.2px; }
  header .sub { font-size: 12px; color: var(--muted); }
  .steps { display: flex; gap: 6px; }
  .step-btn {
    background: var(--panel-alt); color: var(--muted); border: 1px solid var(--border);
    padding: 6px 10px; border-radius: 6px; font-size: 12px; cursor: pointer; font-weight: 500;
  }
  .step-btn.active { background: #0284c7; color: #fff; border-color: var(--accent); }
  main { display: flex; flex: 1; min-height: 0; }
  aside {
    width: 390px; background: var(--panel); border-right: 1px solid var(--border);
    padding: 14px; overflow-y: auto; display: flex; flex-direction: column; gap: 12px;
  }
  .viewport-wrap { flex: 1; position: relative; background: radial-gradient(circle at 50% 40%, #1e293b 0%, #0b1120 100%); }
  canvas { width: 100%; height: 100%; display: block; cursor: grab; }
  canvas:active { cursor: grabbing; }
  .hud-top {
    position: absolute; top: 12px; left: 12px; right: 12px; pointer-events: none;
    display: flex; justify-content: space-between; align-items: flex-start; gap: 8px;
  }
  .hud-card {
    pointer-events: auto; background: rgba(15, 23, 42, 0.86); border: 1px solid var(--border);
    border-radius: 8px; padding: 8px 12px; font-size: 12px; backdrop-filter: blur(4px);
  }
  .hud-bottom {
    position: absolute; bottom: 12px; left: 12px; background: rgba(15, 23, 42, 0.82);
    border: 1px solid var(--border); border-radius: 6px; padding: 6px 10px; font-size: 11px; color: var(--muted);
  }
  .card {
    background: var(--panel-alt); border: 1px solid var(--border); border-radius: 8px; padding: 12px;
  }
  .card h2 { margin: 0 0 8px 0; font-size: 13px; text-transform: uppercase; letter-spacing: 0.5px; color: var(--accent); }
  .row { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
  .btn {
    background: #0284c7; color: #fff; border: none; border-radius: 6px; padding: 7px 12px;
    font-size: 12px; font-weight: 600; cursor: pointer;
  }
  .btn.secondary { background: #334155; color: var(--text); }
  .btn.ok { background: #16a34a; }
  .btn:disabled { opacity: 0.5; cursor: not-allowed; }
  .badge { display: inline-block; padding: 2px 8px; border-radius: 999px; font-size: 11px; font-weight: 700; }
  .badge.PASS { background: rgba(34,197,94,0.2); color: var(--ok); border: 1px solid var(--ok); }
  .badge.WARN { background: rgba(245,158,11,0.2); color: var(--warn); border: 1px solid var(--warn); }
  .badge.FAIL { background: rgba(239,68,68,0.2); color: var(--bad); border: 1px solid var(--bad); }
  label.field { display: flex; flex-direction: column; gap: 4px; font-size: 12px; margin-bottom: 8px; }
  label.field .lbl { display: flex; justify-content: space-between; color: var(--text); }
  label.field .val { color: var(--accent); font-family: monospace; }
  input[type="range"] { width: 100%; accent-color: var(--accent); }
  select, input[type="file"] {
    background: var(--bg); color: var(--text); border: 1px solid var(--border);
    border-radius: 6px; padding: 6px 8px; font-size: 12px; width: 100%;
  }
  table { width: 100%; border-collapse: collapse; font-size: 11px; margin-top: 6px; }
  th, td { text-align: right; padding: 4px 6px; border-bottom: 1px solid var(--border); }
  th:first-child, td:first-child { text-align: left; }
  .lm-item {
    display: flex; justify-content: space-between; align-items: center;
    padding: 6px 8px; border-radius: 6px; background: var(--bg); margin-bottom: 6px; font-size: 12px;
  }
  .lm-item.armed { outline: 2px solid var(--accent); }
  .dl-link {
    display: block; padding: 8px 10px; background: var(--bg); border: 1px solid var(--border);
    border-radius: 6px; color: var(--accent); text-decoration: none; font-size: 12px; margin-bottom: 6px;
  }
  .dl-link:hover { border-color: var(--accent); }
  .hidden { display: none !important; }
  .mono { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; font-size: 11px; color: var(--muted); }
</style>
</head>
<body>
<header>
  <div>
    <h1>op-core 3D Clinical Workstation</h1>
    <div class="sub">Zero-dependency local browser UI · .oprecipe.json v0.1</div>
  </div>
  <div class="steps">
    <button class="step-btn active" data-step="1">1. Scan QA</button>
    <button class="step-btn" data-step="2">2. Landmarks</button>
    <button class="step-btn" data-step="3">3. Rectify</button>
    <button class="step-btn" data-step="4">4. Shell & Lint</button>
    <button class="step-btn" data-step="5">5. Export</button>
  </div>
</header>
<main>
  <aside>
    <div id="step-1" class="step-pane">
      <div class="card">
        <h2>Load Scan (.stl) or Sample</h2>
        <div class="row" style="margin-bottom:8px;">
          <button class="btn" onclick="loadSample('tt')">Sample TT</button>
          <button class="btn secondary" onclick="loadSample('tf')">Sample TF</button>
          <button class="btn secondary" onclick="loadSample('afo')">Sample AFO</button>
        </div>
        <label class="field">
          <span class="lbl">Upload patient STL (mm)</span>
          <input type="file" id="stlFile" accept=".stl" onchange="uploadSTL(this.files[0])" />
        </label>
      </div>
      <div class="card">
        <h2>Chairside Scan QA</h2>
        <div id="qaSummary" class="mono">Loading sample scan...</div>
        <div id="sliceTableWrap"></div>
      </div>
    </div>

    <div id="step-2" class="step-pane hidden">
      <div class="card">
        <h2>Anatomical Landmarks</h2>
        <p class="mono" style="margin-top:0;">Click "Pick on 3D Mesh", then click directly on the limb surface in the 3D viewport.</p>
        <div id="landmarkList"></div>
      </div>
    </div>

    <div id="step-3" class="step-pane hidden">
      <div class="card">
        <h2>Non-Destructive Rectification (M_0)</h2>
        <div id="rectifyControls"></div>
        <div class="row" style="margin-top:8px;">
          <button class="btn" onclick="runRectify()">Apply & Preview Heatmap</button>
        </div>
      </div>
      <div class="card">
        <h2>Revision Diff vs. Intake Baseline</h2>
        <pre id="diffWrap" class="mono" style="margin:0; white-space:pre-wrap;">No parameter edits yet.</pre>
      </div>
    </div>

    <div id="step-4" class="step-pane hidden">
      <div class="card">
        <h2>Shell, Alignment & Safety Lint</h2>
        <div id="shellControls"></div>
        <div class="row" style="margin-top:8px;">
          <button class="btn ok" onclick="runBuild()">Compile Shell, .3MF & Carver</button>
        </div>
        <div id="lintSummary" style="margin-top:10px;" class="mono">Click Compile to generate watertight shell and run safety checks.</div>
      </div>
    </div>

    <div id="step-5" class="step-pane hidden">
      <div class="card">
        <h2>Dual-CAM & Dossier Downloads</h2>
        <p class="mono" style="margin-top:0;">Generated artifacts from your deterministic .oprecipe.json build:</p>
        <div id="downloadList" class="mono">Run Step 4 ("Compile Shell, .3MF & Carver") first.</div>
      </div>
    </div>
  </aside>

  <section class="viewport-wrap">
    <canvas id="glCanvas"></canvas>
    <div class="hud-top">
      <div class="hud-card" id="meshHud">No mesh loaded</div>
      <div class="hud-card row">
        <label style="font-size:12px; display:flex; align-items:center; gap:4px;">
          <input type="checkbox" id="chkWire" onchange="drawScene()" /> Wireframe
        </label>
        <label style="font-size:12px; display:flex; align-items:center; gap:4px;">
          <input type="checkbox" id="chkLandmarks" checked onchange="drawScene()" /> Landmarks
        </label>
        <label style="font-size:12px; display:flex; align-items:center; gap:4px;">
          <input type="checkbox" id="chkSlices" checked onchange="drawScene()" /> 20mm Slices
        </label>
        <label style="font-size:12px; display:flex; align-items:center; gap:4px;">
          <input type="checkbox" id="chkShell" checked onchange="toggleViewMesh()" /> Show Shell (if built)
        </label>
        <button class="btn secondary" onclick="resetCamera()">Reset View</button>
      </div>
    </div>
    <div class="hud-bottom">
      Orbit: Left-drag · Zoom: Scroll · Pan: Shift+Drag · Pick Landmark (Step 2): Click surface
    </div>
  </section>
</main>

<script>
const state = {
  sessionId: null,
  recipe: null,
  qa: null,
  baseMesh: null,
  rectMesh: null,
  shellMesh: null,
  activeMeshKey: 'base',
  armedLandmark: null,
  lint: null,
  downloads: null,
  cam: { yaw: 0.55, pitch: 0.3, dist: 380, target: [0, 0, -60] }
};

document.querySelectorAll('.step-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    const step = btn.dataset.step;
    document.querySelectorAll('.step-btn').forEach(b => b.classList.toggle('active', b === btn));
    document.querySelectorAll('.step-pane').forEach(p => p.classList.toggle('hidden', p.id !== 'step-' + step));
    if (step !== '2') { state.armedLandmark = null; renderLandmarks(); }
  });
});

async function apiJson(url, payload) {
  const res = await fetch(url, {
    method: payload ? 'POST' : 'GET',
    headers: payload ? { 'Content-Type': 'application/json' } : {},
    body: payload ? JSON.stringify(payload) : undefined
  });
  const data = await res.json();
  if (!res.ok) throw new Error(data.error || 'Request failed');
  return data;
}

async function loadSample(device) {
  const data = await apiJson('/api/sample?device=' + encodeURIComponent(device));
  hydrateSession(data);
}

async function uploadSTL(file) {
  if (!file) return;
  const buf = await file.arrayBuffer();
  const bytes = new Uint8Array(buf);
  let bin = '';
  for (let i = 0; i < bytes.length; i++) bin += String.fromCharCode(bytes[i]);
  const data = await apiJson('/api/upload', { filename: file.name, stl_base64: btoa(bin) });
  hydrateSession(data);
}

function hydrateSession(data) {
  state.sessionId = data.session_id;
  state.qa = data.qa;
  state.recipe = data.recipe;
  state.baseMesh = data.mesh;
  state.rectMesh = null;
  state.shellMesh = null;
  state.activeMeshKey = 'base';
  state.lint = null;
  state.downloads = null;
  resetCamera();
  renderQA();
  renderLandmarks();
  renderRectifyControls();
  renderShellControls();
  renderDownloads();
  uploadMeshToGL();
}

function resetCamera() {
  const m = getDisplayMesh();
  if (!m) return;
  state.cam.target = [...m.center];
  const minB = m.bounds.min, maxB = m.bounds.max;
  const span = Math.hypot(maxB[0]-minB[0], maxB[1]-minB[1], maxB[2]-minB[2]);
  state.cam.dist = Math.max(180, span * 1.65);
  state.cam.yaw = 0.65;
  state.cam.pitch = 0.25;
  drawScene();
}

function getDisplayMesh() {
  if (state.activeMeshKey === 'shell' && state.shellMesh) return state.shellMesh;
  if (state.rectMesh) return state.rectMesh;
  return state.baseMesh;
}

function toggleViewMesh() {
  const showShell = document.getElementById('chkShell').checked;
  state.activeMeshKey = (showShell && state.shellMesh) ? 'shell' : (state.rectMesh ? 'rect' : 'base');
  uploadMeshToGL();
}

function renderQA() {
  const qa = state.qa;
  if (!qa) return;
  const warnHtml = qa.warnings.length ? `<div>Warnings: ${qa.warnings.join('; ')}</div>` : '';
  const errHtml = qa.errors.length ? `<div style="color:var(--bad)">Errors: ${qa.errors.join('; ')}</div>` : '';
  document.getElementById('qaSummary').innerHTML = `
    <div style="margin-bottom:6px;">Status: <span class="badge ${qa.status}">${qa.status}</span></div>
    <div>Floaters removed: ${qa.floater_components_removed} · Cuff loops: ${qa.proximal_cuff_loops} · Tears: ${qa.internal_tear_loops}</div>
    <div>Double-wall verts: ${qa.double_wall_vertices} · Seam score: ${qa.seam_step_score_mm.toFixed(2)} mm</div>
    ${warnHtml}${errHtml}
  `;
  const rows = (qa.slices || []).map(s =>
    `<tr><td>-${s.depth_from_anchor_mm.toFixed(0)} mm</td><td>${s.taut_circumference_mm.toFixed(1)} mm</td><td>${s.hull_area_mm2.toFixed(0)} mm²</td></tr>`
  ).join('');
  document.getElementById('sliceTableWrap').innerHTML = rows ? `
    <table><thead><tr><th>Depth from MPT</th><th>Taut Tape</th><th>Area</th></tr></thead><tbody>${rows}</tbody></table>
  ` : '';
}

function renderLandmarks() {
  const wrap = document.getElementById('landmarkList');
  const lms = (state.recipe && state.recipe.landmarks) || {};
  wrap.innerHTML = Object.entries(lms).map(([name, spec]) => {
    const xyz = (spec.xyz || [0,0,0]).map(v => Number(v).toFixed(1)).join(', ');
    const armed = state.armedLandmark === name;
    return `<div class="lm-item ${armed ? 'armed' : ''}">
      <div><strong>${name}</strong><div class="mono">[${xyz}]</div></div>
      <button class="btn ${armed ? 'ok' : 'secondary'}" onclick="armLandmark('${name}')">${armed ? 'Click Mesh...' : 'Pick on 3D Mesh'}</button>
    </div>`;
  }).join('');
}

function armLandmark(name) {
  state.armedLandmark = (state.armedLandmark === name) ? null : name;
  renderLandmarks();
  drawScene();
}

function sliderField(id, label, min, max, step, val, unit, onChange) {
  return `<label class="field">
    <span class="lbl"><span>${label}</span><span class="val" id="val_${id}">${Number(val).toFixed(1)} ${unit}</span></span>
    <input type="range" id="inp_${id}" min="${min}" max="${max}" step="${step}" value="${val}"
      oninput="document.getElementById('val_${id}').textContent = Number(this.value).toFixed(1) + ' ${unit}'; (${onChange})(Number(this.value));" />
  </label>`;
}

function renderRectifyControls() {
  const r = state.recipe;
  if (!r) return;
  let html = '';
  if (r.device_type !== 'afo') {
    html += sliderField('liner', 'Liner Thickness Subtract', 0, 8, 0.5, r.liner_thickness_mm, 'mm',
      'v => { state.recipe.liner_thickness_mm = v; }');
  }
  r.operations.forEach((op, idx) => {
    if (op.op === 'global_compression' || op.op === 'zoned_compression') {
      html += sliderField('op_' + idx, op.id + ' (%)', -10, 5, 0.5, op.percent, '%',
        `v => { state.recipe.operations[${idx}].percent = v; }`);
    } else if (op.op === 'landmark_relief') {
      html += sliderField('op_' + idx, op.landmark + ' relief', -10, 10, 0.5, op.offset_mm, 'mm',
        `v => { state.recipe.operations[${idx}].offset_mm = v; }`);
    } else if (op.op === 'afo_posture') {
      html += sliderField('op_df_' + idx, 'Dorsiflexion correction', -15, 25, 1, op.dorsiflexion_deg, '°',
        `v => { state.recipe.operations[${idx}].dorsiflexion_deg = v; }`);
      html += sliderField('op_ev_' + idx, 'Eversion correction', -15, 15, 1, op.eversion_deg, '°',
        `v => { state.recipe.operations[${idx}].eversion_deg = v; }`);
      html += sliderField('op_hp_' + idx, 'Footwear heel pitch', 0, 25, 1, op.heel_pitch_mm, 'mm',
        `v => { state.recipe.operations[${idx}].heel_pitch_mm = v; }`);
    }
  });
  document.getElementById('rectifyControls').innerHTML = html;
}

function renderShellControls() {
  const r = state.recipe;
  if (!r) return;
  const sh = r.shell || {};
  const al = r.alignment || {};
  let html = sliderField('wd', 'Distal / Ankle Neck Wall', 2.5, 7.0, 0.5, sh.wall_distal_mm ?? 4.5, 'mm',
    'v => { state.recipe.shell.wall_distal_mm = v; }');
  html += sliderField('wp', 'Proximal Brim / Calf Wall', 2.0, 5.0, 0.5, sh.wall_proximal_mm ?? 2.5, 'mm',
    'v => { state.recipe.shell.wall_proximal_mm = v; }');
  if (r.device_type !== 'afo') {
    html += sliderField('flex', 'Bench Alignment Flexion', -5, 15, 0.5, al.flexion_deg ?? 5.0, '°',
      'v => { state.recipe.alignment.flexion_deg = v; }');
    html += sliderField('add', 'Bench Alignment Adduction', -5, 15, 0.5, al.adduction_deg ?? 5.0, '°',
      'v => { state.recipe.alignment.adduction_deg = v; }');
  } else {
    html += sliderField('fillet', 'Malleolar Trimline Fillet R', 6, 18, 1, sh.trimline_fillet_mm ?? 10.0, 'mm',
      'v => { state.recipe.shell.trimline_fillet_mm = v; }');
  }
  html += `<label class="field"><span class="lbl">Slicer .3MF Target</span>
    <select id="selTarget"><option value="orcaslicer">OrcaSlicer (model_settings.config)</option><option value="prusaslicer">PrusaSlicer (Slic3r_PE_model.config)</option></select>
  </label>`;
  document.getElementById('shellControls').innerHTML = html;
}

async function runRectify() {
  const data = await apiJson('/api/rectify', { session_id: state.sessionId, recipe: state.recipe });
  state.rectMesh = data.rectified;
  state.activeMeshKey = 'rect';
  await refreshDiff();
  uploadMeshToGL();
}

async function refreshDiff() {
  if (!state.sessionId || !state.recipe) return;
  try {
    const d = await apiJson('/api/diff', { session_id: state.sessionId, recipe: state.recipe });
    const el = document.getElementById('diffWrap');
    if (el) {
      el.textContent = Object.keys(d.diff || {}).length ? JSON.stringify(d.diff, null, 2) : 'No parameter edits vs baseline.';
    }
  } catch (_) {}
}

async function runBuild() {
  const target = document.getElementById('selTarget') ? document.getElementById('selTarget').value : 'orcaslicer';
  const data = await apiJson('/api/build', { session_id: state.sessionId, recipe: state.recipe, target });
  state.rectMesh = data.rectified;
  state.shellMesh = data.shell;
  state.lint = data.lint;
  state.downloads = data.downloads;
  document.getElementById('chkShell').checked = true;
  state.activeMeshKey = 'shell';
  const lt = data.lint;
  document.getElementById('lintSummary').innerHTML = `
    <div>Safety Lint: <span class="badge ${lt.passed ? 'PASS' : 'FAIL'}">${lt.passed ? 'PASS' : 'FAIL'}</span></div>
    <div>Watertight Manifold: ${lt.is_watertight_manifold} · Min Wall: ${lt.min_estimated_wall_mm.toFixed(2)} mm</div>
    <div>Trimline Min R: ${lt.trimline_min_radius_mm.toFixed(1)} mm · Print-Z Angle: ${lt.load_to_print_z_angle_deg.toFixed(1)}°</div>
  `;
  renderDownloads();
  uploadMeshToGL();
}

function renderDownloads() {
  const el = document.getElementById('downloadList');
  if (!state.downloads) {
    el.textContent = 'Run Step 4 ("Compile Shell, .3MF & Carver") first.';
    return;
  }
  el.innerHTML = Object.entries(state.downloads).map(([label, url]) =>
    `<a class="dl-link" href="${url}" download>${label}</a>`
  ).join('');
}

// --- WebGL1 Minimal Renderer + Moller-Trumbore Landmark Ray Picker ---
const canvas = document.getElementById('glCanvas');
const gl = canvas.getContext('webgl', { antialias: true });
const extUint = gl ? gl.getExtension('OES_element_index_uint') : null;
let prog = null, lineProg = null, vboPos = null, vboNorm = null, vboCol = null, ibo = null;
let vboOvPos = null, vboOvCol = null, indexCount = 0;
let indexType = gl ? gl.UNSIGNED_SHORT : 0;
let lastMVP = null;

function initGL() {
  if (!gl) return;
  const vs = `
    attribute vec3 aPos;
    attribute vec3 aNorm;
    attribute vec3 aCol;
    uniform mat4 uMVP;
    varying vec3 vNorm;
    varying vec3 vCol;
    void main() {
      vNorm = aNorm;
      vCol = aCol;
      gl_Position = uMVP * vec4(aPos, 1.0);
    }`;
  const fs = `
    precision mediump float;
    varying vec3 vNorm;
    varying vec3 vCol;
    uniform vec3 uLightDir;
    uniform float uWire;
    void main() {
      if (uWire > 0.5) { gl_FragColor = vec4(0.15, 0.7, 0.95, 0.6); return; }
      vec3 n = normalize(vNorm);
      float diff = max(abs(dot(n, uLightDir)), 0.15);
      vec3 lit = vCol * (0.28 + 0.72 * diff);
      gl_FragColor = vec4(lit, 1.0);
    }`;
  const lvs = `
    attribute vec3 aPos;
    attribute vec3 aCol;
    uniform mat4 uMVP;
    varying vec3 vCol;
    void main() { vCol = aCol; gl_Position = uMVP * vec4(aPos, 1.0); }`;
  const lfs = `
    precision mediump float;
    varying vec3 vCol;
    void main() { gl_FragColor = vec4(vCol, 1.0); }`;
  function compile(type, src) {
    const s = gl.createShader(type);
    gl.shaderSource(s, src);
    gl.compileShader(s);
    return s;
  }
  prog = gl.createProgram();
  gl.attachShader(prog, compile(gl.VERTEX_SHADER, vs));
  gl.attachShader(prog, compile(gl.FRAGMENT_SHADER, fs));
  gl.linkProgram(prog);
  lineProg = gl.createProgram();
  gl.attachShader(lineProg, compile(gl.VERTEX_SHADER, lvs));
  gl.attachShader(lineProg, compile(gl.FRAGMENT_SHADER, lfs));
  gl.linkProgram(lineProg);
  vboPos = gl.createBuffer();
  vboNorm = gl.createBuffer();
  vboCol = gl.createBuffer();
  ibo = gl.createBuffer();
  vboOvPos = gl.createBuffer();
  vboOvCol = gl.createBuffer();
}

function deltaToColor(d) {
  if (d === undefined || d === null) return [0.78, 0.82, 0.87];
  if (Math.abs(d) < 0.15) return [0.78, 0.82, 0.87];
  const t = Math.min(1.0, Math.abs(d) / 5.0);
  if (d > 0) return [0.78 + 0.20 * t, 0.82 - 0.35 * t, 0.87 - 0.55 * t]; // build-up: warm amber
  return [0.78 - 0.48 * t, 0.82 - 0.15 * t, 0.87 + 0.10 * t];            // compression: cool blue
}

function uploadMeshToGL() {
  const m = getDisplayMesh();
  if (!m) return;
  document.getElementById('meshHud').textContent =
    `${state.activeMeshKey.toUpperCase()} · ${m.vertex_count} verts · ${m.face_count} tris`;
  if (!gl) return;
  const nVerts = m.vertex_count;
  const cols = new Float32Array(nVerts * 3);
  for (let i = 0; i < nVerts; i++) {
    const c = deltaToColor(m.deltas ? m.deltas[i] : null);
    cols[i*3] = c[0]; cols[i*3+1] = c[1]; cols[i*3+2] = c[2];
  }
  gl.bindBuffer(gl.ARRAY_BUFFER, vboPos);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(m.vertices), gl.STATIC_DRAW);
  gl.bindBuffer(gl.ARRAY_BUFFER, vboNorm);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(m.normals), gl.STATIC_DRAW);
  gl.bindBuffer(gl.ARRAY_BUFFER, vboCol);
  gl.bufferData(gl.ARRAY_BUFFER, cols, gl.STATIC_DRAW);
  gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, ibo);
  if (extUint && nVerts > 65535) {
    gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, new Uint32Array(m.indices), gl.STATIC_DRAW);
    indexType = gl.UNSIGNED_INT;
  } else {
    gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, new Uint16Array(m.indices), gl.STATIC_DRAW);
    indexType = gl.UNSIGNED_SHORT;
  }
  indexCount = m.indices.length;
  drawScene();
}

function computeMVP(aspect) {
  const { yaw, pitch, dist, target } = state.cam;
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const cy = Math.cos(yaw), sy = Math.sin(yaw);
  const eye = [
    target[0] + dist * cp * cy,
    target[1] + dist * cp * sy,
    target[2] + dist * sp
  ];
  function sub(a, b) { return [a[0]-b[0], a[1]-b[1], a[2]-b[2]]; }
  function cross(a, b) { return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]; }
  function dot(a, b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
  function norm(a) { const l = Math.hypot(...a) || 1; return [a[0]/l, a[1]/l, a[2]/l]; }

  const f = norm(sub(target, eye));
  const r = norm(cross(f, [0, 0, 1]));
  const u = cross(r, f);

  const view = [
    r[0], u[0], -f[0], 0,
    r[1], u[1], -f[1], 0,
    r[2], u[2], -f[2], 0,
    -dot(r, eye), -dot(u, eye), dot(f, eye), 1
  ];
  const fov = 45 * Math.PI / 180;
  const near = 1.0, far = 3000.0;
  const t = 1 / Math.tan(fov / 2);
  const proj = [
    t / aspect, 0, 0, 0,
    0, t, 0, 0,
    0, 0, (far + near) / (near - far), -1,
    0, 0, (2 * far * near) / (near - far), 0
  ];
  const mvp = new Float32Array(16);
  for (let c = 0; c < 4; c++) {
    for (let row = 0; row < 4; row++) {
      mvp[c*4 + row] =
        proj[0*4+row]*view[c*4+0] + proj[1*4+row]*view[c*4+1] +
        proj[2*4+row]*view[c*4+2] + proj[3*4+row]*view[c*4+3];
    }
  }
  return { mvp, eye, f, r, u, fov };
}

function drawScene() {
  if (!gl || !prog || indexCount === 0) return;
  const dpr = window.devicePixelRatio || 1;
  const w = Math.floor(canvas.clientWidth * dpr);
  const h = Math.floor(canvas.clientHeight * dpr);
  if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
  gl.viewport(0, 0, w, h);
  gl.enable(gl.DEPTH_TEST);
  gl.clearColor(0.05, 0.08, 0.15, 1.0);
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
  gl.useProgram(prog);

  const camInfo = computeMVP(w / Math.max(1, h));
  lastMVP = camInfo;
  gl.uniformMatrix4fv(gl.getUniformLocation(prog, 'uMVP'), false, camInfo.mvp);
  gl.uniform3f(gl.getUniformLocation(prog, 'uLightDir'), 0.45, 0.65, 0.60);
  gl.uniform1f(gl.getUniformLocation(prog, 'uWire'), 0.0);

  const locP = gl.getAttribLocation(prog, 'aPos');
  gl.bindBuffer(gl.ARRAY_BUFFER, vboPos);
  gl.enableVertexAttribArray(locP);
  gl.vertexAttribPointer(locP, 3, gl.FLOAT, false, 0, 0);

  const locN = gl.getAttribLocation(prog, 'aNorm');
  gl.bindBuffer(gl.ARRAY_BUFFER, vboNorm);
  gl.enableVertexAttribArray(locN);
  gl.vertexAttribPointer(locN, 3, gl.FLOAT, false, 0, 0);

  const locC = gl.getAttribLocation(prog, 'aCol');
  gl.bindBuffer(gl.ARRAY_BUFFER, vboCol);
  gl.enableVertexAttribArray(locC);
  gl.vertexAttribPointer(locC, 3, gl.FLOAT, false, 0, 0);

  gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, ibo);
  gl.drawElements(gl.TRIANGLES, indexCount, indexType, 0);
  if (document.getElementById('chkWire').checked) {
    gl.uniform1f(gl.getUniformLocation(prog, 'uWire'), 1.0);
    gl.drawElements(gl.LINES, indexCount, indexType, 0);
  }
  drawOverlays(camInfo.mvp);
}

function drawOverlays(mvp) {
  if (!lineProg) return;
  const pts = [], cols = [];
  const m = getDisplayMesh();
  if (document.getElementById('chkSlices').checked && state.qa && state.qa.slices && m) {
    const cx = m.center[0], cy = m.center[1];
    const r = Math.max(Math.abs(m.bounds.max[0]-m.bounds.min[0]), Math.abs(m.bounds.max[1]-m.bounds.min[1])) * 0.58;
    const segs = 36;
    for (const s of state.qa.slices) {
      const z = s.z_mm;
      for (let k = 0; k < segs; k++) {
        const a0 = 2 * Math.PI * k / segs, a1 = 2 * Math.PI * (k + 1) / segs;
        pts.push(cx + r*Math.cos(a0), cy + r*Math.sin(a0), z, cx + r*Math.cos(a1), cy + r*Math.sin(a1), z);
        cols.push(0.22, 0.74, 0.97, 0.22, 0.74, 0.97);
      }
    }
  }
  if (document.getElementById('chkLandmarks').checked && state.recipe && state.recipe.landmarks) {
    for (const [name, spec] of Object.entries(state.recipe.landmarks)) {
      const p = spec.xyz;
      if (!p) continue;
      const armed = state.armedLandmark === name;
      const col = armed ? [0.13, 0.85, 0.45] : [0.98, 0.75, 0.15];
      const s = armed ? 9.0 : 6.0;
      const segs = [
        [p[0]-s, p[1], p[2], p[0]+s, p[1], p[2]],
        [p[0], p[1]-s, p[2], p[0], p[1]+s, p[2]],
        [p[0], p[1], p[2]-s, p[0], p[1], p[2]+s],
        [p[0]-s, p[1], p[2], p[0], p[1]+s, p[2]],
        [p[0], p[1]+s, p[2], p[0]+s, p[1], p[2]],
        [p[0]+s, p[1], p[2], p[0], p[1]-s, p[2]],
        [p[0], p[1]-s, p[2], p[0]-s, p[1], p[2]]
      ];
      for (const sg of segs) { pts.push(...sg); cols.push(...col, ...col); }
    }
  }
  if (!pts.length) return;
  gl.useProgram(lineProg);
  gl.uniformMatrix4fv(gl.getUniformLocation(lineProg, 'uMVP'), false, mvp);
  const lp = gl.getAttribLocation(lineProg, 'aPos');
  gl.bindBuffer(gl.ARRAY_BUFFER, vboOvPos);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(pts), gl.DYNAMIC_DRAW);
  gl.enableVertexAttribArray(lp);
  gl.vertexAttribPointer(lp, 3, gl.FLOAT, false, 0, 0);
  const lc = gl.getAttribLocation(lineProg, 'aCol');
  gl.bindBuffer(gl.ARRAY_BUFFER, vboOvCol);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(cols), gl.DYNAMIC_DRAW);
  gl.enableVertexAttribArray(lc);
  gl.vertexAttribPointer(lc, 3, gl.FLOAT, false, 0, 0);
  gl.drawArrays(gl.LINES, 0, pts.length / 3);
}

function pickSurfacePoint(clientX, clientY) {
  const m = getDisplayMesh();
  if (!m || !lastMVP) return null;
  const rect = canvas.getBoundingClientRect();
  const ndcX = ((clientX - rect.left) / rect.width) * 2 - 1;
  const ndcY = 1 - ((clientY - rect.top) / rect.height) * 2;
  const aspect = rect.width / Math.max(1, rect.height);
  const tanHalf = Math.tan(lastMVP.fov / 2);
  const { eye, f, r, u } = lastMVP;
  const dir = [
    f[0] + r[0]*ndcX*aspect*tanHalf + u[0]*ndcY*tanHalf,
    f[1] + r[1]*ndcX*aspect*tanHalf + u[1]*ndcY*tanHalf,
    f[2] + r[2]*ndcX*aspect*tanHalf + u[2]*ndcY*tanHalf
  ];
  const dl = Math.hypot(...dir) || 1;
  dir[0] /= dl; dir[1] /= dl; dir[2] /= dl;

  const verts = m.vertices, idxs = m.indices;
  let bestT = Infinity, bestPt = null;
  for (let i = 0; i < idxs.length; i += 3) {
    const a = idxs[i]*3, b = idxs[i+1]*3, c = idxs[i+2]*3;
    const e1 = [verts[b]-verts[a], verts[b+1]-verts[a+1], verts[b+2]-verts[a+2]];
    const e2 = [verts[c]-verts[a], verts[c+1]-verts[a+1], verts[c+2]-verts[a+2]];
    const pvec = [dir[1]*e2[2]-dir[2]*e2[1], dir[2]*e2[0]-dir[0]*e2[2], dir[0]*e2[1]-dir[1]*e2[0]];
    const det = e1[0]*pvec[0] + e1[1]*pvec[1] + e1[2]*pvec[2];
    if (Math.abs(det) < 1e-6) continue;
    const invDet = 1 / det;
    const tvec = [eye[0]-verts[a], eye[1]-verts[a+1], eye[2]-verts[a+2]];
    const u_b = (tvec[0]*pvec[0] + tvec[1]*pvec[1] + tvec[2]*pvec[2]) * invDet;
    if (u_b < 0 || u_b > 1) continue;
    const qvec = [tvec[1]*e1[2]-tvec[2]*e1[1], tvec[2]*e1[0]-tvec[0]*e1[2], tvec[0]*e1[1]-tvec[1]*e1[0]];
    const v_b = (dir[0]*qvec[0] + dir[1]*qvec[1] + dir[2]*qvec[2]) * invDet;
    if (v_b < 0 || u_b + v_b > 1) continue;
    const t = (e2[0]*qvec[0] + e2[1]*qvec[1] + e2[2]*qvec[2]) * invDet;
    if (t > 1 && t < bestT) {
      bestT = t;
      bestPt = [eye[0]+dir[0]*t, eye[1]+dir[1]*t, eye[2]+dir[2]*t];
    }
  }
  return bestPt;
}

let dragging = false, dragMoved = false, lastX = 0, lastY = 0;
canvas.addEventListener('mousedown', e => { dragging = true; dragMoved = false; lastX = e.clientX; lastY = e.clientY; });
window.addEventListener('mouseup', e => {
  if (dragging && !dragMoved && state.armedLandmark) {
    const pt = pickSurfacePoint(e.clientX, e.clientY);
    if (pt) {
      state.recipe.landmarks[state.armedLandmark] = { xyz: pt.map(v => Number(v.toFixed(2))) };
      state.armedLandmark = null;
      renderLandmarks();
      drawScene();
    }
  }
  dragging = false;
});
window.addEventListener('mousemove', e => {
  if (!dragging) return;
  const dx = e.clientX - lastX, dy = e.clientY - lastY;
  if (Math.hypot(dx, dy) > 2) dragMoved = true;
  lastX = e.clientX; lastY = e.clientY;
  if (e.shiftKey) {
    state.cam.target[2] += dy * 0.4;
  } else {
    state.cam.yaw -= dx * 0.01;
    state.cam.pitch = Math.max(-1.45, Math.min(1.45, state.cam.pitch + dy * 0.01));
  }
  drawScene();
});
canvas.addEventListener('wheel', e => {
  e.preventDefault();
  state.cam.dist = Math.max(60, Math.min(1200, state.cam.dist * (1 + e.deltaY * 0.001)));
  drawScene();
}, { passive: false });
window.addEventListener('resize', drawScene);

initGL();
loadSample('tt');
</script>
</body>
</html>
"""


def _make_handler(store: SessionStore) -> type[BaseHTTPRequestHandler]:
    class OpUIHandler(BaseHTTPRequestHandler):
        def log_message(self, format: str, *args: Any) -> None:
            return  # Suppress noisy stdout access logs during automated runs

        def _send_json(self, payload: dict[str, Any], status: int = HTTPStatus.OK) -> None:
            raw = json.dumps(payload).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def _read_json(self) -> dict[str, Any]:
            length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(length) if length > 0 else b"{}"
            return json.loads(raw.decode("utf-8"))

        def do_GET(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)
            if parsed.path == "/":
                body = _HTML_PAGE.encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if parsed.path == "/api/health":
                self._send_json({"status": "ok", "version": "0.1.0"})
                return

            if parsed.path == "/api/sample":
                qs = parse_qs(parsed.query)
                dev = qs.get("device", ["tt"])[0]
                if dev == "afo":
                    case = generate_foot_ankle_mesh(plantarflexion_deg=12.0, inversion_deg=5.0)
                    aligned = case.mesh
                    qa_rep = ScanQAReport(
                        status="PASS",
                        floater_components_removed=0,
                        proximal_cuff_loops=1,
                        internal_tear_loops=0,
                        double_wall_vertices=0,
                        seam_step_score_mm=0.0,
                        detected_scale_ratio=None,
                        slices=measure_circumferences(aligned, anchor_z_mm=220.0, step_mm=20.0, max_depth_mm=140.0),
                    )
                    recipe = _default_recipe_for_device("afo", case.landmark_coords)
                elif dev == "tf":
                    case = generate_transtibial_limb(length_mm=240.0, proximal_radius=68.0, distal_radius=44.0, open_cuff=True, seed=7)
                    aligned, qa_rep = run_scan_qa(
                        case.mesh,
                        mpt_point=case.landmark_coords["mpt"],
                        distal_point=case.landmark_coords["distal_end"],
                    )
                    recipe = _default_recipe_for_device("tf", case.landmark_coords)
                else:
                    case = generate_transtibial_limb(open_cuff=True, seed=0)
                    aligned, qa_rep = run_scan_qa(
                        case.mesh,
                        mpt_point=case.landmark_coords["mpt"],
                        distal_point=case.landmark_coords["distal_end"],
                    )
                    recipe = _default_recipe_for_device("tt", case.landmark_coords)
                recipe.scan_sha256 = mesh_sha256(aligned)
                sid = store.create(aligned, qa_rep, recipe)
                self._send_json({
                    "session_id": sid,
                    "mesh": _mesh_payload(aligned),
                    "qa": asdict(qa_rep),
                    "recipe": recipe.to_dict(),
                })
                return

            if parsed.path.startswith("/api/download/"):
                parts = parsed.path.split("/")
                if len(parts) != 5:
                    self._send_json({"error": "Invalid download path"}, HTTPStatus.NOT_FOUND)
                    return
                _, _, _, sid, fname = parts
                if not _SAFE_NAME_RE.match(fname):
                    self._send_json({"error": "Unsafe filename"}, HTTPStatus.BAD_REQUEST)
                    return
                rec = store.get(sid)
                if rec is None:
                    self._send_json({"error": "Unknown session"}, HTTPStatus.NOT_FOUND)
                    return
                fpath = rec.out_path / fname
                if not fpath.exists():
                    self._send_json({"error": "File not found"}, HTTPStatus.NOT_FOUND)
                    return
                blob = fpath.read_bytes()
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Content-Disposition", f'attachment; filename="{fname}"')
                self.send_header("Content-Length", str(len(blob)))
                self.end_headers()
                self.wfile.write(blob)
                return

            self._send_json({"error": "Not found"}, HTTPStatus.NOT_FOUND)

        def do_POST(self) -> None:  # noqa: N802
            try:
                body = self._read_json()
            except Exception as exc:  # pragma: no cover - malformed JSON guard
                self._send_json({"error": f"Invalid JSON: {exc}"}, HTTPStatus.BAD_REQUEST)
                return

            if self.path == "/api/upload":
                try:
                    raw_bytes = base64.b64decode(body["stl_base64"])
                    with tempfile.TemporaryDirectory() as tmp:
                        stl_p = Path(tmp) / "upload.stl"
                        stl_p.write_bytes(raw_bytes)
                        raw_mesh = read_stl(stl_p)
                    aligned, qa_rep = run_scan_qa(raw_mesh)
                    recipe = _default_recipe_for_device("tt", {})
                    recipe.scan_sha256 = mesh_sha256(aligned)
                    sid = store.create(aligned, qa_rep, recipe)
                    self._send_json({
                        "session_id": sid,
                        "mesh": _mesh_payload(aligned),
                        "qa": asdict(qa_rep),
                        "recipe": recipe.to_dict(),
                    })
                except Exception as exc:
                    self._send_json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
                return

            if self.path == "/api/rectify":
                rec = store.get(body.get("session_id", ""))
                if rec is None:
                    self._send_json({"error": "Unknown session_id"}, HTTPStatus.NOT_FOUND)
                    return
                try:
                    recipe = OpRecipe.from_dict(body["recipe"])
                    rec.recipe = recipe
                    res = apply_recipe(rec.canonical_mesh, recipe)
                    # Compute radial delta relative to M_0 for heatmap
                    if len(res.rectified_mesh.vertices) == len(rec.canonical_mesh.vertices):
                        norms = rec.canonical_mesh.vertex_normals()
                        deltas = [
                            v_dot(v_sub(res.rectified_mesh.vertices[i], rec.canonical_mesh.vertices[i]), norms[i])
                            for i in range(len(norms))
                        ]
                    else:
                        deltas = res.relief_field_mm
                    self._send_json({"rectified": _mesh_payload(res.rectified_mesh, deltas=deltas)})
                except Exception as exc:
                    self._send_json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
                return

            if self.path == "/api/diff":
                rec = store.get(body.get("session_id", ""))
                if rec is None:
                    self._send_json({"error": "Unknown session_id"}, HTTPStatus.NOT_FOUND)
                    return
                try:
                    cur = OpRecipe.from_dict(body["recipe"])
                    self._send_json({"diff": diff_recipes(rec.baseline_recipe, cur)})
                except Exception as exc:
                    self._send_json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
                return

            if self.path == "/api/build":
                rec = store.get(body.get("session_id", ""))
                if rec is None:
                    self._send_json({"error": "Unknown session_id"}, HTTPStatus.NOT_FOUND)
                    return
                try:
                    recipe = OpRecipe.from_dict(body["recipe"])
                    target = body.get("target", "orcaslicer")
                    rec.recipe = recipe
                    res = apply_recipe(rec.canonical_mesh, recipe)
                    out = rec.out_path
                    write_binary_stl(res.rectified_mesh, out / "rectified_positive.stl")
                    recipe.save(out / "recipe.oprecipe.json")

                    if recipe.device_type == "afo":
                        sh = build_afo_shell(
                            res.rectified_mesh,
                            wall_neck_mm=float(recipe.shell.get("wall_distal_mm", 4.5)),
                            wall_calf_mm=float(recipe.shell.get("wall_proximal_mm", 2.5)),
                            trimline_fillet_mm=float(recipe.shell.get("trimline_fillet_mm", 10.0)),
                        )
                    elif recipe.device_type == "transfemoral_socket":
                        sh = build_tf_socket_shell(
                            res.rectified_mesh,
                            wall_distal_mm=float(recipe.shell.get("wall_distal_mm", 5.5)),
                            wall_proximal_mm=float(recipe.shell.get("wall_proximal_mm", 3.0)),
                            flexion_deg=float(recipe.alignment.get("flexion_deg", 5.0)),
                            adduction_deg=float(recipe.alignment.get("adduction_deg", 8.0)),
                        )
                    else:
                        sh = build_tt_socket_shell(
                            res.rectified_mesh,
                            wall_distal_mm=float(recipe.shell.get("wall_distal_mm", 4.5)),
                            wall_proximal_mm=float(recipe.shell.get("wall_proximal_mm", 2.5)),
                            flexion_deg=float(recipe.alignment.get("flexion_deg", 5.0)),
                            adduction_deg=float(recipe.alignment.get("adduction_deg", 5.0)),
                        )

                    write_binary_stl(sh.shell_mesh, out / "device_shell.stl")
                    threemf_name = f"device_{target}.3mf"
                    export_3mf(sh.shell_mesh, sh.modifier_mesh, out / threemf_name, target=target)
                    downloads = {
                        "Shell STL (device_shell.stl)": f"/api/download/{body['session_id']}/device_shell.stl",
                        f"Slicer 3MF ({threemf_name})": f"/api/download/{body['session_id']}/{threemf_name}",
                        "Rectified Positive STL": f"/api/download/{body['session_id']}/rectified_positive.stl",
                        "Recipe (.oprecipe.json)": f"/api/download/{body['session_id']}/recipe.oprecipe.json",
                    }
                    if recipe.device_type != "afo":
                        carver = export_carver_positive(res.rectified_mesh)
                        write_binary_stl(carver.positive_mesh, out / "carver_positive.stl")
                        generate_4axis_gcode_and_simulate(
                            carver.positive_mesh,
                            output_path=out / "carver_4axis.gcode",
                        )
                        downloads["Carver Positive STL"] = f"/api/download/{body['session_id']}/carver_positive.stl"
                        downloads["Carver 4-Axis G-Code (.gcode)"] = f"/api/download/{body['session_id']}/carver_4axis.gcode"

                    lt = lint_device(
                        sh.shell_mesh,
                        measured_wall_override_mm=float(recipe.shell.get("wall_proximal_mm", 2.5)),
                    )
                    export_fabrication_dossier(recipe, rec.qa_report, lt, out)
                    downloads["Dossier PDF (device_dossier.pdf)"] = f"/api/download/{body['session_id']}/device_dossier.pdf"
                    downloads["Dossier Markdown"] = f"/api/download/{body['session_id']}/device_dossier.md"
                    downloads["Dossier JSON"] = f"/api/download/{body['session_id']}/device_dossier.json"

                    self._send_json({
                        "rectified": _mesh_payload(res.rectified_mesh),
                        "shell": _mesh_payload(sh.shell_mesh),
                        "lint": asdict(lt),
                        "hardware_metrics": sh.hardware_metrics,
                        "downloads": downloads,
                    })
                except Exception as exc:
                    self._send_json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
                return

            self._send_json({"error": "Not found"}, HTTPStatus.NOT_FOUND)

    return OpUIHandler


def create_server(host: str = "127.0.0.1", port: int = 8765) -> tuple[ThreadingHTTPServer, SessionStore]:
    """Create a configured local HTTP server and SessionStore for the 3D browser UI."""
    store = SessionStore()
    server = ThreadingHTTPServer((host, port), _make_handler(store))
    return server, store


def serve_ui(host: str = "127.0.0.1", port: int = 8765, open_browser: bool = True) -> None:
    """Run the 3D clinician browser UI server until Ctrl-C."""
    server, store = create_server(host=host, port=port)
    actual_port = server.server_address[1]
    url = f"http://{host}:{actual_port}/"
    print(f"op-core 3D Browser UI running at {url} (Ctrl-C to stop)")
    if open_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        store.close_all()
