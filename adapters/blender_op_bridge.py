"""Standalone Blender 3.6/4.0+ O&P Technician Workspace Adapter (GPL-3.0 boundary).

This adapter lives outside MIT `src/op_core/` and delegates all geometry compilation to
`op-core` via subprocess CLI + `.oprecipe.json`, avoiding tight `bpy` coupling.

Installation in Blender:
  Edit -> Preferences -> Add-ons -> Install... -> select `adapters/blender_op_bridge.py`
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path
from typing import Any

bl_info = {
    "name": "op-core O&P Technician Bridge",
    "author": "props contributors",
    "version": (0, 2, 0),
    "blender": (3, 6, 0),
    "location": "View3D > Sidebar > O&P (op-core)",
    "description": "Stripped O&P clinical workspace and .oprecipe.json bridge to op-core CLI",
    "category": "Medical",
}

NON_CLINICAL_WORKSPACES = (
    "Animation",
    "Compositing",
    "Geometry Nodes",
    "Rendering",
    "Scripting",
    "Shading",
    "Texture Paint",
    "UV Editing",
    "Video Editing",
)


def build_cli_command(
    action: str,
    scan_path: str | Path,
    recipe_path: str | Path | None = None,
    out_dir: str | Path | None = None,
    target: str = "orcaslicer",
    python_exe: str = sys.executable,
) -> list[str]:
    """Build a deterministic `op-core` subprocess invocation."""
    cmd = [python_exe, "-m", "op_core.cli", action, str(scan_path)]
    if action == "build":
        if recipe_path is None or out_dir is None:
            raise ValueError("'build' requires recipe_path and out_dir")
        cmd.extend([str(recipe_path), "--out-dir", str(out_dir), "--target", target])
    elif action == "qa" and out_dir is not None:
        cmd.extend(["--out-stl", str(Path(out_dir) / "aligned_scan.stl")])
    return cmd


def run_op_core_subprocess(cmd: list[str], repo_src: str | Path | None = None) -> dict[str, Any]:
    """Execute an `op-core` CLI command and parse its JSON stdout summary."""
    import os

    env = dict(os.environ)
    if repo_src is not None:
        prev = env.get("PYTHONPATH", "")
        env["PYTHONPATH"] = f"{repo_src}{os.pathsep}{prev}" if prev else str(repo_src)
    proc = subprocess.run(cmd, capture_output=True, text=True, env=env, check=True)
    return json.loads(proc.stdout)


def workspaces_to_hide(existing_names: list[str]) -> list[str]:
    """Return non-clinical Blender workspace tab names to hide from O&P technicians."""
    hide_set = set(NON_CLINICAL_WORKSPACES)
    return [name for name in existing_names if name in hide_set]


try:  # pragma: no cover - Blender runtime UI registration
    import bpy  # type: ignore

    class OPCORE_OT_strip_workspace(bpy.types.Operator):
        bl_idname = "opcore.strip_workspace"
        bl_label = "Simplify O&P Workspace"
        bl_description = "Remove non-clinical animation/rendering/UV tabs for focused O&P work"

        def execute(self, context):
            names = [ws.name for ws in bpy.data.workspaces]
            for name in workspaces_to_hide(names):
                ws = bpy.data.workspaces.get(name)
                if ws and len(bpy.data.workspaces) > 1:
                    bpy.data.batch_remove([ws])
            return {"FINISHED"}

    class OPCORE_OT_build_recipe(bpy.types.Operator):
        bl_idname = "opcore.build_recipe"
        bl_label = "Compile .oprecipe.json via op-core"
        bl_description = "Export active mesh and run headless op-core build"

        def execute(self, context):
            obj = context.active_object
            if obj is None or obj.type != "MESH":
                self.report({"ERROR"}, "Select a limb scan mesh object first")
                return {"CANCELLED"}
            scene = context.scene
            out_dir = Path(bpy.path.abspath(scene.opcore_out_dir))
            out_dir.mkdir(parents=True, exist_ok=True)
            scan_stl = out_dir / "blender_export.stl"
            bpy.ops.wm.stl_export(filepath=str(scan_stl), export_selected_objects=True)
            cmd = build_cli_command(
                "build",
                scan_stl,
                recipe_path=bpy.path.abspath(scene.opcore_recipe_path),
                out_dir=out_dir,
                target=scene.opcore_slicer_target,
            )
            res = run_op_core_subprocess(cmd)
            self.report({"INFO"}, f"op-core build lint_passed={res.get('lint_passed')}")
            return {"FINISHED"}

    class OPCORE_PT_main_panel(bpy.types.Panel):
        bl_label = "O&P (op-core Bridge)"
        bl_idname = "OPCORE_PT_main_panel"
        bl_space_type = "VIEW_3D"
        bl_region_type = "UI"
        bl_category = "O&P"

        def draw(self, context):
            layout = self.layout
            scene = context.scene
            layout.operator("opcore.strip_workspace", icon="WORKSPACE")
            layout.prop(scene, "opcore_recipe_path")
            layout.prop(scene, "opcore_out_dir")
            layout.prop(scene, "opcore_slicer_target")
            layout.operator("opcore.build_recipe", icon="EXPORT")

    _CLASSES = (OPCORE_OT_strip_workspace, OPCORE_OT_build_recipe, OPCORE_PT_main_panel)

    def register():
        bpy.types.Scene.opcore_recipe_path = bpy.props.StringProperty(name="Recipe (.oprecipe.json)", subtype="FILE_PATH")
        bpy.types.Scene.opcore_out_dir = bpy.props.StringProperty(name="Output Dir", default="//op_out", subtype="DIR_PATH")
        bpy.types.Scene.opcore_slicer_target = bpy.props.EnumProperty(
            name="Slicer",
            items=[("orcaslicer", "OrcaSlicer", ""), ("prusaslicer", "PrusaSlicer", "")],
            default="orcaslicer",
        )
        for cls in _CLASSES:
            bpy.utils.register_class(cls)

    def unregister():
        for cls in reversed(_CLASSES):
            bpy.utils.unregister_class(cls)
        del bpy.types.Scene.opcore_slicer_target
        del bpy.types.Scene.opcore_out_dir
        del bpy.types.Scene.opcore_recipe_path

except ImportError:
    bpy = None  # Running outside Blender
