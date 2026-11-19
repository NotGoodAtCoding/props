"""Project 3 & 4: Dual-schema .3mf exporter (--target prusaslicer | orcaslicer) with embedded structural modifier volumes."""

from __future__ import annotations

import zipfile
from pathlib import Path
from xml.sax.saxutils import escape

from .mesh import TriMesh

VALID_TARGETS = ("prusaslicer", "orcaslicer")


def _mesh_xml(obj_id: int, name: str, mesh: TriMesh) -> str:
    verts_xml = "\n".join(
        f'          <vertex x="{x:.4f}" y="{y:.4f}" z="{z:.4f}" />'
        for x, y, z in mesh.vertices
    )
    tris_xml = "\n".join(
        f'          <triangle v1="{a}" v2="{b}" v3="{c}" />'
        for a, b, c in mesh.faces
    )
    return (
        f'    <object id="{obj_id}" name="{escape(name)}" type="model">\n'
        f"      <mesh>\n"
        f"        <vertices>\n{verts_xml}\n        </vertices>\n"
        f"        <triangles>\n{tris_xml}\n        </triangles>\n"
        f"      </mesh>\n"
        f"    </object>"
    )


def _prusaslicer_config(infill_pct: int, wall_loops: int) -> str:
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<config>
  <object id="1" instances_count="1">
    <metadata type="object" key="name" value="op_device_assembly"/>
    <volume firstid="0" lastid="0">
      <metadata type="volume" key="name" value="device_shell"/>
    </volume>
    <volume firstid="1" lastid="1">
      <metadata type="volume" key="name" value="distal_reinforcement_modifier"/>
      <metadata type="volume" key="modifier" value="1"/>
      <metadata type="volume" key="fill_density" value="{infill_pct}%"/>
      <metadata type="volume" key="perimeters" value="{wall_loops}"/>
    </volume>
  </object>
</config>
"""


def _orcaslicer_config(infill_pct: int, wall_loops: int) -> str:
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<config>
  <object id="1">
    <metadata key="name" value="op_device_assembly"/>
    <part id="1" subtype="normal_part">
      <metadata key="name" value="device_shell"/>
      <metadata key="matrix" value="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"/>
    </part>
    <part id="2" subtype="modifier_part">
      <metadata key="name" value="distal_reinforcement_modifier"/>
      <metadata key="sparse_infill_density" value="{infill_pct}%"/>
      <metadata key="wall_loops" value="{wall_loops}"/>
      <metadata key="matrix" value="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"/>
    </part>
  </object>
</config>
"""


def export_3mf(
    shell_mesh: TriMesh,
    modifier_mesh: TriMesh,
    output_path: str | Path,
    target: str = "orcaslicer",
    infill_pct: int = 100,
    wall_loops: int = 6,
) -> Path:
    """Write a slicer-targeted .3mf archive containing device shell + modifier volume."""
    if target not in VALID_TARGETS:
        raise ValueError(f"Unsupported slicer target {target!r}; choose from {VALID_TARGETS}")

    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)

    content_types = """<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
  <Default Extension="config" ContentType="text/xml"/>
</Types>
"""

    rels = """<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
</Relationships>
"""

    obj_shell = _mesh_xml(1, "device_shell", shell_mesh)
    obj_mod = _mesh_xml(2, "distal_reinforcement_modifier", modifier_mesh)
    model_xml = f"""<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
  <resources>
{obj_shell}
{obj_mod}
  </resources>
  <build>
    <item objectid="1" />
    <item objectid="2" />
  </build>
</model>
"""

    with zipfile.ZipFile(out, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("[Content_Types].xml", content_types)
        zf.writestr("_rels/.rels", rels)
        zf.writestr("3D/3dmodel.model", model_xml)
        if target == "prusaslicer":
            zf.writestr("Metadata/Slic3r_PE_model.config", _prusaslicer_config(infill_pct, wall_loops))
        else:
            zf.writestr("Metadata/model_settings.config", _orcaslicer_config(infill_pct, wall_loops))

    return out
