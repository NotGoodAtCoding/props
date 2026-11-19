"""Integration tests for the Phase 4 local 3D browser UI server and embedded JavaScript."""

from __future__ import annotations

import base64
import json
import re
import shutil
import subprocess
import tempfile
import threading
import unittest
import urllib.request
from pathlib import Path

from op_core.mesh import write_binary_stl
from op_core.synthetic import generate_transtibial_limb
from op_core.ui_server import _HTML_PAGE, create_server


class TestUIServer(unittest.TestCase):
    def test_embedded_javascript_syntax_is_valid(self) -> None:
        node = shutil.which("node")
        if not node:
            self.skipTest("node not available for JS syntax check")
        match = re.search(r"<script>(.*)</script>", _HTML_PAGE, re.DOTALL)
        self.assertIsNotNone(match)
        with tempfile.TemporaryDirectory() as tmp:
            js_path = Path(tmp) / "app.js"
            js_path.write_text(match.group(1), encoding="utf-8")
            proc = subprocess.run([node, "--check", str(js_path)], capture_output=True, text=True)
            self.assertEqual(proc.returncode, 0, proc.stderr)

    def test_ui_rest_workflow_sample_rectify_build_upload_and_download(self) -> None:
        server, store = create_server(host="127.0.0.1", port=0)
        port = server.server_address[1]
        base = f"http://127.0.0.1:{port}"
        th = threading.Thread(target=server.serve_forever, daemon=True)
        th.start()
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        try:
            # 1. Index HTML + Health
            html = opener.open(f"{base}/").read().decode("utf-8")
            self.assertIn("op-core 3D Clinical Workstation", html)
            health = json.loads(opener.open(f"{base}/api/health").read().decode("utf-8"))
            self.assertEqual(health["status"], "ok")

            # 2. Sample TT session -> Rectify -> Build -> Download
            sample = json.loads(opener.open(f"{base}/api/sample?device=tt").read().decode("utf-8"))
            sid = sample["session_id"]
            self.assertGreater(sample["mesh"]["vertex_count"], 500)

            req_rect = urllib.request.Request(
                f"{base}/api/rectify",
                data=json.dumps({"session_id": sid, "recipe": sample["recipe"]}).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            rect_data = json.loads(opener.open(req_rect).read().decode("utf-8"))
            self.assertEqual(len(rect_data["rectified"]["deltas"]), sample["mesh"]["vertex_count"])

            req_build = urllib.request.Request(
                f"{base}/api/build",
                data=json.dumps({"session_id": sid, "recipe": sample["recipe"], "target": "orcaslicer"}).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            build_data = json.loads(opener.open(req_build).read().decode("utf-8"))
            self.assertTrue(build_data["lint"]["passed"])
            self.assertIn("Slicer 3MF (device_orcaslicer.3mf)", build_data["downloads"])

            dl_url = base + build_data["downloads"]["Slicer 3MF (device_orcaslicer.3mf)"]
            blob = opener.open(dl_url).read()
            self.assertTrue(blob.startswith(b"PK"))  # Valid ZIP/.3mf archive

            # 3. Sample AFO session + build
            afo = json.loads(opener.open(f"{base}/api/sample?device=afo").read().decode("utf-8"))
            req_afo_build = urllib.request.Request(
                f"{base}/api/build",
                data=json.dumps({"session_id": afo["session_id"], "recipe": afo["recipe"], "target": "prusaslicer"}).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            afo_build = json.loads(opener.open(req_afo_build).read().decode("utf-8"))
            self.assertTrue(afo_build["lint"]["passed"])

            # 4. Upload raw STL via base64
            case = generate_transtibial_limb(open_cuff=True, seed=2)
            with tempfile.TemporaryDirectory() as tmp:
                stl_p = Path(tmp) / "up.stl"
                write_binary_stl(case.mesh, stl_p)
                b64 = base64.b64encode(stl_p.read_bytes()).decode("ascii")
            req_up = urllib.request.Request(
                f"{base}/api/upload",
                data=json.dumps({"filename": "up.stl", "stl_base64": b64}).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            up_data = json.loads(opener.open(req_up).read().decode("utf-8"))
            self.assertEqual(up_data["qa"]["status"], "PASS")
        finally:
            server.shutdown()
            server.server_close()
            store.close_all()


if __name__ == "__main__":
    unittest.main()
