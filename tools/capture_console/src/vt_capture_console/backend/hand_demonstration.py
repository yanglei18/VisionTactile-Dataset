"""Generate teaching clips in the configured numerical Python, never in Qt."""
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import tempfile
import time

import numpy as np


MAX_CLIP_BYTES = 4 * 1024 * 1024


def validate_clips(document, hand, neutral):
    if (not isinstance(document, dict) or document.get("schema_version") != 1
            or document.get("hand") != hand or document.get("synthetic_demonstration") is not True
            or set(document.get("clips", {})) != {"flex_fingers", "flex_thumb"}):
        raise ValueError("Invalid hand demonstration identity")
    result = {}
    for action, pair in document["clips"].items():
        if not isinstance(pair, list) or len(pair) != 2:
            raise ValueError("Invalid demonstration geometry")
        vertices, joints = (np.asarray(value, dtype=np.float32) for value in pair)
        if (vertices.shape != (17, len(neutral["vertices_m"]), 3) or joints.shape != (17, 21, 3)
                or not np.isfinite(vertices).all() or not np.isfinite(joints).all()
                or np.max(np.abs(vertices)) > 2 or np.max(np.abs(joints)) > 2
                or not np.allclose(vertices[0], neutral["vertices_m"], atol=1e-7)
                or not np.allclose(joints[0], neutral["joints_m"], atol=1e-7)):
            raise ValueError("Demonstration geometry does not match the selected hand")
        result[action] = (vertices, joints)
    return result


def load_demonstration(interpreter, root, hand, neutral, *, cancelled, timeout=60.):
    python = Path(interpreter).expanduser()
    if not python.is_absolute() or not python.is_file() or not os.access(python, os.X_OK):
        raise ValueError("A configured numerical Python is required for hand demonstrations")
    if cancelled.is_set():
        raise RuntimeError("Hand resource preparation cancelled")
    with tempfile.TemporaryDirectory(prefix=f"vt-{hand}-demo-") as directory:
        output = Path(directory) / "clips.json"
        with (Path(directory) / "error.log").open("w+b") as errors:
            process = subprocess.Popen([str(python), "-I", "-m", "vt_fsglove12.teaching_geometry",
                "--resources", str(root), "--hand", hand, "--output", str(output)],
                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=errors)
            try:
                deadline = time.monotonic() + timeout
                while process.poll() is None:
                    if cancelled.wait(.02):
                        raise RuntimeError("Hand resource preparation cancelled")
                    if time.monotonic() > deadline:
                        raise TimeoutError("Hand demonstration preparation timed out")
                if process.returncode:
                    errors.seek(0)
                    detail = errors.read(4096).decode("utf-8", errors="replace")
                    raise RuntimeError("Hand demonstration preparation failed: " + detail)
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=.5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=1.)
        with output.open("rb") as stream:
            payload = stream.read(MAX_CLIP_BYTES + 1)
        if len(payload) > MAX_CLIP_BYTES:
            raise ValueError("Hand demonstration exceeds its size limit")
        return validate_clips(json.loads(payload), hand, neutral)
