"""Optional offline playback of experimental poses using real MANO geometry.

No device access, synthetic mesh, model download, or installation is performed.
A recording gap over 0.25 s hides the held frame while playing, not while paused.
"""
from __future__ import annotations

import bisect
import importlib
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Iterable

import numpy as np

from .config import digest
from .mano import DependencyError, ManoBackend


class ViewerError(RuntimeError):
    """The optional desktop viewer cannot run in this environment."""


def parse_pose_frames(lines: Iterable[str]) -> list[dict]:
    """Pure JSONL validation; unavailable poses remain explicit timeline gaps."""
    frames = []
    qualities = {"experimental", "experimental_partial", "experimental_not_converged", "unavailable"}
    for line_number, line in enumerate(lines, 1):
        if not line.strip():
            continue
        try:
            frame = json.loads(line)
            if not isinstance(frame, dict):
                raise ValueError("frame must be an object")
            stamp = frame.get("timestamp_s")
            if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
                raise ValueError("timestamp_s must be a finite number")
            if frames and stamp <= frames[-1]["timestamp_s"]:
                raise ValueError("timestamps must be strictly increasing")
            quality = frame.get("quality")
            if not isinstance(quality, str) or quality not in qualities:
                raise ValueError("unknown or missing experimental quality")
            labels = frame.get("source_labels")
            if not isinstance(labels, list) or len(labels) != 16 or not all(isinstance(x, str) for x in labels):
                raise ValueError("source_labels must contain 16 strings")
            checksum = frame.get("model_rest_sha256")
            if not isinstance(checksum, str) or len(checksum) != 64 or any(c not in "0123456789abcdef" for c in checksum):
                raise ValueError("model_rest_sha256 must be a lowercase SHA-256 digest")
            if frames and checksum != frames[0]["model_rest_sha256"]:
                raise ValueError("recording mixes model rest digests")
            renderable = quality in {"experimental", "experimental_partial"} and frame.get("solver_success") is not False
            if renderable:
                angles = np.asarray(frame.get("anatomical_euler_rad"), dtype=float)
                root = np.asarray(frame.get("root_quat_xyzw"), dtype=float)
                if angles.shape != (16, 3) or not np.isfinite(angles).all():
                    raise ValueError("anatomical_euler_rad must be finite with shape (16, 3)")
                if root.shape != (4,) or not np.isfinite(root).all() or np.linalg.norm(root) < 1e-12:
                    raise ValueError("root_quat_xyzw must be a finite nonzero quaternion")
            frames.append({**frame, "renderable": renderable,
                           "inferred_joint_count": sum(label.startswith("inferred_") for label in labels[1:])})
        except (ValueError, TypeError, OverflowError) as exc:
            raise ValueError(f"Pose JSONL line {line_number}: {exc}") from exc
    if not frames:
        raise ValueError("Pose JSONL contains no frames")
    return frames


def run_viewer(pose_path, assets_root, *, speed: float = 1.0, hand="right") -> None:
    """Open an offline Polyscope player; imports/models are deferred until now."""
    if isinstance(speed, bool) or not isinstance(speed, (int, float)) or not math.isfinite(speed) or speed <= 0:
        raise ValueError("speed must be a finite positive number")
    if sys.platform.startswith("linux") and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        raise ViewerError("No DISPLAY or WAYLAND_DISPLAY; the viewer needs a desktop OpenGL session. Other commands remain usable.")
    with Path(pose_path).open(encoding="utf-8") as stream:
        frames = parse_pose_frames(stream)
    backend = ManoBackend(assets_root, hand=hand)  # Validate assets before optional GUI imports.
    try:
        ps = importlib.import_module("polyscope")
        ui = importlib.import_module("polyscope.imgui")
    except (ImportError, OSError) as exc:
        raise DependencyError(f"Optional Polyscope desktop dependency is unavailable: {exc}. Nothing was installed.") from exc
    if digest(backend.export_rest()) != frames[0]["model_rest_sha256"]:
        raise ViewerError("model_rest_sha256 mismatch: this model/backend rest export differs from the solver's. Refusing playback.")
    try:
        ps.init()
    except Exception as exc:
        raise ViewerError(f"Polyscope could not initialize the desktop/OpenGL session: {exc}") from exc
    timestamps = [frame["timestamp_s"] for frame in frames]
    state = {"index": 0, "playing": False, "anchor_wall": 0.0,
             "anchor_pose": timestamps[0], "drawn": None, "mesh": None, "error": None}

    def callback():
        if ui.Button("Pause" if state["playing"] else "Play"):
            state["playing"] = not state["playing"]
            state["anchor_wall"] = time.monotonic()
            state["anchor_pose"] = timestamps[state["index"]]
        changed, selected = ui.SliderInt("Frame", state["index"], 0, len(frames) - 1)
        if changed:
            state["index"], state["playing"] = selected, False
        gap = False
        if state["playing"]:
            playhead = state["anchor_pose"] + (time.monotonic() - state["anchor_wall"]) * speed
            state["index"] = max(0, bisect.bisect_right(timestamps, playhead) - 1)
            if playhead >= timestamps[-1]:
                state["playing"] = False
            else:
                gap = playhead - timestamps[state["index"]] > 0.25
        index = state["index"]
        frame = frames[index]
        visible = frame["renderable"] and not gap and state["error"] is None
        if not visible and state["mesh"] is not None:
            state["mesh"].set_enabled(False)
        if visible:
            try:
                if state["drawn"] != index:
                    # Hide the old mesh before model evaluation; failures cannot leave it visible.
                    if state["mesh"] is not None:
                        state["mesh"].set_enabled(False)
                    geometry = backend.render(frame["anatomical_euler_rad"], frame["root_quat_xyzw"])
                    vertices = np.asarray(geometry["vertices_m"], dtype=float)
                    if state["mesh"] is None:
                        state["mesh"] = ps.register_surface_mesh(
                            "Experimental MANO (metres)", vertices,
                            np.asarray(geometry["faces"], dtype=int), enabled=True, smooth_shade=True)
                    else:
                        state["mesh"].update_vertex_positions(vertices)
                    state["drawn"] = index
                state["mesh"].set_enabled(True)
            except Exception as exc:
                if state["mesh"] is not None:
                    state["mesh"].set_enabled(False)
                state["error"], state["playing"] = str(exc), False
        ui.TextUnformatted("EXPERIMENTAL / OFFLINE — not anatomically verified; yaw drift is not corrected")
        ui.TextUnformatted(f"Frame {index + 1}/{len(frames)} | t={frame['timestamp_s']:.3f} s | {frame['quality']}")
        ui.TextUnformatted(f"Inferred finger joints: {frame['inferred_joint_count']}/15 | speed {speed:g}x")
        status = "Playing" if state["playing"] else "Paused / offline frame inspection"
        if gap:
            status = "Recording gap (>0.25 s): mesh hidden"
        elif not frame["renderable"]:
            status = f"No usable pose: {frame.get('reason', frame['quality'])}; mesh hidden"
        if state["error"]:
            status = f"Model rendering failed: {state['error']}; mesh hidden"
        ui.TextUnformatted(status)

    ps.set_user_callback(callback)
    try:
        ps.show()
    finally:
        ps.clear_user_callback()
    if state["error"]:
        raise ViewerError(f"MANO rendering failed: {state['error']}")
