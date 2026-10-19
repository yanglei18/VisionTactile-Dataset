"""Recover explicitly guided stationary holds, without guessing motion labels."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path


def stationary_windows_from_events(path):
    """Read completed official three-pose captures (3 s prepare + 2 s hold).

    Use the central 0.5 seconds of each hold. Raw stationarity is independently
    checked by the fitter; a completed button workflow is not proof of stillness.
    Canceled/replaced/incomplete captures cannot become measurement windows.
    These timings are the existing live_runtime PREPARE/CAPTURE_SECONDS contract,
    not timings inferred from a sensor or the event completion scheduling delay.
    """
    source = Path(path)
    pending = None
    windows = []
    hasher = hashlib.sha256()
    size = 0
    with source.open("rb") as stream:
        for number, line in enumerate(stream, 1):
            hasher.update(line)
            size += len(line)
            if not line.strip():
                continue
            try:
                event = json.loads(line)
            except (ValueError, UnicodeDecodeError) as exc:
                raise ValueError(f"Invalid events JSON at line {number}") from exc
            if not isinstance(event, dict):
                raise ValueError(f"Events line {number} must be an object")
            kind = event.get("event")
            if kind in {"clear_calibration", "disconnected", "connecting", "cancel_capture"}:
                pending = None
            elif kind == "capture":
                pending = None
                if event.get("step") not in {"neutral", "fingers_down", "thumb_up"}:
                    continue
                value = event.get("timestamp_s")
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                    raise ValueError(f"Invalid capture timestamp at events line {number}")
                pending = event
            elif kind == "capture_complete" and pending is not None:
                if (event.get("step") != pending["step"]
                        or event.get("session_id") != pending.get("session_id")):
                    continue
                finish = event.get("timestamp_s")
                start = pending["timestamp_s"]
                if (isinstance(finish, bool) or not isinstance(finish, (int, float))
                        or not math.isfinite(finish) or not 5 <= finish - start <= 10):
                    pending = None
                    continue
                windows.append({"start_s": start + 3.75, "end_s": start + 4.25,
                                "label": pending["step"], "time_basis": "absolute_monotonic"})
                pending = None
    if len(windows) < 3:
        raise ValueError("At least three completed stationary captures are required in events.jsonl")
    return windows, {"path": str(source.resolve()), "sha256": hasher.hexdigest(), "bytes": size,
                     "prepare_s": 3., "capture_s": 2., "central_window_s": .5,
                     "timing_basis": "official_live_three_pose_workflow",
                     "stationarity_requires_raw_validation": True}
