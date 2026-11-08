"""Stable per-user, per-hand control endpoints; this module performs no I/O."""
import os
from pathlib import Path


def default_hand_control_socket(hand):
    if hand not in {"left", "right"}:
        raise ValueError("hand must be left or right")
    runtime = os.environ.get("XDG_RUNTIME_DIR")
    root = Path(runtime) / "visiontactile" if runtime else Path(f"/tmp/vt-hands-{os.getuid()}")
    return root / "hands" / hand / "control.sock"
