"""Standalone runtime location; validation is shared with the workbench."""
from pathlib import Path
from vt_capture_console.backend.hand_resources import MAX_JSON_BYTES, read_json, sha256
from vt_capture_console.backend.hand_resources import load_resources as _load_resources


def bundled_root():
    return Path(__file__).resolve().parent / "assets" / "runtime"


def load_resources(root=None, *, hand="right"):
    return _load_resources(bundled_root() if root is None else root, hand=hand)
