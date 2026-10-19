"""Configuration-only composition of independent hands and modalities.

Importing this module never imports torch/MANO, Qt or ROS, and never opens a
serial device. Disabled modalities have no launch action and no expected topic.
"""
from __future__ import annotations

from copy import deepcopy
import os
from pathlib import Path
import stat

import yaml

from .launch_support import pose_options


SIDES = ("left", "right")
HAND_DEFAULTS = {
    "pose_enabled": False, "tactile_enabled": False,
    "imu_port": "", "tactile_port": "", "pose_config": "", "tactile_config": "",
    "tactile_layout": "", "tactile_baseline": "", "pose_gui": True,
    "tactile_gui": True, "tactile_overlay": True, "output_rate_hz": "",
}
RUNTIME_DEFAULTS = {
    "glove_python": "", "fsglove_source": "", "mano_assets": "",
    "tactile_gui_python": "/usr/bin/python3",
    "output_root": str(Path.home() / "visiontactile-data" / "glove_sessions"),
}
_PATHS = {"imu_port", "tactile_port", "pose_config", "tactile_config",
          "tactile_layout", "tactile_baseline", *RUNTIME_DEFAULTS}


def selected_hands(value):
    if value == "both":
        return set(SIDES)
    if value == "none":
        return set()
    items = value.split(",")
    if not items or any(item not in SIDES for item in items) or len(set(items)) != len(items):
        raise ValueError("hand selection must be none, left, right, both or left,right")
    return set(items)


def _mapping(value, allowed, name):
    if not isinstance(value, dict) or set(value) - set(allowed):
        raise ValueError(f"{name} must be a mapping containing only {', '.join(sorted(allowed))}")
    return value


def _path(value, base):
    if not value:
        return ""
    path = Path(os.path.expandvars(value)).expanduser()
    return str(path if path.is_absolute() else (base / path).resolve())


def load_hands(path=None, *, overrides=None):
    """Load a profile, then apply explicit CLI selections/paths without mutation."""
    base = Path.cwd()
    document = {}
    if path:
        location = Path(path).expanduser().resolve()
        base = location.parent
        document = yaml.safe_load(location.read_text(encoding="utf-8"))
        _mapping(document, {"schema_version", "runtime", "hands"}, "hands profile")
        if type(document.get("schema_version")) is not int or document["schema_version"] != 1:
            raise ValueError("hands profile requires schema_version=1")
    runtime = dict(RUNTIME_DEFAULTS)
    runtime.update(_mapping(document.get("runtime", {}), runtime, "runtime"))
    hands = {side: dict(HAND_DEFAULTS) for side in SIDES}
    configured = _mapping(document.get("hands", {}), SIDES, "hands")
    for side, values in configured.items():
        hands[side].update(_mapping(values, HAND_DEFAULTS, f"hands.{side}"))
    for values in (runtime, *hands.values()):
        for key, value in values.items():
            if key in _PATHS:
                if not isinstance(value, str):
                    raise ValueError(f"{key} must be a path string")
                values[key] = _path(value, base)
    overrides = {} if overrides is None else overrides
    allowed = {*runtime, "pose_hands", "tactile_hands",
               *(f"{side}_{field}" for side in SIDES for field in ("imu_port", "tactile_port", "pose_config", "tactile_layout"))}
    _mapping(overrides, allowed, "launch overrides")
    for key, value in overrides.items():
        if not isinstance(value, str):
            raise ValueError(f"override {key} must be a string")
        if not value:
            continue
        if key in runtime:
            runtime[key] = _path(value, Path.cwd())
        elif key in ("pose_hands", "tactile_hands"):
            selected = selected_hands(value)
            for side in SIDES:
                hands[side][key.replace("_hands", "_enabled")] = side in selected
        else:
            side, field = key.split("_", 1)
            hands[side][field] = _path(value, Path.cwd())
    devices = {}
    for side, values in hands.items():
        for key, default in HAND_DEFAULTS.items():
            if isinstance(default, bool) and type(values[key]) is not bool:
                raise ValueError(f"{side}.{key} must be a boolean")
        rate = values["output_rate_hz"]
        if rate != "" and (type(rate) not in (int, float) or not 1 <= rate <= 240):
            raise ValueError(f"{side}.output_rate_hz must be empty or a number in [1,240]")
        for enabled, field in (("pose_enabled", "imu_port"), ("tactile_enabled", "tactile_port")):
            if not values[enabled]:
                continue
            port = values[field]
            # A custom tactile YAML may carry the selected device. Resolve the
            # supported wildcard config too, so aliases cannot bypass isolation.
            if not port and field == "tactile_port" and values["tactile_config"]:
                tactile = yaml.safe_load(Path(values["tactile_config"]).read_text(encoding="utf-8"))
                if (not isinstance(tactile, dict) or set(tactile) != {"/**"}
                        or not isinstance(tactile["/**"], dict)
                        or not isinstance(tactile["/**"].get("ros__parameters"), dict)):
                    raise ValueError("set an explicit tactile_port for node-specific tactile_config; "
                                     "automatic port resolution only supports /**: ros__parameters")
                port = tactile["/**"]["ros__parameters"].get("port", "")
                if not isinstance(port, str):
                    raise ValueError("tactile config port must be a string")
            if not port:
                continue  # Explicitly disconnected / GUI manual selection.
            path = Path(port).expanduser().resolve()
            identity = ("path", str(path))
            try:
                info = path.stat()
            except FileNotFoundError:
                pass  # A disconnected device may reconnect later.
            else:
                if stat.S_ISCHR(info.st_mode):
                    identity = ("device", info.st_rdev)
            label = f"{side}.{field}"
            if identity in devices:
                raise ValueError(f"serial device reused by {devices[identity]} and {label}")
            devices[identity] = label
    return {"schema_version": 1, "runtime": runtime, "hands": hands}


def hand_plan(document):
    """Return launch descriptors and the exact standalone recording topic list."""
    runtime = document["runtime"]
    plan = []
    for side, values in document["hands"].items():
        entry = {"hand": side, "pose": None, "tactile": None, "record_topics": []}
        if values["pose_enabled"]:
            overlay = (values["tactile_enabled"] and values["tactile_overlay"]
                       and values["pose_gui"] and (side == "right" or bool(values["tactile_layout"])))
            entry["pose"] = pose_options({
                **{key: runtime[key] for key in ("glove_python", "fsglove_source", "mano_assets")},
                "hand": side, "namespace": f"/gloves/{side}",
                "output_root": str(Path(runtime["output_root"]) / side),
                "config": values["pose_config"], "port": values["imu_port"],
                "launch_gui": str(values["pose_gui"]).lower(),
                "output_rate_hz": str(values["output_rate_hz"]),
                "tactile_overlay": str(bool(overlay)).lower(),
                "tactile_layout": values["tactile_layout"],
            })
            entry["pose"].pop("reference_node_id")  # Child launch resolves its own config.
            entry["record_topics"].append(f"/gloves/{side}/state")
        if values["tactile_enabled"]:
            entry["tactile"] = {
                "hand": side, "namespace": f"/gloves/{side}/tactile",
                "port": values["tactile_port"], "gui": str(values["tactile_gui"]).lower(),
                "gui_python": runtime["tactile_gui_python"],
                "hand_layout_path": values["tactile_layout"],
                "baseline_path": values["tactile_baseline"],
                "initial_view": "hand" if side == "right" or values["tactile_layout"] else "matrix",
            }
            if values["tactile_config"]:
                entry["tactile"]["config"] = values["tactile_config"]
            entry["record_topics"].append(f"/gloves/{side}/tactile/samples")
        if entry["pose"] is not None or entry["tactile"] is not None:
            plan.append(entry)
    return deepcopy(plan)
