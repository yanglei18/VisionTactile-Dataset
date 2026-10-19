"""Pure workstation configuration validation shared by GUI and ROS manager.

This module neither imports ROS/Qt nor opens hardware. Launch parameters are
allowlisted recipes, never client-supplied shell commands or arbitrary argv.
"""
from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path
import re


PRESETS = {
    "cameras": ("camera", "vt_realsense_capture", "triple_realsense.launch.py",
                {"config_path", "output_root"}, {"config_path", "output_root"}),
    "trackers": ("tracker", "vt_vive_tracker", "triple_tracker.launch.py",
                 {"bundle_path", "role_map_path", "frame_id", "pyvut_source", "pyvut_site_packages"},
                 {"bundle_path", "role_map_path"}),
    "glove": ("glove", "vt_glove_bridge", "glove.launch.py",
              {"hand", "launch_gui", "glove_python", "fsglove_source", "mano_assets", "config",
               "port", "socket_path", "output_root", "output_rate_hz"}, {"hand"}),
    "tactile": ("tactile", "vt_tactile_capture", "tactile.launch.py",
                {"hand", "config", "port"}, {"hand", "port"}),
}
PATH_KEYS = {"config_path", "output_root", "bundle_path", "role_map_path", "glove_python",
             "fsglove_source", "mano_assets", "config", "port", "socket_path", "pyvut_source", "pyvut_site_packages"}


def parse_profiles(raw):
    if not isinstance(raw, list) or len(raw) > 32:
        raise ValueError("process_groups must be a list of at most 32 launch profiles")
    result, identifiers = [], set()
    for entry in raw:
        if not isinstance(entry, dict) or set(entry) - {"id", "label", "preset", "enabled", "parameters", "startup_timeout_s"}:
            raise ValueError("process group only accepts id, label, preset, enabled, parameters, startup_timeout_s")
        identity, title, preset = entry.get("id"), entry.get("label"), entry.get("preset")
        if not isinstance(identity, str) or not re.fullmatch(r"[a-z][a-z0-9_]{0,63}", identity) or identity in identifiers:
            raise ValueError("process group ids must be unique lowercase identifiers")
        if not isinstance(title, str) or not title.strip() or len(title.encode()) > 512 or "\0" in title:
            raise ValueError("process group label must be nonempty and bounded")
        if not isinstance(preset, str) or preset not in PRESETS:
            raise ValueError("unsupported device launch preset")
        enabled = entry.get("enabled", False)
        if type(enabled) is not bool:
            raise ValueError("process group enabled must be boolean")
        timeout = entry.get("startup_timeout_s", 30.)
        if type(timeout) not in (int, float) or not math.isfinite(timeout) or not 1 <= timeout <= 600:
            raise ValueError("startup_timeout_s must be between 1 and 600")
        parameters = entry.get("parameters", {})
        if not isinstance(parameters, dict) or set(parameters) - PRESETS[preset][3]:
            raise ValueError(f"unsupported parameters for {preset}; arbitrary commands are not accepted")
        parameters = dict(parameters)
        for key, value in parameters.items():
            if not isinstance(value, str) or len(value.encode()) > 4096 or any(c in value for c in ("\0", "\n", "\r")):
                raise ValueError("launch parameters must be bounded single-line strings")
            if key in PATH_KEYS and value:
                path = Path(value).expanduser()
                if not path.is_absolute():
                    raise ValueError(f"{key} must be an explicit absolute path")
                parameters[key] = str(path)
                if key in {"pyvut_source", "pyvut_site_packages"} and ":" in parameters[key]:
                    raise ValueError(f"{key} must be one directory, without a PYTHONPATH separator")
        if preset in {"glove", "tactile"} and parameters.get("hand") not in {"left", "right"}:
            raise ValueError("each hand process requires explicit hand: left or right")
        if preset == "glove":
            if parameters.get("launch_gui", "true") not in {"true", "false"}:
                raise ValueError("launch_gui must be the string true or false")
            if parameters.get("output_rate_hz"):
                try:
                    rate = float(parameters["output_rate_hz"])
                except ValueError as exc:
                    raise ValueError("output_rate_hz must be numeric") from exc
                if not math.isfinite(rate) or not 1 <= rate <= 1000:
                    raise ValueError("output_rate_hz must be between 1 and 1000")
        if enabled:
            required = set(PRESETS[preset][4])
            if preset == "glove" and parameters.get("launch_gui", "true") == "true":
                required |= {"glove_python", "fsglove_source", "mano_assets", "port"}
            if any(not parameters.get(key) for key in required):
                raise ValueError(f"enabled {identity} requires explicit parameters: {', '.join(sorted(required))}")
        identifiers.add(identity)
        result.append(dict(id=identity, label=title, preset=preset, enabled=enabled,
                           parameters=parameters, startup_timeout_s=float(timeout)))
    return tuple(result)


def validate_configuration(value):
    """Return a normalized ManagerConfig-compatible JSON object, or raise."""
    allowed = {"devices", "capture_status_topic", "stale_after_s", "offline_after_s", "disk_path",
               "quality_enabled", "quality_timeout_s", "quality_alignment_config", "metadata_refs", "process_groups", "scene_config_path"}
    if type(value) is not dict or set(value) - allowed:
        raise ValueError("unknown manager configuration keys")
    try:
        if len(json.dumps(value, ensure_ascii=False, allow_nan=False).encode()) > 1024 * 1024:
            raise ValueError("workstation configuration exceeds 1 MiB")
    except (TypeError, UnicodeError) as exc:
        raise ValueError("configuration must contain bounded JSON values") from exc
    result = deepcopy(value)
    result["process_groups"] = list(parse_profiles(result.get("process_groups", [])))

    def topic(text):
        if not isinstance(text, str) or not re.fullmatch(r"(?:/[A-Za-z_][A-Za-z0-9_]*)+", text):
            raise ValueError("topics must be absolute ROS topic names")

    if "devices" in result:
        devices = result["devices"]
        if not isinstance(devices, list) or len(devices) > 64:
            raise ValueError("devices must contain at most 64 subscriptions")
        ids = set()
        for device in devices:
            required = {"id", "label", "kind", "side", "topic"}
            if (type(device) is not dict or not required <= set(device)
                    or set(device) - required - {"image_reliability"}):
                raise ValueError("device requires id, label, kind, side and topic; image_reliability is optional")
            if any(not isinstance(v, str) or len(v) > 512 for v in device.values()):
                raise ValueError("device configuration fields must be bounded strings")
            if not device["id"] or not device["label"] or device["id"] in ids or device["kind"] not in {"camera", "tracker", "glove", "tactile"}:
                raise ValueError("invalid or duplicate device identity/kind")
            ids.add(device["id"])
            if device["side"] not in {"", "left", "right"} or (device["kind"] in {"glove", "tactile"} and not device["side"]):
                raise ValueError("glove/tactile require an explicit side")
            reliability = device.get("image_reliability", "best_effort")
            if reliability not in {"best_effort", "reliable"}:
                raise ValueError("image_reliability must be best_effort or reliable")
            if device["kind"] != "camera" and reliability != "best_effort":
                raise ValueError("image_reliability only applies to camera subscriptions")
            topic(device["topic"])
    topic(result.get("capture_status_topic", "/capture/status"))
    stale, offline = result.get("stale_after_s", 1.), result.get("offline_after_s", 5.)
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in (stale, offline)) or not .05 <= stale < offline <= 3600:
        raise ValueError("require 0.05 <= stale_after_s < offline_after_s <= 3600")
    if not isinstance(result.get("disk_path", "."), str):
        raise ValueError("disk_path must be a string")
    timeout = result.get("quality_timeout_s", 900.)
    if (type(result.get("quality_enabled", True)) is not bool or type(timeout) not in (int, float)
            or not math.isfinite(timeout) or not 1 <= timeout <= 86400):
        raise ValueError("quality_enabled must be boolean; quality_timeout_s must be between 1 and 86400")
    alignment = result.get("quality_alignment_config", "")
    if not isinstance(alignment, str) or len(alignment.encode()) > 4096:
        raise ValueError("quality_alignment_config must be a bounded file path")
    scene = result.get("scene_config_path", "")
    if not isinstance(scene, str) or len(scene.encode()) > 4096:
        raise ValueError("scene_config_path must be a bounded file path")
    refs = result.get("metadata_refs", [])
    if not isinstance(refs, list) or len(refs) > 16:
        raise ValueError("metadata_refs must contain at most 16 file references")
    names = set()
    for ref in refs:
        if type(ref) is not dict or set(ref) != {"name", "kind", "path"} or ref["kind"] not in {"config", "calibration"}:
            raise ValueError("metadata reference requires name, kind and path")
        if any(not isinstance(ref[k], str) or not ref[k] or len(ref[k].encode()) > 4096 for k in ("name", "path")) or ref["name"] in names:
            raise ValueError("metadata reference names and paths must be bounded and unique")
        names.add(ref["name"])
    return result
