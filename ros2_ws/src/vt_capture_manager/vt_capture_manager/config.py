"""Explicit subscription configuration; no discovery starts devices."""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import re


@dataclass(frozen=True)
class DeviceConfig:
    id: str
    label: str
    kind: str
    side: str
    topic: str
    # Preserve compatibility with explicit subscriptions to sensor-data sources.
    image_reliability: str = "best_effort"


def default_devices():
    devices = [DeviceConfig(f"{camera}_{stream}", f"{camera.upper()} · {'RGB' if stream == 'color' else '深度'}", "camera", "",
                            f"/{camera}/{suffix}", image_reliability="reliable")
               for camera in ("d405_1", "d405_2", "d436")
               for stream, suffix in (("color", "color/image_raw"), ("depth", "depth/image_rect_raw"))]
    devices += [DeviceConfig(role, {"left_wrist": "左腕 Tracker", "right_wrist": "右腕 Tracker", "torso": "躯干 Tracker"}[role], "tracker", role.split("_")[0] if "_" in role else "",
                             f"/vive/{role}/sample")
                for role in ("left_wrist", "right_wrist", "torso")]
    for hand in ("left", "right"):
        label = "左手" if hand == "left" else "右手"
        devices.extend((DeviceConfig(f"{hand}_glove", f"{label}姿态", "glove", hand, f"/gloves/{hand}/state"),
                        DeviceConfig(f"{hand}_tactile", f"{label}触觉", "tactile", hand,
                                     f"/gloves/{hand}/tactile/state")))
    return devices


@dataclass(frozen=True)
class ManagerConfig:
    devices: tuple[DeviceConfig, ...]
    capture_status_topic: str = "/capture/status"
    stale_after_s: float = 1.0
    offline_after_s: float = 5.0
    disk_path: str = "."
    quality_enabled: bool = True
    quality_timeout_s: float = 900.0
    quality_alignment_config: str = ""
    metadata_refs: tuple = ()
    process_groups: tuple = ()
    scene_config_path: str = ""


def load_config(path=None):
    raw = {}
    if path:
        text = Path(path).read_text(encoding="utf-8")
        if str(path).endswith(".json"):
            raw = json.loads(text)
        else:
            import yaml
            raw = yaml.safe_load(text)
        if not isinstance(raw, dict):
            raise ValueError("manager config must be a mapping")
    allowed = {"devices", "capture_status_topic", "stale_after_s", "offline_after_s", "disk_path",
               "quality_enabled", "quality_timeout_s", "quality_alignment_config", "metadata_refs", "process_groups", "scene_config_path"}
    if set(raw) - allowed:
        raise ValueError("unknown manager configuration keys")
    devices = tuple(DeviceConfig(**value) for value in raw.get("devices", [vars(d) for d in default_devices()]))
    if len(devices) > 64 or len({d.id for d in devices}) != len(devices):
        raise ValueError("device ids must be unique; at most 64 subscriptions")
    for d in devices:
        if not all(isinstance(v, str) and len(v) <= 512 for v in vars(d).values()):
            raise ValueError("device configuration fields must be bounded strings")
        if not d.id or not d.label or d.kind not in {"camera", "tracker", "glove", "tactile"}:
            raise ValueError("invalid device identity/kind")
        if d.side not in {"", "left", "right"} or (d.kind in {"glove", "tactile"} and not d.side):
            raise ValueError("glove/tactile require an explicit side")
        if d.image_reliability not in {"best_effort", "reliable"}:
            raise ValueError("image_reliability must be best_effort or reliable")
        if d.kind != "camera" and d.image_reliability != "best_effort":
            raise ValueError("image_reliability only applies to camera subscriptions")
        _topic(d.topic)
    refs = raw.get("metadata_refs", [])
    if not isinstance(refs, list) or len(refs) > 16:
        raise ValueError("metadata_refs must contain at most 16 explicit file references")
    names = set()
    for ref in refs:
        if not isinstance(ref, dict) or set(ref) != {"name", "kind", "path"}:
            raise ValueError("each metadata reference requires name, kind, path")
        if ref["kind"] not in {"config", "calibration"}:
            raise ValueError("metadata references only allow config or calibration files")
        if any(not isinstance(ref[key], str) or not ref[key] or len(ref[key].encode()) > 4096 for key in ("name", "path")):
            raise ValueError("metadata reference names and paths must be nonempty bounded strings")
        if ref["name"] in names:
            raise ValueError("metadata reference names must be unique")
        names.add(ref["name"])
    from .launch_profiles import parse_profiles
    result = ManagerConfig(devices=devices, metadata_refs=tuple(dict(ref) for ref in refs),
                           process_groups=parse_profiles(raw.get("process_groups", [])),
                           **{k: v for k, v in raw.items() if k not in {"devices", "metadata_refs", "process_groups"}})
    _topic(result.capture_status_topic)
    if not (0.05 <= result.stale_after_s < result.offline_after_s <= 3600):
        raise ValueError("require 0.05 <= stale_after_s < offline_after_s <= 3600")
    if not isinstance(result.disk_path, str):
        raise ValueError("disk_path must be a string")
    if type(result.quality_enabled) is not bool or not 1 <= result.quality_timeout_s <= 86400:
        raise ValueError("quality_enabled must be boolean; quality_timeout_s must be between 1 and 86400")
    if not isinstance(result.quality_alignment_config, str) or len(result.quality_alignment_config.encode()) > 4096:
        raise ValueError("quality_alignment_config must be a bounded file path")
    if not isinstance(result.scene_config_path, str) or len(result.scene_config_path.encode()) > 4096:
        raise ValueError("scene_config_path must be a bounded file path")
    return result


def _topic(value):
    if not isinstance(value, str) or not re.fullmatch(r"(?:/[A-Za-z_][A-Za-z0-9_]*)+", value):
        raise ValueError("topics must be absolute ROS topic names")
