"""Display labels for the program's fixed synthetic demonstration fixtures.

Device/group helpers are called only from explicit demo UI paths. They still
require the exact authored identity, kind, and original label, so a renamed
entity is never mistaken for a translatable application string. Backend
payloads, metadata, and saved user labels are left untouched.
"""
from __future__ import annotations

from collections.abc import Mapping

from . import tr
from .labels import builtin_label


_DEVICES = {
    "d405_1": ("camera", "", "D405 · 左视角"),
    "d405_2": ("camera", "", "D405 · 右视角"),
    "d436": ("camera", "", "D436 · 主视角"),
    "left_wrist": ("tracker", "", "Tracker · 左腕"),
    "right_wrist": ("tracker", "", "Tracker · 右腕"),
    "torso": ("tracker", "", "Tracker · 躯干"),
    "left_glove": ("glove", "left", "左手 · 姿态"),
    "right_glove": ("glove", "right", "右手 · 姿态"),
    "left_tactile": ("tactile", "left", "左手 · 触觉"),
    "right_tactile": ("tactile", "right", "右手 · 触觉"),
}
_GROUPS = {
    "cameras": ("camera", "三相机 · 演示"),
    "trackers": ("tracker", "Tracker · 演示"),
    "left_glove": ("glove", "左手 姿态 · 演示"),
    "right_glove": ("glove", "右手 姿态 · 演示"),
    "left_tactile": ("tactile", "左手 触觉 · 演示"),
    "right_tactile": ("tactile", "右手 触觉 · 演示"),
}
_TRACKERS = {
    "left_wrist": ("DEMO-1", "左腕 Tracker"),
    "right_wrist": ("DEMO-2", "右腕 Tracker"),
    "torso": ("DEMO-3", "躯干 Tracker"),
}


def demo_device_label(device):
    """Render an unchanged built-in device label in an explicit demo UI."""
    label = device.get("label", device.get("id", ""))
    expected = _DEVICES.get(device.get("id"))
    preview = device.get("preview")
    if (expected is not None
            and expected == (device.get("kind"), device.get("side", ""), label)
            and isinstance(preview, Mapping) and preview.get("demo") is True):
        return builtin_label(label)
    return label


def demo_group_label(group):
    """Render an unchanged built-in process group in an explicit demo UI."""
    label = group.get("label", group.get("id", ""))
    expected = _GROUPS.get(group.get("id"))
    if expected is not None and expected == (group.get("kind"), label):
        return builtin_label(label)
    return label


def demo_scene_frame(frame):
    """Return a display copy with only verified demo Tracker labels marked.

    Copy only the frame and changed Tracker records; sensor values stay shared
    and are not written here. Live/replay frames and user-renamed labels pass
    through. SpatialViewModel subsequently freezes its own presentation copy.
    """
    if (not isinstance(frame, Mapping) or frame.get("mode") != "demo"
            or frame.get("manager_instance_id") != "demo-manager"):
        return frame
    changed = False
    trackers = []
    for tracker in frame.get("trackers", ()):
        replacement = tracker
        if isinstance(tracker, Mapping):
            identifier = tracker.get("id")
            expected = _TRACKERS.get(identifier)
            if (expected is not None and tracker.get("role") == identifier
                    and expected == (tracker.get("tracker_id"), tracker.get("label"))):
                replacement = {**tracker, "label": builtin_label(tracker["label"])}
                changed = True
        trackers.append(replacement)
    return {**frame, "trackers": trackers} if changed else frame
