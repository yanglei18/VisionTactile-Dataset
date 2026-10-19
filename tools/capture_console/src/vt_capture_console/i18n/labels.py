"""Display names for fixed built-ins; unknown IDs and user labels stay opaque."""
from . import LocalizedText, tr

_TRACKER_LABELS = {"left_wrist": "左腕 Tracker", "right_wrist": "右腕 Tracker", "torso": "躯干 Tracker"}


def tracker_role(role):
    return {"left_wrist": tr('左腕'), "right_wrist": tr('右腕'),
            "torso": tr('躯干')}.get(role, role)


def builtin_label(label):
    """Call only after verifying a built-in identity and its exact original label."""
    return {
        "Tracker · 左腕": tr('追踪器 · 左腕'), "Tracker · 右腕": tr('追踪器 · 右腕'),
        "Tracker · 躯干": tr('追踪器 · 躯干'), "Tracker · 演示": tr('追踪器 · 演示'),
        "左腕 Tracker": tr('左腕追踪器'), "右腕 Tracker": tr('右腕追踪器'),
        "躯干 Tracker": tr('躯干追踪器'), "三 Tracker ROS 读取": tr('三追踪器数据读取'),
    }.get(label, tr(label))


def device_label(device, *, demo=False):
    if demo:
        from .demo import demo_device_label
        return demo_device_label(device)
    label = device.get("label", device.get("id", ""))
    identifier, kind = device.get("id"), device.get("kind")
    if kind == "tracker" and identifier in _TRACKER_LABELS and label == _TRACKER_LABELS[identifier]:
        return builtin_label(label)
    if kind == "camera":
        for camera in ("d405_1", "d405_2", "d436"):
            for stream, original, shown in (("color", "RGB", tr('彩色')), ("depth", "深度", tr('深度'))):
                if identifier == f"{camera}_{stream}" and label == f"{camera.upper()} · {original}":
                    return tr('{v0} · {v1}', v0=camera.upper(), v1=shown)
    return label


def scene_entity_label(entity, *, short=False):
    label = entity.get("label", entity.get("id", ""))
    role = entity.get("role")
    original = _TRACKER_LABELS.get(role)
    if original is not None and entity.get("id") == role:
        if label == original or (isinstance(label, LocalizedText)
                                 and str(label) == str(builtin_label(original))):
            return tracker_role(role) if short else builtin_label(original)
    return label
