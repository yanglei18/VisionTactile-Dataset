"""Immutable spatial presentation state; no Qt, ROS or device operations."""
from __future__ import annotations

from ..i18n import tr
from ..i18n.labels import scene_entity_label

from collections.abc import Mapping
import math
from types import MappingProxyType


TRACKER_COLORS = {"left_wrist": "#16a8b7", "right_wrist": "#ce5c9c", "torso": "#d88b28"}
CAMERA_COLORS = {"d405_1": "#377fd2", "d405_2": "#8e65c5", "d436": "#3e987a"}
STATE_LABELS = {"fresh": tr('有效'), "ready": tr('已配准'), "delayed": tr('数据延迟'), "stale": tr('数据陈旧'),
                "waiting": tr('等待数据'), "invalid": tr('位姿无效'), "offline": tr('已断流'),
                "missing": tr('缺少标定'), "identity_unverified": tr('设备身份未核验'),
                "identity_mismatch": tr('设备身份不匹配'), "frame_mismatch": tr('坐标系不匹配')}
STATE_LABELS.update(missing_calibration=tr('缺少有效外参'), tracker_mismatch=tr('绑定追踪器身份不匹配'),
                    frustum_requires_rectification=tr('成像模型需要校正后才能绘制视锥'), missing_intrinsics=tr('缺少相机内参'),
                    no_before=tr('缺少前侧位姿样本'), no_after=tr('等待后侧位姿样本'), gap_too_large=tr('位姿插值间隙过大'),
                    unavailable=tr('尚不可用'), unsupported=tr('暂不支持该几何路径'))


def freeze(value):
    if isinstance(value, Mapping):
        return MappingProxyType({key: freeze(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(freeze(item) for item in value)
    return value


def entity_color(entity):
    return TRACKER_COLORS.get(entity.get("role"), CAMERA_COLORS.get(entity.get("id"), "#6286ad"))


def drawable(entity):
    position, rotation = entity.get("position_m"), entity.get("quaternion_xyzw")
    try:
        return (position is not None and len(position) == 3 and rotation is not None and len(rotation) == 4
                and all(math.isfinite(float(v)) for v in (*position, *rotation))
                and sum(float(v) ** 2 for v in rotation) > 1e-12
                and entity.get("registration_state", "ready") == "ready"
                and entity.get("state") not in {"missing", "identity_unverified", "identity_mismatch", "frame_mismatch"})
    except (TypeError, ValueError):
        return False


def rotate(point, quaternion):
    x, y, z, w = quaternion
    length = math.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/length, y/length, z/length, w/length
    px, py, pz = point
    tx, ty, tz = 2*(y*pz-z*py), 2*(z*px-x*pz), 2*(x*py-y*px)
    return (px+w*tx+y*tz-z*ty, py+w*ty+z*tx-x*tz, pz+w*tz+x*ty-y*tx)


def transform(point, position=(0., 0., 0.), quaternion=(0., 0., 0., 1.)):
    return tuple(a+b for a, b in zip(rotate(point, quaternion), position))


def display_point(frame, point):
    display = frame.get("display_from_world") or {}
    return transform(point, display.get("translation_m", (0, 0, 0)),
                     display.get("quaternion_xyzw", (0, 0, 0, 1)))


class SpatialViewModel:
    def __init__(self):
        self._latest = self._held = None
        self.frozen = False
        self.preset = "combined"
        self.layers = {"tracker", "camera", "axes", "grid", "trail", "frustum"}
        self.hidden_entities = set()
        self.selected_id = None
        self.source_available = True

    @property
    def frame(self):
        return self._held if self.frozen else self._latest

    def clear_frame(self):
        """Release live and frozen samples while preserving view preferences."""
        self._latest = self._held = None

    def update(self, frame):
        if frame is None:
            return
        if self._latest and self._latest.get("manager_instance_id") != frame.get("manager_instance_id"):
            if not self.frozen:
                self.selected_id = None
        clean = dict(frame)
        seen, dropped = set(), 0
        for kind in ("trackers", "cameras"):
            entities = []
            for entity in frame.get(kind, ()):
                identifier = entity.get("id") if isinstance(entity, Mapping) else None
                if not isinstance(identifier, str) or not identifier or identifier in seen:
                    dropped += 1
                    continue
                seen.add(identifier)
                entity = dict(entity)
                if not isinstance(entity.get("label"), str):
                    entity["label"] = identifier
                entity["label"] = scene_entity_label(entity)
                entity.setdefault("state", "waiting")
                if "registration_state" in entity and not isinstance(entity["registration_state"], str):
                    entity["registration_state"] = "unavailable"
                if "reason" in entity and not isinstance(entity["reason"], str):
                    entity["reason"] = str(entity["reason"])[:512]
                entities.append(entity)
            clean[kind] = entities
        if dropped:
            clean["diagnostics"] = list(frame.get("diagnostics", ()))[:127]+[tr('{v0} 个无效实体未显示', v0=dropped)]
        self._latest = freeze(clean)

    def set_frozen(self, frozen):
        if frozen and not self.frozen:
            self._held = self._latest
        if not frozen:
            self._held = None
        self.frozen = bool(frozen)

    def set_preset(self, preset):
        if preset not in {"tracker", "camera", "combined"}:
            raise ValueError("unknown spatial preset")
        self.preset = preset

    def all_entities(self):
        if not self.frame:
            return ()
        return tuple(self.frame.get("trackers", ())) + tuple(self.frame.get("cameras", ()))

    def entities(self):
        frame = self.frame
        if not frame:
            return ()
        result = []
        if self.preset != "camera" and "tracker" in self.layers:
            result.extend(frame.get("trackers", ()))
        if self.preset != "tracker" and "camera" in self.layers:
            result.extend(frame.get("cameras", ()))
        return tuple(e for e in result if e.get("id") not in self.hidden_entities and drawable(e))

    def select(self, identifier):
        self.selected_id = identifier

    @property
    def selected(self):
        return next((e for e in self.all_entities() if e.get("id") == self.selected_id), None)
