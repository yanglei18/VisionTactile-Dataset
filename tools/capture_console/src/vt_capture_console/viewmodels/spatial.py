"""Immutable spatial presentation state; no Qt, ROS or device operations."""
from __future__ import annotations

from ..i18n import tr
from ..i18n.labels import scene_entity_label

from collections.abc import Mapping
import math
from types import MappingProxyType


TRACKER_COLORS = {"left_wrist": "#16a8b7", "right_wrist": "#ce5c9c", "torso": "#d88b28"}
HAND_COLORS = {"left": "#54d8d0", "right": "#ffae70"}
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
    if entity.get("hand") in HAND_COLORS:
        return HAND_COLORS[entity["hand"]]
    return TRACKER_COLORS.get(entity.get("role"), CAMERA_COLORS.get(entity.get("id"), "#6286ad"))


def drawable(entity):
    if "hand" in entity and (entity.get("valid") is not True or entity.get("state") != "fresh"):
        return False
    position, rotation = entity.get("position_m"), entity.get("quaternion_xyzw")
    try:
        return (position is not None and len(position) == 3 and rotation is not None and len(rotation) == 4
                and all(math.isfinite(float(v)) for v in (*position, *rotation))
                and sum(float(v) ** 2 for v in rotation) > 1e-12
                and entity.get("registration_state", "ready") == "ready"
                and entity.get("state") not in {"missing", "identity_unverified", "identity_mismatch", "frame_mismatch"})
    except (TypeError, ValueError):
        return False


def entity_state_label(entity):
    if "hand" in entity:
        if drawable(entity):
            return tr('21 点手形') if entity.get("joint_positions_m") else tr('仅腕部')
        if entity.get("state") == "waiting":
            return tr('等待手部定位')
        if entity.get("state") == "offline":
            return tr('手部定位已中断')
        return tr('手部定位不可用')
    return STATE_LABELS.get(entity.get("state"), str(entity.get("state", "")))


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
        self.layers = {"tracker", "camera", "hand", "axes", "grid", "trail", "frustum"}
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
            if self._latest and any(hand.get("valid") for hand in self._latest.get("hands", ())):
                self._latest = MappingProxyType({**self._latest, "hands": self._offline_hands(self._latest)})
            return
        if self._latest and self._latest.get("manager_instance_id") != frame.get("manager_instance_id"):
            if not self.frozen:
                self.selected_id = None
        clean = dict(frame)
        seen, dropped = set(), 0
        for kind in ("trackers", "cameras", "hands"):
            if kind == "hands" and kind not in frame:
                continue
            entities = []
            for entity in frame.get(kind, ()):
                identifier = entity.get("id") if isinstance(entity, Mapping) else None
                if kind == "hands":
                    hand = entity.get("hand") if isinstance(entity, Mapping) else None
                    identifier = ("hand/" + hand if hand in ("left", "right")
                                  and identifier in (hand, "hand/" + hand) else None)
                if not isinstance(identifier, str) or not identifier or identifier in seen:
                    dropped += 1
                    continue
                seen.add(identifier)
                entity = dict(entity, id=identifier)
                if kind == "hands":
                    entity["label"] = tr('左手') if entity["hand"] == "left" else tr('右手')
                    if not drawable(entity):
                        entity.update(position_m=None, quaternion_xyzw=None, joint_positions_m=(), valid=False)
                    source_ns, generated_ns = entity.get("source_realtime_ns"), frame.get("generated_realtime_ns")
                    if type(source_ns) is int and type(generated_ns) is int and 0 < source_ns <= generated_ns:
                        entity["age_s"] = (generated_ns - source_ns) / 1e9
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
            if self._held and not self.source_available and "hands" in self._held:
                self._held = freeze({**self._held, "hands": self._hands(self._held)})
        if not frozen:
            self._held = None
        self.frozen = bool(frozen)

    def set_preset(self, preset):
        if preset not in {"tracker", "camera", "combined"}:
            raise ValueError("unknown spatial preset")
        self.preset = preset

    @staticmethod
    def _offline_hands(frame):
        return tuple(freeze({**hand, "state": "offline", "valid": False, "reason": "offline",
                             "position_m": None, "quaternion_xyzw": None, "joint_positions_m": ()})
                     for hand in frame.get("hands", ()))

    def _hands(self, frame):
        return frame.get("hands", ()) if self.source_available or self.frozen else self._offline_hands(frame)

    def all_entities(self):
        if not self.frame:
            return ()
        return tuple(self.frame.get("trackers", ())) + tuple(self.frame.get("cameras", ())) + self._hands(self.frame)

    def entities(self):
        frame = self.frame
        if not frame:
            return ()
        result = []
        if self.preset != "camera" and "tracker" in self.layers:
            result.extend(frame.get("trackers", ()))
        if self.preset != "tracker" and "camera" in self.layers:
            result.extend(frame.get("cameras", ()))
        if self.preset == "combined" and "hand" in self.layers:
            result.extend(self._hands(frame))
        return tuple(e for e in result if e.get("id") not in self.hidden_entities and drawable(e))

    def select(self, identifier):
        self.selected_id = identifier

    @property
    def selected(self):
        return next((e for e in self.all_entities() if e.get("id") == self.selected_id), None)
