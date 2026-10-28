"""Bounded, read-only previews. No ROS or acquisition imports.

OpenGL loads lazily for enlarged camera images and selected 3D hand views.

The console supplies snapshots through ``set_device``. This widget does not own
a timer or a sensor, and never extrapolates live measurements. Synthetic scenes
are opt-in and visibly labelled; a held picture is not a new measurement.
"""

from __future__ import annotations

from ...i18n import tr

import base64
import binascii
from collections import deque
from collections.abc import Mapping
from copy import deepcopy
import math
import os

from PySide6 import QtCore, QtGui
from ...i18n import widgets as QtWidgets
from ...i18n.widgets import QPainter as LocalizedPainter
from ...viewmodels.glove_sources import source_summary

from .tactile_hand import right_hand_geometry
from .tactile_palette import RAW_HEAT_STOPS, heat_stops


MAX_BASE64_BYTES = 8_000_000
MAX_IMAGE_DIMENSION = 4096
MAX_IMAGE_PIXELS = 8_000_000
MAX_TRAIL_POINTS = 80

_KINDS = {"camera", "tracker", "glove", "tactile"}
_ONLINE = {"online", "live", "streaming", "connected", "ok", "ready", "active"}
_MUTED = QtGui.QColor("#A0A0A0")
_INK = QtGui.QColor("#F4F4F4")
_ACCENT = QtGui.QColor("#FFFFFF")
_INVALID = QtGui.QColor("#313131")
_HEAT_STOPS = RAW_HEAT_STOPS
_BONES = tuple((0 if offset == 0 else base + offset - 1, base + offset)
               for base in (1, 5, 9, 13, 17) for offset in range(4))
_HEADERS = {"camera": tr('相机画面'), "tracker": tr('空间位姿'),
            "glove": tr('手部骨架'), "tactile": tr('20 × 20 触觉')}
_EMPTY = {"camera": tr('等待相机画面'), "tracker": tr('等待追踪器位姿'),
          "glove": tr('等待手部关节点'), "tactile": tr('等待触觉数据')}


def _finite(value):
    return (not isinstance(value, bool) and isinstance(value, (float, int))
            and math.isfinite(value))


def _vector(value, length):
    if (not isinstance(value, (tuple, list)) or len(value) != length
            or not all(_finite(v) and abs(v) <= 1e12 for v in value)):
        raise ValueError(tr('无效的位姿数组'))
    return tuple(float(v) for v in value)


def _color(value, lo, hi, stops=_HEAT_STOPS):
    fraction = min(1., max(0., (value - lo) / (hi - lo)))
    for (start, low), (end, high) in zip(stops, stops[1:]):
        if fraction <= end:
            t = (fraction - start) / (end - start)
            return QtGui.QColor(*(round(a + (b - a) * t) for a, b in zip(low, high)))
    return QtGui.QColor(*stops[-1][1])


def _demo_joints(phase, side):
    """An illustrative skeleton, never a solver result or MANO reconstruction."""
    points = [(0., 0., 0.)]
    fingers = [(-.044, .025, -.68, .73, .061), (-.028, .052, -.05, 1., .072),
               (-.008, .057, 0., 1., .083), (.013, .054, .03, 1., .076),
               (.033, .043, .12, .99, .061)]
    for finger, (x, y, dx, dy, length) in enumerate(fingers):
        bend = .35 + .24 * math.sin(phase + finger * .32)
        for index in range(4):
            progress = index / 3
            angle = bend * progress
            distance = length * progress
            points.append((x + dx * distance, y + dy * distance * math.cos(angle),
                           distance * math.sin(angle)))
    if side == "left":
        points = [(-x, y, z) for x, y, z in points]
    return tuple(points)


class PreviewWidget(QtWidgets.QWidget):
    """Paint the most recent usable device preview without ever opening hardware.

    ``kind`` is camera/tracker/glove/tactile. Empty or malformed incoming
    previews retain the last good frame and label it as held. The 21-joint order
    is wrist, then four joints each for thumb/index/middle/ring/little, matching
    the glove export (already root-rotated; no root rotation is applied here).
    Drag a tracker or skeleton preview to inspect its orthographic view;
    double-click resets it.
    """

    tactile_view_changed = QtCore.Signal(str)
    glove_view_changed = QtCore.Signal(str)
    glove_display_changed = QtCore.Signal()

    def __init__(self, kind: str, parent=None):
        super().__init__(parent)
        if kind not in _KINDS:
            raise ValueError("kind must be camera, tracker, glove, or tactile")
        self.kind = kind
        self._presentation = "standard"
        self._stage_intro = False
        self._motion_enabled = True
        self._decoration_phase = 0.
        self._device = {}
        self._identity = None
        self._frame = None
        self._image = None
        self._encoded_image = None
        self._camera_gpu = None
        self._camera_gpu_unavailable = False
        self._mode = "live"
        self._held = False
        self._preview_error = ""
        self._levels = None
        self._cell_colors = ()
        self._requested_tactile_view = "hand"
        self._tactile_view = "matrix"
        self._tactile_geometry = None
        self._tactile_layout_side = object()
        self._tactile_layout_note = ""
        self._tactile_3d = None
        self._glove_3d = None
        self._glove_view = "skeleton"
        self._trail = deque(maxlen=MAX_TRAIL_POINTS)
        self._tracker_origin = None
        self._tracker_span = .25
        self._glove_span = None
        self._yaw, self._pitch = -.24, .32
        self._zoom = 1.
        self._drag = None
        self.setMinimumSize(150, 135)
        self.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Expanding)
        self.setMouseTracking(kind == "tactile")
        self.setAccessibleName(_HEADERS[kind])
        if kind in ("tracker", "glove"):
            self.setCursor(QtCore.Qt.CursorShape.OpenHandCursor)
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_OpaquePaintEvent, False)
        self._sync_tactile_view()

    @property
    def device_snapshot(self) -> dict:
        """Return an isolated snapshot, including nested mutable metadata.

        Immutable JPEG strings remain shared; callers cannot change this
        widget's input or validated cache by editing the returned mapping.
        """
        return deepcopy(self._device)

    @property
    def identity(self):
        """The selected device ID and hand, or None before the first input."""
        return self._identity

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def held(self) -> bool:
        return self._held

    @property
    def preview_error(self) -> str:
        return self._preview_error

    @property
    def has_frame(self) -> bool:
        return self._frame is not None

    @property
    def requested_tactile_view(self) -> str:
        """The user's preference before unsupported-hand fallback is applied."""
        return self._requested_tactile_view

    def refresh_metadata(self, device: Mapping):
        """Refresh labels, rates and ages without decoding or changing a frame.

        The complete input snapshot must retain the same identity, status,
        mode and preview payload. Use ``set_device`` for a new sample or a
        connection/context change. Validation completes before any mutation.
        """
        if (not isinstance(device, Mapping) or self._identity is None
                or any(device.get(key) != self._device.get(key)
                       for key in ("kind", "id", "side", "status", "mode", "preview"))):
            raise ValueError("metadata refresh requires the same device and preview payload")
        repaint = device.get("recorded_count") != self._device.get("recorded_count")
        self._device = deepcopy(dict(device))
        self.setAccessibleName(str(device.get("label", _HEADERS[self.kind])))
        if repaint:
            self.update()

    def copy_view_state_from(self, source):
        """Seed yaw, pitch and zoom once without copying measurement state.

        An unbound receiver can adopt a matching kind's view. An already bound
        receiver must retain its device identity. Later cache updates preserve
        the receiver's independently adjusted inspection view.
        """
        if not isinstance(source, PreviewWidget) or source.kind != self.kind:
            raise ValueError("view state copying requires matching preview kinds")
        if self._identity is not None and self._identity != source._identity:
            raise ValueError("view state must belong to the same device and side")
        self._yaw, self._pitch, self._zoom = source._yaw, source._pitch, source._zoom
        self.update()

    @property
    def presentation(self):
        return self._presentation

    def set_presentation(self, presentation):
        """Choose chrome density without touching sensor state or view angles."""
        if presentation not in ("standard", "thumbnail", "stage"):
            raise ValueError("preview presentation must be standard, thumbnail or stage")
        if presentation == self._presentation:
            return
        self._presentation = presentation
        width, height = {"standard": (150, 135), "thumbnail": (100, 64), "stage": (150, 120)}[presentation]
        self.setMinimumSize(width, height)
        for renderer in (self._tactile_3d, self._glove_3d):
            if renderer is not None:
                renderer.setGeometry(self._canvas_rect().toRect())
        self.updateGeometry()
        self._refresh_camera_gpu()
        self.update()

    def set_stage_intro(self, visible):
        """Reserve a title area only for synthetic scenes and empty states."""
        visible = bool(visible)
        if visible != self._stage_intro:
            self._stage_intro = visible
            for renderer in (self._tactile_3d, self._glove_3d):
                if renderer is not None:
                    renderer.setGeometry(self._canvas_rect().toRect())
            self.update()

    def set_motion_enabled(self, enabled):
        """Freeze illustrative camera motion; never freeze incoming data."""
        if self._motion_enabled and not enabled and self._mode == "demo" and self._frame:
            self._decoration_phase = self._frame.get("phase", 0.)
        self._motion_enabled = bool(enabled)
        self.update()

    @property
    def tactile_view(self):
        """Effective view; unsupported hands always fall back to raw matrix."""
        return self._tactile_view

    @property
    def tactile_hand_available(self):
        return self._tactile_geometry is not None

    @property
    def tactile_3d_available(self):
        # Missing model/GL is explained inside the selected view, so operators
        # can find its setup instructions instead of facing a dead button.
        return self.tactile_hand_available

    @property
    def tactile_layout_note(self):
        return self._tactile_layout_note

    def _tactile_response(self):
        return self._frame is not None and self._frame.get("value_kind") == "tactile_response"

    def _tactile_palette(self):
        return heat_stops("tactile_response" if self._tactile_response() else "raw_adc")

    @property
    def glove_view(self):
        return self._glove_view

    def set_glove_view(self, view):
        if self.kind != "glove" or view not in ("skeleton", "3d"):
            raise ValueError("glove view must be skeleton or 3d")
        previous = self._glove_view
        self._glove_view = view
        self._refresh_glove_3d()
        if view != previous:
            self.glove_view_changed.emit(view)
        self.update()

    def _refresh_glove_3d(self):
        if self.kind != "glove":
            return
        side = self._device.get("side")
        if self._glove_3d is not None and self._glove_3d.hand != side:
            self._glove_3d.dispose()
            self._glove_3d = None
        active = self.glove_view == "3d" and side in ("right", "left")
        if active and self._glove_3d is None:
            from .glove_3d import Glove3DWidget
            self._glove_3d = Glove3DWidget(side, self)
            self._glove_3d.display_changed.connect(self._notify_glove_display_changed)
        if self._glove_3d is not None:
            if active:
                self._glove_3d.setGeometry(self._canvas_rect().toRect())
                # Update the target before showEvent can submit a hidden view's
                # previous sample to the shared reconstruction worker.
                self._glove_3d.set_sample(self._frame, held=self._held, mode=self._mode,
                                           device=self._device)
            self._glove_3d.setVisible(active)
        self._notify_glove_display_changed()

    def _notify_glove_display_changed(self):
        self.setToolTip(self.glove_source_summary)
        self.glove_display_changed.emit()

    @property
    def glove_source_summary(self):
        """Describe the selected view's displayed geometry, including history."""
        if self.kind != "glove" or self._mode == "demo":
            return ""
        if self.glove_view == "3d":
            frame = self._glove_3d.displayed_frame if self._glove_3d is not None else None
        else:
            frame = self._frame if self._frame and self._frame.get("relative") else None
        return source_summary(frame) if frame else ""

    def copy_glove_state_from(self, source):
        if self.kind != "glove" or source.kind != "glove":
            raise ValueError("glove state copying requires glove widgets")
        if self._identity is not None and self._identity != source._identity:
            raise ValueError("glove display must belong to the same device and hand")
        if self._mode != source._mode or source._frame is None:
            self._reset_frame()
        self._identity = source._identity
        self._device = source.device_snapshot
        self._frame = dict(source._frame) if source._frame is not None else None
        self._glove_span = source._glove_span
        self._mode, self._held = source._mode, source._held
        self._preview_error = source._preview_error
        self._refresh_glove_3d()
        self.update()

    def set_tactile_view(self, view):
        if self.kind != "tactile" or view not in ("hand", "matrix", "3d"):
            raise ValueError("tactile view must be hand, matrix or 3d on a tactile widget")
        self._requested_tactile_view = view
        self.setToolTip("")
        self._sync_tactile_view()
        self._refresh_tactile_3d()
        self.update()

    def _sync_tactile_view(self):
        if self.kind != "tactile":
            return
        before = (self._tactile_view, self.tactile_hand_available, self._tactile_layout_note)
        side = self._device.get("side")
        if side != self._tactile_layout_side:
            if self._tactile_3d is not None:
                self._tactile_3d.dispose()
                self._tactile_3d = None
            self._tactile_layout_side = side
            self._tactile_geometry = None
            self.setToolTip("")
            if side == "right":
                try:
                    self._tactile_geometry = right_hand_geometry()
                except (ImportError, OSError, ValueError) as exc:
                    self._tactile_layout_note = tr('右手布局不可用 · 仅原始矩阵；未映射：{v0}', v0=exc)
                else:
                    count = self._tactile_geometry.anchor_count
                    self._tactile_layout_note = (
                        tr('右手候选手形 · {v0} 个已观测锚点，其余位置未逐点验证；原始 ADC 不变，不代表压力。', v0=count) + self._tactile_geometry.layout.note)
            else:
                label = tr('左手') if side == "left" else tr('未指定左右手')
                self._tactile_layout_note = tr('{v0}未映射 · 仅原始矩阵；不镜像或推断触点的解剖位置。', v0=label)
        self._tactile_view = self._requested_tactile_view if self.tactile_hand_available else "matrix"
        self.setAccessibleDescription(self._tactile_layout_note)
        after = (self._tactile_view, self.tactile_hand_available, self._tactile_layout_note)
        if after != before:
            self.tactile_view_changed.emit(self._tactile_view)

    def _refresh_tactile_3d(self):
        if self.kind != "tactile":
            return
        active = self.tactile_view == "3d"
        if active and self._tactile_3d is None:
            from .tactile_3d import Tactile3DWidget
            self._tactile_3d = Tactile3DWidget(self._device.get("side"), self)
        if self._tactile_3d is not None:
            self._tactile_3d.setVisible(active)
            if active:
                self._tactile_3d.setGeometry(self._canvas_rect().toRect())
                self._tactile_3d.set_frame(self._frame, self._levels)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self._tactile_3d is not None:
            self._tactile_3d.setGeometry(self._canvas_rect().toRect())
        if self._glove_3d is not None:
            self._glove_3d.setGeometry(self._canvas_rect().toRect())
        self._refresh_camera_gpu()

    def showEvent(self, event):
        super().showEvent(event)
        self._refresh_camera_gpu()

    def _refresh_camera_gpu(self):
        eligible = (self.kind == "camera" and self._presentation != "thumbnail"
                    and self._mode != "demo" and self._image is not None
                    and self.isVisible() and self._canvas_rect().width() >= 480
                    and os.environ.get("VT_CAMERA_RENDERER") != "raster"
                    and QtWidgets.QApplication.platformName() not in ("offscreen", "minimal"))
        if eligible and self._camera_gpu is None and not self._camera_gpu_unavailable:
            try:
                from .camera_gl import create_camera_canvas
                self._camera_gpu = create_camera_canvas(self)
            except (ImportError, RuntimeError):
                self._camera_gpu = None
            if self._camera_gpu is None:
                self._camera_gpu_unavailable = True
            else:
                # Context/FBO failures must not delete a surface inside paintGL.
                self._camera_gpu.failed.connect(self._camera_gpu_failed, QtCore.Qt.ConnectionType.QueuedConnection)
        if self._camera_gpu is not None:
            if eligible:
                self._camera_gpu.setGeometry(self._canvas_rect().toRect())
                self._camera_gpu.set_frame(self._image, self._held)
            self._camera_gpu.setVisible(eligible)

    def _camera_gpu_failed(self):
        canvas, self._camera_gpu = self._camera_gpu, None
        self._camera_gpu_unavailable = True
        if canvas is not None:
            canvas.hide()
            canvas.deleteLater()
        self.update()

    def _using_camera_gpu(self):
        return (self._camera_gpu is not None and self._camera_gpu.available
                and self._camera_gpu.isVisible())

    def copy_display_state_from(self, source):
        """Seed an enlarged tactile view with already validated, read-only state.

        A held/offline frame stays held: this never fabricates an online update.
        The receiver keeps its own requested hand/matrix display preference.
        """
        if (self.kind != "tactile" or not isinstance(source, PreviewWidget)
                or source.kind != "tactile"):
            raise ValueError("display state copying requires two tactile widgets")
        if self._identity is not None and self._identity != source._identity:
            raise ValueError("display state must belong to the same tactile device and side")
        self._identity = source._identity
        self._device = source.device_snapshot
        self._frame = dict(source._frame) if source._frame is not None else None
        self._cell_colors = source._cell_colors
        self._levels = source._levels
        self._mode, self._held = source._mode, source._held
        self._preview_error = source._preview_error
        self.setAccessibleName(source.accessibleName())
        self.setToolTip("")
        self._sync_tactile_view()
        self._refresh_tactile_3d()
        self.update()

    def copy_camera_tracker_state_from(self, source):
        """Copy a camera/Tracker's validated display cache, without new input.

        QImage's implicit sharing avoids a second JPEG decode or pixel copy;
        each widget still owns its wrapper. Tracker history is an independent
        bounded deque of the same measured points, never a new trajectory.
        View-only yaw/pitch/zoom remain those selected in this receiver.
        """
        if (self.kind not in ("camera", "tracker") or not isinstance(source, PreviewWidget)
                or source.kind != self.kind):
            raise ValueError("camera/Tracker display copying requires matching preview kinds")
        if self._identity is not None and self._identity != source._identity:
            raise ValueError("camera/Tracker display must belong to the same device and side")
        if self._camera_tracker_display_key() == source._camera_tracker_display_key():
            # An age/rate update is not a new picture. Keep cache comparison
            # private so consumers do not depend on its rendering internals.
            self._device = source.device_snapshot
            return
        self._identity = source._identity
        self._device = source.device_snapshot
        self._frame = dict(source._frame) if source._frame is not None else None
        self._image = QtGui.QImage(source._image) if source._image is not None else None
        self._encoded_image = source._encoded_image
        self._mode, self._held = source._mode, source._held
        self._preview_error = source._preview_error
        self._trail = deque(source._trail, maxlen=MAX_TRAIL_POINTS)
        self._tracker_origin, self._tracker_span = source._tracker_origin, source._tracker_span
        self.setAccessibleName(source.accessibleName())
        self.setAccessibleDescription(source.accessibleDescription())
        self._refresh_camera_gpu()
        self.update()

    def _camera_tracker_display_key(self):
        frame = self._frame
        # These validated values are small; never compare encoded JPEG data.
        frame_key = (None if frame is None else
                     tuple(frame.get(key) for key in ("demo", "phase", "width", "height"))
                     if self.kind == "camera" else
                     (tuple(frame["position"]), tuple(frame["orientation"])))
        return (self._identity, frame_key,
                self._image.cacheKey() if self._image is not None else None,
                self._mode, self._held, self._preview_error,
                tuple(self._trail), self._tracker_origin, self._tracker_span,
                self._device.get("recorded_count"),
                self.accessibleName(), self.accessibleDescription())

    def clear_frame(self):
        """Discard the display cache and refresh all renderers immediately.

        Device identity, live/replay context and view preferences remain set.
        This does not fetch data, alter the source snapshot or stop a sensor.
        """
        self._reset_frame()
        self._held = False
        self._preview_error = ""
        self.setToolTip("")
        if self._camera_gpu is not None:
            self._camera_gpu.set_frame(QtGui.QImage(), False)
        if self._tactile_3d is not None:
            self._tactile_3d.set_frame(None, None)
        self._refresh_camera_gpu()
        self._refresh_tactile_3d()
        self._refresh_glove_3d()
        self.update()

    def _reset_frame(self):
        self._frame = self._image = self._encoded_image = None
        if self._glove_3d is not None:
            self._glove_3d.clear_sample()
        self._levels = None
        self._cell_colors = ()
        self._trail.clear()
        self._tracker_origin = None
        self._tracker_span = .25
        self._glove_span = None

    def set_device(self, device: dict):
        self._set_device(device)
        self._refresh_camera_gpu()
        self._refresh_tactile_3d()
        self._refresh_glove_3d()

    def _set_device(self, device: dict):
        if not isinstance(device, Mapping):
            self._preview_error = tr('预览数据格式无效')
            self._held = self._frame is not None
            self.update()
            return
        supplied_kind = device.get("kind", self.kind)
        if supplied_kind != self.kind:
            self._preview_error = tr('设备类型与预览不匹配')
            self._held = self._frame is not None
            self.update()
            return
        identity = (device.get("id", self._device.get("id")), device.get("side", self._device.get("side")))
        if self._identity is not None and identity != self._identity:
            self._reset_frame()
        self._identity = identity
        self._device = deepcopy(dict(device))
        self._sync_tactile_view()
        self.setAccessibleName(str(device.get("label", _HEADERS[self.kind])))
        preview = device.get("preview")
        self._preview_error = ""
        if not isinstance(preview, Mapping) or not preview:
            # Empty archive samples still have a historical context. A mode
            # transition must not retain a frame from live capture or demo.
            mode = device.get("mode")
            if mode in ("live", "demo", "replay") and mode != self._mode:
                self._reset_frame()
                self._mode = mode
            self._held = self._frame is not None
            self.update()
            return
        mode = "demo" if preview.get("demo") is True else (
            "replay" if preview.get("replay") is True or preview.get("mode") == "replay"
            or device.get("mode") == "replay" else "live")
        # A synthetic frame must never survive as an apparently live frame.
        if mode != self._mode:
            self._reset_frame()
        self._mode = mode
        online = str(device.get("status", "unknown")).lower() in _ONLINE
        # Stale/offline updates must not animate even synthetic measurements.
        # Their new payload may describe old or unavailable data, not recovery.
        if not online and (self._frame is not None or mode == "demo"):
            self._held = self._frame is not None
            self.update()
            return
        try:
            frame = self._prepare_demo(preview) if mode == "demo" else self._prepare(preview)
            if frame is not None:
                self._frame = frame
            self._held = frame is None or not online
        except (ValueError, TypeError, OverflowError, binascii.Error) as exc:
            self._preview_error = str(exc) or tr('预览数据无效')
            self._held = self._frame is not None
        self.update()

    def _prepare_demo(self, preview):
        phase = preview.get("phase", 0.)
        if not _finite(phase):
            raise ValueError(tr('演示时间无效'))
        phase = float(phase) % (2 * math.pi)
        if self.kind == "camera":
            return {"demo": True, "phase": phase}
        if self.kind == "glove":
            return self._prepare({"joints": _demo_joints(phase, self._device.get("side")), "valid": True})
        if self.kind == "tracker":
            return self._prepare({"position": [.12 * math.sin(phase), .07 * math.cos(phase), .04 * math.sin(phase * 2)],
                                  "orientation": [0., 0., math.sin(phase / 2), math.cos(phase / 2)], "valid": True})
        values = []
        for row in range(20):
            for col in range(20):
                a = math.exp(-((row - 13 - math.sin(phase) * 3) ** 2 + (col - 6) ** 2) / 13)
                b = math.exp(-((row - 6) ** 2 + (col - 14 - math.cos(phase) * 2) ** 2) / 8)
                values.append(20700 + 4200 * a + 3200 * b + 90 * math.sin(row * .8 + col * .4))
        return self._prepare({"values": values, "valid_mask": [True] * 400, "rows": 20, "cols": 20,
                              "levels": [20000., 25000.]})

    def _prepare(self, preview):
        if self.kind == "camera":
            return self._prepare_image(preview)
        if self.kind == "tracker":
            if preview.get("valid") is not True:
                return None
            position = _vector(preview.get("position"), 3)
            quaternion = _vector(preview.get("orientation"), 4)
            norm = math.hypot(*quaternion)
            if not math.isfinite(norm) or norm < 1e-12:
                raise ValueError(tr('姿态四元数无效'))
            quaternion = tuple(v / norm for v in quaternion)
            if self._tracker_origin is None:
                self._tracker_origin = position
            if not self._trail or self._trail[-1] != position:
                self._trail.append(position)
            self._tracker_span = max(self._tracker_span, *(abs(v - o) * 1.6 for v, o in zip(position, self._tracker_origin)))
            return {"position": position, "orientation": quaternion}
        if self.kind == "glove":
            if preview.get("valid") is not True:
                return None
            from vt_capture_core.glove_sources import glove_sources

            sources = glove_sources(preview.get("glove_sources", {}))
            source_fields = {"glove_sources": sources} if sources else {}
            mano = preview.get("mano")
            if mano is not None:
                # Keep the common 2D path independent of numpy/OpenGL/model packages.
                if not isinstance(mano, Mapping) or mano.get("hand") != self._device.get("side"):
                    raise ValueError(tr('MANO 手别与数据流不一致'))
                mano = dict(mano)
                mano["pose"] = _vector(mano.get("pose"), 48)
                mano["shape"] = _vector(mano.get("shape"), 10)
                if mano.get("source_mode") not in ("live", "replay"):
                    raise ValueError(tr('MANO 数据来源无效'))
                for key in ("session_id", "model_rest_sha256", "config_sha256"):
                    if not isinstance(mano.get(key), str) or len(mano[key]) > 256:
                        raise ValueError(tr('MANO 会话或模型身份无效'))
                for key in ("calibration_version", "frame_index", "source_realtime_ns"):
                    if type(mano.get(key)) is not int or not 0 <= mano[key] < 2**64:
                        raise ValueError(tr('MANO 帧标识无效'))
            raw = preview.get("joints")
            if not raw and mano is not None:
                return {"joints": (), "relative": (), "mano": mano, **source_fields}
            if not isinstance(raw, (tuple, list)) or len(raw) != 21:
                raise ValueError(tr('手骨架需要 21 个关节点'))
            joints = tuple(_vector(point, 3) for point in raw)
            relative = tuple(tuple(v - origin for v, origin in zip(point, joints[0])) for point in joints)
            span = max(math.hypot(*point) for point in relative)
            if not math.isfinite(span) or span < 1e-10:
                raise ValueError(tr('手关节点范围无效'))
            self._glove_span = max(self._glove_span or span, span)
            return {"joints": joints, "relative": relative, **({"mano": mano} if mano else {}), **source_fields}
        rows, cols = preview.get("rows", 20), preview.get("cols", 20)
        if type(rows) is not int or type(cols) is not int or (rows, cols) != (20, 20):
            raise ValueError(tr('触觉预览必须是 20 × 20'))
        raw, mask = preview.get("values"), preview.get("valid_mask")
        if (not isinstance(raw, (tuple, list)) or len(raw) != 400
                or not isinstance(mask, (tuple, list)) or len(mask) != 400
                or any(type(v) is not bool for v in mask)):
            raise ValueError(tr('触觉需要 400 个数值及有效掩码'))
        if any((not _finite(value) or abs(value) > 1e12) and mask[index] for index, value in enumerate(raw)):
            raise ValueError(tr('有效触觉单元包含非有限数值'))
        values = tuple(float(value) if _finite(value) else 0. for value in raw)
        mask = tuple(mask)
        if not any(mask):
            return None
        levels = preview.get("levels")
        value_kind = preview.get("value_kind", "raw_adc")
        palette = heat_stops(value_kind)
        response = value_kind == "tactile_response"
        reference_ready = preview.get("reference_ready")
        if response and (levels is None or type(reference_ready) is not bool
                         or any(v < 0 for v, valid in zip(values, mask) if valid)):
            raise ValueError(tr('触觉响应需要有效参考状态和从零开始的固定色阶'))
        if self._frame is not None and self._frame.get("value_kind", "raw_adc") != value_kind:
            self._levels = None
        if levels is not None:
            lo, hi = _vector(levels, 2)
            if lo >= hi or not math.isfinite(hi - lo) or (response and lo != 0.):
                raise ValueError(tr('触觉色标范围无效'))
            self._levels = (lo, hi)
        if self._levels is None:
            observed = [v for v, valid in zip(values, mask) if valid]
            lo, hi = min(observed), max(observed)
            if hi - lo < 100:
                center = lo / 2 + hi / 2
                lo, hi = center - 50, center + 50
            self._levels = (lo, hi)
        self._cell_colors = tuple(_color(v, *self._levels, palette) if valid else _INVALID
                                  for v, valid in zip(values, mask))
        result = {"values": values, "valid_mask": mask, "rows": 20, "cols": 20}
        if response:
            result.update(value_kind=value_kind, reference_ready=reference_ready)
        return result

    def _prepare_image(self, preview):
        if str(preview.get("format", "")).lower() not in ("jpeg", "jpg"):
            raise ValueError(tr('仅支持 JPEG 预览'))
        encoded = preview.get("data_b64")
        if not isinstance(encoded, str) or not encoded or len(encoded) > MAX_BASE64_BYTES:
            raise ValueError(tr('图像预览超过大小限制或缺少数据'))
        for name in ("width", "height"):
            if name in preview and (type(preview[name]) is not int or not 0 < preview[name] <= MAX_IMAGE_DIMENSION):
                raise ValueError(tr('图像预览尺寸无效'))
        if encoded == self._encoded_image and self._image is not None:
            image = self._image
        else:
            binary = base64.b64decode(encoded, validate=True)
            if not binary.startswith(b"\xff\xd8"):
                raise ValueError(tr('预览不是 JPEG 图像'))
            buffer = QtCore.QBuffer()
            buffer.setData(QtCore.QByteArray(binary))
            buffer.open(QtCore.QIODevice.OpenModeFlag.ReadOnly)
            reader = QtGui.QImageReader(buffer, b"jpeg")
            size = reader.size()
            if (not size.isValid() or size.width() > MAX_IMAGE_DIMENSION or size.height() > MAX_IMAGE_DIMENSION
                    or size.width() * size.height() > MAX_IMAGE_PIXELS):
                raise ValueError(tr('图像预览超过尺寸限制'))
            image = reader.read()
            if image.isNull():
                raise ValueError(tr('图像预览解码失败'))
        if (preview.get("width", image.width()) != image.width()
                or preview.get("height", image.height()) != image.height()):
            raise ValueError(tr('声明的图像尺寸与内容不匹配'))
        self._image, self._encoded_image = image, encoded
        return {"width": image.width(), "height": image.height()}

    def _canvas_rect(self):
        if self._presentation == "thumbnail":
            return QtCore.QRectF(4., 4., max(1., self.width() - 8.), max(1., self.height() - 24.))
        if self._presentation == "stage":
            # Live/replay frames always retain the full aspect-fit surface.
            intro = self._stage_intro and (self._mode == "demo" or self._frame is None)
            left = self.width() * .4 if intro else 0.
            return QtCore.QRectF(left, 0., max(1., self.width() - left), max(1., self.height() - 27.))
        return QtCore.QRectF(12., 33., max(1., self.width() - 24.), max(1., self.height() - 63.))

    @staticmethod
    def _text(painter, rect, text, *, color=_MUTED, size=9., bold=False,
              alignment=QtCore.Qt.AlignmentFlag.AlignLeft | QtCore.Qt.AlignmentFlag.AlignVCenter):
        painter.setPen(color)
        font = painter.font()
        font.setPointSizeF(size)
        font.setWeight(QtGui.QFont.Weight.DemiBold if bold else QtGui.QFont.Weight.Normal)
        painter.setFont(font)
        elided = painter.fontMetrics().elidedText(str(text), QtCore.Qt.TextElideMode.ElideRight, max(0, int(rect.width())))
        painter.drawText(rect, alignment, elided)

    def paintEvent(self, event):
        painter = LocalizedPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        painter.setRenderHint(QtGui.QPainter.RenderHint.SmoothPixmapTransform)
        outer = QtCore.QRectF(self.rect()).adjusted(.5, .5, -.5, -.5)
        surface = QtGui.QLinearGradient(outer.topLeft(), outer.bottomLeft())
        surface.setColorAt(0., QtGui.QColor("#121212"))
        surface.setColorAt(1., QtGui.QColor("#0C0C0C"))
        if self._presentation != "stage":
            painter.setPen(QtGui.QPen(QtGui.QColor("#313131"), .7))
            painter.setBrush(surface)
            painter.drawRoundedRect(outer, 3., 3.)
        heading = ((tr('静态手形 · 触觉响应') if self._tactile_response() else tr('静态 MANO · ADC'))
                   if self.kind == "tactile" and self.tactile_view == "3d"
                   else tr('MANO 手势 · 解算结果') if self.kind == "glove" and self.glove_view == "3d"
                   else tr('候选手形触觉') if self.kind == "tactile" and self.tactile_view == "hand"
                   else _HEADERS[self.kind])
        if self._presentation == "standard":
            self._text(painter, QtCore.QRectF(14, 8, max(0, self.width() - 120), 17),
                       heading, color=_INK, size=8.6, bold=True)
            self._paint_badge(painter)
        rect = self._canvas_rect()
        painter.save()
        clip = QtGui.QPainterPath()
        clip.addRoundedRect(rect, 2., 2.)
        painter.setClipPath(clip)
        if ((self.kind == "tactile" and self.tactile_view == "3d")
                or (self.kind == "glove" and self.glove_view == "3d" and self._glove_3d is not None)):
            pass  # Child OpenGL canvas; header/history badge and footer stay visible.
        elif self._frame is None:
            self._paint_empty(painter, rect)
        elif self.kind == "camera":
            if self._mode == "demo":
                self._paint_demo_camera(painter, rect)
            elif not self._using_camera_gpu():
                self._paint_image(painter, rect)
        elif self.kind == "tracker":
            self._paint_tracker(painter, rect)
        elif self.kind == "glove":
            self._paint_glove(painter, rect)
        else:
            self._paint_tactile(painter, rect)
        if self._frame is not None and self._held and not self._using_camera_gpu():
            self._paint_history(painter, rect)
        painter.restore()
        self._paint_footer(painter)
        painter.end()

    def paint_history_overlay(self, painter, rect):
        """Paint the shared held-frame marker for a child rendering surface.

        The caller owns the active painter and decides whether a held frame is
        being displayed. This does not change the preview or its cached data.
        """
        self._paint_history(painter, rect)

    def _paint_history(self, painter, rect):
        painter.fillRect(rect, QtGui.QColor(12, 12, 12, 72))
        label = QtCore.QRectF(rect.center().x() - 56, rect.bottom() - 28, 112, 23)
        painter.setPen(QtGui.QPen(QtGui.QColor(173, 173, 173, 100), .6))
        painter.setBrush(QtGui.QColor(23, 23, 23, 240))
        painter.drawRoundedRect(label, 3., 3.)
        self._text(painter, label, tr('历史画面 · 已保持'), color=QtGui.QColor("#B6B6B6"), size=8.,
                   alignment=QtCore.Qt.AlignmentFlag.AlignCenter)

    def _paint_badge(self, painter):
        if self._mode == "demo":
            label, foreground, background = tr('演示'), "#BFBFBF", "#232323"
        elif self._mode == "replay":
            label, foreground, background = tr('回放'), "#C0C0C0", "#272727"
        elif self._held:
            label, foreground, background = tr('保留画面'), "#B6B6B6", "#282828"
        elif self._frame is not None:
            label, foreground, background = tr('实时预览'), "#D1D1D1", "#2A2A2A"
        else:
            label, foreground, background = tr('等待数据'), "#A0A0A0", "#242424"
        font = QtGui.QFont(painter.font())
        font.setPointSizeF(7.7)
        font.setWeight(QtGui.QFont.Weight.DemiBold)
        # The heading reserves this space; fit translated status text inside it.
        measured = QtGui.QFontMetricsF(font).horizontalAdvance(str(label)) + 14
        width = min(98, max(92 if self._mode == "demo" else 72, math.ceil(measured)))
        rect = QtCore.QRectF(max(4, self.width() - width - 12), 7, width, 21)
        painter.setBrush(QtGui.QColor(background))
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.drawRoundedRect(rect, 3., 3.)
        self._text(painter, rect, label, color=QtGui.QColor(foreground), size=7.7, bold=True,
                   alignment=QtCore.Qt.AlignmentFlag.AlignCenter)

    def _paint_empty(self, painter, rect):
        center = rect.center()
        painter.setPen(QtGui.QPen(QtGui.QColor("#646464"), 1.2))
        painter.setBrush(QtGui.QColor("#1F1F1F"))
        icon = QtCore.QRectF(center.x() - 20, center.y() - 28, 40, 31)
        painter.drawRoundedRect(icon, 7, 7)
        painter.drawEllipse(QtCore.QPointF(center.x(), center.y() - 13), 6, 6)
        painter.drawLine(QtCore.QPointF(center.x() + 13, center.y() - 18), QtCore.QPointF(center.x() + 15, center.y() - 18))
        title = (tr('未录制') if self._device.get("recorded_count") == 0 else tr('该时刻无样本')) if self._mode == "replay" else _EMPTY[self.kind]
        self._text(painter, QtCore.QRectF(rect.left(), center.y() + 12, rect.width(), 20), title,
                   color=_MUTED, size=9., alignment=QtCore.Qt.AlignmentFlag.AlignCenter)

    def _paint_image(self, painter, rect):
        painter.fillRect(rect, QtGui.QColor("#000000"))
        size = QtCore.QSizeF(self._image.size())
        size.scale(rect.size(), QtCore.Qt.AspectRatioMode.KeepAspectRatio)
        target = QtCore.QRectF(rect.center().x() - size.width() / 2, rect.center().y() - size.height() / 2,
                               size.width(), size.height())
        painter.drawImage(target, self._image)

    def _paint_demo_camera(self, painter, rect):
        """Procedural optical instrument; explicitly illustrative, not a frame.

        Its gentle turn follows the already supplied demo phase. No render
        timer, frame generation, acquisition or image decoding is introduced.
        """
        stage = self._presentation == "stage"
        phase = self._frame["phase"] if self._motion_enabled else self._decoration_phase
        # A large machined body and lens create depth without fake telemetry.
        scale = min(rect.width() / 3.25, rect.height() / 2.5)
        origin = QtCore.QPointF(rect.center().x() - scale * .05,
                               rect.center().y() + scale * .47 + math.sin(phase) * scale * .018)
        turn = .08 * math.sin(phase)
        cosine, sine = math.cos(turn), math.sin(turn)

        def project(x, y, z):
            xx, yy = x * cosine - y * sine, x * sine + y * cosine
            return origin + QtCore.QPointF((.88 * xx + .48 * yy) * scale,
                                           (.27 * xx - .43 * yy - z) * scale)

        def face(points, colors, stroke="#616161"):
            polygon = QtGui.QPolygonF([project(*point) for point in points])
            gradient = QtGui.QLinearGradient(polygon.boundingRect().topLeft(), polygon.boundingRect().bottomRight())
            for offset, color in colors:
                gradient.setColorAt(offset, QtGui.QColor(color))
            painter.setBrush(gradient)
            painter.setPen(QtGui.QPen(QtGui.QColor(stroke), max(.55, scale * .004)))
            painter.drawPolygon(polygon)

        def plane_transform(x, y, z):
            base = project(x, y, z)
            u, v = project(x + 1, y, z) - base, project(x, y, z - 1) - base
            return QtGui.QTransform(u.x(), u.y(), v.x(), v.y(), base.x(), base.y())

        painter.save()
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        haze = QtGui.QRadialGradient(rect.center(), max(rect.width(), rect.height()) * .48)
        haze.setColorAt(0., QtGui.QColor(76, 76, 76, 36))
        haze.setColorAt(.55, QtGui.QColor(42, 42, 42, 15))
        haze.setColorAt(1., QtGui.QColor(12, 12, 12, 0))
        painter.fillRect(rect, haze)
        # Ground contact is layered and soft; no infinite grid or dashboard box.
        shadow_center = project(0., 0., -.16)
        painter.save()
        painter.translate(shadow_center)
        painter.scale(1., .25)
        shadow = QtGui.QRadialGradient(QtCore.QPointF(), scale * 1.65)
        shadow.setColorAt(0., QtGui.QColor(0, 0, 0, 160))
        shadow.setColorAt(.48, QtGui.QColor(4, 4, 4, 84))
        shadow.setColorAt(1., QtGui.QColor(0, 0, 0, 0))
        painter.setBrush(shadow)
        painter.drawEllipse(QtCore.QPointF(), scale * 1.65, scale * 1.65)
        painter.restore()
        # Bevelled mounting plate, then neutral-metal rear and top planes.
        face([(-1.13, -.76, .03), (1.13, -.76, .03), (1.13, .73, .03), (-1.13, .73, .03)],
             [(0., "#323232"), (.45, "#1C1C1C"), (1., "#0C0C0C")], "#474747")
        face([(-1.13, -.76, -.03), (1.13, -.76, -.03), (1.13, -.76, .03), (-1.13, -.76, .03)],
             [(0., "#191919"), (1., "#474747")], "#494949")
        face([(1.13, -.76, -.03), (1.13, .73, -.03), (1.13, .73, .03), (1.13, -.76, .03)],
             [(0., "#333333"), (1., "#161616")], "#545454")
        face([(1., -.60, .09), (1., .60, .09), (1., .60, .99), (1., -.60, .99)],
             [(0., "#222222"), (.52, "#414141"), (1., "#161616")], "#545454")
        face([(-1., -.60, .99), (1., -.60, .99), (1., .60, .99), (-1., .60, .99)],
             [(0., "#727272"), (.22, "#434343"), (.74, "#272727"), (1., "#4E4E4E")], "#909090")
        face([(-1., -.60, .09), (1., -.60, .09), (1., -.60, .99), (-1., -.60, .99)],
             [(0., "#323232"), (.32, "#141414"), (1., "#303030")], "#7E7E7E")
        # Recessed front, white indicator light, and layered silver lens reflections.
        face([(-.91, -.612, .18), (.91, -.612, .18), (.91, -.612, .90), (-.91, -.612, .90)],
             [(0., "#101010"), (1., "#212121")], "#101010")
        painter.setPen(QtGui.QPen(QtGui.QColor("#191919"), max(.7, scale * .013)))
        for index in range(8):
            y = -.40 + index * .115
            painter.drawLine(project(1.005, y, .28), project(1.005, y, .73))
        painter.setPen(QtGui.QPen(QtGui.QColor(196, 196, 196, 66), max(.5, scale * .003)))
        for index in range(7):
            y = -.28 + index * .10
            painter.drawLine(project(-.75, y, 1.002), project(.76, y, 1.002))
        painter.setPen(QtGui.QPen(QtGui.QColor("#313131"), scale * .045, QtCore.Qt.PenStyle.SolidLine,
                                 QtCore.Qt.PenCapStyle.RoundCap))
        painter.drawLine(project(-.71, -.63, .37), project(-.71, -.63, .69))
        indicator = QtGui.QColor("#ACACAC")
        indicator.setAlpha(175 + int(45 * (.5 + .5 * math.sin(phase))))
        painter.setPen(QtGui.QPen(indicator, scale * .025, QtCore.Qt.PenStyle.SolidLine,
                                 QtCore.Qt.PenCapStyle.RoundCap))
        painter.drawLine(project(-.71, -.64, .48), project(-.71, -.64, .64))
        painter.save()
        painter.setTransform(plane_transform(.16, -.77, .57), True)
        for radius, low, high in ((.47, "#202020", "#979797"), (.432, "#C6C6C6", "#292929"),
                                  (.397, "#191919", "#5B5B5B"), (.355, "#595959", "#0A0A0A")):
            metal = QtGui.QLinearGradient(-radius, -radius, radius, radius)
            metal.setColorAt(0., QtGui.QColor(low))
            metal.setColorAt(.5, QtGui.QColor(high))
            metal.setColorAt(1., QtGui.QColor(low))
            painter.setBrush(metal)
            painter.setPen(QtCore.Qt.PenStyle.NoPen)
            painter.drawEllipse(QtCore.QPointF(), radius, radius)
        # Fine concentric engraved rings are physical detailing, not gauges.
        for radius in (.365, .378, .412, .447):
            painter.setPen(QtGui.QPen(QtGui.QColor(186, 186, 186, 75), .004))
            painter.setBrush(QtCore.Qt.BrushStyle.NoBrush)
            painter.drawEllipse(QtCore.QPointF(), radius, radius)
        glass = QtGui.QRadialGradient(QtCore.QPointF(-.1, -.13), .53)
        glass.setColorAt(0., QtGui.QColor("#BABABA"))
        glass.setColorAt(.23, QtGui.QColor("#525252"))
        glass.setColorAt(.47, QtGui.QColor("#282828"))
        glass.setColorAt(.72, QtGui.QColor("#0B0B0B"))
        glass.setColorAt(1., QtGui.QColor("#262626"))
        painter.setBrush(glass)
        painter.setPen(QtGui.QPen(QtGui.QColor("#6C6C6C"), .009))
        painter.drawEllipse(QtCore.QPointF(), .332, .332)
        painter.setPen(QtGui.QPen(QtGui.QColor(146, 146, 146, 70), .006))
        painter.setBrush(QtGui.QColor(8, 8, 8, 105))
        painter.drawEllipse(QtCore.QPointF(.02, .025), .18, .18)
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        reflection = QtGui.QRadialGradient(QtCore.QPointF(-.12, -.15), .17)
        reflection.setColorAt(0., QtGui.QColor(244, 244, 244, 100))
        reflection.setColorAt(1., QtGui.QColor(209, 209, 209, 0))
        painter.setBrush(reflection)
        painter.drawEllipse(QtCore.QPointF(-.12, -.15), .18, .13)
        painter.setBrush(QtGui.QColor(190, 190, 190, 80))
        painter.drawEllipse(QtCore.QPointF(.19, .16), .031, .066)
        painter.restore()
        # Small machined screws add scale; no fabricated measurement readouts.
        for x, z in ((-.84, .82), (.83, .82), (-.84, .25), (.83, .25)):
            point = project(x, -.63, z)
            painter.setPen(QtGui.QPen(QtGui.QColor("#8F8F8F"), max(.4, scale * .003)))
            painter.setBrush(QtGui.QColor("#262626"))
            painter.drawEllipse(point, scale * .017, scale * .022)
        painter.restore()
        if self._presentation != "thumbnail":
            caption = tr('合成传感器示意') if stage else tr('合成场景 · 非真实影像')
            self._text(painter, QtCore.QRectF(rect.left() + 12, rect.bottom() - 28, rect.width() - 24, 18),
                       caption, size=7.7, color=_MUTED,
                       alignment=QtCore.Qt.AlignmentFlag.AlignCenter)

    def _project(self, point, *, tracker=False):
        x, y, z = point
        if tracker:
            y, z = z, y
        xr = math.cos(self._yaw) * x + math.sin(self._yaw) * z
        zr = -math.sin(self._yaw) * x + math.cos(self._yaw) * z
        yr = math.cos(self._pitch) * y - math.sin(self._pitch) * zr
        depth = math.sin(self._pitch) * y + math.cos(self._pitch) * zr
        return xr, -yr, depth

    def _paint_tracker(self, painter, rect):
        span = self._tracker_span
        scale = min(rect.width(), rect.height() * 1.4) / (span * 3.3) * self._zoom
        center = QtCore.QPointF(rect.center().x(), rect.top() + rect.height() * .57)

        def project(point):
            x, y, _ = self._project(point, tracker=True)
            return center + QtCore.QPointF(x * scale, y * scale)

        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        plane = QtGui.QLinearGradient(rect.topLeft(), rect.bottomRight())
        plane.setColorAt(0., QtGui.QColor(36, 36, 36, 170))
        plane.setColorAt(1., QtGui.QColor(20, 20, 20, 120))
        painter.setBrush(plane)
        painter.drawPolygon(QtGui.QPolygonF([project(point) for point in
                            ((-span, -span, 0), (-span, span, 0), (span, span, 0), (span, -span, 0))]))
        for index in range(-4, 5):
            painter.setPen(QtGui.QPen(QtGui.QColor("#5C5C5C" if index == 0 else "#373737"), .65))
            value = index / 4 * span
            painter.drawLine(project((value, -span, 0)), project((value, span, 0)))
            painter.drawLine(project((-span, value, 0)), project((span, value, 0)))
        relative = [tuple(v - origin for v, origin in zip(point, self._tracker_origin)) for point in self._trail]
        for index, (start, end) in enumerate(zip(relative, relative[1:])):
            color = QtGui.QColor(209, 209, 209, 30 + round(155 * (index + 1) / len(relative)))
            painter.setPen(QtGui.QPen(color, 1.65, QtCore.Qt.PenStyle.SolidLine, QtCore.Qt.PenCapStyle.RoundCap))
            painter.drawLine(project(start), project(end))
        current = relative[-1]
        at = project(current)
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        aura = QtGui.QRadialGradient(at, 19)
        aura.setColorAt(0., QtGui.QColor(209, 209, 209, 45))
        aura.setColorAt(1., QtGui.QColor(209, 209, 209, 0))
        painter.setBrush(aura)
        painter.drawEllipse(at, 19, 19)
        qx, qy, qz, qw = self._frame["orientation"]
        basis = ((1 - 2 * (qy*qy + qz*qz), 2 * (qx*qy + qz*qw), 2 * (qx*qz - qy*qw)),
                 (2 * (qx*qy - qz*qw), 1 - 2 * (qx*qx + qz*qz), 2 * (qy*qz + qx*qw)),
                 (2 * (qx*qz + qy*qw), 2 * (qy*qz - qx*qw), 1 - 2 * (qx*qx + qy*qy)))
        for axis, vector, color in zip("XYZ", basis, ("#D86D73", "#31A086", "#087EFF")):
            end = project(tuple(v + span * .52 * delta for v, delta in zip(current, vector)))
            painter.setPen(QtGui.QPen(QtGui.QColor(color), 1.8, QtCore.Qt.PenStyle.SolidLine, QtCore.Qt.PenCapStyle.RoundCap))
            painter.drawLine(at, end)
            painter.setPen(QtCore.Qt.PenStyle.NoPen)
            painter.setBrush(QtGui.QColor(color))
            painter.drawEllipse(end, 1.7, 1.7)
            self._text(painter, QtCore.QRectF(end.x() - 8, end.y() - 18, 22, 18), axis,
                       color=QtGui.QColor(color), size=8., bold=True)
        painter.setPen(QtGui.QPen(_ACCENT, 1.4))
        painter.setBrush(QtGui.QColor("#F6F6F6"))
        painter.drawEllipse(at, 4.2, 4.2)
        if self._presentation != "thumbnail":
            self._text(painter, QtCore.QRectF(rect.left() + 4, rect.top() + 1, rect.width() - 8, 17),
                       tr('相对首帧 · 拖动查看'), size=7.7)

    def _paint_glove(self, painter, rect):
        if not self._frame.get("relative"):
            self._text(painter, rect, tr('没有可用骨架，请查看 3D 手形'), alignment=QtCore.Qt.AlignmentFlag.AlignCenter)
            return
        # A quiet drafting surface: decoration only, never extra anatomy.
        backdrop = QtGui.QRadialGradient(rect.center(), max(rect.width(), rect.height()) * .55)
        backdrop.setColorAt(0., QtGui.QColor(56, 56, 56, 112))
        backdrop.setColorAt(1., QtGui.QColor(12, 12, 12, 0))
        painter.fillRect(rect, backdrop)
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.setBrush(QtGui.QColor(123, 123, 123, 72))
        for x in range(int(rect.left()) + 8, int(rect.right()), 16):
            for y in range(int(rect.top()) + 23, int(rect.bottom()), 16):
                painter.drawEllipse(QtCore.QPointF(x, y), .55, .55)
        relative = self._frame["relative"]
        points = [self._project(point) for point in relative]
        span = self._glove_span
        scale = min(rect.width() / (span * 1.05), rect.height() / (span * 1.15)) * self._zoom
        center = QtCore.QPointF(rect.center().x(), rect.top() + rect.height() * .89)
        positions = [center + QtCore.QPointF(x * scale, y * scale) for x, y, _ in points]
        painter.setPen(QtGui.QPen(QtGui.QColor(209, 209, 209, 80), .7))
        palm = QtGui.QLinearGradient(rect.topLeft(), rect.bottomRight())
        palm.setColorAt(0., QtGui.QColor(159, 159, 159, 35))
        palm.setColorAt(1., QtGui.QColor(209, 209, 209, 18))
        painter.setBrush(palm)
        painter.drawPolygon(QtGui.QPolygonF([positions[index] for index in (0, 5, 9, 13, 17)]))
        bones = sorted(_BONES, key=lambda pair: points[pair[0]][2] + points[pair[1]][2])
        for start, end in bones:
            painter.setPen(QtGui.QPen(QtGui.QColor(209, 209, 209, 22), 5., QtCore.Qt.PenStyle.SolidLine, QtCore.Qt.PenCapStyle.RoundCap))
            painter.drawLine(positions[start], positions[end])
            painter.setPen(QtGui.QPen(QtGui.QColor("#ADADAD"), 1.65, QtCore.Qt.PenStyle.SolidLine, QtCore.Qt.PenCapStyle.RoundCap))
            painter.drawLine(positions[start], positions[end])
        for index, position in enumerate(positions):
            tip = index in (4, 8, 12, 16, 20)
            if tip:
                painter.setPen(QtCore.Qt.PenStyle.NoPen)
                painter.setBrush(QtGui.QColor(209, 209, 209, 25))
                painter.drawEllipse(position, 6.5, 6.5)
            painter.setPen(QtGui.QPen(_ACCENT if tip else QtGui.QColor("#B7B7B7"), 1.))
            painter.setBrush(_ACCENT if tip else QtGui.QColor("#272727"))
            painter.drawEllipse(position, 2.7 if tip else 2.15, 2.7 if tip else 2.15)
        if self._presentation != "thumbnail":
            self._text(painter, QtCore.QRectF(rect.left() + 3, rect.top(), rect.width() - 6, 17),
                       tr('21 关节点 · 腕部相对 · 非 MANO 网格'), size=7.7)

    @staticmethod
    def _heatmap_rect(rect):
        side = max(1., min(rect.height() - 13, rect.width() - 48))
        return QtCore.QRectF(rect.center().x() - side / 2 - 5, rect.center().y() - side / 2 - 3, side, side)

    def _paint_tactile(self, painter, rect):
        if self.tactile_view == "hand":
            self._paint_tactile_hand(painter, rect)
            return
        if not self.tactile_hand_available:
            # Resolve the cached translation before shortening it; str methods
            # on LocalizedText otherwise use its original-language payload.
            note = str(self._tactile_layout_note).replace("；", ";").split(";", 1)[0]
            self._text(painter, QtCore.QRectF(rect.left() + 3, rect.top(), rect.width() - 6, 14),
                       note, size=7.2)
            rect = rect.adjusted(0, 15, 0, 0)
        matrix = self._heatmap_rect(rect)
        cell = matrix.width() / 20
        gap = min(1.2, cell * .14)
        painter.setPen(QtGui.QPen(QtGui.QColor("#4A4A4A"), .6))
        painter.setBrush(QtGui.QColor("#171717"))
        painter.drawRoundedRect(matrix.adjusted(-2, -2, 2, 2), 3, 3)
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        for index, color in enumerate(self._cell_colors):
            row, col = divmod(index, 20)
            painter.setBrush(color)
            tile = QtCore.QRectF(matrix.left() + col * cell + gap / 2,
                                 matrix.top() + row * cell + gap / 2, cell - gap, cell - gap)
            painter.drawRoundedRect(tile, min(1.7, cell * .16), min(1.7, cell * .16))
        for index in (0, 19):
            self._text(painter, QtCore.QRectF(matrix.left() - 18, matrix.top() + index * cell - 2, 15, cell + 4),
                       str(index), size=7., alignment=QtCore.Qt.AlignmentFlag.AlignRight | QtCore.Qt.AlignmentFlag.AlignVCenter)
            self._text(painter, QtCore.QRectF(matrix.left() + index * cell - 3, matrix.bottom() + 1, cell + 7, 12),
                       str(index), size=7., alignment=QtCore.Qt.AlignmentFlag.AlignCenter)
        legend = QtCore.QRectF(matrix.right() + 10, matrix.top(), 5, matrix.height())
        gradient = QtGui.QLinearGradient(legend.bottomLeft(), legend.topLeft())
        for position, rgb in self._tactile_palette():
            gradient.setColorAt(position, QtGui.QColor(*rgb))
        painter.setBrush(gradient)
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.drawRoundedRect(legend, 2.5, 2.5)

    @staticmethod
    def _hand_rect(rect):
        return rect.adjusted(3., 17., -19., -3.)

    def _paint_tactile_hand(self, painter, rect):
        geometry = self._tactile_geometry
        self._text(painter, QtCore.QRectF(rect.left() + 3, rect.top(), rect.width() - 6, 14),
                   tr('候选示意 · {v0} 锚点 · 其余未验证', v0=geometry.anchor_count), size=7.2)
        hand_rect = self._hand_rect(rect)
        transform = geometry.transform(hand_rect)
        painter.setPen(QtGui.QPen(QtGui.QColor("#6A6A6A"), 1.))
        painter.setBrush(QtGui.QColor("#242424"))
        painter.drawPath(transform.map(geometry.outline))
        for point, path in geometry.cells:
            painter.setPen(QtCore.Qt.PenStyle.NoPen)
            painter.setBrush(self._cell_colors[point.row * 20 + point.col])
            mapped = transform.map(path)
            painter.drawPath(mapped)
            if point.verified_anchor and transform.m11() >= 6:
                painter.setPen(QtGui.QPen(QtGui.QColor("#797979"), .7))
                painter.setBrush(QtCore.Qt.BrushStyle.NoBrush)
                painter.drawPath(mapped)
        legend = QtCore.QRectF(rect.right() - 9, hand_rect.top() + 5, 5, max(1., hand_rect.height() - 10))
        gradient = QtGui.QLinearGradient(legend.bottomLeft(), legend.topLeft())
        for position, rgb in self._tactile_palette():
            gradient.setColorAt(position, QtGui.QColor(*rgb))
        painter.setBrush(gradient)
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.drawRoundedRect(legend, 2.5, 2.5)

    def _footer_text(self):
        if self._preview_error:
            label = tr('预览数据无效 · 保留上一画面') if self._frame is not None else tr('预览暂不可用')
        elif self._frame is None:
            if self._mode == "replay":
                label = tr('未录制') if self._device.get("recorded_count") == 0 else tr('该时刻无样本')
            else:
                label = tr('数据接入后自动显示')
        elif self.kind == "camera":
            label = tr('合成示意 · 非真实相机') if self._mode == "demo" else tr('{v0} × {v1} · JPEG', v0=self._frame['width'], v1=self._frame['height'])
        elif self.kind == "tracker":
            label = "  ".join(tr('{v0} {v1:+.3f}', v0=axis, v1=value) for axis, value in zip("XYZ", self._frame["position"])) + " m"
        elif self.kind == "glove":
            label = (tr('示意骨架 · 非求解结果') if self._mode == "demo" else
                     tr('MANO 参数重建 · 腕部相对，非世界定位') if self.glove_view == "3d" else
                     tr('测量关节点 · 拖动旋转 · 双击复位'))
        else:
            count = sum(self._frame["valid_mask"])
            if self._tactile_response():
                label = (tr('历史 {v0}/400 · ΔADC', v0=count) if self._held else
                         tr('有效 {v0}/400 · ΔADC', v0=count))
                if not self._frame["reference_ready"]:
                    label = tr('等待无载归零 · 中性示意')
            else:
                label = (tr('历史 {v0}/400 · ADC', v0=count) if self._held else tr('有效 {v0}/400 · ADC', v0=count))
            label += tr('   {v0:g}—{v1:g}', v0=self._levels[0], v1=self._levels[1])
        if self._presentation == "thumbnail" and self._frame is not None and not self._preview_error:
            if self.kind == "tracker":
                label = tr('相对首帧 · 位姿（米）')
            elif self.kind == "glove":
                label = (tr('示意骨架') if self._mode == "demo" else
                         tr('MANO 手势 · 腕部相对') if self.glove_view == "3d" else tr('测量骨架 · 腕部相对'))
            elif self.kind == "tactile":
                value_label = (tr('{v0}/400 · ΔADC', v0=sum(self._frame['valid_mask']))
                               if self._tactile_response() else
                               tr('{v0}/400 · ADC', v0=sum(self._frame['valid_mask'])))
                label = (tr('静态手 · ') if self.tactile_view == "3d" else "") + value_label
        if self._held and not self._preview_error and tr('历史') not in label:
            label = tr('历史保持 · ') + label
        if self._presentation != "standard" and not self._preview_error:
            if self._mode == "demo":
                label = tr('演示 · ') + label
            elif self._mode == "replay":
                label = tr('回放 · ') + label
        return label

    def _paint_footer(self, painter):
        inset = 7 if self._presentation == "thumbnail" else 14
        bottom = 18 if self._presentation == "thumbnail" else 23
        self._text(painter, QtCore.QRectF(inset, self.height() - bottom, max(0, self.width() - inset * 2), 16),
                   self._footer_text(), color=QtGui.QColor("#B6B6B6") if self._preview_error or self._held else _MUTED,
                   size=7.2 if self._presentation == "thumbnail" else 7.7)

    def mousePressEvent(self, event):
        if self.kind in ("tracker", "glove") and event.button() == QtCore.Qt.MouseButton.LeftButton:
            self._drag = event.position()
            self.setCursor(QtCore.Qt.CursorShape.ClosedHandCursor)
            event.accept()
        else:
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._drag is not None:
            delta = event.position() - self._drag
            self._drag = event.position()
            self._yaw += delta.x() * .009
            self._pitch = max(-1.35, min(1.35, self._pitch + delta.y() * .009))
            self.update()
            event.accept()
        elif self.kind == "tactile" and self._frame is not None and self.tactile_view != "3d":
            point = None
            canvas = self._canvas_rect()
            rect = self._heatmap_rect(canvas if self.tactile_hand_available else canvas.adjusted(0, 15, 0, 0))
            if self.tactile_view == "hand":
                point = self._tactile_geometry.point_at(event.position(), self._hand_rect(canvas))
                row, col = (point.row, point.col) if point is not None else (-1, -1)
            elif rect.contains(event.position()):
                col = min(19, int((event.position().x() - rect.left()) / rect.width() * 20))
                row = min(19, int((event.position().y() - rect.top()) / rect.height() * 20))
            else:
                row, col = -1, -1
            if row >= 0:
                index = row * 20 + col
                valid = self._frame["valid_mask"][index]
                value = (tr('{v0:g} ΔADC', v0=self._frame['values'][index]) if self._tactile_response() else
                         tr('{v0:g} ADC', v0=self._frame['values'][index])) if valid else tr('无效 / 未观测')
                description = tr('第 {v0} 行，第 {v1} 列 · {v2}', v0=row, v1=col, v2=value) + (tr(' · 历史画面') if self._held else "")
                if point is not None:
                    region = {"palm": tr('手掌（掌心 / 掌根）'), "thumb": tr('拇指'), "index": tr('食指'), "middle": tr('中指'),
                              "ring": tr('无名指'), "little": tr('小指')}[point.region]
                    description += tr(' · {v0} · ', v0=region) + (tr('已观测锚点') if point.verified_anchor else tr('候选位置未验证'))
                self.setToolTip(description)
            else:
                self.setToolTip(self._tactile_layout_note)
            super().mouseMoveEvent(event)
        else:
            super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if self._drag is not None and event.button() == QtCore.Qt.MouseButton.LeftButton:
            self._drag = None
            self.setCursor(QtCore.Qt.CursorShape.OpenHandCursor)
            event.accept()
        else:
            super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event):
        if self.kind in ("tracker", "glove"):
            self._yaw, self._pitch, self._zoom = -.24, .32, 1.
            self.update()
            event.accept()
        else:
            super().mouseDoubleClickEvent(event)

    def wheelEvent(self, event):
        if self.kind in ("tracker", "glove"):
            self._zoom = min(2., max(.55, self._zoom * (1.1 if event.angleDelta().y() > 0 else 1 / 1.1)))
            self.update()
            event.accept()
        else:
            super().wheelEvent(event)
