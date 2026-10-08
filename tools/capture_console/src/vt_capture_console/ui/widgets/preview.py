"""Bounded, read-only previews. No ROS or acquisition imports.

OpenGL and the shared MANO renderer load only when selecting tactile 3D.

The console supplies snapshots through ``set_device``. This widget does not own
a timer or a sensor, and never extrapolates live measurements. Synthetic scenes
are opt-in and visibly labelled; a held picture is not a new measurement.
"""

from __future__ import annotations

import base64
import binascii
from collections import deque
from collections.abc import Mapping
import math

from PySide6 import QtCore, QtGui, QtWidgets

from .tactile_hand import right_hand_geometry


MAX_BASE64_BYTES = 8_000_000
MAX_IMAGE_DIMENSION = 4096
MAX_IMAGE_PIXELS = 8_000_000
MAX_TRAIL_POINTS = 80

_KINDS = {"camera", "tracker", "glove", "tactile"}
_ONLINE = {"online", "live", "streaming", "connected", "ok", "ready", "active"}
_MUTED = QtGui.QColor("#7D8796")
_INK = QtGui.QColor("#303844")
_ACCENT = QtGui.QColor("#087EFF")
_INVALID = QtGui.QColor("#E5E8ED")
_HEAT_STOPS = ((0., (237, 244, 252)), (.32, (136, 198, 251)),
               (.58, (14, 124, 237)), (.78, (249, 191, 101)), (1., (238, 105, 77)))
_BONES = tuple((0 if offset == 0 else base + offset - 1, base + offset)
               for base in (1, 5, 9, 13, 17) for offset in range(4))
_HEADERS = {"camera": "相机画面", "tracker": "空间位姿",
            "glove": "手部骨架", "tactile": "20 × 20 触觉"}
_EMPTY = {"camera": "等待相机画面", "tracker": "等待 Tracker 位姿",
          "glove": "等待手部关节点", "tactile": "等待触觉数据"}


def _finite(value):
    return (not isinstance(value, bool) and isinstance(value, (float, int))
            and math.isfinite(value))


def _vector(value, length):
    if (not isinstance(value, (tuple, list)) or len(value) != length
            or not all(_finite(v) and abs(v) <= 1e12 for v in value)):
        raise ValueError("无效的位姿数组")
    return tuple(float(v) for v in value)


def _color(value, lo, hi):
    fraction = min(1., max(0., (value - lo) / (hi - lo)))
    for (start, low), (end, high) in zip(_HEAT_STOPS, _HEAT_STOPS[1:]):
        if fraction <= end:
            t = (fraction - start) / (end - start)
            return QtGui.QColor(*(round(a + (b - a) * t) for a, b in zip(low, high)))
    return QtGui.QColor(*_HEAT_STOPS[-1][1])


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

    def __init__(self, kind: str, parent=None):
        super().__init__(parent)
        if kind not in _KINDS:
            raise ValueError("kind must be camera, tracker, glove, or tactile")
        self.kind = kind
        self._device = {}
        self._identity = None
        self._frame = None
        self._image = None
        self._encoded_image = None
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
        if self._glove_3d is not None:
            self._glove_3d.setVisible(active)
            if active:
                self._glove_3d.setGeometry(self._canvas_rect().toRect())
                self._glove_3d.set_sample(self._frame, held=self._held, mode=self._mode,
                                           device=self._device)

    def copy_glove_state_from(self, source):
        if self.kind != "glove" or source.kind != "glove":
            raise ValueError("glove state copying requires glove widgets")
        if self._identity is not None and self._identity != source._identity:
            raise ValueError("glove display must belong to the same device and hand")
        self._identity = source._identity
        self._device = dict(source._device)
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
                    self._tactile_layout_note = f"右手布局不可用 · 仅原始矩阵；未映射：{exc}"
                else:
                    count = self._tactile_geometry.anchor_count
                    self._tactile_layout_note = (
                        f"右手候选手形 · {count} 个已观测锚点，其余位置未逐点验证；"
                        "原始 ADC 不变，不代表压力。" + self._tactile_geometry.layout.note)
            else:
                label = "左手" if side == "left" else "未指定左右手"
                self._tactile_layout_note = f"{label}未映射 · 仅原始矩阵；不镜像或推断触点的解剖位置。"
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
        self._device = dict(source._device)
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
        self._identity = source._identity
        self._device = dict(source._device)
        self._frame = dict(source._frame) if source._frame is not None else None
        self._image = QtGui.QImage(source._image) if source._image is not None else None
        self._encoded_image = source._encoded_image
        self._mode, self._held = source._mode, source._held
        self._preview_error = source._preview_error
        self._trail = deque(source._trail, maxlen=MAX_TRAIL_POINTS)
        self._tracker_origin, self._tracker_span = source._tracker_origin, source._tracker_span
        self.setAccessibleName(source.accessibleName())
        self.setAccessibleDescription(source.accessibleDescription())
        self.update()

    def _reset_frame(self):
        self._frame = self._image = self._encoded_image = None
        self._levels = None
        self._cell_colors = ()
        self._trail.clear()
        self._tracker_origin = None
        self._tracker_span = .25
        self._glove_span = None

    def set_device(self, device: dict):
        self._set_device(device)
        self._refresh_tactile_3d()
        self._refresh_glove_3d()

    def _set_device(self, device: dict):
        if not isinstance(device, Mapping):
            self._preview_error = "预览数据格式无效"
            self._held = self._frame is not None
            self.update()
            return
        supplied_kind = device.get("kind", self.kind)
        if supplied_kind != self.kind:
            self._preview_error = "设备类型与预览不匹配"
            self._held = self._frame is not None
            self.update()
            return
        identity = (device.get("id", self._device.get("id")), device.get("side", self._device.get("side")))
        if self._identity is not None and identity != self._identity:
            self._reset_frame()
        self._identity = identity
        self._device = dict(device)
        self._sync_tactile_view()
        self.setAccessibleName(str(device.get("label", _HEADERS[self.kind])))
        preview = device.get("preview")
        self._preview_error = ""
        if not isinstance(preview, Mapping) or not preview:
            self._held = self._frame is not None
            self.update()
            return
        mode = "demo" if preview.get("demo") is True else (
            "replay" if preview.get("replay") is True or preview.get("mode") == "replay" else "live")
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
            self._preview_error = str(exc) or "预览数据无效"
            self._held = self._frame is not None
        self.update()

    def _prepare_demo(self, preview):
        phase = preview.get("phase", 0.)
        if not _finite(phase):
            raise ValueError("演示时间无效")
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
                raise ValueError("姿态四元数无效")
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
            mano = preview.get("mano")
            if mano is not None:
                # Keep the common 2D path independent of numpy/OpenGL/model packages.
                if not isinstance(mano, Mapping) or mano.get("hand") != self._device.get("side"):
                    raise ValueError("MANO 手别与数据流不一致")
                mano = dict(mano)
                mano["pose"] = _vector(mano.get("pose"), 48)
                mano["shape"] = _vector(mano.get("shape"), 10)
                if mano.get("source_mode") not in ("live", "replay"):
                    raise ValueError("MANO 数据来源无效")
                for key in ("session_id", "model_rest_sha256", "config_sha256"):
                    if not isinstance(mano.get(key), str) or len(mano[key]) > 256:
                        raise ValueError("MANO 会话或模型身份无效")
                for key in ("calibration_version", "frame_index", "source_realtime_ns"):
                    if type(mano.get(key)) is not int or not 0 <= mano[key] < 2**64:
                        raise ValueError("MANO 帧标识无效")
            raw = preview.get("joints")
            if not raw and mano is not None:
                return {"joints": (), "relative": (), "mano": mano}
            if not isinstance(raw, (tuple, list)) or len(raw) != 21:
                raise ValueError("手骨架需要 21 个关节点")
            joints = tuple(_vector(point, 3) for point in raw)
            relative = tuple(tuple(v - origin for v, origin in zip(point, joints[0])) for point in joints)
            span = max(math.hypot(*point) for point in relative)
            if not math.isfinite(span) or span < 1e-10:
                raise ValueError("手关节点范围无效")
            self._glove_span = max(self._glove_span or span, span)
            return {"joints": joints, "relative": relative, **({"mano": mano} if mano else {})}
        rows, cols = preview.get("rows", 20), preview.get("cols", 20)
        if type(rows) is not int or type(cols) is not int or (rows, cols) != (20, 20):
            raise ValueError("触觉预览必须是 20 × 20")
        raw, mask = preview.get("values"), preview.get("valid_mask")
        if (not isinstance(raw, (tuple, list)) or len(raw) != 400
                or not isinstance(mask, (tuple, list)) or len(mask) != 400
                or any(type(v) is not bool for v in mask)):
            raise ValueError("触觉需要 400 个数值及有效掩码")
        if any((not _finite(value) or abs(value) > 1e12) and mask[index] for index, value in enumerate(raw)):
            raise ValueError("有效触觉单元包含非有限数值")
        values = tuple(float(value) if _finite(value) else 0. for value in raw)
        mask = tuple(mask)
        if not any(mask):
            return None
        levels = preview.get("levels")
        if levels is not None:
            lo, hi = _vector(levels, 2)
            if lo >= hi or not math.isfinite(hi - lo):
                raise ValueError("触觉色标范围无效")
            self._levels = (lo, hi)
        if self._levels is None:
            observed = [v for v, valid in zip(values, mask) if valid]
            lo, hi = min(observed), max(observed)
            if hi - lo < 100:
                center = lo / 2 + hi / 2
                lo, hi = center - 50, center + 50
            self._levels = (lo, hi)
        self._cell_colors = tuple(_color(v, *self._levels) if valid else _INVALID for v, valid in zip(values, mask))
        return {"values": values, "valid_mask": mask, "rows": 20, "cols": 20}

    def _prepare_image(self, preview):
        if str(preview.get("format", "")).lower() not in ("jpeg", "jpg"):
            raise ValueError("仅支持 JPEG 预览")
        encoded = preview.get("data_b64")
        if not isinstance(encoded, str) or not encoded or len(encoded) > MAX_BASE64_BYTES:
            raise ValueError("图像预览超过大小限制或缺少数据")
        for name in ("width", "height"):
            if name in preview and (type(preview[name]) is not int or not 0 < preview[name] <= MAX_IMAGE_DIMENSION):
                raise ValueError("图像预览尺寸无效")
        if encoded == self._encoded_image and self._image is not None:
            image = self._image
        else:
            binary = base64.b64decode(encoded, validate=True)
            if not binary.startswith(b"\xff\xd8"):
                raise ValueError("预览不是 JPEG 图像")
            buffer = QtCore.QBuffer()
            buffer.setData(QtCore.QByteArray(binary))
            buffer.open(QtCore.QIODevice.OpenModeFlag.ReadOnly)
            reader = QtGui.QImageReader(buffer, b"jpeg")
            size = reader.size()
            if (not size.isValid() or size.width() > MAX_IMAGE_DIMENSION or size.height() > MAX_IMAGE_DIMENSION
                    or size.width() * size.height() > MAX_IMAGE_PIXELS):
                raise ValueError("图像预览超过尺寸限制")
            image = reader.read()
            if image.isNull():
                raise ValueError("图像预览解码失败")
        if (preview.get("width", image.width()) != image.width()
                or preview.get("height", image.height()) != image.height()):
            raise ValueError("声明的图像尺寸与内容不匹配")
        self._image, self._encoded_image = image, encoded
        return {"width": image.width(), "height": image.height()}

    def _canvas_rect(self):
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
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        painter.setRenderHint(QtGui.QPainter.RenderHint.SmoothPixmapTransform)
        outer = QtCore.QRectF(self.rect()).adjusted(.5, .5, -.5, -.5)
        surface = QtGui.QLinearGradient(outer.topLeft(), outer.bottomLeft())
        surface.setColorAt(0., QtGui.QColor("#FEFEFF"))
        surface.setColorAt(1., QtGui.QColor("#F3F5F8"))
        painter.setPen(QtGui.QPen(QtGui.QColor("#E1E5EB"), .7))
        painter.setBrush(surface)
        painter.drawRoundedRect(outer, 18., 18.)
        painter.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 205), .8))
        painter.setBrush(QtCore.Qt.BrushStyle.NoBrush)
        painter.drawRoundedRect(outer.adjusted(1., 1., -1., -1.), 17., 17.)
        heading = ("静态 MANO · ADC" if self.kind == "tactile" and self.tactile_view == "3d"
                   else "MANO 手势 · 解算结果" if self.kind == "glove" and self.glove_view == "3d"
                   else "候选手形触觉" if self.kind == "tactile" and self.tactile_view == "hand"
                   else _HEADERS[self.kind])
        self._text(painter, QtCore.QRectF(14, 8, max(0, self.width() - 120), 17),
                   heading, color=_INK, size=8.6, bold=True)
        self._paint_badge(painter)
        rect = self._canvas_rect()
        painter.save()
        clip = QtGui.QPainterPath()
        clip.addRoundedRect(rect, 12., 12.)
        painter.setClipPath(clip)
        if ((self.kind == "tactile" and self.tactile_view == "3d")
                or (self.kind == "glove" and self.glove_view == "3d" and self._glove_3d is not None)):
            pass  # Child OpenGL canvas; header/history badge and footer stay visible.
        elif self._frame is None:
            self._paint_empty(painter, rect)
        elif self.kind == "camera":
            self._paint_demo_camera(painter, rect) if self._mode == "demo" else self._paint_image(painter, rect)
        elif self.kind == "tracker":
            self._paint_tracker(painter, rect)
        elif self.kind == "glove":
            self._paint_glove(painter, rect)
        else:
            self._paint_tactile(painter, rect)
        if self._frame is not None and self._held:
            painter.fillRect(rect, QtGui.QColor(248, 249, 251, 62))
            label = QtCore.QRectF(rect.center().x() - 56, rect.bottom() - 28, 112, 23)
            painter.setPen(QtGui.QPen(QtGui.QColor(191, 159, 108, 42), .6))
            painter.setBrush(QtGui.QColor(255, 255, 255, 240))
            painter.drawRoundedRect(label, 11.5, 11.5)
            self._text(painter, label, "历史画面 · 已保持", color=QtGui.QColor("#8B6C3E"), size=8.,
                       alignment=QtCore.Qt.AlignmentFlag.AlignCenter)
        painter.restore()
        self._paint_footer(painter)
        painter.end()

    def _paint_badge(self, painter):
        if self._mode == "demo":
            label, foreground, background = "DEMO · 演示", "#766095", "#F0EBF6"
        elif self._mode == "replay":
            label, foreground, background = "回放", "#4D6894", "#EAF0F8"
        elif self._held:
            label, foreground, background = "保留画面", "#927041", "#F6EFE3"
        elif self._frame is not None:
            label, foreground, background = "实时预览", "#33766B", "#EAF4F0"
        else:
            label, foreground, background = "等待数据", "#808996", "#EEF0F3"
        width = 92 if self._mode == "demo" else 72
        rect = QtCore.QRectF(max(4, self.width() - width - 12), 7, width, 21)
        painter.setBrush(QtGui.QColor(background))
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.drawRoundedRect(rect, 10.5, 10.5)
        self._text(painter, rect, label, color=QtGui.QColor(foreground), size=7.7, bold=True,
                   alignment=QtCore.Qt.AlignmentFlag.AlignCenter)

    def _paint_empty(self, painter, rect):
        center = rect.center()
        painter.setPen(QtGui.QPen(QtGui.QColor("#B9C3D0"), 1.2))
        painter.setBrush(QtGui.QColor("#EEF1F5"))
        icon = QtCore.QRectF(center.x() - 20, center.y() - 28, 40, 31)
        painter.drawRoundedRect(icon, 7, 7)
        painter.drawEllipse(QtCore.QPointF(center.x(), center.y() - 13), 6, 6)
        painter.drawLine(QtCore.QPointF(center.x() + 13, center.y() - 18), QtCore.QPointF(center.x() + 15, center.y() - 18))
        self._text(painter, QtCore.QRectF(rect.left(), center.y() + 12, rect.width(), 20), _EMPTY[self.kind],
                   color=_MUTED, size=9., alignment=QtCore.Qt.AlignmentFlag.AlignCenter)

    def _paint_image(self, painter, rect):
        painter.fillRect(rect, QtGui.QColor("#E9EDF2"))
        size = QtCore.QSizeF(self._image.size())
        size.scale(rect.size(), QtCore.Qt.AspectRatioMode.KeepAspectRatio)
        target = QtCore.QRectF(rect.center().x() - size.width() / 2, rect.center().y() - size.height() / 2,
                               size.width(), size.height())
        painter.drawImage(target, self._image)

    def _paint_demo_camera(self, painter, rect):
        gradient = QtGui.QLinearGradient(rect.topLeft(), rect.bottomRight())
        gradient.setColorAt(0, QtGui.QColor("#EBF1F8"))
        gradient.setColorAt(1, QtGui.QColor("#FAFBFD"))
        painter.fillRect(rect, gradient)
        painter.setPen(QtGui.QPen(QtGui.QColor("#DCE4EE"), .65))
        horizon = rect.top() + rect.height() * .35
        for i in range(8):
            x = rect.left() + i / 7 * rect.width()
            painter.drawLine(QtCore.QPointF(rect.center().x() + (x - rect.center().x()) * .25, horizon), QtCore.QPointF(x, rect.bottom()))
        for fraction in (.35, .45, .58, .76, 1.):
            y = rect.top() + fraction * rect.height()
            painter.drawLine(QtCore.QPointF(rect.left(), y), QtCore.QPointF(rect.right(), y))
        w, h = min(rect.width() * .68, 290), min(rect.height() * .45, 120)
        x, y = rect.center().x() - w / 2, rect.top() + rect.height() * .43
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.setBrush(QtGui.QColor(50, 78, 108, 12))
        painter.drawRoundedRect(QtCore.QRectF(x + 2, y + 5, w, h), 12, 12)
        painter.setPen(QtGui.QPen(QtGui.QColor("#D4DCE7"), .8))
        painter.setBrush(QtGui.QColor("#FFFFFF"))
        painter.drawRoundedRect(QtCore.QRectF(x, y, w, h), 12, 12)
        phase = self._frame["phase"]
        radius = min(w, h) * .22
        ball = QtCore.QPointF(x + w * .68 + math.sin(phase) * w * .04, y + h * .48)
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.setBrush(QtGui.QColor(56, 101, 163, 36))
        painter.drawEllipse(ball + QtCore.QPointF(2, radius * .7), radius * 1.2, radius * .33)
        sphere = QtGui.QRadialGradient(ball - QtCore.QPointF(radius * .35, radius * .4), radius * 1.5)
        sphere.setColorAt(0., QtGui.QColor("#8CC9FF"))
        sphere.setColorAt(.42, _ACCENT)
        sphere.setColorAt(1., QtGui.QColor("#1260B9"))
        painter.setBrush(sphere)
        painter.drawEllipse(ball, radius, radius)
        painter.setBrush(QtGui.QColor("#DEE5EF"))
        painter.drawRoundedRect(QtCore.QRectF(x + w * .17, y + h * .19, w * .24, h * .58), 5, 5)
        self._text(painter, QtCore.QRectF(rect.left() + 10, rect.top() + 8, rect.width() - 20, 19),
                   "合成场景 · 非真实影像", size=7.7, color=QtGui.QColor("#8391A3"))

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
        plane.setColorAt(0., QtGui.QColor(242, 246, 252, 170))
        plane.setColorAt(1., QtGui.QColor(228, 237, 249, 120))
        painter.setBrush(plane)
        painter.drawPolygon(QtGui.QPolygonF([project(point) for point in
                            ((-span, -span, 0), (-span, span, 0), (span, span, 0), (span, -span, 0))]))
        for index in range(-4, 5):
            painter.setPen(QtGui.QPen(QtGui.QColor("#CCD7E5" if index == 0 else "#E0E6EF"), .65))
            value = index / 4 * span
            painter.drawLine(project((value, -span, 0)), project((value, span, 0)))
            painter.drawLine(project((-span, value, 0)), project((span, value, 0)))
        relative = [tuple(v - origin for v, origin in zip(point, self._tracker_origin)) for point in self._trail]
        for index, (start, end) in enumerate(zip(relative, relative[1:])):
            color = QtGui.QColor(8, 126, 255, 30 + round(155 * (index + 1) / len(relative)))
            painter.setPen(QtGui.QPen(color, 1.65, QtCore.Qt.PenStyle.SolidLine, QtCore.Qt.PenCapStyle.RoundCap))
            painter.drawLine(project(start), project(end))
        current = relative[-1]
        at = project(current)
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        aura = QtGui.QRadialGradient(at, 19)
        aura.setColorAt(0., QtGui.QColor(8, 126, 255, 27))
        aura.setColorAt(1., QtGui.QColor(8, 126, 255, 0))
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
        painter.setBrush(QtGui.QColor("#FFFFFF"))
        painter.drawEllipse(at, 4.2, 4.2)
        self._text(painter, QtCore.QRectF(rect.left() + 4, rect.top() + 1, rect.width() - 8, 17),
                   "相对首帧 · 拖动查看", size=7.7)

    def _paint_glove(self, painter, rect):
        if not self._frame.get("relative"):
            self._text(painter, rect, "没有可用骨架，请查看 3D 手形", alignment=QtCore.Qt.AlignmentFlag.AlignCenter)
            return
        # A quiet drafting surface: decoration only, never extra anatomy.
        backdrop = QtGui.QRadialGradient(rect.center(), max(rect.width(), rect.height()) * .55)
        backdrop.setColorAt(0., QtGui.QColor(224, 237, 253, 112))
        backdrop.setColorAt(1., QtGui.QColor(244, 247, 251, 0))
        painter.fillRect(rect, backdrop)
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.setBrush(QtGui.QColor(128, 153, 185, 42))
        for x in range(int(rect.left()) + 8, int(rect.right()), 16):
            for y in range(int(rect.top()) + 23, int(rect.bottom()), 16):
                painter.drawEllipse(QtCore.QPointF(x, y), .55, .55)
        relative = self._frame["relative"]
        points = [self._project(point) for point in relative]
        span = self._glove_span
        scale = min(rect.width() / (span * 1.05), rect.height() / (span * 1.15)) * self._zoom
        center = QtCore.QPointF(rect.center().x(), rect.top() + rect.height() * .89)
        positions = [center + QtCore.QPointF(x * scale, y * scale) for x, y, _ in points]
        painter.setPen(QtGui.QPen(QtGui.QColor(92, 145, 208, 80), .7))
        palm = QtGui.QLinearGradient(rect.topLeft(), rect.bottomRight())
        palm.setColorAt(0., QtGui.QColor(213, 231, 252, 75))
        palm.setColorAt(1., QtGui.QColor(8, 126, 255, 24))
        painter.setBrush(palm)
        painter.drawPolygon(QtGui.QPolygonF([positions[index] for index in (0, 5, 9, 13, 17)]))
        bones = sorted(_BONES, key=lambda pair: points[pair[0]][2] + points[pair[1]][2])
        for start, end in bones:
            painter.setPen(QtGui.QPen(QtGui.QColor(8, 126, 255, 15), 5., QtCore.Qt.PenStyle.SolidLine, QtCore.Qt.PenCapStyle.RoundCap))
            painter.drawLine(positions[start], positions[end])
            painter.setPen(QtGui.QPen(QtGui.QColor("#536A84"), 1.65, QtCore.Qt.PenStyle.SolidLine, QtCore.Qt.PenCapStyle.RoundCap))
            painter.drawLine(positions[start], positions[end])
        for index, position in enumerate(positions):
            tip = index in (4, 8, 12, 16, 20)
            if tip:
                painter.setPen(QtCore.Qt.PenStyle.NoPen)
                painter.setBrush(QtGui.QColor(8, 126, 255, 17))
                painter.drawEllipse(position, 6.5, 6.5)
            painter.setPen(QtGui.QPen(_ACCENT if tip else QtGui.QColor("#7896B8"), 1.))
            painter.setBrush(_ACCENT if tip else QtGui.QColor("#FFFFFF"))
            painter.drawEllipse(position, 2.7 if tip else 2.15, 2.7 if tip else 2.15)
        self._text(painter, QtCore.QRectF(rect.left() + 3, rect.top(), rect.width() - 6, 17),
                   "21 关节点 · 腕部相对 · 非 MANO 网格", size=7.7)

    @staticmethod
    def _heatmap_rect(rect):
        side = max(1., min(rect.height() - 13, rect.width() - 48))
        return QtCore.QRectF(rect.center().x() - side / 2 - 5, rect.center().y() - side / 2 - 3, side, side)

    def _paint_tactile(self, painter, rect):
        if self.tactile_view == "hand":
            self._paint_tactile_hand(painter, rect)
            return
        if not self.tactile_hand_available:
            note = self._tactile_layout_note.split("；", 1)[0]
            self._text(painter, QtCore.QRectF(rect.left() + 3, rect.top(), rect.width() - 6, 14),
                       note, size=7.2)
            rect = rect.adjusted(0, 15, 0, 0)
        matrix = self._heatmap_rect(rect)
        cell = matrix.width() / 20
        gap = min(1.2, cell * .14)
        painter.setPen(QtGui.QPen(QtGui.QColor("#DCE3EC"), .6))
        painter.setBrush(QtGui.QColor("#FFFFFF"))
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
        for position, rgb in _HEAT_STOPS:
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
                   f"候选示意 · {geometry.anchor_count} 锚点 · 其余未验证", size=7.2)
        hand_rect = self._hand_rect(rect)
        transform = geometry.transform(hand_rect)
        painter.setPen(QtGui.QPen(QtGui.QColor("#B5C9DD"), 1.))
        painter.setBrush(QtGui.QColor("#EEF4FA"))
        painter.drawPath(transform.map(geometry.outline))
        for point, path in geometry.cells:
            painter.setPen(QtCore.Qt.PenStyle.NoPen)
            painter.setBrush(self._cell_colors[point.row * 20 + point.col])
            mapped = transform.map(path)
            painter.drawPath(mapped)
            if point.verified_anchor and transform.m11() >= 6:
                painter.setPen(QtGui.QPen(QtGui.QColor("#466D91"), .7))
                painter.setBrush(QtCore.Qt.BrushStyle.NoBrush)
                painter.drawPath(mapped)
        legend = QtCore.QRectF(rect.right() - 9, hand_rect.top() + 5, 5, max(1., hand_rect.height() - 10))
        gradient = QtGui.QLinearGradient(legend.bottomLeft(), legend.topLeft())
        for position, rgb in _HEAT_STOPS:
            gradient.setColorAt(position, QtGui.QColor(*rgb))
        painter.setBrush(gradient)
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.drawRoundedRect(legend, 2.5, 2.5)

    def _paint_footer(self, painter):
        if self._preview_error:
            label = "预览数据无效 · 保留上一画面" if self._frame is not None else "预览暂不可用"
        elif self._frame is None:
            label = "数据接入后自动显示"
        elif self.kind == "camera":
            label = "几何演示 · 非真实相机" if self._mode == "demo" else f"{self._frame['width']} × {self._frame['height']} · JPEG"
        elif self.kind == "tracker":
            label = "  ".join(f"{axis} {value:+.3f}" for axis, value in zip("XYZ", self._frame["position"])) + " m"
        elif self.kind == "glove":
            label = ("示意骨架 · 非求解结果" if self._mode == "demo" else
                     "MANO 参数重建 · 腕部相对，非世界定位" if self.glove_view == "3d" else
                     "测量关节点 · 拖动旋转 · 双击复位")
        else:
            count = sum(self._frame["valid_mask"])
            label = (f"历史 {count}/400 · ADC" if self._held else f"有效 {count}/400 · ADC")
            label += f"   {self._levels[0]:g}—{self._levels[1]:g}"
        self._text(painter, QtCore.QRectF(14, self.height() - 23, max(0, self.width() - 28), 16), label,
                   color=QtGui.QColor("#9C7C4E") if self._preview_error else _MUTED, size=7.7)

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
                value = f"{self._frame['values'][index]:g} ADC" if valid else "无效 / 未观测"
                description = f"row {row}, col {col} · {value}" + (" · 历史画面" if self._held else "")
                if point is not None:
                    region = {"palm": "手掌（掌心 / 掌根）", "thumb": "拇指", "index": "食指", "middle": "中指",
                              "ring": "无名指", "little": "小指"}[point.region]
                    description += f" · {region} · " + ("已观测锚点" if point.verified_anchor else "候选位置未验证")
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
