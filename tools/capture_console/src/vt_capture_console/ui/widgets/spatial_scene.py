"""OpenGL spatial viewport with a readable QPainter projection fallback."""
from __future__ import annotations

from ...i18n import tr

import math
import os
import time
from functools import lru_cache

from PySide6.QtCore import QEvent, QPointF, QRectF, Qt, QTimer, Signal
from PySide6.QtGui import (QColor, QGradient, QMatrix4x4, QPen, QPolygonF, QQuaternion,
                          QRadialGradient, QVector3D, QVector4D)
from ...i18n.widgets import QApplication, QFrame, QHBoxLayout, QLabel, QVBoxLayout, QWidget
from ...i18n.widgets import QPainter

from ...viewmodels.spatial import display_point, entity_color, transform
from ..gpu_status import OpenGLRenderer, inspect_current_context
from .spatial_glyphs import local_glyph
from .spatial_labels import layout_entity_labels, draw_entity_labels


BOX_EDGES = ((0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
             (0, 4), (1, 5), (2, 6), (3, 7))
FRUSTUM_EDGES = ((0, 1), (0, 2), (0, 3), (0, 4), (1, 2), (2, 3), (3, 4), (4, 1))
BOX_FACES = ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5),
             (2, 3, 7, 6), (3, 0, 4, 7))
FRUSTUM_FACES = ((0, 1, 2), (0, 2, 3), (0, 3, 4), (0, 4, 1), (1, 4, 3, 2))


def _pose_signature(pose):
    pose = pose or {}
    return (tuple(pose.get("translation_m", (0., 0., 0.))),
            tuple(pose.get("quaternion_xyzw", (0., 0., 0., 1.))))


def entity_geometry(entity):
    """Fixed-size illustrative body, positioned using the measured pose only."""
    if "role" in entity:
        return [transform(p, entity["position_m"], entity["quaternion_xyzw"])
                for p in local_glyph().bounds]
    size = .032
    corners = ((-size, -size, -size), (size, -size, -size), (size, size, -size), (-size, size, -size),
               (-size, -size, size), (size, -size, size), (size, size, size), (-size, size, size))
    return [transform(p, entity["position_m"], entity["quaternion_xyzw"]) for p in corners]


@lru_cache(maxsize=12)
def _tracker_body(position, rotation):
    """Transform shared vertices once per measured pose; bound the display cache."""
    glyph = local_glyph()
    points = {point for surface in glyph.surfaces for point in surface.vertices}
    points.update(point for line in glyph.lines for point in (line.start, line.end))
    world = {point: transform(point, position, rotation) for point in points}
    return (tuple((tuple(world[p] for p in surface.vertices), surface.color, surface.alpha)
                  for surface in glyph.surfaces),
            tuple((world[line.start], world[line.end], line.color, line.width) for line in glyph.lines))


def _glyph_color(token, model, entity):
    selected = entity.get("id") == model.selected_id
    return {"body": "#555555" if selected else "#3b3b3b",
            "light": "#dedede" if selected else "#999999", "dark": "#111111",
            "accent": visible_entity_color(model, entity)}[token]


def scene_bounds_points(model):
    """Fit bodies and calibrated frusta, excluding the infinite reference grid."""
    points = []
    for entity in model.entities():
        points.extend(entity_geometry(entity))
        if "frustum" in model.layers:
            points.extend(transform(p, entity["position_m"], entity["quaternion_xyzw"])
                          for p in entity.get("frustum_vertices", ()))
    return [display_point(model.frame, point) for point in points]


def visible_entity_color(model, entity):
    if (entity.get("state") in {"waiting", "offline", "delayed", "stale", "invalid"} or entity.get("valid") is False
            or (not model.source_available and not model.frozen)):
        return "#adadad"
    return entity_color(entity)


def scene_surfaces(model):
    """Schematic device shells and calibrated frusta at measured poses."""
    surfaces = []
    if not model.frame:
        return surfaces
    for entity in model.entities():
        color = visible_entity_color(model, entity)
        selected = entity.get("id") == model.selected_id
        if "role" in entity:
            body, _lines = _tracker_body(tuple(entity["position_m"]), tuple(entity["quaternion_xyzw"]))
            projected = {point: display_point(model.frame, point) for polygon, _token, _alpha in body for point in polygon}
            surfaces.extend(([projected[p] for p in polygon], _glyph_color(token, model, entity), alpha)
                            for polygon, token, alpha in body)
        else:
            body = [display_point(model.frame, point) for point in entity_geometry(entity)]
            for index, face in enumerate(BOX_FACES):
                surfaces.append(([body[i] for i in face], color, (.38 if selected else .24) + index % 3 * .035))
        vertices = entity.get("frustum_vertices") or ()
        if "frustum" in model.layers and len(vertices) in (5, 8):
            frustum = [display_point(model.frame, transform(p, entity["position_m"], entity["quaternion_xyzw"]))
                       for p in vertices]
            for index, face in enumerate(FRUSTUM_FACES if len(vertices) == 5 else BOX_FACES):
                surfaces.append(([frustum[i] for i in face], color, .032 + index % 3 * .013))
    return surfaces


def camera_components(point, azimuth, elevation):
    """Camera-aligned coordinates, shared by fit calculations and projection."""
    x, y, z = point
    a, e = math.radians(azimuth), math.radians(elevation)
    horizontal = math.cos(a)*x + math.sin(a)*y
    return (-math.sin(a)*x + math.cos(a)*y,
            -math.sin(e)*horizontal + math.cos(e)*z,
            math.cos(e)*horizontal + math.sin(e)*z)


def camera_fit_bounds(points, azimuth, elevation):
    """Center the projected bounding area without changing a single source pose."""
    projected = [camera_components(point, azimuth, elevation) for point in points]
    center = tuple((min(p[i] for p in projected)+max(p[i] for p in projected))/2 for i in range(3))
    right, up, depth = center
    a, e = math.radians(azimuth), math.radians(elevation)
    target = (-math.sin(a)*right-math.sin(e)*math.cos(a)*up+math.cos(e)*math.cos(a)*depth,
              math.cos(a)*right-math.sin(e)*math.sin(a)*up+math.cos(e)*math.sin(a)*depth,
              math.cos(e)*up+math.sin(e)*depth)
    return target, [tuple(point[i]-center[i] for i in range(3)) for point in projected]


def display_cloud_data(cloud, *, max_points=50000, max_depth=0.):
    """Bound display uploads using the cloud's optical Z; source data stays intact."""
    import numpy as np
    positions, colors = cloud["positions"], cloud["colors"]
    if max_depth > 0:
        indices = np.flatnonzero(positions[:, 2] <= max_depth)
        stride = max(1, math.ceil(len(indices)/max_points))
        indices = indices[::stride]
        return positions[indices], colors[indices]
    stride = max(1, math.ceil(len(positions)/max_points))
    return positions[::stride], colors[::stride]


def scene_lines(model, *, include_grid=True, include_entities=True):
    """World-space lines with colors and widths, shared by both renderers."""
    frame = model.frame
    if not frame:
        return []
    result = []

    def add(a, b, color, width=1.):
        result.append((display_point(frame, a), display_point(frame, b), color, width))

    if include_grid and "grid" in model.layers:
        for i in range(-10, 11):
            t = i / 5
            add((-2, t, 0), (2, t, 0), "#303030")
            add((t, -2, 0), (t, 2, 0), "#303030")
        for point, color in (((.4, 0, 0), "#c55d63"), ((0, .4, 0), "#54996c"), ((0, 0, .4), "#537fd4")):
            add((0, 0, 0), point, color, 2.)
    for entity in model.entities() if include_entities else ():
        position, rotation = entity["position_m"], entity["quaternion_xyzw"]
        color = visible_entity_color(model, entity)
        selected = model.selected_id == entity.get("id")
        if "role" in entity:
            _surfaces, lines = _tracker_body(tuple(position), tuple(rotation))
            for a, b, token, width in lines:
                add(a, b, _glyph_color(token, model, entity), width + (.5 if selected else 0.))
        else:
            world = entity_geometry(entity)
            for a, b in BOX_EDGES:
                add(world[a], world[b], color, 3. if selected else 2.)
        if "axes" in model.layers:
            length = .072 if "role" in entity else .13
            for axis, axis_color in enumerate(("#cb4c55", "#40a46a", "#487ddd")):
                point = tuple(length if index == axis else 0. for index in range(3))
                add(position, transform(point, position, rotation), axis_color, 2.)
                for sign in (-1, 1):
                    tip = list(point)
                    tip[axis] -= .012
                    tip[(axis+1) % 3] += sign*.004
                    add(transform(point, position, rotation), transform(tip, position, rotation), axis_color, 1.5)
        if "trail" in model.layers:
            segments = [segment.get("points", ()) if hasattr(segment, "get") else segment
                        for segment in entity.get("trail_segments", ())]
            count = sum(max(0, len(points)-1) for points in segments)
            step = 0
            for points in segments:
                for a, b in zip(points, points[1:]):
                    step += 1
                    fading = QColor(color)
                    fading.setAlphaF(.12 + .68*step/max(1, count))
                    add(a, b, fading.name(QColor.NameFormat.HexArgb), 1.5)
        if "frustum" in model.layers:
            vertices = entity.get("frustum_vertices") or ()
            if len(vertices) in (5, 8):
                vertices = [transform(p, position, rotation) for p in vertices]
                for a, b in FRUSTUM_EDGES if len(vertices) == 5 else BOX_EDGES:
                    add(vertices[a], vertices[b], color, 1.5)
    return result


class ProjectionViewport(QWidget):
    selected = Signal(str)
    interacted = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.model = None
        self.azimuth, self.elevation = -55., 25.
        self.target = (0., 0., .7)
        self.scale = 180.
        self._press = self._last = None
        self._labels = []
        self._label_placements = []
        self._clouds = {}
        self._cloud_cache = {}
        self._point_size = 2.
        self._cloud_color = "source"
        self.setMinimumSize(230, 150)
        self.setMouseTracking(True)

    def _project(self, point):
        right, up, _ = camera_components(tuple(point[i]-self.target[i] for i in range(3)),
                                         self.azimuth, self.elevation)
        return QPointF(self.width()/2+right*self.scale, self.height()/2-up*self.scale)

    def set_model(self, model):
        self.model = model
        self.update()

    def fit(self):
        if not self.model or not self.model.entities():
            return
        points = scene_bounds_points(self.model)
        self.target, projected = camera_fit_bounds(points, self.azimuth, self.elevation)
        extent_x = max(.12, 2 * max(abs(p[0]) for p in projected))
        extent_y = max(.12, 2 * max(abs(p[1]) for p in projected))
        self.scale = .62 * min(self.width()/extent_x, self.height()/extent_y)
        self.update()

    def standard_view(self, view):
        self.azimuth, self.elevation = {"top": (-90., 90.), "front": (-90., 0.),
                                       "side": (0., 0.), "reset": (-55., 25.)}[view]
        self.update()

    def orbit(self, degrees):
        self.azimuth = (self.azimuth + degrees) % 360
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        # A soft elliptical scrim lets the workbench's ambient scene continue
        # behind the projection instead of introducing another black rectangle.
        background = QRadialGradient(.5, .48, .56)
        background.setCoordinateMode(QGradient.CoordinateMode.ObjectBoundingMode)
        background.setColorAt(0., QColor(13, 13, 13, 70))
        background.setColorAt(.75, QColor(13, 13, 13, 20))
        background.setColorAt(.95, QColor(13, 13, 13, 0))
        background.setColorAt(1., QColor(13, 13, 13, 0))
        painter.fillRect(self.rect(), background)
        self._labels = []
        self._label_placements = []
        if not self.model or not self.model.frame:
            painter.setPen(QColor("#9b9b9b"))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, tr('等待公共空间数据'))
            return
        for key, cloud in self._clouds.items():
            header = cloud["header"]
            pose = header["world_from_color"]
            _signature, points, colors = self._cloud_cache[key]
            # This portable fallback deliberately samples a bounded projection.
            stride = max(1, len(points)//600)
            for point, color in zip(points[::stride], colors[::stride]):
                world = transform(point, pose["translation_m"], pose["quaternion_xyzw"])
                pen_color = QColor.fromRgbF(*[float(c) for c in color]) if self._cloud_color == "rgb" else QColor(entity_color({"id": key}))
                painter.setPen(QPen(pen_color, self._point_size))
                painter.drawPoint(self._project(display_point(self.model.frame, world)))
        for a, b, color, width in scene_lines(self.model, include_entities=False):
            painter.setPen(QPen(QColor(color), width))
            painter.drawLine(self._project(a), self._project(b))
        surfaces = scene_surfaces(self.model)
        surfaces.sort(key=lambda surface: sum(camera_components(p, self.azimuth, self.elevation)[2]
                                              for p in surface[0])/len(surface[0]))
        painter.setPen(Qt.PenStyle.NoPen)
        for vertices, color, alpha in surfaces:
            shade = QColor(color)
            shade.setAlphaF(alpha)
            painter.setBrush(shade)
            painter.drawPolygon(QPolygonF([self._project(point) for point in vertices]))
        for a, b, color, width in scene_lines(self.model, include_grid=False):
            painter.setPen(QPen(QColor(color), width))
            painter.drawLine(self._project(a), self._project(b))
        projected = []
        for entity in self.model.entities():
            center = self._project(display_point(self.model.frame, entity["position_m"]))
            self._labels.append((entity["id"], center))
            projected.append((entity, center, visible_entity_color(self.model, entity)))
        self._label_placements = layout_entity_labels(self.model, projected, QRectF(self.rect()))
        draw_entity_labels(painter, self._label_placements)
        painter.setPen(QColor("#9b9b9b"))
        painter.drawText(16, self.height()-16, tr('公共坐标 · 米    网格 0.2 m    实体外形为示意'))

    def mousePressEvent(self, event):
        self.interacted.emit()
        self._press = self._last = event.position()

    def mouseMoveEvent(self, event):
        if self._last is None or not event.buttons():
            return
        delta = event.position()-self._last
        self._last = event.position()
        if event.buttons() & Qt.MouseButton.MiddleButton or event.modifiers() & Qt.KeyboardModifier.ShiftModifier:
            a = math.radians(self.azimuth)
            self.target = (self.target[0]+math.sin(a)*delta.x()/self.scale,
                           self.target[1]-math.cos(a)*delta.x()/self.scale,
                           self.target[2]+delta.y()/self.scale)
        elif event.buttons() & Qt.MouseButton.LeftButton:
            self.azimuth -= delta.x()*.5
            self.elevation = max(-89., min(89., self.elevation+delta.y()*.4))
        self.update()

    def mouseReleaseEvent(self, event):
        if self._press is not None and (event.position()-self._press).manhattanLength() < 6:
            plate = next((p for p in self._label_placements if p.rect.contains(event.position())), None)
            if plate:
                self.selected.emit(plate.identifier)
            else:
                closest = min(self._labels, key=lambda item: (item[1]-event.position()).manhattanLength(), default=None)
                if closest and (closest[1]-event.position()).manhattanLength() < 35:
                    self.selected.emit(closest[0])
        self._press = self._last = None

    def mouseDoubleClickEvent(self, event):
        self.interacted.emit()
        self.standard_view("reset")
        self.fit()

    def wheelEvent(self, event):
        self.interacted.emit()
        self.scale = min(4000., max(15., self.scale*math.exp(event.angleDelta().y()/1200)))
        self.update()

    def set_clouds(self, clouds, color_mode="source", point_size=2., max_points=50000, max_depth=0.):
        self._clouds, self._cloud_color = clouds, color_mode
        self._point_size = point_size
        self._cloud_cache = {key: value for key, value in self._cloud_cache.items() if key in clouds}
        for key, cloud in clouds.items():
            header = cloud["header"]
            signature = (header["worker_instance_id"], header["cloud_sequence"], max_points, max_depth)
            cached = self._cloud_cache.get(key)
            if cached is None or cached[0] != signature:
                self._cloud_cache[key] = (signature, *display_cloud_data(cloud, max_points=max_points, max_depth=max_depth))
        self.update()

    def clear_clouds(self):
        """Release source arrays even when a hidden scene cannot paint."""
        self._clouds = {}
        self._cloud_cache.clear()
        self.update()


def create_gl_view(parent):
    """Load PySide-bound OpenGL lazily; offscreen never creates a GL context."""
    # A Wayland session may run this window through XWayland. Match Qt's actual
    # platform instead of letting PyOpenGL infer EGL from WAYLAND_DISPLAY alone.
    if QApplication.platformName() == "xcb":
        platform = "egl" if os.environ.get("QT_XCB_GL_INTEGRATION") == "xcb_egl" else "glx"
        os.environ.setdefault("PYOPENGL_PLATFORM", platform)
    import numpy as np
    import pyqtgraph.opengl as gl

    class LabelOverlay(QWidget):
        """Qt overlay keeps text outside the OpenGL translucent blend state."""
        def __init__(self, viewport):
            super().__init__(viewport)
            self.viewport = viewport
            self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
            self.setStyleSheet("background: transparent;")

        def paintEvent(self, event):
            viewport = self.viewport
            viewport._label_placements = []
            if not viewport.model or not viewport.model.frame:
                return
            painter = QPainter(self)
            painter.setRenderHint(QPainter.RenderHint.TextAntialiasing)
            matrix = viewport.projectionMatrix()*viewport.viewMatrix()
            projected = []
            for entity in viewport.model.entities():
                position = display_point(viewport.model.frame, entity["position_m"])
                point = matrix.map(QVector4D(*position, 1.))
                if point.w() > 0:
                    screen = QPointF((point.x()/point.w()+1)*self.width()/2,
                                     (1-point.y()/point.w())*self.height()/2)
                    projected.append((entity, screen, visible_entity_color(viewport.model, entity)))
            viewport._label_placements = layout_entity_labels(viewport.model, projected, QRectF(self.rect()))
            draw_entity_labels(painter, viewport._label_placements)
            painter.setPen(QColor("#9b9b9b"))
            painter.drawText(16, self.height()-16, tr('公共坐标 · 米    网格 0.2 m    实体外形为示意'))

    class GLViewport(gl.GLViewWidget):
        selected = Signal(str)
        failed = Signal(str)
        renderer_ready = Signal(object)
        interacted = Signal()

        def __init__(self):
            super().__init__(parent)
            surface_format = self.format()
            surface_format.setSamples(4)
            surface_format.setDepthBufferSize(24)
            self.setFormat(surface_format)
            self.model = None
            self._context_failed = False
            self._press = None
            self._cloud_items = {}
            self._label_placements = []
            self.setBackgroundColor("#000000")
            self.setCameraPosition(distance=2.8, elevation=25, azimuth=-55)
            self.lines = gl.GLLinePlotItem(pos=np.empty((0, 3)), mode="lines", width=1.2, antialias=True, glOptions="translucent")
            self.addItem(self.lines)
            self.emphasis = gl.GLLinePlotItem(pos=np.empty((0, 3)), mode="lines", width=2.4, antialias=True, glOptions="translucent")
            self.addItem(self.emphasis)
            self.bodies = gl.GLMeshItem(vertexes=np.empty((0, 3)), faces=np.empty((0, 3), dtype=int),
                                       smooth=False, drawEdges=False, glOptions="opaque")
            self.bodies.setDepthValue(-20)
            self.addItem(self.bodies)
            self.surfaces = gl.GLMeshItem(vertexes=np.empty((0, 3)), faces=np.empty((0, 3), dtype=int),
                                          smooth=False, drawEdges=False, glOptions="translucent")
            self.surfaces.setDepthValue(-10)
            self.addItem(self.surfaces)
            self.selection = gl.GLScatterPlotItem(pos=np.empty((0, 3)), color=(1., 1., 1., 1.), size=11., pxMode=True, glOptions="opaque")
            self.addItem(self.selection)
            self.label_overlay = LabelOverlay(self)

        def initializeGL(self):
            try:
                from OpenGL import contextdata
                contextdata.getContext()
                super().initializeGL()
                if not self.isValid():
                    self._context_failed = True
                    self.failed.emit(tr('OpenGL 上下文不可用'))
                else:
                    # initializeGL runs with this viewport's context current;
                    # never infer hardware acceleration from isValid alone.
                    self._context_failed = False
                    self.renderer_info = inspect_current_context()
                    self.renderer_ready.emit(self.renderer_info)
            except Exception as exc:
                self._context_failed = True
                self.failed.emit(str(exc))

        def paintGL(self, *args, **kwargs):
            if not self._context_failed:
                super().paintGL(*args, **kwargs)

        def resizeEvent(self, event):
            super().resizeEvent(event)
            self.label_overlay.setGeometry(self.rect())

        def setCameraPosition(self, *args, **kwargs):
            super().setCameraPosition(*args, **kwargs)
            if hasattr(self, "label_overlay"):
                self.label_overlay.update()

        def showEvent(self, event):
            super().showEvent(event)
            QTimer.singleShot(150, self._check_context)

        def _check_context(self):
            if self.isVisible() and not self.isValid():
                self.failed.emit(tr('OpenGL 上下文不可用'))

        def set_model(self, model):
            self.model = model
            segments = scene_lines(model)
            for item, thick in ((self.lines, False), (self.emphasis, True)):
                positions, colors = [], []
                for a, b, color, width in segments:
                    if (width >= 2.) != thick:
                        continue
                    positions.extend((a, b))
                    rgba = QColor(color).getRgbF()
                    colors.extend((rgba, rgba))
                item.setData(pos=np.asarray(positions, dtype=float).reshape(-1, 3),
                             color=np.asarray(colors, dtype=float).reshape(-1, 4))
            surfaces = scene_surfaces(model)
            for item, opaque in ((self.bodies, True), (self.surfaces, False)):
                vertices, faces, face_colors = [], [], []
                for polygon, color, alpha in surfaces:
                    if (alpha >= 1.) != opaque:
                        continue
                    offset = len(vertices)
                    vertices.extend(polygon)
                    rgba = (*QColor(color).getRgbF()[:3], alpha)
                    for index in range(1, len(polygon)-1):
                        faces.append((offset, offset+index, offset+index+1))
                        face_colors.append(rgba)
                item.setMeshData(vertexes=np.asarray(vertices, dtype=float).reshape(-1, 3),
                                 faces=np.asarray(faces, dtype=int).reshape(-1, 3),
                                 faceColors=np.asarray(face_colors, dtype=float).reshape(-1, 4))
            selected_position = []
            for entity in model.entities():
                identifier = entity["id"]
                position = display_point(model.frame, entity["position_m"])
                if identifier == model.selected_id:
                    selected_position.append(position)
            self.selection.setData(pos=np.asarray(selected_position, dtype=float).reshape(-1, 3))
            self.label_overlay.update()
            self.update()

        def fit(self):
            if not self.model or not self.model.entities():
                return
            points = scene_bounds_points(self.model)
            center, projected = camera_fit_bounds(points, self.opts["azimuth"], self.opts["elevation"])
            self.opts["center"] = QVector3D(*center)
            tangent = math.tan(math.radians(self.opts["fov"])/2)
            aspect = self.width()/max(1, self.height())
            distance = max(max(z + abs(x)/(.62*tangent), z + abs(y)*aspect/(.62*tangent))
                           for x, y, z in projected)
            self.setCameraPosition(distance=max(.25, distance))

        def standard_view(self, view):
            azimuth, elevation = {"top": (-90., 90.), "front": (-90., 0.),
                                  "side": (0., 0.), "reset": (-55., 25.)}[view]
            self.setCameraPosition(azimuth=azimuth, elevation=elevation)

        def mousePressEvent(self, event):
            self.interacted.emit()
            self._press = event.position()
            super().mousePressEvent(event)

        def mouseMoveEvent(self, event):
            super().mouseMoveEvent(event)
            self.label_overlay.update()

        def mouseReleaseEvent(self, event):
            if self.model and self._press is not None and (event.position()-self._press).manhattanLength() < 6:
                plate = next((p for p in self._label_placements if p.rect.contains(event.position())), None)
                matrix = self.projectionMatrix()*self.viewMatrix()
                choices = []
                for entity in self.model.entities():
                    p = matrix.map(QVector4D(*display_point(self.model.frame, entity["position_m"]), 1.))
                    if p.w() > 0:
                        point = QPointF((p.x()/p.w()+1)*self.width()/2, (1-p.y()/p.w())*self.height()/2)
                        choices.append(((point-event.position()).manhattanLength(), entity["id"]))
                nearest = min(choices, default=(100, ""))
                if plate:
                    self.selected.emit(plate.identifier)
                elif nearest[0] < 35:
                    self.selected.emit(nearest[1])
            self._press = None
            super().mouseReleaseEvent(event)

        def mouseDoubleClickEvent(self, event):
            self.interacted.emit()
            self.standard_view("reset")
            self.fit()

        def wheelEvent(self, event):
            self.interacted.emit()
            super().wheelEvent(event)
            self.label_overlay.update()

        def orbit(self, azimuth, elevation=0.):
            super().orbit(azimuth, elevation)
            self.label_overlay.update()

        def set_clouds(self, clouds, color_mode="source", point_size=2., max_points=50000, max_depth=0.):
            # Missing sources have ended their display lifetime. Keep objects
            # for continuing cameras, but never retain arbitrary historical IDs.
            self._release_cloud_items(set(self._cloud_items) - clouds.keys())
            for key, cloud in clouds.items():
                item = self._cloud_items.get(key)
                if item is None:
                    item = gl.GLScatterPlotItem(glOptions="opaque")
                    self.addItem(item)
                    self._cloud_items[key] = item
                signature = (cloud["header"]["worker_instance_id"], cloud["header"]["cloud_sequence"], color_mode, point_size, max_points, max_depth)
                if getattr(item, "_spatial_signature", None) != signature:
                    positions, colors = display_cloud_data(cloud, max_points=max_points, max_depth=max_depth)
                    color = colors if color_mode == "rgb" else QColor(entity_color({"id": key})).getRgbF()
                    item.setData(pos=positions, color=color, size=point_size, pxMode=True)
                    item._spatial_signature = signature
                matrix = QMatrix4x4()
                for pose in (self.model.frame.get("display_from_world", {}), cloud["header"]["world_from_color"]):
                    matrix.translate(*pose.get("translation_m", (0., 0., 0.)))
                    x, y, z, w = pose.get("quaternion_xyzw", (0., 0., 0., 1.))
                    matrix.rotate(QQuaternion(w, x, y, z))
                item.setTransform(matrix)
                item.setVisible(True)

        def _release_cloud_items(self, identifiers):
            items = [self._cloud_items.pop(key) for key in identifiers]
            if not items:
                return
            # pyqtgraph 0.13 scatter items own a point-sprite texture but do not
            # delete it on removeItem(). Free it while our context is current;
            # after context loss the driver owns its destruction instead.
            valid_context = self.isValid()
            if valid_context:
                self.makeCurrent()
            try:
                for item in items:
                    self.removeItem(item)
                    texture = getattr(item, "pointTexture", None)
                    if texture is not None:
                        if valid_context:
                            from OpenGL.GL import glDeleteTextures
                            glDeleteTextures([texture])
                        item.pointTexture = None
                    item.setData(pos=np.empty((0, 3), dtype="f4"), color=(1., 1., 1., 1.))
                    item.deleteLater()
            finally:
                if valid_context:
                    self.doneCurrent()

        def clear_clouds(self):
            self._release_cloud_items(tuple(self._cloud_items))

        def closeEvent(self, event):
            self.clear_clouds()
            super().closeEvent(event)

    return GLViewport()


class SpatialScene(QWidget):
    selected = Signal(str)
    cruise_changed = Signal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("spatialViewport")
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.model = None
        self._fitted = False
        self._fit_identity = None
        self._model_pending = False
        self._render_signature = None
        self._pending_clouds = None
        self._cloud_signature = None
        self._motion_enabled = True
        self._cruising = False
        self._last_motion = 0.
        self._observed_window = None
        self.motion_timer = QTimer(self)
        self.motion_timer.setInterval(40)  # Bounded 25 fps camera motion, independent of source refresh.
        self.motion_timer.timeout.connect(self._advance_camera)
        self.render_mode = "projection"
        self.renderer_info = None
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        header = QFrame()
        header.setObjectName("spatialCanvasHeader")
        heading = QHBoxLayout(header)
        heading.setContentsMargins(10, 6, 10, 6)
        title = QLabel(tr('空间观察'))
        title.setObjectName("spatialCanvasTitle")
        heading.addWidget(title)
        heading.addStretch()
        self.note = QLabel()
        self.note.setObjectName("spatialCanvasBadge")
        heading.addWidget(self.note)
        layout.addWidget(header)
        use_gl = QApplication.platformName() not in {"offscreen", "minimal"} and os.environ.get("VT_SPATIAL_RENDERER") != "projection"
        if use_gl:
            try:
                self.viewport = create_gl_view(self)
                self.viewport.failed.connect(self._fallback, Qt.ConnectionType.QueuedConnection)
                self.viewport.renderer_ready.connect(self._set_renderer_info, Qt.ConnectionType.QueuedConnection)
                self.render_mode = "opengl"
            except Exception as exc:
                self.viewport = ProjectionViewport(self)
                self.note.setToolTip(str(exc))
        else:
            self.viewport = ProjectionViewport(self)
        self.note.setText(tr('三维 · 待确认') if self.render_mode == "opengl" else tr('投影 · CPU'))
        self.note.setToolTip(tr('OpenGL 三维场景') if self.render_mode == "opengl" else tr('OpenGL 不可用，当前使用可交互的空间投影预览。精确坐标见场景面板中的数据页。'))
        self.viewport.selected.connect(self.selected)
        self.viewport.interacted.connect(self._manual_camera)
        layout.addWidget(self.viewport, 1)

    def _set_renderer_info(self, info: OpenGLRenderer):
        # A failed/deleted GL viewport may still have a queued status signal.
        if self.render_mode != "opengl":
            return
        self.renderer_info = info
        if info.acceleration == "hardware":
            self.note.setText(tr('三维 · GPU'))
            explanation = tr('当前三维上下文由 GPU 渲染')
        elif info.acceleration == "software":
            self.note.setText(tr('三维 · CPU'))
            explanation = tr('当前三维上下文使用 CPU 软件渲染，GPU 加速尚未生效')
        else:
            self.note.setText(tr('三维 · 待确认'))
            explanation = tr('当前三维上下文的硬件加速状态尚未确认')
        details = tr('渲染器：{renderer}\n厂商：{vendor}\nOpenGL：{version}',
                     renderer=info.renderer or "—", vendor=info.vendor or "—", version=info.version or "—")
        self.note.setToolTip(explanation + "\n" + details)

    def _fallback(self, reason):
        if self.render_mode != "opengl":
            return
        old = self.viewport
        old.clear_clouds()
        self.layout().removeWidget(old)
        old.hide()
        old.deleteLater()
        self.viewport = ProjectionViewport(self)
        self.viewport.selected.connect(self.selected)
        self.viewport.interacted.connect(self._manual_camera)
        self.layout().addWidget(self.viewport, 1)
        self.render_mode = "projection"
        self.renderer_info = None
        self.note.setText(tr('投影 · CPU'))
        self.note.setToolTip(reason)
        self._render_signature = None
        self._cloud_signature = None
        if self.model:
            self.set_model(self.model)
            if self.isVisible():
                self.viewport.fit()

    def set_model(self, model):
        self.model = model
        self.viewport.model = model
        self._model_pending = True
        self._flush_display()
        self._fit_initial()
        self._sync_motion()

    def _flush_display(self):
        """Upload only changed visible geometry; keep the latest model hidden."""
        if not self.isVisible() or self.window().isMinimized():
            return
        if self._model_pending and self.model is not None:
            model, frame = self.model, self.model.frame
            keys = ("id", "label", "role", "position_m", "quaternion_xyzw", "state", "valid",
                    "frustum_vertices", "trail_segments")
            # SpatialViewModel recursively freezes frame values, including
            # trails and frusta; keeping those immutable values avoids copies.
            # Ages/timestamps belong to telemetry, not geometry or labels.
            signature = (bool(frame), _pose_signature(frame.get("display_from_world") if frame else None),
                         frozenset(model.layers), model.selected_id, model.frozen, model.source_available,
                         tuple(("role" in entity, *(entity.get(key) for key in keys))
                               for entity in model.entities()))
            if signature != self._render_signature:
                self.viewport.set_model(model)
                self._render_signature = signature
            self._model_pending = False
        if self._pending_clouds is not None:
            clouds, color_mode, point_size, max_points, max_depth = self._pending_clouds
            frame = self.model.frame if self.model is not None else None
            signature = (color_mode, point_size, max_points, max_depth,
                         _pose_signature(frame.get("display_from_world") if frame else None),
                         tuple((key, cloud["header"]["worker_instance_id"], cloud["header"]["cloud_sequence"],
                                _pose_signature(cloud["header"]["world_from_color"])) for key, cloud in clouds.items()))
            if signature != self._cloud_signature:
                self.viewport.set_clouds(clouds, color_mode, point_size, max_points, max_depth)
                self._cloud_signature = signature
            self._pending_clouds = None

    def _fit_initial(self):
        if not self.model or not self.model.frame or not self.isVisible() or not self.model.entities():
            return
        frame = self.model.frame
        identity = tuple(str(frame.get(key)) for key in
                         ("mode", "manager_instance_id", "scene_epoch", "world_frame", "map_identity", "coordinate_epoch"))
        if not self._fitted or identity != self._fit_identity:
            self.viewport.fit()
            self._fitted, self._fit_identity = True, identity

    def set_motion_enabled(self, enabled):
        self._motion_enabled = bool(enabled)
        self._sync_motion()

    def set_cruising(self, enabled):
        enabled = bool(enabled)
        if enabled != self._cruising:
            self._cruising = enabled
            self.cruise_changed.emit(enabled)
        self._sync_motion()

    def _manual_camera(self):
        self.set_cruising(False)

    def _sync_motion(self):
        active = (self._motion_enabled and self._cruising and self.isVisible() and not self.window().isMinimized()
                  and self.model is not None
                  and not self.model.frozen and self.model.source_available and bool(self.model.entities()))
        if active and not self.motion_timer.isActive():
            self._last_motion = time.monotonic()
            self.motion_timer.start()
        elif not active:
            self.motion_timer.stop()

    def _advance_camera(self):
        now = time.monotonic()
        elapsed = min(.1, max(0., now-self._last_motion))
        self._last_motion = now
        self.viewport.orbit(elapsed*4.)

    def fit(self):
        self.viewport.fit()

    def reset_view_fit(self):
        """Fit the next visible scene, without changing layers or camera now."""
        self._fitted = False

    def standard_view(self, view):
        self.set_cruising(False)
        self.viewport.standard_view(view)
        if view == "reset":
            self.viewport.fit()

    def set_clouds(self, clouds, color_mode="source", point_size=2., max_points=50000, max_depth=0.):
        self._pending_clouds = (clouds, color_mode, point_size, max_points, max_depth)
        self._flush_display()

    def clear_clouds(self):
        """End the source lifetime immediately, independent of visibility.

        Ordinary hidden updates are coalesced until display resumes. Source
        switches and window closure instead release both pending CPU data and
        viewport resources now, so hidden windows cannot retain old recordings.
        """
        self._pending_clouds = None
        self._cloud_signature = None
        self.viewport.clear_clouds()

    def showEvent(self, event):
        super().showEvent(event)
        window = self.window()
        if window is not self._observed_window:
            if self._observed_window is not None:
                self._observed_window.removeEventFilter(self)
            self._observed_window = window
            window.installEventFilter(self)
        self._flush_display()
        QTimer.singleShot(0, self._fit_initial)
        self._sync_motion()

    def eventFilter(self, watched, event):
        if watched is self._observed_window and event.type() == QEvent.Type.WindowStateChange:
            self._flush_display()
            self._sync_motion()
        return super().eventFilter(watched, event)

    def hideEvent(self, event):
        self.motion_timer.stop()
        super().hideEvent(event)

    def closeEvent(self, event):
        self.motion_timer.stop()
        self.clear_clouds()
        super().closeEvent(event)
