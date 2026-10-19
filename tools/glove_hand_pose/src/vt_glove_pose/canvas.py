"""Software-rendered procedural 3D hand and diagnostic skeleton view."""
from __future__ import annotations
from copy import deepcopy
import math
import time
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
from .mesh import build_hand_mesh, display_root_matrix, orientation_matrix

FINGER_LABELS = {"thumb": "拇指", "index": "食指", "middle": "中指", "ring": "无名指", "little": "小指"}
FINGER_COLORS = {"thumb": "#f9bd69", "index": "#6bd5a7", "middle": "#64bdff", "ring": "#b79aff", "little": "#f095c4"}

def anatomical_display_model(model, config):
    """Correct legacy template chirality without mirroring measured rotations.

    Reflect the palm outline and finger *bases*, then redirect each chain's
    configured neutral splay about its base. Dynamic flexion/splay increments
    and the physical root rotation remain unchanged. This is display-only:
    never mutate the solver's model, saved calibration, or diagnostic stream.
    An explicit template is required; do not guess a custom thumb's heading.

    Quick-mode models marked ``anatomical_direct_v1`` already use the physical
    template for both their joint mathematics and geometry. Applying the
    historical display correction again would mirror their bases and rotate
    their thumb a second time. Legacy/unmarked models retain the old path.
    """
    if model.get("coordinate_convention") == "anatomical_direct_v1":
        return model
    if config is None or model.get("hand") not in {"right", "left"}:
        return model
    result = deepcopy(model)
    palm = result.get("palm_points", [])
    if palm:
        points = np.asarray(palm, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("palm geometry must contain xyz rows")
        points[:, 0] *= -1
        result["palm_points"] = points.tolist()
    for name, finger in result.get("fingers", {}).items():
        points = np.asarray(finger.get("points", []), dtype=float)
        if points.size == 0:
            continue
        if points.ndim != 2 or points.shape[1] != 3 or not np.all(np.isfinite(points)):
            raise ValueError("finger geometry must contain finite xyz rows")
        template = config.get("fingers", {}).get(name)
        if template is None:
            raise ValueError(f"Missing display template for {name}")
        if "neutral_splay_deg" not in template:
            raise ValueError(f"Missing neutral splay in display template for {name}")
        neutral = float(template["neutral_splay_deg"])
        if not math.isfinite(neutral):
            raise ValueError("neutral splay must be finite")
        if model["hand"] == "left":
            neutral = -neutral
        angle = math.radians(-2 * neutral)
        c, s = math.cos(angle), math.sin(angle)
        rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        base = points[0].copy()
        base[0] *= -1
        finger["points"] = ((points - points[0]) @ rotation.T + base).tolist()
    return result


class HandCanvas(QtWidgets.QWidget):
    """Render an anatomical template with the unchanged physical root attitude.

    Camera framing follows the palm center. No global hand translation is
    inferred. Surface thickness and nails are illustrative, not measurements.
    """
    def __init__(self, parent=None, *, config=None):
        super().__init__(parent)
        self.setMinimumSize(440, 380)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.model, self.mode, self.stale = {}, "live", True
        self._geometry_config = deepcopy(config)
        self._display_model = {}
        self.render_mode, self.follow_root = "solid", True
        self.yaw, self.pitch, self.zoom = -0.38, 0.28, 1.0
        self._drag, self._mesh, self._mesh_dirty = None, None, True
        self.published_monotonic_s = None
        self.paint_elapsed_ms = self.mesh_build_elapsed_ms = 0.0
        self.snapshot_to_paint_ms = None
        self.drawn_triangles = 0
        self._materials = {}
        self.setToolTip("手背：浅色、有指甲；手心：较深青调。顶部文字按当前观察方向标面，侧面不强判正反。灰/红告警仍优先。左右手名称来自配置，不靠屏幕拇指位置猜测。青环只标模型关节位置；金色为推算/越界。拖动旋转、滚轮缩放、双击复位；不估计平移。")

    def set_model(self, model: dict, *, mode="live", stale=False, published_monotonic_s=None):
        model = model or {}
        if model != self.model or bool(stale) != self.stale:
            self._mesh_dirty = True
        self.model, self.mode, self.stale = model, mode, bool(stale)
        self.published_monotonic_s = published_monotonic_s
        self.update()

    def set_geometry_config(self, config):
        """Supply display-template metadata only; does not touch an engine."""
        self._geometry_config = deepcopy(config)
        self._mesh_dirty = True
        self.update()

    def set_render_mode(self, mode):
        if mode not in {"solid", "skeleton"}:
            raise ValueError("render mode must be solid or skeleton")
        self.render_mode = mode
        self.update()

    def set_root_follow(self, enabled):
        self.follow_root = bool(enabled)
        self.update()

    def view_matrix(self):
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        return np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]]) @ np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])

    def model_to_view(self, points):
        """R_view * R_root * display point, without changing physical attitude."""
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or not np.all(np.isfinite(points)):
            raise ValueError("model points must be finite xyz rows")
        return points @ display_root_matrix(self.model, self.follow_root).T @ self.view_matrix().T

    def visible_surface(self):
        """Which *drawn* palm face points toward the viewer, not a sensor claim."""
        dorsal_z = float(self.model_to_view([[0.0, 0.0, 1.0]])[0, 2])
        return "dorsal" if dorsal_z > 0.25 else "palmar" if dorsal_z < -0.25 else "edge"

    def _root_is_valid(self):
        if not self.model.get("root_orientation_valid", False):
            return False
        try:
            orientation_matrix(self.model.get("root_orientation_wxyz"))
        except (TypeError, ValueError):
            return False
        return True

    def view_identity(self):
        hand = {"right": "右手", "left": "左手"}.get(self.model.get("hand"), "手别未标注")
        surface = {"dorsal": "手背朝向你", "palmar": "手心朝向你", "edge": "侧面视角"}[self.visible_surface()]
        qualifier = " · 历史画面" if self.stale else " · 参考外形" if not self._root_is_valid() else ""
        if self._geometry_config is None:
            qualifier += " · 显示模板未提供"
        return hand + " · " + surface + qualifier

    def set_surface_view(self, surface):
        """Aim the camera at the current palm face; never change pose/zero."""
        if surface not in {"dorsal", "palmar"}:
            raise ValueError("surface must be dorsal or palmar")
        normal = display_root_matrix(self.model, self.follow_root) @ np.array([0., 0., 1. if surface == "dorsal" else -1.])
        self.yaw = math.atan2(-normal[0], normal[2])
        self.pitch = math.atan2(normal[1], math.hypot(normal[0], normal[2]))
        self.update()

    def _header(self, painter):
        painter.setPen(QtGui.QColor("#edf2f9"))
        font = painter.font()
        font.setBold(True)
        painter.setFont(font)
        mode = {"live": "实时手套", "replay": "CSV 回放 · 非实时设备", "demo": "模拟数据 · 非真机"}.get(self.mode, self.mode)
        painter.drawText(18, 27, mode + "  /  " + ("实体 3D" if self.render_mode == "solid" else "骨架调试"))
        font.setBold(False)
        painter.setFont(font)
        root = "整掌朝向跟随 · 本次零位参考" if self.follow_root else "手掌局部锁定 · 不显示整掌旋转"
        if self.follow_root and not self._root_is_valid():
            root = "手掌朝向未就绪 · 仅显示局部参考外形"
        elif self.follow_root and self.stale:
            root = "历史手掌朝向 · 非当前测量"
        painter.setPen(QtGui.QColor("#9caec6"))
        painter.drawText(18, 49, root)
        labels = [("#cfdfe3", "有效"), ("#cfa14d", "估计 / 不确定标记"), ("#718294", "未标定"), ("#b26870", "断流")] if self.render_mode == "solid" else [(FINGER_COLORS[n], label) for n, label in FINGER_LABELS.items()]
        x = 18
        for color, label in labels:
            painter.setPen(QtGui.QColor(color))
            painter.drawText(x, 73, "● " + label)
            x += painter.fontMetrics().horizontalAdvance("● " + label) + 16
        painter.setPen(QtGui.QColor("#e5edf5"))
        font.setBold(True)
        painter.setFont(font)
        painter.drawText(18, 97, self.view_identity())
        font.setBold(False)
        painter.setFont(font)

    def _footer(self, painter):
        live = any(f.get("geometry_live", f.get("valid", False)) for f in self.model.get("fingers", {}).values())
        painter.setPen(QtGui.QColor("#dba3aa" if self.stale else "#a4b5cc"))
        status = "数据失效 / 历史画面，不代表当前手姿" if self.stale else "未完成标定：蓝灰外形是示意占位，不是测量结果" if not live and self.mode != "demo" else "青环 = 模型关节位置；金色 = 推算 / 越界；灰环 = 未观测"
        painter.drawText(18, self.height()-39, status)
        painter.setPen(QtGui.QColor("#899db9"))
        painter.drawText(18, self.height()-17, "六轴航向无绝对参考、仍会漂移；不估计全局平移")

    def paintEvent(self, event):
        started = time.monotonic()
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        background = QtGui.QLinearGradient(0, 0, self.width(), self.height())
        background.setColorAt(0, QtGui.QColor("#172539"))
        background.setColorAt(1, QtGui.QColor("#0b1421"))
        painter.fillRect(self.rect(), background)
        self._header(painter)
        self.drawn_triangles = 0
        try:
            if not self.model.get("fingers"):
                painter.setPen(QtGui.QColor("#9caec6"))
                painter.drawText(self.rect(), QtCore.Qt.AlignCenter, "等待手部模型\n本窗口不会自动连接串口")
            else:
                if self._mesh_dirty or self._mesh is None:
                    build_start = time.monotonic()
                    self._display_model = anatomical_display_model(self.model, self._geometry_config)
                    self._mesh = build_hand_mesh(self._display_model, stale=self.stale)
                    self.mesh_build_elapsed_ms = (time.monotonic()-build_start)*1000
                    self._mesh_dirty = False
                mesh = self._mesh
                viewport = QtCore.QRectF(14, 112, self.width()-28, max(30, self.height()-169))
                center = self.model_to_view(np.asarray([mesh.center]))[0]
                # Fixed bone-length bound remains safe across flexion and root
                # rotation, rather than scaling with each changing pose bbox.
                scale = self.zoom * min(viewport.width(), viewport.height()) / (2*mesh.radius)

                def project_view(points):
                    points = np.asarray(points)
                    return np.column_stack((viewport.center().x() + (points[:, 0]-center[0])*scale,
                                            viewport.center().y() - (points[:, 1]-center[1])*scale))

                def project(points):
                    return project_view(self.model_to_view(points))

                painter.save()
                painter.setClipRect(viewport)
                painter.setPen(QtGui.QPen(QtGui.QColor(88, 118, 156, 25), 1))
                painter.setBrush(QtCore.Qt.NoBrush)
                radius = min(viewport.width(), viewport.height())*0.45
                painter.drawEllipse(viewport.center(), radius, radius)
                if self.render_mode == "solid":
                    self._draw_solid(painter, project_view)
                    self._draw_estimated_overlay(painter, project)
                else:
                    self._draw_skeleton(painter, project)
                painter.restore()
        except (ValueError, TypeError, IndexError, OverflowError, KeyError):
            painter.setPen(QtGui.QColor("#ed8585"))
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, "模型几何无效，暂停绘制")
        self._footer(painter)
        painter.end()
        now = time.monotonic()
        self.paint_elapsed_ms = (now-started)*1000
        self.snapshot_to_paint_ms = max(0, (now-self.published_monotonic_s)*1000) if self.published_monotonic_s is not None else None

    def _draw_solid(self, painter, project_view):
        mesh = self._mesh
        if not len(mesh.triangles):
            return
        view_points = self.model_to_view(mesh.vertices)
        projected = project_view(view_points)
        triangles = view_points[mesh.triangles]
        geometric_normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
        # Primitive winding is outward for both hands. Cull the occluded backs
        # before Python/Qt work; smooth vertex normals are only for lighting.
        visible = np.flatnonzero(geometric_normals[:, 2] > 1e-7)
        vertex_normals = self.model_to_view(mesh.normals)
        normals = vertex_normals[mesh.triangles].mean(axis=1)
        normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
        light = np.array([-0.35, 0.5, 0.79])
        light /= np.linalg.norm(light)
        diffuse = np.clip(normals @ light, 0, 1)
        specular = np.maximum(0, normals @ np.array([-0.18, 0.25, 0.951]))**18*0.025
        colors = (np.clip(mesh.colors*(0.80+0.20*diffuse[:, None]) + 255*specular[:, None], 0, 255).astype(np.uint8)//3)*3
        packed_colors = (colors[:, 0].astype(np.uint32) << 16) | (colors[:, 1].astype(np.uint32) << 8) | colors[:, 2].astype(np.uint32)
        projected_triangles = projected[mesh.triangles]
        depth = triangles[:, :, 2].mean(axis=1)
        # QPainter consumes polygon coordinates synchronously. Keep this buffer
        # alive and never resize/share it, avoiding three QPointF allocations
        # and one polygon allocation for every triangle.
        polygon = QtGui.QPolygonF(3)
        data = polygon.data()
        data.setsize(3*2*8)
        coordinates = np.frombuffer(data, dtype=np.float64).reshape(3, 2)
        if len(self._materials) > 4096:
            self._materials.clear()
        # Filling shared triangle edges without per-triangle antialiasing avoids
        # dark background seams and repeated blending across the solid surface.
        # Text and diagnostic overlays retain antialiasing outside this block.
        painter.setRenderHint(QtGui.QPainter.Antialiasing, False)
        painter.setPen(QtCore.Qt.NoPen)
        for index in visible[np.argsort(depth[visible])]:
            key = int(packed_colors[index])
            material = self._materials.get(key)
            if material is None:
                color = QtGui.QColor((key >> 16) & 255, (key >> 8) & 255, key & 255)
                material = QtGui.QBrush(color)
                self._materials[key] = material
            painter.setBrush(material)
            coordinates[:] = projected_triangles[index]
            painter.drawPolygon(polygon)
            self.drawn_triangles += 1
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)

    def _draw_estimated_overlay(self, painter, project):
        painter.setPen(QtGui.QPen(QtGui.QColor(177, 137, 68, 190), 1.25, QtCore.Qt.DashLine))
        display = self._display_model if not self._mesh_dirty else anatomical_display_model(self.model, self._geometry_config)
        for finger in display.get("fingers", {}).values():
            if (not finger.get("valid", False) or self.stale or "stale" in str(finger.get("quality", ""))
                    or finger.get("dip_source") != "model_estimate" or not finger.get("dip_valid", False)):
                continue
            points = finger.get("points", [])
            for index in finger.get("estimated_segments", []):
                if 0 <= index < len(points)-1:
                    xy = project([points[index], points[index+1]])
                    painter.drawLine(QtCore.QPointF(*xy[0]), QtCore.QPointF(*xy[1]))

    def _draw_skeleton(self, painter, project):
        palm = self._display_model.get("palm_points", [])
        if len(palm) >= 3:
            painter.setPen(QtGui.QPen(QtGui.QColor("#687f9e"), 2))
            painter.setBrush(QtGui.QColor(82, 113, 155, 45))
            painter.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(*p) for p in project(palm)]))
        for name, finger in self._display_model.get("fingers", {}).items():
            points = finger.get("points", [])
            if len(points) < 2:
                continue
            stale = self.stale or "stale" in str(finger.get("quality", ""))
            color = QtGui.QColor("#d5686c" if stale else FINGER_COLORS.get(name, "#ffffff") if finger.get("valid") else "#77818d")
            xy = project(points)
            for index in range(len(points)-1):
                pen = QtGui.QPen(color, 4, QtCore.Qt.DashLine if index in finger.get("estimated_segments", []) else QtCore.Qt.SolidLine, QtCore.Qt.RoundCap)
                painter.setPen(pen)
                painter.drawLine(QtCore.QPointF(*xy[index]), QtCore.QPointF(*xy[index+1]))
            painter.setPen(QtCore.Qt.NoPen)
            painter.setBrush(color)
            for point in xy:
                painter.drawEllipse(QtCore.QPointF(*point), 3.5, 3.5)

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self._drag = event.pos()

    def mouseMoveEvent(self, event):
        if self._drag is not None:
            delta = event.pos()-self._drag
            self._drag = event.pos()
            self.yaw += delta.x()*0.01
            self.pitch = max(-1.5, min(1.5, self.pitch+delta.y()*0.01))
            self.update()

    def mouseReleaseEvent(self, event):
        self._drag = None

    def wheelEvent(self, event):
        self.zoom = max(0.4, min(3.0, self.zoom*math.exp(event.angleDelta().y()/1200)))
        self.update()

    def mouseDoubleClickEvent(self, event):
        self.yaw, self.pitch, self.zoom = -0.38, 0.28, 1.0
        self.update()
