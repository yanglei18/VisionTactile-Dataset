"""Lazy, read-only MANO surface view using the calibration tool's renderer.

Only explicitly choosing 3D loads OpenGL. A short-lived separate process loads
the trusted MANO model; this process never imports torch, ROS or a serial driver.
"""

from __future__ import annotations

from ...i18n import tr
from ...i18n.messages import exception_message, message as display_message
from ...i18n.widgets import bind_text

import math

from PySide6 import QtCore, QtGui
from ...i18n import widgets as QtWidgets

from .tactile_model_loader import model_loader


class Tactile3DWidget(QtWidgets.QWidget):
    def __init__(self, hand, parent=None):
        super().__init__(parent)
        self.setObjectName("handCanvas")
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet("QWidget#handCanvas { background: #000000; }")
        self.hand = hand
        self.view = None
        self.binding = None
        self._bundle = None
        self._frame = self._levels = None
        self._last_colors_key = None
        self._disposed = False
        self._palmar = True
        self._auto_fit = True
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.setSpacing(3)
        toolbar = QtWidgets.QHBoxLayout()
        toolbar.setContentsMargins(0, 0, 0, 0)
        toolbar.setSpacing(3)
        self.buttons = []
        for title, tip, callback in (
            (tr('掌面'), tr('查看掌面触觉'), lambda: self.reset_camera(True)),
            (tr('手背'), tr('查看手背（未映射处保留模型底色）'), lambda: self.reset_camera(False)),
            (tr('复位'), tr('恢复掌面视角和缩放；不修改任何传感器或标定'), lambda: self.reset_camera(True)),
        ):
            button = QtWidgets.QPushButton(title)
            button.setAutoDefault(False)
            button.setToolTip(tip)
            button.setAccessibleName(tip)
            button.setFixedHeight(23)
            button.setStyleSheet("font-size: 10px; padding: 2px 5px; border-radius: 5px;")
            button.clicked.connect(callback)
            button.setEnabled(False)
            toolbar.addWidget(button)
            self.buttons.append(button)
        self.joints_button = QtWidgets.QPushButton(tr('关节'))
        self.joints_button.setAutoDefault(False)
        self.joints_button.setCheckable(True)
        self.joints_button.setToolTip(tr('显示模型关节中心，仅作结构标注，不是 IMU 测量位置'))
        self.joints_button.setFixedHeight(23)
        self.joints_button.setStyleSheet("font-size: 10px; padding: 2px 5px; border-radius: 5px;")
        self.joints_button.setEnabled(False)
        self.joints_button.toggled.connect(self._show_joints)
        toolbar.addWidget(self.joints_button)
        self.buttons.append(self.joints_button)
        toolbar.addStretch()
        self.layout.addLayout(toolbar)
        self.message = QtWidgets.QLabel(tr('正在加载标定工具的 MANO 模型…\n仅加载静态参考手，不连接 IMU。'))
        self.message.setWordWrap(True)
        self.message.setTextFormat(QtCore.Qt.TextFormat.PlainText)
        self.message.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.message.setStyleSheet("color: #a3a3a3; font-size: 11px; background: transparent;")
        self.layout.addWidget(self.message, 1)
        self.retry_button = QtWidgets.QPushButton(tr('重新加载模型'))
        self.retry_button.setAutoDefault(False)
        self.retry_button.clicked.connect(self._retry)
        self.retry_button.hide()
        self.layout.addWidget(self.retry_button)
        self.loader = model_loader()
        self.loader.ready.connect(self._model_ready)
        self.loader.failed.connect(self._model_failed)
        self.loader.request(hand)
        bundle = self.loader.get(hand)
        if bundle is not None:
            self._model_ready(hand, bundle)
        elif self.loader.error(hand):
            self._model_failed(hand, self.loader.error(hand))

    def _model_failed(self, hand, message):
        if hand != self.hand or self._disposed:
            return
        detail = display_message(message)
        title = tr('模型更新失败 · 保留旧静态参考手') if self.view is not None else tr('3D 手形暂不可用')
        self.message.setText(tr('{v0}\n{v1}\n请检查模型配置，然后重新加载。', v0=title, v1=detail))
        self.message.setToolTip(detail)
        self.message.show()
        self.retry_button.show()

    def _retry(self):
        self.retry_button.hide()
        self.message.setText(tr('正在加载静态 MANO 模型…'))
        self.loader.request(self.hand)

    def _model_ready(self, hand, bundle):
        if hand != self.hand or self._disposed:
            return
        if self.view is not None and self._bundle is bundle:
            self.message.hide()
            self.retry_button.hide()
            return
        try:
            # Offscreen/minimal platforms cannot compose QOpenGLWidget safely.
            if QtGui.QGuiApplication.platformName() in ("offscreen", "minimal"):
                raise RuntimeError(tr('当前显示平台不支持 OpenGL，请在桌面会话打开工作台'))
            context = QtGui.QOpenGLContext()
            if not context.create():
                raise RuntimeError(tr('无法创建 OpenGL 上下文，请检查桌面显卡驱动'))
            import pyqtgraph
            if pyqtgraph.Qt.QT_LIB != "PySide6":
                raise RuntimeError(tr('3D 显示需要 PySide6；不能混用其他 Qt 绑定'))
            from vt_fsglove12.hand_view import HandView
            from vt_fsglove12.tactile_surface import TactileSurfaceBinding

            neutral, axes = bundle["neutral_geometry"], bundle["model_axes"]
            binding = TactileSurfaceBinding(neutral, axes, hand=hand)
            view = HandView(neutral, axes, parent=self)
            view.setMinimumSize(0, 0)
            view.setBackgroundColor("#000000")
            view.set_show_joints(self.joints_button.isChecked())
            metadata = binding.metadata()
            description = tr('静态参考手，非实时手势。{v0}/400 个地址具有候选表面位置；{v1} 个地址未映射；并非逐点皮肤标定。', v0=metadata['bound_count'], v1=metadata['unbound_count'])
            bind_text(view, 'setAccessibleDescription', description)
            bind_text(view, 'setToolTip',
                      tr('静态 MANO 参考手 · 非实时手势\n左键拖动旋转 · 滚轮缩放\n触觉为候选表面映射，颜色表示原始 ADC。')
                      + "\n" + description)
            if self.view is not None:
                self.layout.removeWidget(self.view)
                self.view.hide()
                self.view.deleteLater()
            self.binding, self.view, self._bundle = binding, view, bundle
            view.installEventFilter(self)
            self.layout.addWidget(view, 1)
            self.message.hide()
            self.retry_button.hide()
            self._last_colors_key = None
            for button in self.buttons:
                button.setEnabled(True)
            self._apply_colors()
            self.reset_camera(True)
        except (ImportError, OSError, RuntimeError, ValueError, KeyError) as exc:
            self._model_failed(hand, exception_message(exc))

    def set_frame(self, frame, levels):
        self._frame, self._levels = frame, levels
        self._apply_colors()

    def _apply_colors(self):
        if self.view is None:
            return
        key = (self._frame, self._levels)
        if key == self._last_colors_key:
            return
        from .tactile_surface_colors import surface_colors
        from .tactile_palette import heat_stops

        kind = self._frame.get("value_kind", "raw_adc") if self._frame is not None else "raw_adc"
        colors = surface_colors(self.binding, self._frame, self.view.colors,
                                self._levels, heat_stops(kind))
        self.view.set_surface_colors(colors)
        self._last_colors_key = key

    def reset_camera(self, palmar=True):
        if self.view is not None:
            self._palmar, self._auto_fit = palmar, True
            self._fit_camera()

    def _show_joints(self, checked):
        if self.view is not None:
            self.view.set_show_joints(checked)

    def _fit_camera(self):
        vertices = self.view.mesh_data.vertexes()
        lo, hi = vertices.min(axis=0), vertices.max(axis=0)
        center = (lo + hi) * .5
        aspect = max(1, self.view.width()) / max(1, self.view.height())
        fov = math.radians(self.view.opts["fov"])
        distance = max(hi[0] - lo[0], (hi[1] - lo[1]) * aspect) / (2 * math.tan(fov / 2)) * 1.25
        distance += hi[2] - lo[2]
        self.view.opts["center"] = QtGui.QVector3D(*[float(v) for v in center])
        self.view.setCameraPosition(distance=float(distance), elevation=-78 if self._palmar else 78,
                                    azimuth=90 if self._palmar else -90)

    def eventFilter(self, watched, event):
        if watched is self.view:
            if event.type() in (QtCore.QEvent.Type.MouseButtonPress, QtCore.QEvent.Type.Wheel):
                self._auto_fit = False
            elif event.type() == QtCore.QEvent.Type.Resize and self._auto_fit:
                self._fit_camera()
        return super().eventFilter(watched, event)

    def dispose(self):
        self._disposed = True
        self.loader.ready.disconnect(self._model_ready)
        self.loader.failed.disconnect(self._model_failed)
        self.hide()
        self.deleteLater()
