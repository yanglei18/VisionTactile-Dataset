"""Render received MANO results with the calibration tool's shared surface.

No sensor reads or pose solving: a separate model process reconstructs received
parameters. A neutral model is never presented as a measured gesture.
"""
from __future__ import annotations

from ...i18n import tr
from ...i18n.messages import message as display_message

import math

from PySide6 import QtCore, QtGui
from ...i18n import widgets as QtWidgets

from ...backend.pose_preview import key_for, pose_preview


class Glove3DWidget(QtWidgets.QWidget):
    def __init__(self, hand, parent=None):
        super().__init__(parent)
        self.setObjectName("handCanvas")
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet("QWidget#handCanvas { background: #000000; }")
        self.hand = hand
        self.view = None
        self._bundle = None
        self._pending_bundle = self._pending_result = None
        self._desired_key = self._display_key = None
        self._session = None
        self._current = None
        self._invalid = True
        self._disposed = False
        self._palmar, self._auto_fit = False, True
        self._camera_dirty = self._joints_dirty = self._hide_hand = False
        self._radius = .13
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.setSpacing(3)
        toolbar = QtWidgets.QHBoxLayout()
        toolbar.setSpacing(3)
        self.buttons = []
        for title, callback in ((tr('掌面'), lambda: self.reset_camera(True)),
                                (tr('手背'), lambda: self.reset_camera(False)),
                                (tr('复位'), lambda: self.reset_camera(False))):
            button = QtWidgets.QPushButton(title)
            button.setAutoDefault(False)
            button.setFixedHeight(23)
            button.setStyleSheet("font-size: 10px; padding: 2px 5px; border-radius: 5px;")
            button.clicked.connect(callback)
            button.setEnabled(False)
            toolbar.addWidget(button)
            self.buttons.append(button)
        self.joints_button = QtWidgets.QPushButton(tr('关节'))
        self.joints_button.setAutoDefault(False)
        self.joints_button.setCheckable(True)
        self.joints_button.setFixedHeight(23)
        self.joints_button.setStyleSheet("font-size: 10px; padding: 2px 5px; border-radius: 5px;")
        self.joints_button.toggled.connect(self._show_joints)
        self.joints_button.setEnabled(False)
        toolbar.addWidget(self.joints_button)
        self.buttons.append(self.joints_button)
        toolbar.addStretch()
        self.layout.addLayout(toolbar)
        self.message = QtWidgets.QLabel(tr('等待有效手势；请打开标定工具连接 IMU 并完成标定。'))
        self.message.setTextFormat(QtCore.Qt.TextFormat.PlainText)
        self.message.setWordWrap(True)
        self.message.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.message.setStyleSheet("font-size: 11px; color: #a3a3a3; background: transparent;")
        self.layout.addWidget(self.message, 1)
        self.retry_button = QtWidgets.QPushButton(tr('重新加载模型'))
        self.retry_button.setAutoDefault(False)
        self.retry_button.clicked.connect(self._retry)
        self.layout.addWidget(self.retry_button)
        self.retry_button.hide()
        self.worker = pose_preview(hand)
        self.worker.initialized.connect(self._initialized)
        self.worker.ready.connect(self._ready)
        self.worker.failed.connect(self._failed)

    def clear_sample(self):
        """Discard this view's pose, including hidden and asynchronous state.

        Unlike ``set_sample(..., held=True)``, an explicit clear must not
        retain a historical hand. The worker is shared with other views, so
        invalidate only this consumer; late results cannot match a desired
        sample until a new valid input is supplied.
        """
        if self._disposed:
            return
        self._invalid = True
        self._desired_key = self._display_key = self._session = None
        self._current = self._pending_bundle = self._pending_result = None
        self._hide_hand = False
        if self.view is not None:
            self.view.set_hand_visible(False)
        self.message.setText(tr('等待有效手势；请打开标定工具连接 IMU 并完成标定。'))
        self.message.show()
        self.retry_button.hide()

    def set_sample(self, frame, *, held, mode, device):
        if self._disposed:
            return
        mano = (frame or {}).get("mano")
        if held or not mano or mode == "demo":
            if not self._invalid and self.isVisible():
                self.worker.clear()
            self._invalid = True
            self._desired_key = None
            self._current = None
            self._pending_result = None
            if self.view is None or self._display_key is None:
                if mode == "demo":
                    text = tr('演示只有示意骨架；未伪造 MANO 手势。请切换骨架视图。')
                elif frame and not mano:
                    text = tr('当前后台仅提供骨架。更新管理后台后可显示 MANO；骨架视图仍可用。')
                else:
                    text = device.get("glove_hint") or tr('等待有效手势；请打开标定工具连接 IMU 并完成标定。')
                self.message.setText(display_message(text))
                self.message.show()
            else:
                self.message.setText(tr('历史姿态 · 已保持，等待新的有效解算结果'))
                self.message.show()
            return
        try:
            key = key_for(mano)
            if mano["hand"] != self.hand:
                raise ValueError(tr('手势数据与模型手别不一致'))
        except (ValueError, TypeError) as exc:
            if self.isVisible():
                self.worker.clear()
            self._invalid = True
            self._desired_key = self._current = None
            self._pending_result = None
            self._failed(str(exc))
            return
        self._invalid = False
        session = tuple(mano.get(k) for k in ("hand", "session_id", "calibration_version",
                                             "model_rest_sha256", "config_sha256", "source_mode"))
        if self._session is not None and self._session != session:
            self._display_key = None
            self._hide_hand = True
            self.message.setText(tr('新会话或标定结果，等待对应的三维姿态…'))
            self.message.show()
        self._session = session
        if key != self._desired_key:
            self._pending_result = None
        self._desired_key = key
        self._current = (mano, frame.get("joints") or None)
        self._sync_visible()

    def _sync_visible(self):
        """Catch up once on show; hidden tiles leave the shared worker alone."""
        if self._disposed or not self.isVisible():
            return
        if self.view is not None:
            if self._hide_hand:
                self.view.set_hand_visible(False)
                self._hide_hand = False
            if self._joints_dirty:
                self._show_joints(self.joints_button.isChecked())
            if self._camera_dirty:
                self._fit_camera()
        if self._invalid or self._current is None:
            return
        self.worker.submit(*self._current)
        bundle = self.worker.bundle if self.worker.bundle is not None else self._pending_bundle
        if bundle is not None:
            self._initialized(bundle)
        if self.worker.geometry is not None:
            self._ready(self.worker.geometry)
        if self._pending_result is not None:
            self._ready(self._pending_result)

    def showEvent(self, event):
        super().showEvent(event)
        self._sync_visible()

    def _initialized(self, bundle):
        if self._disposed or self._invalid or self._current is None or self._bundle is bundle:
            return
        self._pending_bundle = bundle
        if not self.isVisible():
            return
        try:
            if QtGui.QGuiApplication.platformName() in ("offscreen", "minimal"):
                raise RuntimeError(tr('当前显示平台不支持 OpenGL；可使用骨架视图'))
            context = QtGui.QOpenGLContext()
            if not context.create():
                raise RuntimeError(tr('无法创建 OpenGL 上下文；请检查桌面显卡驱动'))
            import numpy as np
            import pyqtgraph
            if pyqtgraph.Qt.QT_LIB != "PySide6":
                raise RuntimeError(tr('工作台 3D 显示不能混用其他 Qt 绑定'))
            from vt_fsglove12.hand_view import HandView
            view = HandView(bundle["neutral_geometry"], bundle["model_axes"], self)
            view.setMinimumSize(0, 0)
            view.setBackgroundColor("#000000")
            view.set_show_joints(self.joints_button.isChecked())
            view.set_hand_visible(False)
            self._joints_dirty = self._hide_hand = False
            vertices = view.mesh_data.vertexes()
            center = (vertices.min(axis=0) + vertices.max(axis=0)) * .5
            self._radius = float(np.linalg.norm(vertices - center, axis=1).max())
            if self.view is not None:
                self.layout.removeWidget(self.view)
                self.view.hide()
                self.view.deleteLater()
            self.view, self._bundle = view, bundle
            self._pending_bundle = None
            self._display_key = None
            view.installEventFilter(self)
            self.layout.addWidget(view, 1)
            self.message.setText(tr('正在重建收到的 MANO 姿态…'))
            self.message.show()
            self.retry_button.hide()
            self.reset_camera(self._palmar)
            for button in self.buttons:
                button.setEnabled(True)
            if self._pending_result is not None:
                self._ready(self._pending_result)
        except (ImportError, OSError, ValueError, RuntimeError, KeyError) as exc:
            self._failed(str(exc))

    def _ready(self, result):
        if (self._disposed or self._invalid
                or result["key"] != self._desired_key):
            return
        self._pending_result = result
        if not self.isVisible() or self.view is None:
            return
        if result["key"] == self._display_key:
            # A held pose may become current again without changing geometry.
            self._pending_result = None
            self.message.hide()
            self.retry_button.hide()
            return
        try:
            self.view.set_geometry(result["geometry"])
            # Only camera translation follows the wrist-relative object bounds;
            # measured root rotation is retained, never applied twice/cancelled.
            vertices = self.view.mesh_data.vertexes()
            center = (vertices.min(axis=0) + vertices.max(axis=0)) * .5
            self.view.opts["center"] = QtGui.QVector3D(*[float(v) for v in center])
            self.view.update()
            self._display_key = result["key"]
            self._pending_result = None
            self.message.hide()
            self.retry_button.hide()
        except (ValueError, TypeError) as exc:
            self._failed(str(exc))

    def _failed(self, message):
        if self._disposed:
            return
        self._pending_bundle = self._pending_result = None
        self.message.setText(tr('三维模型暂不可用 · 可切换骨架\n') + display_message(message))
        self.message.show()
        self.retry_button.show()

    def _retry(self):
        self.worker.retry()
        self._bundle = None
        self._pending_bundle = self._pending_result = None
        self.retry_button.hide()
        self.message.setText(tr('正在重新加载 MANO…'))
        if self._current is not None and self.isVisible():
            self.worker.submit(*self._current)
        else:
            self.message.setText(tr('等待新的有效手势；请确认原工具已连接并完成标定。'))

    def _show_joints(self, checked):
        self._joints_dirty = True
        if self.view is not None and self.isVisible():
            self.view.set_show_joints(checked)
            self._joints_dirty = False

    def reset_camera(self, palmar=False):
        self._palmar, self._auto_fit = palmar, True
        self._fit_camera()

    def _fit_camera(self):
        self._camera_dirty = True
        if self.view is None or not self.isVisible():
            return
        aspect = max(1, self.view.width()) / max(1, self.view.height())
        angle = math.radians(self.view.opts["fov"]) / 2
        half = min(angle, math.atan(math.tan(angle) / aspect))
        self.view.setCameraPosition(distance=self._radius / math.sin(half) * 1.05,
                                    elevation=-78 if self._palmar else 78,
                                    azimuth=90 if self._palmar else -90)
        self._camera_dirty = False

    def eventFilter(self, watched, event):
        if watched is self.view:
            if event.type() in (QtCore.QEvent.Type.MouseButtonPress, QtCore.QEvent.Type.Wheel):
                self._auto_fit = False
            elif event.type() == QtCore.QEvent.Type.Resize and self._auto_fit:
                self._fit_camera()
        return super().eventFilter(watched, event)

    def dispose(self):
        self._disposed = True
        self.worker.initialized.disconnect(self._initialized)
        self.worker.ready.disconnect(self._ready)
        self.worker.failed.disconnect(self._failed)
        self.hide()
        self.deleteLater()
