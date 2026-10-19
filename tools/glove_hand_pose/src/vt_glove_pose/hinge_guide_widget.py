"""Palette-aware teaching illustration, deliberately unrelated to live angles.

No samples, observer state, serial devices or controller references enter this
widget. Its slow animation is solely a demonstration of an operator action.
"""
from __future__ import annotations

import math

from PyQt5 import QtCore, QtGui, QtWidgets


DISCLAIMER = "动作教学示意 · 不是传感器测量"
_INSTRUCTIONS = {
    "connect": ("连接手套", "右手掌心朝向自己\n检查线缆，等待连接"),
    "still": ("保持参考手型", "舒适放松\n手掌和拇指都保持不动"),
    "flex": ("缓慢屈伸拇指", "朝掌心弯曲\n再缓慢伸回\n不需要用力或甩手"),
    "rotate": ("固定指形，轻缓转掌", "拇指形状保持不变\n只改变整只手的朝向"),
    "flex_once": ("先向掌心弯曲一次", "缓慢弯曲拇指\n然后保持\n先不要反向伸回"),
    "hold": ("保持当前声明手型", "保持同一姿势\n不必强行伸直\n不要继续屈伸"),
    "return": ("回到已声明的参考手型", "缓慢回到参考手型\n再保持不动\n不一定是完全伸直"),
    "export": ("导出实验记录", "手部无需做动作\n保留数据后再开始下一轮"),
}
_ANIMATED = {"flex", "rotate", "flex_once", "return"}


class MotionGuideCanvas(QtWidgets.QWidget):
    """Compact right-palm action diagram; ``capture`` alone enables animation.

    Repeated calls with the same instruction do not restart the animation.
    Other phase strings are accepted but remain static, so waiting/error/done
    states cannot accidentally animate. An unknown illustration is rejected.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(260, 190)
        self.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        self._illustration = "connect"
        self._phase = "idle"
        self._elapsed = QtCore.QElapsedTimer()
        self._elapsed.start()
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(40)
        self._timer.timeout.connect(self.update)
        self.setAccessibleName(DISCLAIMER)
        self._update_description()

    def sizeHint(self):
        return QtCore.QSize(330, 230)

    def set_instruction(self, illustration: str, phase: str):
        if not isinstance(illustration, str) or illustration not in _INSTRUCTIONS:
            raise ValueError("unknown motion guide illustration")
        if not isinstance(phase, str):
            raise ValueError("motion guide phase must be a string")
        if (illustration, phase) == (self._illustration, self._phase):
            return
        self._illustration, self._phase = illustration, phase
        self._elapsed.restart()
        self._update_description()
        self._update_timer()
        self.update()

    def _update_description(self):
        title, instruction = _INSTRUCTIONS[self._illustration]
        description = f"{DISCLAIMER}。右手掌心示意。{title}。{instruction.replace(chr(10), '，')}。"
        if self._phase == "prepare":
            description += "准备阶段可以调整姿势；开始提示后再做动作。"
        self.setAccessibleDescription(description)
        self.setToolTip(description)

    def _update_timer(self):
        if self.isVisible() and self._phase == "capture" and self._illustration in _ANIMATED:
            self._timer.start()
        else:
            self._timer.stop()

    def showEvent(self, event):
        super().showEvent(event)
        self._update_timer()

    def hideEvent(self, event):
        self._timer.stop()
        super().hideEvent(event)

    def _illustrated_pose(self, elapsed_s=None):
        """Return dimensionless demonstration bend and display rotation only."""
        if self._phase != "capture":
            return (.65 if self._illustration == "return" else .0), 0.
        t = max(0., self._elapsed.elapsed() / 1000. if elapsed_s is None else elapsed_s)
        if self._illustration == "flex":
            return .5 - .5 * math.cos(2. * math.pi * t / 5.), 0.
        if self._illustration == "flex_once":
            return .5 - .5 * math.cos(math.pi * min(t / 3., 1.)), 0.
        if self._illustration == "return":
            return .65 * (.5 + .5 * math.cos(math.pi * min(t / 3., 1.))), 0.
        if self._illustration == "rotate":
            return .15, 16. * math.sin(2. * math.pi * t / 7.)
        return 0., 0.

    @staticmethod
    def _arrow(painter, start, control, end, color):
        start, control, end = (QtCore.QPointF(*point) for point in (start, control, end))
        path = QtGui.QPainterPath(start)
        path.quadTo(control, end)
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.setPen(QtGui.QPen(color, 2.2, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
        painter.drawPath(path)
        vector = end - control
        length = math.hypot(vector.x(), vector.y())
        if length:
            along = vector / length
            across = QtCore.QPointF(-along.y(), along.x())
            painter.setBrush(color)
            painter.setPen(QtCore.Qt.NoPen)
            painter.drawPolygon(QtGui.QPolygonF([end, end - along * 8. + across * 3.5,
                                                 end - along * 8. - across * 3.5]))

    def _draw_hand(self, painter, bend, rotation):
        palette = self.palette()
        outline = palette.color(QtGui.QPalette.Mid)
        fill = palette.color(QtGui.QPalette.AlternateBase)
        ink = palette.color(QtGui.QPalette.Text)
        accent = palette.color(QtGui.QPalette.Highlight)
        painter.save()
        painter.translate(192., 159.)
        painter.rotate(rotation)
        painter.translate(-192., -159.)
        painter.setPen(QtGui.QPen(outline, 1.2))
        painter.setBrush(fill)
        # Palm-facing right hand: the thumb is on the viewer's left.
        for x, y in ((153., 66.), (174., 52.), (195., 58.), (216., 76.)):
            painter.drawRoundedRect(QtCore.QRectF(x, y, 17., 121. - y), 8., 8.)
            painter.drawLine(QtCore.QPointF(x + 4., y + (117. - y) * .56),
                             QtCore.QPointF(x + 13., y + (117. - y) * .56))
        palm = QtGui.QPainterPath(QtCore.QPointF(153., 112.))
        palm.cubicTo(146., 137., 151., 157., 164., 172.)
        palm.lineTo(167., 185.)
        palm.lineTo(212., 185.)
        palm.lineTo(215., 173.)
        palm.cubicTo(232., 153., 235., 129., 233., 111.)
        palm.closeSubpath()
        painter.drawPath(palm)
        # A couple of palm creases make the viewing side explicit.
        painter.drawLine(QtCore.QPointF(172., 120.), QtCore.QPointF(219., 127.))
        painter.drawLine(QtCore.QPointF(172., 150.), QtCore.QPointF(202., 155.))
        painter.setPen(ink)
        painter.drawText(QtCore.QRectF(174., 132., 50., 18.), QtCore.Qt.AlignCenter, "掌心")
        base = QtCore.QPointF(159., 153.)
        joint = QtCore.QPointF(128., 133.)
        theta = math.radians(-133. + 116. * bend)
        tip = joint + QtCore.QPointF(33. * math.cos(theta), 33. * math.sin(theta))
        for a, b in ((base, joint), (joint, tip)):
            painter.setPen(QtGui.QPen(outline, 20., QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
            painter.drawLine(a, b)
            painter.setPen(QtGui.QPen(fill, 17., QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
            painter.drawLine(a, b)
        # Highlight the target bend, without claiming its anatomical identity.
        painter.setBrush(palette.base())
        painter.setPen(QtGui.QPen(accent, 2.5))
        painter.drawEllipse(joint, 7., 7.)
        painter.drawEllipse(joint, 2., 2.)
        painter.setPen(QtGui.QPen(outline, 1., QtCore.Qt.DashLine))
        painter.drawLine(QtCore.QPointF(99., 153.), joint + QtCore.QPointF(-5., 5.))
        painter.setPen(ink)
        painter.drawText(QtCore.QRectF(43., 149., 69., 19.), QtCore.Qt.AlignLeft, "目标关节")
        painter.restore()
        if self._illustration in {"flex", "flex_once", "return"}:
            # Fixed direction cue is more readable than following the animation.
            if self._illustration == "return":
                self._arrow(painter, (163., 121.), (146., 88.), (109., 102.), accent)
            else:
                self._arrow(painter, (109., 102.), (148., 89.), (163., 121.), accent)
        elif self._illustration == "rotate":
            self._arrow(painter, (254., 81.), (280., 117.), (257., 155.), accent)
            self._arrow(painter, (260., 156.), (291., 120.), (266., 86.), accent)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), self.palette().base())
        scale = min(self.width() / 330., self.height() / 230.)
        painter.translate((self.width() - 330. * scale) / 2., (self.height() - 230. * scale) / 2.)
        painter.scale(scale, scale)
        ink = self.palette().color(QtGui.QPalette.Text)
        painter.setPen(ink)
        font = QtGui.QFont(self.font())
        font.setPixelSize(13)
        font.setBold(True)
        painter.setFont(font)
        painter.drawText(QtCore.QRectF(12., 8., 306., 23.), QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter,
                         _INSTRUCTIONS[self._illustration][0])
        font.setPixelSize(10)
        font.setBold(False)
        painter.setFont(font)
        phase = {"prepare": "准备中 · 尚未采集", "capture": "动作示范", "idle": "等待开始"}.get(self._phase, "等待开始")
        painter.drawText(QtCore.QRectF(12., 31., 304., 17.), QtCore.Qt.AlignLeft, f"右手 · 掌心示意    {phase}")
        painter.drawText(QtCore.QRectF(12., 66., 87., 77.),
                         QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop | QtCore.Qt.TextWordWrap,
                         _INSTRUCTIONS[self._illustration][1])
        self._draw_hand(painter, *self._illustrated_pose())
        painter.setPen(ink)
        painter.drawText(QtCore.QRectF(12., 189., 306., 17.), QtCore.Qt.AlignCenter,
                         "幅度以舒适为准；图形不是实时角度")
        painter.setPen(self.palette().color(QtGui.QPalette.Mid))
        painter.drawLine(QtCore.QPointF(12., 209.), QtCore.QPointF(318., 209.))
        font.setBold(True)
        font.setPixelSize(11)
        painter.setFont(font)
        painter.setPen(ink)
        painter.drawText(QtCore.QRectF(8., 212., 314., 17.), QtCore.Qt.AlignCenter, DISCLAIMER)
