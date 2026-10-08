"""Original scalable line icons painted by Qt; no fonts or branded assets."""

from PySide6 import QtCore, QtGui


_NAMES = frozenset({
    "dashboard", "trials", "devices", "settings", "play", "stop",
    "camera", "tracker", "glove", "tactile", "chevron", "orbit", "expand",
})


def _line(painter, x1, y1, x2, y2):
    painter.drawLine(QtCore.QPointF(x1, y1), QtCore.QPointF(x2, y2))


def _rounded(painter, x, y, width, height, radius=2):
    painter.drawRoundedRect(QtCore.QRectF(x, y, width, height), radius, radius)


def _path(painter, points, *, closed=False):
    path = QtGui.QPainterPath(QtCore.QPointF(*points[0]))
    for point in points[1:]:
        path.lineTo(*point)
    if closed:
        path.closeSubpath()
    painter.drawPath(path)


def _draw(painter, name):
    if name == "dashboard":
        for x, y in ((3.5, 3.5), (13.5, 3.5), (3.5, 13.5), (13.5, 13.5)):
            _rounded(painter, x, y, 7, 7, 1.6)
    elif name == "trials":
        _rounded(painter, 5, 3.5, 14, 17, 2.3)
        for y in (8, 12, 16):
            _line(painter, 10.5, y, 15.8, y)
            painter.drawEllipse(QtCore.QPointF(8.2, y), .45, .45)
    elif name == "devices":
        _rounded(painter, 3, 4, 18, 12, 2.3)
        _line(painter, 9, 20, 15, 20)
        _line(painter, 12, 16, 12, 20)
        _line(painter, 7, 9, 10, 9)
        _line(painter, 14, 9, 17, 9)
        _line(painter, 7, 12, 17, 12)
    elif name == "settings":
        for x, y in ((6, 8), (12, 15), (18, 10)):
            _line(painter, x, 3.5, x, y - 2)
            _line(painter, x, y + 2, x, 20.5)
            painter.drawEllipse(QtCore.QPointF(x, y), 2, 2)
    elif name == "play":
        painter.setBrush(painter.pen().color())
        _path(painter, ((8, 4.8), (19, 12), (8, 19.2)), closed=True)
    elif name == "stop":
        painter.setBrush(painter.pen().color())
        _rounded(painter, 6.5, 6.5, 11, 11, 2)
    elif name == "camera":
        _path(painter, ((3, 7), (7.1, 7), (9, 4.5), (15, 4.5), (16.9, 7), (21, 7),
                        (21, 19.5), (3, 19.5)), closed=True)
        painter.drawEllipse(QtCore.QPointF(12, 12.8), 3.8, 3.8)
        painter.drawEllipse(QtCore.QPointF(18, 9.8), .5, .5)
    elif name == "tracker":
        _path(painter, ((12, 3), (20, 7.5), (20, 16.5), (12, 21), (4, 16.5), (4, 7.5)), closed=True)
        _path(painter, ((4, 7.5), (12, 12), (20, 7.5)))
        _line(painter, 12, 12, 12, 21)
        _line(painter, 8, 5.3, 15.8, 9.8)
    elif name == "glove":
        path = QtGui.QPainterPath(QtCore.QPointF(7.5, 12.8))
        path.lineTo(7.5, 5.7)
        path.cubicTo(7.5, 3.8, 10, 3.8, 10, 5.7)
        path.lineTo(10, 11)
        path.lineTo(10, 3.9)
        path.cubicTo(10, 2.1, 12.5, 2.1, 12.5, 3.9)
        path.lineTo(12.5, 10.8)
        path.lineTo(12.5, 4.8)
        path.cubicTo(12.5, 3, 15, 3, 15, 4.8)
        path.lineTo(15, 11.8)
        path.lineTo(15, 7.3)
        path.cubicTo(15, 5.5, 17.5, 5.5, 17.5, 7.3)
        path.lineTo(17.5, 15.1)
        path.cubicTo(17.5, 19.4, 15.4, 21.2, 11.8, 21.2)
        path.cubicTo(9.2, 21.2, 7.9, 19.5, 6.5, 17.6)
        path.lineTo(3.9, 13.8)
        path.cubicTo(2.8, 12.2, 4.3, 10.7, 5.6, 11.9)
        path.lineTo(7.5, 13.8)
        painter.drawPath(path)
    elif name == "tactile":
        _rounded(painter, 3, 3, 18, 18, 3.2)
        painter.setBrush(painter.pen().color())
        for x in (7.5, 12, 16.5):
            for y in (7.5, 12, 16.5):
                painter.drawEllipse(QtCore.QPointF(x, y), .75, .75)
    elif name == "chevron":
        _path(painter, ((6, 9), (12, 15), (18, 9)))
    elif name == "expand":
        for corner in (((4, 9), (4, 4), (9, 4)), ((15, 4), (20, 4), (20, 9)),
                       ((20, 15), (20, 20), (15, 20)), ((9, 20), (4, 20), (4, 15))):
            _path(painter, corner)
    elif name == "orbit":
        painter.drawEllipse(QtCore.QPointF(12, 12), 8.8, 8.8)
        painter.save()
        painter.translate(12, 12)
        painter.rotate(-35)
        painter.drawEllipse(QtCore.QRectF(-11, -3.8, 22, 7.6))
        painter.restore()
        painter.setBrush(painter.pen().color())
        painter.drawEllipse(QtCore.QPointF(12, 12), 2.1, 2.1)


class _VectorIconEngine(QtGui.QIconEngine):
    def __init__(self, name, color, size):
        super().__init__()
        self.name, self.color, self.size = name, QtGui.QColor(color), size

    def clone(self):
        return _VectorIconEngine(self.name, self.color, self.size)

    def availableSizes(self, mode=QtGui.QIcon.Mode.Normal, state=QtGui.QIcon.State.Off):
        return [QtCore.QSize(self.size, self.size)]

    def actualSize(self, size, mode, state):
        side = min(size.width(), size.height())
        return QtCore.QSize(side, side)

    def paint(self, painter, rect, mode, state):
        painter.save()
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        side = min(rect.width(), rect.height())
        painter.translate(rect.x() + (rect.width() - side) / 2,
                          rect.y() + (rect.height() - side) / 2)
        painter.scale(side / 24., side / 24.)
        color = QtGui.QColor(self.color)
        if mode == QtGui.QIcon.Mode.Disabled:
            color.setAlphaF(color.alphaF() * .36)
        pen = QtGui.QPen(color, 1.65)
        pen.setCapStyle(QtCore.Qt.PenCapStyle.RoundCap)
        pen.setJoinStyle(QtCore.Qt.PenJoinStyle.RoundJoin)
        painter.setPen(pen)
        painter.setBrush(QtCore.Qt.BrushStyle.NoBrush)
        _draw(painter, self.name)
        painter.restore()

    def pixmap(self, size, mode, state):
        result = QtGui.QPixmap(size)
        result.fill(QtCore.Qt.GlobalColor.transparent)
        painter = QtGui.QPainter(result)
        self.paint(painter, QtCore.QRect(QtCore.QPoint(0, 0), size), mode, state)
        painter.end()
        return result


def icon(name, color="#5f636b", size=20) -> QtGui.QIcon:
    """Build a vector icon that renders crisply at the caller's requested size."""
    if name not in _NAMES:
        raise ValueError(f"unknown capture console icon: {name}")
    if type(size) is not int or not 1 <= size <= 512:
        raise ValueError("icon size must be an integer between 1 and 512")
    if not QtGui.QColor(color).isValid():
        raise ValueError(f"invalid icon color: {color}")
    return QtGui.QIcon(_VectorIconEngine(name, color, size))
