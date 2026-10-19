"""PySide-only geometry for the existing, explicit tactile hand schematic.

Only the dependency-free layout reader is shared with the standalone viewer.
Neither its Qt binding nor ROS/acquisition code is imported by the workbench.
"""

from functools import lru_cache

from PySide6 import QtCore, QtGui


class TactileHandGeometry:
    """Cached region-local tiles with the standalone viewer's exact geometry."""

    def __init__(self, layout):
        self.layout = layout
        self.outline = QtGui.QPainterPath()
        for index, (x, y) in enumerate(layout.outline):
            self.outline.moveTo(x, y) if index == 0 else self.outline.lineTo(x, y)
        self.outline.closeSubpath()
        bounds = self.outline.boundingRect()
        by_address = {(point.row, point.col): point for point in layout.points}
        unit = QtGui.QPainterPath()
        unit.addRoundedRect(QtCore.QRectF(-.42, -.42, .84, .84), .12, .12)
        cells = []
        for point in layout.points:
            vectors = []
            for dr, dc in ((0, 1), (1, 0)):
                vector = None
                for sign in (1, -1):
                    neighbor = by_address.get((point.row + sign * dr, point.col + sign * dc))
                    if neighbor is not None and neighbor.region == point.region:
                        vector = (sign * (neighbor.x - point.x), sign * (neighbor.y - point.y))
                        break
                vectors.append(vector or ((.8, 0) if dc else (0, .8)))
            u, v = vectors
            path = QtGui.QTransform(u[0], u[1], v[0], v[1], point.x, point.y).map(unit)
            cells.append((point, path))
            bounds = bounds.united(path.boundingRect())
        self.cells = tuple(cells)
        self.bounds = bounds.adjusted(-.6, -1.5, .6, 1.5)
        self.anchor_count = sum(point.verified_anchor for point in layout.points)

    def transform(self, rect):
        """Fit without mirroring X; positive layout Y points toward fingertips."""
        scale = max(.001, min(rect.width() / self.bounds.width(), rect.height() / self.bounds.height()))
        center = self.bounds.center()
        return QtGui.QTransform(scale, 0., 0., -scale,
                                rect.center().x() - scale * center.x(),
                                rect.center().y() + scale * center.y())

    def point_at(self, position, rect):
        inverse, invertible = self.transform(rect).inverted()
        if not invertible:
            return None
        local = inverse.map(position)
        for point, path in self.cells:
            if path.contains(local):
                return point
        return None


@lru_cache(maxsize=1)
def right_hand_geometry():
    # Keep legacy viewer.py, ROS and serial entirely outside the Qt6 process.
    from vt_tactile_capture.hand_layout import load_layout

    return TactileHandGeometry(load_layout(expected_hand="right"))
