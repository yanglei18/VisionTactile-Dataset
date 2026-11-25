"""Camera image display and metric projection shared by tracking and mount calibration."""
import cv2
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets


class Preview(QtWidgets.QLabel):
    clicked = QtCore.pyqtSignal(object, object)

    def __init__(self):
        super().__init__()
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setMinimumSize(480, 270)
        self.setStyleSheet("background:#202329;color:white")
        self.source_geometry = None

    def show_frame(self, frame, stamp):
        height, width = frame.shape[:2]
        image = QtGui.QImage(frame.data, width, height, frame.strides[0], QtGui.QImage.Format_BGR888)
        pixmap = QtGui.QPixmap.fromImage(image).scaled(
            self.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        self.setPixmap(pixmap)
        self.source_geometry = (width, height, stamp)

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton and self.source_geometry:
            width, height, stamp = self.source_geometry
            pixmap = self.pixmap()
            if pixmap and not pixmap.isNull():
                x = event.x() - (self.width() - pixmap.width()) / 2
                y = event.y() - (self.height() - pixmap.height()) / 2
                if 0 <= x < pixmap.width() and 0 <= y < pixmap.height():
                    self.clicked.emit((x * width / pixmap.width(), y * height / pixmap.height()), stamp)
        super().mousePressEvent(event)


def draw_hand(frame, camera, distortion, coordinates, color):
    """The five MANO chains follow wrist, thumb, index, middle, ring, little."""
    points = np.asarray(coordinates, dtype=np.float64).reshape(21, 3)
    if not np.isfinite(points).all() or np.min(points[:, 2]) <= 0:
        return
    xy = cv2.projectPoints(points, np.zeros(3), np.zeros(3), camera, distortion)[0]
    if not np.isfinite(xy).all() or np.max(np.abs(xy)) > 1e6:
        return
    xy = np.rint(xy.reshape(21, 2)).astype('int32')
    for base in (1, 5, 9, 13, 17):
        cv2.polylines(frame, [xy[[0, base, base+1, base+2, base+3]]], False, color, 2, cv2.LINE_AA)
        cv2.circle(frame, tuple(xy[base+3]), 3, color, -1, cv2.LINE_AA)
