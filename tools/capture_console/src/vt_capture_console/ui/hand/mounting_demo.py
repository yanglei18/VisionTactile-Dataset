"""Teaching poses for mounting and finger calibration on the selected MANO hand."""
from __future__ import annotations

import queue
import threading

import numpy as np
from pyqtgraph import Transform3D
from pyqtgraph.opengl import GLGridItem
from PySide6 import QtCore

from .display_view import DisplayHandView


# These command keys predate the official three-pose protocol. In HandView's
# proper display basis, +Y points toward the fingers and +Z is dorsal for
# either hand. Positive rotation about display X therefore lifts the tips.
_PITCH_DEGREES = {"neutral": 0., "fingers_down": 50., "thumb_up": -50.}


from vt_fsglove12.teaching_geometry import finger_clips as _finger_clips


class MountingDemo(DisplayHandView):
    """Keep static capture targets steady and loop functional flexion examples."""

    def __init__(self, neutral_geometry, model_axes, parent=None, *, mano_assets=None, demo_clips=None):
        super().__init__(neutral_geometry, model_axes, parent)
        self.setMinimumSize(220, 200)
        self.set_show_joints(False)
        joints = np.asarray(neutral_geometry["joints_m"], dtype=float) @ self.basis.T
        self._wrist = joints[0]
        length = max(.10, float(np.linalg.norm(joints[12] - self._wrist)))

        # The level surface ends at the wrist so the hand can tilt below it.
        grid = GLGridItem(color=(128, 151, 157, 145))
        grid.setSize(x=.8 * length, y=.4 * length, z=0.)
        grid.setSpacing(x=.1 * length, y=.1 * length, z=1.)
        grid.translate(*(self._wrist + [0., -.2 * length, -.06 * length]))
        self.addItem(grid)

        self.step = "neutral"
        self._pitch = 0.
        self._animation = QtCore.QVariantAnimation(self)
        self._animation.setDuration(900)
        self._animation.setEasingCurve(QtCore.QEasingCurve.Type.InOutSine)
        self._animation.valueChanged.connect(self._apply_pitch)
        self._neutral = neutral_geometry
        self._clips = demo_clips or {}
        self._requested_step = None
        self._clip_results = queue.Queue(maxsize=1)
        self._disposed = threading.Event()
        self.clip_error = ""
        self._clip_timer = QtCore.QTimer(self)
        self._clip_timer.setInterval(50)
        self._clip_timer.timeout.connect(self._receive_clips)
        if mano_assets is not None and demo_clips is None:
            # Numerical initialization and all 32 model forwards run off Qt.
            # The worker owns only plain data, never a widget or GL object.
            result_queue, stopped, hand = self._clip_results, self._disposed, self.hand
            def prepare():
                try:
                    from vt_fsglove12.mano import ManoBackend
                    result = _finger_clips(ManoBackend(mano_assets, hand=hand,
                                          compact_live_geometry=True), neutral_geometry)
                    item = (result, "")
                except Exception as exc:
                    item = ({}, str(exc)[:1000])
                if not stopped.is_set():
                    result_queue.put_nowait(item)
            threading.Thread(target=prepare, name=f"{hand}-hand-demo", daemon=True).start()
            self._clip_timer.start()
        self._flexion = QtCore.QVariantAnimation(self)
        self._flexion.setDuration(2400)
        self._flexion.setLoopCount(-1)
        self._flexion.setStartValue(0.)
        self._flexion.setKeyValueAt(.5, 1.)
        self._flexion.setEndValue(0.)
        self._flexion.setEasingCurve(QtCore.QEasingCurve.Type.InOutSine)
        self._flexion.valueChanged.connect(self._apply_flexion)

    def _receive_clips(self):
        try:
            self._clips, self.clip_error = self._clip_results.get_nowait()
        except queue.Empty:
            return
        self._clip_timer.stop()
        requested, self._requested_step = self._requested_step, None
        if requested in self._clips:
            self.set_step(requested)

    def dispose(self):
        self._disposed.set()
        self._clip_timer.stop()
        self._animation.stop()
        self._flexion.stop()

    def _apply_flexion(self, phase):
        vertices, joints = self._clips[self.step]
        position = phase * (len(vertices) - 1)
        index = min(int(position), len(vertices) - 2)
        weight = position - index
        self.set_geometry(dict(self._neutral, synthetic_demonstration=True,
                               vertices_m=(1 - weight) * vertices[index] + weight * vertices[index + 1],
                               joints_m=(1 - weight) * joints[index] + weight * joints[index + 1]))

    def _apply_pitch(self, degrees):
        self._pitch = degrees
        transform = Transform3D()
        transform.translate(*self._wrist)
        transform.rotate(degrees, 1., 0., 0.)
        transform.translate(*-self._wrist)
        self.mesh.setTransform(transform)
        self.joints.setTransform(transform)

    def set_step(self, step, *, hold=False):
        if step in ("flex_fingers", "flex_thumb") and step not in self._clips:
            self._requested_step = step
            return
        self._requested_step = None
        if step in self._clips:
            if step != self.step:
                self._animation.stop()
                self._flexion.stop()
                self.step = step
                self._apply_pitch(0.)
                self._apply_flexion(0.)
                if self.isVisible():
                    self._flexion.start()
            return
        target = _PITCH_DEGREES[step]
        if step == self.step:
            if hold and self._animation.state() == QtCore.QAbstractAnimation.State.Running:
                self._animation.stop()
                self._apply_pitch(target)
            return
        if self.step in self._clips:
            self._flexion.stop()
            self.set_geometry(self._neutral)
        self.step = step
        self._animation.stop()
        if hold or not self.isVisible():
            self._apply_pitch(target)
        else:
            self._animation.setStartValue(self._pitch)
            self._animation.setEndValue(target)
            self._animation.start()

    def hideEvent(self, event):
        if self.step in self._clips:
            self._flexion.stop()
        else:
            self.set_step(self.step, hold=True)
        super().hideEvent(event)

    def showEvent(self, event):
        super().showEvent(event)
        if self.step in self._clips:
            self._flexion.start()
