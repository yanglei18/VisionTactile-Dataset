"""Viewport framing for the standalone hand display."""
import math

from vt_fsglove12.hand_view import HandView


def _width_factor(size):
    # The neutral hand fits the shared camera preset at a 3:2 aspect ratio.
    # On shallower canvases, widen the horizontal field to retain that height.
    return max(1., size.width() / (1.5 * max(1, size.height())))


class DisplayHandView(HandView):
    def resizeEvent(self, event):
        # pyqtgraph's field of view is horizontal. Scale its tangent so the
        # user's zoom and orbit survive opening and closing the controls.
        slope = math.tan(math.radians(self.opts["fov"]) / 2)
        # A hidden child can be assigned a size before its first resize
        # event. Only undo a correction that this widget actually applied.
        factor = _width_factor(event.size())
        slope *= factor / getattr(self, "_framing_factor", 1.)
        self._framing_factor = factor
        self.opts["fov"] = math.degrees(2 * math.atan(slope))
        super().resizeEvent(event)
