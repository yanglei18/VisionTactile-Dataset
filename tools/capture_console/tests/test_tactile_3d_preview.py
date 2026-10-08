"""3D presentation contracts, independent of licensed assets or graphics drivers."""

from types import SimpleNamespace

import numpy as np
import pytest
from PySide6 import QtCore, QtWidgets

from vt_capture_console.ui.main_window import DeviceCard
from vt_capture_console.ui.widgets.preview import PreviewWidget, _HEAT_STOPS, _color
from vt_capture_console.ui.widgets.tactile_surface_colors import raw_surface_colors


def device(side="right", **changes):
    result = {"id": side + "_tactile", "kind": "tactile", "side": side,
              "label": side, "status": "online", "topic": "/tactile/state", "age_s": .01,
              "receive_hz": 30., "valid_hz": 30.,
              "preview": {"rows": 20, "cols": 20, "values": list(range(400)),
                          "valid_mask": [True] * 400, "levels": [0., 399.]}}
    return {**result, **changes}


@pytest.fixture
def fake_3d(monkeypatch):
    from vt_capture_console.ui.widgets import tactile_3d

    class Display(QtWidgets.QWidget):
        def __init__(self, hand, parent):
            super().__init__(parent)
            self.hand = hand
            self.frames = []
            self.disposed = False

        def set_frame(self, frame, levels):
            self.frames.append((frame, levels))

        def dispose(self):
            self.disposed = True
            self.hide()
            self.deleteLater()

    monkeypatch.setattr(tactile_3d, "Tactile3DWidget", Display)
    return Display


def test_third_mode_reuses_validated_values_history_and_cache(qapp, fake_3d):
    widget = PreviewWidget("tactile")
    widget.set_device(device())
    widget.show()
    widget.set_tactile_view("3d")
    model = widget._tactile_3d
    assert isinstance(model, fake_3d)
    assert widget.tactile_view == "3d"
    assert model.frames[-1] == (widget._frame, widget._levels)
    previous = widget._frame
    widget.set_device(device(status="offline", preview={}))
    assert widget._held and widget._frame is previous
    assert model.frames[-1][0] is previous
    widget.set_tactile_view("matrix")
    assert model.isHidden()
    widget.set_tactile_view("3d")
    assert widget._tactile_3d is model and not model.isHidden()
    widget.resize(630, 610)
    qapp.processEvents()
    assert model.geometry() == widget._canvas_rect().toRect()
    widget.set_device(device(side="left"))
    assert model.disposed and widget._tactile_3d is None
    assert widget.tactile_view == "matrix" and not widget.tactile_3d_available
    widget.close()
    widget.deleteLater()


def test_third_mode_card_detail_bidirectional_without_hardware(qapp, fake_3d):
    card = DeviceCard(device())
    card.show()
    card.tactile_controls.three_d_button.click()
    assert card.preview.tactile_view == "3d"
    card.open_tactile_detail()
    detail = card.tactile_dialog
    assert detail.controls.three_d_button.isChecked()
    assert detail.preview.tactile_view == "3d"
    assert detail.preview._frame == card.preview._frame
    assert "非实时手势" in detail.layout_note.text()
    detail.controls.matrix_button.click()
    assert detail.preview.tactile_view == card.preview.tactile_view == "matrix"
    detail.controls.three_d_button.click()
    assert detail.preview.tactile_view == card.preview.tactile_view == "3d"
    assert detail.findChildren(QtCore.QTimer) == []
    card.close_tactile_detail()
    card.close()
    card.deleteLater()


def test_3d_missing_assets_is_local_actionable_and_2d_still_works(qapp, monkeypatch):
    from vt_capture_console.ui.widgets import tactile_3d
    from vt_capture_console.ui.widgets.tactile_model_loader import TactileModelLoader

    loader = TactileModelLoader(lambda: {"process_groups": []})
    monkeypatch.setattr(tactile_3d, "model_loader", lambda: loader)
    widget = PreviewWidget("tactile")
    widget.set_device(device())
    widget.set_tactile_view("3d")
    assert "配置设备" in widget._tactile_3d.message.text()
    assert widget._tactile_3d.view is None
    assert not widget._tactile_3d.retry_button.isHidden()
    widget.set_tactile_view("hand")
    assert widget.tactile_view == "hand" and widget._frame is not None
    assert not loader._jobs
    widget.close()
    widget.deleteLater()
    loader.shutdown()


def test_raw_adc_uses_same_palette_and_fixed_region_weights_without_mutation():
    # Vertex 0 and 1 depend on different sensor addresses; 2 blends them;
    # vertex 3 represents the unmapped back of the hand.
    binding = SimpleNamespace(addresses=np.array([[0, 0], [0, 1]]),
                              valid=np.array([True, True]),
                              vertex_weights=np.array([[1., 0.], [0., 1.], [.75, .25], [0., 0.]]))
    base = np.tile([.25, .62, .57, 1.], (4, 1))
    frame = {"values": [0., 1000.] + [0.] * 398, "valid_mask": [True] * 400}
    colors = raw_surface_colors(binding, frame, base, (0., 1000.), _HEAT_STOPS)
    for vertex, value in enumerate([0., 1000., 250.]):
        rgb = _color(value, 0., 1000.).getRgbF()[:3]
        np.testing.assert_allclose(colors[vertex, :3], rgb, atol=1/255.)
    np.testing.assert_allclose(colors[3], base[3])
    assert frame["values"][1] == 1000. and base[0, 0] == .25
    frame["valid_mask"][1] = False
    colors = raw_surface_colors(binding, frame, base, (0., 1000.), _HEAT_STOPS)
    np.testing.assert_allclose(colors[1, :3], np.array([229, 232, 237]) / 255.)
    np.testing.assert_allclose(colors[2], colors[0])
    frame["values"][1] = float("nan")
    assert np.isfinite(raw_surface_colors(binding, frame, base, (0., 1000.), _HEAT_STOPS)).all()
    np.testing.assert_allclose(raw_surface_colors(binding, None, base, None, _HEAT_STOPS), base)


def test_same_hand_loader_failure_after_success_can_recover(qapp, monkeypatch):
    from vt_capture_console.ui.widgets import tactile_3d
    from vt_capture_console.ui.widgets.tactile_model_loader import TactileModelLoader

    loader = TactileModelLoader(lambda: {"process_groups": []})
    monkeypatch.setattr(tactile_3d, "model_loader", lambda: loader)
    child = tactile_3d.Tactile3DWidget("right")
    # Simulate an existing valid model without creating GL in headless tests.
    old = QtWidgets.QWidget(child)
    child.view, child._bundle = old, {"model": "accepted"}
    child._model_failed("right", "New model missing")
    assert "保留旧静态参考手" in child.message.text()
    assert not child.retry_button.isHidden()
    child._model_ready("right", child._bundle)
    assert child.message.isHidden() and child.retry_button.isHidden()
    assert child.view is old
    child.dispose()
    loader.shutdown()


def test_raw_adc_rejects_inconsistent_shape_and_levels():
    binding = SimpleNamespace()
    with pytest.raises(ValueError, match="400"):
        raw_surface_colors(binding, {"values": [1.], "valid_mask": [True]}, [[0, 0, 0, 1]], (0, 1), _HEAT_STOPS)
    with pytest.raises(ValueError, match="increasing"):
        raw_surface_colors(binding, {"values": [1.] * 400, "valid_mask": [True] * 400},
                           [[0, 0, 0, 1]], (1, 1), _HEAT_STOPS)
