"""Response semantics survive shared 2D/3D presentation and history copies."""
from types import SimpleNamespace

import numpy as np

from vt_capture_console.ui.widgets.preview import PreviewWidget, _color
from vt_capture_console.ui.widgets.tactile_surface_colors import surface_colors
from vt_capture_console.ui.widgets.tactile_palette import heat_stops


def device(values, **changes):
    frame = dict(values=values, valid_mask=[True] * 400, levels=[0., 1000.],
                 value_kind="tactile_response", reference_ready=True)
    frame.update(changes)
    return dict(id="right_tactile", side="right", kind="tactile", status="online",
                mode="live", age_s=0., preview=frame)


def test_zero_response_is_palette_start_and_never_first_frame_scaled(qapp):
    widget = PreviewWidget("tactile")
    widget.set_device(device([0.] * 400, reference_ready=False))
    assert widget._levels == (0., 1000.)
    assert widget._frame["reference_ready"] is False
    assert widget._cell_colors[0].getRgb()[:3] == heat_stops("tactile_response")[0][1]
    assert "归零" in widget._footer_text()
    widget.set_device(device([100.] * 400))
    assert widget._levels == (0., 1000.)
    assert "ΔADC" in widget._footer_text()
    widget.close()


def test_response_kind_requires_fixed_zero_based_scale(qapp):
    for changes in ({"levels": None}, {"levels": [-1., 1000.]},
                    {"reference_ready": "yes"}, {"value_kind": "pressure_kpa"}):
        widget = PreviewWidget("tactile")
        widget.set_device(device([0.] * 400, **changes))
        assert widget._frame is None and widget._preview_error
        widget.close()


def test_response_metadata_and_scale_are_preserved_in_detail_and_held_frames(qapp):
    source, detail = PreviewWidget("tactile"), PreviewWidget("tactile")
    state = device([100.] * 400)
    for widget in (source, detail):
        widget.set_device(state)
    detail.copy_display_state_from(source)
    assert detail._frame["value_kind"] == "tactile_response"
    source.set_device({**state, "status": "offline", "preview": {}})
    detail.copy_display_state_from(source)
    assert detail.held and "ΔADC" in detail._footer_text()
    source.close()
    detail.close()


def test_surface_uses_response_palette_without_changing_unmapped_skin():
    binding = SimpleNamespace(addresses=np.array([[0, 0], [0, 1]]), valid=np.array([True, True]),
                              vertex_weights=np.array([[1., 0.], [0., 1.], [0., 0.]]))
    base = np.tile([.25, .62, .57, 1.], (3, 1))
    frame = device([0., 1000.] + [0.] * 398)["preview"]
    palette = heat_stops(frame["value_kind"])
    colors = surface_colors(binding, frame, base, (0., 1000.), palette)
    np.testing.assert_allclose(colors[0, :3], np.array(palette[0][1]) / 255.)
    np.testing.assert_allclose(colors[1, :3], np.array(palette[-1][1]) / 255.)
    np.testing.assert_allclose(colors[2], base[2])
    assert _color(0., 0., 1000.).getRgb()[:3] == heat_stops()[0][1]
