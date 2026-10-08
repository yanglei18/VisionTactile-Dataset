"""Recorded orientation viewer interactions; no serial or hardware required."""
from copy import deepcopy
import json
import math
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
import pytest
from PyQt5 import QtCore, QtWidgets

from vt_glove_pose.thumb_preview_gui import (AnglePlot, OrientationPairWidget, RotationPlotWidget,
                                           ThumbPreviewWindow, _angle, load_preview, validate_preview)


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def row(t, angle):
    theta = math.radians(angle)/2
    return dict(time_s=t, quat_prox=[math.cos(theta), math.sin(theta), 0., 0.],
                quat_dist=[math.cos(theta/2), 0., math.sin(theta/2), 0.],
                rotation_prox_deg=angle, rotation_dist_deg=angle/2,
                gravity_prox_deg=angle, gravity_dist_deg=angle/2)


def data():
    return dict(kind="vt_glove_pose.thumb_orientation_preview", schema_version=1, recorded_only=True,
                source_directory="recorded/export", config_provenance={"hardware_verified": False},
                runs=[dict(name="第一次运动", samples=[row(i*.1, i*10.) for i in range(6)],
                           legacy_samples=[row(i*.1, i*5.) for i in range(6)], summary={}),
                      dict(name="第二次运动", samples=[row(i*.2, i*5.) for i in range(4)],
                           legacy_samples=[], summary={})])


@pytest.fixture
def window(app):
    result = ThumbPreviewWindow(data())
    yield result
    result.close()
    app.processEvents()


def test_load_json_preserves_source_and_optional_legacy_run(tmp_path):
    path = tmp_path/"preview.json"
    path.write_text(json.dumps(data()), encoding="utf-8")
    result = load_preview(path)
    assert result["recorded_only"] is True
    assert len(result["runs"]) == 2
    assert result["runs"][1]["legacy_samples"] == []


@pytest.mark.parametrize("failure", ["kind", "recorded_only", "backwards", "quaternion", "nonfinite"])
def test_invalid_preview_is_not_silently_rendered(failure):
    value = data()
    if failure == "kind": value["kind"] = "some_other_report"
    elif failure == "recorded_only": value["recorded_only"] = False
    elif failure == "backwards": value["runs"][0]["samples"][2]["time_s"] = -.1
    elif failure == "quaternion": value["runs"][0]["samples"][0]["quat_prox"] = [0, 0, 0, 0]
    else: value["runs"][0]["samples"][0]["rotation_dist_deg"] = float("nan")
    with pytest.raises(ValueError):
        validate_preview(value)


def test_seek_changes_both_independent_orientations_and_readout(window):
    window.seek.setValue(3)
    assert window.sample_index == 3
    assert _angle(window.canvas.quaternions[0]) == pytest.approx(30.)
    assert _angle(window.canvas.quaternions[1]) == pytest.approx(15.)
    assert "30.0°" in window.values.text()
    assert "0.30 / 0.50" in window.clock_label.text()
    assert window.plot.current_time == pytest.approx(.3)
    assert "不是解剖关节角" in window.warning.text()


def test_reference_button_changes_only_display_not_data_or_plotted_curves(window):
    original = deepcopy(window.data)
    window.seek.setValue(3)
    window.zero_button.click()
    assert _angle(window.canvas.quaternions[0]) == pytest.approx(0.)
    assert _angle(window.canvas.quaternions[1]) == pytest.approx(0.)
    assert window.data == original
    assert window.plot.rows[3]["rotation_prox_deg"] == 30.
    window.seek.setValue(4)
    assert _angle(window.canvas.quaternions[0]) == pytest.approx(10.)
    assert _angle(window.canvas.quaternions[1]) == pytest.approx(5.)
    window.restore_button.click()
    assert _angle(window.canvas.quaternions[0]) == pytest.approx(40.)
    assert window.data == original


def test_playback_uses_elapsed_time_speed_and_pauses_at_last_sample(window):
    window.speed.setCurrentIndex(3)  # 2x
    window.play_button.click()
    assert window.playing and window.timer.isActive()
    window.advance(.16)
    assert window.sample_index == 3
    window.advance(.2)
    assert window.sample_index == 5
    assert not window.playing and not window.timer.isActive()
    window.play_button.click()
    assert window.sample_index == 0 and window.playing
    window.play_button.click()
    assert not window.playing


def test_run_switch_resets_time_reference_and_play_state(window):
    window.seek.setValue(4)
    window.zero_display()
    window.toggle_play()
    window.run_selector.setCurrentIndex(1)
    assert window.run_index == 1 and window.sample_index == 0
    assert not window.playing
    assert window.seek.maximum() == 3
    assert not window.legacy_checkbox.isEnabled()
    np.testing.assert_array_equal(window.references[0], [1, 0, 0, 0])


def test_legacy_checkbox_only_changes_plot_visibility(window):
    before = deepcopy(window.data)
    window.legacy_checkbox.setChecked(False)
    assert not window.plot.show_legacy
    assert window.data == before


def test_shared_live_widgets_accept_same_sample_schema_without_io(app):
    canvas, plot = OrientationPairWidget(), RotationPlotWidget()
    sample = row(1., 70.)
    try:
        canvas.set_sample(sample)
        assert _angle(canvas.quaternions[0]) == pytest.approx(70.)
        canvas.set_sample(sample, [sample["quat_prox"], sample["quat_dist"]])
        assert _angle(canvas.quaternions[0]) == pytest.approx(0.)
        plot.set_series([row(0., 0.), sample])
        plot.set_time(.5)
        assert plot.current_time == .5 and plot.legacy == []
    finally:
        canvas.close(); plot.close()


def test_disconnected_or_stale_live_sample_clears_pose_and_node_labels_configurable(app):
    canvas = OrientationPairWidget()
    try:
        assert canvas.quaternions == [None, None]
        canvas.set_node_ids(3, 4)
        assert canvas.node_ids == [3, 4]
        canvas.set_sample(row(0., 55.))
        assert _angle(canvas.quaternions[0]) == pytest.approx(55.)
        canvas.set_sample(None)
        assert canvas.quaternions == [None, None]
        canvas.show(); app.processEvents()
        assert not canvas.grab().isNull()
        with pytest.raises(ValueError):
            canvas.set_node_ids(5, 5)
    finally:
        canvas.close()


def test_qpainter_renders_nonempty_window_and_offscreen_screenshot(window, app, tmp_path):
    window.show()
    window.seek.setValue(4)
    app.processEvents()
    pixmap = window.grab()
    assert not pixmap.isNull() and pixmap.width() >= 1000 and pixmap.height() >= 700
    assert pixmap.save(str(tmp_path/"preview.png"))
    canvas_image = window.canvas.grab().toImage()
    colors = {canvas_image.pixel(x, y) for x in range(0, canvas_image.width(), 10)
              for y in range(0, canvas_image.height(), 10)}
    assert len(colors) > 15
    assert isinstance(window.plot, AnglePlot)


def test_one_frame_run_can_play_and_finish_without_timer_loop(app):
    value = data()
    value["runs"][0]["samples"] = [row(0., 20.)]
    window = ThumbPreviewWindow(value)
    try:
        window.toggle_play()
        window.advance(.1)
        assert not window.playing and window.sample_index == 0
    finally:
        window.close()
