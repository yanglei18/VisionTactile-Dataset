"""Experimental angle display distinguishes observations from predictions."""
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from copy import deepcopy
import pytest
from PyQt5 import QtWidgets

from vt_glove_pose.thumb_angle_widget import ThumbAngleWidget, angle_display_state, angle_segments


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture
def widget(app):
    result = ThumbAngleWidget()
    yield result
    result.close()
    app.processEvents()


def snapshot(**changes):
    return dict(angle_deg=-24., quality="tracking", current_angle_observed=True,
                paired_fresh=True, history_loss=False, reasons=[], node_ids=[3, 4], **changes)


@pytest.mark.parametrize("state,mode,label", [
    (None, "waiting", "等待"),
    (dict(angle_deg=20., paired_fresh=False), "stale", "过期"),
    (dict(angle_deg=None, paired_fresh=True), "unavailable", "没有可用"),
    (dict(angle_deg=float("nan"), paired_fresh=True), "unavailable", "没有可用"),
    (dict(angle_deg=20., paired_fresh=True, current_angle_observed=True), "observed", "有观测"),
    (dict(angle_deg=20., paired_fresh=True, current_angle_observed=False), "predicted", "仅预测"),
])
def test_state_categories_are_not_quality_or_accuracy_claims(state, mode, label):
    result = angle_display_state(state)
    assert result["mode"] == mode and label in result["label"]
    if mode not in {"observed", "predicted"}:
        assert result["angle_deg"] is None


def test_signed_readout_node_mapping_and_no_zero_button(widget):
    widget.set_state(snapshot(), [])
    assert widget.readout.text() == "-24.0°"
    assert "3 / 4" in widget.title.text()
    assert "不是已认证" in widget.warning.text()
    assert "不会" in widget.warning.text()
    assert widget.findChildren(QtWidgets.QPushButton) == []


def test_reference_and_history_inputs_are_not_modified(widget):
    state = snapshot()
    history = [dict(time_s=0., angle_deg=-20., current_angle_observed=True),
               dict(time_s=.1, angle_deg=-24., current_angle_observed=True)]
    original = deepcopy((state, history))
    widget.set_state(state, history)
    assert (state, history) == original
    state["angle_deg"] = 99.
    history[0]["angle_deg"] = 100.
    assert widget.snapshot["angle_deg"] == -24. and widget.history[0]["angle_deg"] == -20.


def test_stale_sample_clears_current_readout_but_keeps_history(widget):
    rows = [dict(time_s=0., angle_deg=30., current_angle_observed=True)]
    widget.set_state(dict(snapshot(), angle_deg=30.), rows)
    widget.set_state(dict(snapshot(), paired_fresh=False), rows)
    assert widget.mode == "stale" and widget.readout.text() == "—"
    assert len(widget.history) == 1


def test_prediction_and_history_loss_are_visible(widget):
    widget.set_state(dict(snapshot(), current_angle_observed=False, history_loss=True,
                          reasons=["gravity_unobservable"]), [])
    assert widget.mode == "predicted"
    assert "仅预测" in widget.status.text() and "连续性已丢失" in widget.status.text()
    assert "gravity_unobservable" in widget.details.text()
    assert widget.readout.text() == "-24.0°"


def test_uninitialized_geometry_failure_is_not_labelled_disconnected(widget):
    state = dict(snapshot(), angle_deg=None, quality="uninitialized",
                 reasons=["waiting_for_observable_current_gravity", "axis_gravity_mismatch"])
    widget.set_state(state, [])
    assert widget.mode == "unavailable" and widget.readout.text() == "—"
    assert "数据正常" in widget.status.text()
    assert "运动轴与当前姿态不一致" in widget.status.text()
    assert "重新连接或显示归零不能修复" in widget.status.text()


def test_painter_handles_missing_samples_large_signed_angles_and_real_render(widget, app, tmp_path):
    widget.resize(900, 650)
    rows = [dict(time_s=i*.1, angle_deg=20*i-80., current_angle_observed=i%2 == 0, paired_fresh=True)
            for i in range(20)]
    rows[7]["angle_deg"] = None
    rows[12]["paired_fresh"] = False
    widget.set_state(dict(snapshot(), angle_deg=300.), rows)
    widget.show(); app.processEvents()
    assert widget.readout.text() == "+300.0°"
    image = widget.grab()
    assert not image.isNull() and image.save(str(tmp_path/"angle.png"))


def test_plot_does_not_bridge_gaps_but_retains_same_stamp_and_latched_history_loss():
    rows = [dict(time_s=t, angle_deg=i*3., paired_fresh=True, current_angle_observed=True,
                 history_loss=i > 2) for i, t in enumerate([0., .01, .01, .7, .71, .72])]
    assert [(a["time_s"], b["time_s"]) for a,b,_ in angle_segments(rows)] == [(0., .01), (.01, .01), (.7, .71), (.71, .72)]
    rows[4]["angle_deg"] = None
    assert len(angle_segments(rows)) == 2
