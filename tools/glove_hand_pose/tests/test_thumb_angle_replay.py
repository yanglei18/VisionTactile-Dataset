import copy

import pytest
from PyQt5 import QtWidgets

from vt_glove_pose.config import load_config
from vt_glove_pose.thumb_angle_replay import ThumbAngleReplayWindow, validate_replay


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def recording():
    identity = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]
    profile = dict(kind="vt_glove_pose.thumb_angle_candidate", schema_version=1,
        scope="conditional_scalar_preview", certified_calibration=False, anatomical_accuracy_verified=False,
        config=load_config(), node_ids=[5, 6], bias_dps={"5": [0., 0., 0.], "6": [0., 0., 0.]},
        frames=dict(status="frames_ready", prox_sensor_to_functional=identity, dist_sensor_to_functional=identity,
                    functional_neutral_prox_acc_g=[0., 0., 1.], functional_neutral_dist_acc_g=[0., 0., 1.]),
        axis_fit=dict(candidate_status="inconsistent"))
    rows = [dict(time_s=8. + i / 10., angle_deg=float(i * 5), quality="tracking", node_ids=[5, 6],
                 paired_fresh=True, current_angle_observed=True) for i in range(10)]
    rows[3].update(quality="prediction_only", current_angle_observed=False)
    rows[5].update(angle_deg=None, quality="unavailable", paired_fresh=False, current_angle_observed=False)
    return dict(kind="vt_glove_pose.thumb_angle_replay", schema_version=1, recorded_only=True,
                profile=profile, series=rows)


def test_show_seek_predict_gap_and_paint_without_serial(app, tmp_path):
    data = recording()
    before = copy.deepcopy(data)
    window = ThumbAngleReplayWindow(data)
    window.show()
    app.processEvents()
    try:
        assert not window.playing and window.angle.mode == "observed"
        window.seek(300)
        assert window.angle.mode == "predicted" and "+15.0" in window.angle.readout.text()
        window.seek(500)
        assert window.angle.mode == "stale" and window.angle.readout.text() == "—"
        window.seek(700)
        assert window.angle.mode == "observed"
        assert window.grab().save(str(tmp_path / "replay.png"))
        window.toggle_play()
        assert window.playing
        window.seek(900)
        window.advance()
        assert not window.playing
        window.toggle_play()
        assert window.position_s == 8.
        assert data == before
    finally:
        window.close()


@pytest.mark.parametrize("change", [lambda d: d.update(recorded_only=False),
                                    lambda d: d.update(series=[]),
                                    lambda d: d["series"][1].update(time_s=1.),
                                    lambda d: d["series"][1].update(angle_deg=float("nan")),
                                    lambda d: d["series"][1].update(node_ids=[6, 5])])
def test_invalid_replay_does_not_render(change):
    data = recording()
    change(data)
    with pytest.raises(ValueError):
        validate_replay(data)


def test_repeated_host_timestamps_are_permitted():
    data = recording()
    data["series"][1]["time_s"] = data["series"][0]["time_s"]
    assert len(validate_replay(data)["series"]) == 10


def test_missing_rows_do_not_display_old_angle_as_current_during_gap(app):
    data = recording()
    data["series"] = [data["series"][0], data["series"][-1]]
    data["series"][-1]["time_s"] = 10.
    window = ThumbAngleReplayWindow(data)
    try:
        window.seek(1000)
        assert window.position_s == 9.
        assert window.angle.mode == "stale"
        assert window.angle.readout.text() == "—"
        assert data["series"][0]["paired_fresh"]  # Never mutate recorded evidence.
        window.seek(2000)
        assert window.angle.mode == "observed"
    finally:
        window.close()
