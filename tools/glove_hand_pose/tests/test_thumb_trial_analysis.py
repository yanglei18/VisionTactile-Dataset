"""Known quaternion trajectories validate measurements, not device accuracy."""
from copy import deepcopy
import json

import numpy as np
import pytest

from vt_glove_pose.math3d import from_axis_angle
from vt_glove_pose.thumb_trial_analysis import analyze_trial


START = 100.


def rows(orientation=None):
    result = []
    for i in range(31*20):
        t = i/20
        nodes = {}
        for node in (5, 6):
            q = from_axis_angle([1, 0, 0], 0.) if orientation is None else orientation(t, node)
            nodes[str(node)] = dict(sensor_quat_wxyz=q.tolist(), relative_quat_wxyz=q.tolist(),
                                    gyro_dps=[.3, -.2, .1], accel_g=[0., 0., 1.], fresh=True)
        result.append(dict(host_monotonic_s=START+t, pair_count=i+1, reference_ready=True,
                           valid=True, reasons=[], nodes=nodes))
    return result


def analyze(data, **kwargs):
    return analyze_trial(data, START, [5, 6], **kwargs)


def test_stationary_trial_measures_zero_without_claiming_pass():
    report = analyze(rows())
    assert report["status"] == "available"
    assert not report["acceptance_evaluated"] and not report["anatomical_joint_angles"]
    for value in report["nodes"].values():
        for stage in ("initial_hold", "final_hold"):
            assert value[stage]["orientation_change_deg"] == pytest.approx(0.)
            assert value[stage]["acceleration_direction_change_deg"] == pytest.approx(0.)
            assert value[stage]["gyro_rms_dps"] == pytest.approx(np.sqrt(.14))
            assert value[stage]["gyro_variation_rms_dps"] == pytest.approx(0., abs=1e-12)
        assert value["return_orientation_difference_deg"] == pytest.approx(0.)
    json.dumps(report, allow_nan=False)
    assert "不是关节误差" in report["summary"]


def test_known_orientation_drift_measures_each_hold_and_first_last_difference():
    data = rows(lambda t, node: from_axis_angle([0, 0, 1], np.deg2rad(t*(.2 if node == 5 else .4))))
    report = analyze(data)
    for node, rate in (("5", .2), ("6", .4)):
        result = report["nodes"][node]
        assert result["initial_hold"]["orientation_change_deg"] == pytest.approx(4*rate, abs=1e-8)
        assert result["final_hold"]["orientation_change_deg"] == pytest.approx(7*rate, abs=1e-8)
        assert result["return_orientation_difference_deg"] == pytest.approx(23*rate, abs=1e-8)
        assert result["return_window_basis"] == {"initial_relative_s": [7., 8.], "final_relative_s": [30., 31.]}
        assert result["return_acceleration_direction_difference_deg"] == pytest.approx(0.)


def test_planned_hold_is_not_reported_as_verified_static_even_with_complete_data():
    data = rows()
    for row in data:
        for node in row["nodes"].values():
            node["gyro_dps"] = [30., 0., 0.]
    report = analyze(data)
    assert report["status"] == "available"
    assert report["actual_static_holds_verified"] is False
    assert "计划保持段" in report["summary"]
    assert "不证明实际静止" in report["summary"]
    assert "静止变化" not in report["summary"]
    assert report["nodes"]["5"]["final_hold"]["gyro_rms_dps"] == pytest.approx(30.)


def test_same_rotation_magnitude_different_axes_is_not_zero_return_difference():
    # Scalar rotation magnitude is always30°, but Rx(30) and Ry(30) differ.
    qx, qy = (from_axis_angle(axis, np.deg2rad(30)) for axis in ([1, 0, 0], [0, 1, 0]))
    report = analyze(rows(lambda t, node: qx if t < 10 else qy))
    expected = np.rad2deg(2*np.arccos(abs(qx @ qy)))
    assert expected > 40
    for result in report["nodes"].values():
        assert result["return_orientation_difference_deg"] == pytest.approx(expected)


def test_quaternion_sign_changes_leave_means_and_angles_unchanged():
    data = rows(lambda t, node: from_axis_angle([1, 2, 3], np.deg2rad(t*.3)))
    flipped = deepcopy(data)
    for i, row in enumerate(flipped):
        for node in ("5", "6"):
            if i % 2:
                row["nodes"][node]["sensor_quat_wxyz"] = [-v for v in row["nodes"][node]["sensor_quat_wxyz"]]
    a, b = analyze(data), analyze(flipped)
    for node in ("5", "6"):
        assert a["nodes"][node]["return_orientation_difference_deg"] == pytest.approx(b["nodes"][node]["return_orientation_difference_deg"], abs=1e-9)


@pytest.mark.parametrize("problem", ["missing", "gap", "invalid", "bad_quaternion", "stale"])
def test_bad_data_never_becomes_zero_or_a_success(problem):
    data = rows()
    index = 25*20
    if problem == "missing":
        del data[index]
    elif problem == "gap":
        del data[index:index+10]
    elif problem == "invalid":
        data[index]["valid"] = False
    elif problem == "bad_quaternion":
        data[index]["nodes"]["5"]["sensor_quat_wxyz"] = [float("nan"), 0, 0, 0]
    else:
        data[index]["nodes"]["5"]["fresh"] = False
    report = analyze(data)
    assert report["nodes"]["5"]["initial_hold"]["status"] == "available"
    assert report["nodes"]["5"]["final_hold"]["orientation_change_deg"] is None
    assert report["nodes"]["5"]["return_orientation_difference_deg"] is None
    assert report["nodes"]["5"]["return_reasons"]
    json.dumps(report, allow_nan=False)


def test_gap_in_motion_keeps_local_holds_but_invalidates_cross_gap_comparison():
    data = rows()
    del data[12*20:13*20]
    report = analyze(data)
    for value in report["nodes"].values():
        assert value["initial_hold"]["status"] == "available"
        assert value["final_hold"]["status"] == "available"
        assert value["return_orientation_difference_deg"] is None
        assert "host_gap" in value["return_reasons"]


def test_cancelled_capture_retains_completed_initial_hold_not_final_result():
    report = analyze(rows()[:15*20], completion_reason="operator_cancelled")
    assert report["status"] == "partial"
    assert report["nodes"]["5"]["initial_hold"]["orientation_change_deg"] == pytest.approx(0.)
    assert report["nodes"]["5"]["final_hold"]["orientation_change_deg"] is None
    assert report["nodes"]["5"]["return_orientation_difference_deg"] is None
    assert "trial_not_completed" in report["nodes"]["5"]["return_reasons"]
    assert "未完成" in report["summary"]


def test_even_full_data_cancelled_does_not_claim_completed_return_result():
    report = analyze(rows(), completion_reason="operator_cancelled")
    assert report["nodes"]["5"]["return_orientation_difference_deg"] is None


def test_empty_data_has_explicit_null_metrics_and_unavailable_summary():
    report = analyze([])
    assert report["status"] == "unavailable"
    assert report["nodes"]["5"]["initial_hold"]["orientation_change_deg"] is None
    assert "不可计算" in report["summary"]
    json.dumps(report, allow_nan=False)


def test_reference_lost_during_a_started_trial_prevents_cross_reset_comparison():
    data = rows()
    for row in data:
        row.update(valid=False, reference_ready=False, reasons=["display_reference_not_set"])
    assert analyze(data)["status"] == "unavailable"


def test_acceleration_direction_and_gyro_variation_are_separate_observations():
    data = rows()
    for i, row in enumerate(data):
        t = row["host_monotonic_s"]-START
        for node in row["nodes"].values():
            node["gyro_dps"] = [.3 + (.1 if i % 2 else -.1), -.2, .1]
            if t >= 23:
                node["accel_g"] = [0., .5, np.sqrt(.75)]
    report = analyze(data)
    value = report["nodes"]["5"]
    assert value["return_acceleration_direction_difference_deg"] == pytest.approx(30.)
    assert value["return_orientation_difference_deg"] == pytest.approx(0.)
    assert value["initial_hold"]["gyro_variation_rms_dps"] == pytest.approx(.1)


@pytest.mark.parametrize("row_edit", [lambda r: r.update(pair_count=0),
    lambda r: r.update(pair_count=True), lambda r: r.pop("host_monotonic_s"),
    lambda r: r.update(host_monotonic_s=float("nan"))])
def test_malformed_rows_do_not_get_silently_dropped(row_edit):
    data = rows()
    row_edit(data[4*20])
    report = analyze(data)
    assert report["nodes"]["5"]["return_orientation_difference_deg"] is None


def test_duplicate_timestamps_not_duplicate_pair_counts_are_preserved():
    data = rows()
    for i in range(1, len(data), 2):
        data[i]["host_monotonic_s"] = data[i-1]["host_monotonic_s"]
    assert analyze(data)["status"] == "available"
    data[80]["pair_count"] = data[79]["pair_count"]
    assert analyze(data)["nodes"]["5"]["initial_hold"]["orientation_change_deg"] is None
