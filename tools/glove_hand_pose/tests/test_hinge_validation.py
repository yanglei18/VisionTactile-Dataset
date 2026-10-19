"""Statistics must not turn missing observations or repeatability into accuracy."""
import copy
import json

import numpy as np
import pytest

from vt_glove_pose.hinge_validation import analyze_hinge_validation, render_validation_html


def replay(duration=115., rate=20., angle=25.):
    return dict(replay_status="completed", settings={"sample_rate_hz": rate},
                observer="synthetic_test_only", series=[
                    dict(host_elapsed_s=float(t), nominal_time_s=float(t), angle_deg=angle,
                         current_angle_observed=True, valid=True, history_loss=False,
                         host_gap_before=False, raw_near_limit=False)
                    for t in np.arange(round(duration * rate)) / rate])


def window(kind, start, end, pose="bent_reference", **extra):
    return dict(kind=kind, pose_id=pose, start_s=start, end_s=end, **extra)


def protocol(count=20):
    return ([window("reference", 0., 1.)]
            + [window("return", 2. + 2 * i, 2.5 + 2 * i, window_id=i + 1) for i in range(count)]
            + [window("hold", 45., 105.), window("common_rotation", 110., 112.)])


def select(data, start, end):
    return [row for row in data["series"] if start <= row["host_elapsed_s"] < end]


def test_complete_candidate_protocol_with_nonzero_reference_and_json_html():
    data = replay()
    for i in range(20):
        for row in select(data, 1.5 + 2 * i, 2. + 2 * i):
            row["angle_deg"] = 45.
    result = analyze_hinge_validation(data, protocol())
    assert result["candidate_status"] == "pass"
    assert all(c["status"] == "pass" for c in result["criteria"].values())
    assert result["windows"][1]["comparison"]["baseline_deg"] == 25.
    assert result["summary"]["return_intervals_with_output_excursion"] == 20
    assert result["absolute_accuracy"]["status"] == "not_evaluated"
    assert result["end_to_end_latency"]["status"] == "not_evaluated"
    assert result["production_acceptance"] == "not_evaluated"
    assert not result["summary"]["output_excursion_is_independent_truth"]
    json.dumps(result, allow_nan=False)
    rendered = render_validation_html(result)
    assert "不是产品合格证明" in rendered
    assert "绝对角度精度：未评估" in rendered
    assert "http://" not in rendered and "https://" not in rendered


def test_constant_output_can_only_show_stability_and_warns_no_motion_evidence():
    result = analyze_hinge_validation(replay(), protocol())
    assert result["candidate_status"] == "pass"  # Not an action recognizer or accuracy claim.
    assert result["summary"]["return_intervals_with_output_excursion"] == 0
    assert result["warnings"]
    assert any("冻结" in text for text in result["limitations"])


def test_nineteen_returns_cannot_conclude_twenty_cycle_criterion():
    result = analyze_hinge_validation(replay(), protocol(19))
    metric = result["criteria"]["return_repeatability"]
    assert metric["adequate_window_count"] == 19
    assert metric["status"] == "insufficient_data"


def test_prediction_outlier_is_included_and_fails_maximum_not_p95_only():
    data = replay()
    row = select(data, 2., 2.5)[2]
    row.update(angle_deg=80., current_angle_observed=False, valid=False)
    result = analyze_hinge_validation(data, protocol())
    returned = result["windows"][1]
    assert returned["angle"]["max_deg"] == 80.
    assert returned["comparison"]["max_abs_deviation_deg"] == 55.
    assert returned["quality"]["prediction_fraction"] == .1
    assert returned["data_sufficient"]
    assert result["criteria"]["return_repeatability"]["status"] == "fail"


def test_all_prediction_zero_error_is_not_sufficient_observed_validation():
    data = replay()
    for row in data["series"]:
        row.update(current_angle_observed=False, valid=False)
    result = analyze_hinge_validation(data, protocol())
    assert result["candidate_status"] == "insufficient_data"
    assert result["windows"][1]["angle"]["median_deg"] == 25.
    assert result["windows"][1]["quality"]["prediction_fraction"] == 1.


@pytest.mark.parametrize("marker_status", ["cancelled", "interrupted"])
def test_incomplete_marker_keeps_statistics_but_never_passes(marker_status):
    windows = protocol()
    windows[1]["status"] = marker_status
    result = analyze_hinge_validation(replay(), windows)
    assert result["windows"][1]["angle"]["median_deg"] == 25.
    assert result["windows"][1]["status"] == "insufficient_data"
    assert result["criteria"]["return_repeatability"]["status"] == "insufficient_data"


def test_missing_windows_and_empty_replay_have_explicit_insufficient_reports():
    result = analyze_hinge_validation(replay(duration=0), protocol())
    assert result["candidate_status"] == "insufficient_data"
    assert all(window["sample_count"] == 0 for window in result["windows"])
    assert result["windows"][0]["angle"]["median_deg"] is None
    json.dumps(result, allow_nan=False)
    result = analyze_hinge_validation(replay(), [])
    assert all(c["status"] == "insufficient_data" for c in result["criteria"].values())


def test_hold_requires_full_sixty_seconds_not_just_sixty_second_label():
    short = analyze_hinge_validation(replay(), [window("hold", 1, 60)])
    assert "hold_shorter_than_60s" in short["windows"][0]["reasons"]
    truncated = analyze_hinge_validation(replay(duration=40), [window("hold", 1, 61)])
    assert truncated["criteria"]["hold_stability"]["status"] == "insufficient_data"
    assert truncated["windows"][0]["coverage"]["host_span_fraction"] < .7


def test_hold_reports_first_last_and_entire_peak_including_returning_spike():
    data = replay(duration=61)
    select(data, 30, 30.1)[0]["angle_deg"] = 30.
    result = analyze_hinge_validation(data, [window("hold", 0, 60)])
    comparison = result["windows"][0]["comparison"]
    assert comparison["first_to_last_change_deg"] == 0
    assert comparison["peak_to_peak_change_deg"] == 5.
    assert comparison["max_abs_deviation_deg"] == 5.
    assert result["criteria"]["hold_stability"]["status"] == "fail"


def test_hold_slow_drift_cannot_be_renormalized_to_final_pose():
    data = replay(duration=61)
    for row in data["series"]:
        row["angle_deg"] += row["host_elapsed_s"] / 15.
    result = analyze_hinge_validation(data, [window("hold", 0, 60)])
    comparison = result["windows"][0]["comparison"]
    assert comparison["first_to_last_change_deg"] > 3.9
    assert result["criteria"]["hold_stability"]["status"] == "fail"


def test_common_rotation_without_reference_only_claims_within_window_change():
    result = analyze_hinge_validation(replay(duration=3), [window("common_rotation", 0, 2)])
    comparison = result["windows"][0]["comparison"]
    assert comparison["scope"] == "within_window_stability_only"
    assert not comparison["physical_common_rotation_verified"]
    assert comparison["baseline_source"] == "window_first_second"


def test_common_rotation_compares_declared_pose_not_fresh_zero():
    data = replay(duration=4)
    for row in select(data, 2., 3.):
        row["angle_deg"] = 31.
    result = analyze_hinge_validation(data, [window("reference", 0, 1), window("common_rotation", 2, 3)])
    assert result["windows"][1]["comparison"]["max_abs_deviation_deg"] == 6.
    assert result["criteria"]["common_rotation"]["status"] == "fail"


def test_duplicate_overlapping_returns_cannot_inflate_twenty_cycle_count():
    windows = [window("reference", 0, 1)] + [window("return", 2, 3)] * 20
    result = analyze_hinge_validation(replay(duration=4), windows)
    assert result["criteria"]["return_repeatability"]["adequate_window_count"] == 0
    assert all("overlapping_return_windows" in w["reasons"] for w in result["windows"][1:])


def test_first_reference_is_not_replaced_with_later_favorable_reference():
    data = replay(duration=6)
    for row in select(data, 2, 3):
        row["angle_deg"] = 33.
    for row in select(data, 4, 5):
        row["angle_deg"] = 33.
    windows = [window("reference", 2, 3), window("return", 4, 5), window("reference", 0, 1)]
    result = analyze_hinge_validation(data, windows)
    assert result["windows"][1]["comparison"]["baseline_deg"] == 25.
    assert result["windows"][1]["comparison"]["max_abs_deviation_deg"] == 8.
    for row in select(data, 0, 1):
        row.update(valid=False, current_angle_observed=False)
    result = analyze_hinge_validation(data, windows)
    assert "first_same_pose_reference_insufficient" in result["windows"][1]["reasons"]


def test_future_or_different_pose_reference_is_not_used_for_return():
    result = analyze_hinge_validation(replay(duration=5), [window("reference", 3, 4),
                                                          window("return", 1, 2)])
    assert "missing_prior_same_pose_reference" in result["windows"][1]["reasons"]
    result = analyze_hinge_validation(replay(duration=5), [window("reference", 0, 1, pose="other"),
                                                          window("return", 2, 3)])
    assert result["windows"][1]["comparison"]["baseline_deg"] is None


def test_negative_start_keeps_missing_head_and_host_time_not_nominal_selects():
    data = replay(duration=3)
    for row in data["series"]:
        row["nominal_time_s"] += 500.
    result = analyze_hinge_validation(data, [window("reference", -.5, .5), window("common_rotation", 1, 2)])
    assert result["windows"][0]["start_s"] == -.5
    assert not result["windows"][0]["data_sufficient"]
    assert result["windows"][1]["sample_count"] == 20


def test_recovered_observations_and_historical_loss_are_distinct():
    data = replay()
    for row in data["series"]:
        row["history_loss"] = True
    result = analyze_hinge_validation(data, protocol())
    assert result["candidate_status"] == "pass"
    assert result["windows"][1]["quality"]["history_loss_fraction"] == 1.
    assert result["windows"][1]["quality"]["observed_fraction"] == 1.


@pytest.mark.parametrize("flag,reason", [("host_gap_before", "interior_host_gap"),
                                        ("raw_near_limit", "raw_near_limit_in_window")])
def test_known_interior_missing_history_or_saturation_cannot_pass(flag, reason):
    data = replay(duration=3)
    select(data, 1, 2)[3][flag] = True
    result = analyze_hinge_validation(data, [window("common_rotation", 1, 2)])
    assert reason in result["windows"][0]["reasons"]
    assert result["criteria"]["common_rotation"]["status"] == "insufficient_data"


def test_sparse_samples_and_large_host_gaps_are_visible():
    data = replay(duration=4)
    data["series"] = [row for row in data["series"] if not 1.4 <= row["host_elapsed_s"] < 2.2]
    result = analyze_hinge_validation(data, [window("common_rotation", 1, 3)])
    assert result["windows"][0]["coverage"]["interior_host_gap_count"] == 1
    assert result["windows"][0]["coverage"]["nominal_sample_fraction"] < .9


def test_batched_identical_host_timestamps_are_legal_not_false_dropped_pairs():
    data = replay(duration=3)
    for index, row in enumerate(data["series"]):
        row["host_elapsed_s"] = (index // 2) / 10.
    result = analyze_hinge_validation(data, [window("common_rotation", 0, 2)])
    assert result["windows"][0]["coverage"]["interior_host_gap_count"] == 0
    assert result["criteria"]["common_rotation"]["status"] == "pass"


def test_baseline_observer_without_current_observed_key_uses_its_actual_flags():
    data = replay(duration=3)
    for row in data["series"]:
        del row["current_angle_observed"]
        row.update(acc_observable=True, correction_active=True)
    result = analyze_hinge_validation(data, [window("common_rotation", 0, 2)])
    assert result["criteria"]["common_rotation"]["status"] == "pass"


def test_nonfinite_angles_count_as_missing_while_finite_errors_still_count():
    data = replay(duration=3)
    for row, angle in zip(data["series"], [None, float("nan"), float("inf"), 99.]):
        row["angle_deg"] = angle
    result = analyze_hinge_validation(data, [window("common_rotation", 0, 2)])
    item = result["windows"][0]
    assert item["finite_angle_count"] == 37
    assert item["angle"]["max_deg"] == 99.
    assert item["status"] == "insufficient_data"
    json.dumps(result, allow_nan=False)


def test_external_reference_argument_does_not_enable_unimplemented_accuracy():
    result = analyze_hinge_validation(replay(), protocol(), reference_source={"claimed_truth": [0, 0]})
    assert result["reference_source_provided"]
    assert result["absolute_accuracy"]["status"] == "not_evaluated"
    assert result["external_reference_processing"] == "not_implemented"


def test_incomplete_replay_cannot_be_promoted_by_good_windows():
    data = replay()
    data["replay_status"] = "initial_gravity_unobservable"
    result = analyze_hinge_validation(data, protocol())
    assert result["candidate_status"] == "insufficient_data"
    assert all("replay_not_completed" in w["reasons"] for w in result["windows"])


def test_stats_p95_definition_and_no_input_mutation_or_automatic_zero():
    data = replay(duration=2)
    for index, row in enumerate(data["series"]):
        row["angle_deg"] = 50. + index
    windows = [window("reference", 0, 1)]
    saved = copy.deepcopy((data, windows))
    result = analyze_hinge_validation(data, windows)
    stats = result["windows"][0]["angle"]
    assert stats["median_deg"] == 59.5
    assert stats["range_deg"] == 19.
    assert stats["p95_range_deg"] == pytest.approx(18.05)
    assert (data, windows) == saved


@pytest.mark.parametrize("bad", [None, {}, "windows", [window("unknown", 0, 1)],
                                  [window([], 0, 1)], [window("hold", 1, 1)],
                                  [window("hold", float("nan"), 2)], [window("hold", 0, 2, pose="")],
                                  [window("hold", 0, 2, status="auto")],
                                  [window("hold", 0, 2, status=[])],
                                  [window("hold", 0, 2, window_id=True)],
                                  [window("hold", 0, 2, note=12)],
                                  [window("hold", 0, 2)] * 257])
def test_invalid_windows_raise_value_error(bad):
    with pytest.raises(ValueError):
        analyze_hinge_validation(replay(duration=0), bad)


@pytest.mark.parametrize("field,value", [("host_elapsed_s", -1), ("host_elapsed_s", float("nan")),
                                        ("host_elapsed_s", "1"), ("angle_deg", True),
                                        ("angle_deg", "1"), ("valid", 1)])
def test_invalid_row_fields_raise_value_error(field, value):
    data = replay(duration=1)
    data["series"][0][field] = value
    with pytest.raises(ValueError):
        analyze_hinge_validation(data, [])


def test_bounds_rate_order_and_series_shapes():
    data = replay(duration=1)
    data["settings"]["sample_rate_hz"] = 0
    with pytest.raises(ValueError):
        analyze_hinge_validation(data, [])
    data = replay(duration=1)
    data["series"][1]["host_elapsed_s"] = .2
    with pytest.raises(ValueError):
        analyze_hinge_validation(data, [])
    data["series"] = [data["series"][0]] * 30001
    with pytest.raises(ValueError):
        analyze_hinge_validation(data, [])


def test_html_escapes_markers_and_pose_text():
    result = analyze_hinge_validation(replay(duration=2), [window("reference", 0, 1,
        pose='<script>alert("x")</script>', label='<img src=x onerror=alert(1)>')])
    rendered = render_validation_html(result)
    assert "<script>" not in rendered and "<img" not in rendered
    assert "&lt;script&gt;" in rendered and "&lt;img" in rendered
    with pytest.raises(ValueError):
        render_validation_html({})
