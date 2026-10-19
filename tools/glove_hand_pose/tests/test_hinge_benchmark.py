import json

import numpy as np
import pytest

from vt_glove_pose.hinge_benchmark import (CASE_NAMES, make_scenario, benchmark_observers,
                                         _metrics, _crossing_time, _rotate, _run_trace)


@pytest.mark.parametrize("name", CASE_NAMES)
def test_fixed_scenario_shapes_reproducibility_and_finite_truth(name):
    case = make_scenario(name, sample_rate_hz=60., seed=3)
    again = make_scenario(name, sample_rate_hz=60., seed=3)
    n = len(case["time_s"])
    for key in ("prox_gyro_dps", "dist_gyro_dps", "prox_acc_g", "dist_acc_g"):
        assert case[key].shape == (n, 3)
        np.testing.assert_array_equal(case[key], again[key])
        assert np.isfinite(case[key]).all()
    assert case["truth_angle_deg"].shape == (n,)
    assert case["near_limit_mask"].shape == (n, 2)
    np.testing.assert_allclose(np.diff(case["time_s"]), 1/60.)


def test_common_rotation_satisfies_hinge_vector_kinematics_within_scripted_noise():
    case = make_scenario("common_rotation")
    angle = np.deg2rad(case["truth_angle_deg"])
    gd = _rotate(case["prox_gyro_dps"], angle, 0)
    ad = _rotate(case["prox_acc_g"], angle, 0)
    assert np.sqrt(np.mean((gd-case["dist_gyro_dps"])**2)) < .15
    assert np.sqrt(np.mean((ad-case["dist_acc_g"])**2)) < .003
    np.testing.assert_array_equal(case["truth_rate_dps"], 0.)


def test_saturation_scenario_really_loses_reported_rotation():
    case = make_scenario("saturation_recovery")
    assert case["near_limit_mask"].any()
    assert np.max(np.abs(case["dist_gyro_dps"])) <= 286.7
    assert np.max(case["truth_rate_dps"]) > 1000.
    before = case["time_s"] < 4.
    predicted = np.sum(case["prox_gyro_dps"][before,0]-case["dist_gyro_dps"][before,0])/120.
    assert 80.-predicted > 40.


def test_metrics_include_uncertain_frames_and_do_not_treat_one_good_sample_as_recovery():
    n=200; t=np.arange(n)/100.
    case=dict(time_s=t, truth_angle_deg=np.zeros(n), sample_rate_hz=100.,
              recovery_events=[dict(name="event",time_s=0.)], rising_crossing=None)
    trace=[dict(angle_deg=10.,quality="prediction_only",uncertain=True,history_loss=True) for _ in t]
    trace[100]["angle_deg"]=0.
    metrics=_metrics(case,trace)
    assert metrics["error_p95_abs_deg"]==10.
    assert metrics["error_rms_deg"]>9.9
    assert metrics["uncertain_fraction"]==1.
    assert metrics["recovery"]["event"]["within_two_deg_for_half_second_after_s"] is None


def test_crossing_interpolates_only_in_prescribed_event():
    assert _crossing_time(np.array([0.,1.,2.]),np.array([0.,10.,20.]),
                          dict(start_s=0.,end_s=2.,angle_deg=5.))==.5
    assert _crossing_time(np.array([0.,1.,2.]),np.array([0.,10.,20.]),
                          dict(start_s=2.,end_s=2.,angle_deg=5.)) is None


def test_benchmark_runs_both_identical_synthetic_inputs_without_hardware_claim():
    report=benchmark_observers(cases=["bent_hold_bias"], sample_rate_hz=30., seed=5, include_series=True)
    assert not report["hardware_accuracy_verified"]
    case=report["cases"]["bent_hold_bias"]
    assert set(case["observers"])=={"baseline","constrained"}
    for result in case["observers"].values():
        assert result["samples"]==len(result["series"])==1801
        assert abs(result["series"][-1]["angle_deg"])>40.  # no fake return-to-zero
    json.dumps(report,allow_nan=False)


@pytest.mark.parametrize("options",[dict(cases=[]),dict(cases=["missing"]),
                                      dict(cases=["bent_hold_bias"]*2),
                                      dict(sample_rate_hz=True),dict(sample_rate_hz=10.),dict(seed=-1)])
def test_invalid_benchmark_arguments(options):
    with pytest.raises(ValueError):
        benchmark_observers(**options)


def test_benchmark_cli_reports_metrics_without_series_by_default(tmp_path):
    from vt_glove_pose.cli import main
    path=tmp_path/"benchmark.json"
    assert main(["benchmark-hinge","--cases","bent_hold_bias","--sample-rate-hz","30",
                 "--output",str(path)])==0
    report=json.loads(path.read_text())
    assert report["source"]=="analytic_synthetic_not_hardware"
    assert "series" not in report["cases"]["bent_hold_bias"]["observers"]["baseline"]


@pytest.mark.parametrize("name",["linear_acceleration_burst","unit_norm_direction_jump"])
def test_rejected_observation_recovers_without_claiming_missing_raw_motion(name):
    case=make_scenario(name,sample_rate_hz=60.)
    trace=_run_trace(case,"constrained")
    assert trace[-1]["current_angle_observed"]
    assert not trace[-1]["history_loss"]
    assert not trace[-1]["uncertain"]
    assert sum(trace[-1]["rejection_counts"].values())>0


def test_saturation_keeps_history_but_distinguishes_current_observation():
    case=make_scenario("saturation_recovery",sample_rate_hz=60.)
    trace=_run_trace(case,"constrained")
    assert trace[-1]["history_loss"]
    assert trace[-1]["current_angle_observed"]
    assert abs(trace[-1]["angle_deg"]-case["truth_angle_deg"][-1])<1.


def test_unobservable_hold_does_not_report_confident_zero_or_verified_error_bound():
    case=make_scenario("unobservable_hold",sample_rate_hz=60.)
    trace=_run_trace(case,"constrained")
    assert not trace[-1]["current_angle_observed"]
    assert trace[-1]["uncertain"]
    assert trace[-1]["time_since_angle_observation_s"]>10.
    assert trace[-1]["angle_std_deg"]>trace[0]["angle_std_deg"]
    assert trace[-1]["uncertainty_is_model_based_not_validated_accuracy"]
