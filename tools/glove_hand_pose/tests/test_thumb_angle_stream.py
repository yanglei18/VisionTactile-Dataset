"""Raw-sample candidate preview with unchanged observer and no hardware."""
from copy import deepcopy
import json
import math

import numpy as np
import pytest

from vt_glove_pose.config import gyro_scale, load_config
from vt_glove_pose.hinge_frames import build_hinge_frames
from vt_glove_pose.records import Sample
from vt_glove_pose.thumb_angle_stream import CandidateAngleProcessor, validate_candidate


def rx(degrees):
    x = math.radians(degrees)
    return np.array([[1.,0.,0.],[0.,math.cos(x),-math.sin(x)],[0.,math.sin(x),math.cos(x)]])


def profile(*, neutral_angle=0., nodes=(5,6), overrides=None):
    config = load_config()
    config["gyro_dps_per_lsb_by_node"] = {"5": .0175, "6": .0175} if overrides is None else overrides
    acceleration = np.tile([.2, 0., 1.], (30, 1))
    frames = build_hinge_frames([1.,0.,0.], [1.,0.,0.], acceleration, acceleration,
                                neutral_prox_acc_g=[.2,0.,1.], neutral_dist_acc_g=[.2,0.,1.])
    assert frames["status"] == "frames_ready"
    frames["functional_neutral_dist_acc_g"] = (rx(neutral_angle)@np.array(frames["functional_neutral_prox_acc_g"])).tolist()
    return dict(kind="vt_glove_pose.thumb_angle_candidate", schema_version=1,
                scope="conditional_scalar_preview", certified_calibration=False,
                anatomical_accuracy_verified=False, config=config, node_ids=list(nodes),
                bias_dps={str(n):[.105, -.035, .0175] for n in nodes}, frames=frames,
                axis_fit=dict(candidate_status="inconsistent", reason_codes=["synthetic_strict_model_rejected"]),
                provenance={"source":"synthetic_not_hardware"})


def samples(candidate, angle, rate=0., *, stamp=10., raw_overrides=None):
    output = []
    na = np.array(candidate["frames"]["functional_neutral_prox_acc_g"])
    for node, role, g, a in zip(candidate["node_ids"], ("prox", "dist"),
            (np.zeros(3), np.array([-rate,0.,0.])), (na, rx(angle)@na)):
        frame = np.array(candidate["frames"][role+"_sensor_to_functional"])
        gyro = frame.T@g + candidate["bias_dps"][str(node)]
        acc = frame.T@a
        values = np.rint(np.r_[gyro/gyro_scale(candidate["config"],node), acc/candidate["config"]["accel_g_per_lsb"]]).astype(int)
        if raw_overrides and node in raw_overrides:
            values = np.array(raw_overrides[node], dtype=int)
        output.append(Sample(node, tuple(values), stamp, 1000+stamp))
    return sorted(output, key=lambda s:s.node_id)


def pair(processor, candidate, angle, rate=0., **kwargs):
    result = None
    for row in samples(candidate, angle, rate, **kwargs):
        result = processor.process(row)
    assert result is not None
    return result


def test_validation_preserves_rejected_evidence_and_returns_independent_copy():
    source = profile()
    copy = validate_candidate(source)
    assert copy == source
    copy["axis_fit"]["reason_codes"].append("changed")
    assert source["axis_fit"]["reason_codes"] == ["synthetic_strict_model_rejected"]
    candidate = CandidateAngleProcessor(source)
    source["bias_dps"]["5"][0] = 8
    assert candidate.profile["bias_dps"]["5"][0] == .105


@pytest.mark.parametrize("edit", [
    lambda p:p.update(kind="formal_calibration"), lambda p:p.update(schema_version=True),
    lambda p:p.update(scope="certified"), lambda p:p.update(certified_calibration=True),
    lambda p:p.update(anatomical_accuracy_verified=True), lambda p:p.update(node_ids=[5,7]),
    lambda p:p.update(node_ids=[5,5]), lambda p:p.update(node_ids=[True,6]),
    lambda p:p.update(bias_dps={5:[0,0,0],6:[0,0,0]}),
    lambda p:p["bias_dps"].update({"5":[11,0,0]}),
    lambda p:p["bias_dps"].update({"5":[0,0,float("nan")]}),
    lambda p:p["bias_dps"].update({"5":[True,0,0]}),
    lambda p:p["frames"].update(status="inconsistent"),
    lambda p:p["frames"].update(prox_sensor_to_functional=[[1,0,0],[0,1,0],[0,0,-1]]),
    lambda p:p["frames"].update(prox_sensor_to_functional=[[2,0,0],[0,1,0],[0,0,1]]),
    lambda p:p["frames"].update(prox_sensor_to_functional=[[True,0,0],[0,1,0],[0,0,1]]),
    lambda p:p["frames"].update(functional_neutral_prox_acc_g=[0,0,0]),
    lambda p:p["frames"].update(functional_neutral_prox_acc_g=[0,0,True]),
    lambda p:p["frames"].update(functional_neutral_dist_acc_g=[1,0,0]),
    lambda p:p.update(axis_fit=None),
    lambda p:p["config"].update(gyro_dps_per_lsb_by_node={5:.0175})])
def test_invalid_or_certified_profile_is_not_silently_installed(edit):
    source = profile()
    edit(source)
    with pytest.raises(ValueError):
        validate_candidate(source)


@pytest.mark.parametrize("change", ["gyro", "accel", "rate", "mapping"])
def test_runtime_mismatch_rejected_before_consuming_samples(change):
    source = profile()
    config = deepcopy(source["config"])
    if change == "gyro":config["gyro_dps_per_lsb_by_node"]["5"] = .00875
    elif change == "accel":config["accel_g_per_lsb"] *= 2
    elif change == "rate":config["sample_rate_hz"] = 100
    else:config["fingers"]["thumb"]["proximal"],config["fingers"]["thumb"]["distal"] = 6,5
    with pytest.raises(ValueError, match="runtime"):
        CandidateAngleProcessor(source, config)


def test_effectively_equal_unit_representations_and_unrelated_geometry_are_allowed():
    source = profile()
    config = deepcopy(source["config"])
    config["gyro_dps_per_lsb"] = .0175
    config["gyro_dps_per_lsb_by_node"] = {}
    config["fingers"]["index"]["lengths_mm"][0] += 1
    CandidateAngleProcessor(source, config)


def test_first_pair_initializes_bent_relative_to_saved_neutral_not_zero():
    source = profile(neutral_angle=20.)
    processor = CandidateAngleProcessor(source)
    assert processor.snapshot()["angle_deg"] is None
    result = pair(processor,source,55.,rate=100.)
    assert result["angle_deg"] == pytest.approx(35.,abs=.01)
    assert result["pair_count"] == 1 and result["samples"] == 0
    assert result["current_angle_observed"] and not result["history_loss"]
    assert result["native_axis_fit_status"] == "inconsistent"
    assert not result["certified_calibration"] and not result["anatomical_accuracy_verified"]
    json.dumps(result,allow_nan=False)


def test_forward_reverse_motion_and_mixed_node_scales_follow_synthetic_truth():
    source = profile(overrides={"5":.0175})
    processor = CandidateAngleProcessor(source)
    pair(processor,source,0.)
    worst = 0.
    old = 0.
    for i in range(1,721):
        angle = 30.*(1.-math.cos(math.pi*i/120))
        result = pair(processor,source,angle,(angle-old)*120,stamp=10+i/120)
        worst = max(worst,abs(result["angle_deg"]-angle))
        old = angle
    assert worst < .1
    assert abs(result["angle_deg"]) < .05
    assert not result["history_loss"]


def test_reversed_role_ids_use_wire_order_but_keep_proximal_distal_roles():
    source = profile(nodes=(6,5))
    result = pair(CandidateAngleProcessor(source),source,42.)
    assert result["node_ids"] == [6,5]
    assert result["angle_deg"] == pytest.approx(42.,abs=.01)


def test_equal_usb_timestamps_are_multiple_updates_at_nominal_period():
    source = profile(); processor = CandidateAngleProcessor(source)
    pair(processor,source,0.)
    for i in range(1,61):
        result = pair(processor,source,i*.25,30.,stamp=10.)
    assert result["pair_count"] == 61 and result["samples"] == 60
    assert result["angle_deg"] == pytest.approx(15.,abs=.02)
    assert not result["history_loss"]


def test_snapshot_polling_does_not_integrate_or_repeat_gap_updates():
    source = profile();processor = CandidateAngleProcessor(source)
    initial = pair(processor,source,25.)
    for _ in range(10):assert processor.snapshot(now=10.01)["angle_deg"] == initial["angle_deg"]
    first = processor.snapshot(now=11.)
    assert first["angle_deg"] is None and not first["paired_fresh"] and first["history_loss"]
    for _ in range(10):assert processor.snapshot(now=11.)["samples"] == first["samples"]


def test_gap_does_not_integrate_elapsed_time_or_reset_neutral():
    source = profile();processor = CandidateAngleProcessor(source)
    pair(processor,source,35.)
    result = pair(processor,source,35.,120.,stamp=12.)
    assert result["history_loss"]
    assert result["angle_deg"] == pytest.approx(36.,abs=.1)  # One sample, not two seconds.
    for i in range(1,241):result=pair(processor,source,45.,stamp=12+i/120)
    assert result["angle_deg"] == pytest.approx(45.,abs=.1)
    assert result["current_angle_observed"] and result["history_loss"]


def test_backward_timestamp_is_rejected_not_rezeroed_and_recovery_keeps_loss():
    source=profile();processor=CandidateAngleProcessor(source)
    pair(processor,source,35.)
    for row in samples(source,80.,stamp=9.):assert processor.process(row) is None
    result=processor.snapshot()
    assert result["angle_deg"] is None and result["history_loss"]
    for i in range(1,61):result=pair(processor,source,35.,stamp=10+i/120)
    assert result["angle_deg"] == pytest.approx(35.,abs=.01)
    assert result["history_loss"]


@pytest.mark.parametrize("missing", ["proximal", "distal", "timestamp", "wall_timestamp"])
def test_incomplete_pair_never_mixes_stale_mate(missing):
    source=profile();processor=CandidateAngleProcessor(source)
    pair(processor,source,25.)
    low,high=samples(source,65.,stamp=10.01)
    if missing=="proximal":processor.process(high)
    elif missing=="distal":
        processor.process(low)
        processor.process(samples(source,30.,stamp=10.02)[0])
    else:
        processor.process(low)
        altered=Sample(high.node_id,high.values,high.host_monotonic_s+(.001 if missing=="timestamp" else 0),
                       high.host_wall_s+(.001 if missing=="wall_timestamp" else 0))
        assert processor.process(altered) is None
    assert processor.pair_count==1
    for row in samples(source,30.,stamp=10.03):result=processor.process(row)
    assert result["pair_count"]==2 and result["history_loss"]


def test_missing_intermediate_wire_node_cannot_form_nonadjacent_pair():
    source=profile(nodes=(1,3),overrides={})
    processor=CandidateAngleProcessor(source)
    low,high=samples(source,30.)
    processor.process(low)
    assert processor.process(high) is None and processor.pair_count==0
    low,high=samples(source,30.,stamp=10.01)
    processor.process(low)
    processor.process(Sample(2,(0,0,0,0,0,8197),10.01,1010.01))
    result=processor.process(high)
    assert result["pair_count"]==1 and result["history_loss"]
    assert result["angle_deg"]==pytest.approx(30.,abs=.01)


def test_initial_saturation_waits_and_preserves_history_after_later_initialization():
    source=profile();processor=CandidateAngleProcessor(source)
    result=pair(processor,source,35.,raw_overrides={6:[32767,0,0,0,0,8197]})
    assert result["angle_deg"] is None and result["paired_fresh"]
    assert result["history_loss"] and "initial_gyro_saturated" in result["reasons"]
    result=pair(processor,source,45.,stamp=10.01)
    assert result["angle_deg"]==pytest.approx(45.,abs=.01)
    assert result["history_loss"]


def test_saturated_input_is_integrated_once_and_labeled_unobserved():
    source=profile();processor=CandidateAngleProcessor(source)
    pair(processor,source,35.)
    result=pair(processor,source,35.,stamp=10.01,raw_overrides={6:[32767,0,0,0,0,8197]})
    assert result["samples"]==1 and result["history_loss"]
    assert not result["current_angle_observed"] and not result["valid"]
    assert "saturated_input" in result["reasons"]
    assert result["angle_deg"] is not None


def test_unobservable_current_gravity_waits_without_zeroing_other_processors():
    source=profile();processor=CandidateAngleProcessor(source)
    result=pair(processor,source,35.,raw_overrides={5:[0,0,0,0,0,0],6:[0,0,0,0,0,0]})
    assert result["angle_deg"] is None and result["paired_fresh"]
    assert "waiting_for_observable_current_gravity" in result["reasons"]
    result=pair(processor,source,55.,stamp=10.01)
    assert result["angle_deg"]==pytest.approx(55.,abs=.01)


def test_unrelated_node_and_snapshot_mutation_do_not_change_candidate():
    source=profile();processor=CandidateAngleProcessor(source)
    result=pair(processor,source,35.)
    result["reasons"].append("caller_change")
    processor.process(Sample(12,(0,0,0,0,0,8197),500.,1500.))
    snapshot=processor.snapshot()
    assert snapshot["paired_fresh"] and snapshot["pair_count"]==1
    assert "caller_change" not in snapshot["reasons"]


def test_current_axis_mismatch_is_reported_as_geometry_not_connection_failure():
    source = profile()
    processor = CandidateAngleProcessor(source)
    result = pair(processor, source, 0., raw_overrides={
        5: [0, 0, 0, 1640, 0, 8197], 6: [0, 0, 0, -2460, 0, 8197]})
    assert result["paired_fresh"] and result["angle_deg"] is None
    assert "axis_gravity_mismatch" in result["reasons"]
    assert "waiting_for_observable_current_gravity" in result["reasons"]
    recovered = pair(processor, source, 42., stamp=10.01)
    assert recovered["angle_deg"] == pytest.approx(42., abs=.01)
    assert "axis_gravity_mismatch" not in recovered["reasons"]
