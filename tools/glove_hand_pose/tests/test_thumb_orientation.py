"""Actual VQF sensor-only previews; synthetic truth is not hardware acceptance."""
import json

import numpy as np
import pytest

from vt_glove_pose.config import load_config
from vt_glove_pose.records import Sample
from vt_glove_pose.thumb_orientation import ThumbOrientationProcessor


def config():
    value = load_config()
    value["gyro_dps_per_lsb_by_node"] = {"5": .0175, "6": .0175}
    return value


def pair(processor, tick=0, *, raw=0, acc=(0, 0, 8197), stamp=None):
    t = 10+tick/120 if stamp is None else stamp
    assert processor.process(Sample(5, (raw, 0, 0, *acc), t, 1000+t)) is None
    return processor.process(Sample(6, (raw, 0, 0, *acc), t, 1000+t))


def test_first_pair_is_visible_without_axis_or_reference_then_explicit_reference():
    processor = ThumbOrientationProcessor(config())
    snapshot = pair(processor)
    assert not snapshot["valid"] and not snapshot["reference_set"]
    assert snapshot["paired_fresh"]
    assert snapshot["nodes"]["5"]["sensor_quat_wxyz"] is not None
    assert snapshot["nodes"]["5"]["relative_quat_wxyz"] is None
    snapshot = processor.set_reference(source="first_motion_pair_not_neutral")
    assert snapshot["valid"] and snapshot["reference_set"]
    assert snapshot["reference_source"] == "first_motion_pair_not_neutral"
    assert snapshot["nodes"]["5"]["rotation_magnitude_deg"] == pytest.approx(0., abs=1e-5)
    assert snapshot["joint_angle_deg"] is None
    assert not snapshot["anatomical_pose_estimated"]
    assert not snapshot["heading_drift_corrected"]
    assert not processor.engine.neutral_ready and processor.engine.mounts == {}
    json.dumps(snapshot, allow_nan=False)


def test_reference_accepts_moving_pose_but_does_not_estimate_static_bias():
    processor = ThumbOrientationProcessor(config())
    pair(processor, raw=4000)
    assert processor.set_reference()["valid"]
    np.testing.assert_array_equal(processor.engine.bias[5], [0., 0., 0.])


def test_bias_and_node_units_are_applied_before_vqf():
    processor = ThumbOrientationProcessor(config(), bias_dps={5: [.1, -.2, .3], "6": [0., .2, 0.]})
    snapshot = pair(processor, raw=2000)
    assert snapshot["nodes"]["5"]["gyro_dps"][0] == pytest.approx(35.)
    np.testing.assert_allclose(np.rad2deg(processor.engine.states[5]["filter"].getBiasEstimate()[0]), [.1, -.2, .3])
    assert snapshot["nodes"]["6"]["gyro_dps_per_lsb"] == .0175


def test_no_reference_before_complete_pair():
    processor = ThumbOrientationProcessor(config())
    with pytest.raises(ValueError, match="paired"):
        processor.set_reference()
    processor.process(Sample(5, (0, 0, 0, 0, 0, 8197), 10., 1000.))
    with pytest.raises(ValueError, match="paired"):
        processor.set_reference()


def test_real_vqf_follows_analytic_single_sensor_rotation_without_joint_calibration():
    processor = ThumbOrientationProcessor(config())
    pair(processor)
    processor.set_reference()
    speed = np.deg2rad(30.)
    for tick in range(1, 241):
        theta = speed*tick/120
        accel = tuple(int(round(v/.000122)) for v in (0., np.sin(theta), np.cos(theta)))
        snapshot = pair(processor, tick, raw=int(round(30/.0175)), acc=accel)
    assert snapshot["valid"]
    for row in snapshot["nodes"].values():
        assert row["rotation_magnitude_deg"] == pytest.approx(60., abs=.6)
        assert row["gravity_direction_change_deg"] == pytest.approx(60., abs=.01)
        assert np.linalg.norm(row["relative_quat_wxyz"]) == pytest.approx(1.)
    assert snapshot["joint_angle_deg"] is None


def test_duplicate_host_times_still_advance_each_filter_once_per_sample():
    processor = ThumbOrientationProcessor(config())
    pair(processor)
    processor.set_reference()
    for _ in range(30):
        for node in (5, 6):
            result = processor.process(Sample(node, (0, 0, 4000, 0, 0, 8197), 10., 1010.))
    assert result["pair_count"] == 31
    assert processor.engine.states[5]["count"] == 31
    assert result["nodes"]["5"]["rotation_magnitude_deg"] == pytest.approx(17.5, abs=.03)


@pytest.mark.parametrize("problem", ["gap", "invalid_acc", "backward", "saturation", "unmatched"])
def test_invalid_input_or_missing_interval_discards_reference_without_auto_zero(problem):
    processor = ThumbOrientationProcessor(config())
    pair(processor)
    processor.set_reference()
    if problem == "gap":
        pair(processor, stamp=11.)
        next_stamp = 11.01
    elif problem == "invalid_acc":
        pair(processor, 1, acc=(0, 0, 0))
        next_stamp = 10.02
    elif problem == "backward":
        pair(processor, stamp=9.)
        next_stamp = 10.02
    elif problem == "saturation":
        pair(processor, 1, raw=32767)
        next_stamp = 10.02
    else:
        processor.process(Sample(6, (0, 0, 0, 0, 0, 8197), 10.01, 1000.))
        next_stamp = 10.02
    assert not processor.snapshot()["reference_set"]
    result = pair(processor, stamp=next_stamp)
    assert not result["valid"]
    assert result["nodes"]["5"]["relative_quat_wxyz"] is None
    assert processor.set_reference()["valid"]


def test_tick_staleness_clears_reference_even_when_data_later_resumes():
    processor = ThumbOrientationProcessor(config())
    pair(processor)
    processor.set_reference()
    stale = processor.snapshot(now=11.)
    assert not stale["valid"] and stale["reasons"] == ["stale_data"]
    assert not stale["nodes"]["5"]["fresh"]
    assert not stale["paired_fresh"]
    result = pair(processor, 1)
    assert not result["reference_set"]


def test_snapshots_are_copies_and_other_nodes_do_not_advance_pair():
    processor = ThumbOrientationProcessor(config())
    pair(processor)
    before = processor.set_reference()
    before["nodes"]["5"]["relative_quat_wxyz"][0] = 99.
    processor.process(Sample(1, (0, 0, 0, 0, 0, 8197), 100., 1000.))
    after = processor.snapshot()
    assert after["pair_count"] == 1 and after["valid"]
    assert after["nodes"]["5"]["relative_quat_wxyz"][0] == pytest.approx(1.)


@pytest.mark.parametrize("bias", [[0., 0., 0.], {True: [0, 0, 0]}, {"05": [0, 0, 0]},
                                   {5: [0, 0]}, {5: [0, 0, float("nan")]}, {5: [11, 0, 0]},
                                   {5: [0, 0, 0], "5": [0, 0, 0]}])
def test_invalid_bias_is_rejected(bias):
    with pytest.raises(ValueError, match="bias_dps"):
        ThumbOrientationProcessor(config(), bias_dps=bias)
