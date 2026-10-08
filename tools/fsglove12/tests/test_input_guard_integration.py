"""Before-VQF protection; replay parity and original-input preservation."""
import csv
from itertools import groupby
import json

import numpy as np
import pytest

from vt_fsglove12.config import load_config, validate_config
from vt_fsglove12.live_stream import CSV_FIELDS, OnlineFusion, RawSample
from vt_fsglove12.replay import replay_raw


def sample(stamp, values, node=1):
    return RawSample(node, tuple(values), stamp, stamp + 1000.)


def test_default_right_hand_enables_protection_but_legacy_missing_policy_is_unchanged():
    config = load_config()
    assert config["input_filter"]["enabled"] is True
    assert OnlineFusion({}).input_guard.settings["enabled"] is False
    assert validate_config(config)["input_filter"] == config["input_filter"]


def test_isolated_boundary_spike_stays_visible_but_does_not_enter_vqf():
    fusion = OnlineFusion({"input_filter": {"enabled": True}})
    neutral = (0, 0, 0, 0, 0, 8197)
    fusion.update([sample(1., neutral)])
    expected = fusion.snapshot(1.)["nodes"]["1"]["quat_xyzw"]
    bad = (0, 32764, 0, 0, 0, 8197)
    fusion.update([sample(1.01, bad)])
    frame = fusion.snapshot(1.01)
    node = frame["nodes"]["1"]
    assert node["raw_values"] == list(bad)
    assert node["gyro_rad_s"][1] > 4.
    assert node["fusion_gyro_rad_s"] == [0., 0., 0.]
    assert node["quat_xyzw"] == pytest.approx(expected, abs=1e-13)
    assert node["input_filter"]["gyro_rejected"] is True
    assert frame["input_filter"]["rejected_samples"] == 1
    assert frame["sensor_quality"]["gyro_near_rail"]["nodes"]["1"]["last_boundary_event"]["gyro_lsb"] == [0, 32764, 0]
    node["input_filter"]["reasons"].append("external_mutation")
    assert "external_mutation" not in fusion.snapshot(1.01)["nodes"]["1"]["input_filter"]["reasons"]


def test_continued_invalid_data_is_marked_not_silently_valid_and_recovers():
    fusion = OnlineFusion({"input_filter": {"enabled": True}, "sample_rate_hz": 120.})
    neutral = (0, 0, 0, 0, 0, 8197)
    fusion.update([sample(1., neutral)])
    for index in range(1, 35):
        # Alternating spikes cannot confirm a continuous reported rate.
        fusion.update([sample(1. + index / 120., ((-1) ** index * 32764, 0, 0, 0, 0, 8197))])
    bad = fusion.snapshot(1. + 34 / 120.)["nodes"]["1"]
    assert bad["orientation_valid"] is False
    assert bad["input_filter"]["gyro_action"] == "zero_motion"
    assert np.linalg.norm(bad["fusion_gyro_rad_s"]) < 1e-10
    fusion.update([sample(1.3, neutral)])
    recovered = fusion.snapshot(1.3)["nodes"]["1"]
    assert recovered["orientation_valid"] is True
    assert recovered["input_filter"]["gyro_action"] == "pass"
    assert recovered["epoch"] == 0


def test_live_offline_filtered_inputs_and_orientations_match_with_gaps(tmp_path):
    config = {"input_filter": {"enabled": True}, "stale_after_s": .1,
              "filter_reset_gap_s": 1., "output_rate_hz": 120.}
    rows = []
    for index in range(70):
        # Equal timestamps exercise USB batches without losing sample steps.
        stamp = 1. + (index // 2) * .02
        gyro = 32764 if index in (20, 21, 22, 23, 24) else 20000 if index == 40 else 5
        acc = 32764 if index == 50 else 8197
        rows.extend(sample(stamp, (0, gyro, 0, 0, 0, acc), node) for node in (1, 12))
    rows.extend([sample(4., (0, 0, 0, 0, 0, 8197)),
                 sample(.5, (0, 0, 0, 0, 0, 8197))])
    source = tmp_path / "raw.csv"
    with source.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(CSV_FIELDS)
        writer.writerows((row.host_monotonic_s, row.host_wall_s, row.node_id,
                          *row.values, row.status) for row in rows)
    original = source.read_bytes()
    result = tmp_path / "filtered.jsonl"
    report = replay_raw(source, config, result)
    assert source.read_bytes() == original
    expected = [json.loads(line) for line in result.read_text().splitlines()]
    fusion, actual = OnlineFusion(config), []
    for stamp, batch in groupby(rows, key=lambda row: row.host_monotonic_s):
        fusion.update(list(batch))
        actual.append(fusion.snapshot(stamp))
    assert len(actual) == len(expected)
    assert report["input_filter"]["rejected_samples"] == 8
    for left, right in zip(actual, expected):
        assert left["input_filter"] == right["input_filter"]
        assert set(left["nodes"]) == set(right["nodes"])
        for key in left["nodes"]:
            for field in ("raw_values", "sample_index", "epoch", "input_filter", "orientation_valid",
                          "fusion_gyro_rad_s", "fusion_accel_m_s2"):
                assert left["nodes"][key][field] == right["nodes"][key][field]
            assert left["nodes"][key]["quat_xyzw"] == pytest.approx(right["nodes"][key]["quat_xyzw"], abs=1e-13)


def test_accel_boundary_does_not_poison_tilt_or_modify_raw_gyro():
    fusion = OnlineFusion({"input_filter": {"enabled": True}})
    fusion.update([sample(1., (0, 0, 0, 0, 0, 8197))])
    fusion.update([sample(1.01, (0, 0, 100, 32764, 0, 8197))])
    node = fusion.snapshot(1.01)["nodes"]["1"]
    assert node["raw_values"] == [0, 0, 100, 32764, 0, 8197]
    assert node["fusion_accel_m_s2"] == [0., 0., 0.]
    assert node["fusion_gyro_rad_s"] == node["gyro_rad_s"]
    assert node["input_filter"]["accel_action"] == "skip_accel"


def test_smooth_encoded_boundary_yaw_preserves_reported_rotation_with_real_vqf():
    from scipy.spatial.transform import Rotation

    protected = OnlineFusion({"input_filter": {"enabled": True}})
    control = OnlineFusion({"input_filter": {"enabled": False}})
    counts = [0, 10000, 20000, 30000] + [32764] * 18 + [30000, 20000, 10000, 0]
    for index, gyro in enumerate(counts):
        row = sample(1. + index / 120., (0, 0, gyro, 0, 0, 8197))
        protected.update([row])
        control.update([row])
        actual = protected.snapshot(row.host_monotonic_s)["nodes"]["1"]
        expected = control.snapshot(row.host_monotonic_s)["nodes"]["1"]
        np.testing.assert_array_equal(actual["fusion_gyro_rad_s"], actual["gyro_rad_s"])
        np.testing.assert_allclose(actual["quat_xyzw"], expected["quat_xyzw"], atol=1e-13)
        assert actual["input_filter"]["gyro_rejected"] is False
    # Previously the plateau was replaced with zero yaw after two samples.
    # This verifies conservation of the REPORTED rate, not physical angle truth.
    assert np.rad2deg(Rotation.from_quat(actual["quat_xyzw"]).magnitude()) > 40.
    assert actual["input_filter"]["gyro_boundary_samples_by_axis"] == {"x": 0, "y": 0, "z": 18}
    assert actual["input_filter"]["gyro_zero_motion_samples_by_axis"] == {"x": 0, "y": 0, "z": 0}
