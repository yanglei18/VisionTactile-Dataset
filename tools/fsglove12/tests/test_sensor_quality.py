"""Passive raw rail diagnostics, without serial devices or hardware writes."""
from copy import deepcopy
from itertools import groupby
import json
import math

import numpy as np
import pytest
from vqf import VQF

from vt_fsglove12.live_stream import OnlineFusion, RawRecorder, RawSample
from vt_fsglove12.replay import iter_orientation_frames, replay_raw
from vt_fsglove12.sensor_quality import (
    GYRO_NEAR_RAIL_THRESHOLD_LSB, GYRO_NEAR_RAIL_WINDOW_S,
    MAX_RECENT_GROUPS_PER_NODE, GyroNearRailMonitor, quality_warnings,
)


def raw(node=1, stamp=1., gyro=(0, 0, 0), accel=(0, 0, 8197)):
    return RawSample(node, (*gyro, *accel), stamp, 1000. + stamp)


def test_threshold_both_signs_counts_samples_and_axes_separately():
    monitor = GyroNearRailMonitor()
    monitor.observe(1, (32759, -32759, 0), 1.)
    monitor.observe(1, (32760, -32768, 32767), 1.1)
    monitor.observe(1, (0, 0, -32760), 1.2)
    monitor.observe(2, (0, 10, 0), 1.2)
    quality = monitor.snapshot(1.2)
    report = quality["gyro_near_rail"]
    node = report["nodes"]["1"]
    assert report["threshold_lsb"] == GYRO_NEAR_RAIL_THRESHOLD_LSB == 32760
    assert report["recent_window_s"] == GYRO_NEAR_RAIL_WINDOW_S == 2.
    assert report["active_nodes"] == ["1"]
    assert report["interpretation"] == "encoding_boundary_event_not_physical_saturation"
    assert report["physical_saturation"] == "unknown"
    assert report["observation_domain"] == "decoded_signed_int16_gyro_fields"
    assert not report["physical_range_verified"]
    assert not report["event_alone_proves_continuous_plateau"]
    assert node["total_samples"] == 3 and node["near_rail_samples"] == 2
    assert node["recent_near_rail_samples"] == 2
    assert node["total_by_axis"] == node["recent_by_axis"] == dict(x=1, y=1, z=2)
    assert node["axes"] == ["x", "y", "z"]
    assert node["last_host_monotonic_s"] == 1.2
    assert node["recent_counts_complete"] and not node["recent_overflow_samples_total"]
    assert quality_warnings(quality) == ["node_1_gyro_near_rail"]
    assert json.loads(json.dumps(quality, allow_nan=False)) == quality


@pytest.mark.parametrize("gyro", [(0, 0, 0), (32764, 0, 0), (-32768, 0, 0)])
def test_boundary_event_never_becomes_physical_saturation_verdict(gyro):
    monitor = GyroNearRailMonitor()
    monitor.observe(11, gyro, 1.)
    for stamp in (1., 4.):
        report = monitor.snapshot(stamp)["gyro_near_rail"]
        assert report["physical_saturation"] == "unknown"
        assert not report["physical_range_verified"]
        assert not report["raw_values_modified"] and not report["filter_state_modified"]


def test_numpy_int16_minimum_does_not_overflow_abs():
    monitor = GyroNearRailMonitor()
    monitor.observe(np.int64(6), np.array([-32768, 32759, -32760], dtype=np.int16), 1.)
    assert monitor.snapshot(1.)["gyro_near_rail"]["nodes"]["6"]["axes"] == ["x", "z"]


def test_last_boundary_event_is_none_before_any_boundary_sample():
    monitor = GyroNearRailMonitor()
    monitor.observe(1, (32759, -32759, 20), 1.)
    monitor.observe(1, (1, 2, 3), 1.1)
    node = monitor.snapshot(1.1)["gyro_near_rail"]["nodes"]["1"]
    assert node["total_samples"] == 2
    assert node["last_boundary_event"] is None


def test_last_boundary_event_preserves_exact_trigger_after_small_values():
    monitor = GyroNearRailMonitor()
    monitor.observe(6, (10, 20, 30), 1.)
    trigger = np.array([-32768, 42, 32760], dtype=np.int16)
    monitor.observe(6, trigger, 1.1)
    trigger[:] = 0
    monitor.observe(6, (1, 2, 3), 1.2)
    node = monitor.snapshot(1.3)["gyro_near_rail"]["nodes"]["6"]
    assert node["total_samples"] == 3
    event = node["last_boundary_event"]
    assert event == dict(sample_index=2, host_monotonic_s=1.1,
        gyro_lsb=[-32768, 42, 32760], axes=["x", "z"],
        age_s=pytest.approx(.2), in_recent_window=True)
    assert all(type(value) is int for value in event["gyro_lsb"])
    assert json.loads(json.dumps(event, allow_nan=False)) == event


def test_last_boundary_event_age_expires_without_losing_trigger_evidence():
    monitor = GyroNearRailMonitor()
    monitor.observe(1, (32760, -12, 23), 1.)
    at_limit = monitor.snapshot(3.)["gyro_near_rail"]["nodes"]["1"]["last_boundary_event"]
    assert at_limit["age_s"] == 2. and at_limit["in_recent_window"]
    monitor.observe(1, (1, 2, 3), 3.1)
    node = monitor.snapshot(3.1)["gyro_near_rail"]["nodes"]["1"]
    assert not node["recent_near_rail_samples"]
    event = node["last_boundary_event"]
    assert event == dict(sample_index=1, host_monotonic_s=1.,
        gyro_lsb=[32760, -12, 23], axes=["x"],
        age_s=pytest.approx(2.1), in_recent_window=False)
    earlier = monitor.snapshot(.5)["gyro_near_rail"]["nodes"]["1"]["last_boundary_event"]
    assert earlier["age_s"] is None and not earlier["in_recent_window"]
    assert monitor.snapshot(3.1)["gyro_near_rail"]["nodes"]["1"]["last_boundary_event"] == event


def test_same_timestamp_last_boundary_event_uses_node_sample_index_not_group_count():
    monitor = GyroNearRailMonitor()
    monitor.observe(1, (1, 2, 3), 1.)
    monitor.observe(1, (32760, 10, 20), 1.)
    monitor.observe(2, (0, -32768, 0), 1.)
    monitor.observe(1, (4, 5, 6), 1.)
    monitor.observe(1, (7, 8, -32768), 1.)
    monitor.observe(1, (9, 10, 11), 1.)
    nodes = monitor.snapshot(1.)["gyro_near_rail"]["nodes"]
    assert nodes["1"]["total_samples"] == 5
    assert nodes["1"]["recent_near_rail_samples"] == 2
    assert nodes["1"]["last_boundary_event"] == dict(sample_index=4, host_monotonic_s=1.,
        gyro_lsb=[7, 8, -32768], axes=["z"], age_s=0., in_recent_window=True)
    assert nodes["2"]["last_boundary_event"]["sample_index"] == 1


def test_recent_warning_expires_during_silence_but_history_is_retained():
    monitor = GyroNearRailMonitor()
    monitor.observe(1, (32764, 0, 0), 1.)
    assert quality_warnings(monitor.snapshot(3.)) == ["node_1_gyro_near_rail"]
    future = monitor.snapshot(3.000001)
    assert quality_warnings(future) == []
    node = future["gyro_near_rail"]["nodes"]["1"]
    assert node["total_samples"] == node["near_rail_samples"] == 1
    assert node["recent_near_rail_samples"] == 0 and node["axes"] == []
    assert node["total_by_axis"]["x"] == 1 and node["recent_by_axis"]["x"] == 0
    # Querying a snapshot did not destroy the event or change the monitor.
    assert quality_warnings(monitor.snapshot(1.)) == ["node_1_gyro_near_rail"]
    assert quality_warnings(monitor.snapshot(.5)) == []


def test_snapshot_is_nondestructive_and_returned_dicts_are_detached():
    monitor = GyroNearRailMonitor()
    monitor.observe("5", (32764, 0, 0), 4.)
    expected = monitor.snapshot(4.)
    before = deepcopy(monitor.__dict__)
    first, second = monitor.snapshot(4.), monitor.snapshot(4.)
    assert first == second == expected and monitor.__dict__ == before
    first["gyro_near_rail"]["nodes"]["5"]["total_by_axis"]["x"] = 999
    first["gyro_near_rail"]["nodes"]["5"]["last_boundary_event"]["gyro_lsb"][0] = 999
    first["gyro_near_rail"]["nodes"]["5"]["last_boundary_event"]["axes"].clear()
    first["gyro_near_rail"]["nodes"]["5"]["last_boundary_event"]["sample_index"] = 999
    first["gyro_near_rail"]["active_nodes"].clear()
    assert monitor.snapshot(4.) == expected


def test_equal_timestamp_burst_is_not_deduplicated_or_unbounded():
    monitor = GyroNearRailMonitor()
    for _ in range(20000):
        monitor.observe(1, (32764, -32768, 0), 1.)
    node = monitor.snapshot(1.)["gyro_near_rail"]["nodes"]["1"]
    assert node["total_samples"] == node["near_rail_samples"] == 20000
    assert node["recent_near_rail_samples"] == 20000
    assert node["recent_by_axis"] == dict(x=20000, y=20000, z=0)
    assert len(monitor._nodes["1"]["recent"]) == 1
    assert node["recent_counts_complete"]


def test_unique_burst_is_hard_bounded_and_reports_lower_bound_until_expiry():
    monitor = GyroNearRailMonitor()
    count = MAX_RECENT_GROUPS_PER_NODE + 100
    for index in range(count):
        monitor.observe(1, (32764, 0, 0), 1. + index * .0001)
    assert len(monitor._nodes["1"]["recent"]) == MAX_RECENT_GROUPS_PER_NODE
    node = monitor.snapshot(1.5)["gyro_near_rail"]["nodes"]["1"]
    assert node["near_rail_samples"] == count
    assert node["recent_near_rail_samples"] == MAX_RECENT_GROUPS_PER_NODE
    assert not node["recent_counts_complete"] and node["recent_overflow_samples_total"] == 100
    assert monitor.snapshot(4.)["gyro_near_rail"]["nodes"]["1"]["recent_counts_complete"]
    monitor.observe(1, (0, 0, 0), 4.)
    assert len(monitor._nodes["1"]["recent"]) == 0
    assert monitor.snapshot(4.)["gyro_near_rail"]["nodes"]["1"]["near_rail_samples"] == count


def test_input_clock_rollback_clears_recent_time_domain_but_not_history():
    monitor = GyroNearRailMonitor()
    monitor.observe(1, (32764, 0, 0), 10.)
    monitor.observe(2, (0, -32768, 0), 10.)
    monitor.observe(2, (0, 0, 0), 1.)
    report = monitor.snapshot(1.)["gyro_near_rail"]
    assert report["clock_resets"] == 1 and not report["active_nodes"]
    assert report["nodes"]["1"]["near_rail_samples"] == report["nodes"]["2"]["near_rail_samples"] == 1
    assert report["nodes"]["1"]["last_boundary_event"] is None
    assert report["nodes"]["2"]["last_boundary_event"] is None
    assert report["nodes"]["1"]["last_host_monotonic_s"] == 10.
    assert report["nodes"]["2"]["total_samples"] == 2
    monitor.observe(1, (0, 0, 32764), 1.1)
    report = monitor.snapshot(1.1)["gyro_near_rail"]
    assert report["active_nodes"] == ["1"]
    assert report["nodes"]["1"]["total_by_axis"] == dict(x=1, y=0, z=1)
    assert report["nodes"]["1"]["recent_by_axis"] == dict(x=0, y=0, z=1)
    assert report["nodes"]["1"]["last_boundary_event"] == dict(sample_index=2, host_monotonic_s=1.1,
        gyro_lsb=[0, 0, 32764], axes=["z"], age_s=0., in_recent_window=True)


@pytest.mark.parametrize("node,values,time", [(0, (0, 0, 0), 1.), (True, (0, 0, 0), 1.),
    ("05", (0, 0, 0), 1.), (1, (32768, 0, 0), 1.), (1, (0., 0, 0), 1.),
    (1, (True, 0, 0), 1.), (1, (0, 0), 1.), (1, (0, 0, 0), float("nan")),
    (1, (0, 0, 0), -1.)])
def test_invalid_monitor_input_rejected_without_counts(node, values, time):
    monitor = GyroNearRailMonitor()
    with pytest.raises(ValueError):
        monitor.observe(node, values, time)
    assert monitor.snapshot(1.)["gyro_near_rail"]["nodes"] == {}


def test_live_rail_warning_does_not_modify_raw_vqf_epoch_or_filter_lifetime():
    fusion = OnlineFusion({"gyro_dps_per_lsb_by_node": {"5": .0175}})
    rows = [raw(5, 1. + i / 120., (32764, -32768, i)) for i in range(20)]
    before = deepcopy(rows)
    reference = VQF(1. / 120.)
    for sample in rows:
        reference.update(np.asarray(sample.values[:3]) * .0175 * math.pi / 180.,
                         np.asarray(sample.values[3:]) * .000122 * 9.80665)
    fusion.update(rows)
    filt = fusion.states["5"]["filter"]
    epochs, counts = dict(fusion.epochs), dict(fusion.counts)
    first, repeated = fusion.snapshot(1.2), fusion.snapshot(1.2)
    assert first["sensor_quality"] == repeated["sensor_quality"]
    assert "node_5_gyro_near_rail" in first["warnings"] and "node_5_gyro_near_rail" in repeated["warnings"]
    np.testing.assert_array_equal(first["nodes"]["5"]["quat_xyzw"], reference.getQuat6D()[[1, 2, 3, 0]])
    later = fusion.snapshot(3.2)
    assert "node_5_gyro_near_rail" not in later["warnings"]
    assert later["sensor_quality"]["gyro_near_rail"]["nodes"]["5"]["near_rail_samples"] == 20
    assert rows == before and fusion.states["5"]["filter"] is filt
    assert fusion.epochs == epochs and fusion.counts == counts


def test_accelerometer_rail_does_not_become_a_gyro_warning():
    fusion = OnlineFusion({})
    fusion.update([raw(1, 1., (0, 0, 0), (32767, -32768, 32764))])
    report = fusion.snapshot(1.)
    assert not report["sensor_quality"]["gyro_near_rail"]["active_nodes"]
    assert not any(w.endswith("gyro_near_rail") for w in report["warnings"])


@pytest.mark.parametrize("scale", [.00875, .0175])
def test_live_and_offline_quality_parity_with_historical_scale_and_clock_reset(tmp_path, scale):
    config = dict(gyro_dps_per_lsb_by_node={"5": scale}, stale_after_s=.1, filter_reset_gap_s=1.)
    rows = [raw(1, 1., (32764, 0, 0)), raw(5, 1., (0, -32768, 0)),
            raw(5, 1., (0, -32768, 32767)), raw(5, 1.1, (20, 0, 0)),
            raw(1, 3.2), raw(5, 3.2), raw(5, .5, (32760, 0, 0)), raw(5, 2.6)]
    path, output = tmp_path / "raw.csv", tmp_path / "orientation.jsonl"
    recorder = RawRecorder(path).open()
    recorder.write(rows)
    recorder.close()
    original_bytes = path.read_bytes()
    summary = replay_raw(path, config, output)
    offline = list(iter_orientation_frames(output))
    fusion = OnlineFusion(config)
    live = []
    for stamp, batch in groupby(rows, key=lambda sample: sample.host_monotonic_s):
        fusion.update(batch)
        live.append(fusion.snapshot(stamp))
    assert len(live) == len(offline)
    for actual, expected in zip(live, offline):
        assert actual["sensor_quality"] == expected["sensor_quality"]
        assert actual["warnings"] == expected["warnings"]
        for key in actual["nodes"]:
            np.testing.assert_array_equal(actual["nodes"][key]["quat_xyzw"], expected["nodes"][key]["quat_xyzw"])
            assert actual["nodes"][key]["epoch"] == expected["nodes"][key]["epoch"]
    assert summary["sensor_quality"] == live[-1]["sensor_quality"]
    assert summary["used_gyro_dps_per_lsb_by_node"]["5"] == scale
    assert summary["sensor_quality"]["gyro_near_rail"]["nodes"]["5"]["near_rail_samples"] == 3
    assert not summary["sensor_quality"]["gyro_near_rail"]["active_nodes"]
    assert path.read_bytes() == original_bytes


def test_offline_summary_covers_samples_after_last_emitted_frame_and_empty_file(tmp_path):
    path, output = tmp_path / "raw.csv", tmp_path / "orientation.jsonl"
    recorder = RawRecorder(path).open()
    recorder.write([raw(1, 1.), raw(1, 1.001, (32764, 0, 0))])
    recorder.close()
    summary = replay_raw(path, {}, output)
    assert summary["frames"] == 1
    assert not next(iter_orientation_frames(output))["sensor_quality"]["gyro_near_rail"]["active_nodes"]
    assert summary["sensor_quality"]["gyro_near_rail"]["active_nodes"] == ["1"]
    assert "node_1_gyro_near_rail" in summary["warnings"]
    empty = tmp_path / "empty.csv"
    recorder = RawRecorder(empty).open()
    recorder.close()
    summary = replay_raw(empty, {}, tmp_path / "empty.jsonl")
    assert summary["sensor_quality"]["gyro_near_rail"]["nodes"] == {}


def test_sustained_normal_rate_window_is_exact_and_memory_stays_bounded():
    monitor = GyroNearRailMonitor()
    for index in range(12000):
        monitor.observe(12, (32764, 0, 0), index / 120.)
    report = monitor.snapshot(11999 / 120.)["gyro_near_rail"]["nodes"]["12"]
    assert report["near_rail_samples"] == 12000
    assert 240 <= report["recent_near_rail_samples"] <= 241
    assert len(monitor._nodes["12"]["recent"]) <= 241
    assert report["recent_counts_complete"] and not report["recent_overflow_samples_total"]
