"""Explicit fusion reset uses synthetic samples only, never a serial device."""
from copy import deepcopy

import numpy as np
import pytest

from vt_fsglove12.live_stream import OnlineFusion, RawRecorder, RawSample, iter_csv_samples


def sample(node=1, stamp=1., values=(0, 0, 0, 0, 0, 8197)):
    return RawSample(node, values, stamp, 1000. + stamp)


def test_reset_clears_all_orientations_and_rebuilds_vqf_lazily():
    fusion = OnlineFusion({"input_filter": {"enabled": True}})
    created = []
    factory = fusion._factory

    def tracked_factory(period):
        result = factory(period)
        created.append(result)
        return result

    fusion._factory = tracked_factory
    rotating = (0, 0, 5000, 0, 0, 8197)
    for i in range(120):
        fusion.update([sample(node, 1. + i / 120., rotating) for node in (1, 9)])
    before = fusion.snapshot(2.)
    assert len(created) == 2
    old_filters = {key: state["filter"] for key, state in fusion.states.items()}
    old_counts = dict(fusion.counts)
    next_frame = fusion.frame_index
    result = fusion.reset_for_calibration()
    assert result == {"reset_nodes": ["1", "9"], "epochs": {"1": 1, "9": 1}}
    assert fusion.epochs["12"] == 0
    assert fusion.states == {}
    assert fusion.counts == old_counts
    assert fusion.frame_index == next_frame
    assert len(created) == 2  # No filter or synthetic observation on reset.

    empty = fusion.snapshot(2.)
    assert empty["nodes"] == {}
    assert empty["frame_index"] == next_frame
    assert empty["input_filter"]["nodes"] == {}
    assert empty["input_filter"]["active_nodes"] == []
    assert "calibration_reset_filters_reinitialized" in empty["warnings"]
    assert "calibration_reset_filters_reinitialized" not in fusion.snapshot(2.)["warnings"]
    # A previously returned frame remains a detached, truthful old observation.
    assert before["nodes"]["1"]["epoch"] == 0

    current = sample(1, 2.01)
    fusion.update([current])
    assert len(created) == 3
    assert fusion.states["1"]["filter"] is not old_filters["1"]
    after = fusion.snapshot(2.01)
    assert set(after["nodes"]) == {"1"}  # Node 9 must not reappear from cache.
    node = after["nodes"]["1"]
    assert node["epoch"] == 1 and node["sample_index"] == 121
    assert node["input_filter"]["raw_samples"] == 1
    assert node["input_filter"]["rejected_samples"] == 0
    fresh = OnlineFusion({"input_filter": {"enabled": True}})
    fresh.update([current])
    expected = fresh.snapshot(2.01)["nodes"]["1"]
    assert node["quat_xyzw"] == pytest.approx(expected["quat_xyzw"], abs=1e-14)
    assert node["fusion_state"] == expected["fusion_state"]
    assert not np.allclose(node["quat_xyzw"], before["nodes"]["1"]["quat_xyzw"])


def test_reset_drops_guard_accepted_values_candidates_and_invalid_runs():
    fusion = OnlineFusion({"input_filter": {"enabled": True}})
    fusion.update([sample(1, 1., (1000, 0, 0, 0, 0, 8197))])
    boundary = (32764, 0, 0, 0, 0, 8197)
    fusion.update([sample(1, 1.01, boundary)])
    old = fusion.snapshot(1.01)["nodes"]["1"]
    assert old["input_filter"]["gyro_action"] == "held"
    assert old["fusion_gyro_rad_s"][0] != 0.
    fusion.reset_for_calibration()
    fusion.update([sample(1, 1.02, boundary)])
    new = fusion.snapshot(1.02)["nodes"]["1"]
    assert new["input_filter"]["gyro_action"] == "zero_motion"
    assert new["fusion_gyro_rad_s"] == [0., 0., 0.]
    assert new["input_filter"]["gyro_run"] == 1
    assert new["input_filter"]["raw_samples"] == 1
    assert new["input_filter"]["rejected_samples"] == 1
    assert new["orientation_valid"] is False  # No accepted pre-reset baseline.
    assert new["sample_index"] == 3
    assert fusion.input_guard_rejections == 2  # Session history is preserved.

    fusion.update([sample(1, 1.03)])
    jump = (20000, 0, 0, 0, 0, 8197)
    fusion.update([sample(1, 1.04, jump)])
    assert fusion.snapshot(1.04)["nodes"]["1"]["input_filter"]["reasons"] == ["gyro_jump"]
    fusion.reset_for_calibration()
    fusion.update([sample(1, 1.05, jump)])
    first = fusion.snapshot(1.05)["nodes"]["1"]
    assert first["input_filter"]["gyro_action"] == "pass"
    assert first["input_filter"]["gyro_run"] == 0
    assert first["input_filter"]["raw_samples"] == 1
    assert first["sample_index"] == 6 and first["epoch"] == 2


def test_passive_evidence_configuration_and_clock_are_preserved():
    config = {"input_filter": {"enabled": True}, "stale_after_s": .05,
              "filter_reset_gap_s": .1, "gyro_dps_per_lsb_by_node": {"5": .0175}}
    fusion = OnlineFusion(config)
    fusion.update([sample(1, 1.), sample(1, 1.01, (32764, 0, 0, 0, 0, 8197)), sample(1, 1.2)])
    settings, scales, accel_scales = deepcopy((fusion.settings, fusion.scales, fusion.accel_scales))
    quality = fusion.sensor_quality.snapshot(1.2)
    observation = fusion.input_observation.snapshot()
    gaps = deepcopy(fusion.gap_diagnostics)
    last_input = fusion.last_input
    total_rejections = fusion.input_guard_rejections
    fusion.reset_for_calibration()
    assert fusion.sensor_quality.snapshot(1.2) == quality
    assert fusion.input_observation.snapshot() == observation
    assert fusion.gap_diagnostics == gaps
    assert fusion.last_input == last_input
    assert fusion.input_guard_rejections == total_rejections == 1
    assert (fusion.settings, fusion.scales, fusion.accel_scales) == (settings, scales, accel_scales)
    assert fusion.epochs["1"] == 2  # One genuine gap reset and one explicit reset.
    reset_frame = fusion.snapshot(1.2)
    assert "node_1_gap_filter_reinitialized" in reset_frame["warnings"]
    assert reset_frame["sensor_quality"]["gyro_near_rail"]["nodes"]["1"]["last_boundary_event"]["sample_index"] == 2
    # Reset did not mask a subsequent real host-clock rollback.
    fusion.update([sample(1, .1)])
    rolled_back = fusion.snapshot(.1)
    assert rolled_back["nodes"]["1"]["epoch"] == 3
    assert rolled_back["nodes"]["1"]["sample_index"] == 4
    assert "host_clock_reset_filters_reinitialized" in rolled_back["warnings"]
    assert fusion.input_observation.clock_resets == 1


def test_empty_and_repeated_resets_do_not_invent_samples_or_restart_indices():
    fusion = OnlineFusion({})
    assert fusion.reset_for_calibration() == {"reset_nodes": [], "epochs": {}}
    assert set(fusion.epochs.values()) == {0}
    fusion.update([sample(9, 1.)])
    first = fusion.snapshot(1.)
    fusion.reset_for_calibration()
    assert fusion.reset_for_calibration() == {"reset_nodes": ["9"], "epochs": {"9": 2}}
    assert fusion.counts["9"] == 1
    assert fusion.snapshot(1.)["nodes"] == {}
    fusion.update([sample(9, 1.01), sample(12, 1.01)])
    after = fusion.snapshot(1.01)
    assert after["frame_index"] > first["frame_index"]
    assert after["nodes"]["9"]["sample_index"] == 2
    assert after["nodes"]["9"]["epoch"] == 2
    assert after["nodes"]["12"]["sample_index"] == 1
    assert after["nodes"]["12"]["epoch"] == 0
    assert "input_filter" not in after


def test_same_open_raw_recorder_keeps_every_original_row_across_reset(tmp_path):
    fusion = OnlineFusion({"input_filter": {"enabled": True}})
    path = tmp_path / "raw.csv"
    recorder = RawRecorder(path).open()
    before = [sample(1, 1.), sample(9, 1.)]
    after = [sample(1, 1.01, (32764, 0, 0, 0, 0, 8197)), sample(9, 1.01)]
    try:
        recorder.write(before)
        fusion.update(before)
        stream = recorder._stream
        fusion.reset_for_calibration()
        assert recorder._stream is stream and not stream.closed
        recorder.write(after)
        fusion.update(after)
    finally:
        recorder.close()
    assert list(iter_csv_samples(path)) == before + after
    assert fusion.counts["1"] == fusion.counts["9"] == 2
