"""Host-gap policy: hide stale poses without destroying short-gap references."""
from itertools import groupby
import json
import math

import numpy as np
import pytest
from vqf import VQF

from vt_fsglove12.live_stream import OnlineFusion, RawRecorder, RawSample
from vt_fsglove12.replay import _configuration, iter_orientation_frames, replay_raw


def sample(node, stamp, values=(0, 0, 6000, 0, 0, 8197)):
    return RawSample(node, values, stamp, 1000. + stamp)


@pytest.mark.parametrize("gap", [.164, .300, 1.0])
def test_short_gap_preserves_filter_and_nominal_integration_but_never_shows_stale(gap):
    fusion = OnlineFusion({"stale_after_s": .15})
    fusion.update([sample(1, 1.)])
    original = fusion.states["1"]["filter"]
    assert fusion.snapshot(1.151)["nodes"] == {}
    # Being hidden alone does not create an arrival-gap diagnostic or a reset.
    assert fusion.gap_diagnostics == {} and fusion.epochs["1"] == 0
    fusion.update([sample(1, 1. + gap)])
    frame = fusion.snapshot(1. + gap)
    assert fusion.states["1"]["filter"] is original
    assert frame["nodes"]["1"]["epoch"] == 0
    assert frame["nodes"]["1"]["sample_index"] == 2
    assert "node_1_short_gap_filter_preserved" in frame["warnings"]
    official = VQF(1 / 120.)
    for _ in range(2):
        official.update(np.array([0., 0., 6000 * .00875 * math.pi / 180.]),
                        np.array([0., 0., 8197 * .000122 * 9.80665]))
    np.testing.assert_allclose(frame["nodes"]["1"]["quat_xyzw"], official.getQuat6D()[[1, 2, 3, 0]], atol=1e-13)
    info = frame["gap_diagnostics"]["1"]
    assert info["short_gaps"] == 1 and info["filter_resets"] == 0
    assert info["last_gap_s"] == pytest.approx(gap)
    assert info["last_gap_timestamp_s"] == pytest.approx(1. + gap)
    assert info["last_action"] == "filter_preserved"
    # Diagnostics survive warning consumption, but callers cannot mutate them.
    again = fusion.snapshot(1. + gap)
    assert again["gap_diagnostics"] == frame["gap_diagnostics"]
    assert "node_1_short_gap_filter_preserved" not in again["warnings"]
    frame["gap_diagnostics"]["1"]["short_gaps"] = 999
    assert fusion.gap_diagnostics["1"]["short_gaps"] == 1
    assert not fusion.snapshot(1. + gap + .151)["nodes"]
    json.dumps(again, allow_nan=False)


def test_long_gap_resets_only_returning_node_and_clock_rollback_still_resets_all():
    fusion = OnlineFusion({"stale_after_s": .15})
    fusion.update([sample(1, 0.), sample(2, 0.)])
    first, second = fusion.states["1"]["filter"], fusion.states["2"]["filter"]
    fusion.update([sample(2, .5), sample(2, 1.), sample(1, 1.001)])
    frame = fusion.snapshot(1.001)
    assert fusion.states["1"]["filter"] is not first
    assert fusion.states["2"]["filter"] is second
    assert [frame["nodes"][key]["epoch"] for key in ("1", "2")] == [1, 0]
    assert frame["gap_diagnostics"]["1"] == dict(short_gaps=0, filter_resets=1,
        last_gap_s=1.001, last_gap_timestamp_s=1.001, last_action="filter_reset")
    assert frame["gap_diagnostics"]["2"]["short_gaps"] == 2
    assert "node_1_gap_filter_reinitialized" in frame["warnings"]
    fusion.update([sample(2, .2)])
    reset = fusion.snapshot(.2)
    assert set(reset["nodes"]) == {"2"}
    assert fusion.epochs["1"] == 2 and fusion.epochs["2"] == 1 and fusion.epochs["12"] == 1
    assert fusion.states["2"]["filter"] is not second
    assert "host_clock_reset_filters_reinitialized" in reset["warnings"]
    # Gap counts exclude clock rollback: its separate warning/epoch remains.
    assert reset["gap_diagnostics"] == frame["gap_diagnostics"]


def test_delayed_batch_keeps_every_raw_sample_and_units(tmp_path):
    config = {"stale_after_s": .15, "gyro_dps_per_lsb_by_node": {"5": .0175, "6": .0175}}
    samples = [sample(node, 1.) for node in (5, 6)]
    samples += [sample(node, 1.3) for _ in range(36) for node in (5, 6)]
    fusion = OnlineFusion(config)
    fusion.update(samples)
    frame = fusion.snapshot(1.3)
    raw = tmp_path / "raw.csv"
    recorder = RawRecorder(raw).open()
    recorder.write(samples)
    recorder.close()
    report = replay_raw(raw, config, tmp_path / "replay.jsonl")
    replayed = list(iter_orientation_frames(tmp_path / "replay.jsonl"))[-1]
    for key in ("5", "6"):
        assert frame["nodes"][key]["sample_index"] == 37
        assert frame["nodes"][key]["epoch"] == 0
        assert report["nodes"][key]["samples"] == 37
        assert report["nodes"][key]["short_gaps"] == 1
        assert report["nodes"][key]["gaps"] == 0
        angle = 37 / 120 * 6000 * .0175 * math.pi / 180.
        np.testing.assert_allclose(frame["nodes"][key]["quat_xyzw"],
                                   [0., 0., math.sin(angle / 2), math.cos(angle / 2)], atol=1e-12)
        np.testing.assert_allclose(replayed["nodes"][key]["quat_xyzw"], frame["nodes"][key]["quat_xyzw"], atol=1e-13)
    assert replayed["gap_diagnostics"] == frame["gap_diagnostics"] == report["gap_diagnostics"]
    assert report["settings"]["filter_reset_gap_s"] == 1.


def test_replay_omits_stale_node_preserves_short_gaps_then_resets_long_gap(tmp_path):
    config = {"stale_after_s": .15}
    samples = [sample(1, 0.), sample(2, .16), sample(1, .164), sample(1, .464), sample(1, 1.465)]
    raw, out = tmp_path / "raw.csv", tmp_path / "frames.jsonl"
    recorder = RawRecorder(raw).open()
    recorder.write(samples)
    recorder.close()
    report = replay_raw(raw, config, out)
    frames = list(iter_orientation_frames(out))
    assert "1" not in frames[1]["nodes"]
    # The .164 sample is still integrated although not yet time to emit output.
    restored = next(frame for frame in frames if frame["timestamp_s"] == .464)
    assert restored["nodes"]["1"]["epoch"] == 0
    assert restored["nodes"]["1"]["sample_index"] == 3
    assert frames[-1]["nodes"]["1"]["epoch"] == 1
    assert report["nodes"]["1"]["gaps"] == 1
    assert report["nodes"]["1"]["short_gaps"] == 2
    assert report["gap_diagnostics"]["1"]["filter_resets"] == 1
    fusion, last_output, online = OnlineFusion(config), None, []
    for stamp, batch in groupby(samples, key=lambda item: item.host_monotonic_s):
        fusion.update(batch)
        if last_output is None or stamp - last_output >= 1 / 30 - 1e-9:
            online.append(fusion.snapshot(stamp)); last_output = stamp
    for actual, saved in zip(online, frames):
        assert actual["gap_diagnostics"] == saved["gap_diagnostics"]
        assert set(actual["warnings"]) == set(saved["warnings"])


@pytest.mark.parametrize("value", [0., -.1, .149, float("nan"), float("inf"), True, "1"])
def test_invalid_reset_threshold_rejected_before_replay_output(tmp_path, value):
    config = {"stale_after_s": .15, "filter_reset_gap_s": value}
    with pytest.raises(ValueError, match="filter_reset_gap_s"):
        OnlineFusion(config)
    out = tmp_path / "must_not_exist.jsonl"
    with pytest.raises(ValueError, match="filter_reset_gap_s"):
        replay_raw(tmp_path / "not_needed.csv", config, out)
    assert not out.exists()


def test_default_reset_threshold_and_explicit_legacy_policy():
    assert _configuration({})[0]["filter_reset_gap_s"] == 1.
    assert _configuration({"stale_after_s": 2.})[0]["filter_reset_gap_s"] == 2.
    fusion = OnlineFusion({"stale_after_s": .15, "filter_reset_gap_s": .15})
    fusion.update([sample(1, 1.), sample(1, 1.164)])
    assert fusion.snapshot(1.164)["nodes"]["1"]["epoch"] == 1
    assert fusion.gap_diagnostics["1"]["short_gaps"] == 0
