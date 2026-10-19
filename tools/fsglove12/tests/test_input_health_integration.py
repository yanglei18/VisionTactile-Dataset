"""The interlock pauses VQF without dropping raw evidence or laundering faults."""
import numpy as np
import pytest

from vt_fsglove12.live_stream import OnlineFusion, RawSample


FROZEN_BANK = {
    7: (-468, 253, -974, 501, 759, 8140),
    8: (681, -1233, -1044, 1089, 5356, 6315),
    9: (-165, 1398, -1926, 759, 48, 8702),
    10: (403, -349, -603, 531, 5798, 6168),
    11: (2200, 5765, -3241, 1631, 4274, 7582),
    12: (-1798, 3455, -1541, -247, 3116, 7514),
}


def make_fusion():
    fusion = OnlineFusion({"input_filter": {"enabled": True},
                           "input_health": {"enabled": True}})
    original = fusion._factory
    fusion.update_calls = {str(node): 0 for node in range(1, 13)}
    # Count filter calls through the actual VQF; no hardware or timing threads.
    class Filter:
        def __init__(self, period):
            self.inner, self.calls = original(period), 0
        def update(self, *args):
            self.calls += 1
            return self.inner.update(*args)
        def __getattr__(self, key):
            return getattr(self.inner, key)
    fusion._factory = Filter
    return fusion


def rows(index, stamp, *, frozen=False, nodes=range(1, 13)):
    result = []
    for node in nodes:
        values = (10 + index % 40, 0, 100 + node, index % 100, 0, 8197)
        if frozen and node in FROZEN_BANK:
            values = FROZEN_BANK[node]
        result.append(RawSample(node, values, stamp, stamp + 1000., status=0))
    return result


def test_legacy_stream_unchanged_when_policy_is_absent():
    fusion = OnlineFusion({})
    for index in range(30):
        fusion.update(rows(index, 10. + index / 6.))
    frame = fusion.snapshot(10. + 29 / 6.)
    assert "input_health" not in frame
    assert all(epoch == 0 for epoch in fusion.epochs.values())


def test_rate_fault_stops_vqf_once_but_raw_guard_and_receive_counts_continue():
    fusion = make_fusion()
    for index in range(25):
        fusion.update(rows(index, 10. + index / 6.))
    old = {key: (value["filter"].calls, list(value["quat_xyzw"])) for key, value in fusion.states.items()}
    for index in range(25, 40):
        fusion.update(rows(index, 10. + index / 6.))
    frame = fusion.snapshot(10. + 39 / 6.)
    for key, data in frame["nodes"].items():
        assert data["orientation_valid"] is False
        assert data["epoch"] == 1
        assert data["sample_index"] == 40
        assert data["fusion_gyro_rad_s"] is None
        assert data["input_filter"]["raw_samples"] == 40
        assert fusion.states[key]["filter"].calls == old[key][0]
        np.testing.assert_array_equal(data["quat_xyzw"], old[key][1])
    assert all(value["samples"] == 40 for value in frame["input_observation"]["nodes"].values())


def test_real_recorded_frozen_bank_payload_cannot_continue_integrating():
    fusion = make_fusion()
    for index in range(480):
        fusion.update(rows(index, 10. + index / 120., frozen=True))
    frame = fusion.snapshot(10. + 479 / 120.)
    assert frame["input_health"]["faulted_nodes"] == [str(n) for n in range(7, 13)]
    old = {key: (value["filter"].calls, list(value["quat_xyzw"])) for key, value in fusion.states.items()}
    for index in range(480, 600):
        fusion.update(rows(index, 10. + index / 120., frozen=True))
    frame = fusion.snapshot(10. + 599 / 120.)
    for node in range(7, 13):
        key = str(node)
        assert frame["nodes"][key]["orientation_valid"] is False
        assert frame["nodes"][key]["epoch"] == 1
        assert fusion.states[key]["filter"].calls == old[key][0]
        np.testing.assert_array_equal(frame["nodes"][key]["quat_xyzw"], old[key][1])
        assert frame["nodes"][key]["raw_values"] == list(FROZEN_BANK[node])
    assert fusion.states["1"]["filter"].calls == old["1"][0] + 120


def test_calibration_reset_does_not_launder_bad_stream_and_recovery_recreates_filters():
    fusion = make_fusion()
    for index in range(480):
        fusion.update(rows(index, 10. + index / 120., frozen=True))
    fusion.reset_for_calibration()
    assert fusion.counts["12"] == 480
    for index in range(480, 510):
        fusion.update(rows(index, 10. + index / 120., frozen=True))
    assert fusion.states["12"]["filter"].calls == 0
    assert fusion.snapshot(10. + 509 / 120.)["nodes"]["12"]["orientation_valid"] is False
    paused = fusion.states["12"]["filter"]
    for index in range(510, 780):
        fusion.update(rows(index, 10. + index / 120.))
    frame = fusion.snapshot(10. + 779 / 120.)
    assert not frame["input_health"]["faulted_nodes"]
    assert frame["nodes"]["12"]["orientation_valid"] is True
    assert fusion.states["12"]["filter"] is not paused
    assert fusion.epochs["12"] == 2  # One fault, one explicit calibration reset.
    assert fusion.counts["12"] == 780
    assert frame["input_health"]["nodes"]["12"]["samples"] == 780


def test_clock_rollback_preserves_fault_latch_but_clears_old_time_evidence():
    fusion = make_fusion()
    for index in range(25):
        fusion.update(rows(index, 10. + index / 6.))
    fusion.update(rows(25, 1.))
    frame = fusion.snapshot(1.)
    assert frame["input_health"]["clock_resets"] == 1
    assert len(frame["input_health"]["faulted_nodes"]) == 12
    assert all(data["orientation_valid"] is False for data in frame["nodes"].values())
    assert fusion.counts["12"] == 26
    assert fusion.epochs["12"] == 2


def test_already_faulted_slow_stream_does_not_reset_filter_every_long_gap():
    fusion = make_fusion()
    for index in range(25):
        fusion.update(rows(index, 10. + index / 6.))
    epoch, paused = fusion.epochs["12"], fusion.states["12"]["filter"]
    calls = paused.calls
    for index in range(5):
        fusion.update(rows(25 + index, 16. + index * 2.))
    assert fusion.epochs["12"] == epoch
    assert fusion.states["12"]["filter"] is paused
    assert paused.calls == calls


def test_snapshot_health_is_detached_including_hidden_stale_nodes():
    fusion = make_fusion()
    for index in range(260):
        fusion.update(rows(index, 10. + index / 120.))
    now = 10. + 259 / 120.
    snapshot = fusion.snapshot(now)
    snapshot["input_health"]["nodes"]["12"]["reasons"].append("external")
    snapshot["nodes"]["12"]["input_health"]["healthy"] = False
    assert fusion.snapshot(now)["nodes"]["12"]["input_health"]["healthy"] is True
    stale = fusion.snapshot(now + .2)
    assert stale["nodes"] == {}
    assert stale["input_health"]["nodes"]["12"]["state"] == "stale"
