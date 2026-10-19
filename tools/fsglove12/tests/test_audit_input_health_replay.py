"""Bounded recorded-row audits with real VQF; no serial or model startup."""
import csv
import importlib.util
from pathlib import Path

import pytest

from vt_fsglove12.config import load_config
from vt_fsglove12.live_stream import CSV_FIELDS


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "audit_input_health.py"
SPEC = importlib.util.spec_from_file_location("audit_input_health_script", SCRIPT)
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


def _write_rows(path, rows):
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(CSV_FIELDS)
        writer.writerows(rows)


def _healthy_rows(seconds=2.2):
    for index in range(round(seconds * 120) + 1):
        stamp = 100. + index / 120.
        for node in range(1, 13):
            # Explicitly synthetic test input, not production/audit samples.
            yield [stamp, stamp + 1000., node, 20 + index % 3, -15, 5,
                   index % 2, 0, 8192, index % 251]


def _failed_rows(healthy_seconds=2.2, failed_seconds=4.5):
    yield from _healthy_rows(healthy_seconds)
    for index in range(1, round(failed_seconds * (80. / 13.)) + 1):
        stamp = 100. + healthy_seconds + index * 13. / 80.
        for node in range(1, 13):
            if node >= 7:
                values, status = [-1798, 3455, -1541, -247, 3116, 7514], 208
            else:
                values, status = [index % 31, -index % 11, index, index % 2, 0, 8192], index % 251
            yield [stamp, stamp + 1000., node, *values, status]


def test_healthy_recorded_rows_are_cadence_invariant_and_never_upsampled(tmp_path):
    raw = tmp_path / "raw.csv"
    rows = list(_healthy_rows())
    _write_rows(raw, rows)
    original = raw.read_bytes()
    config = load_config()
    nominal = config["sample_rate_hz"]
    result = audit.audit_replay(raw, config)
    assert raw.read_bytes() == original
    assert result["received_rows"] == len(rows)
    assert result["source"]["replayed_rows"] == len(rows)
    assert result["synthetic_input_rows"] == 0 and result["receipt_counts_preserved"]
    assert config["sample_rate_hz"] == result["nominal_sample_rate_hz"] == nominal
    comparison = result["cadence_comparison"]
    assert comparison["invariant"] and comparison["matched_checks"] > 60
    assert comparison["max_quaternion_component_difference"] == 0.
    assert comparison["snapshot_counts"][120] > comparison["snapshot_counts"][30] * 3
    for state in result["final"].values():
        assert state["actual_vqf_update_calls"] == len(rows)
        assert state["raw_rows_not_integrated"] == 0
        assert state["input_health"]["faulted_nodes"] == []
        assert all(epoch == 0 for epoch in state["epochs"].values())
        assert len(state["valid_orientation_nodes"]) == 12
    assert result["final"]["30"]["stored_quaternions"] == result["final"]["120"]["stored_quaternions"]
    assert result["raw_nodes"]["12"]["observed_receive_rate_hz"] == pytest.approx(120.)


def test_recorded_rate_drop_and_frozen_bank_pause_vqf_but_preserve_every_row(tmp_path):
    raw = tmp_path / "raw.csv"
    rows = list(_failed_rows())
    _write_rows(raw, rows)
    original = raw.read_bytes()
    result = audit.audit_replay(raw, load_config())
    assert raw.read_bytes() == original
    assert result["received_rows"] == len(rows) and result["receipt_counts_preserved"]
    assert result["cadence_comparison"]["invariant"]
    for state in result["final"].values():
        assert state["raw_rows_not_integrated"] > 0
        assert state["actual_vqf_update_calls"] < result["received_rows"]
        assert state["valid_orientation_nodes"] == []
        for node in map(str, range(1, 13)):
            health = state["input_health"]["nodes"][node]
            assert health["faulted"] and not health["integration_allowed"]
            assert health["observed_receive_rate_hz"] == pytest.approx(80. / 13.)
            assert state["epochs"][node] == health["fault_count"] == 1
            assert state["received_counts"][node] == result["raw_nodes"][node]["received_samples"]
            if int(node) >= 7:
                assert "frozen_bank_payload" in health["reasons"]
    assert result["final"]["30"]["stored_quaternions"] == result["final"]["120"]["stored_quaternions"]
    # Extending an already-faulted input cannot move its stored orientations.
    longer = tmp_path / "longer.csv"
    _write_rows(longer, _failed_rows(failed_seconds=5.5))
    extended = audit.audit_replay(longer, load_config())
    assert extended["received_rows"] > result["received_rows"]
    assert extended["final"]["120"]["actual_vqf_update_calls"] == result["final"]["120"]["actual_vqf_update_calls"]
    assert extended["final"]["120"]["stored_quaternions"] == result["final"]["120"]["stored_quaternions"]


def test_disabled_control_integrates_identical_bad_recorded_rows_without_hiding_cadence(tmp_path):
    raw = tmp_path / "raw.csv"
    rows = list(_failed_rows(healthy_seconds=0., failed_seconds=3.5))
    _write_rows(raw, rows)
    result = audit.audit_replay(raw, load_config(), input_health="disabled")
    assert result["received_rows"] == len(rows) and result["receipt_counts_preserved"]
    assert result["cadence_comparison"]["invariant"]
    assert result["raw_nodes"]["12"]["observed_receive_rate_hz"] < 7.
    for state in result["final"].values():
        assert state["actual_vqf_update_calls"] == len(rows)
        assert state["raw_rows_not_integrated"] == 0


def test_byte_offset_window_never_scans_prefix_or_rewrites_timestamps(tmp_path):
    raw = tmp_path / "raw.csv"
    rows = list(_healthy_rows(.03))
    _write_rows(raw, rows)
    data = raw.read_bytes()
    lines = data.splitlines(keepends=True)
    boundary = sum(map(len, lines[:7]))
    for offset, expected_index in ((boundary, 6), (boundary + 5, 7)):
        window = audit.BoundedRawWindow(raw, start_byte=offset, max_samples=3)
        replayed = list(window)
        assert len(replayed) == 3
        assert replayed[0].node_id == rows[expected_index][2]
        assert replayed[0].host_monotonic_s == rows[expected_index][0]
        assert replayed[0].host_wall_s == rows[expected_index][1]
        assert window.summary["bytes_read"] < len(data)
        assert window.summary["stop_reason"] == "sample_limit"
        assert window.summary["actual_start_byte"] >= offset
    assert raw.read_bytes() == data


def test_byte_limit_and_incomplete_tail_are_bounded_without_invented_rows(tmp_path):
    raw = tmp_path / "raw.csv"
    _write_rows(raw, _healthy_rows(.1))
    window = audit.BoundedRawWindow(raw, max_bytes=256)
    values = list(window)
    assert window.summary["bytes_read"] <= 256
    assert len(values) == window.summary["replayed_rows"] < 12
    raw.write_bytes(raw.read_bytes() + b"101.0,broken")
    window = audit.BoundedRawWindow(raw)
    values = list(window)
    assert window.summary["stop_reason"] == "incomplete_or_bounded_tail"
    assert len(values) == 13 * 12


@pytest.mark.parametrize("kwargs", [dict(max_bytes=50_000_001), dict(max_samples=50_001),
    dict(start_byte=-1), dict(max_duration_s=float("inf")), dict(max_duration_s=61.)])
def test_unbounded_or_invalid_audit_requests_are_rejected(tmp_path, kwargs):
    raw = tmp_path / "raw.csv"
    _write_rows(raw, _healthy_rows(.01))
    with pytest.raises(ValueError):
        audit.BoundedRawWindow(raw, **kwargs)
