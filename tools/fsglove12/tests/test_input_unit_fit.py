"""Synthetic raw counts with independently known tilt truth; no hardware."""
import copy
import csv
import hashlib
import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from vt_fsglove12.config import load_config, validate_config
from vt_fsglove12.input_unit_fit import fit_input_units


WINDOWS = [dict(start_s=.2, end_s=.8, label="flat"),
           dict(start_s=2.5, end_s=3.1, label="tilted"),
           dict(start_s=4.7, end_s=5.3, label="returned")]


def recording(tmp_path, *, gains=None, corrupt=None, degrees=60., rate=120.):
    config = load_config()
    config["gyro_dps_per_lsb_by_node"] = {}
    config["sample_rate_hz"] = rate
    times = np.arange(round(6 * rate) + 1) / rate
    angles = degrees * np.clip(times - 1.2, 0., 1.) - .75 * degrees * np.clip(times - 3.4, 0., 1.)
    speeds = np.r_[0., np.diff(angles)] * rate
    gravity = Rotation.from_rotvec(np.deg2rad(angles)[:, None] * [1., 0., 0.]).inv().apply([0., 0., 1.])
    rows = []
    gains = gains or {}
    for i, stamp in enumerate(times):
        for node in range(1, 13):
            scale = config["gyro_dps_per_lsb"] * gains.get(node, 1.)
            gyro = np.rint(np.array([speeds[i] / scale, 0., 0.]) + [12, -8, 4]).astype(int)
            accel = np.rint(gravity[i] / config["accel_g_per_lsb"]).astype(int)
            row = [1000. + stamp, 2000. + stamp, node, *gyro.tolist(), *accel.tolist()]
            if corrupt:
                row = corrupt(row, stamp, node)
            if row is not None:
                rows.append(row)
    path = tmp_path / "raw.csv"
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["host_parse_monotonic_s", "host_parse_wall_s", "node_id",
                         "gx_lsb", "gy_lsb", "gz_lsb", "ax_lsb", "ay_lsb", "az_lsb"])
        writer.writerows(rows)
    return path, config


def test_known_mixed_scales_use_heldout_and_preserve_nominal_nodes_raw_and_config(tmp_path):
    path, config = recording(tmp_path, gains={5: 2., 6: 2., 7: 2., 10: 2.})
    before = copy.deepcopy(config)
    original = path.read_bytes()
    report = fit_input_units(path, config, WINDOWS)
    assert report["recommended_nodes"] == ["5", "6", "7", "10"]
    assert report["nominal_supported_nodes"] == ["1", "2", "3", "4", "8", "9", "11", "12"]
    assert report["rejected_nodes"] == []
    for node, item in report["nodes"].items():
        assert abs(item["continuous_gain"] - (2. if node in report["recommended_nodes"] else 1.)) < .005
        assert [t["role"] for t in item["transitions"]] == ["fit", "heldout"]
        assert all(t["candidate_residual_deg"] < .1 for t in item["transitions"])
    assert report["candidate_config"]["gyro_dps_per_lsb_by_node"] == {str(n): .0175 for n in (5, 6, 7, 10)}
    assert not report["candidate_config"]["units_verified"]
    assert report["candidate_config"]["input_unit_fit_evidence"]["original_provenance"] == config.get("provenance")
    assert "node 5=0.0175" in report["candidate_config"]["provenance"]
    assert "not sensor full-scale register readback" in report["candidate_config"]["provenance"]
    assert "not verification of all 12 IMUs" in report["candidate_config"]["provenance"]
    assert not report["physical_range_verified"] and not report["hardware_accessed"]
    assert config == before and path.read_bytes() == original
    prefix = original[:report["source_prefix_bytes"]]
    assert hashlib.sha256(prefix).hexdigest() == report["source_prefix_sha256"]
    assert report["stop_reason"] == "window_end_lookahead" and len(prefix) < len(original)
    assert json.loads(json.dumps(report, allow_nan=False)) == report
    validate_config(report["candidate_config"])


def test_half_scale_is_a_candidate_and_continuous_near_one_does_not_rewrite(tmp_path):
    path, config = recording(tmp_path, gains={5: .5, 6: .99})
    report = fit_input_units(path, config, WINDOWS)
    assert report["recommended_nodes"] == ["5"]
    assert report["nodes"]["5"]["recommended_gain"] == .5
    assert report["nodes"]["6"]["status"] == "nominal_supported"
    assert "6" not in report["candidate_config"]["gyro_dps_per_lsb_by_node"]


def test_nominal_only_returns_unchanged_config(tmp_path):
    path, config = recording(tmp_path)
    report = fit_input_units(path, config, WINDOWS)
    assert not report["recommended_nodes"] and len(report["nominal_supported_nodes"]) == 12
    assert report["candidate_config"] == config


def test_wrong_axis_only_in_heldout_cannot_pass_first_fit(tmp_path):
    def bad(row, time, node):
        if node == 5 and 3.4 < time <= 4.4:
            row[3], row[4] = 12, row[3] - 12 - 8
        return row
    path, config = recording(tmp_path, gains={5: 2.}, corrupt=bad)
    result = fit_input_units(path, config, WINDOWS)
    node = result["nodes"]["5"]
    assert node["status"] == "rejected"
    assert node["transitions"][0]["candidate_residual_deg"] < .1
    assert node["transitions"][1]["candidate_residual_deg"] > 20
    assert "5" not in result["candidate_config"]["gyro_dps_per_lsb_by_node"]
    assert len(result["nominal_supported_nodes"]) == 11


@pytest.mark.parametrize("failure", ["boundary", "gap", "rollback", "moving_anchor", "missing"])
def test_bad_node_is_rejected_without_blocking_others(tmp_path, failure):
    def bad(row, time, node):
        if node != 5:
            return row
        if failure == "missing":
            return None
        if failure == "boundary" and 1.3 < time < 1.31:
            row[3] = 32764
        if failure == "gap" and 1.4 < time < 1.7:
            return None
        if failure == "rollback" and 1.5 < time < 1.51:
            row[0] -= .2
        if failure == "moving_anchor" and .2 <= time <= .8:
            row[3] += 2000 if round(time * 120) % 2 else -2000
        return row
    path, config = recording(tmp_path, corrupt=bad)
    result = fit_input_units(path, config, WINDOWS)
    assert result["nodes"]["5"]["status"] == "rejected"
    assert len(result["nominal_supported_nodes"]) == 11


def test_small_tilt_and_non_dyadic_gain_are_not_recommended(tmp_path):
    path, config = recording(tmp_path, degrees=10.)
    report = fit_input_units(path, config, WINDOWS)
    assert len(report["rejected_nodes"]) == 12
    path, config = recording(tmp_path, gains={5: 1.4})
    report = fit_input_units(path, config, WINDOWS)
    assert report["nodes"]["5"]["reasons"] == ["continuous_gain_not_supported_dyadic_candidate"]


def test_absolute_window_basis_and_explicit_bias_are_equivalent(tmp_path):
    path, config = recording(tmp_path, gains={5: 2.})
    windows = [{**w, "start_s": w["start_s"] + 1000., "end_s": w["end_s"] + 1000.,
                "time_basis": "absolute_monotonic"} for w in WINDOWS]
    bias = dict(start_s=1000., end_s=1001., label="independent_bias", time_basis="absolute_monotonic")
    report = fit_input_units(path, config, windows, bias_window=bias)
    assert report["recommended_nodes"] == ["5"]
    assert report["nodes"]["5"]["transitions"][0]["bias_source"] == "explicit_independent_stationary_window"
    assert report["nodes"]["5"]["bias_window"]["accepted"]


@pytest.mark.parametrize("absolute_anchors", [True, False])
def test_bias_window_can_use_independent_time_basis(tmp_path, absolute_anchors):
    path, config = recording(tmp_path, gains={5: 2.})
    windows = copy.deepcopy(WINDOWS)
    if absolute_anchors:
        windows = [{**w, "start_s": w["start_s"] + 1000., "end_s": w["end_s"] + 1000.,
                    "time_basis": "absolute_monotonic"} for w in windows]
        bias = dict(start_s=0., end_s=1., label="bias")
    else:
        bias = dict(start_s=1000., end_s=1001., label="bias", time_basis="absolute_monotonic")
    original = copy.deepcopy(bias)
    report = fit_input_units(path, config, windows, bias_window=bias)
    assert report["recommended_nodes"] == ["5"] and bias == original
    assert report["bias_window"]["start_s"] == original["start_s"]
    assert report["normalized_bias_window"]["start_s"] == (1000. if absolute_anchors else 0.)


def test_endpoint_micro_motion_is_not_silently_used_as_bias(tmp_path):
    def micro(row, time, node):
        if node == 5 and any(w["start_s"] <= time <= w["end_s"] for w in WINDOWS):
            row[3] += 60 if round(time * 120) % 2 else -60
        return row
    path, config = recording(tmp_path, gains={5: 2.}, corrupt=micro)
    without = fit_input_units(path, config, WINDOWS)
    assert without["nodes"]["5"]["status"] == "rejected"
    assert "provide_independent_bias_window" in without["nodes"]["5"]["transitions"][0]["reasons"][0]
    # The first .2 seconds are shorter than our required bias window, so take
    # a separate, unmodified one-second rest immediately after the first hold.
    with_bias = fit_input_units(path, config, WINDOWS,
        bias_window=dict(start_s=.85, end_s=1.15, label="independent_rest"))
    assert with_bias["nodes"]["5"]["status"] == "recommended"


def test_independent_bias_window_motion_rejects_only_affected_node(tmp_path):
    def bad(row, time, node):
        if node == 5 and 0 <= time <= 1.:
            row[3] += 100 if round(time * 120) % 2 else -100
        return row
    path, config = recording(tmp_path, corrupt=bad)
    report = fit_input_units(path, config, WINDOWS,
        bias_window=dict(start_s=0., end_s=1., label="bad_bias"))
    assert report["nodes"]["5"]["reasons"] == ["independent_bias_window_not_stationary"]
    assert len(report["nominal_supported_nodes"]) == 11


def test_truncated_prefix_does_not_claim_heldout_success(tmp_path):
    path, config = recording(tmp_path)
    report = fit_input_units(path, config, WINDOWS, max_samples=120)
    assert report["input_samples"] == 120 and report["stop_reason"] == "sample_limit"
    assert not report["recommended_nodes"] and not report["nominal_supported_nodes"]


@pytest.mark.parametrize("windows", [[], WINDOWS[:2], list(reversed(WINDOWS)),
    [{**WINDOWS[0], "start_s": float("nan")}, *WINDOWS[1:]],
    [{**WINDOWS[0], "label": ""}, *WINDOWS[1:]]])
def test_invalid_annotations_rejected_before_file_read(windows):
    with pytest.raises(ValueError):
        fit_input_units("/no/such/file", load_config(), windows)
