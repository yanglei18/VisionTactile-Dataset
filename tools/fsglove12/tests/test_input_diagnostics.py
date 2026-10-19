"""Known-input regressions, not hardware accuracy claims."""
from copy import deepcopy
import hashlib
import importlib.util
from itertools import groupby
import json
from pathlib import Path

import numpy as np
import pytest
from vqf import VQF

from vt_fsglove12.cli import main
from vt_fsglove12.config import load_config
from vt_fsglove12.input_contract import effective_input_scales, raw_to_si
from vt_fsglove12.input_diagnostics import InputObservationMonitor, inspect_raw_input, describe_filter_state
from vt_fsglove12.live_stream import ImuParser, OnlineFusion, RawRecorder, RawSample
from vt_fsglove12.replay import iter_orientation_frames, replay_raw


def write_rows(tmp_path, rows):
    path = tmp_path / "raw.csv"
    recorder = RawRecorder(path).open()
    recorder.write(rows)
    recorder.close()
    return path


def test_twelve_nodes_report_observed_rate_without_changing_nominal_or_dropping_duplicates():
    monitor = InputObservationMonitor(120.)
    for i in range(241):
        for node in range(1, 13):
            monitor.observe(node, (0, 0, 0, 0, 0, 8197), 10. + i / 100.)
    report = monitor.snapshot()
    assert report["physical_saturation"] == "unknown"
    assert report["automatic_scale_or_timing_changes"] is False
    for node in report["nodes"].values():
        assert node["samples"] == 241
        assert node["observed_receive_rate_hz"] == pytest.approx(100.)
        assert node["nominal_sample_rate_hz"] == 120.
        assert node["repeated_payload_samples"] == 240
        assert node["receive_rate_to_nominal_ratio"] == pytest.approx(100. / 120.)
    report["nodes"]["1"]["raw_min"]["gx"] = 999
    assert monitor.snapshot()["nodes"]["1"]["raw_min"]["gx"] == 0


def test_clock_reset_and_equal_host_times_do_not_invent_sensor_rate():
    monitor = InputObservationMonitor()
    for i in range(120):
        monitor.observe(1, (32764, 0, 0, 0, 0, 8197), 10.)
    state = monitor.snapshot()["nodes"]["1"]
    assert state["samples"] == 120
    assert state["observed_receive_rate_hz"] is None
    assert state["equal_host_timestamp_intervals"] == 119
    assert state["near_boundary_by_axis"]["gx"] == 120
    monitor.observe(1, (0, 0, 0, 0, 0, 8197), 1.)
    report = monitor.snapshot()
    assert report["host_clock_resets"] == 1
    assert report["nodes"]["1"]["samples"] == 121
    assert report["nodes"]["1"]["epoch_samples"] == 1
    assert report["nodes"]["1"]["observed_receive_rate_hz"] is None
    assert report["nodes"]["2"]["samples"] == 0


def test_inspection_hashes_exact_bounded_prefix_and_keeps_source(tmp_path):
    values = (32764, -32768, 0, 0, 0, 8197)
    path = write_rows(tmp_path, [RawSample(n, values, 1. + i / 120., 100.)
                               for i in range(130) for n in range(1, 13)])
    original = path.read_bytes()
    result = inspect_raw_input(path, load_config(), max_samples=1440)
    prefix = b"".join(original.splitlines(keepends=True)[:1441])
    assert result["source_prefix_sha256"] == hashlib.sha256(prefix).hexdigest()
    assert result["source_prefix_bytes"] == len(prefix)
    assert result["input_samples"] == 1440 and result["stop_reason"] == "sample_limit"
    assert result["hardware_accessed"] is False
    assert result["units_auto_verified"] is False
    assert result["observation"]["nodes"]["1"]["near_boundary_by_axis"]["gy"] == 120
    assert path.read_bytes() == original


def test_inspection_empty_and_bad_fields(tmp_path):
    path = write_rows(tmp_path, [])
    report = inspect_raw_input(path, {})
    assert report["input_samples"] == 0
    assert all(v["observed_receive_rate_hz"] is None for v in report["observation"]["nodes"].values())
    with pytest.raises(ValueError, match="positive integer"):
        inspect_raw_input(path, {}, max_samples=0)
    path.write_bytes(path.read_bytes() + b"1,2,1,32768,0,0,0,0,8197,0\n")
    with pytest.raises(ValueError, match="signed 16-bit"):
        inspect_raw_input(path, {})


def test_cli_configuration_and_read_only_report_do_not_require_device(tmp_path, capsys):
    assert main(["inspect-input"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["input_contract"]["sensor_model"] == "LSM6DSRTR"
    assert payload["hardware_accessed"] is False
    path = write_rows(tmp_path, [RawSample(12, (0, 0, 0, 0, 0, 8197), 1., 100.)])
    output = tmp_path / "report.json"
    args = ["inspect-input", "--raw", str(path), "--output", str(output)]
    assert main(args) == 0
    saved = output.read_bytes()
    assert main(args) == 2
    assert output.read_bytes() == saved


def test_all_twelve_nodes_live_replay_si_and_actual_vqf_parity(tmp_path):
    config = load_config()
    config["gyro_dps_per_lsb_by_node"] = {"5": .0175}
    config["accel_g_per_lsb_by_node"] = {"9": .000244}
    rows = [RawSample(n, (n * 17, -n * 8, 300, n * 90, -n * 40, 8197),
                      10. + i / 120., 100. + i / 120.) for i in range(125) for n in range(1, 13)]
    original = deepcopy(rows)
    raw = write_rows(tmp_path, rows)
    destination = tmp_path / "orientations.jsonl"
    report = replay_raw(raw, config, destination)
    offline = {f["timestamp_s"]: f for f in iter_orientation_frames(destination)}
    fusion = OnlineFusion(config)
    gyros, accels = effective_input_scales(config)
    reference = {str(n): VQF(1. / 120.) for n in range(1, 13)}
    for stamp, batch in groupby(rows, key=lambda r: r.host_monotonic_s):
        batch = list(batch)
        fusion.update(batch)
        for sample in batch:
            key = str(sample.node_id)
            gyro, accel = raw_to_si(sample.values, gyro_dps_per_lsb=gyros[key], accel_g_per_lsb=accels[key])
            reference[key].update(gyro, accel)
        if stamp in offline:
            frame = fusion.snapshot(stamp)
            assert frame["input_observation"] == offline[stamp]["input_observation"]
            for key in frame["nodes"]:
                node = frame["nodes"][key]
                for field in ("raw_values", "gyro_rad_s", "accel_m_s2", "quat_xyzw"):
                    np.testing.assert_array_equal(node[field], offline[stamp]["nodes"][key][field])
                assert node["fusion_state"] == offline[stamp]["nodes"][key]["fusion_state"]
                np.testing.assert_array_equal(node["quat_xyzw"], reference[key].getQuat6D()[[1, 2, 3, 0]])
    assert report["used_accel_g_per_lsb_by_node"]["9"] == .000244
    assert rows == original


def test_official_bias_rest_diagnostics_do_not_advance_or_retune_filter():
    filter = VQF(1. / 120.)
    for _ in range(1200):
        filter.update(np.zeros(3), np.asarray([0., 0., 9.80665]))
    before = filter.getQuat6D().copy()
    first = describe_filter_state(filter)
    second = describe_filter_state(filter)
    assert first == second
    assert first["rest_detected"]
    assert not first["absolute_heading_observed"]
    assert first["bias_sigma_rad_s"] >= 0
    np.testing.assert_array_equal(before, filter.getQuat6D())


def test_monitor_per_node_scales_match_pose_adapter_without_second_serial_reader():
    pytest.importorskip("PyQt5")
    source = Path(__file__).resolve().parents[3] / "Ploy_IMU.py"
    if not source.is_file():
        pytest.skip("standalone waveform application is not part of the installed package")
    pytest.importorskip("pyqtgraph")
    spec = importlib.util.spec_from_file_location("vt_imu_plot_scale_test", source)
    monitor = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(monitor)
    config = load_config()
    config.update(gyro_dps_per_lsb_by_node={"6": .0175}, accel_g_per_lsb_by_node={"11": .000244})
    actual = monitor.node_unit_scales(config)
    gyros, accels = effective_input_scales(config)
    for node in range(1, 13):
        assert actual[node - 1] == (gyros[str(node)],) * 3 + (accels[str(node)],) * 3
    with pytest.raises(ValueError, match="accel"):
        monitor.node_unit_scales({"accel_g_per_lsb_by_node": {"01": .000244}})


def test_explicit_unsupported_wire_contract_cannot_silently_reach_fusion(tmp_path):
    config = {"input_contract": {"encoding": "float32_le"}}
    with pytest.raises(ValueError, match="encoding"):
        OnlineFusion(config)
    raw = write_rows(tmp_path, [])
    with pytest.raises(ValueError, match="encoding"):
        replay_raw(raw, config, tmp_path / "out.jsonl")
    assert not (tmp_path / "out.jsonl").exists()
