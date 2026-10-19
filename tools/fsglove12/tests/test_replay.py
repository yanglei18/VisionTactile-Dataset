"""Offline replay tests; no serial device or old hand solver is involved."""

import csv
import json
import math

import numpy as np
import pytest
from vqf import VQF

from vt_fsglove12.replay import iter_orientation_frames, replay_raw


FIELDS = ["host_parse_monotonic_s", "host_parse_wall_s", "node_id",
          "gx_lsb", "gy_lsb", "gz_lsb", "ax_lsb", "ay_lsb", "az_lsb"]


def raw_file(tmp_path, rows, optional=False):
    path = tmp_path / "raw.csv"
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(FIELDS + (["status", "phase"] if optional else []))
        for stamp, node, values in rows:
            writer.writerow([stamp, 1000 + stamp, node, *values] + ([0, "move"] if optional else []))
    return path


def static(stamp, node):
    return stamp, node, [0, 0, 0, 0, 0, 8197]


def test_full_batch_has_twelve_nodes_and_explicit_xyzw(tmp_path):
    path = raw_file(tmp_path, [static(1., node) for node in range(1, 13)], optional=True)
    output = tmp_path / "orientation.jsonl"
    report = replay_raw(path, {}, output)
    frames = list(iter_orientation_frames(output))
    assert report["frames"] == 1
    assert report["input_samples"] == 12
    assert report["hardware_synchronization_verified"] is False
    assert report["common_heading_established"] is False
    assert set(frames[0]["nodes"]) == {str(n) for n in range(1, 13)}
    node = frames[0]["nodes"]["1"]
    assert {key: node[key] for key in ("quat_xyzw", "age_s", "sample_index", "epoch")} == dict(
        quat_xyzw=[0., 0., 0., 1.], age_s=0., sample_index=1, epoch=0)
    assert node["raw_values"] == [0, 0, 0, 0, 0, 8197]
    assert node["gyro_rad_s"] == [0., 0., 0.]
    assert node["accel_m_s2"] == pytest.approx([0., 0., 8197 * .000122 * 9.80665])
    assert frames[0]["orientation_frame"] == "independent_initial_heading"


def test_equal_host_stamps_preserve_samples_nominal_time_and_per_node_scales(tmp_path):
    rows = [(4., node, [0, 0, 6000, 0, 0, 8197]) for _ in range(120) for node in (5, 6)]
    path = raw_file(tmp_path, rows)
    output = tmp_path / "out.jsonl"
    config = {"gyro_dps_per_lsb_by_node": {"5": .0175}}
    report = replay_raw(path, config, output)
    frame = next(iter_orientation_frames(output))
    for key, scale in (("5", .0175), ("6", .00875)):
        # Pure yaw is unobservable from gravity and integrates nominal samples.
        angle = 6000 * scale * math.pi / 180.
        expected = [0, 0, math.sin(angle / 2), math.cos(angle / 2)]
        assert frame["nodes"][key]["quat_xyzw"] == pytest.approx(expected)
        assert frame["nodes"][key]["sample_index"] == 120
        assert report["used_gyro_dps_per_lsb_by_node"][key] == scale
    assert report["frames"] == 1


def test_matches_official_vqf_with_si_units_and_scalar_first_conversion(tmp_path):
    values = [500, -250, 100, 1200, 1700, 7900]
    path = raw_file(tmp_path, [(1., 3, values) for _ in range(30)])
    output = tmp_path / "out.jsonl"
    replay_raw(path, {}, output)
    official = VQF(1 / 120)
    for _ in range(30):
        official.update(np.array(values[:3]) * .00875 * math.pi / 180.,
                        np.array(values[3:]) * .000122 * 9.80665)
    expected = official.getQuat6D()[[1, 2, 3, 0]]
    assert next(iter_orientation_frames(output))["nodes"]["3"]["quat_xyzw"] == pytest.approx(expected)


def test_stale_node_is_omitted_then_gap_creates_new_epoch(tmp_path):
    path = raw_file(tmp_path, [static(0., 1), static(.2, 2), static(.3, 1)])
    output = tmp_path / "out.jsonl"
    # Explicit legacy policy for reproducing evidence made with staleness reset.
    report = replay_raw(path, {"filter_reset_gap_s": .1}, output)
    frames = list(iter_orientation_frames(output))
    assert "1" not in frames[1]["nodes"]
    assert "node_1_stale" in frames[1]["warnings"]
    assert frames[2]["nodes"]["1"]["epoch"] == 1
    assert frames[2]["nodes"]["1"]["sample_index"] == 2
    assert report["nodes"]["1"]["gaps"] == 1
    assert report["nodes"]["1"]["stale_frames"] == 1
    assert report["nodes"]["12"]["missing_frames"] == 3


def test_clock_reset_clears_old_nodes_and_advances_epochs(tmp_path):
    path = raw_file(tmp_path, [static(2., 1), static(2., 2), static(.1, 1), static(.2, 2)])
    output = tmp_path / "out.jsonl"
    report = replay_raw(path, {}, output)
    frames = list(iter_orientation_frames(output))
    assert set(frames[1]["nodes"]) == {"1"}
    assert frames[1]["nodes"]["1"]["epoch"] == 1
    assert frames[2]["nodes"]["2"]["epoch"] == 1
    assert "host_clock_reset_filters_reinitialized" in frames[1]["warnings"]
    assert report["host_clock_resets"] == 1


def test_output_rate_and_max_frames_do_not_change_input_period(tmp_path):
    rows = [static(index / 120, node) for index in range(120) for node in (1, 2)]
    path = raw_file(tmp_path, rows)
    output = tmp_path / "out.jsonl"
    report = replay_raw(path, {}, output, max_frames=3)
    frames = list(iter_orientation_frames(output))
    assert [frame["frame_index"] for frame in frames] == [0, 1, 2]
    assert [frame["timestamp_s"] for frame in frames] == pytest.approx([0, 1 / 30, 2 / 30])
    assert frames[-1]["nodes"]["1"]["sample_index"] == 9
    assert report["stop_reason"] == "max_frames"


@pytest.mark.parametrize("config", [{"sample_rate_hz": 0}, {"stale_after_s": float("nan")},
    {"accel_g_per_lsb": True}, {"output_rate_hz": -1},
    {"gyro_dps_per_lsb_by_node": {"05": .0175}},
    {"gyro_dps_per_lsb_by_node": {"13": .0175}},
    {"gyro_dps_per_lsb_by_node": {"5": float("inf")}},
    {"gyro_dps_per_lsb_by_node": {"5": True}}, {"gyro_dps_per_lsb_by_node": []}])
def test_invalid_config_does_not_create_output(tmp_path, config):
    source = raw_file(tmp_path, [static(0., 1)])
    output = tmp_path / "out.jsonl"
    with pytest.raises(ValueError):
        replay_raw(source, config, output)
    assert not output.exists()


@pytest.mark.parametrize("maximum", [0, -1, True, 1.5])
def test_invalid_max_frames(tmp_path, maximum):
    source = raw_file(tmp_path, [static(0., 1)])
    with pytest.raises(ValueError, match="max_frames"):
        replay_raw(source, {}, tmp_path / "out.jsonl", max_frames=maximum)


@pytest.mark.parametrize("row", [(float("nan"), 1, [0] * 6), (0., 13, [0] * 6),
                                  (0., 1, [40000, 0, 0, 0, 0, 8197])])
def test_invalid_raw_row_is_not_completed(tmp_path, row):
    source = raw_file(tmp_path, [row])
    with pytest.raises(ValueError, match="raw CSV row"):
        replay_raw(source, {}, tmp_path / "out.jsonl")


def test_output_is_exclusive_and_input_is_unchanged(tmp_path):
    source = raw_file(tmp_path, [static(0., 1)])
    before = source.read_bytes()
    output = tmp_path / "out.jsonl"
    output.write_text("existing")
    with pytest.raises(FileExistsError):
        replay_raw(source, {}, output)
    with pytest.raises(ValueError, match="raw input"):
        replay_raw(source, {}, source)
    assert source.read_bytes() == before
    assert output.read_text() == "existing"


def test_reader_rejects_nonfinite_quaternion_and_missing_epoch(tmp_path):
    source = raw_file(tmp_path, [static(0., 1)])
    output = tmp_path / "out.jsonl"
    replay_raw(source, {}, output)
    frame = json.loads(output.read_text())
    for mutation in (lambda node: node.update(quat_xyzw=[0, 0, 0, float("nan")]),
                     lambda node: node.pop("epoch")):
        bad = json.loads(json.dumps(frame))
        mutation(bad["nodes"]["1"])
        invalid = tmp_path / "invalid.jsonl"
        invalid.write_text(json.dumps(bad) + "\n")
        with pytest.raises(ValueError, match="orientation frame"):
            list(iter_orientation_frames(invalid))
