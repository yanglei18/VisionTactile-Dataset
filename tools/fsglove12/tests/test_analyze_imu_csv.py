"""Offline CSV snapshot and integer range checks, no hardware."""
import csv
import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "analyze_imu_csv.py"
SPEC = importlib.util.spec_from_file_location("analyze_imu_csv_script", SCRIPT)
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)
HEADER = b"host_parse_monotonic_s,host_parse_wall_s,node_id,gx_lsb,gy_lsb,gz_lsb,ax_lsb,ay_lsb,az_lsb,status\r\n"


def row(stamp, node=1, values=(32000, -32768, 32767, 40000, -40000, 0)):
    return (f"{stamp},1000,{node}," + ",".join(str(v) for v in values) + ",253\r\n").encode()


def run(tmp_path, content, **kwargs):
    source = tmp_path / "raw.csv"
    source.write_bytes(content)
    output = tmp_path / "analysis"
    return audit.analyze_csv(source, output, **kwargs), output


def read_csv(output, filename):
    with (output / filename).open(newline="") as stream:
        return list(csv.DictReader(stream))


def test_integer_extremes_not_cast_or_clamped_and_all_nodes(tmp_path):
    data = HEADER + b"".join(row(100, node) for node in range(1, 13))
    report, output = run(tmp_path, data)
    assert report["selected_rows"] == 12 and report["nodes_missing"] == []
    assert report["totals"] == dict(exact_minus_32768_count=12, exact_plus_32767_count=12,
                                     near_boundary_abs_ge_32760_count=48, outside_signed16_count=24)
    assert report["signed_observed_min"] == -40000 and report["signed_observed_max"] == 40000
    stats = {(info["node_id"], info["axis"]): info for info in report["axis_summary"]}
    assert stats[1, "gx"]["max_abs"] == 32000 and stats[1, "gy"]["max_abs"] == 32768
    assert stats[1, "ax"]["signed_max"] == 40000
    assert (output / "samples.csv").read_bytes() == data
    assert report["source_prefix_sha256"] == hashlib.sha256(data).hexdigest()
    assert report["samples_csv_sha256"] == hashlib.sha256(data).hexdigest()
    assert len(read_csv(output, "summary.csv")) == 72
    events = read_csv(output, "boundary_events.csv")
    assert len(events) == 48 and events[0]["source_line_number"] == "2"
    assert events[0]["host_parse_monotonic_s"] == "100"
    assert events[2]["value_lsb"] == "40000" and events[2]["outside_signed16"] == "1"
    assert json.loads((output / "report.json").read_text()) == report
    assert not report["wire_bytes_available"] and not report["hardware_accessed"]


def test_select_only_inclusive_last_thirty_seconds_and_original_bytes(tmp_path):
    selected = row("070.000") + row(80) + row(100)
    report, output = run(tmp_path, HEADER + row(0) + row(69) + selected, last_seconds=30)
    assert (output / "samples.csv").read_bytes() == HEADER + selected
    assert report["source_complete_data_rows"] == 5 and report["selected_rows"] == 3
    assert report["first_selected_source_line"] == 4
    assert report["window_start_host_monotonic_s"] == 70
    assert report["window_end_host_monotonic_s"] == 100
    assert report["nodes"]["1"]["observed_receive_rate_hz"] == pytest.approx(2 / 30)
    assert report["nodes"]["2"]["observed_receive_rate_hz"] is None


def test_partial_last_row_excluded_but_included_in_prefix_hash(tmp_path):
    partial = b"101,1000,1,bad,incomplete"
    data = HEADER + row(100) + partial
    report, output = run(tmp_path, data)
    assert (output / "samples.csv").read_bytes() == HEADER + row(100)
    assert report["source_incomplete_final_row_bytes_excluded"] == len(partial)
    assert report["source_prefix_bytes"] == len(data)
    assert report["source_prefix_sha256"] == hashlib.sha256(data).hexdigest()
    assert report["window_end_host_monotonic_s"] == 100


def test_rows_appended_after_initial_size_are_not_read(tmp_path, monkeypatch):
    source = tmp_path / "raw.csv"
    data = HEADER + row(100)
    source.write_bytes(data)
    original = audit._snapshot_size

    def append_after_size(stream):
        size = original(stream)
        with source.open("ab") as writer:
            writer.write(row(200))
        return size

    monkeypatch.setattr(audit, "_snapshot_size", append_after_size)
    report = audit.analyze_csv(source, tmp_path / "analysis")
    assert report["source_prefix_bytes"] == len(data)
    assert report["source_prefix_sha256"] == hashlib.sha256(data).hexdigest()
    assert report["window_end_host_monotonic_s"] == 100
    assert report["selected_rows"] == 1


@pytest.mark.parametrize("bad", [
    row("nan"), row("inf"), row(100, node=13), row(100, values=("1.5", 0, 0, 0, 0, 0)),
    b"100,0,1,0\n", b"\n", row(100).replace(b",1000,", b",nan,"),
])
def test_malformed_complete_rows_fail_without_creating_outputs(tmp_path, bad):
    with pytest.raises(ValueError, match="source line"):
        run(tmp_path, HEADER + bad + row(1000))
    assert not (tmp_path / "analysis").exists()


def test_time_decrease_anywhere_in_source_fails(tmp_path):
    with pytest.raises(ValueError, match="decreased"):
        run(tmp_path, HEADER + row(100) + row(99) + row(1000))
    assert not (tmp_path / "analysis").exists()


def test_empty_or_invalid_headers_fail(tmp_path):
    for data in (b"", HEADER, HEADER.rstrip(), b"node_id,node_id\n" + row(100)):
        with pytest.raises(ValueError):
            run(tmp_path, data)
        assert not (tmp_path / "analysis").exists()


def test_exclusive_directory_never_overwrites_input_or_output(tmp_path):
    data = HEADER + row(100)
    report, output = run(tmp_path, data)
    originals = {path.name: path.read_bytes() for path in output.iterdir()}
    with pytest.raises(FileExistsError):
        audit.analyze_csv(tmp_path / "raw.csv", output)
    assert {path.name: path.read_bytes() for path in output.iterdir()} == originals
    assert (tmp_path / "raw.csv").read_bytes() == data


def test_input_and_output_bounds(tmp_path, monkeypatch):
    source = tmp_path / "too_big.csv"
    with source.open("wb") as stream:
        stream.truncate(audit.MAX_INPUT_BYTES + 1)
    with pytest.raises(ValueError, match="byte limit"):
        audit.analyze_csv(source, tmp_path / "analysis")
    monkeypatch.setattr(audit, "MAX_SELECTED_ROWS", 2)
    with pytest.raises(ValueError, match="row limit"):
        run(tmp_path, HEADER + row(100) * 3)
    assert not (tmp_path / "analysis").exists()


@pytest.mark.parametrize("duration", [0, -1, float("nan"), float("inf"), True])
def test_invalid_window(tmp_path, duration):
    with pytest.raises(ValueError, match="finite and positive"):
        run(tmp_path, HEADER + row(100), last_seconds=duration)


def test_cli(tmp_path, capsys):
    source = tmp_path / "raw.csv"
    source.write_bytes(HEADER + row(100))
    args = ["--raw-csv", str(source), "--last-seconds", "30", "--output-dir", str(tmp_path / "analysis")]
    assert audit.main(args) == 0
    assert json.loads(capsys.readouterr().out)["totals"]["outside_signed16_count"] == 2
    assert audit.main(args) == 2
    assert "File exists" in capsys.readouterr().err
