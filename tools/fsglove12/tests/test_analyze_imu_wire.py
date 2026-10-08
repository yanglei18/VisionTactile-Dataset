"""Integer and byte-evidence regressions; never open hardware."""
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import struct

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "analyze_imu_wire.py"
SPEC = importlib.util.spec_from_file_location("analyze_imu_wire_script", SCRIPT)
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


def frame(first=1, values=(-32768, 32767, -32760, 32760, -1, 0), status=253):
    return b"".join(struct.pack("<BB6h", node, status, *values) for node in range(first, first + 6))


def rows(directory, name):
    with (directory / name).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def run(tmp_path, data, **kwargs):
    wire = tmp_path / "raw.wire.bin"
    wire.write_bytes(data)
    output = tmp_path / "analysis"
    return audit.analyze_wire(wire, output, **kwargs), output


def test_all_bit_patterns_use_python_integers_and_exact_bounds():
    for unsigned in range(65536):
        signed, decoded_unsigned = audit.decode_word(unsigned % 256, unsigned // 256)
        assert type(signed) is int and type(decoded_unsigned) is int
        assert decoded_unsigned == unsigned
        assert -32768 <= signed <= 32767
        assert signed == unsigned if unsigned < 32768 else signed == unsigned - 65536
    assert audit.decode_word(0, 128) == (-32768, 32768)
    assert abs(audit.decode_word(0, 128)[0]) == 32768
    assert audit.decode_word(255, 127) == (32767, 32767)
    for values in ((0.0, 128), (True, 0), (-1, 0), (0, 256)):
        with pytest.raises(ValueError):
            audit.decode_word(*values)


def test_all_twelve_nodes_every_axis_and_exact_byte_evidence(tmp_path):
    data = frame() + frame(7)
    report, output = run(tmp_path, data)
    assert report["frames_decoded"] == 2 and report["samples_decoded"] == 12
    assert report["nodes_missing"] == []
    assert report["bank_frames"] == {"1-6": 1, "7-12": 1}
    assert report["signed_observed_min"] == -32768 and report["signed_observed_max"] == 32767
    assert report["unsigned_observed_max"] == 65535
    assert report["totals"]["outside_signed16_count"] == 0
    assert report["totals"]["exact_minus_32768_count"] == 12
    assert report["totals"]["exact_plus_32767_count"] == 12
    assert report["totals"]["near_boundary_abs_ge_32760_count"] == 48
    assert report["theoretical_bounds"] == dict(signed_min=-32768, signed_max=32767, unsigned_min=0, unsigned_max=65535)
    assert report["wire_sha256"] == hashlib.sha256(data).hexdigest()
    assert len(rows(output, "summary.csv")) == 72
    samples = rows(output, "samples.csv")
    assert len(samples) == 12
    assert samples[0]["gx_signed"] == "-32768" and samples[0]["gx_unsigned"] == "32768"
    assert samples[6]["block_wire_offset"] == "84" and samples[6]["node_id"] == "7"
    assert all(row["status"] == "253" and row["node_sequence"] == "1" for row in samples)
    events = rows(output, "boundary_events.csv")
    assert len(events) == 48 and events[0]["wire_bytes_hex_le"] == "0080"
    for event in events:
        offset = int(event["axis_wire_offset"])
        assert data[offset:offset + 2].hex() == event["wire_bytes_hex_le"]
        assert struct.unpack("<h", data[offset:offset + 2])[0] == int(event["signed_value"])
    assert "timestamp" not in ",".join(samples[0])
    assert not report["timestamps_available"] and not report["hardware_accessed"]
    assert json.loads((output / "report.json").read_text()) == report


def test_near_runs_use_per_node_order_reset_and_wrap_is_only_suspicion(tmp_path):
    extreme = (32767, -32768, 0, 0, 0, 0)
    opposite = (-32768, 32767, 0, 0, 0, 0)
    data = frame(values=extreme) + frame(7) + frame(values=opposite) + frame(values=extreme)
    data += frame(values=(0, 0, 0, 0, 0, 0)) + frame(values=extreme)
    report, output = run(tmp_path, data)
    gx = next(row for row in report["axis_summary"] if row["node_id"] == 1 and row["axis"] == "gx")
    assert gx["longest_consecutive_near_boundary_run"] == 3
    assert gx["near_boundary_abs_ge_32760_count"] == 4
    assert gx["samples"] == 5
    assert report["wrap_candidate_count"] == 24
    wrap = rows(output, "wrap_candidates.csv")[0]
    assert wrap["absolute_raw_jump"] == "65535"
    assert wrap["previous_node_sequence"] == "1" and wrap["node_sequence"] == "2"
    assert wrap["interpretation"] == "suspicion_only_not_proof_of_wrap_or_overflow"


def test_resynchronization_preserves_absolute_offsets_at_every_chunk_split():
    malformed = bytearray(frame())
    malformed[14] = 99
    prefix = b"noise\xff" + bytes(malformed)
    data = prefix + frame(7) + frame() + frame()[:27]
    for split in range(len(data) + 1):
        accounting = {}
        decoded = list(audit.iter_frames([data[:split], data[split:]], accounting))
        assert decoded == [(len(prefix), frame(7)), (len(prefix) + 84, frame())]
        assert accounting["bytes_discarded_during_framing"] == len(prefix)
        assert accounting["unparsed_tail_bytes"] == 27
        assert accounting["unparsed_tail_wire_offset"] == len(prefix) + 168


def test_all_saved_bytes_parsed_even_when_probe_stopped_at_sample_limit(tmp_path):
    data = frame() * 60 + frame(7) * 60 + b"tail"
    probe = tmp_path / "probe.json"
    probe.write_text(json.dumps(dict(wire_sha256=hashlib.sha256(data).hexdigest(), wire_bytes_saved=len(data),
                                    status="sample_limit", samples_decoded=6, frames_decoded=1,
                                    unparsed_tail_bytes=len(data) - 84)))
    report, output = run(tmp_path, data, probe_report=probe)
    assert report["samples_decoded"] == 720
    assert report["probe_crosscheck"]["probe_samples_decoded"] == 6
    assert report["probe_crosscheck"]["sha256_match"] is True
    assert not report["capture_parser_sample_limit_applied"]
    assert report["all_saved_bytes_examined"] and report["byte_accounting_valid"]
    assert report["unparsed_tail_bytes"] == 4
    assert len(rows(output, "samples.csv")) == 720


@pytest.mark.parametrize("data", [b"", b"short", frame(7)])
def test_empty_tail_and_missing_nodes_are_explicit(tmp_path, data):
    report, output = run(tmp_path, data)
    assert report["byte_accounting_valid"]
    assert report["nodes_missing"] == (list(range(1, 13)) if len(data) < 84 else list(range(1, 7)))
    assert len(rows(output, "summary.csv")) == 72
    missing = report["axis_summary"][0]
    assert missing["samples"] == 0 and missing["signed_min"] is None and missing["signed_max"] is None


@pytest.mark.parametrize("probe_values", [
    {"wire_sha256": "0" * 64}, {"wire_sha256": "invalid"}, {},
    {"wire_sha256": hashlib.sha256(frame()).hexdigest(), "wire_bytes_saved": 0},
])
def test_hash_and_size_mismatches_do_not_create_outputs(tmp_path, probe_values):
    wire = tmp_path / "wire.bin"
    wire.write_bytes(frame())
    probe = tmp_path / "probe.json"
    probe.write_text(json.dumps(probe_values))
    output = tmp_path / "analysis"
    with pytest.raises(ValueError):
        audit.analyze_wire(wire, output, probe_report=probe)
    assert not output.exists()
    assert wire.read_bytes() == frame()


def test_existing_output_is_never_overwritten_and_default_directory_is_exclusive(tmp_path):
    wire = tmp_path / "wire.bin"
    wire.write_bytes(frame())
    report = audit.analyze_wire(wire)
    output = tmp_path / "wire.bin.analysis"
    assert report["output_dir"] == str(output)
    original = {path.name: path.read_bytes() for path in output.iterdir()}
    with pytest.raises(FileExistsError):
        audit.analyze_wire(wire)
    assert {path.name: path.read_bytes() for path in output.iterdir()} == original
    assert wire.read_bytes() == frame()


def test_input_bound_and_regular_file_only(tmp_path):
    wire = tmp_path / "big.bin"
    with wire.open("wb") as stream:
        stream.truncate(audit.MAX_BYTES + 1)
    with pytest.raises(ValueError, match="limit"):
        audit.analyze_wire(wire, tmp_path / "analysis")
    with pytest.raises(ValueError, match="regular file"):
        audit.analyze_wire(Path("/dev/null"), tmp_path / "analysis")
    assert not (tmp_path / "analysis").exists()


def test_cli_has_no_hardware_flags_and_outputs_valid_files(tmp_path, capsys):
    wire = tmp_path / "wire.bin"
    wire.write_bytes(frame())
    assert audit.main(["--wire", str(wire), "--output-dir", str(tmp_path / "out")]) == 0
    assert json.loads(capsys.readouterr().out)["samples_decoded"] == 6
    assert audit.main(["--wire", str(wire), "--output-dir", str(tmp_path / "out")]) == 2
    assert "File exists" in capsys.readouterr().err
