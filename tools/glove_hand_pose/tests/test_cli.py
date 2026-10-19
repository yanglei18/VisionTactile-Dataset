import json

from vt_glove_pose.cli import main
from vt_glove_pose.records import Sample
from vt_glove_pose.stream import RawRecorder


def test_demo_headless_reports_synthetic(tmp_path):
    output = tmp_path / "demo.json"
    assert main(["demo", "--headless", "--seconds", "0.1", "--output", str(output)]) == 0
    result = json.loads(output.read_text())
    assert result["source"] == "demo"
    assert result["processed_frames"] == 3
    assert not result["model"]["root_fixed"]
    assert result["model"]["root_translation_fixed"]
    assert result["model"]["root_orientation_valid"]
    assert all(not f["dip_measured"] for f in result["model"]["fingers"].values())
    assert main(["demo", "--headless", "--seconds", "0.1", "--output", str(output)]) == 2


def test_config_roundtrip_no_overwrite(tmp_path):
    path = tmp_path / "config.json"
    assert main(["config", "--output", str(path)]) == 0
    assert json.loads(path.read_text())["palm_node"] == 12
    assert main(["config", "--output", str(path)]) == 2


def test_inspect_and_replay_do_not_silently_calibrate(tmp_path):
    path = tmp_path / "raw.csv"
    with RawRecorder(path) as recorder:
        for frame in range(20):
            recorder.write([Sample(n, (0, 0, 0, 0, 0, 8197), frame/120, 100+frame/120) for n in range(1, 13)])
    output = tmp_path / "inspect.json"
    assert main(["inspect", str(path), "--output", str(output)]) == 0
    result = json.loads(output.read_text())
    assert result["total_records"] == 240
    assert abs(result["nodes"]["1"]["received_rate_hz"]-120) < 1e-8
    output = tmp_path / "replay.json"
    assert main(["replay", str(path), "--headless", "--output", str(output)]) == 0
    result = json.loads(output.read_text())
    assert result["source"] == "replay" and result["eof"]
    assert not result["calibration"]["neutral_ready"]
    assert all(not f["valid"] for f in result["model"]["fingers"].values())


def test_bad_csv_controlled_error(tmp_path):
    path = tmp_path / "wrong.csv"
    path.write_text("time,N1_GX_LSB\n0,3\n")
    assert main(["inspect", str(path)]) == 2
