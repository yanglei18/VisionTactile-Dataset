import csv
import json

import numpy as np
from scipy.spatial.transform import Rotation as R

from vt_fsglove12.cli import main
from vt_fsglove12.config import PARENTS, load_config


def test_layout_has_readable_output(capsys):
    assert main(["layout"]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["sensors_mapped"] == 11


def test_replay_actual_vqf_and_exclusive_outputs(tmp_path, capsys):
    source, destination, report = (tmp_path / name for name in ("raw.csv", "orientations.jsonl", "report.json"))
    with source.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["host_parse_monotonic_s", "host_parse_wall_s", "node_id",
                         "gx_lsb", "gy_lsb", "gz_lsb", "ax_lsb", "ay_lsb", "az_lsb"])
        for frame in range(12):
            for node in range(1, 13):
                writer.writerow([10 + frame / 120, 100 + frame / 120, node, 0, 0, 0, 0, 0, 8197])
    args = ["replay", "--raw", str(source), "--output", str(destination), "--report", str(report)]
    assert main(args) == 0
    payload = json.loads(report.read_text())
    assert payload["input_samples"] == 144
    assert payload["common_heading_established"] is False
    before = destination.read_bytes()
    assert main(args) == 2
    assert destination.read_bytes() == before
    assert "Refusing to replace" in capsys.readouterr().err


def test_missing_model_has_no_traceback_or_fake_rest(tmp_path, capsys):
    output = tmp_path / "rest.json"
    assert main(["export-rest", "--mano-assets", str(tmp_path), "--output", str(output)]) == 2
    assert not output.exists()
    assert "MANO" in capsys.readouterr().err


def test_output_report_collision_prevents_write(tmp_path):
    target = tmp_path / "out"
    assert main(["replay", "--raw", str(tmp_path / "absent.csv"), "--output", str(target),
                 "--report", str(target)]) == 2
    assert not target.exists()


def test_input_fit_cli_writes_separate_config_and_report(tmp_path, capsys, monkeypatch):
    from vt_fsglove12 import input_unit_fit
    config = load_config()
    config_path, labels_path, bias_path, report_path, candidate_path = (
        tmp_path / name for name in ("config.json", "windows.json", "bias.json", "report.json", "candidate.json"))
    config_path.write_text(json.dumps(config))
    labels = [{"start_s": t, "end_s": t + .5, "label": "stationary"} for t in (2., 5., 8.)]
    labels_path.write_text(json.dumps(labels))
    bias = {"start_s": 0., "end_s": 1., "label": "independent_stationary"}
    bias_path.write_text(json.dumps(bias))
    calls = []

    def fit(raw, actual, windows, **kwargs):
        calls.append((raw, actual, windows, kwargs))
        candidate = dict(actual, gyro_dps_per_lsb_by_node={"5": .0175})
        return {"recommended_nodes": ["5"], "nominal_supported_nodes": ["12"],
                "rejected_nodes": {}, "candidate_config": candidate, "hardware_accessed": False}

    monkeypatch.setattr(input_unit_fit, "fit_input_units", fit)
    args = ["fit-input-units", "--raw", str(tmp_path / "raw.csv"), "--config", str(config_path),
            "--windows", str(labels_path), "--bias-window", str(bias_path), "--report", str(report_path),
            "--output-config", str(candidate_path)]
    assert main(args) == 0
    assert len(calls) == 1
    assert calls[0][2] == labels
    assert calls[0][3]["bias_window"] == bias
    assert json.loads(candidate_path.read_text())["gyro_dps_per_lsb_by_node"] == {"5": .0175}
    assert json.loads(config_path.read_text()) == config
    assert not json.loads(report_path.read_text())["hardware_accessed"]
    assert main(args) == 2
    assert len(calls) == 1
    assert "Refusing to replace" in capsys.readouterr().err


def test_calibrate_solve_cli_contract_with_explicit_synthetic_data(tmp_path, capsys):
    # Generated orientation fixtures test file/interface wiring only, not a
    # real VQF recording, MANO asset, human calibration, or physical accuracy.
    config = load_config()
    rest = {"parents": list(PARENTS), "rest_global_xyzw": [[0, 0, 0, 1]] * 16,
            "provenance": {"kind": "synthetic_cli_test_not_mano"}}
    rest_path, manifest_path, source, profile, poses = (
        tmp_path / name for name in ("rest.json", "labels.json", "orientations.jsonl", "profile.json", "poses.jsonl"))
    rest_path.write_text(json.dumps(rest))
    roots = R.from_euler("XYZ", [[0, 0, 0], [.9, 0, 0], [0, 1.1, 0]])
    manifest = {"schema_version": 1, "time_origin": "first_frame", "poses": []}
    with source.open("w") as stream:
        for p, root in enumerate(roots):
            manifest["poses"].append({"start_s": p, "end_s": p + .9, "root_quat_xyzw": root.as_quat().tolist()})
            for sample in range(5):
                nodes = {}
                for node in config["node_to_slot"]:
                    mount = R.from_euler("XYZ", [.1 * int(node), .2, .3])
                    heading = R.from_euler("z", .07 * int(node))
                    sensor = heading.inv() * root * mount.inv()
                    nodes[node] = {"quat_xyzw": sensor.as_quat().tolist(), "age_s": 0,
                                   "sample_index": 5 * p + sample + 1, "epoch": 0}
                stream.write(json.dumps({"schema_version": 1, "frame_index": 5 * p + sample,
                                         "timestamp_s": 10 + p + sample * .1,
                                         "orientation_source": "vqf_6d",
                                         "orientation_frame": "independent_initial_heading",
                                         "nodes": nodes, "warnings": ["SYNTHETIC_TEST_ONLY"]}) + "\n")
    manifest_path.write_text(json.dumps(manifest))
    assert main(["calibrate", "--orientations", str(source), "--poses", str(manifest_path),
                 "--rest", str(rest_path), "--output", str(profile)]) == 0
    assert main(["solve", "--orientations", str(source), "--profile", str(profile),
                 "--rest", str(rest_path), "--output", str(poses)]) == 0
    results = [json.loads(line) for line in poses.read_text().splitlines()]
    assert len(results) == 15
    assert all(result["quality"] == "experimental" for result in results)
    for result in results:
        np.testing.assert_allclose(result["anatomical_euler_rad"], np.zeros((16, 3)), atol=1e-6)
        assert result["inferred_mask"][13]
        assert not result["upstream_fsglove_solver_executed"]
    actual_root = R.from_quat(results[-1]["root_quat_xyzw"])
    assert (actual_root.inv() * roots[-1]).magnitude() < 1e-7
