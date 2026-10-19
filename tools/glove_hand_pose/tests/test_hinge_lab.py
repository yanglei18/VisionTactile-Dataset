import hashlib
import json

import numpy as np
import pytest

from vt_glove_pose.config import load_config
from vt_glove_pose.hinge_lab import build_hinge_report, load_pair_archive, run_hinge_experiment


def make_archive(tmp_path, **updates):
    raw = np.zeros((480, 6), dtype=np.int16)
    raw[:, 5] = 8197
    data = dict(sample_rate_hz=np.array(120.), node_ids=np.array([5, 6]),
                a1_prox_gyro_dps=np.zeros((480, 3)), a1_dist_gyro_dps=np.zeros((480, 3)),
                a1_raw_prox=raw, a1_raw_dist=raw,
                a1_trim_mask=np.zeros(480, dtype=bool))
    data.update(updates)
    path = tmp_path / "capture.npz"
    np.savez(path, **data)
    return path


def test_archive_keeps_whole_capture_and_tracks_assumptions(tmp_path):
    path = make_archive(tmp_path)
    capture = load_pair_archive(path, prefix="a1_")
    assert capture["prox_gyro_dps"].shape == (480, 3)
    assert capture["near_limit_mask"].shape == (480, 2)
    info = capture["metadata"]
    assert not info["hardware_synchronization_verified"]
    assert not info["units_verified"]
    assert info["source_sha256"] == hashlib.sha256(path.read_bytes()).hexdigest()
    assert info["stop_reason"] == "capture_complete"


def test_archive_bounds_prefix_and_node_identity(tmp_path):
    path = make_archive(tmp_path)
    capture = load_pair_archive(path, prefix="a1_", max_pairs=200)
    assert len(capture["prox_gyro_dps"]) == 200
    assert capture["metadata"]["stop_reason"] == "max_pairs"
    with pytest.raises(ValueError, match="missing fields"):
        load_pair_archive(path)
    with pytest.raises(ValueError, match="node_ids"):
        load_pair_archive(path, prefix="a1_", proximal=3, distal=4)
    with pytest.raises(ValueError, match="max_pairs"):
        load_pair_archive(path, prefix="a1_", max_pairs=True)


def test_archive_raw_limit_is_not_hidden_by_debiased_gyro(tmp_path, monkeypatch):
    raw = np.zeros((480, 6), dtype=np.int16)
    raw[100, 0] = -32768
    path = make_archive(tmp_path, a1_raw_dist=raw)
    import vt_glove_pose.hinge_axes as axes
    monkeypatch.setattr(axes, "fit_hinge_axes", lambda *a, **k: pytest.fail("clipped data was fitted"))
    report = run_hinge_experiment(path, load_config(), archive_prefix="a1_")
    assert report["raw_near_limit_rows"] == 1
    assert report["result"]["candidate_status"] == "input_near_limit"
    assert report["offline_only"]
    assert not report["live_calibration_changed"]
    assert report["validation"]["production_acceptance"] == "not_evaluated"
    json.dumps(report, allow_nan=False)


def test_missing_raw_is_explicit_not_a_clean_saturation_check(tmp_path):
    path = tmp_path / "pair.npz"
    np.savez(path, sample_rate_hz=120., node_ids=[5, 6],
             prox_gyro_dps=np.zeros((240, 3)), dist_gyro_dps=np.zeros((240, 3)))
    capture = load_pair_archive(path)
    assert capture["near_limit_mask"] is None
    assert not capture["metadata"]["saturation_assessed_from_raw"]


def test_explicit_bias_is_applied_once_without_mutating_input(monkeypatch):
    import vt_glove_pose.hinge_axes as axes
    captured = []
    def candidate(p, d, **kwargs):
        captured.append((p.copy(), d.copy(), kwargs))
        return {"candidate_status": "candidate", "usable_for_live": False}
    monkeypatch.setattr(axes, "fit_hinge_axes", candidate)
    gp, gd = np.ones((240, 3)), np.ones((240, 3)) * 2
    report = build_hinge_report(
        {"prox_gyro_dps": gp, "dist_gyro_dps": gd, "metadata": {}},
        sample_rate_hz=120., proximal_bias_dps=[1, 0, 0], distal_bias_dps=[0, 2, 0])
    np.testing.assert_array_equal(captured[0][0], np.tile([0, 1, 1], (240, 1)))
    np.testing.assert_array_equal(captured[0][1], np.tile([2, 0, 2], (240, 1)))
    assert np.all(gp == 1) and np.all(gd == 2)
    assert report["raw_near_limit_rows"] is None


@pytest.mark.parametrize("options", [
    {"proximal_bias_dps": [0, 0, 0]}, {"byte_limit": 1000},
    {"start_s": 1}, {"duration_s": 2},
])
def test_npz_does_not_silently_ignore_csv_options(tmp_path, options):
    path = make_archive(tmp_path)
    with pytest.raises(ValueError):
        run_hinge_experiment(path, load_config(), archive_prefix="a1_", **options)


def test_cli_creates_report_not_live_calibration(tmp_path, monkeypatch):
    import vt_glove_pose.hinge_axes as axes
    from vt_glove_pose.cli import main
    monkeypatch.setattr(axes, "fit_hinge_axes", lambda *a, **k: {"candidate_status": "insufficient_excitation"})
    source = make_archive(tmp_path)
    before = source.read_bytes()
    output = tmp_path / "report.json"
    args = ["fit-hinge", str(source), "--archive-prefix", "a1_", "--output", str(output)]
    assert main(args) == 0
    report = json.loads(output.read_text())
    assert report["result"]["candidate_status"] == "insufficient_excitation"
    assert "mounts" not in report and report["offline_only"]
    assert source.read_bytes() == before
    first = output.read_bytes()
    assert main(args) == 2
    assert output.read_bytes() == first


def test_empty_capture_produces_actionable_report(monkeypatch):
    import vt_glove_pose.hinge_axes as axes
    monkeypatch.setattr(axes, "fit_hinge_axes", lambda *a, **k: pytest.fail("empty capture"))
    report = build_hinge_report({"prox_gyro_dps": np.empty((0, 3)),
                                 "dist_gyro_dps": np.empty((0, 3)), "metadata": {}},
                                sample_rate_hz=120.)
    assert report["result"]["candidate_status"] == "insufficient_capture"


def test_archive_rejects_nonfinite_gyro(tmp_path):
    with pytest.raises(ValueError, match="nonfinite"):
        load_pair_archive(make_archive(tmp_path, a1_prox_gyro_dps=np.full((480, 3), np.nan)), prefix="a1_")


def test_cli_invalid_archive_has_clear_error_without_traceback(tmp_path, capsys):
    from vt_glove_pose.cli import main
    path = tmp_path / "invalid.npz"
    path.write_bytes(b"not an archive")
    assert main(["fit-hinge", str(path)]) == 2
    message = capsys.readouterr().err
    assert "not a valid NPZ" in message
    assert "Traceback" not in message
