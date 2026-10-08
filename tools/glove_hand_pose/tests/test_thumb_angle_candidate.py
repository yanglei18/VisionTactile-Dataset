"""Pipeline contracts, not physical angle-accuracy certification."""
import hashlib
import json
import math

import numpy as np
import pytest

from vt_glove_pose.config import load_config
from vt_glove_pose.records import Sample
from vt_glove_pose.stream import RawRecorder
from vt_glove_pose import thumb_angle_candidate as candidate


@pytest.fixture
def trial(tmp_path):
    folder = tmp_path / "trial"
    folder.mkdir()
    config = load_config()
    config["sample_rate_hz"] = 20.
    metadata = dict(kind="vt_glove_pose.thumb_trial", schema_version=1, config=config,
                    start_s=100., node_ids=[5, 6])
    (folder / "metadata.json").write_text(json.dumps(metadata))
    (folder / "online.jsonl").write_text("")
    (folder / "report.json").write_text("{}")
    recorder = RawRecorder(folder / "raw.csv").open()
    try:
        for i in range(200):
            t = i / 20.
            # The distal unit starts bent during replay; never assume current=zero.
            angle = 0. if t < 3. else math.radians(30.)
            for node in (5, 6):
                theta = 0. if node == 5 else angle
                acc = np.array([.3, -.953939 * math.sin(theta), .953939 * math.cos(theta)])
                raw = (0, 0, 0) + tuple(int(round(v / config["accel_g_per_lsb"])) for v in acc)
                recorder.write([Sample(node, raw, 100. + t, 1000. + t)])
    finally:
        recorder.close()
    files = {}
    for path in folder.iterdir():
        raw = path.read_bytes()
        files[path.name] = dict(bytes=len(raw), sha256=hashlib.sha256(raw).hexdigest())
    (folder / "manifest.json").write_text(json.dumps(dict(status="completed", export_complete=True, files=files)))
    return folder


@pytest.fixture
def solver(monkeypatch):
    calls = []
    def fit(*arrays, sample_rate_hz):
        calls.append([a.copy() for a in arrays])
        return dict(candidate_status="inconsistent", axis_prox=[1., 0., 0.], axis_dist=[1., 0., 0.],
                    reason_codes=["synthetic_pipeline_test_not_real_fit"], usable_as_live_calibration=False)
    monkeypatch.setattr(candidate, "fit_joint_hinge_axes", fit)
    return calls


def test_export_keeps_rejection_and_uses_raw_runtime_without_terminal_zero(trial, solver):
    before = {p.name: p.read_bytes() for p in trial.iterdir()}
    profile, replay = candidate.build_candidate(trial, neutral=(0, 3), train=(3, 6), holdout=(6, 9))
    assert len(solver) == 1 and len(solver[0][0]) == 60
    assert profile["axis_fit"]["candidate_status"] == "inconsistent"
    assert not profile["certified_calibration"] and not profile["anatomical_accuracy_verified"]
    assert profile["protocol"]["holdout_used_for_fitting_or_selection"] is False
    assert replay["recorded_only"] and replay["reference_not_reset_at_holdout"]
    assert replay["forced_return_zero"] is False
    assert len(replay["series"]) == 140
    assert abs(replay["series"][0]["angle_deg"]) == pytest.approx(30., abs=.02)
    assert abs(replay["series"][-1]["angle_deg"]) > 29.
    assert replay["interval_summaries"]["holdout"]["samples"] == 60
    assert replay["summary"]["innovation_is_not_independent_angle_error"]
    assert {p.name: p.read_bytes() for p in trial.iterdir()} == before


def test_optional_sign_strategy_keeps_original_pair_indices(trial, solver, monkeypatch):
    calls = []
    def inspect_frames(*arrays, **kwargs):
        calls.append(kwargs)
        raise ValueError("end routing inspection without creating a candidate")
    monkeypatch.setattr(candidate, "build_hinge_frames", inspect_frames)
    with pytest.raises(ValueError, match="end routing inspection"):
        candidate.build_candidate(trial, neutral=(0, 3), train=(3, 6), holdout=(6, 9),
                                  frame_sign_strategy="informative_blocks")
    assert len(calls) == 1
    assert calls[0]["sign_strategy"] == "informative_blocks"
    np.testing.assert_array_equal(calls[0]["sample_indices"], np.arange(60, 120))


def test_unknown_sign_strategy_is_rejected_before_fitting(trial, solver):
    with pytest.raises(ValueError, match="frame_sign_strategy"):
        candidate.build_candidate(trial, frame_sign_strategy="relax")
    assert solver == []


@pytest.mark.parametrize("windows", [dict(neutral=(0, 4), train=(3, 6), holdout=(6, 9)),
                                     dict(neutral=(0, 3), train=(3, 7), holdout=(6, 9)),
                                     dict(neutral=(0, float("nan")), train=(3, 6), holdout=(6, 9)),
                                     dict(neutral=(0, 1), train=(3, 6), holdout=(6, 9))])
def test_overlapping_invalid_or_short_windows_do_not_fit(trial, solver, windows):
    with pytest.raises(ValueError):
        candidate.build_candidate(trial, **windows)
    assert solver == []


def test_hash_mismatch_does_not_fit_or_rewrite(trial, solver):
    (trial / "report.json").write_text("[]")  # Same size but changed content.
    with pytest.raises(ValueError, match="hash mismatch"):
        candidate.build_candidate(trial)
    assert solver == [] and (trial / "report.json").read_text() == "[]"


def test_cli_writes_new_directory_only(trial, solver, tmp_path):
    output = tmp_path / "candidate"
    args = [str(trial), "--output-dir", str(output), "--neutral", "0", "3", "--train", "3", "6",
            "--holdout", "6", "9"]
    assert candidate.main(args) == 0
    assert {p.name for p in output.iterdir()} == {"candidate.json", "replay.json"}
    before = (output / "candidate.json").read_bytes()
    with pytest.raises(SystemExit):
        candidate.main(args)
    assert (output / "candidate.json").read_bytes() == before and len(solver) == 1


def test_replay_bad_interval(trial, solver):
    profile, _ = candidate.build_candidate(trial, neutral=(0, 3), train=(3, 6), holdout=(6, 9))
    with pytest.raises(ValueError, match="replay stop"):
        candidate.replay_candidate(trial, profile, start_s=3., stop_s=2.)
    with pytest.raises(ValueError, match="no complete"):
        candidate.replay_candidate(trial, profile, start_s=100.)
