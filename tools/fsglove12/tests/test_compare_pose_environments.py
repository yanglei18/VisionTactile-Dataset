"""Pure comparison/scope tests; no model, child interpreter or hardware."""
from copy import deepcopy
import importlib.util
from pathlib import Path

import pytest

source = Path(__file__).parents[1] / "scripts" / "compare_pose_environments.py"
spec = importlib.util.spec_from_file_location("compare_pose_environments_test", source)
comparison = importlib.util.module_from_spec(spec)
spec.loader.exec_module(comparison)


def sample_result():
    rows = [dict(frame_index=i, mano_pose_rotvec=[[0., 0., 0.]] * 16,
                 anatomical_euler_rad=[[0., 0., 0.]] * 16, root_quat_xyzw=[0., 0., 0., 1.],
                 joints_m=[[0., 0., 0.]] * 21, status=dict(valid_mask=[True] * 16,
                     source_labels=["measured"] * 16, nested=dict(quality="valid", display_available=True)))
            for i in range(3)]
    return dict(frame_count=3, input_window_sha256="same-input", input=dict(unique_source_vectors=3), outputs=rows,
                calibration=dict(seed=0, state={"_base_quat": [[0., 0., 0., 1.]] * 16},
                    state_sha256="saved-digest", measured_slots=[0, 1], rejected_nodes={},
                    capture_kind="reconstructed_from_saved_three_pose_means_not_new_hardware_capture"))


def test_status_extraction_retains_all_nested_qualitative_flags():
    value = dict(angle_rad=.8, angles=[.1, .2], joint_quality=["valid", "degraded"],
                 joint_valid_mask=[True, False], diagnostic=dict(reason=None, failed=False,
                     residual_deg=1.5, slot=2, continuity=dict(reset_reason="epoch_changed")))
    status = comparison.semantic_status(value)
    assert "angle_rad" not in status and "angles" not in status
    assert status["joint_valid_mask"] == [True, False]
    assert status["diagnostic"] == dict(reason=None, failed=False, slot=2,
                                         continuity=dict(reset_reason="epoch_changed"))


def test_same_arrays_and_status_pass_while_tiny_float_error_is_quantified():
    baseline, candidate = sample_result(), sample_result()
    candidate["outputs"][0]["joints_m"] = deepcopy(candidate["outputs"][0]["joints_m"])
    candidate["outputs"][0]["joints_m"][0] = [1e-9, 0., 0.]
    report = comparison.compare_results(baseline, candidate)
    assert report["passed"]
    assert report["numerical"]["joints_m"]["max_absolute"] == pytest.approx(1e-9)
    assert report["qualitative_status"]["exactly_equal"]


@pytest.mark.parametrize("change", ["mask", "quality", "labels", "joints", "calibration"])
def test_numerical_and_semantic_regressions_fail(change):
    baseline, candidate = sample_result(), sample_result()
    row = candidate["outputs"][1]
    if change == "mask":
        row["status"]["valid_mask"][2] = False
    elif change == "quality":
        row["status"]["nested"]["quality"] = "unavailable"
    elif change == "labels":
        row["status"]["source_labels"][2] = "inferred"
    elif change == "joints":
        row["joints_m"][0] = [1e-3, 0., 0.]
    else:
        candidate["calibration"]["state"]["_base_quat"][0] = [.1, 0., 0., 1.]
    assert not comparison.compare_results(baseline, candidate)["passed"]


def test_candidate_path_changes_only_child_environment(monkeypatch, tmp_path):
    monkeypatch.setenv("PYTHONPATH", "/existing/path")
    environment = comparison.environment_for_child(tmp_path)
    assert environment["PYTHONPATH"].startswith(str(tmp_path) + ":/existing/path")
    assert comparison.os.environ["PYTHONPATH"] == "/existing/path"
    assert all(environment[key] == "1" for key in comparison.THREAD_VARIABLES)


def test_reconstructed_calibration_uses_saved_means_epochs_and_unique_indices():
    profile = dict(session_id="session", nodes={"12": dict(slot=0, epoch=7)},
                   calibration_inputs={key: [[0., 0., 0., 1.]] * 16
                                       for key in ("base_pose", "z_rot_pose_1", "z_rot_pose_2")})
    original = deepcopy(profile)
    captures = comparison.calibration_captures(profile)
    assert len({row["nodes"]["12"]["sample_index"] for frames in captures.values() for row in frames}) == 9
    assert all(row["nodes"]["12"]["epoch"] == 7 for frames in captures.values() for row in frames)
    assert profile == original
    captures["base_pose"][0]["nodes"]["12"]["quat_xyzw"][0] = .2
    assert profile == original
