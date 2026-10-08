"""Offline CLI wiring tests for the real-core entry points.

Numerical correctness is covered by test_upstream_core.py. These tests replace
the UI, model loader and optional torch import explicitly; they never open a
serial port or claim that their synthetic contacts are physical measurements.
"""

from copy import deepcopy
import hashlib
import json
import os
import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

from vt_fsglove12 import cli, shape_calibration, upstream_core


PROVENANCE = {
    "upstream_project": "davidliyutong/fsglove",
    "upstream_commit": upstream_core.UPSTREAM_COMMIT,
    "sources": {
        "calibration": {"sha256": "a" * 64, "path": "/trusted/source/calibration.py"},
        "utils": {"sha256": "b" * 64, "path": "/trusted/source/utils.py"},
    },
    "numerical_ast_modified": False,
}
MODEL_PROVENANCE = {"hand": "right", "sha256": "c" * 64, "path": "/trusted/mano/models/MANO_RIGHT.pkl"}


class TensorStub:
    def __init__(self, data, dtype=None):
        self.data = np.asarray(data)
        self.dtype = dtype

    def tolist(self):
        return self.data.tolist()

    def __array__(self, dtype=None, copy=None):
        return np.asarray(self.data, dtype=dtype)


@pytest.fixture
def official_cli(monkeypatch):
    calls = SimpleNamespace(load=[], live=[], shape=[], threads=[])
    core = SimpleNamespace(provenance=deepcopy(PROVENANCE))

    def load(source, assets):
        calls.load.append((source, assets))
        return core

    def optimize_shape(poses, labels, hand_category, **kwargs):
        calls.shape.append((poses, labels, hand_category, kwargs))
        return TensorStub(np.linspace(-.2, .3, 10))

    def run_live(config, assets, **kwargs):
        calls.live.append((deepcopy(config), assets, kwargs))
        return 0

    core.optimize_shape = optimize_shape
    monkeypatch.setattr(upstream_core, "load_upstream_core", load)
    monkeypatch.setattr(shape_calibration, "model_provenance", lambda assets, hand="right": {**deepcopy(MODEL_PROVENANCE), "hand": hand})
    fake_ui = ModuleType("vt_fsglove12.live_ui")
    fake_ui.run_live = run_live
    monkeypatch.setitem(sys.modules, "vt_fsglove12.live_ui", fake_ui)
    fake_torch = ModuleType("torch")
    fake_torch.tensor = TensorStub
    fake_torch.float32 = "float32"
    fake_torch.long = "long"
    fake_torch.set_num_threads = calls.threads.append
    monkeypatch.setitem(sys.modules, "torch", fake_torch)
    monkeypatch.setattr(cli, "_solve", lambda _: pytest.fail("official commands must not call the legacy solver"))
    return calls, core


def common_args(command):
    return [command, "--fsglove-source", "/trusted/fsglove", "--mano-assets", "/trusted/mano"]


def test_upstream_live_tactile_display_socket_is_passed_only_when_requested(official_cli):
    calls, _core = official_cli
    assert cli.main(common_args("upstream-live") + ["--tactile-socket", "/private/tactile.sock"]) == 0
    assert calls.live[-1][2]["tactile_socket"] == "/private/tactile.sock"
    assert cli.main(common_args("upstream-live")) == 0
    assert "tactile_socket" not in calls.live[-1][2]


def test_left_live_and_tactile_layout_reach_gui_without_changing_right_default(official_cli):
    calls, _core = official_cli
    assert cli.main(common_args("upstream-live") + ["--hand", "left", "--tactile-layout", "/explicit/left.json"]) == 0
    assert calls.live[-1][0]["hand"] == "left"
    assert calls.live[-1][2]["tactile_layout"] == "/explicit/left.json"
    assert cli.main(common_args("upstream-live")) == 0
    assert calls.live[-1][0]["hand"] == "right"


def test_left_shape_cli_uses_left_optimizer_and_model_identity(official_cli, tmp_path):
    calls, _core = official_cli
    source, output = tmp_path / "contacts.json", tmp_path / "left-shape.json"
    source.write_text(json.dumps(synthetic_contacts()))
    assert cli.main(common_args("upstream-shape") + ["--hand", "left", "--contacts", str(source), "--output", str(output)]) == 0
    assert calls.shape[-1][2] == "left"
    assert json.loads(output.read_text())["mano_model"]["hand"] == "left"


def synthetic_contacts():
    return {"mano_pose_rotvec": np.zeros((4, 16, 3)).tolist(), "finger_category": [2, 3, 4, 5],
            "contact_labels_confirmed": True, "fixture_note": "SYNTHETIC TEST DATA, NOT HUMAN CONTACTS"}


def write_shape_profile(path, **changes):
    profile = {"algorithm": "upstream_fsglove_touch_shape", "shape_params": np.linspace(-.2, .3, 10).tolist(),
               "shape_calibrated": True, "upstream": deepcopy(PROVENANCE),
               "contact_labels_source": "user_explicit_not_inferred_from_imu"}
    profile.update(changes)
    path.write_text(json.dumps(profile))
    return profile


def test_upstream_check_reports_actual_loaded_core_without_ui(official_cli, capsys):
    calls, core = official_cli
    assert cli.main(common_args("upstream-check")) == 0
    report = json.loads(capsys.readouterr().out)
    assert report == {"status": "loaded", "upstream": core.provenance, "hardware_accessed": False,
                      "paper_complete_reproduction": False}
    assert calls.load == [("/trusted/fsglove", "/trusted/mano")]
    assert not calls.live and not calls.shape


def test_upstream_check_failure_is_reported_without_fallback(official_cli, monkeypatch, capsys):
    calls, _ = official_cli

    def fail(*args):
        raise upstream_core.UpstreamSourceError("source hash mismatch")

    monkeypatch.setattr(upstream_core, "load_upstream_core", fail)
    assert cli.main(common_args("upstream-check")) == 2
    assert "source hash mismatch" in capsys.readouterr().err
    assert not calls.live and not calls.shape


def test_upstream_live_can_open_same_gui_with_results_socket(official_cli, tmp_path):
    calls, _ = official_cli
    assert cli.main(common_args("upstream-live") + ["--ros-socket", str(tmp_path / "right.sock")]) == 0
    assert calls.live[0][2]["ros_socket"] == str(tmp_path / "right.sock")
    assert calls.live[0][2]["port"] == ""  # No automatic device connection.


def test_upstream_live_can_request_120hz_without_changing_saved_config(official_cli, tmp_path):
    from vt_fsglove12.config import load_config
    calls, _ = official_cli
    path = tmp_path / "config.json"
    original = json.dumps(load_config())
    path.write_text(original)
    assert cli.main(common_args("upstream-live") + [
        "--config", str(path), "--output-rate-hz", "120"]) == 0
    assert calls.live[0][0]["output_rate_hz"] == 120.
    assert calls.live[0][2]["port"] == ""
    assert path.read_text() == original


@pytest.mark.parametrize("rate", ["0", "-1", "nan", "inf", "121", "241"])
def test_upstream_live_rejects_invalid_or_above_input_output_rate(official_cli, rate, capsys):
    calls, _ = official_cli
    assert cli.main(common_args("upstream-live") + ["--output-rate-hz", rate]) == 2
    assert not calls.live
    assert "output_rate_hz" in capsys.readouterr().err


def test_upstream_live_passes_official_backend_and_no_shape_claim(official_cli, tmp_path):
    calls, core = official_cli
    output = tmp_path / "session"
    assert cli.main(common_args("upstream-live") + ["--port", "/dev/DO_NOT_OPEN", "--output-root", str(output),
                                                    "--replay-raw", "offline.csv", "--replay-speed", "2"]) == 0
    assert len(calls.live) == 1 and not calls.shape
    config, assets, kwargs = calls.live[0]
    assert config["pose_backend"] == "official_fsglove_sparse_adapter"
    assert config["upstream_provenance"] == core.provenance
    assert config.get("upstream_shape_calibrated", False) is False
    assert "upstream_shape_params" not in config
    assert assets == "/trusted/mano"
    assert kwargs == {"output_root": str(output), "port": "/dev/DO_NOT_OPEN", "replay_raw": "offline.csv",
                      "replay_speed": 2., "upstream_checkout": "/trusted/fsglove"}
    assert not output.exists(), "CLI stubs must not create a session or touch hardware"


def test_upstream_live_loads_matching_official_shape_profile(official_cli, tmp_path):
    calls, _ = official_cli
    path = tmp_path / "shape.json"
    profile = write_shape_profile(path)
    assert cli.main(common_args("upstream-live") + ["--shape-profile", str(path)]) == 0
    config = calls.live[0][0]
    assert config["upstream_shape_params"] == profile["shape_params"]
    assert config["upstream_shape_calibrated"] is True
    assert config["upstream_shape_provenance"]["profile_sha256"] == hashlib.sha256(path.read_bytes()).hexdigest()
    assert config["upstream_shape_provenance"]["upstream"] == profile["upstream"]
    assert config["upstream_shape_provenance"]["physical_accuracy_verified"] is False


@pytest.mark.parametrize("failure", ["wrong_algorithm", "unconfirmed", "wrong_commit", "wrong_source", "missing_source",
                                     "wrong_beta_length", "nonfinite_beta"])
def test_upstream_live_rejects_foreign_or_unconfirmed_shape(official_cli, tmp_path, capsys, failure):
    calls, _ = official_cli
    path = tmp_path / "shape.json"
    profile = write_shape_profile(path)
    if failure == "wrong_algorithm":
        profile["algorithm"] = "legacy_custom_shape"
    elif failure == "unconfirmed":
        profile["shape_calibrated"] = False
    elif failure == "wrong_commit":
        profile["upstream"]["upstream_commit"] = "0" * 40
    elif failure == "wrong_source":
        profile["upstream"]["sources"]["calibration"]["sha256"] = "0" * 64
    elif failure == "missing_source":
        del profile["upstream"]["sources"]["utils"]
    elif failure == "wrong_beta_length":
        profile["shape_params"] = [0.] * 9
    else:
        profile["shape_params"][0] = float("nan")
    path.write_text(json.dumps(profile))
    assert cli.main(common_args("upstream-live") + ["--shape-profile", str(path)]) == 2
    assert capsys.readouterr().err
    assert not calls.live


def test_upstream_shape_passes_tensors_to_original_optimizer_and_saves(official_cli, tmp_path, capsys):
    calls, core = official_cli
    source, output = tmp_path / "contacts.json", tmp_path / "shape.json"
    contacts = synthetic_contacts()
    source.write_text(json.dumps(contacts))
    args = common_args("upstream-shape") + ["--contacts", str(source), "--output", str(output),
                                            "--iterations", "13", "--seed", "17"]
    assert cli.main(args) == 0
    assert calls.load == [("/trusted/fsglove", "/trusted/mano")]
    assert len(calls.shape) == 1 and not calls.live
    poses, labels, hand, kwargs = calls.shape[0]
    np.testing.assert_array_equal(poses.data, contacts["mano_pose_rotvec"])
    np.testing.assert_array_equal(labels.data, contacts["finger_category"])
    assert poses.dtype == "float32" and labels.dtype == "long"
    assert hand == "right" and kwargs == {"iter_num": 13, "seed": 17}
    result = json.loads(output.read_text())
    assert result == json.loads(capsys.readouterr().out)
    assert result["algorithm"] == "upstream_fsglove_touch_shape"
    assert result["shape_calibrated"] is True
    assert result["upstream"] == core.provenance
    assert result["source_sha256"] == hashlib.sha256(source.read_bytes()).hexdigest()
    assert result["physical_accuracy_verified"] is False
    assert result["contact_labels_source"] == "user_explicit_not_inferred_from_imu"
    assert result["mano_model"] == MODEL_PROVENANCE
    np.testing.assert_allclose(result["shape_params"], np.linspace(-.2, .3, 10))
    assert result["iterations"] == 13 and result["seed"] == 17


@pytest.mark.parametrize("failure", ["missing_confirmation", "false_confirmation", "missing_category", "invalid_category",
                                     "nan_pose", "wrong_pose_shape", "wrong_label_count"])
def test_upstream_shape_rejects_invalid_contact_input_before_loading(official_cli, tmp_path, capsys, failure):
    calls, _ = official_cli
    contacts = synthetic_contacts()
    if failure == "missing_confirmation":
        del contacts["contact_labels_confirmed"]
    elif failure == "false_confirmation":
        contacts["contact_labels_confirmed"] = False
    elif failure == "missing_category":
        contacts["finger_category"] = [2, 3, 4, 4]
    elif failure == "invalid_category":
        contacts["finger_category"] = [2, 3, 4, 8]
    elif failure == "nan_pose":
        contacts["mano_pose_rotvec"][0][0][0] = float("nan")
    elif failure == "wrong_pose_shape":
        contacts["mano_pose_rotvec"] = np.zeros((4, 12, 3)).tolist()
    else:
        contacts["finger_category"] = [2, 3, 4, 5, 2]
    source, output = tmp_path / "contacts.json", tmp_path / "shape.json"
    source.write_text(json.dumps(contacts))
    assert cli.main(common_args("upstream-shape") + ["--contacts", str(source), "--output", str(output)]) == 2
    assert capsys.readouterr().err
    assert not calls.load and not calls.shape and not calls.live
    assert not output.exists()


def test_upstream_shape_refuses_to_overwrite_existing_output(official_cli, tmp_path, capsys):
    calls, _ = official_cli
    output = tmp_path / "shape.json"
    output.write_text("keep existing calibration")
    assert cli.main(common_args("upstream-shape") + ["--contacts", str(tmp_path / "unused.json"), "--output", str(output)]) == 2
    assert "Refusing to replace" in capsys.readouterr().err
    assert output.read_text() == "keep existing calibration"
    assert not calls.load and not calls.shape


@pytest.mark.parametrize("betas", [[0.] * 9, [float("nan")] + [0.] * 9])
def test_upstream_shape_rejects_invalid_optimizer_output(official_cli, tmp_path, capsys, betas):
    calls, core = official_cli
    core.optimize_shape = lambda *args, **kwargs: TensorStub(betas)
    source, output = tmp_path / "contacts.json", tmp_path / "shape.json"
    source.write_text(json.dumps(synthetic_contacts()))
    assert cli.main(common_args("upstream-shape") + ["--contacts", str(source), "--output", str(output)]) == 2
    assert "invalid MANO beta" in capsys.readouterr().err
    assert not output.exists() and not calls.live


def test_upstream_shape_source_paths_are_not_mistaken_for_algorithm_identity(official_cli, tmp_path):
    calls, _ = official_cli
    path = tmp_path / "relocated_shape.json"
    profile = write_shape_profile(path)
    for source in profile["upstream"]["sources"].values():
        source["path"] = "/another/machine/checkout/source.py"
    path.write_text(json.dumps(profile))
    assert cli.main(common_args("upstream-live") + ["--shape-profile", str(path)]) == 0
    assert calls.live[0][0]["upstream_shape_calibrated"] is True


def test_generated_cli_shape_can_be_loaded_by_gui_shared_validator(official_cli, tmp_path):
    _, core = official_cli
    source, output = tmp_path / "contacts.json", tmp_path / "shape.json"
    source.write_text(json.dumps(synthetic_contacts()))
    assert cli.main(common_args("upstream-shape") + ["--contacts", str(source), "--output", str(output)]) == 0
    document = json.loads(output.read_text())
    expected = {**core.provenance, "mano_model": {**MODEL_PROVENANCE, "path": "/relocated/MANO_RIGHT.pkl"}}
    assert shape_calibration.validate_shape_profile(document, expected) == document
    expected["mano_model"]["sha256"] = "d" * 64
    with pytest.raises(ValueError, match="different MANO model"):
        shape_calibration.validate_shape_profile(document, expected)


def test_upstream_shape_missing_model_fingerprint_fails_before_optimizer(official_cli, tmp_path, monkeypatch, capsys):
    calls, _ = official_cli
    source, output = tmp_path / "contacts.json", tmp_path / "shape.json"
    source.write_text(json.dumps(synthetic_contacts()))

    def absent(assets, hand="right"):
        raise FileNotFoundError("missing trusted MANO model")

    monkeypatch.setattr(shape_calibration, "model_provenance", absent)
    assert cli.main(common_args("upstream-shape") + ["--contacts", str(source), "--output", str(output)]) == 2
    assert "missing trusted MANO model" in capsys.readouterr().err
    assert not calls.shape and not output.exists()


@pytest.mark.skipif(not os.environ.get("FSGLOVE12_UPSTREAM_CHECKOUT") or not os.environ.get("FSGLOVE12_MANO_ASSETS"),
                    reason="Explicit trusted upstream checkout and MANO assets required")
def test_real_cli_generated_shape_profile_validates_for_gui(tmp_path, capsys):
    """Real optimizer/model smoke; synthetic poses are not physical measurements."""
    checkout, assets = os.environ["FSGLOVE12_UPSTREAM_CHECKOUT"], os.environ["FSGLOVE12_MANO_ASSETS"]
    source, output = tmp_path / "contacts.json", tmp_path / "shape.json"
    source.write_text(json.dumps(synthetic_contacts()))
    assert cli.main(["upstream-shape", "--fsglove-source", checkout, "--mano-assets", assets,
                     "--contacts", str(source), "--output", str(output), "--iterations", "2", "--seed", "7"]) == 0
    document = json.loads(output.read_text())
    assert json.loads(capsys.readouterr().out) == document
    expected = {**upstream_core.inspect_upstream_sources(checkout),
                "mano_model": shape_calibration.model_provenance(assets)}
    assert shape_calibration.validate_shape_profile(document, expected) == document
    assert document["mano_model"]["sha256"] == expected["mano_model"]["sha256"]
    assert document["iterations"] == 2 and document["seed"] == 7
    assert document["physical_accuracy_verified"] is False
