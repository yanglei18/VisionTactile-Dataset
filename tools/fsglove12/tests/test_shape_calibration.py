"""Offline shape fit wiring; synthetic poses are not physical contact evidence."""
from contextlib import nullcontext
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from vt_fsglove12 import shape_calibration as shape
from vt_fsglove12 import upstream_core


PROVENANCE = dict(upstream_project="davidliyutong/fsglove", upstream_commit=upstream_core.UPSTREAM_COMMIT,
                  numerical_ast_modified=False, sources={"utils": {"sha256": "a" * 64}})


def contacts():
    return dict(mano_pose_rotvec=np.zeros((4, 16, 3)).tolist(), finger_category=[2, 3, 4, 5],
                contact_labels_confirmed=True, fixture_note="SYNTHETIC SOFTWARE TEST, NOT HUMAN CONTACTS")


def profile():
    return dict(algorithm=shape.ALGORITHM, shape_calibrated=True, shape_params=[0.] * 10,
                upstream=deepcopy(PROVENANCE), contact_labels_source=shape.CONTACT_SOURCE,
                physical_accuracy_verified=False)


class Tensor:
    def __init__(self, data, **kwargs):
        self.data = np.asarray(data)

    def __array__(self, dtype=None, copy=None):
        return np.asarray(self.data, dtype=dtype)

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.data


@pytest.fixture
def mocked_core(tmp_path, monkeypatch):
    assets = tmp_path / "assets"
    (assets / "models").mkdir(parents=True)
    (assets / "models" / "MANO_RIGHT.pkl").write_bytes(b"explicit mock asset, never unpickled")
    calls = SimpleNamespace(load=[], shape=[], model=[], threads=[])

    class Model:
        def __init__(self, **kwargs):
            calls.model.append(kwargs)

        def eval(self):
            return self

        def __call__(self, poses, betas):
            poses, betas = np.asarray(poses), np.asarray(betas)
            result = np.zeros((len(poses), 21, 3))
            for category in (2, 3, 4, 5):
                result[:, category * 4, 0] = .01 * category - .005 * betas[:, 0]
            return SimpleNamespace(joints=Tensor(result))

    def optimize(poses, labels, hand, **kwargs):
        calls.shape.append((np.asarray(poses).copy(), np.asarray(labels).copy(), hand, kwargs))
        return Tensor(np.ones(10))

    core = SimpleNamespace(provenance=deepcopy(PROVENANCE), optimize_shape=optimize,
        utils=SimpleNamespace(ManoLayer=Model), torch=SimpleNamespace(tensor=Tensor,
        float32="float32", long="long", set_num_threads=calls.threads.append, no_grad=nullcontext))

    def load(checkout, mano_assets):
        calls.load.append((checkout, mano_assets))
        return core

    monkeypatch.setattr(upstream_core, "load_upstream_core", load)
    return core, calls, assets


def test_profile_validation_copies_and_checks_model_by_bytes_not_paths():
    document = profile()
    model = dict(hand="right", sha256="b" * 64, path="/old/model.pkl")
    document["mano_model"] = model
    expected = {**PROVENANCE, "mano_model": {**model, "path": "/new/model.pkl"}}
    document["upstream"]["sources"]["utils"]["path"] = "/old/source.py"
    result = shape.validate_shape_profile(document, expected)
    result["shape_params"][0] = 2.
    assert document["shape_params"][0] == 0.
    assert result["mano_model"] is not document["mano_model"]


def test_left_profile_hand_must_match_even_when_model_digest_is_identical():
    document = {**profile(), "mano_model": dict(hand="left", sha256="b" * 64)}
    expected = {**PROVENANCE, "mano_model": dict(hand="left", sha256="b" * 64)}
    assert shape.validate_shape_profile(document, expected)["mano_model"]["hand"] == "left"
    expected["mano_model"]["hand"] = "right"
    with pytest.raises(ValueError, match="different MANO model"):
        shape.validate_shape_profile(document, expected)


def test_left_shape_optimization_and_contact_metrics_use_left_model(tmp_path, mocked_core):
    _core, calls, assets = mocked_core
    (assets / "models" / "MANO_LEFT.pkl").write_bytes(b"mock left asset, never unpickled")
    model = shape.model_provenance(assets, hand="left")
    sample = {**contacts(), "mano_model": model}
    result = shape.optimize_contacts(sample, "checkout", assets, tmp_path / "left-output")
    assert result["status"] == "completed"
    assert calls.shape[0][2] == "left" and calls.model[0]["side"] == "left"
    assert result["shape_profile"]["mano_model"] == model
    with pytest.raises(ValueError, match="hand"):
        shape.optimize_contacts(sample, "checkout", assets, tmp_path / "wrong-output", hand="right")
    assert not (tmp_path / "wrong-output").exists()


@pytest.mark.parametrize("mutation", [
    lambda p: p.update(algorithm="legacy"),
    lambda p: p.update(shape_calibrated=1),
    lambda p: p.update(shape_params=[0.] * 9),
    lambda p: p.update(shape_params=[float("nan")] * 10),
    lambda p: p.update(physical_accuracy_verified=True),
    lambda p: p.update(contact_labels_source="inferred_from_imu"),
    lambda p: p["upstream"].update(upstream_commit="foreign"),
    lambda p: p["upstream"].update(numerical_ast_modified=True),
    lambda p: p["upstream"]["sources"].update(utils={"sha256": "f" * 64}),
    lambda p: p.update(mano_model=dict(hand="both", sha256="b" * 64)),
    lambda p: p.update(source_sha256="invalid"),
])
def test_profile_rejects_invalid_algorithm_numbers_labels_and_provenance(mutation):
    document = profile()
    mutation(document)
    with pytest.raises(ValueError):
        shape.validate_shape_profile(document, PROVENANCE)


def test_profile_requires_model_identity_when_expected():
    expected = {**PROVENANCE, "mano_model": dict(hand="right", sha256="b" * 64)}
    with pytest.raises(ValueError, match="no MANO model fingerprint"):
        shape.validate_shape_profile(profile(), expected)
    document = {**profile(), "mano_model": dict(hand="right", sha256="c" * 64)}
    with pytest.raises(ValueError, match="different MANO model"):
        shape.validate_shape_profile(document, expected)


@pytest.mark.parametrize("mutation", [
    lambda c: c.update(contact_labels_confirmed=1),
    lambda c: c.update(contact_labels_source="inferred_from_imu"),
    lambda c: c.update(finger_category=[2, 3, 4, 4]),
    lambda c: c.update(finger_category=[2., 3., 4., 5.]),
    lambda c: c.update(finger_category=[2, 3, 4, 6]),
    lambda c: c.update(mano_pose_rotvec=np.zeros((4, 15, 3)).tolist()),
    lambda c: c["mano_pose_rotvec"][0][0].__setitem__(0, float("inf")),
    lambda c: c.update(before_shape_params=[0.] * 9),
    lambda c: c.update(mano_pose_rotvec=np.zeros((481, 16, 3)).tolist(), finger_category=[2] * 478 + [3, 4, 5]),
    lambda c: c.update(mano_pose_rotvec=np.zeros((124, 16, 3)).tolist(), finger_category=[2] * 121 + [3, 4, 5]),
])
def test_invalid_contacts_fail_before_optional_model_load(tmp_path, mocked_core, mutation):
    _, calls, assets = mocked_core
    document = contacts()
    mutation(document)
    with pytest.raises(ValueError):
        shape.optimize_contacts(document, "checkout", assets, tmp_path / "output")
    assert not calls.load and not (tmp_path / "output").exists()


@pytest.mark.parametrize("kwargs", [{"iterations": 0}, {"iterations": 2001}, {"iterations": True},
                                   {"seed": -1}, {"seed": 2**32}, {"seed": False}])
def test_iteration_seed_limits(tmp_path, mocked_core, kwargs):
    _, calls, assets = mocked_core
    with pytest.raises(ValueError):
        shape.optimize_contacts(contacts(), "checkout", assets, tmp_path / "output", **kwargs)
    assert not calls.load


def test_original_optimizer_exact_inputs_and_real_geometry_contract(tmp_path, mocked_core):
    _, calls, assets = mocked_core
    document = contacts()
    document["before_shape_params"] = [.2] * 10
    document["mano_pose_rotvec"][2][6][1] = .35
    stages = []
    result = shape.optimize_contacts(document, "checkout", assets, tmp_path / "output", progress=stages.append)
    assert result["status"] == "completed" and result["candidate_available"]
    assert calls.threads == [1] and len(calls.shape) == 1
    poses, labels, hand, kwargs = calls.shape[0]
    np.testing.assert_array_equal(poses, document["mano_pose_rotvec"])
    np.testing.assert_array_equal(labels, document["finger_category"])
    assert hand == "right" and kwargs == dict(iter_num=500, seed=0)
    assert calls.model == [dict(rot_mode="axisang", use_pca=False, side="right", center_idx=0, flat_hand_mean=True)]
    assert result["metrics"]["per_pair"]["index"]["before"]["mean_gap_mm"] == pytest.approx(19.)
    assert result["metrics"]["per_pair"]["index"]["after"]["mean_gap_mm"] == pytest.approx(15.)
    assert result["preview_samples"]["ring"]["mano_pose_rotvec"] == document["mano_pose_rotvec"][2]
    assert not result["physical_accuracy_verified"] and not result["hardware_accessed"]
    saved = json.loads(Path(result["shape_path"]).read_text())
    assert saved == result["shape_profile"]
    assert saved["source_sha256"] == hashlib.sha256(Path(result["contacts_path"]).read_bytes()).hexdigest()
    assert saved["mano_model"] == shape.model_provenance(assets)
    assert json.loads(Path(result["report_path"]).read_text()) == result
    assert [s["stage"] for s in stages] == ["loading_model", "optimizing", "evaluating_contacts", "completed"]
    assert not list((tmp_path / "output").glob("*.partial"))


def test_cancel_before_start_has_no_output_or_import(tmp_path, mocked_core):
    _, calls, assets = mocked_core
    result = shape.optimize_contacts(contacts(), "checkout", assets, tmp_path / "output", cancelled=lambda: True)
    assert result["status"] == "cancelled" and not result["candidate_available"]
    assert not calls.load and not (tmp_path / "output").exists()


@pytest.mark.parametrize("at_stage", ["loading_model", "optimizing", "evaluating_contacts"])
def test_cooperative_cancellation_at_stage_boundaries(tmp_path, mocked_core, at_stage):
    _, calls, assets = mocked_core
    cancelled = False

    def progress(snapshot):
        nonlocal cancelled
        cancelled = snapshot["stage"] == at_stage

    result = shape.optimize_contacts(contacts(), "checkout", assets, tmp_path / "output",
                                      progress=progress, cancelled=lambda: cancelled)
    assert result["status"] == "cancelled" and not result["candidate_available"]
    if at_stage == "loading_model":
        assert not calls.load
    if at_stage in ("loading_model", "optimizing"):
        assert not calls.shape
    assert not calls.model and not (tmp_path / "output" / "shape.json").exists()


def test_cancel_during_original_optimizer_discards_fit(tmp_path, mocked_core):
    core, calls, assets = mocked_core
    cancelled = False
    original = core.optimize_shape

    def optimize(*args, **kwargs):
        nonlocal cancelled
        result = original(*args, **kwargs)
        cancelled = True
        return result

    core.optimize_shape = optimize
    result = shape.optimize_contacts(contacts(), "checkout", assets, tmp_path / "output", cancelled=lambda: cancelled)
    assert calls.shape and not calls.model
    assert result["status"] == "cancelled" and result["shape_profile"] is None
    assert (tmp_path / "output" / "contacts.json").exists()
    assert not (tmp_path / "output" / "shape.json").exists()
    assert json.loads(Path(result["report_path"]).read_text())["status"] == "cancelled"


def test_existing_directory_and_json_are_never_replaced(tmp_path, mocked_core):
    _, calls, assets = mocked_core
    directory = tmp_path / "existing"
    directory.mkdir()
    (directory / "shape.json").write_bytes(b"untouched prior result")
    with pytest.raises(FileExistsError):
        shape.optimize_contacts(contacts(), "checkout", assets, directory)
    assert not calls.load and (directory / "shape.json").read_bytes() == b"untouched prior result"
    with pytest.raises(FileExistsError):
        shape._write_json(directory / "shape.json", {"new": True})
    assert (directory / "shape.json").read_bytes() == b"untouched prior result"
    assert list(directory.iterdir()) == [directory / "shape.json"]


@pytest.mark.parametrize("betas", [np.zeros(9), np.full(10, np.nan), np.full(10, np.inf)])
def test_invalid_optimizer_output_never_becomes_a_profile(tmp_path, mocked_core, betas):
    core, _, assets = mocked_core
    core.optimize_shape = lambda *args, **kwargs: betas
    with pytest.raises(ValueError, match="official optimizer shape_params"):
        shape.optimize_contacts(contacts(), "checkout", assets, tmp_path / "output")
    assert not (tmp_path / "output" / "shape.json").exists()


@pytest.mark.skipif(not os.environ.get("FSGLOVE12_UPSTREAM_CHECKOUT") or not os.environ.get("FSGLOVE12_MANO_ASSETS"),
                    reason="Explicit trusted upstream checkout and MANO assets required")
def test_real_upstream_fit_and_report_match_live_mano_geometry(tmp_path):
    """Real model/software smoke with synthetic poses; no anatomical truth claim."""
    from vt_fsglove12.mano import ManoBackend
    checkout, assets = os.environ["FSGLOVE12_UPSTREAM_CHECKOUT"], os.environ["FSGLOVE12_MANO_ASSETS"]
    document = contacts()
    document["mano_pose_rotvec"][1][0] = [.1, -.2, .05]
    result = shape.optimize_contacts(document, checkout, assets, tmp_path / "real", iterations=3, seed=7)
    assert result["shape_profile"]["upstream"]["numerical_ast_modified"] is False
    assert result["shape_profile"]["iterations"] == 3
    backend = ManoBackend(assets)
    for index, category in enumerate(document["finger_category"]):
        for stage, betas in (("before", result["before_shape_params"]), ("after", result["after_shape_params"])):
            joints = np.asarray(backend.render_mano(document["mano_pose_rotvec"][index], betas)["joints_m"])
            gap = np.linalg.norm(joints[4] - joints[category * 4]) * 1000.
            assert result["metrics"]["per_pair"][shape.FINGERS[category]][stage]["mean_gap_mm"] == pytest.approx(gap, abs=1e-4)
