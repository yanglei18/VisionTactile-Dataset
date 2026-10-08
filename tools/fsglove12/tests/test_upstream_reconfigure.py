"""Input-scale reconfiguration shares loaded resources, never calibration state."""
from copy import deepcopy
from types import SimpleNamespace

import pytest

from test_upstream_backend import fit_data, make_data, official_core, synthetic_state
from vt_fsglove12.config import digest
from vt_fsglove12.upstream_backend import ALGORITHM


def prepared_backend():
    backend, _ = make_data(SimpleNamespace(provenance={"kind": "test_core_only"}))
    state = synthetic_state()
    backend._rotation_cache.get(state)
    backend._functional_pipeline.hinges._states[8] = object()
    profile = dict(method=ALGORITHM, config_sha256=backend._config_sha,
        rest_sha256=backend._rest_sha, upstream_provenance=backend.core.provenance,
        missing_slot_policy=backend._missing_policy,
        missing_slot_options=deepcopy(backend._prior_options), upstream_state={})
    profile["upstream_state_sha256"] = digest(profile["upstream_state"])
    backend._cached_key = profile["upstream_state_sha256"]
    backend._cached_calibrator = state
    return backend, profile


def test_reconfiguration_reuses_core_and_rest_but_not_any_calibration_cache(monkeypatch):
    import vt_fsglove12.upstream_core as module
    backend, profile = prepared_backend()
    old_config, old_rest = deepcopy(backend.config), deepcopy(backend.rest)
    original_calibrator = backend._cached_calibrator
    original_rotations = backend._rotation_cache._rotations
    original_hinge = backend._functional_pipeline.hinges._states[8]

    def forbidden(*args, **kwargs):
        pytest.fail("reconfiguration must not load MANO or upstream resources again")

    monkeypatch.setattr(module, "load_upstream_core", forbidden)
    config = deepcopy(backend.config)
    config["gyro_dps_per_lsb_by_node"] = {"12": .0175}
    config["input_scale_candidate_provenance"] = {"kind": "explicit_user_test"}
    fresh = backend.with_config(config)
    assert fresh is not backend and fresh.core is backend.core
    assert fresh.config == config and fresh.config is not config
    assert fresh.rest == backend.rest and fresh.rest is not backend.rest
    assert fresh._config_sha == digest(config) != backend._config_sha
    assert fresh._rest_sha == backend._rest_sha
    assert fresh._cached_key is None and fresh._cached_calibrator is None
    assert fresh._rotation_cache is not backend._rotation_cache
    assert fresh._rotation_cache._calibrator is None
    assert fresh._rotation_cache._rotations is None
    assert fresh._rotation_cache._snapshots == ()
    assert fresh._functional_pipeline is not backend._functional_pipeline
    assert fresh._functional_pipeline.hinges._states == {}
    assert backend.config == old_config and backend.rest == old_rest
    assert backend._cached_calibrator is original_calibrator
    assert backend._rotation_cache._rotations is original_rotations
    assert backend._functional_pipeline.hinges._states[8] is original_hinge
    assert backend._calibrator(profile) is original_calibrator

    config["gyro_dps_per_lsb_by_node"]["12"] = .035
    fresh.rest["rest_global_xyzw"][0][0] = .1
    assert fresh.config["gyro_dps_per_lsb_by_node"]["12"] == .0175
    assert backend.rest == old_rest


def test_new_input_scale_rejects_old_profile_without_mutating_its_old_owner():
    backend, profile = prepared_backend()
    snapshot = deepcopy(profile)
    new_config = {**backend.config, "gyro_dps_per_lsb_by_node": {"12": .0175}}
    fresh = backend.with_config(new_config)
    with pytest.raises(ValueError, match="configuration or MANO model digest"):
        fresh._calibrator(profile)
    assert fresh._cached_key is None and fresh._cached_calibrator is None
    assert profile == snapshot
    assert backend._calibrator(profile) is backend._cached_calibrator


def test_invalid_reconfiguration_preserves_live_backend_and_profile():
    backend, profile = prepared_backend()
    previous_config = deepcopy(backend.config)
    previous_calibrator = backend._cached_calibrator
    with pytest.raises(ValueError):
        backend.with_config({**backend.config, "gyro_dps_per_lsb_by_node": {"12": 0.}})
    assert backend.config == previous_config
    assert backend._calibrator(profile) is previous_calibrator


def test_identical_config_still_creates_new_empty_state():
    backend, _ = prepared_backend()
    fresh = backend.with_config(backend.config)
    assert fresh._config_sha == backend._config_sha
    assert fresh.core is backend.core
    assert fresh._cached_calibrator is None
    assert not fresh._functional_pipeline.hinges._states


def test_real_official_profile_cannot_cross_scale_change_and_old_inference_is_unchanged(official_core):
    backend, profile, frame = fit_data(official_core)
    before = backend.solve(frame, profile)
    assert before["pose_available"]
    fresh = backend.with_config({**backend.config,
        "gyro_dps_per_lsb_by_node": {"12": .0175}})
    assert fresh.core is official_core
    with pytest.raises(ValueError, match="configuration or MANO model digest"):
        fresh.solve(frame, profile)
    assert fresh._cached_calibrator is None
    after = backend.solve(frame, profile)
    assert after["pose_available"]
    assert after["mano_pose_rotvec"] == before["mano_pose_rotvec"]
    assert after["anatomical_euler_rad"] == before["anatomical_euler_rad"]
