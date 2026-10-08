"""Calibration lifecycle checks without hardware or licensed MANO assets.

The core double only creates serialized calibration state. These tests verify
cache ownership and reset behavior, not official inference accuracy.
"""
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest

from test_upstream_backend import make_data, synthetic_state


@pytest.fixture
def backend_data():
    created = []

    def create_calibrator(**_):
        calibrator = synthetic_state()
        created.append(calibrator)
        return calibrator

    core = SimpleNamespace(create_calibrator=create_calibrator,
                           provenance={"upstream_commit": "reset-lifecycle-test-double"})
    backend, captures = make_data(core)
    return backend, captures, created


def seed_pose_history(backend):
    """Populate each runtime cache so a partial reset cannot pass unnoticed."""
    for slot in (2, 5, 8, 11, 15):
        backend._functional_pipeline.hinges._states[slot] = object()


def assert_pose_history_empty(backend):
    assert backend._functional_pipeline.hinges._states == {}


@pytest.mark.parametrize("method", ["reset_calibration", "reset"])
def test_reset_clears_all_calibration_state_and_is_idempotent(backend_data, method):
    backend, captures, created = backend_data
    profile = backend.fit_calibration(captures, session_id="test-session")
    seed_pose_history(backend)
    before = deepcopy((backend.config, backend.rest, profile, captures))
    core = backend.core
    helpers = (backend._functional_pipeline, backend._functional_pipeline.hinges)
    heading_settings = deepcopy(backend._functional_pipeline.hinges.settings)

    for _ in range(2):
        assert getattr(backend, method)() is None
        assert backend._cached_key is None and backend._cached_calibrator is None
        assert_pose_history_empty(backend)
        assert backend.core is core and len(created) == 1
        assert helpers == (backend._functional_pipeline, backend._functional_pipeline.hinges)
        assert backend._functional_pipeline.hinges.settings == heading_settings
        assert (backend.config, backend.rest, profile, captures) == before


def test_reset_before_first_calibration_requires_no_loaded_calibrator(backend_data):
    backend, _, created = backend_data
    backend.reset_calibration()
    backend.reset()
    assert backend._cached_key is None and backend._cached_calibrator is None
    assert_pose_history_empty(backend)
    assert created == []


def test_reset_rebuilds_same_saved_profile_instead_of_reusing_cached_instance(backend_data):
    backend, captures, created = backend_data
    profile = backend.fit_calibration(captures, session_id="test-session")
    before = deepcopy(profile)
    cached = backend._calibrator(profile)
    assert cached is created[0] and len(created) == 1

    backend.reset_calibration()
    restored = backend._calibrator(profile)
    assert restored is created[1] and restored is not cached
    assert backend._cached_key == profile["upstream_state_sha256"]
    assert backend._calibrator(profile) is restored and len(created) == 2
    for name, expected in profile["upstream_state"].items():
        np.testing.assert_array_equal(getattr(restored, name), expected)
    assert profile == before


def test_clear_then_recalibrate_reuses_backend_and_core_without_old_history(backend_data):
    backend, captures, created = backend_data
    first = backend.fit_calibration(captures, session_id="test-session")
    original_calibrator = backend._cached_calibrator
    seed_pose_history(backend)
    backend.reset_calibration()

    unavailable = backend.solve(captures["base_pose"][-1], None)
    assert unavailable["reason"] == "calibration_required"
    assert not unavailable["pose_available"] and unavailable["mano_pose_rotvec"] is None
    assert backend._cached_calibrator is None and len(created) == 1

    second = backend.fit_calibration(captures, session_id="test-session")
    assert second == first
    assert backend._cached_calibrator is created[1]
    assert backend._cached_calibrator is not original_calibrator
    assert_pose_history_empty(backend)
    assert backend._functional_pipeline.hinges._states == {}
