"""Native QMT batching parity; no timing thresholds or hardware access."""
import importlib

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

pytest.importorskip("qmt")

from vt_fsglove12.hinge_estimator import _FrameProjection, FunctionalHingeEstimator


def test_batched_projections_match_independent_official_qmt_including_pi():
    rng = np.random.default_rng(4576)
    for index in range(200):
        parent, child = R.random(2, random_state=rng)
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        deltas = (rng.uniform(-np.pi, np.pi), np.pi - 1e-12 if index % 2 else -np.pi + 1e-12)
        projector = _FrameProjection(parent, child, axis)
        actual = projector.pair(*deltas)
        expected = [FunctionalHingeEstimator._project(parent, child, axis, d) for d in deltas]
        np.testing.assert_allclose(actual, expected, atol=1e-10, rtol=1e-12)
        assert projector.one(deltas[0]) == actual[0]
        assert projector.one(deltas[1]) == actual[1]


def test_current_frame_calls_qmt_once_for_pair_then_reuses_final_projection(monkeypatch):
    module = importlib.import_module("vt_fsglove12.hinge_estimator")
    original, calls = module._project, []

    def counted(rotations, axis):
        calls.append(rotations.as_quat().shape)
        return original(rotations, axis)

    monkeypatch.setattr(module, "_project", counted)
    projector = _FrameProjection(R.identity(), R.from_rotvec([0., 0., .2]), [0., 0., 1.])
    old, proposed = projector.pair(.1, .11)
    assert calls == [(2, 4)]
    assert projector.one(.11) == proposed
    assert projector.one(.1) == old
    assert projector.pair(.1, .11) == (old, proposed)
    assert len(calls) == 1
    # A new sensor pair never receives a historical cached result.
    next_frame = _FrameProjection(R.identity(), R.from_rotvec([0., 0., .8]), [0., 0., 1.])
    assert next_frame.one(.11) != proposed
    assert len(calls) == 2


def test_nonidentifiable_projection_is_not_cached_as_valid():
    projector = _FrameProjection(R.identity(), R.from_rotvec([np.pi, 0., 0.]), [0., 0., 1.])
    with pytest.raises(ValueError, match="180"):
        projector.one(0.)
    assert not projector.cache


def test_repeated_reference_only_projects_once(monkeypatch):
    projector = _FrameProjection(R.identity(), R.identity(), [0., 0., 1.])
    first, second = projector.pair(.1, .1)
    assert first == second
    assert len(projector.cache) == 1
