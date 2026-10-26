"""The online reference selector uses the same official local rotations as fit."""
from copy import deepcopy
import os

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from test_upstream_backend import official_core, fit_data, frame_for_anatomical_angles
from test_upstream_functional_hand import batch_data
from vt_fsglove12.upstream_functional import FINGER_SLOTS


pytestmark = pytest.mark.skipif(
    not os.environ.get("FSGLOVE12_UPSTREAM_CHECKOUT") or not os.environ.get("FSGLOVE12_MANO_ASSETS"),
    reason="requires explicitly configured official source and licensed model",
)


def test_lightweight_reference_matches_official_fit_domain(official_core):
    backend, profile, template = fit_data(official_core)
    angles = np.random.default_rng(42).normal(0., .25, (16, 3))
    frame = frame_for_anatomical_angles(backend, profile, template, angles)
    saved = deepcopy((frame, profile))
    slots = [slot for values in FINGER_SLOTS.values() for slot in values]
    raw = backend._solve_frame(frame, profile, functional=False)
    result = backend.functional_reference_rotations(frame, profile, slots)
    assert set(result) == set(map(str, slots))
    for slot in slots:
        expected = Rotation.from_euler("XYZ", raw["anatomical_euler_rad"][slot])
        actual = Rotation.from_quat(result[str(slot)]["quat_xyzw"])
        assert (expected.inv() * actual).magnitude() < 1e-12
    assert (frame, profile) == saved


@pytest.mark.parametrize("fault", ["epoch", "stale", "protected", "invalid"])
def test_reference_reader_excludes_invalid_dependency(official_core, fault):
    backend, profile, frame = fit_data(official_core)
    deps = backend._functional_binding(profile)["dependency_nodes_by_slot"]["2"]
    data = frame["nodes"][deps[0]]
    if fault == "epoch": data["epoch"] += 1
    elif fault == "stale": data["age_s"] = 100.
    elif fault == "protected": data["input_filter"] = {"gyro_rejected": True}
    else: data["orientation_valid"] = False
    assert backend.functional_reference_rotations(frame, profile, [2]) == {}


def test_partial_reference_cannot_update_an_unqualified_joint(batch_data):
    backend, profile, neutral, fingers, thumb = batch_data
    saved = deepcopy((profile, neutral, fingers, thumb))
    result = backend.fit_functional_hand(neutral, fingers, thumb, profile,
        session_id=profile["session_id"], reference_selection={"stable_slots": ["1"]})
    assert set(result["entries"]) == {"index"}
    assert set(result["entries"]["index"]["slot_refs"]) == {"1"}
    assert "2" in result["entries"]["index"]["rejected_slots"]
    assert set(result["errors"]) == {"middle", "ring", "little", "thumb"}
    assert (profile, neutral, fingers, thumb) == saved
