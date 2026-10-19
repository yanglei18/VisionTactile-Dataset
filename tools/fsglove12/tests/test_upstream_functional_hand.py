"""Batch capture orchestration with real FSGlove/QMT, not hardware accuracy."""
from copy import deepcopy
import os

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from test_upstream_backend import official_core, fit_data, frame_for_anatomical_angles
from vt_fsglove12.upstream_functional import FINGER_SLOTS


pytestmark = pytest.mark.skipif(
    not os.environ.get("FSGLOVE12_UPSTREAM_CHECKOUT") or not os.environ.get("FSGLOVE12_MANO_ASSETS"),
    reason="requires explicit official source and licensed model",
)


@pytest.fixture(scope="module")
def batch_data(official_core):
    pytest.importorskip("qmt")
    backend, profile, template = fit_data(official_core)
    bases = {slot: Rotation.from_rotvec([.15 + slot * .03, -.22, 0.])
             for slots in FINGER_SLOTS.values() for slot in slots}
    neutral = Rotation.from_euler("XYZ", [.1, -.04, .07])

    def sample(sequence, *, fingers=0., thumb=0., phase="neutral"):
        angles = np.zeros((16, 3))
        for finger, slots in FINGER_SLOTS.items():
            for slot in slots:
                bend = thumb if finger == "thumb" else fingers * (1. + slot * .01)
                local = Rotation.from_rotvec([0., 0., bend])
                # The unrelated motion is deliberately along a different axis.
                # It must not contaminate the selected finger's motion segment.
                if phase == "four_fingers" and finger == "thumb":
                    local = Rotation.from_rotvec([0., fingers * .6, 0.])
                elif phase == "thumb_ip" and finger != "thumb":
                    local = Rotation.from_rotvec([thumb * .7, 0., 0.])
                angles[slot] = (neutral * bases[slot] * local * bases[slot].inv()).as_euler("XYZ")
        result = frame_for_anatomical_angles(backend, profile, template, angles)
        result["timestamp_s"] = 1000. + sequence / 30.
        for data in result["nodes"].values():
            data["sample_index"] = sequence
        return result

    references = [sample(100 + i) for i in range(6)]
    cycle = np.r_[np.linspace(.05, .95, 6), np.linspace(.95, .05, 6)]
    fingers = [sample(110 + i, fingers=float(angle), phase="four_fingers")
               for i, angle in enumerate(np.tile(cycle, 2))]
    thumb = [sample(160 + i, thumb=float(angle), phase="thumb_ip")
             for i, angle in enumerate(np.tile(cycle, 2))]
    return backend, profile, references, fingers, thumb


def fit_batch(data, *, profile=None, neutral=None, fingers=None, thumb=None):
    backend, base, references, four, ip = data
    profile = base if profile is None else profile
    return backend.fit_functional_hand(references if neutral is None else neutral,
        four if fingers is None else fingers, ip if thumb is None else thumb,
        profile, session_id=profile["session_id"])


def test_batch_equals_individual_fits_and_solves_shared_frames_only_once(batch_data, monkeypatch):
    backend, profile, neutral, fingers, thumb = batch_data
    untouched = deepcopy((profile, neutral, fingers, thumb))
    expected = {finger: backend.fit_functional_finger(neutral, thumb if finger == "thumb" else fingers,
        profile, finger=finger, session_id=profile["session_id"]) for finger in FINGER_SLOTS}
    calls = []
    original = backend._solve_frame

    def solve_once(frame, frozen_profile, *, functional):
        assert frozen_profile["functional_enabled"] is False
        assert functional is False
        calls.append(frame["timestamp_s"])
        return original(frame, frozen_profile, functional=functional)

    monkeypatch.setattr(backend, "_solve_frame", solve_once)
    result = fit_batch(batch_data)
    assert len(calls) == len(neutral) + len(fingers) + len(thumb)
    assert result["errors"] == {}
    assert result["entries"] == expected
    assert result["input_frame_counts"] == dict(neutral=6, four_fingers=24, thumb_ip=24)
    assert (profile, neutral, fingers, thumb) == untouched


def test_batch_partial_finger_and_joint_retain_old_parameters(batch_data):
    backend, profile, neutral, fingers, thumb = batch_data
    old = {**profile, "functional_fingers": fit_batch(batch_data)["entries"], "functional_enabled": False}
    before = deepcopy(old)
    missing = deepcopy(fingers)
    for frame in missing:
        del frame["nodes"]["3"]  # Whole index lacks proximal dependency.
        frame["nodes"]["10"]["epoch"] += 1  # Little PIP only; MCP remains measurable.
    result = fit_batch(batch_data, profile=old, fingers=missing)
    assert set(result["errors"]) == {"index"}
    assert set(result["entries"]) == {"middle", "ring", "little", "thumb"}
    little = result["entries"]["little"]
    assert little["diagnostics"]["7"]["status"] == "fitted"
    assert little["diagnostics"]["8"]["status"] == "retained_previous"
    assert little["slot_refs"]["8"] == old["functional_fingers"]["little"]["slot_refs"]["8"]
    merged = {**old["functional_fingers"], **result["entries"]}
    assert merged["index"] == old["functional_fingers"]["index"]
    assert old == before


def test_batch_missing_thumb_does_not_discard_four_fingers_or_use_their_motion(batch_data):
    result = fit_batch(batch_data, thumb=[])
    assert set(result["entries"]) == {"index", "middle", "ring", "little"}
    assert set(result["errors"]) == {"thumb"}


def test_batch_duplicate_motion_indices_cannot_count_as_new_evidence(batch_data):
    _, _, neutral, fingers, thumb = batch_data
    duplicate = deepcopy(fingers)
    for frame in duplicate:
        for node, data in frame["nodes"].items():
            data["sample_index"] = neutral[-1]["nodes"][node]["sample_index"]
    result = fit_batch(batch_data, fingers=duplicate)
    assert set(result["entries"]) == {"thumb"}
    assert set(result["errors"]) == {"index", "middle", "ring", "little"}


def test_batch_missing_root_or_all_static_never_reports_updated_fingers(batch_data):
    _, profile, neutral, fingers, thumb = batch_data
    missing = deepcopy(neutral)
    for frame in missing:
        frame["nodes"].pop("12")
    no_root = fit_batch(batch_data, neutral=missing)
    assert no_root["entries"] == {}
    assert set(no_root["errors"]) == set(FINGER_SLOTS)
    still_fingers, still_thumb = deepcopy(fingers), deepcopy(thumb)
    for frame in still_fingers + still_thumb:
        for node, data in frame["nodes"].items():
            data["quat_xyzw"] = list(neutral[0]["nodes"][node]["quat_xyzw"])
    no_motion = fit_batch(batch_data, fingers=still_fingers, thumb=still_thumb)
    assert no_motion["entries"] == {}
    assert set(no_motion["errors"]) == set(FINGER_SLOTS)
    assert "functional_fingers" not in profile


def test_batch_fits_use_raw_official_results_even_when_previous_corrections_enabled(batch_data):
    _, profile, _, _, _ = batch_data
    first = fit_batch(batch_data)
    old = {**profile, "functional_fingers": first["entries"], "functional_enabled": True}
    actual = fit_batch(batch_data, profile=old)
    assert actual == first


def test_batch_cross_session_input_rejected_without_mutation(batch_data):
    _, profile, _, _, thumb = batch_data
    before = deepcopy(profile)
    foreign = deepcopy(thumb)
    foreign[3]["session_id"] = "different-session"
    with pytest.raises(ValueError, match="会话不一致"):
        fit_batch(batch_data, thumb=foreign)
    assert profile == before
