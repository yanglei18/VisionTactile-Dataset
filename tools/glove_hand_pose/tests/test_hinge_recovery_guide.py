"""Failure reasons must survive the controller-to-operator boundary."""
from copy import deepcopy

import pytest

from vt_glove_pose.hinge_guide import guide_for
from vt_glove_pose.hinge_recovery import classify_hinge_failure


def failed_state(status, reason):
    recovery = classify_hinge_failure(status, [reason])
    return dict(state="needs_neutral" if recovery["requires_new_neutral"] else "neutral_ready",
        needs_neutral=recovery["requires_new_neutral"], recovery=recovery,
        connected=True, pair_age_s=.01, matched_pairs=600,
        status_message="This human-readable wording must not determine the recovery action.",
        runtime=dict(connected=True, validation_remaining_s=None,
                     validation_for_current_candidate=False, unsaved_experiment=True))


def test_reference_nearly_parallel_advice_reaches_main_guide_without_being_lost():
    state = failed_state("frames_neutral_unobservable", "explicit_neutral_gravity_nearly_parallel_to_hinge")
    before = deepcopy(state)
    guide = guide_for(state)
    assert guide["primary_command"] == "neutral"
    assert guide["illustration"] == "rotate"
    assert "倾斜" in guide["instruction"] + guide["detail"]
    assert guide["instruction"] == state["recovery"]["instruction"]
    assert state == before


@pytest.mark.parametrize("reason", ["explicit_neutral_axis_gravity_mismatch",
                                  "explicit_neutral_acceleration_norm_inconsistent"])
def test_reference_related_mismatch_is_not_sent_back_to_only_repeat_motion(reason):
    state = failed_state("frames_inconsistent", reason)
    guide = guide_for(state)
    assert guide["primary_command"] == "export"
    assert "不用重做静止" not in guide["instruction"]
    assert guide["title"] == state["recovery"]["title"]
    assert state["recovery"]["needs_review"]
    state["pair_age_s"] = 10.
    assert guide_for(state)["primary_command"] == "export"


def test_saving_unresolved_failure_stops_the_same_export_retry_loop():
    state = failed_state("frames_inconsistent", "explicit_neutral_axis_gravity_mismatch")
    state["runtime"].update(last_export="saved_failure", unsaved_experiment=False)
    guide = guide_for(state)
    assert guide["phase"] == "waiting"
    assert not guide["primary_enabled"] and guide["primary_command"] is None
    assert "已保存" in guide["instruction"]
    assert "反复" in guide["instruction"]


def test_sign_ambiguity_preserves_reference_and_explains_motion_change():
    state = failed_state("frames_sign_unobservable", "insufficient_parallel_gravity_component_to_resolve_axis_sign")
    guide = guide_for(state)
    assert guide["primary_command"] == "motion"
    assert not state["needs_neutral"]
    assert guide["detail"] == state["recovery"]["detail"]


def test_new_capture_still_requires_fresh_pairs_but_background_export_takes_priority():
    state = failed_state("frames_neutral_unobservable", "explicit_neutral_gravity_nearly_parallel_to_hinge")
    state["pair_age_s"] = None
    assert not guide_for(state)["primary_enabled"]
    state["runtime"]["background_job"] = "export"
    assert guide_for(state)["phase"] == "processing"
    assert guide_for(state)["title"] == "正在导出"


def test_legacy_reference_failure_text_still_explains_the_required_tilt():
    state = failed_state("frames_neutral_unobservable", "explicit_neutral_gravity_nearly_parallel_to_hinge")
    state.pop("recovery")
    state["status_message"] = "拟合未通过：frames_neutral_unobservable: explicit_neutral_gravity_nearly_parallel_to_hinge"
    guide = guide_for(state)
    assert guide["primary_command"] == "neutral"
    assert "倾斜" in guide["detail"] and "原姿势" in guide["detail"]
