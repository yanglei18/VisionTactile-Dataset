"""Actionable coaching is separate from capture/algorithm state and truth."""
from copy import deepcopy

import pytest

from vt_glove_pose.hinge_guide import guide_for


def snapshot(name="needs_neutral", **changes):
    result = dict(state=name, connected=True, pair_age_s=.01, matched_pairs=600,
                  preview_valid=False, needs_neutral=name == "needs_neutral",
                  countdown_s=0., capture_remaining_s=0., fit_pending=False,
                  observer=None, direction={"status": "unconfirmed"},
                  runtime=dict(connected=True, background_job=None, validation_remaining_s=None,
                               validation_attempts=0, unsaved_experiment=True, last_export=None))
    result.update(changes)
    return result


def candidate(*, observed=True, confirmed=False):
    return snapshot("candidate_ready", preview_valid=True,
                    observer={"current_angle_observed": observed, "angle_deg": 37.},
                    direction={"status": "confirmed" if confirmed else "unconfirmed"})


def test_connect_is_manual_and_input_is_not_mutated():
    state = dict(state="disconnected", connected=False, runtime={"connected": False})
    before = deepcopy(state)
    guide = guide_for(state)
    assert guide["step"] == 0 and guide["primary_command"] == "connect"
    assert guide["primary_enabled"] and guide["phase"] == "idle"
    assert state == before
    assert guide_for({})["primary_command"] == "connect"


def test_connected_but_missing_stale_or_unmatched_pair_never_prompts_start():
    for changes in ({"pair_age_s": None}, {"pair_age_s": .501}, {"matched_pairs": 0}):
        guide = guide_for(snapshot(**changes))
        assert guide["phase"] == "waiting"
        assert guide["primary_command"] is None and not guide["primary_enabled"]
        assert "成对数据" in guide["primary_text"]
    assert guide_for(snapshot(pair_age_s=.4), stale_after_s=.3)["phase"] == "waiting"
    assert guide_for(snapshot(pair_age_s=.4), stale_after_s=.5)["primary_command"] == "neutral"


def test_neutral_preparation_and_capture_have_distinct_clock_and_instruction():
    ready = guide_for(snapshot(), finger_name="食指")
    assert ready["step"] == 1 and ready["primary_command"] == "neutral"
    assert "食指" in ready["instruction"]
    preparation = guide_for(snapshot("prepare_neutral", countdown_s=2., capture_remaining_s=5.))
    capture = guide_for(snapshot("capturing_neutral", countdown_s=0., capture_remaining_s=4.))
    assert preparation["phase"] == "prepare" and preparation["seconds_left"] == 2.
    assert preparation["progress"] == pytest.approx(1/3)
    assert "尚未采集" in preparation["detail"]
    assert capture["phase"] == "capture" and capture["seconds_left"] == 4.
    assert capture["progress"] == pytest.approx(.2)
    assert "保持" in capture["instruction"] and "正在采集" in capture["detail"]
    assert not preparation["primary_enabled"] and not capture["primary_enabled"]


def test_successful_neutral_explicitly_skips_repeat_and_offers_motion():
    guide = guide_for(snapshot("neutral_ready"))
    assert guide["step"] == 2 and guide["primary_command"] == "motion"
    assert "静止参考已成功" in guide["title"] and "无需重复" in guide["instruction"]
    assert "20 秒" in guide["detail"]


def test_motion_preparation_capture_and_processing_disable_primary_actions():
    for name, phase, seconds in [("prepare_motion", "prepare", 2.),
                                  ("capturing_motion", "capture", 15.), ("fitting", "processing", None)]:
        guide = guide_for(snapshot(name, countdown_s=2., capture_remaining_s=15.))
        assert guide["step"] == 2 and guide["phase"] == phase and guide["seconds_left"] == seconds
        assert not guide["primary_enabled"]
    state = candidate(confirmed=True)
    state["runtime"].update(background_job="fit", validation_for_current_candidate=True)
    assert guide_for(state)["phase"] == "processing"
    assert not guide_for(state)["primary_enabled"]


def test_direction_requires_current_observation_but_does_not_require_zero_angle():
    state = candidate(observed=False)
    waiting = guide_for(state)
    assert waiting["step"] == 3 and waiting["phase"] == "waiting"
    assert not waiting["primary_enabled"] and "预测" in waiting["title"]
    state["observer"]["current_angle_observed"] = True
    guide = guide_for(state)
    assert guide["primary_command"] == "direction" and guide["primary_enabled"]
    assert "不必先伸直" in guide["detail"]
    state["preview_valid"] = False
    assert guide_for(state)["phase"] == "waiting"
    assert not guide_for(state)["primary_enabled"]
    for name in ("prepare_direction", "capturing_direction"):
        state = snapshot(name, countdown_s=1., capture_remaining_s=3.)
        guide = guide_for(state)
        assert guide["step"] == 3 and not guide["primary_enabled"]
        assert guide["illustration"] == "flex_once"


def test_current_candidate_validation_overrides_old_attempts_then_prioritizes_export():
    state = candidate(confirmed=True)
    state["runtime"].update(validation_attempts=3, validation_for_current_candidate=False)
    guide = guide_for(state)
    assert guide["step"] == 4 and guide["primary_command"] == "validation"
    assert "30 秒" in guide["primary_text"]
    state["runtime"]["validation_for_current_candidate"] = True
    guide = guide_for(state)
    assert guide["step"] == 5 and guide["primary_command"] == "export"
    assert "不是验证通过" in guide["detail"]
    del state["runtime"]["validation_for_current_candidate"]
    assert guide_for(state)["primary_command"] == "export"  # Older runtime compatibility.


def test_quickcheck_has_six_time_guided_segments_not_automatic_action_detection():
    state = candidate(confirmed=True)
    state["runtime"]["validation_guided_quickcheck"] = True
    for elapsed, phase, seconds, illustration, text in [
        (1., "prepare", 2., "still", "参考"),
        (3.5, "capture", 1.5, "hold", "不动"),
        (10., "capture", 10., "flex", "缓慢弯曲"),
        (22., "capture", 3., "hold", "弯曲"),
        (26., "capture", 2., "return", "同一个手型"),
        (29., "capture", 1., "hold", "参考姿势"),
    ]:
        state["runtime"]["validation_remaining_s"] = 30. - elapsed
        before = deepcopy(state)
        guide = guide_for(state)
        assert guide["step"] == 4 and guide["phase"] == phase
        assert guide["seconds_left"] == seconds and 0 <= guide["progress"] <= 1
        assert guide["illustration"] == illustration and text in guide["instruction"]
        assert "不是动作检测" in guide["detail"] and not guide["primary_enabled"]
        assert state == before


def test_advanced_validation_prioritizes_current_marker_over_generic_recording():
    state = candidate(confirmed=True)
    state["runtime"].update(validation_remaining_s=80., validation_duration_s=90.,
        validation_now_s=10., validation_markers=[dict(kind="common_rotation", status="recording",
        start_host_monotonic_s=11., end_host_monotonic_s=21.)])
    prep = guide_for(state)
    assert prep["phase"] == "prepare" and prep["seconds_left"] == 1.
    assert prep["illustration"] == "rotate" and "保持手指弯曲不变" in prep["instruction"]
    state["runtime"]["validation_now_s"] = 12.
    capture = guide_for(state)
    assert capture["phase"] == "capture" and capture["seconds_left"] == 9.
    state["runtime"]["validation_markers"][0]["status"] = "completed"
    general = guide_for(state)
    assert general["title"] == "独立验证录制中" and general["seconds_left"] == 80.
    assert "高级区" in general["detail"]


def test_export_processing_then_current_saved_data_is_complete_not_product_pass():
    state = candidate(confirmed=True)
    state["runtime"].update(background_job="export", validation_for_current_candidate=True)
    guide = guide_for(state)
    assert guide["step"] == 5 and guide["phase"] == "processing" and not guide["primary_enabled"]
    state["runtime"].update(background_job=None, unsaved_experiment=False, last_export="experiment")
    guide = guide_for(state)
    assert guide["phase"] == "complete" and guide["progress"] == 1.
    assert "不代表" in guide["detail"] and not guide["primary_enabled"]
    state["runtime"]["unsaved_experiment"] = True
    assert guide_for(state)["phase"] == "idle"
    assert guide_for(state)["primary_command"] == "export"


def test_exporting_only_reference_does_not_hide_remaining_motion_or_reference_retry():
    state = snapshot("neutral_ready")
    state["runtime"].update(last_export="partial_reference", unsaved_experiment=False,
                            validation_for_current_candidate=False)
    guide = guide_for(state)
    assert guide["primary_command"] == "motion" and guide["phase"] == "idle"
    state.update(state="needs_neutral", needs_neutral=True,
                 status_message="静止参考未通过：neutral_gyro_not_quiet；原始数据已保留。")
    assert guide_for(state)["primary_command"] == "neutral"
    assert "陀螺数据不够平稳" in guide_for(state)["detail"]


def test_failed_fits_and_neutral_have_short_specific_operator_advice():
    for reason, expected in [("frames_sign_unobservable", "整手倾斜"),
                             ("axes_insufficient_excitation", "两段都自然屈伸"),
                             ("axes_inconsistent", "检查固定"), ("input_near_limit", "放慢")]:
        guide = guide_for(snapshot("neutral_ready", status_message=f"拟合未通过：{reason}；记录已保留。"))
        assert guide["title"] == "运动标定需调整，静止参考保留"
        assert guide["primary_command"] == "motion" and expected in guide["detail"]
    for reason, expected in [("neutral_pair_count_insufficient", "成对样本不足"),
                             ("neutral_acceleration_not_stable_gravity_like", "单位配置"),
                             ("neutral_host_span_insufficient", "覆盖时间不足")]:
        guide = guide_for(snapshot(status_message=f"静止参考未通过：{reason}；原始数据已保留。"))
        assert guide["primary_command"] == "neutral" and expected in guide["detail"]
