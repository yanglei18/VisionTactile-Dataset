"""Read-only admission checks for an explicit, versioned recording plan.

The manager observes selected streams, including raw tactile sample arrivals;
it does not subscribe to camera calibration or frame timing for this check.
Storage admission remains owned by the Recorder,
whose output filesystem can differ from the manager's status filesystem.
"""
from __future__ import annotations

from vt_capture_core.capture_plan import (
    CapturePlanError, compare_controller_topics, require_available_streams,
    resolve_capture_plan, topics_fingerprint, validate_capture_plan,
)
from vt_capture_core.recipes import RecipeError, validate_recipe

from .storage import utc_now


_AUXILIARY_TYPES = {
    "sensor_msgs/msg/CameraInfo",
    "realsense2_camera_msgs/msg/Extrinsics",
    "vt_camera_msgs/msg/CameraFrameTiming",
}


class PlanPreflightError(ValueError):
    """A known, unsent START rejection with a stable operation code."""

    def __init__(self, code: str, detail: str):
        super().__init__(detail)
        self.code = code


def check_capture_plan(params: dict, context: dict, topic_activity: dict, *, frozen_plan=None):
    """Return the resolved plan and bounded observations, or reject admission.

    Freeze receiving streams as required; absent candidates remain subscribed.
    A dispatch recheck uses the accepted frozen plan, so a stream that disappears
    while queued cannot silently become optional. Inputs are never mutated.
    """
    if params["expected_controller_instance_id"] != context["controller_instance_id"]:
        raise PlanPreflightError("controller_instance_mismatch", "录制控制器已变化，请刷新后重新开始。")
    if params["expected_state_revision"] != context["state_revision"]:
        raise PlanPreflightError("state_revision_mismatch", "录制状态已变化，请刷新后重新开始。")
    if context["recorder_alive"] or context["state"] not in (0, 5, 6):
        raise PlanPreflightError("active_session", "已有录制或状态未确认，未发送新的录制命令。")
    try:
        recipe = validate_recipe(params["recipe"])
        plan = resolve_capture_plan(recipe)
        actual_fingerprint = topics_fingerprint(context["topics"])
        comparison = compare_controller_topics(plan, context["topics"])
    except (CapturePlanError, RecipeError) as exc:
        raise PlanPreflightError("capture_plan_invalid", "采集计划或控制器清单格式无效，未发送录制命令。") from exc
    if recipe.duration_s != params["duration_s"] or recipe.duration_s > context["max_duration_sec"]:
        raise PlanPreflightError("capture_duration_mismatch", "采集计划时长与录制请求或控制器限制不一致。")
    if params["expected_topics_fingerprint"] != actual_fingerprint:
        raise PlanPreflightError("capture_topics_changed", "当前录制清单已变化，请刷新后重新开始。")
    if not comparison.matches:
        raise PlanPreflightError("capture_plan_mismatch", "采集计划与控制器实际录制清单不一致，请检查设备配置。")

    if frozen_plan is not None:
        try:
            accepted = validate_capture_plan(frozen_plan)
            if accepted.recipe != recipe or not compare_controller_topics(accepted, context["topics"]).matches:
                raise CapturePlanError("frozen plan differs from the accepted request")
        except CapturePlanError as exc:
            raise PlanPreflightError("capture_plan_invalid", "已保存的采集计划无效，未发送录制命令。") from exc
        plan = accepted

    observations, unobserved, receiving = [], [], set()
    for stream in plan.streams:
        source = stream.topic
        known = source in topic_activity
        active = known and topic_activity[source] is True
        if not known:
            unobserved.append(stream.topic)
        if active:
            receiving.add(stream.topic)
        observations.append(dict(topic=stream.topic, observation_topic=source if known else None,
                                 state="receiving" if active else
                                       ("not_receiving" if known else "not_observed")))

    if frozen_plan is None:
        # Camera calibration and timing support a receiving image stream, even
        # though the manager does not subscribe to those auxiliary topics.
        active_devices = {stream.device_id for stream in plan.streams
                          if stream.topic in receiving and stream.type_name not in _AUXILIARY_TYPES}
        if not active_devices:
            raise PlanPreflightError("capture_no_streams_available",
                                     "当前没有可录制的数据流，请先启动至少一个采集设备。")
        required = receiving | {stream.topic for stream in plan.streams
                                if stream.device_id in active_devices and stream.type_name in _AUXILIARY_TYPES}
        plan = require_available_streams(plan, tuple(stream.topic for stream in plan.streams if stream.topic in required))
    else:
        missing = [stream.topic for stream in plan.streams if stream.required
                   and stream.type_name not in _AUXILIARY_TYPES and stream.topic not in receiving]
        if missing:
            raise PlanPreflightError("capture_streams_not_ready",
                                     "开始前已接收的数据流已中断：" + ", ".join(missing))
    return plan, dict(version=2, checked_at=utc_now(), plan_fingerprint=plan.fingerprint,
                     controller_topics_fingerprint=actual_fingerprint,
                     controller_instance_id=context["controller_instance_id"],
                     controller_state_revision=context["state_revision"],
                     observations=observations, unobserved_topics=unobserved,
                     optional_topics=list(plan.optional_topics()),
                     message_type_verification="not_observed", storage_admission="recorder_owned",
                     scope="Recent message arrivals only; payload validity is not an admission requirement. Unobserved camera auxiliaries and recorded message coverage require post-record verification.")
