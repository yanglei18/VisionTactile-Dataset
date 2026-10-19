"""Present the configured Recorder allowlist without changing it or probing ROS."""
from __future__ import annotations

from dataclasses import dataclass

from vt_capture_core.capture_plan import (
    CapturePlanError, MAX_CONTROLLER_TOPICS, MAX_TOPIC_NAME_BYTES,
    ResolvedCapturePlan, recipe_for_topics, resolve_capture_plan, topics_fingerprint,
)

from ..i18n import LocalizedText, tr


@dataclass(frozen=True)
class CapturePlanView:
    summary: LocalizedText
    status: LocalizedText
    topics_text: str
    blocks_start: bool = False
    plan: ResolvedCapturePlan | None = None
    expected_topics_fingerprint: str | None = None

    def start_parameters(self) -> dict:
        """Return fresh machine values; legacy and blocked views carry no intent."""
        if self.plan is None or self.blocks_start:
            return {}
        return {
            "recipe": self.plan.recipe.to_dict(),
            "expected_topics_fingerprint": self.expected_topics_fingerprint,
        }


def _hands(values):
    labels = {"left": tr('左手'), "right": tr('右手')}
    return tr('、').join(labels[hand] for hand in values) if values else tr('未录入')


def _summary(plan):
    recipe = plan.recipe
    return tr('相机 {cameras} 台 · 追踪器 {trackers} 个\n手势：{pose} · 触觉：{tactile}',
              cameras=len(recipe.cameras), trackers=len(recipe.trackers),
              pose=_hands(recipe.pose_hands), tactile=_hands(recipe.tactile_hands))


def _plan_error(error):
    """Translate structured domain errors, leaving unknown diagnostics intact."""
    code = getattr(error, "code", None)
    details = getattr(error, "details", {})
    if code == "unsupported_controller_topics":
        problems = []
        if details.get("missing_topics"):
            problems.append(tr('设备数据主题配置不完整：{topics}', topics=", ".join(details["missing_topics"])))
        if details.get("unexpected_topics"):
            problems.append(tr('当前采集配置不支持这些数据主题：{topics}',
                               topics=", ".join(details["unexpected_topics"])))
        return tr('；').join(problems)
    if code == "duplicate_controller_topics":
        return tr('控制器数据主题清单存在重复项：{topics}',
                  topics=", ".join(details.get("duplicate_topics", ())))
    if code == "invalid_controller_topics":
        return tr('未收到有效的控制器数据主题清单，请检查控制器连接与录制配置。')
    if code == "controller_topics_size":
        return tr('控制器数据主题清单必须包含 1 至 {maximum} 项，请检查录制配置。',
                  maximum=MAX_CONTROLLER_TOPICS)
    if code == "invalid_controller_topic":
        return tr('控制器数据主题名称格式无效，必须使用完整的绝对 ROS数据主题名称。')
    if code == "controller_topic_too_long":
        return tr('控制器数据主题名称超过 {maximum} 字节，请检查录制配置。',
                  maximum=MAX_TOPIC_NAME_BYTES)
    return tr('无法建立采集计划：{detail}', detail=str(error))


def capture_plan_view(controller, *, duration_s, supported, required, demo=False):
    """Separate plan support, configured intent and startup validation status.

    ``required`` is negotiated protocol support, not a guessed device state. A
    v5 connection without the active capability must never become a legacy
    start. No publisher freshness or recorded-data quality is inferred here.
    """
    if demo:
        return CapturePlanView(tr('演示数据'), tr('演示模式不执行真实录制计划校验。'), "")
    topics = (controller or {}).get("topics")
    topics_text = "\n".join(topics) if isinstance(topics, (list, tuple)) and all(
        isinstance(topic, str) for topic in topics) else ""
    if required and not supported:
        return CapturePlanView(tr('采集计划不可用'),
                               tr('当前后台的计划校验不可用；请恢复连接或检查后台配置。'),
                               topics_text, blocks_start=True)
    try:
        recipe = recipe_for_topics(topics, duration_s=duration_s)
        plan = resolve_capture_plan(recipe)
    except CapturePlanError as error:
        if supported:
            return CapturePlanView(tr('采集计划需要修正'), _plan_error(error),
                                   topics_text, blocks_start=True)
        summary = tr('按控制器当前配置录制')
    else:
        summary = _summary(plan)
        if supported:
            return CapturePlanView(
                summary,
                tr('按可用数据录制，未连接设备不阻止开始。'),
                topics_text, plan=plan, expected_topics_fingerprint=topics_fingerprint(topics),
            )
    return CapturePlanView(summary, tr('旧版后台未启用计划校验；按控制器配置录制。'), topics_text)
