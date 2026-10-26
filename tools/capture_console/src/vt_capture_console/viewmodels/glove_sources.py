"""Short source summaries from the displayed frame, never local configuration."""
from ..i18n import tr


def source_summary(frame):
    from vt_capture_core.glove_sources import glove_sources

    sources = glove_sources((frame or {}).get("glove_sources", {}))
    if not sources:
        return tr("该帧未提供骨段来源信息。")
    text = tr("当前显示：IMU 观测骨段 {observed}/16；含推断的模型关节 {inferred}/16。",
              observed=sum(sources["segment_observed_mask"]), inferred=sum(sources["inferred_mask"]))
    if "reference_node_id" in sources:
        text += tr(" 掌心参考：IMU {node}。", node=sources["reference_node_id"])
    return text
