"""Short operator instructions derived from snapshots, with no side effects.

This module does not detect gestures, change capture timing, open devices or
issue commands. ``progress`` is local to the currently described phase, not an
estimator confidence or a product acceptance score. The caller maps the guided
``validation`` command to a 30-second guided quick check; advanced recordings
can still use their existing independently selected duration and markers.
"""
from __future__ import annotations

import math
from numbers import Real


def _seconds(value):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        return None
    return max(0., float(value))


def _guide(step, title, instruction, detail, *, phase="idle", seconds=None,
           duration=None, illustration="still", command=None, text="请按上方提示操作"):
    seconds = _seconds(seconds)
    duration = _seconds(duration)
    progress = (max(0., min(1., 1. - seconds / duration))
                if seconds is not None and duration else 1. if phase == "complete" else 0.)
    return dict(step=step, title=title, instruction=instruction, detail=detail,
                phase=phase, seconds_left=seconds, progress=progress,
                illustration=illustration, primary_command=command,
                primary_text=text, primary_enabled=command is not None)


def _neutral_advice(message):
    hints = []
    for terms, hint in [
        (("frames_neutral_unobservable", "explicit_neutral_gravity_nearly_parallel_to_hinge"),
         "参考姿势不利于测角：先保持手指形状，改变整只手的倾斜，再停稳重采参考；不要在原姿势反复静止。"),
        (("neutral_gyro_not_quiet",), "陀螺数据不够平稳：把手掌和手指一起放稳，完整保持 5 秒。"),
        (("neutral_acceleration_not_stable_gravity_like",),
         "加速度波动或幅值不符：先放稳手套；若反复出现，需检查固定和单位配置。"),
        (("neutral_pair_count_insufficient", "no_count"),
         "有效成对样本不足：等两路数据连续后，重新完整采集 5 秒。"),
        (("neutral_host_span_insufficient",), "有效数据覆盖时间不足：保持连接，等准备结束后完整采满 5 秒。"),
        (("neutral_raw_near_limit",), "陀螺读数接近量程：先停止快速动作；若静止仍出现，需检查采集配置。"),
        (("selected_pair_stale", "selected_pair_host_gap", "pair_host_timestamp_mismatch", "pair_wire_order_interrupted"),
         "数据曾中断或配对失效，旧参考已失效；当前数据恢复后需重新采参考。"),
    ]:
        if any(term in message for term in terms):
            hints.append(hint)
    return " ".join(hints)


def _recovery_guide(recovery, runtime):
    """Show the controller's structured decision, not a second guess from text."""
    command = recovery["suggested_action"]
    step = {"neutral": 1, "motion": 2, "export": 5}[command]
    saved = bool(runtime.get("last_export")) and runtime.get("unsaved_experiment") is False
    if command == "export" and saved:
        return _guide(5, recovery["title"], "本次失败记录已保存，先不要按原动作反复采集。",
                      recovery["instruction"] + " " + recovery["detail"],
                      phase="waiting", illustration="export", text="记录已保存，待定位原因")
    return _guide(step, recovery["title"], recovery["instruction"], recovery["detail"],
                  illustration=recovery["illustration"], command=command,
                  text={"neutral": "按提示重采静止参考", "motion": "按提示重采自然运动",
                        "export": "保存失败记录，停止盲目重试"}[command])


def _motion_advice(message):
    if "near_limit" in message:
        return "上次动作接近陀螺量程：放慢屈伸和转掌；若仍发生，检查量程配置。"
    if "sign_unobservable" in message:
        return "上次轴方向信息不足：屈伸时轻缓改变整手倾斜，不要始终保持同一掌向。"
    if "insufficient" in message or "运动数据不足" in message:
        return "上次运动信息不足：让两段都自然屈伸，并小幅转掌，完整记录 20 秒。"
    if "inconsistent" in message or "不一致" in message:
        return "两路运动与当前单铰链假设不一致：先检查固定和所选节点，再放慢动作重采。"
    if "nodes or scales" in message:
        return "候选节点或单位与当前配置不一致：先检查节点和单位设置，不要只反复做动作。"
    # Session already supplies a short Chinese action after the raw status/code.
    for part in message.split("。")[1:]:
        if any(term in part for term in ("请", "检查", "改变", "放慢")):
            return part.split("；")[0].strip() + "。"
    return "保留本次记录；检查传感器是否固定，再让两段缓慢屈伸并小幅转掌，重采 20 秒。"


def _validation_guide(runtime, finger, remaining):
    if runtime.get("validation_guided_quickcheck", False):
        elapsed = max(0., min(30., 30. - remaining))
        # Time-based coaching only: these stages never claim a gesture was seen.
        stages = [
            (0., 3., "准备同一个参考姿势", f"将{finger}摆回刚才选定的舒适参考姿势。",
             "现在先调整姿势；倒计时后保持不动，不要求伸直到零度。", "prepare", "still"),
            (3., 5., "保持参考姿势", "手指和手掌都保持不动。",
             "正在记录本次试跑参考；不会重设标定零位。", "capture", "hold"),
            (5., 20., "慢屈伸，同时小幅转掌", f"缓慢弯曲、伸展{finger}，同时小幅改变手掌朝向。",
             "动作以舒适为准，不要猛甩；观察模型是否随动。", "capture", "flex"),
            (20., 25., "保持自然弯曲", f"把{finger}停在一个舒适的弯曲姿势，保持不动。",
             "不要为了画面回零而伸直；本段只记录保持时的输出。", "capture", "hold"),
            (25., 28., "返回刚才的参考姿势", "慢慢回到试跑开始时的同一个手型。",
             "按真实手型回位，不追着屏幕角度调整。", "capture", "return"),
            (28., 30., "保持参考，等待记录结束", "保持返回的参考姿势不动。",
             "结束后点击导出；完成记录不等于验证通过。", "capture", "hold"),
        ]
        selected = next((stage for stage in stages if elapsed < stage[1]), stages[-1])
        start, end, title, instruction, detail, phase, illustration = selected
        return _guide(4, title, instruction,
                      f"{detail} 全程还剩 {remaining:.1f} 秒；这是操作提示，不是动作检测。",
                      phase=phase, seconds=max(0., end-elapsed), duration=end-start,
                      illustration=illustration, text="试跑记录中，请跟随提示")
    now = _seconds(runtime.get("validation_now_s"))
    markers = runtime.get("validation_markers", [])
    active = next((marker for marker in markers if isinstance(marker, dict)
                   and marker.get("status") == "recording"), None) if isinstance(markers, list) else None
    if active is not None and now is not None:
        start, end = _seconds(active.get("start_host_monotonic_s")), _seconds(active.get("end_host_monotonic_s"))
        if start is not None and end is not None and end > start:
            descriptions = {
                "reference": ("参考手型", "摆好一个之后能重复的舒适手型，然后保持不动。", "hold"),
                "return": ("返回参考", "回到同一参考手型，然后保持不动；不要追着屏幕归零。", "return"),
                "hold": ("固定保持", "保持现在的手指弯曲和手掌朝向，直到本段结束。", "hold"),
                "common_rotation": ("固定指形转掌", "保持手指弯曲不变，缓慢改变整手朝向。", "rotate"),
            }
            title, instruction, illustration = descriptions.get(active.get("kind"),
                ("当前动作", "按本次标记的动作配合，等待倒计时结束。", "hold"))
            preparing = now < start
            return _guide(4, f"{title}：{'准备' if preparing else '采集中'}", instruction,
                          "准备与采集分开计时；动作由你声明，程序不判定真实动作是否成功。",
                          phase="prepare" if preparing else "capture",
                          seconds=max(0., (start if preparing else end)-now),
                          duration=1. if preparing else end-start,
                          illustration=illustration, text="当前标记录制中")
    return _guide(4, "独立验证录制中", f"按计划屈伸{finger}、保持或转掌；需要统计时添加对应动作标记。",
                  "在高级区选择参考、回位、保持或转掌标记。这里只保存记录，不自动判定动作成功。",
                  phase="capture", seconds=remaining, duration=runtime.get("validation_duration_s"),
                  illustration="flex", text="验证录制中")


def guide_for(state, finger_name="拇指", stale_after_s=.5):
    """Return a pure, short guide for a session snapshot plus optional runtime.

    Steps 0..5 are connection, reference, motion, direction, quick check and
    export. ``seconds_left`` is the described phase's countdown; a preparation
    countdown never masquerades as capture time. A stale/missing selected pair
    disables action prompts regardless of the generic serial-connected flag.

    ``runtime.validation_for_current_candidate`` overrides the legacy attempt
    count, preventing an older candidate's validation from skipping new work.
    Export completion means current records were saved, never product approval.
    No input dictionaries are mutated and no clock is sampled.
    """
    if not isinstance(state, dict):
        raise ValueError("state must be a session snapshot dictionary")
    if not isinstance(finger_name, str) or not finger_name.strip():
        raise ValueError("finger_name must be a nonempty string")
    stale = _seconds(stale_after_s)
    if stale is None or stale <= 0:
        raise ValueError("stale_after_s must be a positive finite number")
    finger = finger_name.strip()
    runtime = state.get("runtime") if isinstance(state.get("runtime"), dict) else {}
    name = state.get("state", "disconnected")
    job = runtime.get("background_job")
    remaining = _seconds(runtime.get("validation_remaining_s"))
    active = name in {"prepare_neutral", "capturing_neutral", "prepare_motion", "capturing_motion",
                      "prepare_direction", "capturing_direction", "fitting"}
    if job == "export":
        return _guide(5, "正在导出", "保持窗口开启，等待保存完成。",
                      "正在保存原始记录、候选和在线输出；不要重复点击导出。",
                      phase="processing", illustration="export", text="正在保存，请稍候")
    if name == "fitting" or state.get("fit_pending") or job == "fit":
        return _guide(2, "正在计算运动标定", "自然运动采集已结束；保持连接，等待计算结果。",
                      "现在不用继续重复动作，也不要重做静止；计算完成后进入方向确认。",
                      phase="processing", illustration="flex", text="正在计算，请稍候")
    recovery = state.get("recovery")
    if (name in {"neutral_ready", "needs_neutral"} and remaining is None
            and isinstance(recovery, dict) and recovery.get("suggested_action") in {"neutral", "motion", "export"}):
        # Saving a failed experiment is useful even after data becomes stale;
        # a suggested new capture must still wait for the live pair below.
        age = _seconds(state.get("pair_age_s"))
        fresh = (runtime.get("connected", state.get("connected", False)) is True
                 and age is not None and age <= stale and state.get("matched_pairs", 1) != 0)
        if recovery["suggested_action"] == "export" or fresh:
            return _recovery_guide(recovery, runtime)
    if not active and remaining is None:
        validated = (runtime["validation_for_current_candidate"] is True
                     if "validation_for_current_candidate" in runtime
                     else isinstance(runtime.get("validation_attempts"), Real)
                     and runtime["validation_attempts"] > 0)
        if validated and runtime.get("last_export") and runtime.get("unsaved_experiment") is False:
            return _guide(5, "当前记录已导出", "本次已有记录已保存，可查看导出目录。",
                          "保存完成不代表标定或精度验收通过。若继续采集，新记录仍需再次导出。",
                          phase="complete", illustration="export", text="当前记录已保存")
        if validated:
            return _guide(5, "本次试跑已记录", "点击「导出本次实验」，保存标定、原始数据和实际在线角度。",
                          "录制结束不是验证通过；导出后才能逐项核验。无需重复静止参考。",
                          illustration="export", command="export", text="导出本次实验")
    connected = runtime.get("connected", state.get("connected", False)) is True
    if not connected:
        return _guide(0, "先连接手套", "确认手套已通电并接入电脑，选择串口后点击「连接手套」。",
                      "连接后先等待所选两个节点的连续数据，再开始动作。",
                      illustration="connect", command="connect", text="连接手套")
    age = _seconds(state.get("pair_age_s"))
    fresh = age is not None and age <= stale and state.get("matched_pairs", 1) != 0
    if not fresh:
        return _guide(0, "等待两个节点的实时数据", "先不要开始动作；检查手套供电、线缆和所选节点的数据。",
                      "串口已连接不等于收到成对数据。数据恢复后，再按界面的下一步提示操作。",
                      phase="waiting", illustration="connect", text="等待成对数据")
    if remaining is not None:
        return _validation_guide(runtime, finger, remaining)
    if name in {"prepare_neutral", "capturing_neutral"}:
        preparing = name == "prepare_neutral"
        return _guide(1, "静止参考：准备" if preparing else "静止参考：采集中",
                      f"摆好{finger}和手掌的舒适参考姿势，现在可以调整。" if preparing else
                      f"保持现在的{finger}和手掌不动，直到倒计时结束。",
                      "准备结束后保持 5 秒；现在尚未采集参考。" if preparing else
                      "正在采集参考，不需要强行伸直；只检查当前两个节点。",
                      phase="prepare" if preparing else "capture",
                      seconds=state.get("countdown_s" if preparing else "capture_remaining_s"),
                      duration=3. if preparing else 5., illustration="still", text="请跟随倒计时")
    if name in {"prepare_motion", "capturing_motion"}:
        preparing = name == "prepare_motion"
        return _guide(2, "自然运动：准备" if preparing else "自然运动：采集中",
                      f"摆好手，等倒计时结束后再缓慢屈伸{finger}。" if preparing else
                      f"慢慢弯曲、伸展{finger}，让两段都自然运动，同时小幅转动手掌。",
                      "现在尚未采集动作；接下来连续做 20 秒。" if preparing else
                      "不用固定次数，不要猛甩；不能只转整手而完全不屈伸。",
                      phase="prepare" if preparing else "capture",
                      seconds=state.get("countdown_s" if preparing else "capture_remaining_s"),
                      duration=3. if preparing else 20., illustration="flex", text="请跟随倒计时")
    if name in {"prepare_direction", "capturing_direction"}:
        preparing = name == "prepare_direction"
        return _guide(3, "屈曲方向：准备" if preparing else "向掌心弯一次",
                      "先保持舒服姿势，等倒计时结束后再动。" if preparing else
                      f"先把{finger}向掌心明显弯一次，随后可以缓慢返回。",
                      "不必回到屏幕的零度；准备结束后记录 4 秒。" if preparing else
                      "只看第一次连贯屈曲方向；不用反复往返，不会自动归零。",
                      phase="prepare" if preparing else "capture",
                      seconds=state.get("countdown_s" if preparing else "capture_remaining_s"),
                      duration=3. if preparing else 4., illustration="flex_once", text="请先向掌心弯曲")
    if name == "neutral_ready" and not state.get("needs_neutral", False):
        message = str(state.get("status_message", ""))
        if "拟合未通过" in message or "运动数据不足" in message:
            return _guide(2, "运动标定需调整，静止参考保留",
                          f"不用重做静止；按下面建议调整，再点击「重新采自然运动」屈伸{finger}。",
                          _motion_advice(message), illustration="flex", command="motion", text="重新采自然运动")
        return _guide(2, "静止参考已成功", f"无需重复静止。点击「开始自然运动」，准备后慢慢屈伸{finger}。",
                      "接下来连续 20 秒：两段都自然运动，并少量转掌；不要求固定次数。",
                      illustration="flex", command="motion", text="开始自然运动")
    if name == "candidate_ready" and not state.get("needs_neutral", False):
        if not state.get("preview_valid", False):
            return _guide(3, "等待可观察的当前角度", "轻轻改变整手倾斜，然后短暂保持舒适手型。",
                          "当前还没有可用角度；暂不确认方向，不要反复重做静止或追着画面归零。",
                          phase="waiting", illustration="rotate", text="等待当前角度")
        direction = state.get("direction") if isinstance(state.get("direction"), dict) else {}
        if direction.get("status") == "confirmed":
            return _guide(4, "方向已确认，开始短试跑", "点击「开始 30 秒试跑」，跟随分段提示做动作。",
                          "依次参考保持、慢屈伸小幅转掌、弯曲保持、返回参考；只记录，不自动判通过。",
                          illustration="flex", command="validation", text="开始 30 秒试跑")
        observer = state.get("observer") if isinstance(state.get("observer"), dict) else {}
        if not observer.get("current_angle_observed", False):
            return _guide(3, "当前角度仅预测，先恢复观测", "暂停快速动作，轻转手掌后短暂保持，再确认屈曲方向。",
                          "当前观测不足，暂不开始方向动作；保留数学角，不会自动归零。",
                          phase="waiting", illustration="rotate", text="等待角度观测")
        return _guide(3, "确认向掌心的屈曲方向", f"点击「确认屈曲方向」，准备后把{finger}先向掌心明显弯一次。",
                      "从舒服姿势开始即可，不必先伸直或达到零度；完成后可缓慢返回。",
                      illustration="flex_once", command="direction", text="确认屈曲方向")
    message = str(state.get("status_message", ""))
    advice = _neutral_advice(message)
    if advice or "静止参考未通过" in message:
        return _guide(1, "静止参考需要重采", "先按下面的原因调整，准备好后点击「重新采静止参考」。",
                      advice or "这次参考数据未满足条件；保持舒适手型，再完整保持 5 秒。原始记录已保留。",
                      illustration="still", command="neutral", text="重新采静止参考")
    return _guide(1, "先采一个静止参考", f"把{finger}放在舒服、之后能重复的参考姿势，点击「采集静止参考」。",
                  "先有 3 秒准备，再保持不动 5 秒；不要求强行伸直，也不等于解剖零位。",
                  illustration="still", command="neutral", text="采集静止参考")
