"""Pure ownership and lifecycle decisions for managed capture observations.

A session directory proves an intent was allocated, not that recording began.
Only the controller instance, START request and session together bind ownership.
Terminal decisions additionally require the absence of a possibly live recorder.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping


@dataclass(frozen=True)
class TrialObservation:
    state: str
    detail: str
    bind_context: bool = False
    start_effect: bool = False
    start_failed: bool = False
    stop_effect: bool = False
    freeze_finish: bool = False


def observe_trial(
    trial: Mapping, context: Mapping, *, start_rejection: str | None = None,
    pending_stop: str | None = None, expected_start_revision: int | None = None,
    start_rejection_code: str | None = None,
) -> TrialObservation | None:
    """Interpret an atomic, validated snapshot without sending any commands.

    A negative receipt alone cannot erase an active process or a changed
    controller instance. A matching IDLE snapshot with no recorder establishes a
    failed START, including a lost negative receipt, and leaves quality unassessed.
    A snapshot for a different request provides no evidence for a pending intent.
    """
    if trial["controller_instance_id"] != context["controller_instance_id"]:
        return TrialObservation("UNKNOWN", "Controller instance changed; session ownership is unresolved, no command was retried.")
    if not trial["session_id"] and start_rejection_code == "request_id_conflict":
        return TrialObservation("FAILED", start_rejection or "START request ID belongs to another command.", start_failed=True)
    if trial["start_request_id"] != context["start_request_id"]:
        if trial["session_id"]:
            return TrialObservation("UNKNOWN", "Controller reports another START while the local session has no confirmed termination.")
        if start_rejection is not None and not context["recorder_alive"] and context["state"] in (0, 5, 6):
            return TrialObservation("FAILED", start_rejection, start_failed=True)
        if start_rejection is not None:
            return TrialObservation("UNKNOWN", "Rejected START has no matching session observation; active process ownership is unresolved.")
        return None
    if trial["session_id"] and trial["session_id"] != context["session_id"]:
        return TrialObservation("UNKNOWN", "Controller reported a different session for the same START; confirmed ownership was preserved.")

    # A matching request ID is insufficient if this session already existed at
    # admission. The durable command guards survive manager restart; a newly
    # allocated session necessarily advances the controller's state revision.
    if (not trial["session_id"] and context["session_id"] and expected_start_revision is not None
            and context["state_revision"] <= expected_start_revision):
        if start_rejection is not None and not context["recorder_alive"] and context["state"] in (0, 5, 6):
            return TrialObservation("FAILED", start_rejection, start_failed=True)
        return TrialObservation("UNKNOWN", "Observed session predates this START intent; ownership is unresolved.")

    state = context["state"]
    if state in (0, 5, 6) and context["recorder_alive"]:
        return TrialObservation("UNKNOWN", "Controller reports an idle or terminal state but the recorder is still alive; termination is not confirmed. " + context["detail"])
    if state == 0:
        if trial["session_id"]:
            return TrialObservation("UNKNOWN", "Controller returned to IDLE for a previously observed session; terminal completion was not confirmed.")
        return TrialObservation("FAILED", context["detail"], bind_context=True, start_failed=True)
    if not context["session_id"]:
        return TrialObservation("UNKNOWN", "Controller has no session identity for the observed START; ownership is unresolved.")

    observed = {1: "STARTING", 2: "STARTING", 3: "RECORDING", 4: "STOPPING", 5: "COMPLETE", 6: "FAILED"}[state]
    if observed == "RECORDING" and pending_stop is not None:
        observed = "UNKNOWN" if pending_stop == "unknown" else "STOPPING"
    return TrialObservation(
        observed, context["detail"], bind_context=True, start_effect=True,
        stop_effect=state in (4, 5, 6), freeze_finish=state in (5, 6),
    )
