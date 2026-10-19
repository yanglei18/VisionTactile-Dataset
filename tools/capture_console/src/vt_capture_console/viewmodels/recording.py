"""Only enable controls for explicit, fresh backend capabilities and ownership."""

from ..i18n import tr

TRIAL_LABELS = {
    "STARTING": tr('启动中'), "RECORDING": tr('录制中'), "STOPPING": tr('正在收尾'),
    "COMPLETE": tr('录制结束'), "FAILED": tr('失败'), "UNKNOWN": tr('状态待核实'),
}


def owned_active_trial(management):
    context = management.get("controller") or {}
    session = context.get("session_id")
    if not session:
        return None
    return next((trial for trial in management.get("trials", [])
                 if trial.get("session_id") == session and trial.get("state") in
                 {"STARTING", "RECORDING", "STOPPING", "UNKNOWN"}), None)


def recording_access(snapshot, management, pending=False):
    """Return start/stop permission; UI never creates permission from a label."""
    context = management.get("controller")
    enabled = bool(management.get("enabled") and snapshot.get("capabilities", {}).get("record") and context)
    if not enabled or pending:
        return False, False
    unresolved = any(t.get("state") in {"STARTING", "RECORDING", "STOPPING", "UNKNOWN"}
                     for t in management.get("trials", []))
    can_start = context.get("state") in {0, 5, 6} and not context.get("recorder_alive") and not unresolved
    trial = owned_active_trial(management)
    can_stop = bool(context.get("state") == 3 and trial and trial.get("state") == "RECORDING")
    return can_start, can_stop
