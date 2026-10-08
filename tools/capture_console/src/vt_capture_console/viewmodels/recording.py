"""Only enable controls for explicit, fresh backend capabilities and ownership."""

TRIAL_LABELS = {
    "STARTING": "启动中", "RECORDING": "录制中", "STOPPING": "正在收尾",
    "COMPLETE": "录制结束", "FAILED": "失败", "UNKNOWN": "状态待核实",
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
