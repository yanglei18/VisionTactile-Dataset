"""One explicit save-command entry point for every calibration page.

This object only tracks a submitted GUI request until the existing worker
acknowledges it. It owns no files, sensor state, queues, or automatic retries.
"""
from .calibration_presentation import save_actions_allowed


class CalibrationSaveActions:
    def __init__(self, submit):
        self._submit = submit
        self._pending = None

    @property
    def pending(self):
        return self._pending is not None

    @staticmethod
    def _evidence(snapshot):
        runtime = snapshot.get("runtime") or {}
        calibration = snapshot.get("calibration") or {}
        return (runtime.get("session_id"), calibration.get("clear_generation"),
                runtime.get("shape_command_generation", 0),
                runtime.get("shape_rejected_count", 0))

    def observe(self, snapshot):
        if self._pending is not None and self._pending != self._evidence(snapshot):
            self._pending = None

    def allowed(self, snapshot, storage, *, clearing=False, official=True, other_pending=False):
        self.observe(snapshot)
        return save_actions_allowed(snapshot, storage, clearing=clearing,
                                    pending=self.pending or other_pending, official=official)

    def submit(self, name, snapshot, storage, *, clearing=False, official=True,
               other_pending=False, **args):
        if name not in {"retry_calibration_save", "retry_calibration_save_to"}:
            raise ValueError("Unsupported calibration save action")
        if not self.allowed(snapshot, storage, clearing=clearing, official=official,
                            other_pending=other_pending):
            return False
        evidence = self._evidence(snapshot)
        if not self._submit(name, **args):
            return False
        self._pending = evidence
        return True
