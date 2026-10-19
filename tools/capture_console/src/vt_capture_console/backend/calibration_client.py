"""One explicit calibration command at a time; no ROS, polling or task ownership."""
from __future__ import annotations

import time

from ..i18n import tr
from ..i18n.messages import message as diagnostic
from ..viewmodels.calibration import calibration_view, restorable_revision
from .operation_receipts import operation_for


class CalibrationClient:
    def __init__(self, backend, *, clock=time.monotonic):
        self.backend = backend
        self._clock = clock
        self._snapshot = {}
        self._pending = None
        self.notice = ""

    @property
    def pending(self):
        return self._pending is not None

    def observe(self, snapshot):
        self._snapshot = snapshot
        pending = self._pending
        if pending is None:
            return
        if snapshot.get("instance_id") != pending["manager_id"]:
            self.notice = tr('后台已更换；请核实标定状态，未自动重试。')
            self._pending = None
            return
        receipt = operation_for(self.backend, pending["request_id"], snapshot.get("management"))
        session_id = calibration_view(snapshot).session_id
        if receipt and str(receipt.get("code", "")).lower() != "queued":
            if receipt.get("state") in {"rejected", "unknown"}:
                self.notice = tr('标定操作未完成：{v0}', v0=diagnostic(receipt.get("detail") or receipt.get("code", "")))
                self._pending = None
            elif receipt.get("state") == "accepted" and (pending["command"] != "calibration.start"
                    or session_id and session_id != pending["session_id"]):
                self._pending = None
                self.notice = ""
        if self._pending is not None and self._clock() - pending["started"] > 15.:
            self.notice = tr('尚未确认操作结果；请核实后台状态后重试。')
            self._pending = None

    def _submit(self, command, params):
        if self.pending:
            return False
        try:
            request_id = self.backend.submit(command, {
                **params, "expected_manager_instance_id": self._snapshot["instance_id"]})
        except (OSError, RuntimeError, ValueError) as error:
            self.notice = tr('未提交：{v0}', v0=diagnostic(str(error)))
            return False
        self._pending = dict(request_id=request_id, command=command,
                             manager_id=self._snapshot["instance_id"],
                             session_id=calibration_view(self._snapshot).session_id,
                             started=self._clock())
        self.notice = tr('正在等待后台确认')
        return True

    def start(self, params):
        if not calibration_view(self._snapshot).can_start:
            return False
        return self._submit("calibration.start", params)

    def cancel(self):
        view = calibration_view(self._snapshot)
        return (self._submit("calibration.cancel", {"session_id": view.session_id})
                if view.can_cancel else False)

    def retry_publish(self):
        view = calibration_view(self._snapshot)
        return (self._submit("calibration.retry_publish", {"session_id": view.session_id})
                if view.can_retry_publish else False)

    def restore_previous(self):
        target = restorable_revision(self._snapshot)
        if target is None:
            return False
        camera, previous, current = target
        return self._submit("calibration.restore", dict(camera_name=camera, revision=previous,
                                                        expected_active_revision=current))
