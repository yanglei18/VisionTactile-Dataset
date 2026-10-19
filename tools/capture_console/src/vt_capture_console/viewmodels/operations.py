"""Coordinate one explicit management command without Qt, polling or retries.

The sender is used only by ``submit`` and is never retained. ``observe`` consumes
confirmed backend snapshots and returns presentation changes; it cannot issue a
command, infer ownership from another client's records, or stop a Recorder.
"""
from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Literal

from ..i18n import LocalizedText, tr
from ..i18n.messages import message as diagnostic
from .recording import TRIAL_LABELS


@dataclass(frozen=True)
class PendingOperation:
    request_id: str
    command: str
    manager_id: str | None
    controller_id: str | None
    batch_ids: frozenset[str]


@dataclass(frozen=True)
class OperationUpdate:
    """One presentation update, consumed once by the application shell."""

    notice: LocalizedText
    notice_page: Literal["recording", "archive"] | None = None
    clear_batch_name: bool = False
    update_batch_selection: bool = False
    batch_id: str | None = None
    refresh_details: bool = False


class OperationCoordinator:
    def __init__(self, *, demo: bool = False):
        self.demo = demo
        self._pending: PendingOperation | None = None

    @property
    def pending(self) -> PendingOperation | None:
        return self._pending

    def submit(
        self, command: str, params: dict, *, sender: Callable[[str, dict], str],
        snapshot: Mapping, management: Mapping,
    ) -> OperationUpdate | None:
        if self._pending is not None:
            return None
        # Capture identity before a synchronous sender (including the demo)
        # changes its state. The immutable context does not retain input dicts.
        manager_id = snapshot.get("instance_id")
        controller_id = (management.get("controller") or {}).get("controller_instance_id")
        batch_ids = frozenset(batch["id"] for batch in management.get("batches", ()))
        try:
            request_id = sender(command, params)
        except (ValueError, RuntimeError, OSError) as error:
            return OperationUpdate(tr('未提交：{v0}', v0=error))
        self._pending = PendingOperation(request_id, command, manager_id, controller_id, batch_ids)
        return OperationUpdate(
            tr('命令正在处理，请等待后台确认；不会自动重复提交。'),
            notice_page="archive" if command == "quality.retry" else "recording",
        )

    def observe(
        self, snapshot: Mapping, management: Mapping, latest_operation: Mapping | None,
    ) -> OperationUpdate | None:
        pending = self._pending
        if pending is None:
            return None
        # Preserve the direct receipt's precedence over the sampled receipt.
        operation = next((candidate for candidate in (latest_operation, management.get("operation"))
                          if candidate and candidate.get("request_id") == pending.request_id), None)
        trial = next((value for value in management.get("trials", ())
                      if operation and value["id"] == operation.get("trial_id")), None)
        if not self.demo and management.get("enabled"):
            context = management.get("controller") or {}
            changed_manager = (pending.manager_id and snapshot.get("instance_id")
                               and pending.manager_id != snapshot["instance_id"])
            changed_controller = (pending.controller_id and context.get("controller_instance_id")
                                  and pending.controller_id != context["controller_instance_id"])
            if (changed_manager or changed_controller) and trial is None:
                self._pending = None
                return OperationUpdate(
                    tr('后台或控制器已更换；上次操作结果待核实，未自动重试。请确认原试次状态。'))
        if operation is None:
            return None

        accepted = operation.get("state") == "accepted"
        rejected_or_unknown = operation.get("state") in {"rejected", "unknown"}
        finished = rejected_or_unknown and operation.get("code") != "QUEUED"
        batch_created = pending.command == "batch.create" and accepted
        quality_queued = pending.command == "quality.retry" and accepted
        if pending.command == "capture.start" and trial:
            finished |= trial["state"] in {"RECORDING", "COMPLETE", "FAILED", "UNKNOWN"}
        if pending.command == "capture.stop" and trial:
            finished |= trial["state"] in {"COMPLETE", "FAILED", "UNKNOWN"}
        if not (finished or batch_created or quality_queued):
            return None

        selected_batch = None
        if batch_created and self.demo:
            # v2 live receipts carry no batch ID. Only the in-memory demo can
            # safely associate exactly one new row with its own batch request.
            new_batches = [batch for batch in management.get("batches", ())
                           if batch["id"] not in pending.batch_ids]
            if len(new_batches) == 1:
                selected_batch = new_batches[0]["id"]
        notice_page = None
        if rejected_or_unknown:
            notice = tr('{v0}：{v1}', v0=diagnostic(operation.get("code", "")),
                        v1=diagnostic(operation.get("detail", "")))
        elif trial:
            notice = tr('{v0}：{v1}。{v2}', v0=trial.get("label", trial["id"]),
                        v1=TRIAL_LABELS.get(trial["state"], trial["state"]),
                        v2=diagnostic(trial.get("detail", "")))
            notice_page = "archive" if pending.command == "quality.retry" else "recording"
        else:
            notice = tr('批次已创建。请选择目标批次，填写试次名称后开始录制。')
            notice_page = "recording"
        self._pending = None
        return OperationUpdate(
            notice, notice_page=notice_page, clear_batch_name=batch_created,
            update_batch_selection=batch_created, batch_id=selected_batch,
            refresh_details=quality_queued,
        )
