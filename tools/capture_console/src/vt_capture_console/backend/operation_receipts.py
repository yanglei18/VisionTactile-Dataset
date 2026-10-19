"""Bounded per-request outcomes, shared by independent command consumers.

The IPC owner serializes access. Observations can update only locally submitted
requests; another client's sampled receipt never replaces a caller's outcome.
"""
from collections import OrderedDict
from copy import deepcopy


MAX_OPERATION_RECEIPTS = 128


class OperationReceipts:
    def __init__(self):
        self._values = OrderedDict()
        self._latest_id = None

    @property
    def latest(self):
        return deepcopy(self._values.get(self._latest_id))

    def record(self, operation, *, submitted=False, protected=()):
        identifier = operation["request_id"]
        if not submitted and identifier not in self._values:
            return
        if (not submitted and str(operation.get("code", "")).lower() == "queued"
                and self._values[identifier].get("state") in {"accepted", "rejected"}
                and str(self._values[identifier].get("code", "")).lower() != "queued"):
            # A command's direct queued receipt can precede the final snapshot
            # packed into that same reply. Never regress confirmed progress.
            return
        if submitted:
            self._latest_id = identifier
        self._values[identifier] = deepcopy(operation)
        self._values.move_to_end(identifier)
        while len(self._values) > MAX_OPERATION_RECEIPTS:
            expired = next(key for key in self._values if key not in protected and key != self._latest_id)
            del self._values[expired]

    def result(self, identifier):
        if not identifier:
            return None
        if identifier in self._values:
            return deepcopy(self._values[identifier])
        # Expiry must be explicit, otherwise a long-lived UI could wait forever.
        return dict(request_id=identifier, state="unknown", code="RECEIPT_UNAVAILABLE", trial_id="",
                    detail="本次命令回执已不可用；请核实后台状态，不会自动重试。")


def operation_for(backend, request_id, management=None):
    """Resolve one caller's receipt, including legacy/demo backend adapters."""
    if not request_id:
        return None
    lookup = getattr(backend, "operation_result", None)
    candidates = ([lookup(request_id)] if callable(lookup) else
                  [getattr(backend, "latest_operation", None)])
    candidates.append((management or {}).get("operation"))
    matching = [value for value in candidates if value and value.get("request_id") == request_id]
    return next((value for value in matching if str(value.get("code", "")).lower() != "queued"),
                matching[0] if matching else None)
