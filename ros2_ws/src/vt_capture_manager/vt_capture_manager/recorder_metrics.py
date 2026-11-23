"""Project storage return rates using consecutive observations of the same recorder."""
from __future__ import annotations

from copy import deepcopy

from vt_capture_core.recorder_metrics import MAX_RECORDER_TOPICS, STORAGE_WRITE_SEMANTICS, validate_recorder_metrics


class RecorderMeasurements:
    """Access under the enclosing model lock. All timestamps use the local monotonic clock."""

    def __init__(self):
        self.view = None
        self.stamp = 0

    def observe(self, message, now_ns):
        if message.schema_version != 1:
            raise ValueError("unsupported recorder metrics schema")
        if not message.available:
            if self.view and self.view["session_id"] == message.session_id:
                self.view = None
            return
        if (message.storage_write_semantics != STORAGE_WRITE_SEMANTICS
                or not 0 < message.observed_monotonic_ns <= now_ns
                or len(message.topics) > MAX_RECORDER_TOPICS):
            raise ValueError("invalid recorder counters or clock")
        stamp = message.observed_monotonic_ns
        if stamp <= self.stamp:
            return
        current = dict(session_id=message.session_id, recorder_instance_id=message.recorder_instance_id,
                       closed=message.closed, storage_mb_s=(message.storage_returned_mb_s if message.rate_available else None),
                       writer_errors=message.writer_errors, storage_errors=message.storage_errors,
                       topics=[dict(topic=row.topic, received=row.writer_received_messages,
                                    written=row.storage_returned_messages, written_hz=None) for row in message.topics])
        validate_recorder_metrics(current)
        if self.view and all(self.view[k] == current[k] for k in ("session_id", "recorder_instance_id")):
            previous = {row["topic"]: row for row in self.view["topics"]}
            for row in current["topics"]:
                old = previous.get(row["topic"])
                if old is not None and row["written"] >= old["written"]:
                    row["written_hz"] = (row["written"] - old["written"]) * 1e9 / (stamp - self.stamp)
        validate_recorder_metrics(current)
        self.view, self.stamp = current, stamp

    def snapshot(self, session, now_ns, max_age_ns):
        if self.view and self.view["session_id"] == session and 0 <= now_ns - self.stamp <= max_age_ns:
            return deepcopy(self.view)
        return None
