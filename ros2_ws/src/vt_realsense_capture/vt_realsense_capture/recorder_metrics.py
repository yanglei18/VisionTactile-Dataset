"""Bounded reader for observations made inside the real recorder process.

Writer receipt and storage-plugin write return are different boundaries. Neither
proves acquisition completeness or durable media; closed bags need readback.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import stat
import time
from typing import Callable


SEMANTICS = "storage_plugin_write_returned_not_fsync"
_COUNTERS = (
    "writer_received_messages", "writer_received_bytes",
    "storage_returned_messages", "storage_returned_bytes",
    "storage_returned_last_monotonic_ns",
)


def _uint(value: object) -> bool:
    return type(value) is int and 0 <= value <= (1 << 64) - 1


class RecorderMetricsReader:
    """Reject foreign/stale files and compute rates from producer sample times."""

    def __init__(self, *, session_id: str, bag_path: Path, instance_id: str,
                 topics: tuple[str, ...], clock: Callable[[], int] = time.monotonic_ns):
        self._binding = dict(schema_version=1, session_id=session_id,
                             bag_path=str(bag_path), recorder_instance_id=instance_id)
        self._path = bag_path.parent / "recorder-metrics.json"
        self._topics = frozenset(topics)
        self._clock = clock
        self._previous: dict | None = None
        self._rate: float | None = None

    def read(self) -> dict:
        try:
            fd = os.open(self._path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC | os.O_NONBLOCK)
            with os.fdopen(fd, "rb") as stream:
                info = os.fstat(stream.fileno())
                if not stat.S_ISREG(info.st_mode) or info.st_size > 1024 * 1024:
                    raise ValueError("metrics snapshot must be a bounded regular file")
                payload = stream.read(1024 * 1024 + 1)
                after = os.fstat(stream.fileno())
                if (after.st_size, after.st_mtime_ns, after.st_ctime_ns) != (
                        info.st_size, info.st_mtime_ns, info.st_ctime_ns):
                    raise ValueError("metrics snapshot changed during read")
            if len(payload) > 1024 * 1024:
                raise ValueError("metrics snapshot exceeds size limit")
            value = json.loads(payload)
            self._validate(value)
            now = self._clock()
            observed = value["observed_monotonic_ns"]
            if observed > now + 100_000_000:
                raise ValueError("metrics snapshot has a future monotonic timestamp")
            if not value["closed"] and now - observed > 2_500_000_000:
                raise ValueError("metrics snapshot is stale")
            if self._previous is not None:
                if value["pid"] != self._previous["pid"] or (
                        self._previous["closed"] and not value["closed"]):
                    raise ValueError("recorder process identity or close state changed")
                elapsed = observed - self._previous["observed_monotonic_ns"]
                old = {topic["topic"]: topic for topic in self._previous["topics"]}
                for topic in value["topics"]:
                    if any(topic[key] < old[topic["topic"]][key] for key in _COUNTERS):
                        raise ValueError("recorder counters regressed")
                if elapsed < 0:
                    raise ValueError("recorder sample time regressed")
                if elapsed == 0 and value != self._previous:
                    raise ValueError("recorder snapshot changed without advancing sample time")
                if elapsed > 0:
                    written = sum(topic["storage_returned_bytes"] for topic in value["topics"])
                    before = sum(topic["storage_returned_bytes"] for topic in old.values())
                    self._rate = (written - before) * 1000.0 / elapsed
            self._previous = value
            return {**value, "available": True, "rate_available": self._rate is not None,
                    "storage_returned_mb_s": self._rate or 0.0,
                    "detail": "Storage plugin writes returned; independent bag readback still required"}
        except (OSError, ValueError, TypeError, KeyError) as error:
            return {**self._binding, "available": False, "rate_available": False,
                    "detail": f"Recorder metrics unavailable: {error}"}

    def _validate(self, value: object) -> None:
        if not isinstance(value, dict) or any(value.get(k) != v for k, v in self._binding.items()):
            raise ValueError("metrics session/bag/instance binding does not match")
        if value.get("storage_write_semantics") != SEMANTICS:
            raise ValueError("unknown storage observation semantics")
        if type(value.get("closed")) is not bool:
            raise ValueError("missing recorder close state")
        for key in ("pid", "observed_monotonic_ns", "writer_errors", "storage_errors"):
            if not _uint(value.get(key)):
                raise ValueError(f"invalid {key}")
        if not value["pid"] or not value["observed_monotonic_ns"]:
            raise ValueError("recorder identity/time is missing")
        topics = value.get("topics")
        if not isinstance(topics, list) or len(topics) != len(self._topics) or len(topics) > 512:
            raise ValueError("unexpected recorder topic count")
        seen = set()
        for topic in topics:
            if not isinstance(topic, dict) or not isinstance(topic.get("topic"), str):
                raise ValueError("invalid recorder topic")
            name = topic["topic"]
            if name in seen or name not in self._topics:
                raise ValueError("foreign/duplicate recorder topic")
            seen.add(name)
            if any(not _uint(topic.get(key)) for key in _COUNTERS):
                raise ValueError("invalid recorder counter")
            if (topic["storage_returned_messages"] > topic["writer_received_messages"] or
                    topic["storage_returned_bytes"] > topic["writer_received_bytes"] or
                    topic["storage_returned_last_monotonic_ns"] > value["observed_monotonic_ns"]):
                raise ValueError("inconsistent recorder counters")


def project_recorder_metrics(message, data: dict | None, stamp, topic_factory,
                             *, session_id: str = ""):
    """Keep unavailable explicit; do not promote missing data to observed zero."""
    message.header.stamp = stamp
    message.schema_version = 1
    message.session_id = session_id
    if not data:
        message.detail = "No recorder observations available"
        return message
    for name in ("schema_version", "session_id", "bag_path", "recorder_instance_id",
                 "observed_monotonic_ns", "available", "closed", "rate_available",
                 "storage_returned_mb_s", "writer_errors", "storage_errors",
                 "storage_write_semantics", "detail"):
        if name in data:
            setattr(message, name, data[name])
    message.recorder_pid = data.get("pid", 0)
    message.topics = []
    for row in data.get("topics", ()):
        topic = topic_factory()
        topic.topic = row["topic"]
        for key in _COUNTERS:
            setattr(topic, key, row[key])
        message.topics.append(topic)
    return message
