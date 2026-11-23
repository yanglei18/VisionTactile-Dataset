"""Compare the recorder's final write counters to records read independently from MCAP."""
from __future__ import annotations

import json
import os
from pathlib import Path
import stat


def _read_json(path, maximum):
    try:
        fd = os.open(path, os.O_NOFOLLOW | os.O_NONBLOCK | os.O_RDONLY)
    except FileNotFoundError:
        return None
    with os.fdopen(fd, "rb") as handle:
        if not stat.S_ISREG(os.fstat(fd).st_mode):
            raise ValueError("Invalid recording evidence file")
        payload = handle.read(maximum + 1)
    if len(payload) > maximum:
        raise ValueError("Recording evidence exceeds size limit")
    document = json.loads(payload)
    if not isinstance(document, dict):
        raise ValueError("Recording evidence must be an object")
    return document


def read_recorder_counters(bag):
    """Bind to the original path, or to the matching controller journal in a moved trial."""
    document = _read_json(bag.parent / "recorder-metrics.json", 1024 * 1024)
    if document is None:
        return None
    if type(document.get("schema_version")) is not int or document["schema_version"] != 1:
        raise ValueError("Unsupported recorder metrics schema")
    session = document.get("session_id")
    path = document.get("bag_path")
    if (not isinstance(session, str) or session != bag.parent.name or not isinstance(path, str)
            or not Path(path).is_absolute() or Path(path).parts[-2:] != bag.parts[-2:]):
        raise ValueError("Recorder metrics do not identify this bag and session")
    if path != str(bag):
        journal = _read_json(bag.parent / "controller_session.json", 2 * 1024 * 1024)
        if not journal or journal.get("output_path") != path or journal.get("session_id") != session:
            raise ValueError("Moved bag has no matching controller session journal")
    if (not isinstance(document.get("recorder_instance_id"), str) or not document["recorder_instance_id"]
            or document.get("storage_write_semantics") != "storage_plugin_write_returned_not_fsync"
            or type(document.get("closed")) is not bool):
        raise ValueError("Unknown recorder identity, write semantics or close state")
    for key in ("pid", "observed_monotonic_ns"):
        if type(document.get(key)) is not int or document[key] <= 0:
            raise ValueError("Invalid recorder process identity or sample clock")
    rows = document.get("topics")
    if not isinstance(rows, list) or len(rows) > 512:
        raise ValueError("Invalid recorder topic inventory")
    names = set()
    for row in rows:
        if not isinstance(row, dict) or not isinstance(row.get("topic"), str) or not row["topic"].startswith("/") or row["topic"] in names:
            raise ValueError("Malformed or duplicate recorder topic")
        names.add(row["topic"])
        for key in ("writer_received_messages", "storage_returned_messages", "writer_received_bytes",
                    "storage_returned_bytes", "storage_returned_last_monotonic_ns"):
            if type(row.get(key)) is not int or row[key] < 0:
                raise ValueError("Invalid recorder message count")
    for key in ("writer_errors", "storage_errors"):
        if type(document.get(key)) is not int or document[key] < 0:
            raise ValueError("Invalid recorder error count")
    return document


def reconcile_recorder(document, observed, metadata):
    if document is None:
        return dict(status="unverified", reason="recorder_counters_unavailable")
    rows = {row["topic"]: row for row in document["topics"]}
    topics = {}
    for topic in sorted(set(rows) | set(observed) | set(metadata)):
        row = rows.get(topic, {})
        writer, stored = row.get("writer_received_messages"), row.get("storage_returned_messages")
        count = observed.get(topic, 0)
        declared = metadata.get(topic, {}).get("recorded_message_count", 0)
        bytes_match = row.get("writer_received_bytes") == row.get("storage_returned_bytes")
        topics[topic] = dict(writer_received=writer, storage_returned=stored, mcap_read=count,
                             writer_received_bytes=row.get("writer_received_bytes"),
                             storage_returned_bytes=row.get("storage_returned_bytes"),
                             metadata_count=declared, counts_match=writer == stored == count == declared and bytes_match)
    status = "verified"
    if not document["closed"]:
        status = "unverified"
    elif (not topics or document["writer_errors"] or document["storage_errors"]
          or not all(row["counts_match"] for row in topics.values())):
        status = "incomplete"
    return dict(status=status, scope="recorder writer input through closed readable bag",
                recorder_instance_id=document["recorder_instance_id"], closed=document["closed"],
                writer_errors=document["writer_errors"], storage_errors=document["storage_errors"], topics=topics)
