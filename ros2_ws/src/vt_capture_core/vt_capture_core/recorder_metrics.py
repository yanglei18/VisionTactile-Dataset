"""Versioned UI projection of recorder counters, separate from device arrival rates."""

from .validation import (ProtocolError, UINT64_MAX, boolean, bounded_integer, bounded_number,
                         limited_text, mapping)


STORAGE_WRITE_SEMANTICS = "storage_plugin_write_returned_not_fsync"
MAX_RECORDER_TOPICS = 512


def validate_recorder_metrics(value):
    if value is None:
        return
    mapping(value, {"session_id", "recorder_instance_id", "closed", "writer_errors", "storage_errors",
                    "storage_mb_s", "topics"}, "recorder_metrics")
    for field in ("session_id", "recorder_instance_id"):
        limited_text(value[field], field, nonempty=True)
    boolean(value["closed"], "closed")
    for field in ("writer_errors", "storage_errors"):
        bounded_integer(value[field], field, maximum=UINT64_MAX)
    if value["storage_mb_s"] is not None:
        bounded_number(value["storage_mb_s"], "storage_mb_s")
    rows = value["topics"]
    if type(rows) is not list or len(rows) > MAX_RECORDER_TOPICS:
        raise ProtocolError("invalid recorder topic inventory")
    names = set()
    for row in rows:
        mapping(row, {"topic", "received", "written", "written_hz"}, "recorder topic")
        name = limited_text(row["topic"], "topic", nonempty=True, maximum=512)
        if not name.startswith("/") or name in names:
            raise ProtocolError("invalid or duplicate recorder topic")
        names.add(name)
        for field in ("received", "written"):
            bounded_integer(row[field], field, maximum=UINT64_MAX)
        if row["written_hz"] is not None:
            bounded_number(row["written_hz"], "written_hz")
