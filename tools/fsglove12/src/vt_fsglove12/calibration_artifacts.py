"""Small, independently persisted calibration artifacts and explicit touch TASKs.

This store never consults the bulk-recording latch or accesses hardware. A TASK
is captured evidence, not a fitted shape or a reusable installation calibration.
"""
from __future__ import annotations

from copy import deepcopy
import errno
import json
import math
import os
from pathlib import Path
import shutil
import stat
import tempfile
from uuid import UUID

from .shape_calibration import (CONTACT_SOURCE, FINGERS, MAX_SAMPLES,
                                MAX_SAMPLES_PER_FINGER, _array, _sha,
                                _validate_upstream, _write_json, validate_contacts)


MAX_ARTIFACT_BYTES = 16 * 1024 * 1024
FREE_SPACE_MARGIN_BYTES = 16 * 1024 * 1024
TASK_KIND = "fsglove_touch_task"


def _encoded(document):
    try:
        value = (json.dumps(document, indent=2, ensure_ascii=False, allow_nan=False) + "\n").encode("utf-8")
    except (TypeError, ValueError, OverflowError, RecursionError, UnicodeError) as exc:
        raise ValueError(f"Calibration artifact must contain finite JSON data: {exc}") from exc
    if len(value) > MAX_ARTIFACT_BYTES:
        raise ValueError("Calibration artifact exceeds the 16 MiB size limit")
    return value


def _storage_error(exc, path):
    return OSError(exc.errno, f"Calibration artifact storage unavailable: {exc.strerror or exc}", str(path))


def artifact_size(document):
    """Validate JSON and return its exact bounded on-disk UTF-8 byte count."""
    return len(_encoded(document))


def _destination(path):
    path = Path(path).expanduser()
    # Resolve parents, not the final entry: a symlink must never redirect an
    # exclusive artifact write or make an existing entry look absent.
    return path.parent.resolve() / path.name


def _fsync_directory(directory):
    descriptor = os.open(directory, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def preflight(destination_dir, estimated_bytes=1024 * 1024):
    """Check actual space and writes, independently of continuous log recording.

    Only missing directories and an owned temporary probe are created. Existing
    files, permissions, recording policy, and recording latches are untouched.
    This is a best-effort check; callers must still handle later write failures.
    """
    if type(estimated_bytes) is not int or estimated_bytes < 0:
        raise ValueError("Estimated calibration artifact size must be a nonnegative integer")
    directory = Path(destination_dir).expanduser().resolve()
    try:
        directory.mkdir(parents=True, exist_ok=True)
        free = shutil.disk_usage(directory).free
        if free < FREE_SPACE_MARGIN_BYTES + estimated_bytes:
            raise OSError(errno.ENOSPC, "Insufficient free space for calibration artifact and 16 MiB safety margin")
        with tempfile.TemporaryFile(dir=directory, prefix=".calibration-probe-") as stream:
            stream.write(b"calibration artifact write probe\n")
            stream.flush()
            os.fsync(stream.fileno())
    except OSError as exc:
        raise _storage_error(exc, directory) from exc
    return str(directory)


def write_artifact(path, document):
    """Publish bounded, fsynced JSON atomically without overwriting any file."""
    encoded = _encoded(document)
    # Freeze exactly the document whose bound was checked; a live caller may
    # mutate its own dictionaries while the disk preflight is in progress.
    snapshot = json.loads(encoded)
    path = _destination(path)
    preflight(path.parent, estimated_bytes=len(encoded))
    try:
        _write_json(path, snapshot)
        # Persist the exclusive directory entry as well as the file contents.
        _fsync_directory(path.parent)
    except OSError as exc:
        raise _storage_error(exc, path) from exc
    return str(path)


def resume_artifact(path, document):
    """Explicit durability retry; accept only an identical existing JSON value.

    A failed directory fsync can leave the complete, exclusively published file
    present. Such a retry fsyncs that regular file and its parent without writing
    its contents. Different data, symlinks and nonregular files are never reused.
    Normal ``write_artifact`` remains exclusive even for identical documents.
    """
    expected = json.loads(_encoded(document))
    path = _destination(path)
    try:
        try:
            descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        except FileNotFoundError:
            return write_artifact(path, expected)
        try:
            stream = os.fdopen(descriptor, "rb")
        except OSError:
            os.close(descriptor)
            raise
        with stream:
            opened = os.fstat(stream.fileno())
            if not stat.S_ISREG(opened.st_mode):
                raise FileExistsError(errno.EEXIST, "Calibration retry requires a regular, non-symlink file", str(path))
            encoded = stream.read(MAX_ARTIFACT_BYTES + 1)
            if len(encoded) > MAX_ARTIFACT_BYTES:
                raise ValueError("Existing calibration artifact exceeds the 16 MiB size limit")
            try:
                actual = json.loads(encoded)
                _encoded(actual)  # Reject NaN/Infinity as well as malformed JSON.
            except (ValueError, UnicodeError, RecursionError) as exc:
                raise ValueError(f"Existing calibration artifact is not bounded finite JSON: {exc}") from exc
            # Object key order/whitespace are immaterial; JSON booleans must not
            # accidentally equal integer 1/0 through Python's loose equality.
            canonical = lambda value: json.dumps(value, sort_keys=True, ensure_ascii=False, allow_nan=False)
            if canonical(actual) != canonical(expected):
                raise FileExistsError(errno.EEXIST, "Calibration artifact path already contains different JSON data", str(path))
            current = path.lstat()
            if not stat.S_ISREG(current.st_mode) or (current.st_dev, current.st_ino) != (opened.st_dev, opened.st_ino):
                raise FileExistsError(errno.EEXIST, "Calibration artifact changed during retry", str(path))
            os.fsync(stream.fileno())
        _fsync_directory(path.parent)
    except OSError as exc:
        raise _storage_error(exc, path) from exc
    return str(path)


def _binding(value):
    if (not isinstance(value, (list, tuple)) or len(value) != 7
            or not isinstance(value[0], str) or not value[0]
            or type(value[1]) is not int or value[1] < 0
            or type(value[6]) is not bool):
        raise ValueError("Touch TASK requires its original session/calibration binding")
    for index, name in ((2, "config"), (3, "upstream state"), (4, "nodes"), (5, "functional fingers")):
        _sha(value[index], f"Touch TASK binding {name}")
    return list(value)


def _provenance(document, expected):
    upstream = document.get("upstream")
    if (not isinstance(upstream, dict)
            or not isinstance(upstream.get("upstream_project"), str) or not upstream["upstream_project"]
            or not isinstance(upstream.get("upstream_commit"), str) or not upstream["upstream_commit"]
            or not isinstance(upstream.get("sources"), dict) or not upstream["sources"]):
        raise ValueError("Touch TASK requires upstream source provenance")
    for name, record in upstream["sources"].items():
        if not isinstance(record, dict):
            raise ValueError("Touch TASK upstream source record must be an object")
        _sha(record.get("sha256"), f"Touch TASK upstream {name}")
    _validate_upstream(upstream, upstream if expected is None else expected)
    model = document.get("mano_model")
    if not isinstance(model, dict) or model.get("hand") not in ("right", "left"):
        raise ValueError("Touch TASK requires left/right MANO model provenance")
    _sha(model.get("sha256"), "Touch TASK MANO model")
    if expected is not None and expected.get("mano_model") is not None:
        wanted = expected["mano_model"]
        if not isinstance(wanted, dict) or wanted.get("hand") not in ("right", "left"):
            raise ValueError("Expected MANO model provenance is invalid")
        _sha(wanted.get("sha256"), "expected MANO model")
        if any(wanted[key] != model[key] for key in ("hand", "sha256")):
            raise ValueError("Touch TASK uses a different MANO model")


def _identity(value):
    if not isinstance(value, (list, tuple)) or not value:
        raise ValueError("Touch TASK rows require original input identities")
    result, nodes = [], set()
    for item in value:
        if (not isinstance(item, (list, tuple)) or len(item) != 3
                or not isinstance(item[0], str) or not item[0] or item[0] in nodes
                or type(item[1]) is not int or item[1] < 0
                or type(item[2]) is not int or item[2] <= 0):
            raise ValueError("Touch TASK input identities require unique nodes, epochs and positive sample indices")
        nodes.add(item[0])
        result.append(tuple(item))
    return tuple(sorted(result))


def _finite_array(value, shape, name):
    try:
        return _array(value, shape, name)
    except (TypeError, OverflowError) as exc:
        raise ValueError(f"{name} must be a finite array of shape {shape}") from exc


def validate_task(document, provenance=None):
    """Return an independent validated partial TASK (one through four groups).

    Labels and original session/input identities are mandatory, never invented.
    Supplying expected provenance also verifies upstream and MANO model bytes;
    historical paths may differ. Loading does not rebind evidence to a session.
    """
    if not isinstance(document, dict):
        raise ValueError("Touch TASK must be a JSON object")
    _encoded(document)
    if type(document.get("schema_version")) is not int or document["schema_version"] != 1 or document.get("kind") != TASK_KIND:
        raise ValueError("Unsupported touch TASK schema")
    try:
        if not isinstance(document.get("task_id"), str):
            raise ValueError("missing task ID")
        UUID(document["task_id"])
    except ValueError as exc:
        raise ValueError("Touch TASK requires its original UUID task_id") from exc
    binding = _binding(document.get("binding"))
    if document.get("source_session_id") != binding[0]:
        raise ValueError("Touch TASK source_session_id does not match its original binding")
    if document.get("contact_labels_confirmed") is not True:
        raise ValueError("Touch TASK requires explicit contact_labels_confirmed=true")
    if document.get("contact_labels_source", CONTACT_SOURCE) != CONTACT_SOURCE:
        raise ValueError("Touch TASK contact labels must be explicitly user-observed")
    _provenance(document, provenance)
    before = _finite_array(document.get("before_shape_params"), (10,), "before_shape_params")
    groups = document.get("groups")
    if (not isinstance(groups, dict) or not 1 <= len(groups) <= len(FINGERS)
            or not set(groups).issubset({str(category) for category in FINGERS})):
        raise ValueError("Touch TASK requires one through four contact groups keyed 2..5")
    result, total = deepcopy(document), 0
    result["binding"], result["before_shape_params"] = binding, before.tolist()
    for key, group in groups.items():
        if not isinstance(group, dict) or _binding(group.get("binding")) != binding:
            raise ValueError("Touch TASK contact groups must share the original calibration binding")
        rows = group.get("rows")
        if not isinstance(rows, list) or not 1 <= len(rows) <= MAX_SAMPLES_PER_FINGER:
            raise ValueError("Touch TASK contact groups require 1..120 captured rows each")
        if "category" in group and (type(group["category"]) is not int or group["category"] != int(key)):
            raise ValueError("Touch TASK group category does not match its key")
        total += len(rows)
        identities, previous_time = set(), None
        result["groups"][key]["binding"] = list(binding)
        for index, row in enumerate(rows):
            if (not isinstance(row, dict) or type(row.get("category")) is not int
                    or row["category"] != int(key) or row.get("contact_confirmed") is not True):
                raise ValueError("Touch TASK rows require the matching category and contact_confirmed=true")
            pose = _finite_array(row.get("pose"), (16, 3), "Touch TASK pose")
            timestamp = row.get("timestamp_s")
            if (type(timestamp) not in (int, float) or not math.isfinite(timestamp)
                    or timestamp < 0 or (previous_time is not None and timestamp <= previous_time)):
                raise ValueError("Touch TASK row timestamps must be finite and increasing within each group")
            identity = _identity(row.get("input_identity"))
            if identity in identities:
                raise ValueError("Touch TASK must not repeat an input identity within a contact group")
            identities.add(identity)
            previous_time = timestamp
            result["groups"][key]["rows"][index]["pose"] = pose.tolist()
    if total > MAX_SAMPLES:
        raise ValueError("Touch TASK allows at most 480 total captured rows")
    # Normalize tuple-based live bindings/identities into their JSON form.
    return json.loads(_encoded(result))


def save_task(path, document, provenance=None):
    """Save a validated TASK exclusively; revisions need different file names."""
    return write_artifact(path, validate_task(document, provenance))


def load_task(path, provenance=None):
    """Load bounded TASK JSON without hardware, fitting, or applying any state."""
    path = Path(path).expanduser().resolve()
    with path.open("rb") as stream:
        encoded = stream.read(MAX_ARTIFACT_BYTES + 1)
    if len(encoded) > MAX_ARTIFACT_BYTES:
        raise ValueError("Touch TASK exceeds the 16 MiB size limit")
    try:
        document = json.loads(encoded)
    except (ValueError, UnicodeError, RecursionError) as exc:
        raise ValueError(f"Touch TASK must contain valid JSON: {exc}") from exc
    return validate_task(document, provenance)


def task_to_contacts(document, provenance=None):
    """Convert four saved groups to the unchanged offline optimizer contract."""
    task = validate_task(document, provenance)
    if set(task["groups"]) != {str(category) for category in FINGERS}:
        raise ValueError("Touch TASK needs all four contact groups before optimization")
    rows = [row for key in sorted(task["groups"]) for row in task["groups"][key]["rows"]]
    return validate_contacts(dict(
        mano_pose_rotvec=[row["pose"] for row in rows],
        finger_category=[row["category"] for row in rows],
        contact_labels_confirmed=task["contact_labels_confirmed"],
        contact_labels_source=CONTACT_SOURCE,
        before_shape_params=task["before_shape_params"],
        capture_evidence=[{key: value for key, value in row.items() if key != "pose"} for row in rows],
        task_id=task["task_id"], source_session_id=task["source_session_id"],
        session_id=task["source_session_id"], config_sha256=task["binding"][2],
        binding=task["binding"], upstream=task["upstream"], mano_model=task["mano_model"]))
