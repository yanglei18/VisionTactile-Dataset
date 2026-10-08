"""Bounded, cancellable post-record quality work in an owned Python subprocess.

This module never launches rosbag record or signals a Recorder. It reuses the
offline report generator, preserving its observation-only ``ok``/``warnings``
semantics. Input identity uses a small metadata hash plus file stats, not a
content hash of large recordings; it is not a cryptographic bag integrity audit.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import re
import selectors
import stat
import subprocess
import sys
import tempfile
import time
from uuid import uuid4


MAX_METADATA_BYTES = 4 * 1024 * 1024
MAX_REPORT_BYTES = 8 * 1024 * 1024
MAX_CONFIG_BYTES = 2 * 1024 * 1024
MAX_EXPECTED_TOPICS_BYTES = 64 * 1024
MAX_ADVISORIES_BYTES = 32 * 1024
MAX_INPUT_FILES = 4096
MAX_LOG_BYTES = 8192
_TOPIC = re.compile(r"(?:/[A-Za-z_][A-Za-z0-9_]*)+")


class QualityRunError(RuntimeError):
    """The input, subprocess or report could not be safely verified."""


class QualityCancelled(QualityRunError):
    """Only this owned quality process was cancelled; the bag is untouched."""


def _stat_key(info):
    return [info.st_dev, info.st_ino, info.st_mode, info.st_size,
            info.st_mtime_ns, info.st_ctime_ns]


def _no_symlinks(path):
    for part in (*reversed(path.parents), path):
        try:
            if stat.S_ISLNK(part.lstat().st_mode):
                raise QualityRunError(f"Symlink paths are not accepted for quality input/output: {part}")
        except FileNotFoundError:
            continue


def _read_bounded(path, maximum):
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_NONBLOCK", 0))
    with os.fdopen(descriptor, "rb") as stream:
        before = os.fstat(stream.fileno())
        if not stat.S_ISREG(before.st_mode) or before.st_size > maximum:
            raise QualityRunError(f"Expected a regular file no larger than {maximum} bytes: {path}")
        data = stream.read(maximum + 1)
        if len(data) > maximum or _stat_key(before) != _stat_key(os.fstat(stream.fileno())):
            raise QualityRunError(f"File changed or exceeded its size bound while reading: {path}")
    return data, _stat_key(before)


def _expected_topics(value):
    if type(value) is not list or len(value) > 256:
        raise QualityRunError("expected_topics must be an array of at most 256 topics")
    if any(type(topic) is not str or len(topic) > 512 or _TOPIC.fullmatch(topic) is None for topic in value):
        raise QualityRunError("expected_topics must contain bounded absolute ROS topic names")
    if len(set(value)) != len(value):
        raise QualityRunError("expected_topics must not contain duplicates")
    result = sorted(value)
    if len(json.dumps(result).encode("utf-8")) > MAX_EXPECTED_TOPICS_BYTES:
        raise QualityRunError("expected_topics exceeds the command payload bound")
    return result


def _advisories(value):
    """Bounded caller observations, carried into the published standard warnings."""
    if value is None:
        return []
    if type(value) not in (list, tuple) or len(value) > 16:
        raise QualityRunError("advisories must contain at most 16 observations")
    result = []
    for item in value:
        if (type(item) is not dict or set(item) != {"code", "detail"}
                or type(item["code"]) is not str or len(item["code"]) > 64
                or re.fullmatch(r"[a-z][a-z0-9_]*", item["code"]) is None
                or type(item["detail"]) is not str or not item["detail"]
                or len(item["detail"]) > 2048):
            raise QualityRunError("advisories require bounded code/detail string fields")
        if item not in result:
            result.append(dict(item))
    if len(json.dumps(result).encode("utf-8")) > MAX_ADVISORIES_BYTES:
        raise QualityRunError("advisories exceeds the command payload bound")
    return result


def _fingerprint(bag, alignment_config=None):
    """Bounded metadata read plus recursive stat inventory, never reading bag data."""
    import yaml

    _no_symlinks(bag)
    before = bag.stat()
    if not stat.S_ISDIR(before.st_mode):
        raise QualityRunError(f"Bag path must be a directory: {bag}")
    metadata, metadata_stat = _read_bounded(bag / "metadata.yaml", MAX_METADATA_BYTES)
    document = yaml.safe_load(metadata)
    if type(document) is not dict or type(document.get("rosbag2_bagfile_information")) is not dict:
        raise QualityRunError("Bag metadata lacks rosbag2_bagfile_information")
    info = document["rosbag2_bagfile_information"]
    relative = info.get("relative_file_paths", [])
    if type(relative) is not list or len(relative) > MAX_INPUT_FILES:
        raise QualityRunError("Bag metadata has an invalid file list")
    entries = {}

    def inspect(directory, depth=0):
        if depth > 8:
            raise QualityRunError("Bag directory nesting exceeds the inspection bound")
        with os.scandir(directory) as children:
            for child in children:
                if len(entries) >= MAX_INPUT_FILES:
                    raise QualityRunError("Bag file count exceeds the inspection bound")
                child_path = Path(child.path)
                key = child_path.relative_to(bag).as_posix()
                item = child.stat(follow_symlinks=False)
                if not (stat.S_ISREG(item.st_mode) or stat.S_ISDIR(item.st_mode)):
                    raise QualityRunError(f"Bag contains a symlink or nonregular entry: {key}")
                entries[key] = _stat_key(item)
                if stat.S_ISDIR(item.st_mode):
                    inspect(child_path, depth + 1)

    inspect(bag)
    if entries.get("metadata.yaml") != metadata_stat or _stat_key(before) != _stat_key(bag.stat()):
        raise QualityRunError("Bag changed during fingerprint inspection")
    for name in relative:
        if (type(name) is not str or not name or Path(name).is_absolute()
                or ".." in Path(name).parts or name not in entries
                or not stat.S_ISREG(entries[name][2])):
            raise QualityRunError("Bag metadata references an absent, unsafe or nonregular file")
    configuration = None
    if alignment_config is not None:
        _no_symlinks(alignment_config)
        content, identity = _read_bounded(alignment_config, MAX_CONFIG_BYTES)
        configuration = dict(path=str(alignment_config), identity=identity,
                             sha256=hashlib.sha256(content).hexdigest())
    fingerprint = dict(path=str(bag), directory=_stat_key(before), files=entries,
                       metadata_sha256=hashlib.sha256(metadata).hexdigest(), alignment_config=configuration)
    digest = hashlib.sha256(json.dumps(fingerprint, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    return dict(digest=digest, metadata_sha256=fingerprint["metadata_sha256"],
                file_count=len(entries), alignment_config=configuration)


def _json_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise QualityRunError(f"Duplicate report JSON key: {key}")
        result[key] = value
    return result


def _finite_json(value, depth=0):
    if depth > 40:
        raise QualityRunError("Report JSON nesting exceeds the validation bound")
    if value is None or type(value) in (str, bool, int):
        return
    if type(value) is float and math.isfinite(value):
        return
    if type(value) is list:
        for item in value:
            _finite_json(item, depth + 1)
        return
    if type(value) is dict and all(type(key) is str for key in value):
        for item in value.values():
            _finite_json(item, depth + 1)
        return
    raise QualityRunError("Report contains non-JSON or non-finite data")


def _validate_report(report, bag, expected_topics, invocation_id, fingerprint, advisories=()):
    _finite_json(report)
    if type(report) is not dict or type(report.get("schema_version")) is not int or report["schema_version"] != 1:
        raise QualityRunError("Unsupported quality report schema")
    if report.get("report_kind") != "post_record_capture_quality" or report.get("status") not in {"ok", "warnings"}:
        raise QualityRunError("Unexpected quality report kind or status")
    generated = report.get("generated_at_utc")
    if type(generated) is not str:
        raise QualityRunError("Report generation timestamp is missing")
    try:
        if datetime.fromisoformat(generated).tzinfo is None:
            raise ValueError("timezone required")
    except ValueError as exc:
        raise QualityRunError("Report generation timestamp must carry a timezone") from exc
    limitations = report.get("limitations")
    if type(limitations) is not list or not all(type(value) is str for value in limitations):
        raise QualityRunError("Report limitations must remain explicit strings")
    bag_info = report.get("bag")
    if type(bag_info) is not dict or bag_info.get("path") != str(bag):
        raise QualityRunError("Quality report bag identity does not match the requested input")
    for key in ("duration_ns", "start_realtime_ns", "metadata_message_count"):
        if type(bag_info.get(key)) is not int or bag_info[key] < 0:
            raise QualityRunError(f"Invalid report bag.{key}")
    if bag_info.get("storage_identifier") is not None and type(bag_info["storage_identifier"]) is not str:
        raise QualityRunError("Invalid report storage identifier")
    topics, warnings, summary = report.get("topics"), report.get("warnings"), report.get("summary")
    if type(topics) is not dict or len(topics) > 4096 or type(warnings) is not list or type(summary) is not dict:
        raise QualityRunError("Malformed quality report topic/warning summary")
    for topic, entry in topics.items():
        if type(topic) is not str or _TOPIC.fullmatch(topic) is None or type(entry) is not dict:
            raise QualityRunError("Malformed report topic entry")
        if (type(entry.get("type")) is not str or type(entry.get("recorded_message_count")) is not int
                or entry["recorded_message_count"] < 0 or entry.get("inspection") not in {"metadata_only", "compact_messages"}):
            raise QualityRunError("Malformed report topic count or inspection basis")
    for warning in warnings:
        if (type(warning) is not dict or type(warning.get("code")) is not str
                or type(warning.get("detail")) is not str
                or ("topic" in warning and type(warning["topic"]) is not str)):
            raise QualityRunError("Malformed quality warning")
    for key in ("topic_count", "inspected_topic_count", "metadata_only_topic_count", "warning_count"):
        if type(summary.get(key)) is not int or summary[key] < 0:
            raise QualityRunError(f"Invalid report summary.{key}")
    if (summary["topic_count"] != len(topics) or summary["warning_count"] != len(warnings)
            or summary["inspected_topic_count"] + summary["metadata_only_topic_count"] != len(topics)
            or summary["inspected_topic_count"] != sum(entry["inspection"] == "compact_messages" for entry in topics.values())
            or report["status"] != ("warnings" if warnings else "ok")):
        raise QualityRunError("Report summary disagrees with its observations")
    provenance = report.get("runner_provenance")
    if (type(provenance) is not dict or provenance.get("invocation_id") != invocation_id
            or provenance.get("input_fingerprint") != fingerprint["digest"]
            or provenance.get("expected_topics") != expected_topics
            or provenance.get("advisories") != list(advisories)):
        raise QualityRunError("Report invocation/input/expected-topic identity mismatch")
    for advisory in advisories:
        if not any(all(warning.get(key) == value for key, value in advisory.items()) for warning in warnings):
            raise QualityRunError("Report omitted a caller advisory from its standard warnings")
    for topic in expected_topics:
        if (topic not in topics or topics[topic]["recorded_message_count"] == 0) and not any(
                warning["code"] == "expected_topic_missing" and warning.get("topic") == topic for warning in warnings):
            raise QualityRunError("Report omitted an expected missing-topic observation")
    return report


class QualityRunner:
    def __init__(self, python_executable=sys.executable, timeout_s=1200., *, module_paths=(),
                 terminate_grace_s=1., poll_interval_s=.05):
        for name, value in (("timeout_s", timeout_s), ("terminate_grace_s", terminate_grace_s),
                            ("poll_interval_s", poll_interval_s)):
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be a positive finite duration")
        if poll_interval_s > 1 or terminate_grace_s > 10:
            raise ValueError("Quality cancellation polling/grace bounds are too large")
        self.python_executable = os.fspath(python_executable)
        if not self.python_executable or "\0" in self.python_executable:
            raise ValueError("python_executable must be a valid executable path")
        self.timeout_s, self.terminate_grace_s, self.poll_interval_s = timeout_s, terminate_grace_s, poll_interval_s
        self.module_paths = tuple(str(Path(path).absolute()) for path in module_paths)

    def _environment(self):
        environment = os.environ.copy()
        # Preserve the sourced ROS environment, and support source-tree testing
        # without relying on the parent process's non-exported sys.path changes.
        source = Path(__file__).resolve()
        paths = [str(source.parents[1]), *self.module_paths]
        for parent in source.parents:
            alignment = parent / "tools/multisensor_alignment/src"
            if (alignment / "vt_multisensor_alignment").is_dir():
                paths.append(str(alignment))
                break
        paths.append(environment.get("PYTHONPATH", ""))
        environment["PYTHONPATH"] = os.pathsep.join(path for path in paths if path)
        return environment

    def _terminate_owned(self, process):
        if process.poll() is not None:
            return
        try:
            process.terminate()  # Only the handle returned by our own Popen.
            process.wait(timeout=self.terminate_grace_s)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=self.terminate_grace_s)
        except ProcessLookupError:
            process.wait(timeout=self.terminate_grace_s)

    def _execute(self, argv, cancel):
        process = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, shell=False, start_new_session=True, env=self._environment(), bufsize=0)
        logs = bytearray()
        selector = None
        started = time.monotonic()

        def drain(timeout):
            for key, _ in selector.select(timeout):
                chunk = os.read(key.fileobj.fileno(), 4096)
                if not chunk:
                    selector.unregister(key.fileobj)
                    continue
                logs.extend(chunk)
                del logs[:-MAX_LOG_BYTES]

        try:
            selector = selectors.DefaultSelector()
            selector.register(process.stdout, selectors.EVENT_READ)
            while process.poll() is None:
                if cancel():
                    raise QualityCancelled("Quality check cancelled; Recorder was not signalled")
                if time.monotonic() - started >= self.timeout_s:
                    raise QualityRunError(f"Quality subprocess timed out after {self.timeout_s:g} seconds")
                drain(self.poll_interval_s)
            while selector.get_map():
                if not selector.select(0):
                    break
                drain(0)
            if cancel():
                raise QualityCancelled("Quality check cancelled before report publication")
            if process.returncode != 0:
                detail = logs.decode("utf-8", errors="replace")[-MAX_LOG_BYTES:]
                raise QualityRunError(f"Quality subprocess exited {process.returncode}: {detail}")
        finally:
            try:
                self._terminate_owned(process)
            finally:
                if selector is not None:
                    selector.close()
                process.stdout.close()

    def run(self, bag_path, output_path, *, expected_topics, alignment_config=None,
            advisories=(), cancel=lambda: False) -> dict:
        """Return a verified raw quality report, atomically published without overwrite."""
        try:
            if cancel():
                raise QualityCancelled("Quality check cancelled before launch")
            expected = _expected_topics(expected_topics)
            observations = _advisories(advisories)
            bag, output = Path(bag_path).absolute(), Path(output_path).absolute()
            _no_symlinks(bag)
            _no_symlinks(output)
            bag = bag.resolve(strict=True)
            if output.resolve().is_relative_to(bag):
                raise QualityRunError("Quality output must be outside the input bag directory")
            if os.path.lexists(output):
                raise QualityRunError(f"Quality output already exists; refusing overwrite: {output}")
            config = Path(alignment_config).absolute() if alignment_config is not None else None
            before = _fingerprint(bag, config)
            output.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
            _no_symlinks(output)
            parent_identity = _stat_key(output.parent.stat())[:2]
            invocation = uuid4().hex
            with tempfile.TemporaryDirectory(prefix=".quality-", dir=output.parent) as temporary:
                staged = Path(temporary) / "quality_report.json"
                argv = [self.python_executable, "-m", "vt_capture_manager.quality_runner",
                        "--bag", str(bag), "--output", str(staged),
                        "--expected-topics", json.dumps(expected, separators=(",", ":")),
                        "--advisories", json.dumps(observations, separators=(",", ":")),
                        "--invocation-id", invocation, "--input-fingerprint", before["digest"]]
                if config is not None:
                    argv.extend(["--alignment-config", str(config)])
                self._execute(argv, cancel)
                _no_symlinks(staged)
                content, report_identity = _read_bounded(staged, MAX_REPORT_BYTES)
                report = json.loads(content, object_pairs_hook=_json_pairs)
                _validate_report(report, bag, expected, invocation, before, observations)
                if _fingerprint(bag, config) != before:
                    raise QualityRunError("Quality input changed during execution; report not published")
                if cancel():
                    raise QualityCancelled("Quality check cancelled before report publication")
                _no_symlinks(output)
                if (_stat_key(output.parent.stat())[:2] != parent_identity
                        or _stat_key(staged.stat()) != report_identity):
                    raise QualityRunError("Report destination or staged file changed during execution")
                os.link(staged, output, follow_symlinks=False)  # Atomic, refuses raced-in destinations.
                descriptor = os.open(output.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
                try:
                    os.fsync(descriptor)
                finally:
                    os.close(descriptor)
                return report
        except (QualityRunError, QualityCancelled):
            raise
        except Exception as exc:
            raise QualityRunError(f"Quality execution failed: {type(exc).__name__}: {exc}") from exc


def _worker(bag, output, expected, invocation, fingerprint, alignment_config=None, advisories=()):
    # Heavy ROS-compatible imports happen only inside this dedicated child.
    from vt_multisensor_alignment.capture_quality import generate_capture_quality, _write_report
    from vt_multisensor_alignment.config import load_config

    before = _fingerprint(bag, alignment_config)
    if before["digest"] != fingerprint:
        raise QualityRunError("Input identity changed before subprocess execution")
    config = load_config(alignment_config) if alignment_config is not None else None
    with tempfile.TemporaryDirectory(prefix=".quality-raw-", dir=output.parent) as temporary:
        report = generate_capture_quality(bag, Path(temporary) / "raw.json", config)
    for topic in expected:
        if (topic not in report["topics"] or report["topics"][topic]["recorded_message_count"] == 0) and not any(
                warning["code"] == "expected_topic_missing" and warning.get("topic") == topic for warning in report["warnings"]):
            report["warnings"].append(dict(code="expected_topic_missing", topic=topic,
                detail="expected recorded-topic allowlist entry has no recorded messages"))
    for advisory in advisories:
        if not any(all(warning.get(key) == value for key, value in advisory.items()) for warning in report["warnings"]):
            report["warnings"].append(dict(advisory))
    report["summary"]["warning_count"] = len(report["warnings"])
    report["status"] = "warnings" if report["warnings"] else "ok"
    report["runner_provenance"] = dict(
        invocation_id=invocation, input_fingerprint=fingerprint,
        fingerprint_basis="SHA256 of bounded metadata plus file device/inode/size/mtime/ctime inventory; bag payloads are not hashed",
        metadata_sha256=before["metadata_sha256"], input_file_count=before["file_count"],
        alignment_config=before["alignment_config"], expected_topics=expected,
        advisories=list(advisories),
        expected_topics_source="caller-supplied recorded-topic allowlist",
        topic_statistics_source="rosbag2 metadata.yaml counts; filtered compact payload statistics only where inspection=compact_messages",
    )
    _validate_report(report, bag, expected, invocation, before, advisories)
    if _fingerprint(bag, alignment_config) != before:
        raise QualityRunError("Input changed while generating quality observations")
    payload = json.dumps(report, ensure_ascii=False, allow_nan=False).encode("utf-8")
    if len(payload) > MAX_REPORT_BYTES // 2:
        # The shared writer uses indentation. Reserve bounded headroom for it.
        raise QualityRunError("Generated quality report exceeds the publication bound")
    _write_report(output, report)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Owned post-record quality subprocess; never controls recording")
    parser.add_argument("--bag", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--expected-topics", required=True)
    parser.add_argument("--advisories", default="[]")
    parser.add_argument("--alignment-config")
    parser.add_argument("--invocation-id", required=True)
    parser.add_argument("--input-fingerprint", required=True)
    args = parser.parse_args(argv)
    try:
        if len(args.expected_topics.encode("utf-8")) > MAX_EXPECTED_TOPICS_BYTES:
            raise QualityRunError("Expected-topic argument is too large")
        expected = _expected_topics(json.loads(args.expected_topics))
        if len(args.advisories.encode("utf-8")) > MAX_ADVISORIES_BYTES:
            raise QualityRunError("Advisory argument is too large")
        advisories = _advisories(json.loads(args.advisories, object_pairs_hook=_json_pairs))
        if re.fullmatch(r"[a-f0-9]{32}", args.invocation_id) is None or re.fullmatch(r"[a-f0-9]{64}", args.input_fingerprint) is None:
            raise QualityRunError("Invalid quality invocation identity")
        bag, output = Path(args.bag).absolute(), Path(args.output).absolute()
        _no_symlinks(bag)
        _no_symlinks(output)
        bag = bag.resolve(strict=True)
        if output.resolve().is_relative_to(bag) or os.path.lexists(output):
            raise QualityRunError("Quality output must be new and outside the bag")
        _worker(bag, output, expected, args.invocation_id, args.input_fingerprint,
                Path(args.alignment_config).absolute() if args.alignment_config else None, advisories)
        return 0
    except Exception as exc:
        print(f"{type(exc).__name__}: {exc}"[:2048], file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
