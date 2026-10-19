"""Bounded provenance observations; never deserialize models or scan directories."""
from dataclasses import asdict
from importlib import metadata as packages
import hashlib
import json
import os
from pathlib import Path
import platform
import stat
import subprocess

from .storage import bounded_text, utc_now


MAX_REF_BYTES = 1024 * 1024


def bounded_document(value, *, maximum=128*1024):
    """Produce a bounded JSON preview, explicitly marking every truncation."""
    truncated = False

    def clean(item, depth=0):
        nonlocal truncated
        if depth >= 10:
            truncated = True
            return "[preview depth truncated]"
        if isinstance(item, str):
            result = bounded_text(item, 4096)
            truncated |= result != item
            return result
        if isinstance(item, dict):
            truncated |= len(item) > 200 or any(len(str(key).encode()) > 512 for key in item)
            return {bounded_text(key, 512): clean(val, depth+1) for key, val in list(item.items())[:200]}
        if isinstance(item, (list, tuple)):
            truncated |= len(item) > 200
            return [clean(val, depth+1) for val in item[:200]]
        return item

    result = clean(value)
    while len(json.dumps(result, ensure_ascii=False, allow_nan=False).encode()) > maximum:
        truncated = True
        # Keep identities and summaries; progressively bound bulky collection
        # fields. Full quality reports remain at their original report_path.
        def shrink(item):
            if isinstance(item, dict):
                return {key: shrink(val) for key, val in list(item.items())[:20]}
            if isinstance(item, list):
                return [shrink(val) for val in item[:max(1, len(item)//2)]]
            if isinstance(item, str):
                return bounded_text(item, max(64, len(item.encode())//2))
            return item
        smaller = shrink(result)
        if smaller == result:
            result = {"summary": "Preview omitted because the bounded JSON limit was exceeded."}
            break
        result = smaller
    if truncated:
        result["preview_truncated"] = True
        result["preview_notice"] = "Bounded preview; consult the frozen artifact or complete report for full content."
    return result


def observe_reference(reference):
    result = dict(reference, observed_at=utc_now(), status="unknown", sha256=None, size_bytes=None,
                  detail="File content observation only; does not prove a solver loaded this file.")
    path = Path(reference["path"]).expanduser().absolute()
    result["path"] = str(path)
    forbidden = {".ssh", ".aws", ".gnupg", ".azure", ".kube", "credentials", "secrets"}
    name = path.name.lower()
    if (path.suffix.lower() not in {".json", ".yaml", ".yml"} or any(part.lower() in forbidden for part in path.parts)
            or any(token in name for token in ("secret", "credential", "password", "token", "mano_left", "mano_right"))):
        result["detail"] = "Reference was not read: only explicitly listed non-secret text config/calibration files are permitted."
        return result, None
    try:
        if any(component.is_symlink() for component in (path, *path.parents)):
            raise ValueError("symlink metadata reference paths are not permitted")
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        with os.fdopen(fd, "rb") as stream:
            before = os.fstat(stream.fileno())
            if not stat.S_ISREG(before.st_mode) or before.st_size > MAX_REF_BYTES:
                raise ValueError("reference must be a regular text file no larger than 1 MiB")
            content = stream.read(MAX_REF_BYTES+1)
            after = os.fstat(stream.fileno())
            if (len(content) > MAX_REF_BYTES or before.st_size != after.st_size or before.st_mtime_ns != after.st_mtime_ns):
                raise ValueError("reference changed while being observed")
            content.decode("utf-8")
        result.update(status="observed", sha256=hashlib.sha256(content).hexdigest(), size_bytes=len(content))
        return result, content
    except (OSError, ValueError, UnicodeError) as exc:
        result["detail"] = bounded_text(f"Reference unavailable; no file content was captured: {exc}")
        return result, None


class MetadataCollector:
    def __init__(self, config):
        self.config = config
        versions = {"python": platform.python_version()}
        for name in ("vt_capture_manager", "vt_capture_core"):
            try:
                versions[name] = packages.version(name)
            except packages.PackageNotFoundError:
                versions[name] = "unknown"
        commit = None
        candidate = Path(__file__).resolve().parents[4]
        if (candidate/"ros2_ws/src/vt_capture_manager/package.xml").is_file() and (candidate/".git").exists():
            try:
                process = subprocess.run(["git", "-C", str(candidate), "rev-parse", "HEAD"],
                                         capture_output=True, text=True, timeout=1., check=False)
                value = process.stdout.strip()
                if process.returncode == 0 and len(value) in (40, 64) and all(c in "0123456789abcdef" for c in value):
                    commit = value
            except (OSError, subprocess.TimeoutExpired):
                pass
        self.base = dict(schema_version=1, software=versions, git_commit=commit,
                         git_commit_scope="Base commit only; uncommitted and untracked working-tree changes are not captured.",
                         manager_config=asdict(config), software_observed_at=utc_now())

    def prepare(self):
        """Call before the management control lock, once per START preparation."""
        references = [observe_reference(ref)[0] for ref in self.config.metadata_refs]
        alignment = None
        content = None
        if self.config.quality_alignment_config:
            alignment, content = observe_reference(dict(name="quality_alignment_config", kind="config",
                                                       path=self.config.quality_alignment_config))
        return dict(references=references, alignment_config=alignment,
                    prepared_at=utc_now(), content_observation_not_solver_load=True), content

    def start_document(self, request_id, params, context, prepared):
        return bounded_document(dict(self.base, request_id=request_id, planned=params, controller_before_start=context,
            controller_context_semantics="Pre-START guard context; not this trial's actual session. Actual identity is in the trial and frozen finish.",
            task_name=params.get("task_name") or None, operator_name=params.get("operator_name") or None,
            frozen_at=utc_now(), **prepared))
