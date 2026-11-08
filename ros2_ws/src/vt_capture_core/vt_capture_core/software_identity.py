"""Bound a running manager to its source tree, not the frontend's version label.

Identity is captured once at process startup. A later checkout/edit must not
make an already running manager claim it loaded the new implementation.
Runtime resources and recordings are deliberately outside this code identity.
"""
from functools import lru_cache
import hashlib
import os
from pathlib import Path
import re
import subprocess

from .validation import ProtocolError, mapping, limited_text


FIELDS = {"source_root", "source_revision", "source_sha256", "configuration_path"}
ROOTS = ("ros2_ws/src", "tools/capture_console/src", "tools/fsglove12/src",
         "tools/multisensor_alignment/src", "tools/tracker_deploy/src")
SUFFIXES = {".py", ".sh", ".xml", ".json", ".yaml", ".yml", ".msg", ".srv",
            ".action", ".toml", ".txt", ".cmake", ".cpp", ".hpp", ".h"}
EXCLUDED = {"test", "tests", "__pycache__", ".pytest_cache"}


def empty_identity():
    return dict.fromkeys(FIELDS, "")


def validate_identity(value):
    mapping(value, FIELDS, "software identity")
    for field in ("source_root", "configuration_path"):
        limited_text(value[field], field, maximum=4096)
        if value[field] and not Path(value[field]).is_absolute():
            raise ProtocolError(f"{field} must be absolute")
    limited_text(value["source_revision"], "source_revision", maximum=64)
    limited_text(value["source_sha256"], "source_sha256", maximum=64)
    if value["source_revision"] and not re.fullmatch(r"[0-9a-f]{40,64}", value["source_revision"]):
        raise ProtocolError("invalid source revision")
    if value["source_sha256"] and not re.fullmatch(r"[0-9a-f]{64}", value["source_sha256"]):
        raise ProtocolError("invalid source checksum")
    if bool(value["source_root"]) != bool(value["source_sha256"]):
        raise ProtocolError("source root and checksum must be provided together")


def source_root(anchor):
    for parent in Path(anchor).resolve().parents:
        if (parent / "tools/capture_deploy/start_manager.sh").is_file() and (parent / "ros2_ws/src").is_dir():
            return parent
    return None


@lru_cache(maxsize=4)
def _source_identity(root):
    root = Path(root).resolve(strict=True)
    paths = []
    for name in ROOTS:
        directory = root / name
        if not directory.is_dir():
            raise ValueError(f"Incomplete runtime source tree: {name}")
        for parent, directories, files in os.walk(directory, followlinks=False):
            directories[:] = sorted(d for d in directories if d not in EXCLUDED and not d.startswith("."))
            for name in sorted(files):
                path = Path(parent) / name
                if path.suffix in SUFFIXES:
                    paths.append(path)
    paths.extend(root / name for name in ("tools/capture_deploy/start_manager.sh",))
    if len(paths) > 10000:
        raise ValueError("Runtime source inventory exceeds its limit")
    digest, total = hashlib.sha256(), 0
    for path in sorted(paths):
        path.resolve(strict=True).relative_to(root)
        with path.open("rb") as stream:
            content = stream.read(16 * 1024 * 1024 + 1)
        total += len(content)
        if len(content) > 16 * 1024 * 1024 or total > 64 * 1024 * 1024:
            raise ValueError("Runtime source bytes exceed their limit")
        digest.update(path.relative_to(root).as_posix().encode() + b"\0")
        digest.update(hashlib.sha256(content).digest())
    try:
        revision = subprocess.run(["git", "-C", str(root), "rev-parse", "HEAD"],
            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, timeout=2., check=True).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        revision = ""
    if revision and re.fullmatch(r"[0-9a-f]{40,64}", revision) is None:
        raise ValueError("Invalid source revision")
    return str(root), revision, digest.hexdigest()


def runtime_identity(root, configuration_path=None):
    result = empty_identity()
    if root is not None:
        result.update(zip(("source_root", "source_revision", "source_sha256"), _source_identity(str(root))))
    if configuration_path is not None:
        result["configuration_path"] = str(Path(configuration_path).expanduser().resolve())
    validate_identity(result)
    return result


def require_identity(actual, expected):
    validate_identity(actual)
    validate_identity(expected)
    if not expected["source_sha256"] or actual != expected:
        raise ValueError("后台代码或配置来源与候选版本不一致；未接管旧后台，请使用对应候选入口。")
