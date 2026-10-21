"""Validate delivery bytes and stage a private, versioned user installation.

No installer, hardware library, downloaded source or privileged action is run
here. Installation works from the verified snapshot, not mutable input paths.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import tempfile
from types import MappingProxyType
from typing import Mapping


MAX_SOURCE_BYTES = 128 * 1024 * 1024
MAX_PACKAGE_BYTES = 130 * 1024 * 1024
_HASH = re.compile(r"[0-9a-f]{64}\Z")


class DeliveryError(ValueError):
    pass


@dataclass(frozen=True)
class Delivery:
    source_root: Path
    tracker_package: Path
    installation_id: str
    source_manifest_sha256: str
    tracker_sha256: str
    files: Mapping[str, bytes] = field(repr=False)
    package_bytes: bytes = field(repr=False)


def _read(path, limit):
    path = Path(path).absolute()
    if any(p.is_symlink() for p in (path, *path.parents)):
        raise DeliveryError("安装文件不能是符号链接。")
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(descriptor, "rb") as stream:
        info = os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_size > limit:
            raise DeliveryError("安装文件类型或大小无效。")
        content = stream.read(limit + 1)
    if len(content) > limit:
        raise DeliveryError("安装文件超过大小限制。")
    return content


def _pairs(items):
    result = {}
    for name, value in items:
        if name in result:
            raise DeliveryError("安装清单包含重复字段。")
        result[name] = value
    return result


def _document(content):
    try:
        return json.loads(content, object_pairs_hook=_pairs)
    except (ValueError, UnicodeError, RecursionError) as exc:
        raise DeliveryError("安装清单无效，请重新取得完整安装包。") from exc


def _safe_name(name):
    if (not isinstance(name, str) or not name or "\\" in name
            or any(ord(c) < 32 for c in name)
            or PurePosixPath(name).is_absolute()
            or any(p in {"", ".", ".."} for p in name.split("/"))):
        raise DeliveryError("安装清单包含无效文件路径。")
    return name


def inspect_delivery(directory) -> Delivery:
    directory = Path(directory).expanduser().absolute()
    try:
        manifest = _document(_read(directory / "DELIVERY_MANIFEST.json", 256 * 1024))
        if manifest.get("schema_version") != 1 or manifest.get("distribution") != "restricted-device-delivery":
            raise DeliveryError("不支持此安装包格式。")
        source = manifest["source"]
        if source["manifest_path"] != "Vision-Tactile-Dataset/PUBLIC_SOURCE_MANIFEST.json":
            raise DeliveryError("程序清单位置无效。")
        source_root = directory / "Vision-Tactile-Dataset"
        raw = _read(source_root / "PUBLIC_SOURCE_MANIFEST.json", 2 * 1024 * 1024)
        source_digest = hashlib.sha256(raw).hexdigest()
        if source_digest != source["manifest_sha256"]:
            raise DeliveryError("程序清单校验失败，请重新下载安装包。")
        public = _document(raw)
        entries = public["files"]
        if public.get("schema_version") != 1 or not isinstance(entries, list) or not 1 <= len(entries) <= 4096:
            raise DeliveryError("程序文件清单无效。")
        files = {}
        total = 0
        for entry in entries:
            name = _safe_name(entry["path"])
            if name in files or name == "PUBLIC_SOURCE_MANIFEST.json":
                raise DeliveryError("程序文件清单重复。")
            size, digest = entry["bytes"], entry["sha256"]
            if (type(size) is not int or not 0 <= size <= 5 * 1024 * 1024
                    or not isinstance(digest, str) or not _HASH.fullmatch(digest)):
                raise DeliveryError("程序文件摘要无效。")
            total += size
            if total > MAX_SOURCE_BYTES:
                raise DeliveryError("程序文件总大小超出限制。")
            content = _read(source_root / name, size)
            if len(content) != size or hashlib.sha256(content).hexdigest() != digest:
                raise DeliveryError("程序文件校验失败，请重新下载安装包。")
            files[name] = content
        required = {"tools/capture_deploy/install.sh", "tools/capture_deploy/install_system.py",
                    "tools/capture_deploy/start_workbench.sh"}
        if not required <= files.keys():
            raise DeliveryError("安装包缺少自动安装组件。")
        files["PUBLIC_SOURCE_MANIFEST.json"] = raw
        package = manifest["tracker_package"]
        if package["path"] != "deployment/tracker.vttracker":
            raise DeliveryError("设备依赖包位置无效。")
        package_path = directory / package["path"]
        package_bytes = _read(package_path, MAX_PACKAGE_BYTES)
        package_digest = hashlib.sha256(package_bytes).hexdigest()
        if len(package_bytes) != package["bytes"] or package_digest != package["sha256"]:
            raise DeliveryError("设备依赖包校验失败，请重新下载安装包。")
        from vt_tracker_deploy.archive import read_package
        if read_package(package_path).package_id != package_digest:
            raise DeliveryError("设备依赖包在校验期间发生变化。")
        installation_id = hashlib.sha256(raw + package_digest.encode("ascii")).hexdigest()
        return Delivery(source_root, package_path, installation_id, source_digest,
                        package_digest, MappingProxyType(files), package_bytes)
    except DeliveryError:
        raise
    except (OSError, KeyError, TypeError, AttributeError, ValueError) as exc:
        # Parsing errors must not expose any private input contents.
        raise DeliveryError("安装包不完整或无法读取，请重新取得完整安装包。") from exc


def _user_directory(path, home):
    if not path.is_relative_to(home):
        raise DeliveryError("安装目录必须位于当前用户目录。")
    for current in reversed((path, *path.parents)):
        if not current.is_relative_to(home):
            continue
        try:
            current.mkdir(mode=0o700)
        except FileExistsError:
            pass
        info = current.lstat()
        if (not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid()
                or info.st_mode & 0o022):
            raise DeliveryError("安装目录的所有者或权限不安全。")


def _write(path, content, mode, *, root):
    _user_directory(path.parent, root)
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, mode)
    with os.fdopen(descriptor, "wb") as stream:
        stream.write(content)


def stage_delivery(delivery: Delivery, home=None) -> Path:
    home = Path.home() if home is None else Path(home).absolute()
    versions = home / ".local/share/visiontactile/application/versions"
    _user_directory(versions, home)
    target = versions / delivery.installation_id
    if target.exists() or target.is_symlink():
        _user_directory(target, home)
        for name, content in delivery.files.items():
            if _read(target / "source" / name, len(content)) != content:
                raise DeliveryError("已暂存的程序文件发生变化，不能覆盖。")
        if _read(target / "deployment/tracker.vttracker", MAX_PACKAGE_BYTES) != delivery.package_bytes:
            raise DeliveryError("已暂存的设备依赖包发生变化，不能覆盖。")
        return target / "source"
    temporary = Path(tempfile.mkdtemp(prefix=".staging-", dir=versions))
    try:
        for name, content in delivery.files.items():
            mode = 0o755 if content.startswith(b"#!") or name.endswith(".sh") else 0o644
            _write(temporary / "source" / name, content, mode, root=temporary)
        _write(temporary / "deployment/tracker.vttracker", delivery.package_bytes, 0o600, root=temporary)
        _write(temporary / "installation.json", (json.dumps({
            "schema_version": 1, "installation_id": delivery.installation_id,
            "source_manifest_sha256": delivery.source_manifest_sha256,
            "tracker_sha256": delivery.tracker_sha256,
        }, indent=2) + "\n").encode(), 0o600, root=temporary)
        os.rename(temporary, target)
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)
    return target / "source"
