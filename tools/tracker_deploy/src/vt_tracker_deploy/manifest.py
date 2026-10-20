"""Read one private, coherent Tracker installation; never run or read its inputs."""
from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
import stat


MAX_MANIFEST_BYTES = 64 * 1024
DEPLOYMENT_DIRECTORY = Path(".local/share/visiontactile/tracker-deployment")
_IDENTITY = re.compile(r"[0-9a-f]{64}")
_SITE = re.compile(r"venv/lib/python[0-9]+\.[0-9]+/site-packages")
_FIXED_PATHS = {
    "source": "source",
    "python": "venv/bin/python",
    "bundle": "inputs/bundle.json",
    "roles": "inputs/roles.yaml",
    "capture": "inputs/capture.pcapng",
}


@dataclass(frozen=True)
class Installation:
    installation_id: str
    source: Path
    python: Path
    site_packages: Path
    bundle: Path
    roles: Path
    capture: Path
    capture_sha256: str


def deployment_directory(home=None):
    return (Path.home() if home is None else Path(home).expanduser().absolute()) / DEPLOYMENT_DIRECTORY


def _pairs(items):
    values = {}
    for key, value in items:
        if key in values:
            raise ValueError("Tracker installation metadata contains duplicate fields")
        values[key] = value
    return values


def _private_directory(descriptor):
    info = os.fstat(descriptor)
    if (not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid()
            or stat.S_IMODE(info.st_mode) != 0o700):
        raise ValueError("Tracker installation directories must be owned by the current user with mode 0700")


def _directory_at(name, parent, *, private=False):
    descriptor = os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=parent)
    try:
        if private:
            _private_directory(descriptor)
    except BaseException:
        os.close(descriptor)
        raise
    return descriptor


def _deployment_fd(path):
    descriptor = os.open(path.anchor, os.O_RDONLY | os.O_DIRECTORY)
    try:
        for part in path.parts[1:]:
            child = _directory_at(part, descriptor)
            os.close(descriptor)
            descriptor = child
        _private_directory(descriptor)
        result, descriptor = descriptor, None
        return result
    finally:
        if descriptor is not None:
            os.close(descriptor)


def _document(name, parent):
    descriptor = os.open(name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=parent)
    with os.fdopen(descriptor, "rb") as stream:
        info = os.fstat(stream.fileno())
        if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid()
                or stat.S_IMODE(info.st_mode) != 0o600 or info.st_size > MAX_MANIFEST_BYTES):
            raise ValueError("Tracker installation metadata must be an owner-only 0600 file no larger than 64 KiB")
        raw = stream.read(MAX_MANIFEST_BYTES + 1)
        if len(raw) > MAX_MANIFEST_BYTES:
            raise ValueError("Tracker installation metadata exceeds 64 KiB")
        try:
            return json.loads(raw, object_pairs_hook=_pairs)
        except (ValueError, UnicodeError, RecursionError) as exc:
            raise ValueError("Tracker installation metadata is invalid JSON") from exc


def _identity(value):
    if not isinstance(value, str) or _IDENTITY.fullmatch(value) is None:
        raise ValueError("Tracker installation identifiers must be 64 lowercase hexadecimal characters")
    return value


def _validate_document(value, identity):
    if (type(value) is not dict
            or set(value) != {"schema_version", "installation_id", "capture_sha256", "paths"}
            or type(value["schema_version"]) is not int or value["schema_version"] != 1
            or value["installation_id"] != identity):
        raise ValueError("Tracker installation metadata has an unsupported schema or mismatched identity")
    _identity(value["capture_sha256"])
    paths = value["paths"]
    if type(paths) is not dict or set(paths) != set(_FIXED_PATHS) | {"site_packages"}:
        raise ValueError("Tracker installation metadata has invalid path fields")
    if any(paths[key] != relative for key, relative in _FIXED_PATHS.items()):
        raise ValueError("Tracker installation paths must use the fixed local installation layout")
    site = paths["site_packages"]
    if not isinstance(site, str) or _SITE.fullmatch(site) is None:
        raise ValueError("Tracker installation site-packages must be inside its local virtual environment")
    return paths


def _check_path(relative, version, *, kind, private=False):
    """Inspect metadata through no-follow parents; a venv Python link is expected."""
    parts = Path(relative).parts
    descriptor = os.dup(version)
    try:
        for part in parts[:-1]:
            child = _directory_at(part, descriptor)
            os.close(descriptor)
            descriptor = child
        info = os.stat(parts[-1], dir_fd=descriptor, follow_symlinks=kind == "python")
        if kind == "directory":
            valid = stat.S_ISDIR(info.st_mode) and info.st_uid == os.getuid()
        else:
            valid = stat.S_ISREG(info.st_mode)
        if kind == "python":
            valid = valid and os.access(parts[-1], os.X_OK, dir_fd=descriptor)
        if private:
            valid = valid and info.st_uid == os.getuid() and stat.S_IMODE(info.st_mode) == 0o600
        if not valid:
            raise ValueError("Tracker installation contains a missing, unsafe, or unusable runtime path")
    finally:
        os.close(descriptor)


def load_installation(home=None, installation_id=None) -> Installation | None:
    """Read current or explicitly selected installation metadata without side effects.

    No installation/descriptor is ``None``. Malformed, unsafe, or incomplete
    installations raise ``ValueError``; callers must not treat them as ready.
    One current pointer is read, and its immutable version is used throughout.
    """
    if installation_id is not None:
        _identity(installation_id)
    base = deployment_directory(home)
    descriptors = []
    try:
        if not base.exists() and not base.is_symlink():
            return None
        if not all(hasattr(os, name) for name in ("O_DIRECTORY", "O_NOFOLLOW", "getuid")):
            raise ValueError("Tracker installation metadata is unsupported on this platform")
        try:
            parent = _deployment_fd(base)
        except FileNotFoundError:
            return None
        descriptors.append(parent)
        if installation_id is None:
            try:
                current = _document("current.json", parent)
            except FileNotFoundError:
                return None
            if (type(current) is not dict or set(current) != {"schema_version", "installation_id"}
                    or type(current["schema_version"]) is not int or current["schema_version"] != 1):
                raise ValueError("Tracker installation activation metadata has an unsupported schema")
            installation_id = _identity(current["installation_id"])
        try:
            versions = _directory_at("versions", parent, private=True)
            descriptors.append(versions)
            version = _directory_at(installation_id, versions, private=True)
            descriptors.append(version)
            document = _document("installation.json", version)
        except FileNotFoundError:
            return None
        paths = _validate_document(document, installation_id)
        for key, relative in paths.items():
            _check_path(relative, version,
                        kind="directory" if key in {"source", "site_packages"} else
                             "python" if key == "python" else "file",
                        private=key in {"bundle", "roles", "capture"})
        root = base / "versions" / installation_id
        return Installation(installation_id=installation_id,
                            capture_sha256=document["capture_sha256"],
                            **{key: root / relative for key, relative in paths.items()})
    except (OSError, RuntimeError) as exc:
        raise ValueError("Tracker installation cannot be read safely; check its files and permissions") from exc
    finally:
        for descriptor in reversed(descriptors):
            os.close(descriptor)


def for_bundle(path, home=None) -> Installation | None:
    """Resolve a managed bundle's own version, regardless of the current pointer."""
    if not isinstance(path, (str, os.PathLike)) or not str(path):
        return None
    try:
        candidate = Path(path).expanduser()
        if not candidate.is_absolute():
            return None
        relative = candidate.relative_to(deployment_directory(home) / "versions")
    except (ValueError, OSError, RuntimeError):
        return None
    parts = relative.parts
    if (len(parts) != 3 or parts[1:] != ("inputs", "bundle.json")
            or _IDENTITY.fullmatch(parts[0]) is None):
        return None
    installation = load_installation(home=home, installation_id=parts[0])
    return installation if installation is not None and installation.bundle == candidate else None
