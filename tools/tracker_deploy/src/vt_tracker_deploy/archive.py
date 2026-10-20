"""Bounded private Tracker snapshots, without importing their code or opening HID.

The package is an operator-selected software payload, not a signature or proof
of hardware approval. Hashes bind every cached payload byte to its manifest.
"""
from __future__ import annotations

from dataclasses import dataclass
from email.parser import BytesParser
from hashlib import sha1, sha256
import io
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat
import subprocess
from types import MappingProxyType
from typing import Mapping
import zipfile


MAX_FILES = 512
MAX_FILE_BYTES = 64 * 1024 * 1024
MAX_TOTAL_BYTES = 128 * 1024 * 1024
MAX_MANIFEST_BYTES = 256 * 1024
MAX_PACKAGE_BYTES = MAX_TOTAL_BYTES + 2 * 1024 * 1024
MAX_WHEEL_EXPANDED_BYTES = 128 * 1024 * 1024
REQUIREMENTS = ("hid==1.0.9", "numpy==1.26.4", "scapy==2.7.0", "PyYAML==6.0.2")
TARGET = {"system": "Linux", "machine": "x86_64", "python": "3.12"}
SOURCE_ORIGIN = "https://github.com/yanglei18/pyvut"
SOURCE_NOTES = "DEPLOYMENT_CHANGES.md"
# Transitive imports of the approved bootstrap and ROS read-only pose source.
# Deliberately exclude legacy tracker_core/api, WiFi examples, tests and captures.
RUNTIME_SOURCE_FILES = (
    "LICENSE", "NOTICE", "pyvut/__init__.py",
    "pyvut/bootstrap_capture.py", "pyvut/bootstrap_profile.py",
    "pyvut/bootstrap_protocol.py", "pyvut/bootstrap_state_machine.py",
    "pyvut/bootstrap_transport.py", "pyvut/enums_horusd_dongle.py", "pyvut/enums_usb.py",
    "pyvut/live_bootstrap_bundle.py", "pyvut/live_bootstrap_events.py",
    "pyvut/live_bootstrap_evidence.py", "pyvut/live_bootstrap_policy.py",
    "pyvut/live_bootstrap_router.py", "pyvut/live_bootstrap_runner.py",
    "pyvut/live_bootstrap_transport.py", "pyvut/live_bundle_addresses.py",
    "pyvut/live_hid.py", "pyvut/live_models.py", "pyvut/live_observe.py",
    "pyvut/live_policy.py", "pyvut/live_probe.py", "pyvut/live_wifi_credentials.py",
    "pyvut/pose_decoder.py", "scripts/live_windows_bootstrap.py",
)
_INPUTS = frozenset(("inputs/bundle.json", "inputs/roles.yaml", "inputs/capture.pcapng"))
_DIGEST = re.compile(r"[0-9a-f]{64}\Z")
_REVISION = re.compile(r"(?:[0-9a-f]{40}|[0-9a-f]{64})\Z")
_NORMALIZE = re.compile(r"[-_.]+")


@dataclass(frozen=True)
class VerifiedPackage:
    manifest: dict
    files: Mapping[str, bytes]
    package_id: str


def _name(value):
    if (type(value) is not str or not value or len(value) > 240
            or "\\" in value or ":" in value
            or any(ord(char) < 32 or ord(char) == 127 for char in value)):
        raise ValueError("Invalid package member path")
    parts = value.split("/")
    if value.startswith("/") or any(part in {"", ".", ".."} for part in parts):
        raise ValueError("Invalid package member path")
    return value


def _digest(value):
    if type(value) is not str or _DIGEST.fullmatch(value) is None:
        raise ValueError("Expected a lowercase SHA-256 digest")
    return value


def _pairs(items):
    result = {}
    for key, value in items:
        if key in result:
            raise ValueError("Duplicate JSON key")
        result[key] = value
    return result


def _nonfinite(_value):
    raise ValueError("Non-finite JSON value")


def _json(data):
    try:
        return json.loads(data.decode("utf-8"), object_pairs_hook=_pairs,
                          parse_constant=_nonfinite)
    except (UnicodeError, RecursionError, json.JSONDecodeError) as exc:
        raise ValueError("Invalid package JSON") from exc


def _open_directory(path):
    """Open every path component without following a directory symlink."""
    path = Path(path).expanduser().absolute()
    if ".." in path.parts:
        raise ValueError("Parent traversal is not permitted in local package paths")
    descriptor = os.open(path.anchor, os.O_RDONLY | os.O_DIRECTORY)
    try:
        for part in path.parts[1:]:
            child = os.open(part, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=descriptor)
            os.close(descriptor)
            descriptor = child
        result, descriptor = descriptor, None
        return result
    except OSError as exc:
        raise ValueError("Package paths must use existing directories without symlinks") from exc
    finally:
        if descriptor is not None:
            os.close(descriptor)


def _read_file(path, limit):
    """Capture bytes from one bounded regular file with no symlink traversal."""
    path = Path(path).expanduser().absolute()
    parent = _open_directory(path.parent)
    descriptor = None
    try:
        descriptor = os.open(path.name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=parent)
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode) or not 0 <= before.st_size <= limit:
            raise ValueError("Package input is not a bounded regular file")
        chunks, remaining = [], limit + 1
        while remaining:
            part = os.read(descriptor, min(1024 * 1024, remaining))
            if not part:
                break
            chunks.append(part)
            remaining -= len(part)
        data = b"".join(chunks)
        after = os.fstat(descriptor)
        identity = lambda value: (value.st_dev, value.st_ino, value.st_size,
                                  value.st_mtime_ns, value.st_ctime_ns)
        if len(data) > limit or len(data) != before.st_size or identity(before) != identity(after):
            raise ValueError("Package input changed while being read or exceeds its limit")
        return data
    except OSError as exc:
        raise ValueError("Package input must be a readable regular file without symlinks") from exc
    finally:
        if descriptor is not None:
            os.close(descriptor)
        os.close(parent)


def _git(source, *arguments):
    environment = {key: value for key, value in os.environ.items() if not key.startswith("GIT_")}
    environment.update(GIT_OPTIONAL_LOCKS="0", GIT_CONFIG_NOSYSTEM="1", GIT_CONFIG_GLOBAL="/dev/null")
    try:
        process = subprocess.run(
            ["git", "-c", "core.fsmonitor=false", "-c", "core.hooksPath=/dev/null",
             "-C", str(source), *arguments], check=True, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, timeout=10, env=environment)
    except (OSError, subprocess.SubprocessError) as exc:
        raise ValueError("Cannot read source checkout provenance") from exc
    if len(process.stdout) > MAX_MANIFEST_BYTES:
        raise ValueError("Source checkout metadata exceeds its size limit")
    return process.stdout


def _modified_copy(data, revision):
    """Add a modification notice while retaining a shebang/encoding cookie."""
    prefix = b"\xef\xbb\xbf" if data.startswith(b"\xef\xbb\xbf") else b""
    lines = data[len(prefix):].splitlines(keepends=True)
    index = 1 if lines and lines[0].startswith(b"#!") else 0
    for offset, line in enumerate(lines[:2]):
        if re.search(br"coding[:=]\s*[-\w.]+", line):
            index = max(index, offset + 1)
    notice = (
        f"# Modified deployment snapshot relative to Git revision {revision}.\n"
        "# See DEPLOYMENT_CHANGES.md for provenance and original source SHA-256.\n"
    ).encode("ascii")
    leading = b"".join(lines[:index])
    if leading and not leading.endswith(b"\n"):
        leading += b"\n"
    return prefix + leading + notice + b"".join(lines[index:])


def _add(files, name, data):
    if len(data) > MAX_FILE_BYTES or sum(map(len, files.values())) + len(data) > MAX_TOTAL_BYTES:
        raise ValueError("Expanded package exceeds its size limit")
    files[name] = data


def _source_files(source):
    source = Path(source).expanduser().absolute()
    revision = _git(source, "rev-parse", "HEAD").decode("ascii", errors="replace").strip()
    if _REVISION.fullmatch(revision) is None:
        raise ValueError("Invalid source checkout revision")
    entries = _git(source, "ls-tree", "-r", "-z", revision, "--", *RUNTIME_SOURCE_FILES)
    tracked = {}
    try:
        for entry in entries.split(b"\0"):
            if not entry:
                continue
            metadata, name = entry.decode("utf-8").split("\t", 1)
            mode, kind, identifier = metadata.split()
            if mode not in {"100644", "100755"} or kind != "blob":
                raise ValueError("Source allowlist must contain regular tracked files")
            tracked[name] = identifier
    except (UnicodeError, ValueError) as exc:
        raise ValueError("Invalid source checkout file metadata") from exc
    if set(tracked) != set(RUNTIME_SOURCE_FILES):
        raise ValueError("Source checkout is missing required tracked runtime files")
    files, modified, notes = {}, [], []
    for name in sorted(RUNTIME_SOURCE_FILES):
        data = _read_file(source / name, MAX_FILE_BYTES)
        object_digest = sha1 if len(revision) == 40 else sha256
        identifier = object_digest(b"blob " + str(len(data)).encode("ascii") + b"\0" + data).hexdigest()
        if identifier != tracked[name]:
            modified.append(name)
            notes.append(f"- {name}: original working-copy SHA-256 {sha256(data).hexdigest()}")
            if name.endswith(".py"):
                data = _modified_copy(data, revision)
        _add(files, "source/" + name, data)
    text = (
        "# Deployment source provenance\n\n"
        f"Origin: {SOURCE_ORIGIN}\n\nGit revision: {revision}\n\n"
        "This is a selected runtime snapshot. It retains the checkout's current\n"
        "working-copy corrections; it is not a pristine export of the revision.\n"
        "LICENSE and NOTICE accompany the snapshot. Modified Python copies carry\n"
        "a prominent header; the original checkout was not changed by packaging.\n\n"
        "Changes relative to the recorded Git revision:\n\n"
        + ("\n".join(notes) if notes else "- No selected runtime files were modified.")
        + "\n"
    )
    _add(files, "source/" + SOURCE_NOTES, text.encode("utf-8"))
    return files, dict(origin=SOURCE_ORIGIN, revision=revision, modified_files=modified)


def _compatible_tag(python, abi, platform):
    pythons, abis, platforms = python.split("."), abi.split("."), platform.split(".")
    if "py3" in pythons and "none" in abis and "any" in platforms:
        return True
    return ("cp312" in pythons and any(value in {"cp312", "abi3", "none"} for value in abis)
            and any(value == "linux_x86_64" or
                    value.startswith("manylinux") and value.endswith("_x86_64")
                    for value in platforms))


def _wheel_identity(filename):
    _name(filename)
    if "/" in filename or not filename.endswith(".whl"):
        raise ValueError("Expected a wheel filename")
    parts = filename[:-4].split("-")
    if len(parts) not in {5, 6} or any(re.fullmatch(r"[A-Za-z0-9_.]+", part) is None for part in parts):
        raise ValueError("Invalid wheel filename")
    if len(parts) == 6 and not parts[2][0].isdigit():
        raise ValueError("Invalid wheel build tag")
    name, version = _NORMALIZE.sub("-", parts[0]).lower(), parts[1]
    if not _compatible_tag(*parts[-3:]):
        raise ValueError("Wheel is incompatible with CPython 3.12 on Linux x86_64")
    return name, version, "-".join(parts[-3:])


def _member(archive, info, limit):
    if info.file_size > limit:
        raise ValueError("Archive member exceeds its size limit")
    with archive.open(info) as stream:
        data = stream.read(limit + 1)
    if len(data) > limit or len(data) != info.file_size:
        raise ValueError("Archive member has an invalid expanded size")
    return data


def _wheel_metadata(filename, data):
    name, version, filename_tag = _wheel_identity(filename)
    try:
        with zipfile.ZipFile(io.BytesIO(data)) as wheel:
            infos = wheel.infolist()
            names = [item.filename for item in infos]
            if len(infos) > 10000 or len(names) != len(set(names)):
                raise ValueError("Wheel has an invalid file inventory")
            if sum(info.file_size for info in infos) > MAX_WHEEL_EXPANDED_BYTES:
                raise ValueError("Expanded wheel exceeds its size limit")
            for info in infos:
                if info.orig_filename != info.filename:
                    raise ValueError("Wheel contains a noncanonical member name")
                if info.is_dir():
                    _name(info.filename[:-1])
                else:
                    _name(info.filename)
                kind = stat.S_IFMT(info.external_attr >> 16)
                if (kind not in {0, stat.S_IFREG, stat.S_IFDIR} or info.flag_bits & 1
                        or not 0 <= info.file_size <= MAX_FILE_BYTES
                        or kind == stat.S_IFDIR and not info.is_dir()
                        or info.compress_type not in {zipfile.ZIP_STORED, zipfile.ZIP_DEFLATED}):
                    raise ValueError("Wheel contains an unsupported member")
            matches = [item for item in infos if item.filename.endswith(".dist-info/METADATA")]
            if len(matches) != 1:
                raise ValueError("Wheel must contain one metadata document")
            metadata = BytesParser().parsebytes(_member(wheel, matches[0], MAX_MANIFEST_BYTES))
            if (len(metadata.get_all("Name", [])) != 1 or len(metadata.get_all("Version", [])) != 1
                    or _NORMALIZE.sub("-", metadata["Name"]).lower() != name
                    or metadata["Version"] != version):
                raise ValueError("Wheel metadata does not match its filename")
            wheel_name = str(PurePosixPath(matches[0].filename).parent / "WHEEL")
            headers = BytesParser().parsebytes(_member(wheel, wheel.getinfo(wheel_name), MAX_MANIFEST_BYTES))
            tags = headers.get_all("Tag", [])
            allowed = set()
            for tag in tags:
                fields = tag.split("-")
                if len(fields) == 3 and _compatible_tag(*fields):
                    allowed.add(tag)
            # Compressed filename tags represent the Cartesian product.
            fields = filename_tag.split("-")
            filename_tags = {f"{p}-{a}-{s}" for p in fields[0].split(".")
                             for a in fields[1].split(".") for s in fields[2].split(".")}
            if not allowed & filename_tags:
                raise ValueError("Wheel metadata has no matching supported tag")
    except (OSError, EOFError, RuntimeError, KeyError, zipfile.BadZipFile, NotImplementedError) as exc:
        raise ValueError("Invalid wheel archive") from exc
    return name, version


def _validate_wheels(files):
    expected = {_NORMALIZE.sub("-", item.split("==")[0]).lower(): item.split("==")[1]
                for item in REQUIREMENTS}
    found = {}
    for path, data in files.items():
        if not path.startswith("wheels/"):
            continue
        name, version = _wheel_metadata(path.removeprefix("wheels/"), data)
        if name in found or expected.get(name) != version:
            raise ValueError("Wheel inventory must match the pinned requirements exactly")
        found[name] = version
    if found != expected:
        raise ValueError("Wheel inventory must match the pinned requirements exactly")


def _manifest(manifest):
    fields = {"schema_version", "kind", "target", "capture_sha256", "source", "requirements", "files"}
    if (type(manifest) is not dict or set(manifest) != fields
            or type(manifest["schema_version"]) is not int or manifest["schema_version"] != 1
            or manifest["kind"] != "visiontactile-tracker" or manifest["target"] != TARGET
            or manifest["requirements"] != list(REQUIREMENTS)):
        raise ValueError("Unsupported Tracker package manifest")
    _digest(manifest["capture_sha256"])
    source = manifest["source"]
    if (type(source) is not dict or set(source) != {"origin", "revision", "modified_files"}
            or source["origin"] != SOURCE_ORIGIN
            or type(source["revision"]) is not str or _REVISION.fullmatch(source["revision"]) is None):
        raise ValueError("Invalid package source provenance")
    modified = source["modified_files"]
    if (type(modified) is not list or any(type(name) is not str for name in modified)
            or len(modified) != len(set(modified)) or modified != sorted(modified)
            or not set(modified) <= set(RUNTIME_SOURCE_FILES)):
        raise ValueError("Invalid modified source inventory")
    inventory = manifest["files"]
    if type(inventory) is not list or len(inventory) >= MAX_FILES:
        raise ValueError("Invalid package file inventory")
    paths, total = {}, 0
    for item in inventory:
        if type(item) is not dict or set(item) != {"path", "bytes", "sha256"}:
            raise ValueError("Invalid package file entry")
        name = _name(item["path"])
        if name in paths or type(item["bytes"]) is not int or not 0 <= item["bytes"] <= MAX_FILE_BYTES:
            raise ValueError("Invalid package file entry")
        _digest(item["sha256"])
        total += item["bytes"]
        if total > MAX_TOTAL_BYTES:
            raise ValueError("Expanded package exceeds its size limit")
        paths[name] = item
    required = {"source/" + name for name in (*RUNTIME_SOURCE_FILES, SOURCE_NOTES)} | _INPUTS
    if not required <= paths.keys():
        raise ValueError("Package is missing required source or input files")
    extra = paths.keys() - required
    if len(extra) != len(REQUIREMENTS) or any(not name.startswith("wheels/") for name in extra):
        raise ValueError("Package contains unexpected files")
    for name in extra:
        _wheel_identity(name.removeprefix("wheels/"))
    return paths


def _verify_payload(manifest, files):
    inventory = _manifest(manifest)
    if set(inventory) != set(files):
        raise ValueError("Manifest and archive file inventories differ")
    for name, data in files.items():
        entry = inventory[name]
        if len(data) != entry["bytes"] or sha256(data).hexdigest() != entry["sha256"]:
            raise ValueError("Package content does not match its manifest")
    if sha256(files["inputs/capture.pcapng"]).hexdigest() != manifest["capture_sha256"]:
        raise ValueError("Capture does not match its approved digest")
    bundle = _json(files["inputs/bundle.json"])
    if (type(bundle) is not dict or type(bundle.get("schema_version")) is not int
            or bundle["schema_version"] != 3
            or bundle.get("capture_sha256") != manifest["capture_sha256"]):
        raise ValueError("Bundle schema or approved capture digest is inconsistent")
    if not files["inputs/roles.yaml"]:
        raise ValueError("Role map must not be empty")
    _validate_wheels(files)


def build_package(*, source: Path, bundle: Path, roles: Path, capture: Path,
                  capture_sha256: str, wheelhouse: Path, output: Path) -> dict:
    """Create a new owner-only package from the selected working-copy bytes."""
    _digest(capture_sha256)
    output = Path(output).expanduser().absolute()
    if output.suffix != ".vttracker":
        raise ValueError("Tracker packages must use the .vttracker extension")
    files, provenance = _source_files(source)
    for key, path in (("bundle.json", bundle), ("roles.yaml", roles), ("capture.pcapng", capture)):
        _add(files, "inputs/" + key, _read_file(path, MAX_FILE_BYTES))
    directory = _open_directory(wheelhouse)
    try:
        names = os.listdir(directory)
    finally:
        os.close(directory)
    if len(names) > MAX_FILES:
        raise ValueError("Wheel directory contains too many entries")
    if sum(name.endswith(".whl") for name in names) != len(REQUIREMENTS):
        raise ValueError("Wheel inventory must match the pinned requirements exactly")
    for name in sorted(names):
        if name.endswith(".whl"):
            _wheel_identity(name)
            _add(files, "wheels/" + name, _read_file(Path(wheelhouse) / name, MAX_FILE_BYTES))
    manifest = dict(schema_version=1, kind="visiontactile-tracker", target=dict(TARGET),
                    capture_sha256=capture_sha256, source=provenance, requirements=list(REQUIREMENTS),
                    files=[dict(path=name, bytes=len(data), sha256=sha256(data).hexdigest())
                           for name, data in sorted(files.items())])
    _verify_payload(manifest, files)
    document = (json.dumps(manifest, ensure_ascii=False, sort_keys=True, indent=2) + "\n").encode("utf-8")
    if len(document) > MAX_MANIFEST_BYTES or sum(map(len, files.values())) + len(document) > MAX_TOTAL_BYTES:
        raise ValueError("Expanded package exceeds its size limit")
    parent = _open_directory(output.parent)
    created = None
    try:
        descriptor = os.open(output.name, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW,
                             0o600, dir_fd=parent)
        created = os.fstat(descriptor)
        with os.fdopen(descriptor, "wb") as stream:
            os.fchmod(stream.fileno(), 0o600)
            with zipfile.ZipFile(stream, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
                for name, data in (("manifest.json", document), *sorted(files.items())):
                    info = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
                    info.create_system = 3
                    info.external_attr = (stat.S_IFREG | 0o600) << 16
                    info.compress_type = zipfile.ZIP_DEFLATED
                    archive.writestr(info, data)
            stream.flush()
            os.fsync(stream.fileno())
        os.fsync(parent)
        current = os.stat(output.name, dir_fd=parent, follow_symlinks=False)
        if (current.st_dev, current.st_ino) != (created.st_dev, created.st_ino):
            raise ValueError("Package output changed while being written")
    except BaseException:
        if created is not None:
            try:
                current = os.stat(output.name, dir_fd=parent, follow_symlinks=False)
                if (current.st_dev, current.st_ino) == (created.st_dev, created.st_ino):
                    os.unlink(output.name, dir_fd=parent)
            except FileNotFoundError:
                pass
        raise
    finally:
        os.close(parent)
    return manifest


def read_package(path) -> VerifiedPackage:
    """Read once, reject unsafe ZIPs, and return only verified cached bytes."""
    data = _read_file(path, MAX_PACKAGE_BYTES)
    try:
        with zipfile.ZipFile(io.BytesIO(data)) as archive:
            infos = archive.infolist()
            if not 1 <= len(infos) <= MAX_FILES:
                raise ValueError("Package contains too many files")
            names, total = set(), 0
            for info in infos:
                if info.orig_filename != info.filename:
                    raise ValueError("Package contains a noncanonical member name")
                name = _name(info.filename)
                if name in names:
                    raise ValueError("Duplicate archive member")
                names.add(name)
                kind = stat.S_IFMT(info.external_attr >> 16)
                if (info.is_dir() or kind not in {0, stat.S_IFREG} or info.flag_bits & 1
                        or info.compress_type not in {zipfile.ZIP_STORED, zipfile.ZIP_DEFLATED}):
                    raise ValueError("Unsupported archive member type")
                limit = MAX_MANIFEST_BYTES if name == "manifest.json" else MAX_FILE_BYTES
                if not 0 <= info.file_size <= limit:
                    raise ValueError("Archive member exceeds its size limit")
                total += info.file_size
                if total > MAX_TOTAL_BYTES:
                    raise ValueError("Expanded package exceeds its size limit")
            if "manifest.json" not in names:
                raise ValueError("Package manifest is missing")
            manifest = _json(_member(archive, archive.getinfo("manifest.json"), MAX_MANIFEST_BYTES))
            inventory = _manifest(manifest)
            if names != set(inventory) | {"manifest.json"}:
                raise ValueError("Manifest and archive file inventories differ")
            files = {name: _member(archive, archive.getinfo(name), MAX_FILE_BYTES) for name in inventory}
            _verify_payload(manifest, files)
    except (OSError, EOFError, RuntimeError, KeyError, zipfile.BadZipFile, NotImplementedError) as exc:
        raise ValueError("Invalid Tracker package archive") from exc
    return VerifiedPackage(manifest, MappingProxyType(files), sha256(data).hexdigest())
