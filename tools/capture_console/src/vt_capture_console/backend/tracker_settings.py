"""Private, local Tracker initialization preferences; never hardware actions.

Only file locations and the explicit capture digest are persisted. Credential
contents, command results and execution-approval checkboxes are not settings.
Opening/saving does not hash a capture, launch a process, read a bundle, access
HID, or use the network. Missing private inputs remain visible for repair;
missing software paths can use a unique standard local installation.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import re
import stat
import uuid

from .tracker_workflow import BootstrapConfig, bootstrap_environment_overrides
from .environment_discovery import discover_bootstrap_configuration, managed_tracker_installation
from .workstation_settings import repository_root

FIELDS = frozenset(("python", "script", "capture", "bundle", "expected_sha256"))
STORE_FILENAME = "tracker-bootstrap.json"
MAX_STORE_BYTES = 64 * 1024
_MISSING = object()


def _pairs(items):
    result = {}
    for key, value in items:
        if key in result:
            raise ValueError("Tracker 设置包含重复 JSON 字段")
        result[key] = value
    return result


def _values(value):
    if not isinstance(value, dict) or set(value) != FIELDS:
        raise ValueError("Tracker 设置仅允许 python/script/capture/bundle/expected_sha256 五个字段；不保存凭据正文或执行批准")
    result = {}
    for name in ("python", "script", "capture", "bundle"):
        text = value[name]
        if (not isinstance(text, str) or not text or len(text) > 4096
                or any(character in text for character in "\0\r\n")):
            raise ValueError(f"Tracker 设置 {name} 必须为明确的本地路径")
        path = Path(text).expanduser()
        if not path.is_absolute():
            raise ValueError(f"Tracker 设置 {name} 必须为绝对路径")
        result[name] = str(path)
    if Path(result["script"]).name != "live_windows_bootstrap.py":
        raise ValueError("Tracker 设置只接受 live_windows_bootstrap.py 脚本")
    digest = value["expected_sha256"]
    if not isinstance(digest, str) or re.fullmatch(r"[a-fA-F0-9]{64}", digest) is None:
        raise ValueError("Tracker 设置必须包含明确的 64 位 Capture SHA-256")
    result["expected_sha256"] = digest.lower()
    return result


class TrackerBootstrapSettings:
    def __init__(self, repo_root=None, settings_dir=None):
        self.repo_root = Path(repo_root or os.environ.get("VT_REPO") or repository_root()).expanduser().absolute()
        self.directory = Path(settings_dir or Path.home() / ".config/visiontactile/workbench").expanduser().absolute()
        self.path = self.directory / STORE_FILENAME

    def _directory_fd(self, *, create=False):
        """Walk with no-follow dirfds; never chmod or traverse a store symlink."""
        descriptor = os.open(self.directory.anchor, os.O_RDONLY | os.O_DIRECTORY)
        try:
            for component in self.directory.parts[1:]:
                try:
                    child = os.open(component, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW,
                                    dir_fd=descriptor)
                except FileNotFoundError:
                    if not create:
                        return None
                    try:
                        os.mkdir(component, mode=0o700, dir_fd=descriptor)
                    except FileExistsError:
                        pass
                    child = os.open(component, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW,
                                    dir_fd=descriptor)
                os.close(descriptor)
                descriptor = child
            info = os.fstat(descriptor)
            if info.st_uid != os.getuid() or stat.S_IMODE(info.st_mode) != 0o700:
                raise ValueError("Tracker 设置目录必须属于当前用户且权限为 0700")
            result, descriptor = descriptor, None
            return result
        except OSError as exc:
            raise ValueError("无法访问私有 Tracker 设置目录；不允许目录链接或非目录路径") from exc
        finally:
            if descriptor is not None:
                os.close(descriptor)

    @staticmethod
    def _document(descriptor, name):
        try:
            handle = os.open(name, os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW, dir_fd=descriptor)
        except FileNotFoundError:
            return _MISSING
        except OSError as exc:
            raise ValueError("无法读取 Tracker 设置；不允许设置文件链接") from exc
        with os.fdopen(handle, "r", encoding="utf-8") as stream:
            info = os.fstat(stream.fileno())
            if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid()
                    or stat.S_IMODE(info.st_mode) != 0o600 or info.st_size > MAX_STORE_BYTES):
                raise ValueError("Tracker 设置文件必须是当前用户拥有的 0600 常规文件，且不超过 64 KiB")
            try:
                return json.load(stream, object_pairs_hook=_pairs)
            except (ValueError, UnicodeError, RecursionError) as exc:
                raise ValueError("Tracker 设置文件损坏或格式无效；请修正原文件，不能直接覆盖") from exc

    def _load(self, descriptor):
        document = self._document(descriptor, self.path.name)
        if document is _MISSING:
            return {}
        if (not isinstance(document, dict) or set(document) != {"schema_version", "values"}
                or type(document["schema_version"]) is not int or document["schema_version"] != 1):
            raise ValueError("Tracker 设置 schema 无效；仅接受版本 1，不会覆盖原文件")
        return _values(document["values"])

    def load(self):
        descriptor = self._directory_fd()
        if descriptor is None:
            return {}
        try:
            # Structural validation only: migrated/offline paths must remain
            # visible for the user to repair, rather than silently disappearing.
            return self._load(descriptor)
        finally:
            os.close(descriptor)

    def save(self, values):
        self.load()  # Refuse to overwrite a malformed/non-private existing store.
        clean = _values(values)
        BootstrapConfig(**clean).argv()  # Validate metadata, never execute argv.
        descriptor = self._directory_fd(create=True)
        temporary = "." + self.path.name + "." + uuid.uuid4().hex + ".tmp"
        try:
            self._load(descriptor)
            handle = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW,
                             0o600, dir_fd=descriptor)
            with os.fdopen(handle, "w", encoding="utf-8") as stream:
                os.fchmod(stream.fileno(), 0o600)
                json.dump({"schema_version": 1, "values": clean}, stream, ensure_ascii=False,
                          allow_nan=False, indent=2)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            self._load(descriptor)
            os.replace(temporary, self.path.name, src_dir_fd=descriptor, dst_dir_fd=descriptor)
            os.fsync(descriptor)
        finally:
            try:
                os.unlink(temporary, dir_fd=descriptor)
            except FileNotFoundError:
                pass
            os.close(descriptor)

    def _workstation_parameters(self):
        descriptor = self._directory_fd()
        if descriptor is None:
            return {}
        try:
            document = self._document(descriptor, "workstation.json")
        finally:
            os.close(descriptor)
        if document is _MISSING:
            return {}
        from vt_capture_core.workstation import validate_configuration

        configuration = validate_configuration(document)
        for group in configuration["process_groups"]:
            if group.get("preset") == "trackers":
                parameters = dict(group.get("parameters", {}))
                bundle = parameters.get("bundle_path", "")
                if bundle and Path(bundle).expanduser().resolve().is_relative_to(self.repo_root.resolve()):
                    parameters["bundle_path"] = ""
                return parameters
        return {}

    def defaults(self):
        saved = self.load()
        workstation = self._workstation_parameters()
        workstation_bundle = workstation.get("bundle_path", "")
        # The publisher's explicitly selected managed bundle pins all initializer
        # inputs. An older saved initializer cannot silently select another rig.
        managed = (managed_tracker_installation(workstation_bundle, current=False)
                   if workstation_bundle else None)
        values = {key: "" for key in FIELDS}
        if managed is not None:
            values["bundle"] = str(managed.bundle)
        else:
            values.update(saved)
            if workstation_bundle:
                # A newly selected device set cannot inherit the previous set's
                # capture or approved digest. Software paths are independent.
                previous = values.get("bundle")
                if previous and (Path(previous).expanduser().resolve()
                                 != Path(workstation_bundle).expanduser().resolve()):
                    values["capture"] = values["expected_sha256"] = ""
                values["bundle"] = workstation_bundle
        values = discover_bootstrap_configuration(
            values, repo_root=self.repo_root, environ={}, replace_missing=True,
            workstation_parameters=workstation).configuration
        values.update(bootstrap_environment_overrides())
        return values
