#!/usr/bin/env python3
"""Build a local, device-specific delivery from public sources and a Tracker package.

This does not publish, install, execute bundled code, or access hardware. The
result contains private device material and is not a public source release.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat
import struct
import subprocess
import sys
import tarfile
import tempfile


ROOT = Path(__file__).resolve().parents[2]
PREFIX = "VisionTactile-Delivery/"
SOURCE_PREFIX = "Vision-Tactile-Dataset/"
INSTALLER = "tools/capture_deploy/install.sh"

# Load trusted tooling beside this script, never from the selected source root.
_SPEC = importlib.util.spec_from_file_location(
    "tracker_delivery_source_export", Path(__file__).with_name("prepare_release.py"))
release = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(release)
_BOOTSTRAP_SPEC = importlib.util.spec_from_file_location(
    "tracker_delivery_bootstrap_contract", Path(__file__).with_name("build_linux_installer.py"))
bootstrap_builder = importlib.util.module_from_spec(_BOOTSTRAP_SPEC)
_BOOTSTRAP_SPEC.loader.exec_module(bootstrap_builder)
BOOTSTRAP_NAME = bootstrap_builder.NAME
BOOTSTRAP_TARGET = bootstrap_builder.TARGET
BOOTSTRAP_SOURCE_PATTERNS = bootstrap_builder.SOURCE_PATTERNS
sys.path.insert(0, str(ROOT / "tools/tracker_deploy/src"))
from vt_tracker_deploy.archive import MAX_PACKAGE_BYTES, read_package  # noqa: E402


WRAPPER = b'''#!/usr/bin/env bash
set -euo pipefail
DELIVERY_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$DELIVERY_ROOT/Vision-Tactile-Dataset/tools/capture_deploy/install.sh" \\
    --tracker-package "$DELIVERY_ROOT/deployment/tracker.vttracker" "$@"
'''

FULL_WRAPPER = b'''#!/usr/bin/env bash
set -euo pipefail
DELIVERY_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "$DELIVERY_ROOT/Vision-Tactile-Dataset/tools/capture_deploy/install_full.py" \\
    --delivery "$DELIVERY_ROOT" --headless "$@"
'''

FULL_README = """Vision Tactile + Tracker 图形安装 / Graphical installation

适用：Ubuntu 24.04 x86_64 桌面电脑；首次安装需要联网。
1. 下载对应设备的一体交付包，解压整个 VisionTactile-Delivery 目录。
2. 双击 Install-VisionTactile，点击安装，输入本机管理员密码授权系统配置。
   安装器自动准备系统环境、ROS、相机驱动、工作台和随包 Tracker；无需填写依赖路径。
3. 安装完成后从应用菜单打开 Vision Tactile 采集工作台，按界面提示连接设备。

如文件管理器要求允许执行，请在文件属性中启用“允许作为程序执行”。
安装较久时请等待；取消会等待正在进行的系统软件包安装安全结束。
如提示需要重新登录，请注销并重新登录，使串口和设备用户组权限生效。
安装后的运行文件保存在用户应用目录；安装成功后可删除下载并解压的交付包。
安装器不启动硬件；Tracker 初始化通过工作台内的初始化向导进行。
手套外部 SDK 和需单独许可的手部模型不在本交付中。
故障时保留界面提示的日志；命令行入口：bash install.sh。

Target: Ubuntu 24.04 x86_64 desktop, with Internet access for first installation.
1. Download the delivery for your equipment and extract the complete VisionTactile-Delivery folder.
2. Double-click Install-VisionTactile, click Install, and authorize system setup with this computer's administrator password.
   The installer prepares system dependencies, ROS, camera drivers, the workbench and the included Tracker automatically.
3. Open Vision Tactile from the application menu and follow the device connection prompts.

If your file manager requires it, enable execution in the file's properties.
Cancellation waits for an active system package transaction to finish safely.
If requested, sign out and sign in again to activate device group permissions.
The installed files are copied to your user application directory. After successful installation,
the downloaded archive and extracted delivery can be removed. Installation does not start hardware;
use the workbench's Tracker initialization wizard. External glove SDKs and separately licensed hand
models are not included. Keep the installer log if setup fails. Terminal entry: bash install.sh.

本包含对应设备的身份、启动抓包和网络配置；仅交付给该设备的授权使用者，不公开上传。
This package contains private equipment identities, startup captures and network configuration.
Distribute only to authorized operators of the corresponding equipment, never as a public attachment.
版本与校验信息 / Version and integrity: DELIVERY_MANIFEST.json, BOOTSTRAP_MANIFEST.json.
""".encode("utf-8")

README = """Vision Tactile + Tracker 一体交付 / Combined delivery

适用环境：Ubuntu 24.04、x86_64、Python 3.12。
先按 Vision-Tactile-Dataset/docs 中的部署指南准备 ROS 和传感器基础环境。
按指南第 3 节“一体交付包”分支设置路径，以随包的 Vision-Tactile-Dataset
作为 VT_REPO 并构建 ROS，跳过 git clone 和 Git 查询，不要另取一份源码。
交付版本信息见 DELIVERY_MANIFEST.json。
解压后在此目录运行：bash install.sh
安装器会安装主程序，并导入随包的 Tracker 部署包；无需手动填写依赖路径。
已有有效设备配置会保留，不会强制改为此包中的配置。
主程序 pip 安装可能需要联网；Tracker 部署包包含其配套离线 Python 依赖。
手套依赖的外部 SDK 源码及需单独许可的手部模型不包含在此交付中。
安装完成后从应用菜单启动 Vision Tactile 采集工作台。
安装采用 editable 源码方式，请保留整个解压目录，不要移动或删除。
安装本身不会启动硬件；Tracker 初始化仍通过程序中的初始化向导操作。

此交付包含对应设备的身份、启动抓包及网络配置，只能向获授权的设备用户提供。
请勿上传到公开 GitHub 仓库或公开 Release；不能用于任意其他 Tracker。
源码公开清单仅涵盖 Vision-Tactile-Dataset；设备包独立位于 deployment。
DELIVERY_MANIFEST.json 记录实际源码摘要、HEAD 基线、工作区状态和设备包摘要。
HEAD 仅标识基线；工作区有修改时，本交付不等同于该提交。

Target: Ubuntu 24.04, x86_64, Python 3.12.
Prepare ROS and sensor prerequisites using the deployment guides under
Vision-Tactile-Dataset/docs. Follow the combined-delivery branch in section 3
to set paths and build ROS using the included source directory as VT_REPO;
skip git clone and Git revision queries, and do not fetch a second source copy.
See DELIVERY_MANIFEST.json for
delivery version information. Then run: bash install.sh
The existing installer installs the application and imports the included
Tracker package. Existing valid device configurations are retained.
Application pip dependencies may require Internet access;
the Tracker package includes its matching offline Python dependencies.
External glove SDK sources and separately licensed hand models are not included.
Launch the workbench from the application menu after installation. Hardware
is not started by installation; initialize the Tracker through its wizard.
Keep the complete extracted delivery directory in place after installation:
the application uses an editable source installation.

This delivery contains device identities, a startup capture, and network
configuration. Distribute only to authorized users of the corresponding
equipment. Do not upload it to a public repository or public Release.
PUBLIC_SOURCE_MANIFEST.json covers only the public application sources.
DELIVERY_MANIFEST.json records the captured source digest, baseline HEAD,
working-tree status, and package digest. A dirty snapshot is not that commit.
""".encode("utf-8")


class DeliveryError(ValueError):
    """A diagnostic safe to print without private package contents."""


def _package_bytes(path):
    try:
        package = read_package(path)
        descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        with os.fdopen(descriptor, "rb") as stream:
            info = os.fstat(stream.fileno())
            if not stat.S_ISREG(info.st_mode) or info.st_size > MAX_PACKAGE_BYTES:
                raise ValueError("Invalid package input")
            content = stream.read(MAX_PACKAGE_BYTES + 1)
        if (len(content) > MAX_PACKAGE_BYTES
                or hashlib.sha256(content).hexdigest() != package.package_id):
            raise ValueError("Package changed after validation")
        return content, package.package_id
    except (OSError, ValueError) as exc:
        raise DeliveryError("Tracker package validation failed or the package changed.") from exc


def _add(archive, name, content, mode=0o644):
    info = tarfile.TarInfo(PREFIX + name)
    info.size, info.mode, info.mtime = len(content), mode, 0
    archive.addfile(info, io.BytesIO(content))


def _bootstrap_read(directory, name, maximum):
    """Read a bounded regular file through non-following directory handles."""
    parts = PurePosixPath(name).parts
    if (not parts or name != PurePosixPath(name).as_posix() or ".." in parts
            or PurePosixPath(name).is_absolute() or "\\" in name or "\x00" in name):
        raise DeliveryError("Bootstrap manifest contains an unsafe path.")
    descriptor = os.open(directory, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    try:
        for part in parts[:-1]:
            child = os.open(part, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=descriptor)
            os.close(descriptor)
            descriptor = child
        file_descriptor = os.open(parts[-1], os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK,
                                  dir_fd=descriptor)
        with os.fdopen(file_descriptor, "rb") as stream:
            info = os.fstat(stream.fileno())
            if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_size > maximum:
                raise DeliveryError("Bootstrap file has an invalid type or size.")
            content = stream.read(maximum + 1)
        if len(content) > maximum:
            raise DeliveryError("Bootstrap file exceeds its allowed size.")
        return content
    finally:
        os.close(descriptor)


def _entries(items):
    if not isinstance(items, list) or len(items) > 256:
        raise DeliveryError("Bootstrap manifest has an invalid file list.")
    result = {}
    for item in items:
        if (not isinstance(item, dict) or set(item) != {"path", "bytes", "sha256"}
                or not isinstance(item["path"], str) or item["path"] in result
                or type(item["bytes"]) is not int or item["bytes"] < 0
                or not isinstance(item["sha256"], str)
                or not re.fullmatch(r"[0-9a-f]{64}", item["sha256"])):
            raise DeliveryError("Bootstrap manifest has invalid or duplicate entries.")
        result[item["path"]] = item
    return result


def _bootstrap_payload(directory, public_files):
    """Bind the binary to exactly the immutable sources used for public export."""
    directory = Path(directory).expanduser().absolute()
    try:
        manifest_bytes = _bootstrap_read(directory, "BUILD_MANIFEST.json", 4 * 1024 * 1024)
        manifest = json.loads(manifest_bytes)
        if (not isinstance(manifest, dict) or type(manifest.get("schema_version")) is not int
                or manifest["schema_version"] != 1
                or manifest.get("kind") != "visiontactile-linux-bootstrap"
                or manifest.get("target") != BOOTSTRAP_TARGET):
            raise DeliveryError("Bootstrap manifest kind, schema or target is unsupported.")
        expected = {
            name: {"path": name, "bytes": len(value[0]), "sha256": hashlib.sha256(value[0]).hexdigest()}
            for name, value in public_files.items()
            if any(PurePosixPath(name).match(pattern) for pattern in BOOTSTRAP_SOURCE_PATTERNS)
        }
        required = {"tools/capture_deploy/install_full.py", "tools/capture_deploy/oneclick/engine.py",
                    "tools/capture_deploy/oneclick/delivery.py", "tools/capture_deploy/oneclick/gui.py",
                    "tools/tracker_deploy/src/vt_tracker_deploy/archive.py"}
        if not required.issubset(expected) or _entries(manifest.get("sources")) != expected:
            raise DeliveryError("Bootstrap sources differ from the exported source snapshot; rebuild the bootstrap.")
        entries = _entries(manifest.get("files"))
        if sum(item["bytes"] for item in entries.values()) > 256 * 1024 * 1024:
            raise DeliveryError("Bootstrap payload exceeds the supported total size.")
        if BOOTSTRAP_NAME not in entries or not any(name.startswith("LICENSES/") for name in entries):
            raise DeliveryError("Bootstrap executable or runtime licenses are missing.")
        payload = {}
        for name, entry in entries.items():
            if name != BOOTSTRAP_NAME and not name.startswith("LICENSES/"):
                raise DeliveryError("Bootstrap contains an unsupported payload path.")
            limit = 128 * 1024 * 1024 if name == BOOTSTRAP_NAME else 4 * 1024 * 1024
            content = _bootstrap_read(directory, name, limit)
            if len(content) != entry["bytes"] or hashlib.sha256(content).hexdigest() != entry["sha256"]:
                raise DeliveryError("Bootstrap file digest or byte count does not match its manifest.")
            payload[name] = content
        observed = set()
        for path in directory.rglob("*"):
            name = path.relative_to(directory).as_posix()
            if path.is_symlink():
                raise DeliveryError("Bootstrap directory must not contain links.")
            if path.is_dir():
                if name != "LICENSES" and not name.startswith("LICENSES/"):
                    raise DeliveryError("Bootstrap contains an unexpected directory.")
            elif path.is_file():
                observed.add(name)
            else:
                raise DeliveryError("Bootstrap directory must contain only regular files.")
        if observed != set(entries) | {"BUILD_MANIFEST.json"}:
            raise DeliveryError("Bootstrap directory and manifest file sets differ.")
        binary = payload[BOOTSTRAP_NAME]
        if (len(binary) < 20 or binary[:6] != b"\x7fELF\x02\x01"
                or struct.unpack("<H", binary[18:20])[0] != 62):
            raise DeliveryError("Bootstrap executable must be an ELF64 little-endian x86_64 binary.")
        return payload, manifest_bytes
    except DeliveryError:
        raise
    except (OSError, ValueError, TypeError) as exc:
        raise DeliveryError("Bootstrap validation failed; check its manifest and regular files.") from exc


def build_delivery(root, tracker_package, output, bootstrap_directory=None):
    """Capture, validate and export without changing Git or user configuration."""
    root = Path(root).expanduser().resolve()
    output = Path(output).expanduser().absolute()
    if os.path.lexists(output):
        raise DeliveryError("Output already exists; choose a new file.")
    output = output.resolve()
    if output.is_relative_to(root) or not output.name.endswith(".tar.gz"):
        raise DeliveryError("Output must be a .tar.gz file outside the source repository.")
    if not output.parent.is_dir():
        raise DeliveryError("Output parent directory must already exist.")

    revision = release.git(root, "rev-parse", "HEAD").decode("ascii").strip()
    dirty = bool(release.git(root, "status", "--porcelain=v1", "--untracked-files=all"))
    files, links, _excluded, errors = release.collect(root)
    if errors or release.verify_snapshot(files, links):
        raise DeliveryError("Public source validation failed; run prepare_release.py check.")
    if INSTALLER not in files or b"--tracker-package" not in files[INSTALLER][0]:
        raise DeliveryError("Source installer does not support bundled Tracker installation.")
    bootstrap = _bootstrap_payload(bootstrap_directory, files) if bootstrap_directory is not None else None
    package_bytes, package_id = _package_bytes(Path(tracker_package).expanduser().absolute())

    # Reuse the public exporter unchanged so its source manifest remains exact.
    with tempfile.TemporaryDirectory(prefix="vt-tracker-delivery-") as directory:
        source_archive = Path(directory) / "source.tar.gz"
        release.export_archive(source_archive, files, links)
        with tarfile.open(source_archive, "r:gz") as source:
            source_manifest = source.extractfile(SOURCE_PREFIX + "PUBLIC_SOURCE_MANIFEST.json").read()
            manifest = {
                "schema_version": 1,
                "distribution": "restricted-device-delivery",
                "source": {
                    "baseline_commit": revision,
                    "working_tree_dirty": dirty,
                    "manifest_path": SOURCE_PREFIX + "PUBLIC_SOURCE_MANIFEST.json",
                    "manifest_sha256": hashlib.sha256(source_manifest).hexdigest(),
                },
                "tracker_package": {
                    "path": "deployment/tracker.vttracker",
                    "bytes": len(package_bytes), "sha256": package_id,
                },
            }
            if bootstrap is not None:
                payload, bootstrap_manifest = bootstrap
                binary = payload[BOOTSTRAP_NAME]
                manifest["bootstrap"] = {
                    "path": BOOTSTRAP_NAME, "bytes": len(binary),
                    "sha256": hashlib.sha256(binary).hexdigest(),
                    "manifest_path": "BOOTSTRAP_MANIFEST.json",
                    "manifest_sha256": hashlib.sha256(bootstrap_manifest).hexdigest(),
                }
            manifest_bytes = (json.dumps(manifest, ensure_ascii=False, indent=2) + "\n").encode()
            descriptor = os.open(output, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            created = os.fstat(descriptor)
            try:
                with os.fdopen(descriptor, "wb") as stream:
                    with tarfile.open(fileobj=stream, mode="w:gz") as archive:
                        for member in source:
                            content = source.extractfile(member)
                            relocated = copy.copy(member)
                            relocated.name = PREFIX + member.name
                            relocated.pax_headers = dict(member.pax_headers)
                            # An inherited PAX path overrides TarInfo.name. Let
                            # tarfile regenerate it for the new long/Unicode path.
                            relocated.pax_headers.pop("path", None)
                            archive.addfile(relocated, content)
                        _add(archive, "deployment/tracker.vttracker", package_bytes, 0o600)
                        _add(archive, "install.sh", FULL_WRAPPER if bootstrap else WRAPPER, 0o755)
                        _add(archive, "README.zh-CN-en.txt", FULL_README if bootstrap else README)
                        if bootstrap is not None:
                            for name, content in sorted(payload.items()):
                                _add(archive, name, content, 0o755 if name == BOOTSTRAP_NAME else 0o644)
                            _add(archive, "BOOTSTRAP_MANIFEST.json", bootstrap_manifest)
                        _add(archive, "DELIVERY_MANIFEST.json", manifest_bytes)
            except BaseException:
                # Remove only our partial archive, never a concurrently replaced file.
                try:
                    current = output.lstat()
                    if (current.st_dev, current.st_ino) == (created.st_dev, created.st_ino):
                        output.unlink()
                except FileNotFoundError:
                    pass
                raise
    digest = hashlib.sha256()
    with output.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return {"status": "PASS", "archive": str(output),
            "bytes": output.stat().st_size, "sha256": digest.hexdigest()}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--tracker-package", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--bootstrap-directory", type=Path,
                        help="Validated build_linux_installer output to include for graphical installation")
    args = parser.parse_args(argv)
    try:
        report = build_delivery(args.root, args.tracker_package, args.output, args.bootstrap_directory)
    except DeliveryError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    except (OSError, ValueError, subprocess.SubprocessError):
        # Library exceptions may contain private local paths or input details.
        print("Delivery build failed; check repository and local file access.", file=sys.stderr)
        return 1
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
