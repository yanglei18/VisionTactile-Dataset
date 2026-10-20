"""Build an offline Windows installer from an isolated, pinned environment.

Run on Windows x64 with Python 3.12 and Inno Setup 6.7.3. The end user's
machine needs neither tool. Restricted model assets are supplied explicitly;
no model, device configuration, credentials or developer venv is auto-copied.
"""
from __future__ import annotations

import argparse
import ast
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import tomllib
import venv


HERE = Path(__file__).resolve().parent
REPOSITORY = HERE.parents[1]
NAME = "VisionTactile3D"
INNO_VERSION = (6, 7, 3)
VENDORED_RESOURCES = {
    "ChineseSimplified.isl": "e0b0b350e2245f3c5e65586dfe43d574f6e7f06f2261149aba284954b3fc9a8d",
    "INNO_SETUP_LICENSE.txt": "2e5346868c2a18434489824e11d65c3031620f792fefc415d05f19cd441abf5c",
}
MAX_MODEL_BYTES = 512 * 1024
LOCAL_PACKAGES = (
    "ros2_ws/src/vt_capture_core", "ros2_ws/src/vt_tactile_capture",
    "tools/fsglove12", "tools/capture_console",
)
# List the data used by the tactile entry point explicitly. Collecting all data
# from the shared packages also ships unrelated workstation/gesture profiles.
APPLICATION_DATA = (
    ("tools/capture_console/src/vt_capture_console/theme/theme.qss", "vt_capture_console/theme"),
    ("tools/capture_console/src/vt_capture_console/theme/icons/*.svg", "vt_capture_console/theme/icons"),
    ("tools/capture_console/src/vt_capture_console/i18n/*.json", "vt_capture_console/i18n"),
    ("ros2_ws/src/vt_tactile_capture/vt_tactile_capture/layouts/right_glove_candidate.json",
     "vt_tactile_capture/layouts"),
)
EXCLUDED_MODULES = (
    "torch", "manotorch", "chumpy", "rclpy", "rosbag2_py", "rospy",
    "PyQt5", "PyQt6", "PySide2", "polyscope", "matplotlib",
    "vt_capture_console.app", "vt_fsglove12.tactile_preview_export",
    # Shared widgets have lazy imports for other preview kinds. The standalone
    # entry point injects its own settings and only constructs kind="tactile".
    "vt_capture_core", "vt_tracker_deploy",
    "vt_capture_console.backend.environment_discovery",
    "vt_capture_console.backend.workstation_settings",
    "vt_capture_console.backend.pose_preview", "vt_capture_console.ui.widgets.glove_3d",
    "vt_capture_console.ui.widgets.camera_gl", "vt_capture_console.ui.gpu_status",
)
FORBIDDEN_RESOURCE_PATTERNS = (
    "vt_capture_core*", "vt_capture_console-*.dist-info", "vt_tactile_capture-*.dist-info",
    "vt_fsglove12-*.dist-info", "vt_fsglove12/configs/*", "vt_tracker_deploy*",
    "vt_tactile_capture/layouts/left*",
)
REQUIRED_RESOURCES = (
    "vt_capture_console/assets/tactile/right_hand.json",
    "vt_capture_console/theme/theme.qss",
    "vt_capture_console/theme/icons/check.svg",
    "vt_capture_console/i18n/catalog_a.json",
    "vt_tactile_capture/layouts/right_glove_candidate.json",
    "PySide6/plugins/platforms/qwindows.dll",
)


def sha256(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def read_version(repository: Path = REPOSITORY) -> str:
    with (repository / "tools/tactile_viewer_windows/product.toml").open("rb") as stream:
        version = tomllib.load(stream).get("version")
    if not isinstance(version, str) or not re.fullmatch(r"[0-9]+\.[0-9]+\.[0-9]+", version):
        raise ValueError("安装包版本必须为 major.minor.patch")
    if any(int(part) > 65535 for part in version.split(".")):
        raise ValueError("安装包版本的各部分必须小于 65536")
    return version


def check_vendor_resources(directory: Path = HERE / "vendor") -> None:
    for name, digest in VENDORED_RESOURCES.items():
        resource = directory / name
        if not resource.is_file() or sha256(resource) != digest:
            raise ValueError(f"安装器资源缺失或被修改：{name}；请恢复仓库内固定版本")


def check_model_input(model: Path, license_path: Path) -> dict:
    """Cheap preflight before downloading packages; full geometry is checked later."""
    if not model.is_file() or not 0 < model.stat().st_size <= MAX_MODEL_BYTES:
        raise ValueError("请提供不超过 512 KiB 的静态右手模型 JSON")
    value = json.loads(model.read_text(encoding="utf-8"))
    if (not isinstance(value, dict) or type(value.get("schema_version")) is not int
            or value["schema_version"] != 1):
        raise ValueError("静态模型版本无效")
    geometry, axes = value.get("neutral_geometry"), value.get("model_axes")
    if (not isinstance(geometry, dict) or not isinstance(axes, dict)
            or not isinstance(geometry.get("provenance"), dict)
            or geometry["provenance"].get("hand") != "right" or axes.get("hand") != "right"):
        raise ValueError("安装包需要右手模型及对应坐标轴")
    if not license_path.is_file() or not 0 < license_path.stat().st_size <= 1024 * 1024:
        raise ValueError("请提供模型许可说明文件（UTF-8 文本，1 MiB 以内）")
    if not license_path.read_text(encoding="utf-8-sig").strip():
        raise ValueError("模型许可说明不能为空")
    return {"model_sha256": sha256(model), "license_sha256": sha256(license_path),
            "model_bytes": model.stat().st_size,
            "notice": "记录许可文本不代表取得再分发授权；发布者须按实际许可分发。"}


def check_platform() -> None:
    if (sys.platform != "win32" or platform.machine().lower() not in ("amd64", "x86_64")
            or struct.calcsize("P") != 8):
        raise ValueError("请在 Windows x64 上构建；PyInstaller 不支持从 Linux 交叉生成 Windows 程序")
    if sys.version_info[:2] != (3, 12):
        raise ValueError("请使用 Windows x64 Python 3.12，命令示例：py -3.12 build.py ...")


def compiler_version(path: Path) -> tuple[int, ...]:
    """Ask ISPP for its documented VER value; official PE versions may be 0.0.0.0."""
    with tempfile.TemporaryDirectory(prefix="tactile-inno-probe-") as temporary:
        script = Path(temporary) / "version.iss"
        script.write_text(
            '#pragma message "VT_INNO_VERSION=" + Str(VER)\n'
            '[Setup]\nAppName=CompilerVersionProbe\nAppVersion=1\n'
            'CreateAppDir=no\nUninstallable=no\nOutput=no\n', encoding="utf-8")
        try:
            result = subprocess.run([str(path), str(script)], cwd=temporary,
                                    env=clean_environment(), stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True, encoding="utf-8",
                                    errors="replace", timeout=30, check=False)
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise ValueError("无法运行 Inno Setup 编译器版本检查") from exc
        matches = re.findall(r"\bVT_INNO_VERSION=(\d+)\b", result.stdout)
        if result.returncode or len(matches) != 1:
            raise ValueError("Inno Setup 编译器版本检查失败：" + result.stdout[-2000:])
        version = int(matches[0])
        if not 0 <= version <= 0xFFFFFFFF:
            raise ValueError("Inno Setup 编译器版本无效")
        return tuple((version >> shift) & 255 for shift in (24, 16, 8, 0))


def find_inno(explicit: Path | None) -> Path:
    candidates = [explicit] if explicit else []
    if not explicit:
        if shutil.which("ISCC.exe"):
            candidates.append(Path(shutil.which("ISCC.exe")))
        for name in ("ProgramFiles(x86)", "ProgramFiles", "LOCALAPPDATA"):
            if os.environ.get(name):
                root = Path(os.environ[name])
                candidates += [root / "Inno Setup 6/ISCC.exe",
                               root / "Programs/Inno Setup 6/ISCC.exe"]
    for candidate in candidates:
        if candidate and candidate.is_file():
            candidate = candidate.resolve()
            if compiler_version(candidate)[:3] != INNO_VERSION:
                if explicit:
                    break
                continue
            return candidate
    raise ValueError("需要 Inno Setup 6.7.3；安装后可使用 --iscc 指定 ISCC.exe")


def clean_environment() -> dict[str, str]:
    env = dict(os.environ)
    for name in ("PYTHONHOME", "PYTHONPATH", "QT_PLUGIN_PATH", "QT_QPA_PLATFORM_PLUGIN_PATH",
                 "VIRTUAL_ENV", "QT_API", "QT_QPA_PLATFORM"):
        env.pop(name, None)
    env.update(PYQTGRAPH_QT_LIB="PySide6", PYTHONUTF8="1", PYTHONNOUSERSITE="1")
    return env


def pip_arguments(python: Path, wheelhouse: Path | None) -> list[str]:
    arguments = [str(python), "-I", "-m", "pip", "--isolated", "--disable-pip-version-check", "install"]
    if wheelhouse is not None:
        arguments += ["--no-index", "--find-links", str(wheelhouse)]
    return arguments


def pyinstaller_arguments(python: Path, workspace: Path, model: Path) -> list[str]:
    arguments = [str(python), "-I", "-m", "PyInstaller", "--onedir", "--windowed",
                 "--name", NAME, "--noupx", "--noconfirm", "--clean",
                 "--contents-directory", "_internal", "--distpath", str(workspace / "dist"),
                 "--workpath", str(workspace / "pyinstaller"),
                 "--specpath", str(workspace / "spec"),
                 "--version-file", str(workspace / "version.txt"),
                 "--runtime-hook", str(HERE / "runtime_hook.py"),
                 "--add-data", f"{model}:vt_capture_console/assets/tactile"]
    for source, destination in APPLICATION_DATA:
        arguments += ["--add-data", f"{REPOSITORY / source}:{destination}"]
    for module in ("serial.tools.list_ports_windows", "OpenGL.platform.win32", "pyqtgraph.opengl"):
        arguments += ["--hidden-import", module]
    for module in EXCLUDED_MODULES:
        arguments += ["--exclude-module", module]
    return arguments + [str(HERE / "entrypoint.py")]


def inno_arguments(compiler: Path, version: str, distribution: Path, release: Path) -> list[str]:
    return [str(compiler), f"/DAppVersion={version}", f"/DSourceDir={distribution}",
            f"/DReleaseDir={release}", str(HERE / "installer.iss")]


def write_version_resource(path: Path, version: str) -> None:
    numbers = tuple(int(part) for part in version.split(".")) + (0,)
    path.write_text(
        "VSVersionInfo(ffi=FixedFileInfo(filevers=" + repr(numbers)
        + ", prodvers=" + repr(numbers)
        + ", mask=0x3f, flags=0, OS=0x40004, fileType=1, subtype=0, date=(0, 0)), "
        "kids=[StringFileInfo([StringTable('080404b0', ["
        "StringStruct('CompanyName', 'Vision Tactile'), "
        "StringStruct('FileDescription', '触觉三维预览'), "
        f"StringStruct('FileVersion', {version!r}), "
        "StringStruct('OriginalFilename', 'VisionTactile3D.exe'), "
        "StringStruct('ProductName', '触觉三维预览'), "
        f"StringStruct('ProductVersion', {version!r})])]), "
        "VarFileInfo([VarStruct('Translation', [2052, 1200])])])\n", encoding="utf-8")


def audit_distribution(directory: Path, analysis_toc: Path, expected_model_hash: str) -> None:
    for relative in REQUIRED_RESOURCES:
        if not (directory / "_internal" / relative).is_file():
            raise ValueError(f"安装包缺少运行资源：{relative}")
    if sha256(directory / "_internal" / REQUIRED_RESOURCES[0]) != expected_model_hash:
        raise ValueError("安装包内的模型与输入不一致")
    if not (directory / f"{NAME}.exe").is_file():
        raise ValueError("PyInstaller 没有生成程序")
    for pattern in FORBIDDEN_RESOURCE_PATTERNS:
        if any((directory / "_internal").glob(pattern)):
            raise ValueError(f"安装包意外包含其他工具资源：{pattern}")
    # Analysis TOC records modules inside PYZ, not just files visible in onedir.
    # Excludes are configuration, this is an independent result check.
    value = ast.literal_eval(analysis_toc.read_text(encoding="utf-8"))

    def inspect(node):
        if isinstance(node, (tuple, list)):
            if len(node) == 3 and node[2] in ("PYMODULE", "EXTENSION"):
                if any(node[0] == name or node[0].startswith(name + ".")
                       for name in EXCLUDED_MODULES):
                    raise ValueError(f"安装包意外包含已排除模块：{node[0]}")
            for child in node:
                inspect(child)

    inspect(value)


def file_inventory(directory: Path) -> list[dict]:
    return [{"path": path.relative_to(directory).as_posix(), "bytes": path.stat().st_size,
             "sha256": sha256(path)} for path in sorted(directory.rglob("*")) if path.is_file()]


def read_check_report(path: Path, *, frozen: bool) -> dict:
    if not path.is_file():
        raise RuntimeError("程序没有生成自检报告，停止生成安装包")
    report = json.loads(path.read_text(encoding="utf-8"))
    if (not isinstance(report, dict) or report.get("schema_version") != 1
            or report.get("status") != "ok" or report.get("issues") != []
            or not isinstance(report.get("details"), dict)
            or report["details"].get("frozen") is not frozen):
        raise RuntimeError("程序自检报告未通过，停止生成安装包")
    return report


class Runner:
    def __init__(self, directory: Path):
        self.directory = directory
        self.commands = []

    def run(self, label: str, arguments: list[str], *, timeout: int = 1200) -> None:
        print(label, flush=True)
        number = len(self.commands) + 1
        self.commands.append(arguments)
        (self.directory / "commands.json").write_text(
            json.dumps(self.commands, ensure_ascii=False, indent=2), encoding="utf-8")
        logfile = self.directory / f"step-{number:02d}.log"
        try:
            with logfile.open("w", encoding="utf-8") as stream:
                result = subprocess.run(arguments, cwd=REPOSITORY, env=clean_environment(),
                                        stdout=stream, stderr=subprocess.STDOUT, timeout=timeout,
                                        check=False)
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError(f"步骤超时：{label}；日志：{logfile}") from exc
        if result.returncode:
            raise RuntimeError(f"步骤失败（{result.returncode}）：{label}；日志：{logfile}")


def build(args) -> Path:
    version = read_version()
    model, license_path = args.model_bundle.resolve(), args.model_license.resolve()
    model_record = check_model_input(model, license_path)
    if args.check_inputs:
        from vt_capture_console.ui.widgets.tactile_model_loader import load_bundle_file
        load_bundle_file(model, "right")
        print(json.dumps({"version": version, **model_record}, ensure_ascii=False, indent=2))
        return model
    check_platform()
    check_vendor_resources()
    compiler = find_inno(args.iscc)
    if args.wheelhouse is not None and not args.wheelhouse.is_dir():
        raise ValueError("wheelhouse 目录不存在")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    workspace = Path(tempfile.mkdtemp(prefix="build-", dir=output))
    runner = Runner(workspace)
    print(f"构建日志与中间产物：{workspace}", flush=True)
    stage = workspace / "stage"
    stage.mkdir()
    staged_model = stage / "right_hand.json"
    shutil.copyfile(model, staged_model)
    write_version_resource(workspace / "version.txt", version)
    venv.EnvBuilder(with_pip=True).create(workspace / "venv")
    python = workspace / "venv/Scripts/python.exe"
    pip = pip_arguments(python, args.wheelhouse.resolve() if args.wheelhouse else None)
    runner.run("安装固定版本构建依赖…", pip + ["--only-binary=:all:", "-r", str(HERE / "requirements-build.lock")])
    runner.run("安装仓库内应用包…", pip + ["--no-deps", "--no-build-isolation"]
               + [str(REPOSITORY / package) for package in LOCAL_PACKAGES])
    runner.run("验证依赖一致性…", [str(python), "-I", "-m", "pip", "check"])
    runner.run("验证模型几何和触觉映射…", [str(python), "-I", "-m", "vt_capture_console.tactile_app",
               "--model-bundle", str(staged_model), "--self-check", str(workspace / "source-self-check.json")],
               timeout=60)
    read_check_report(workspace / "source-self-check.json", frozen=False)
    runner.run("生成独立 Windows 程序…", pyinstaller_arguments(python, workspace, staged_model))
    distribution = workspace / "dist" / NAME
    audit_distribution(distribution, workspace / "pyinstaller" / NAME / "Analysis-00.toc",
                       model_record["model_sha256"])
    runner.run("验证冻结程序的模型、界面依赖和资源…", [str(distribution / f"{NAME}.exe"),
               "--self-check", str(workspace / "frozen-self-check.json")], timeout=60)
    frozen_report = read_check_report(workspace / "frozen-self-check.json", frozen=True)
    notices = distribution / "licenses"
    runner.run("收集依赖版本与许可文件…", [str(python), "-I", str(HERE / "collect_notices.py"),
               str(notices), str(distribution / "dependencies.json")])
    shutil.copyfile(license_path, notices / "MODEL_LICENSE.txt")
    shutil.copyfile(REPOSITORY / "LICENSE", notices / "APPLICATION_LICENSE.txt")
    shutil.copyfile(HERE / "vendor/INNO_SETUP_LICENSE.txt", notices / "INNO_SETUP_LICENSE.txt")
    shutil.copyfile(HERE / "USER_GUIDE.txt", distribution / "使用说明.txt")
    manifest = {"schema_version": 1, "product": NAME, "version": version,
                "built_at_utc": datetime.now(timezone.utc).isoformat(),
                "python": platform.python_version(), "platform": "windows-x64",
                "inno_setup": ".".join(map(str, INNO_VERSION)),
                "installer_resources": VENDORED_RESOURCES,
                "requirements_sha256": sha256(HERE / "requirements-build.lock"),
                "model": model_record, "files": file_inventory(distribution)}
    (distribution / "build-manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    release = output / "release"
    release.mkdir(exist_ok=True)
    runner.run("生成双语离线安装包…", inno_arguments(compiler, version, distribution, release))
    installer = release / f"{NAME}-{version}-windows-x64-setup.exe"
    if not installer.is_file():
        raise RuntimeError("Inno Setup 没有生成预期安装包")
    checksum = sha256(installer)
    installer.with_suffix(".exe.sha256").write_text(f"{checksum}  {installer.name}\n", encoding="ascii")
    release_manifest = {**manifest, "installer": {"name": installer.name, "sha256": checksum},
                        "frozen_self_check": frozen_report}
    installer.with_suffix(".manifest.json").write_text(
        json.dumps(release_manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"安装包已生成：{installer}", flush=True)
    return installer


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description="构建触觉三维预览 Windows x64 离线安装包")
    result.add_argument("--model-bundle", type=Path, required=True, help="已取得相应使用/分发许可的右手静态模型 JSON")
    result.add_argument("--model-license", type=Path, required=True, help="随安装包保存的模型许可说明 UTF-8 文本")
    result.add_argument("--output", type=Path, default=REPOSITORY / "dist/tactile-viewer-windows")
    result.add_argument("--iscc", type=Path, help="Inno Setup 6.7.3 的 ISCC.exe；默认自动寻找")
    result.add_argument("--wheelhouse", type=Path, help="仅从此目录安装固定依赖，构建时不访问包索引")
    result.add_argument("--check-inputs", action="store_true", help="仅验证模型输入；可在已安装应用依赖的 Linux 环境执行")
    return result


def main(argv=None) -> int:
    try:
        build(parser().parse_args(argv))
    except (OSError, ValueError, RuntimeError, ImportError) as exc:
        print(f"构建失败：{exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
