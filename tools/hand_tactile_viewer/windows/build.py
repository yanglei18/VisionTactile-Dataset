"""Build the independent Windows hand-motion and touch installer.

Requires Windows x64 Python 3.12 and Inno Setup 6.7.3. Runtime resources are
prepared separately and checked before installation, freezing, and release.
The existing tactile product supplies only shared build utilities, unchanged.
"""
from __future__ import annotations

import argparse
import ast
from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path, PurePosixPath
import platform
import re
import shutil
import sys
import tempfile
import tomllib
import venv


HERE = Path(__file__).resolve().parent
REPOSITORY = HERE.parents[2]
SHARED = REPOSITORY / "tools/tactile_viewer_windows"
NAME = "VisionHandMotionTouch"
RESOURCE_DESTINATION = "vt_hand_tactile_viewer/assets/runtime"

_spec = importlib.util.spec_from_file_location("_vt_windows_build_helpers", SHARED / "build.py")
_shared = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_shared)
Runner = _shared.Runner
sha256 = _shared.sha256

LOCAL_PACKAGES = (
    "ros2_ws/src/vt_capture_core", "ros2_ws/src/vt_tactile_capture",
    "tools/fsglove12", "tools/capture_console", "tools/hand_tactile_viewer",
)
APPLICATION_DATA = (
    ("tools/fsglove12/src/vt_fsglove12/ui_theme/*.qss", "vt_fsglove12/ui_theme"),
    ("tools/fsglove12/src/vt_fsglove12/ui_theme/icons/*.svg", "vt_fsglove12/ui_theme/icons"),
    ("tools/capture_console/src/vt_capture_console/i18n/*.json", "vt_capture_console/i18n"),
    ("ros2_ws/src/vt_tactile_capture/vt_tactile_capture/layouts/right_glove_candidate.json",
     "vt_tactile_capture/layouts"),
)
HIDDEN_IMPORTS = (
    "serial.tools.list_ports_windows", "OpenGL.platform.win32", "pyqtgraph.opengl",
    "torch", "manotorch.manolayer", "manotorch.axislayer", "chumpy", "cv2",
    "pydantic", "sklearn.gaussian_process", "sklearn.gaussian_process.kernels",
    "scipy.optimize", "tqdm",
)
EXCLUDED_MODULES = (
    "PyQt5", "PyQt6", "PySide2", "rclpy", "rosbag2_py", "rospy",
    # The application uses Qt Widgets/OpenGL, never QMT's optional browser UI.
    "PySide6.QtWebEngineCore", "PySide6.QtWebEngineQuick", "PySide6.QtWebEngineWidgets",
    "PySide6.QtWebChannel",
    "polyscope", "open3d", "vt_tracker_deploy", "vt_capture_core",
    "vt_capture_console.app", "vt_capture_console.tactile_app",
    "vt_capture_console.backend.environment_discovery",
    "vt_capture_console.backend.workstation_settings",
    "vt_capture_console.backend.pose_preview", "vt_capture_console.ui.widgets.glove_3d",
    "vt_capture_console.ui.widgets.camera_gl", "vt_capture_console.ui.gpu_status",
)
FORBIDDEN_RESOURCE_PATTERNS = (
    "torch/**/*.lib", "torch/**/*.h", "torch/**/*.hpp", "torch/**/*.cuh",
    "PySide6/*WebEngine*", "PySide6/resources/qtwebengine*",
)
REQUIRED_RESOURCE_FILES = {
    "hand.json", "rest.json", "config.json", "default_response.json",
    "mano/models/MANO_RIGHT.pkl", "MODEL_LICENSE.txt",
    "upstream/src/hand_visualiser/modules/hand/calibration.py",
    "upstream/src/hand_visualiser/modules/datamodels.py",
    "upstream/src/hand_visualiser/modules/enumerates.py",
    "upstream/src/hand_visualiser/modules/utils.py",
}


def read_version(project: Path = HERE.parent) -> str:
    with (project / "pyproject.toml").open("rb") as stream:
        version = tomllib.load(stream).get("project", {}).get("version")
    if (not isinstance(version, str) or not re.fullmatch(r"[0-9]+\.[0-9]+\.[0-9]+", version)
            or any(int(part) > 65535 for part in version.split("."))):
        raise ValueError("安装包版本必须为 major.minor.patch，且各部分小于 65536")
    return version


def check_resources(root: Path) -> dict:
    """Check declared bytes; the real spawned application validates semantics."""
    root = root.resolve(strict=True)
    manifest_path = root / "manifest.json"
    if not 0 < manifest_path.stat().st_size <= 2 * 1024 * 1024:
        raise ValueError("资源清单为空或超过 2 MiB")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if (not isinstance(manifest, dict) or type(manifest.get("schema_version")) is not int
            or manifest["schema_version"] != 1 or manifest.get("product") != "hand-motion-touch"
            or not isinstance(manifest.get("files"), dict)
            or not REQUIRED_RESOURCE_FILES <= manifest["files"].keys()):
        raise ValueError("手势与触觉资源清单不完整")
    names = set()
    for name, digest in manifest["files"].items():
        relative = PurePosixPath(name)
        if (not name or relative.is_absolute() or relative.as_posix() != name
                or any(part in (".", "..") or part.endswith((".", " ")) for part in relative.parts)
                or "\\" in name or ":" in name or name.casefold() in names
                or name.casefold() == "manifest.json"
                or not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest)):
            raise ValueError(f"资源路径或校验值无效：{name}")
        names.add(name.casefold())
        path = (root / relative).resolve(strict=True)
        path.relative_to(root)
        if not path.is_file() or sha256(path) != digest:
            raise ValueError(f"资源校验失败：{name}")
    if not (root / "MODEL_LICENSE.txt").read_text(encoding="utf-8-sig").strip():
        raise ValueError("模型许可说明不能为空")
    return manifest


def stage_resources(source: Path, destination: Path, manifest: dict) -> None:
    """Copy only declared inputs, never arbitrary adjacent developer files."""
    destination.mkdir(parents=True)
    for name in (*manifest["files"], "manifest.json"):
        target = destination / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source / name, target)
    if check_resources(destination) != manifest:
        raise ValueError("准备过程中资源清单发生变化")


def pyinstaller_arguments(python: Path, workspace: Path, resources: Path) -> list[str]:
    arguments = [str(python), "-I", "-m", "PyInstaller", "--onedir", "--windowed",
                 "--name", NAME, "--noupx", "--noconfirm", "--clean",
                 "--contents-directory", "_internal", "--distpath", str(workspace / "dist"),
                 "--workpath", str(workspace / "pyinstaller"), "--specpath", str(workspace / "spec"),
                 "--version-file", str(workspace / "version.txt"),
                 "--runtime-hook", str(SHARED / "runtime_hook.py"),
                 "--add-data", f"{resources}:{RESOURCE_DESTINATION}"]
    for source, destination in APPLICATION_DATA:
        arguments += ["--add-data", f"{REPOSITORY / source}:{destination}"]
    # MANO/chumpy and QMT read their own source files at runtime. Torch uses
    # its official hook: it preserves sources/DLLs but excludes developer .lib
    # archives and headers. collect-all torch would bypass those exclusions.
    for module in ("manotorch", "chumpy", "qmt"):
        arguments += ["--collect-all", module]
    for module in ("manotorch", "torch"):
        arguments += ["--copy-metadata", module]
    for module in HIDDEN_IMPORTS:
        arguments += ["--hidden-import", module]
    for module in EXCLUDED_MODULES:
        arguments += ["--exclude-module", module]
    return arguments + [str(HERE / "entrypoint.py")]


def write_version_resource(path: Path, version: str) -> None:
    numbers = tuple(int(part) for part in version.split(".")) + (0,)
    strings = {"CompanyName": "Vision Tactile", "FileDescription": "Hand Motion and Touch Viewer",
               "FileVersion": version, "OriginalFilename": f"{NAME}.exe",
               "ProductName": "Hand Motion and Touch Viewer", "ProductVersion": version}
    children = ", ".join(f"StringStruct({key!r}, {value!r})" for key, value in strings.items())
    path.write_text(
        f"VSVersionInfo(ffi=FixedFileInfo(filevers={numbers!r}, prodvers={numbers!r}, "
        "mask=0x3f, flags=0, OS=0x40004, fileType=1, subtype=0, date=(0, 0)), "
        f"kids=[StringFileInfo([StringTable('040904b0', [{children}])]), "
        "VarFileInfo([VarStruct('Translation', [1033, 1200])])])\n", encoding="utf-8")


def check_report(path: Path, *, frozen: bool, version: str) -> dict:
    report = _shared.read_check_report(path, frozen=frozen)
    details = report["details"]
    if (details.get("version") != version or details.get("qt_binding") != "PySide6"
            or details.get("serial_device") != "not_opened"
            or details.get("solver_process") != "initialized_without_device"):
        raise RuntimeError("程序未通过独立进程、界面依赖与无设备自检")
    return report


def audit_distribution(distribution: Path, analysis_toc: Path, manifest: dict) -> None:
    internal = distribution / "_internal"
    for path in (distribution / f"{NAME}.exe", internal / "PySide6/plugins/platforms/qwindows.dll",
                 internal / "qmt/functions/quaternion.py", internal / "qmt/functions/heading_correction.py",
                 internal / "chumpy/np_tensordot.py"):
        if not path.is_file():
            raise ValueError(f"缺少程序、平台插件或功能标定所需源码：{path}")
    if check_resources(internal / RESOURCE_DESTINATION) != manifest:
        raise ValueError("冻结资源与输入不一致")
    for pattern in FORBIDDEN_RESOURCE_PATTERNS:
        if any(internal.glob(pattern)):
            raise ValueError(f"安装包包含未使用的开发或网页资源：{pattern}")

    def inspect(node):
        if not isinstance(node, (tuple, list)):
            return
        if len(node) == 3 and node[2] in ("PYMODULE", "EXTENSION"):
            if any(node[0] == name or node[0].startswith(name + ".") for name in EXCLUDED_MODULES):
                raise ValueError(f"安装包包含已排除模块：{node[0]}")
        for child in node:
            inspect(child)

    inspect(ast.literal_eval(analysis_toc.read_text(encoding="utf-8")))


def build(args) -> Path:
    version = read_version()
    source = args.resources.expanduser().resolve(strict=True)
    manifest = check_resources(source)
    _shared.check_platform()
    _shared.check_vendor_resources()
    compiler = _shared.find_inno(args.iscc)
    wheelhouse = args.wheelhouse.resolve(strict=True) if args.wheelhouse else None
    if wheelhouse is not None and not wheelhouse.is_dir():
        raise ValueError("wheelhouse 必须是目录")
    lock = HERE / "requirements-build.lock"
    if not lock.is_file():
        raise ValueError("缺少 requirements-build.lock，不能构建发布版本")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    workspace = Path(tempfile.mkdtemp(prefix="build-", dir=output))
    runner = Runner(workspace)
    print(f"构建日志与中间产物：{workspace}", flush=True)
    staged = workspace / "runtime"
    stage_resources(source, staged, manifest)
    venv.EnvBuilder(with_pip=True).create(workspace / "venv")
    python = workspace / "venv/Scripts/python.exe"
    pip = _shared.pip_arguments(python, wheelhouse)
    runner.run("安装固定版本构建依赖…", pip + ["--only-binary=:all:", "--require-hashes", "-r", str(lock)])
    runner.run("安装应用包…", pip + ["--no-deps", "--no-build-isolation"]
               + [str(REPOSITORY / package) for package in LOCAL_PACKAGES])
    return finish_build(workspace, runner, version=version, manifest=manifest,
                        output=output, compiler=compiler, lock=lock)


def finish_build(workspace: Path, runner: Runner, *, version: str, manifest: dict,
                 output: Path, compiler: Path, lock: Path) -> Path:
    """Freeze and verify a prepared environment, then produce the installer.

    Maintainers may retry this phase with an existing dependency-checked
    workspace. Every call rechecks staged input bytes, source initialization,
    the freezer output and the actual spawned frozen compute process.
    """
    python = workspace / "venv/Scripts/python.exe"
    staged = workspace / "runtime"
    if check_resources(staged) != manifest:
        raise ValueError("构建输入与已准备资源不一致")
    runner.run("验证依赖一致性…", [str(python), "-I", "-m", "pip", "check"])
    write_version_resource(workspace / "version.txt", version)
    source_report = workspace / "source-self-check.json"
    source_report.unlink(missing_ok=True)
    runner.run("验证源码资源及计算进程…", [str(python), "-I", "-m", "vt_hand_tactile_viewer",
               "--resources", str(staged), "--self-check", str(source_report)], timeout=180)
    check_report(source_report, frozen=False, version=version)
    runner.run("生成独立 Windows 程序…", pyinstaller_arguments(python, workspace, staged), timeout=2400)
    distribution = workspace / "dist" / NAME
    audit_distribution(distribution, workspace / "pyinstaller" / NAME / "Analysis-00.toc", manifest)
    frozen_report = workspace / "frozen-self-check.json"
    frozen_report.unlink(missing_ok=True)
    runner.run("验证冻结程序及计算进程…", [str(distribution / f"{NAME}.exe"),
               "--self-check", str(frozen_report)], timeout=180)
    verified = check_report(frozen_report, frozen=True, version=version)
    notices = distribution / "licenses"
    runner.run("收集依赖许可…", [str(python), "-I", str(SHARED / "collect_notices.py"),
               str(notices), str(distribution / "dependencies.json")])
    for source_path, filename in ((REPOSITORY / "LICENSE", "APPLICATION_LICENSE.txt"),
                                  (staged / "MODEL_LICENSE.txt", "MODEL_LICENSE.txt"),
                                  (SHARED / "vendor/INNO_SETUP_LICENSE.txt", "INNO_SETUP_LICENSE.txt")):
        shutil.copyfile(source_path, notices / filename)
    shutil.copyfile(HERE / "USER_GUIDE.txt", distribution / "使用说明.txt")
    readme = HERE.parent / "README.md"
    if readme.is_file():
        shutil.copyfile(readme, distribution / "README.md")
    record = {"schema_version": 1, "product": NAME, "version": version,
              "built_at_utc": datetime.now(timezone.utc).isoformat(),
              "python": platform.python_version(), "platform": "windows-x64",
              "inno_setup": ".".join(map(str, _shared.INNO_VERSION)),
              "requirements_sha256": sha256(lock), "runtime_resources": manifest,
              "files": _shared.file_inventory(distribution)}
    (distribution / "build-manifest.json").write_text(json.dumps(record, indent=2), encoding="utf-8")
    release = output / "release"
    release.mkdir(exist_ok=True)
    runner.run("生成双语离线安装包…", [str(compiler), f"/DAppVersion={version}",
               f"/DSourceDir={distribution}", f"/DReleaseDir={release}", str(HERE / "installer.iss")], timeout=2400)
    installer = release / f"{NAME}-{version}-windows-x64-setup.exe"
    if not installer.is_file():
        raise RuntimeError("Inno Setup 未生成预期安装包")
    checksum = sha256(installer)
    installer.with_suffix(".exe.sha256").write_text(f"{checksum}  {installer.name}\n", encoding="ascii")
    installer.with_suffix(".manifest.json").write_text(json.dumps({
        **record, "installer": {"name": installer.name, "sha256": checksum},
        "frozen_self_check": verified}, indent=2), encoding="utf-8")
    print(f"安装包已生成：{installer}", flush=True)
    return installer


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description="构建手势与触觉 Windows x64 离线安装包")
    result.add_argument("--resources", type=Path, required=True, help="prepare 生成的完整运行资源目录")
    result.add_argument("--wheelhouse", type=Path, help="仅使用该目录的固定依赖，不访问包索引")
    result.add_argument("--output", type=Path, default=REPOSITORY / "dist/hand-tactile-viewer-windows")
    result.add_argument("--iscc", type=Path, help="Inno Setup 6.7.3 的 ISCC.exe")
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
