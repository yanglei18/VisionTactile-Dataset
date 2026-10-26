"""Build contracts without compiling, downloading, or touching a serial device."""
from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest


SPEC = importlib.util.spec_from_file_location("tactile_windows_build", Path(__file__).parents[1] / "build.py")
build = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(build)


def test_product_version_is_independent_of_capture_application(tmp_path):
    product = tmp_path / "tools/tactile_viewer_windows/product.toml"
    product.parent.mkdir(parents=True)
    product.write_text('version = "1.2.3"\n', encoding="utf-8")
    assert build.read_version(tmp_path) == "1.2.3"


@pytest.mark.parametrize("value", ["", 'version = true', 'version = 1',
                                    'version = "1.2"', 'version = "1.2.3-beta"',
                                    'version = "１.２.３"', 'version = "1.2.65536"'])
def test_invalid_product_version_fails_before_building(tmp_path, value):
    product = tmp_path / "tools/tactile_viewer_windows/product.toml"
    product.parent.mkdir(parents=True)
    product.write_text(value, encoding="utf-8")
    with pytest.raises(ValueError, match="安装包版本"):
        build.read_version(tmp_path)


@pytest.fixture
def inputs(tmp_path):
    model = tmp_path / "模型 资源.json"
    model.write_text(json.dumps({"schema_version": 1, "neutral_geometry": {"provenance": {"hand": "right"}},
                                 "model_axes": {"hand": "right"}}), encoding="utf-8")
    license_path = tmp_path / "license.txt"
    license_path.write_text("Test fixture only. Not a distributable hand model.\n", encoding="utf-8")
    return model, license_path


def test_inputs_record_exact_bytes_without_claiming_permission(inputs):
    model, license_path = inputs
    result = build.check_model_input(model, license_path)
    assert result["model_sha256"] == build.sha256(model)
    assert result["license_sha256"] == build.sha256(license_path)
    assert result["model_bytes"] == model.stat().st_size
    assert "不代表" in result["notice"]


@pytest.mark.parametrize("fault", ["missing", "oversized", "array", "bool-version", "left", "bad-axes", "empty-license"])
def test_reject_inputs_before_installing_dependencies(inputs, fault):
    model, license_path = inputs
    data = json.loads(model.read_text())
    if fault == "missing":
        model.unlink()
    elif fault == "oversized":
        model.write_bytes(b" " * (build.MAX_MODEL_BYTES + 1))
    elif fault == "empty-license":
        license_path.write_text("   ")
    else:
        if fault == "array":
            data = []
        elif fault == "bool-version":
            data["schema_version"] = True
        elif fault == "left":
            data["neutral_geometry"]["provenance"]["hand"] = "left"
        elif fault == "bad-axes":
            data["model_axes"] = "right"
        model.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        build.check_model_input(model, license_path)


@pytest.mark.parametrize("target,bits,version,machine", [
    ("linux", 8, (3, 12), "x86_64"), ("win32", 4, (3, 12), "AMD64"),
    ("win32", 8, (3, 11), "AMD64"), ("win32", 8, (3, 12), "ARM64")])
def test_reject_unsupported_build_hosts(monkeypatch, target, bits, version, machine):
    monkeypatch.setattr(build.sys, "platform", target)
    monkeypatch.setattr(build.sys, "version_info", version)
    monkeypatch.setattr(build.struct, "calcsize", lambda _: bits)
    monkeypatch.setattr(build.platform, "machine", lambda: machine)
    with pytest.raises(ValueError):
        build.check_platform()


def test_pinned_compiler_does_not_require_manual_language_installation(tmp_path, monkeypatch):
    compiler = tmp_path / "ISCC.exe"
    compiler.touch()
    monkeypatch.setattr(build, "compiler_version", lambda _: (6, 7, 2, 0))
    with pytest.raises(ValueError, match="6.7.3"):
        build.find_inno(compiler)
    monkeypatch.setattr(build, "compiler_version", lambda _: (6, 7, 3, 0))
    assert build.find_inno(compiler) == compiler


def test_vendored_language_and_license_are_pinned(tmp_path):
    build.check_vendor_resources()
    with pytest.raises(ValueError, match="资源缺失"):
        build.check_vendor_resources(tmp_path)
    for name in build.VENDORED_RESOURCES:
        (tmp_path / name).write_bytes((build.HERE / "vendor" / name).read_bytes())
    (tmp_path / "ChineseSimplified.isl").write_bytes(b"unverified translation")
    with pytest.raises(ValueError, match="被修改"):
        build.check_vendor_resources(tmp_path)


@pytest.mark.parametrize("output,code,expected", [
    ("Compiler engine version: Inno Setup 6.7.3\n   Line 1: VT_INNO_VERSION=101122816\n", 0, (6, 7, 3, 0)),
    ("VT_INNO_VERSION=101122560\n", 0, (6, 7, 2, 0)),
    ("Compiler engine version: Inno Setup 6\n", 0, None),
    ("VT_INNO_VERSION=101122816\nCompilation aborted", 1, None),
    ("VT_INNO_VERSION=101122816\nVT_INNO_VERSION=101122560\n", 0, None),
])
def test_compiler_version_uses_documented_preprocessor_value(tmp_path, monkeypatch, output, code, expected):
    def run(command, **kwargs):
        script = Path(command[-1])
        assert "Str(VER)" in script.read_text()
        assert "Output=no" in script.read_text()
        assert kwargs["timeout"] == 30
        return SimpleNamespace(returncode=code, stdout=output)

    monkeypatch.setattr(build.subprocess, "run", run)
    if expected is None:
        with pytest.raises(ValueError, match="版本检查失败"):
            build.compiler_version(tmp_path / "ISCC.exe")
    else:
        assert build.compiler_version(tmp_path / "ISCC.exe") == expected


def test_windows_paths_are_passed_as_single_arguments(tmp_path):
    python = tmp_path / "Python 环境/python.exe"
    workspace = tmp_path / "构建 & 目录"
    model = workspace / "right_hand.json"
    arguments = build.pyinstaller_arguments(python, workspace, model)
    assert arguments[0] == str(python)
    assert arguments[arguments.index("--add-data") + 1] == f"{model}:vt_capture_console/assets/tactile"
    assert arguments[arguments.index("--contents-directory") + 1] == "_internal"
    assert "--onedir" in arguments and "--windowed" in arguments
    assert "--collect-all" not in arguments
    assert "--collect-data" not in arguments
    assert "--copy-metadata" not in arguments
    for module in ("torch", "manotorch", "rclpy", "PyQt5", "vt_capture_core",
                   "vt_tracker_deploy", "vt_capture_console.backend.environment_discovery",
                   "vt_capture_console.backend.workstation_settings",
                   "vt_capture_console.ui.widgets.glove_3d", "vt_capture_console.ui.widgets.camera_gl"):
        assert arguments[arguments.index(module) - 1] == "--exclude-module"
    compiler = tmp_path / "Inno Setup/ISCC.exe"
    command = build.inno_arguments(compiler, "0.5.0", workspace, tmp_path / "发布")
    assert command[2] == f"/DSourceDir={workspace}"


def test_offline_install_has_no_package_index(tmp_path):
    arguments = build.pip_arguments(tmp_path / "python.exe", tmp_path / "wheel house")
    assert "--isolated" in arguments
    assert "--no-index" in arguments
    assert arguments[-1] == str(tmp_path / "wheel house")


def test_frozen_theme_resources_follow_the_same_owner_as_runtime():
    import re

    theme_path = "tools/fsglove12/src/vt_fsglove12/ui_theme/theme.qss"
    assert (theme_path, "vt_fsglove12/ui_theme") in build.APPLICATION_DATA
    assert not any("vt_capture_console/theme" in source for source, _ in build.APPLICATION_DATA)
    theme = (build.REPOSITORY / theme_path).read_text(encoding="utf-8")
    for icon in set(re.findall(r'@THEME_DIR@/([^\"\s]+)', theme)):
        assert (build.REPOSITORY / Path(theme_path).parent / icon).is_file()


def test_build_env_cannot_inherit_external_python_or_qt(monkeypatch):
    for key in ("PYTHONHOME", "PYTHONPATH", "QT_PLUGIN_PATH", "QT_QPA_PLATFORM_PLUGIN_PATH",
                "VIRTUAL_ENV", "QT_API", "QT_QPA_PLATFORM"):
        monkeypatch.setenv(key, "external")
    monkeypatch.setenv("PYQTGRAPH_QT_LIB", "PyQt5")
    environment = build.clean_environment()
    assert environment["PYQTGRAPH_QT_LIB"] == "PySide6"
    assert "PYTHONPATH" not in environment
    assert "QT_PLUGIN_PATH" not in environment
    assert "QT_QPA_PLATFORM" not in environment


@pytest.fixture
def distribution(tmp_path):
    root = tmp_path / "app"
    root.mkdir()
    (root / "VisionTactile3D.exe").write_bytes(b"fixture")
    for name in build.REQUIRED_RESOURCES:
        path = root / "_internal" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("fixture")
    toc = tmp_path / "Analysis-00.toc"
    toc.write_text(repr(([('numpy', 'numpy/__init__.py', 'PYMODULE')],)))
    digest = build.sha256(root / "_internal" / build.REQUIRED_RESOURCES[0])
    return root, toc, digest


def test_audit_requires_resources_model_integrity_and_executable(distribution):
    root, toc, digest = distribution
    build.audit_distribution(root, toc, digest)
    with pytest.raises(ValueError, match="模型与输入"):
        build.audit_distribution(root, toc, "0" * 64)
    (root / "_internal" / build.REQUIRED_RESOURCES[-1]).unlink()
    with pytest.raises(ValueError, match="缺少运行资源"):
        build.audit_distribution(root, toc, digest)


@pytest.mark.parametrize("forbidden", [
    "torch", "torch.cuda", "rclpy", "manotorch.manolayer", "PyQt5.QtCore",
    "vt_capture_core.workstation", "vt_capture_console.backend.workstation_settings",
    "vt_tracker_deploy.manifest", "vt_capture_console.backend.environment_discovery",
    "vt_capture_console.backend.pose_preview", "vt_capture_console.ui.widgets.glove_3d",
    "vt_capture_console.ui.widgets.camera_gl", "vt_capture_console.ui.gpu_status",
])
def test_audit_checks_modules_inside_archive(distribution, forbidden):
    root, toc, digest = distribution
    toc.write_text(repr(([('numpy', 'numpy/__init__.py', 'PYMODULE'),
                          (forbidden, 'hidden-module.py', 'PYMODULE')],)))
    with pytest.raises(ValueError, match="已排除模块"):
        build.audit_distribution(root, toc, digest)


@pytest.mark.parametrize("forbidden", [
    "vt_fsglove12/configs/right_current.json",
    "vt_tactile_capture/layouts/left_glove_unverified_candidate.json",
    "vt_capture_core-0.1.0.dist-info/METADATA",
    "vt_capture_console-0.5.0.dist-info/entry_points.txt",
    "vt_fsglove12-0.1.0.dist-info/direct_url.json",
    "vt_tracker_deploy-0.1.0.dist-info/entry_points.txt",
])
def test_audit_rejects_unrelated_profiles_and_tool_metadata(distribution, forbidden):
    root, toc, digest = distribution
    path = root / "_internal" / forbidden
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("unrelated tool data")
    with pytest.raises(ValueError, match="其他工具资源"):
        build.audit_distribution(root, toc, digest)


def test_audit_never_executes_toc(distribution):
    root, toc, digest = distribution
    toc.write_text("__import__('os').remove('important')")
    with pytest.raises(ValueError):
        build.audit_distribution(root, toc, digest)


@pytest.mark.parametrize("fault", ["missing", "not-frozen", "failed", "issues"])
def test_self_check_is_fail_closed(tmp_path, fault):
    report = tmp_path / "check.json"
    value = {"schema_version": 1, "status": "ok", "issues": [], "details": {"frozen": True}}
    if fault == "not-frozen":
        value["details"]["frozen"] = False
    elif fault == "failed":
        value["status"] = "error"
    elif fault == "issues":
        value["issues"] = [{"check": "model", "message": "missing"}]
    if fault != "missing":
        report.write_text(json.dumps(value))
    with pytest.raises(RuntimeError):
        build.read_check_report(report, frozen=True)


def test_runner_failure_retains_log_and_stops(tmp_path, monkeypatch):
    calls = []

    def failed(arguments, **kwargs):
        calls.append((arguments, kwargs))
        kwargs["stdout"].write("Failure details")
        return SimpleNamespace(returncode=5)

    monkeypatch.setattr(build.subprocess, "run", failed)
    runner = build.Runner(tmp_path)
    with pytest.raises(RuntimeError, match="步骤失败"):
        runner.run("test", ["program", "path with spaces"])
    assert (tmp_path / "step-01.log").read_text() == "Failure details"
    assert calls[0][0] == ["program", "path with spaces"]
    assert "shell" not in calls[0][1]


def test_orchestration_produces_traceable_release_after_frozen_check(tmp_path, inputs, monkeypatch):
    model, license_path = inputs
    args = build.parser().parse_args(["--model-bundle", str(model), "--model-license", str(license_path),
                                     "--output", str(tmp_path / "output")])
    monkeypatch.setattr(build, "check_platform", lambda: None)
    monkeypatch.setattr(build, "find_inno", lambda _: tmp_path / "ISCC.exe")
    monkeypatch.setattr(build.venv, "EnvBuilder", lambda **_: SimpleNamespace(create=lambda _: None))
    calls = []

    def run(runner, label, command, **kwargs):
        calls.append(label)
        work = runner.directory
        if "--self-check" in command:
            frozen = command[0].endswith("VisionTactile3D.exe")
            Path(command[-1]).write_text(json.dumps({"schema_version": 1, "status": "ok", "issues": [],
                                                    "details": {"frozen": frozen}}))
        elif "PyInstaller" in command:
            root = work / "dist" / build.NAME
            root.mkdir(parents=True)
            (root / f"{build.NAME}.exe").write_bytes(b"fixture executable")
            for name in build.REQUIRED_RESOURCES:
                path = root / "_internal" / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(model.read_bytes() if name.endswith("right_hand.json") else b"fixture")
            toc = work / "pyinstaller" / build.NAME / "Analysis-00.toc"
            toc.parent.mkdir(parents=True)
            toc.write_text(repr(([('numpy', 'numpy.py', 'PYMODULE')],)))
        elif any(item.endswith("collect_notices.py") for item in command):
            Path(command[-2]).mkdir()
            Path(command[-1]).write_text("[]")
        elif command[0].endswith("ISCC.exe"):
            version = command[1].split("=", 1)[1]
            release = Path(command[3].split("=", 1)[1])
            (release / f"{build.NAME}-{version}-windows-x64-setup.exe").write_bytes(b"fixture installer")

    monkeypatch.setattr(build.Runner, "run", run)
    installer = build.build(args)
    report = json.loads(installer.with_suffix(".manifest.json").read_text(encoding="utf-8"))
    assert report["installer"]["sha256"] == build.sha256(installer)
    assert report["model"]["model_sha256"] == build.sha256(model)
    assert report["frozen_self_check"]["details"]["frozen"] is True
    assert any(item["path"] == "licenses/MODEL_LICENSE.txt" for item in report["files"])
    assert calls.index("验证冻结程序的模型、界面依赖和资源…") < calls.index("生成双语离线安装包…")
