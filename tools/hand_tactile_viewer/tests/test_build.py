"""Release contracts verified without a Windows compiler or serial hardware."""
from __future__ import annotations

import ast
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest


WINDOWS = Path(__file__).parents[1] / "windows"
SPEC = importlib.util.spec_from_file_location("hand_motion_windows_build", WINDOWS / "build.py")
build = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(build)


@pytest.fixture
def resource_bundle(tmp_path):
    root = tmp_path / "运行 资源"
    root.mkdir()
    files = {}
    for name in build.REQUIRED_RESOURCE_FILES:
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("Test fixture only: " + name, encoding="utf-8")
        files[name] = build.sha256(path)
    manifest = {"schema_version": 1, "product": "hand-motion-touch", "files": files}
    (root / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    return root, manifest


def test_staging_copies_declared_resources_only_and_keeps_original_bytes(resource_bundle, tmp_path):
    root, manifest = resource_bundle
    (root / "developer-secret.txt").write_text("do not bundle", encoding="utf-8")
    destination = tmp_path / "staged resources"
    assert build.check_resources(root) == manifest
    build.stage_resources(root, destination, manifest)
    assert build.check_resources(destination) == manifest
    assert not (destination / "developer-secret.txt").exists()
    for name in manifest["files"]:
        assert (destination / name).read_bytes() == (root / name).read_bytes()


@pytest.mark.parametrize("fault", ["wrong-product", "missing-model", "missing-source", "boolean-schema",
                                    "corrupt-model", "outside", "windows-absolute", "case-collision"])
def test_invalid_resources_fail_before_any_build_step(resource_bundle, monkeypatch, fault):
    root, manifest = resource_bundle
    if fault == "wrong-product":
        manifest["product"] = "tactile-only"
    elif fault == "missing-model":
        del manifest["files"]["mano/models/MANO_RIGHT.pkl"]
    elif fault == "missing-source":
        del manifest["files"]["upstream/src/hand_visualiser/modules/utils.py"]
    elif fault == "boolean-schema":
        manifest["schema_version"] = True
    elif fault == "corrupt-model":
        (root / "mano/models/MANO_RIGHT.pkl").write_bytes(b"changed")
    elif fault == "outside":
        manifest["files"]["../outside.txt"] = "0" * 64
    elif fault == "windows-absolute":
        manifest["files"]["C:/outside.txt"] = "0" * 64
    elif fault == "case-collision":
        manifest["files"]["HAND.json"] = manifest["files"]["hand.json"]
    (root / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    monkeypatch.setattr(build._shared, "check_platform", lambda: pytest.fail("build started"))
    with pytest.raises(ValueError):
        build.build(SimpleNamespace(resources=root))


def test_staging_detects_changed_source(resource_bundle, tmp_path):
    root, manifest = resource_bundle
    (root / "rest.json").write_bytes(b"changed since preflight")
    with pytest.raises(ValueError, match="校验失败"):
        build.stage_resources(root, tmp_path / "staged", manifest)


def test_release_version_comes_from_own_project(tmp_path):
    (tmp_path / "pyproject.toml").write_text('[project]\nversion = "2.3.4"\n', encoding="utf-8")
    assert build.read_version(tmp_path) == "2.3.4"
    assert build.read_version() == "0.1.0"


@pytest.mark.parametrize("version", ["1.2", "1.2.3-beta", "１.２.３", "1.2.65536"])
def test_invalid_pe_version_is_rejected(tmp_path, version):
    (tmp_path / "pyproject.toml").write_text(f'[project]\nversion = "{version}"\n', encoding="utf-8")
    with pytest.raises(ValueError, match="安装包版本"):
        build.read_version(tmp_path)


def test_freezer_keeps_dynamic_numerics_and_one_qt_binding(tmp_path):
    python, workspace, resources = (tmp_path / name for name in ("Python path/python.exe", "构建 目录", "运行 资源"))
    arguments = build.pyinstaller_arguments(python, workspace, resources)
    assert arguments[:4] == [str(python), "-I", "-m", "PyInstaller"]
    assert f"{resources}:vt_hand_tactile_viewer/assets/runtime" in arguments
    collected = {arguments[index + 1] for index, arg in enumerate(arguments) if arg == "--collect-all"}
    assert {"chumpy", "manotorch", "qmt"} <= collected
    assert "torch" not in collected  # official hook omits multi-GB static development libraries
    hidden = {arguments[index + 1] for index, arg in enumerate(arguments) if arg == "--hidden-import"}
    assert {"torch", "chumpy", "cv2", "pydantic", "sklearn.gaussian_process", "manotorch.axislayer"} <= hidden
    excluded = {arguments[index + 1] for index, arg in enumerate(arguments) if arg == "--exclude-module"}
    assert {"PyQt5", "PyQt6", "PySide2", "rclpy", "rospy", "vt_capture_console.app"} <= excluded
    assert {"PySide6.QtWebEngineCore", "PySide6.QtWebEngineQuick", "PySide6.QtWebEngineWidgets"} <= excluded
    assert not {"PySide6", "torch", "manotorch", "qmt"} & excluded
    assert arguments[-1] == str(WINDOWS / "entrypoint.py")
    assert build._shared.NAME == "VisionTactile3D"  # no mutation of the old builder


def test_frozen_entrypoint_dispatches_spawn_before_app_import():
    tree = ast.parse((WINDOWS / "entrypoint.py").read_text(encoding="utf-8"))
    guard = next(node for node in tree.body if isinstance(node, ast.If))
    assert isinstance(guard.body[0], ast.Expr)
    assert ast.unparse(guard.body[0].value) == "multiprocessing.freeze_support()"
    assert isinstance(guard.body[1], ast.ImportFrom)
    assert guard.body[1].module == "vt_hand_tactile_viewer.app"
    assert not any(isinstance(node, ast.ImportFrom) for node in tree.body)


@pytest.mark.parametrize("fault", [None, "wrong-version", "wrong-binding", "no-worker", "opened-hardware", "source-only"])
def test_frozen_report_requires_real_worker_without_device(tmp_path, fault):
    details = dict(version="0.1.0", frozen=True, qt_binding="PySide6", serial_device="not_opened",
                   solver_process="initialized_without_device")
    if fault == "wrong-version":
        details["version"] = "0.6.2"
    elif fault == "wrong-binding":
        details["qt_binding"] = "PyQt5"
    elif fault == "no-worker":
        details.pop("solver_process")
    elif fault == "opened-hardware":
        details["serial_device"] = "COM3"
    elif fault == "source-only":
        details["frozen"] = False
    path = tmp_path / "report.json"
    report = dict(schema_version=1, status="ok", issues=[], details=details)
    path.write_text(json.dumps(report), encoding="utf-8")
    if fault is None:
        assert build.check_report(path, frozen=True, version="0.1.0") == report
    else:
        with pytest.raises(RuntimeError):
            build.check_report(path, frozen=True, version="0.1.0")


@pytest.mark.parametrize("fault", ["excluded-module", "static-library", "torch-header", "web-engine", "missing-chumpy-source"])
def test_freezer_result_rejects_missing_runtime_files_and_unused_dependencies(resource_bundle, tmp_path, fault):
    root, manifest = resource_bundle
    distribution = tmp_path / "distribution"
    build.stage_resources(root, distribution / "_internal" / build.RESOURCE_DESTINATION, manifest)
    (distribution / "VisionHandMotionTouch.exe").write_bytes(b"fixture")
    plugin = distribution / "_internal/PySide6/plugins/platforms/qwindows.dll"
    plugin.parent.mkdir(parents=True)
    plugin.write_bytes(b"fixture")
    for filename in ("quaternion.py", "heading_correction.py"):
        source = distribution / "_internal/qmt/functions" / filename
        source.parent.mkdir(parents=True, exist_ok=True)
        source.write_text("# fixture\n", encoding="utf-8")
    source = distribution / "_internal/chumpy/np_tensordot.py"
    source.parent.mkdir(parents=True)
    source.write_text("# fixture\n", encoding="utf-8")
    toc = tmp_path / "Analysis-00.toc"
    toc.write_text(repr([[('PySide6.QtCore', 'module.pyd', 'EXTENSION')]]), encoding="utf-8")
    build.audit_distribution(distribution, toc, manifest)
    if fault == "excluded-module":
        toc.write_text(repr([[('PyQt5.QtCore', 'module.pyd', 'EXTENSION')]]), encoding="utf-8")
    elif fault == "missing-chumpy-source":
        source.unlink()
    else:
        relative = {"static-library": "torch/lib/dnnl.lib", "torch-header": "torch/include/ATen.h",
                    "web-engine": "PySide6/Qt6WebEngineCore.dll"}[fault]
        unexpected = distribution / "_internal" / relative
        unexpected.parent.mkdir(parents=True, exist_ok=True)
        unexpected.write_bytes(b"unused file")
    with pytest.raises(ValueError):
        build.audit_distribution(distribution, toc, manifest)


@pytest.mark.parametrize("failed_phase", ["source", "frozen"])
def test_finish_build_requires_new_reports_when_reusing_a_workspace(resource_bundle, tmp_path, monkeypatch, failed_phase):
    root, manifest = resource_bundle
    workspace = tmp_path / "existing-build"
    build.stage_resources(root, workspace / "runtime", manifest)

    def report(frozen):
        return dict(schema_version=1, status="ok", issues=[], details=dict(
            version="0.1.0", frozen=frozen, qt_binding="PySide6", serial_device="not_opened",
            solver_process="initialized_without_device"))

    for phase in ("source", "frozen"):
        (workspace / f"{phase}-self-check.json").write_text(json.dumps(report(phase == "frozen")), encoding="utf-8")

    class Runner:
        commands = []

        def run(self, label, arguments, **kwargs):
            self.commands.append(arguments)
            if "--self-check" in arguments:
                output = Path(arguments[arguments.index("--self-check") + 1])
                assert not output.exists(), "Previous successful report must be removed"
                if output.name == "source-self-check.json" and failed_phase == "frozen":
                    output.write_text(json.dumps(report(False)), encoding="utf-8")

    runner = Runner()
    monkeypatch.setattr(build, "audit_distribution", lambda *args: None)
    with pytest.raises(RuntimeError, match="没有生成自检报告"):
        build.finish_build(workspace, runner, version="0.1.0", manifest=manifest,
                           output=tmp_path, compiler=tmp_path / "ISCC.exe", lock=tmp_path / "requirements.lock")
    assert runner.commands[0][-2:] == ["pip", "check"]
    assert any("PyInstaller" in command for command in runner.commands) is (failed_phase == "frozen")
    assert not any(str(tmp_path / "ISCC.exe") == command[0] for command in runner.commands)


def test_installer_identity_shortcuts_and_storage_are_independent():
    installer = (WINDOWS / "installer.iss").read_text(encoding="utf-8")
    assert "AppId=VisionTactile.HandMotionTouch.Windows.x64" in installer
    assert "DefaultDirName={localappdata}\\Programs\\VisionTactile\\HandMotionTouch" in installer
    assert 'Filename: "{app}\\VisionHandMotionTouch.exe"' in installer
    assert "en.ProductName=Hand Motion and Touch Viewer" in installer
    assert "zh_CN.ProductName=手势与触觉可视化" in installer
    assert "VisionTactile3D.exe" not in installer
    assert "Tactile3D.Windows.x64" not in installer
    assert "filesandordirs" not in installer
    build._shared.check_vendor_resources()


def test_build_parser_preserves_paths_with_spaces():
    args = build.parser().parse_args(["--resources", "D:/运行 资源", "--wheelhouse", "D:/wheel cache",
                                     "--output", "D:/build output", "--iscc", "D:/Inno Setup/ISCC.exe"])
    assert args.resources == Path("D:/运行 资源")
    assert args.wheelhouse == Path("D:/wheel cache")
    assert args.output == Path("D:/build output")
    assert args.iscc == Path("D:/Inno Setup/ISCC.exe")
