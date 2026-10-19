#!/usr/bin/env python3
"""Build sdists, rebuild wheels, and smoke installed public artifacts in isolation.

Requires Python 3.12, venv support, network/pip cache, and the Qt system libraries
documented by the console. Does not import ROS, open sensors, install desktop
shortcuts, or require separately licensed MANO / FSGlove / PyVUT runtimes.
The all profile builds the ROS Python adapters but does not run their ROS nodes.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import stat
import subprocess
import sys
import tarfile
import tempfile
import time
import zipfile


ROOT = Path(__file__).resolve().parents[2]
# key: source directory, import name, console entry point (hardware-free --help).
PACKAGES = {
    "core": ("ros2_ws/src/vt_capture_core", "vt_capture_core", None),
    "manager": ("ros2_ws/src/vt_capture_manager", "vt_capture_manager", None),
    "tactile": ("ros2_ws/src/vt_tactile_capture", "vt_tactile_capture", None),
    "console": ("tools/capture_console", "vt_capture_console", "vt-capture-console"),
    "fsglove12": ("tools/fsglove12", "vt_fsglove12", "vt-fsglove12"),
    "alignment": ("tools/multisensor_alignment", "vt_multisensor_alignment", "vt-multisensor-align"),
    "calibration": ("tools/tracker_camera_calibration", "vt_tracker_camera_calib", "vt-tracker-camera-calibrate"),
    "glove_pose": ("tools/glove_hand_pose", "vt_glove_pose", "vt-glove-pose"),
    "validation": ("tools/vut_validation", "vut_validation", "vt-vut-validate"),
    "glove_bridge": ("ros2_ws/src/vt_glove_bridge", "vt_glove_bridge", None),
    "tracker": ("ros2_ws/src/vt_vive_tracker", "vt_vive_tracker", None),
    "tracker_gui": ("ros2_ws/src/vt_vive_tracker_gui", "vt_vive_tracker_gui", None),
}
PROFILES = {
    "core": ("core", "manager"),
    "offline": ("core", "manager", "alignment", "calibration", "validation"),
    "console": ("core", "manager", "tactile", "fsglove12", "console"),
    "all": tuple(PACKAGES),
}


def clean_environment(work):
    """Prevent a local ROS overlay, editable install or Qt plugin from helping."""
    prefixes = ("PYTHON", "PIP_", "CONDA", "ROS", "AMENT", "COLCON", "QT", "QML",
                "PYSIDE", "VT_", "FSGLOVE", "MANO", "GIT_")
    blocked = {"VIRTUAL_ENV", "LD_LIBRARY_PATH", "LD_PRELOAD", "CMAKE_PREFIX_PATH",
               "DISPLAY", "WAYLAND_DISPLAY", "DBUS_SESSION_BUS_ADDRESS"}
    env = {key: value for key, value in os.environ.items()
           if key not in blocked and not key.startswith(prefixes)}
    env.update(PATH=os.defpath, PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1",
               PIP_CONFIG_FILE=os.devnull, PIP_DISABLE_PIP_VERSION_CHECK="1",
               QT_QPA_PLATFORM="offscreen", QT_API="pyside6")
    for name in ("CONFIG", "CACHE", "DATA", "STATE", "RUNTIME"):
        directory = work / name.lower()
        directory.mkdir(mode=0o700, parents=True, exist_ok=True)
        env["XDG_" + name + ("_DIR" if name == "RUNTIME" else "_HOME")] = str(directory)
    return env


def archive_files(path):
    """Read only; never extract potentially unsafe archive entries."""
    files, seen = {}, set()

    def check_name(name):
        normalized = PurePosixPath(name)
        if (normalized.is_absolute() or ".." in normalized.parts
                or normalized.as_posix() != name.rstrip("/") or normalized.as_posix() in seen):
            raise ValueError("Unsafe archive member in " + path.name)
        seen.add(normalized.as_posix())

    if path.suffix == ".whl":
        with zipfile.ZipFile(path) as archive:
            for item in archive.infolist():
                check_name(item.filename)
                mode = stat.S_IFMT(item.external_attr >> 16)
                if mode not in (0, stat.S_IFREG, stat.S_IFDIR):
                    raise ValueError("Linked or special wheel member in " + path.name)
                if not item.is_dir():
                    files[item.filename] = archive.read(item)
    else:
        with tarfile.open(path) as archive:
            for item in archive:
                check_name(item.name)
                if not item.isfile() and not item.isdir():
                    raise ValueError("Linked or special sdist member in " + path.name)
                if item.isfile():
                    files[item.name] = archive.extractfile(item).read()
    return files


def verify_artifact(path, source, module):
    files = archive_files(path)
    # Compare the actual license and existing third-party notice, not just names.
    for name in ("LICENSE", "THIRD_PARTY_NOTICES.md"):
        document = source / name
        if name == "LICENSE" and not document.is_file():
            raise ValueError("Missing package LICENSE: " + source.name)
        if document.is_file() and not any(
                PurePosixPath(member).name == name and content == document.read_bytes()
                for member, content in files.items()):
            raise ValueError(f"{path.name} omits or changes {name}")
    package_root = source / "src" / module
    if not package_root.is_dir():
        package_root = source / module
    assets = {str(p.relative_to(package_root)): p.read_bytes()
              for p in package_root.rglob("*") if p.is_file()
              and p.suffix in {".json", ".qss", ".svg", ".xml"}
              and "__pycache__" not in p.parts}
    for relative, expected in assets.items():
        suffix = module + "/" + relative
        if not any((name == suffix or name.endswith("/" + suffix)) and data == expected
                   for name, data in files.items()):
            raise ValueError(f"{path.name} omits or changes package asset {suffix}")
    # These ROS setup.py packages declare package.xml, launch/config/rviz data
    # and the license under share/<package>. Check each in the wheel as well.
    share_assets = []
    if (source / "package.xml").is_file():
        share_assets = [source / "package.xml", source / "LICENSE"]
        for pattern in ("launch/*.launch.py", "config/*.yaml", "rviz/*.rviz"):
            share_assets.extend(source.glob(pattern))
        for asset in share_assets:
            relative = asset.relative_to(source).as_posix()
            suffix = ("share/" + module + "/" + relative if path.suffix == ".whl" else relative)
            if not any(name.endswith("/" + suffix) and data == asset.read_bytes()
                       for name, data in files.items()):
                raise ValueError(f"{path.name} omits or changes ROS share asset {relative}")
    return {"file": path.name, "bytes": path.stat().st_size,
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "verified_assets": len(assets), "verified_ros_share_assets": len(share_assets)}


def snapshot_package(source, target, *, policy=None, root=None, allowed=None):
    # Keep normal backend output outside the user's checkout. Build hooks are
    # trusted project code, not OS-sandboxed. Ignore stale editable egg-info.
    source_mode = source.lstat().st_mode
    if not stat.S_ISDIR(source_mode):
        raise ValueError("Package source must be a real directory, not a link or special entry")
    ignore = shutil.ignore_patterns(".git", ".venv*", "__pycache__", "*.pyc", "*.egg-info",
                                    ".pytest_cache", "build", "dist", "output", "artifacts", ".runtime", ".local")
    target.mkdir(parents=True)
    for directory, subdirectories, filenames in os.walk(source, followlinks=False):
        directory = Path(directory)
        ignored = ignore(str(directory), subdirectories + filenames)
        for name in subdirectories[:] + filenames:
            entry = directory / name
            info = entry.lstat()
            if stat.S_ISLNK(info.st_mode) or not (stat.S_ISREG(info.st_mode) or stat.S_ISDIR(info.st_mode)):
                raise ValueError("Linked or special source entry: " + entry.relative_to(source).as_posix())
            is_directory = stat.S_ISDIR(info.st_mode)
            relative = entry.relative_to(root or source).as_posix()
            excluded = name in ignored or (policy and policy.forbidden_path(relative + ("/" if is_directory else "")))
            if not is_directory and allowed is not None and relative not in allowed:
                excluded = True
            if excluded:
                if name in subdirectories:
                    subdirectories.remove(name)
                continue
            destination = target / entry.relative_to(source)
            if is_directory:
                destination.mkdir()
            else:
                if policy and info.st_size > policy.MAX_TRACKED_BYTES:
                    raise ValueError("Oversize source entry: " + relative)
                shutil.copy2(entry, destination, follow_symlinks=False)


def source_inventory(source):
    return [{"path": path.relative_to(source).as_posix(), "bytes": path.stat().st_size,
             "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
            for path in sorted(source.rglob("*")) if path.is_file()]


class Runner:
    def __init__(self, output, env):
        self.output, self.env, self.count = output, env, 0

    def run(self, label, command, cwd, *, env=None, timeout=600):
        self.count += 1
        log = self.output / f"{self.count:02d}-{label}.log"
        print(label, flush=True)
        with log.open("w") as stream:
            result = subprocess.run([str(arg) for arg in command], cwd=cwd,
                                    env=env or self.env, stdout=stream,
                                    stderr=subprocess.STDOUT, timeout=timeout)
        if result.returncode:
            raise RuntimeError(f"{label} failed; inspect {log.name}")
        return log


def venv_python(path):
    return path / "bin" / "python"


def smoke_environment(runner, sandbox, output, wheels, keys, name, constraints):
    environment = sandbox / name
    python = venv_python(environment)
    runner.run(name + "-create", [sys.executable, "-m", "venv", environment], sandbox)
    env = clean_environment(sandbox / (name + "-state"))
    env["QT_API"] = "pyqt5" if name == "pyqt5" else "pyside6"
    runner.run(name + "-install", [python, "-m", "pip", "install", "--no-input",
               "-c", constraints, *[wheels[key] for key in keys]], sandbox, env=env)
    runner.run(name + "-pip-check", [python, "-m", "pip", "check"], sandbox, env=env)
    modules = [PACKAGES[key][1] for key in keys]
    smoke = """
import compileall, importlib, importlib.metadata, json, pathlib, sys
modules = json.loads(sys.argv[1])
prefix = pathlib.Path(sys.prefix).resolve()
for name in modules:
    module = importlib.import_module(name)
    path = pathlib.Path(module.__file__).resolve()
    assert path.is_relative_to(prefix), (name, 'module loaded outside fresh environment')
    assert compileall.compile_dir(str(path.parent), quiet=1), name
for distribution in importlib.metadata.distributions():
    direct = distribution.read_text('direct_url.json')
    assert not direct or not json.loads(direct).get('dir_info', {}).get('editable'), 'editable install'
assert 'rclpy' not in sys.modules, 'ROS imported in offline smoke'
installed = {item.metadata['Name'].lower().replace('_', '-') for item in importlib.metadata.distributions()}
assert not ({'pyqt5', 'pyside6'} <= installed), 'Qt bindings mixed in environment'
if 'vt_capture_core' in modules:
    import xml.etree.ElementTree as ET
    from vt_capture_core.ros_transport import capture_transport_environment
    profile = pathlib.Path(capture_transport_environment({})['FASTRTPS_DEFAULT_PROFILES_FILE'])
    assert profile.is_relative_to(prefix), 'DDS profile loaded outside fresh environment'
    ET.parse(profile)
if 'vt_capture_manager' in modules:
    for name in ('config', 'model', 'server', 'storage'):
        importlib.import_module('vt_capture_manager.' + name)
print('Installed-only imports and Python syntax PASS')
"""
    runner.run(name + "-imports", [python, "-I", "-c", smoke, json.dumps(modules)], sandbox, env=env)
    for key in keys:
        command = PACKAGES[key][2]
        if command:
            runner.run(key + "-help", [environment / "bin" / command, "--help"], sandbox, env=env)
    if "calibration" in keys:
        for module in ("guided_worker", "recorder_runner"):
            runner.run("calibration-" + module + "-help",
                       [python, "-I", "-m", "vt_tracker_camera_calib." + module, "--help"], sandbox, env=env)
    if "fsglove12" in keys:
        runner.run("fsglove12-config", [environment / "bin/vt-fsglove12", "layout"], sandbox, env=env)
    if "console" in keys:
        for language in ("zh_CN", "en"):
            screenshot = output / ("console-" + language + ".png")
            runner.run("console-demo-" + language, [environment / "bin/vt-capture-console",
                       "--demo", "--language", language, "--size", "1366x768", "--smoke-seconds", "2",
                       "--screenshot", screenshot], sandbox, env=env, timeout=60)
            if not screenshot.is_file() or not screenshot.read_bytes().startswith(b"\x89PNG\r\n\x1a\n"):
                raise RuntimeError("Console demo did not produce its screenshot")
    if "glove_pose" in keys:
        runner.run("glove-synthetic", [environment / "bin/vt-glove-pose", "demo", "--headless",
                   "--seconds", "1"], sandbox, env=env, timeout=60)
        runner.run("glove-gui", [environment / "bin/vt-glove-pose", "demo", "--smoke-seconds", "1"],
                   sandbox, env=env, timeout=60)
    inventory_log = runner.run(name + "-inventory", [python, "-m", "pip", "list", "--format=json"],
                               sandbox, env=env)
    # Names/versions only, never direct_url.json or editable local source paths.
    return sorted(({"name": item["name"], "version": item["version"]}
                   for item in json.loads(inventory_log.read_text())), key=lambda item: item["name"].lower())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=PROFILES, default="all")
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--output", type=Path, required=True, help="New directory for artifacts, logs and report.json")
    parser.add_argument("--build-only", action="store_true", help="Check sdists/wheels without claiming install/runtime validation")
    args = parser.parse_args(argv)
    if sys.version_info < (3, 12):
        parser.error("Python 3.12 or newer is required")
    root, output = args.root.resolve(), args.output.resolve()
    if output.is_relative_to(root) and not output.is_relative_to(root / ".runtime"):
        parser.error("In-repository output must be under .runtime")
    output.mkdir(parents=True, exist_ok=False)
    keys = PROFILES[args.profile]
    report = {"schema_version": 1, "profile": args.profile, "status": "FAIL",
              "build_only": args.build_only, "artifacts": {}, "sources": {}, "environments": {},
              "boundaries": ["No hardware or ROS node validation", "No MANO, upstream FSGlove or PyVUT runtime validation",
                             "Not a complete dependency lock or reproducible-build certification"]}
    started = time.monotonic()
    try:
        with tempfile.TemporaryDirectory(prefix="vt-distribution-check-") as temporary:
            sandbox = Path(temporary)
            env = clean_environment(sandbox / "builder-state")
            runner = Runner(output, env)
            builder = sandbox / "builder"
            python = venv_python(builder)
            constraints = output / "constraints-tested.txt"
            shutil.copyfile(root / "tools/release/constraints-tested.txt", constraints)
            report["constraints_sha256"] = hashlib.sha256(constraints.read_bytes()).hexdigest()
            spec = importlib.util.spec_from_file_location("public_source_policy", root / "tools/check_public_tree.py")
            policy = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(policy)
            allowed = None
            if (root / ".git").exists():
                listed = subprocess.run(["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
                                        cwd=root, env=env, check=True, stdout=subprocess.PIPE).stdout
                allowed = set(listed.decode().split("\0"))
            runner.run("builder-create", [sys.executable, "-m", "venv", builder], sandbox)
            runner.run("builder-install", [python, "-m", "pip", "install", "--no-input", "-c", constraints,
                       "build", "setuptools", "wheel"], sandbox)
            wheels = {}
            for key in keys:
                relative, module, _ = PACKAGES[key]
                source = sandbox / "sources" / key
                snapshot_package(root / relative, source, policy=policy, root=root, allowed=allowed)
                artifacts = output / key
                artifacts.mkdir()
                inventory_bytes = (json.dumps(source_inventory(source), indent=2) + "\n").encode()
                (artifacts / "source-manifest.json").write_bytes(inventory_bytes)
                report["sources"][key] = {"directory": relative,
                    "manifest_sha256": hashlib.sha256(inventory_bytes).hexdigest()}
                runner.run(key + "-sdist", [python, "-m", "build", "--no-isolation", "--sdist",
                           "--outdir", artifacts, source], sandbox)
                sdist, = artifacts.glob("*.tar.gz")
                checked_sdist = verify_artifact(sdist, source, module)
                # pip builds from the sdist in its own temporary directory, not
                # the copied source tree. Missing sdist inputs fail this step.
                runner.run(key + "-wheel", [python, "-m", "pip", "wheel", "--no-deps", "--no-build-isolation",
                           "--wheel-dir", artifacts, sdist], sandbox)
                wheel, = artifacts.glob("*.whl")
                report["artifacts"][key] = [checked_sdist, verify_artifact(wheel, source, module)]
                wheels[key] = wheel
            if not args.build_only:
                standard = [key for key in keys if key != "glove_pose"]
                report["environments"]["standard"] = smoke_environment(
                    runner, sandbox, output, wheels, standard, "standard", constraints)
                if "glove_pose" in keys:
                    report["environments"]["pyqt5"] = smoke_environment(
                        runner, sandbox, output, wheels, ["glove_pose"], "pyqt5", constraints)
            report["status"] = "PASS"
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        report["error"] = str(exc)
        print(str(exc), file=sys.stderr)
    finally:
        report["elapsed_seconds"] = round(time.monotonic() - started, 2)
        (output / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(f"{report['status']}: {output / 'report.json'}")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
