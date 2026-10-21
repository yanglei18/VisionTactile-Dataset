#!/usr/bin/env python3
"""Build the Ubuntu GUI bootstrap without installing build or system dependencies.

The executable carries Python, Tcl/Tk and their runtime notices. The matching
source snapshot and private Tracker package are added by build_tracker_delivery.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import struct
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
NAME = "Install-VisionTactile"
PYINSTALLER_VERSION = "6.22.3"
TARGET = {"system": "Linux", "machine": "x86_64", "python": "3.12", "ubuntu": "24.04"}
SOURCE_PATTERNS = ("tools/capture_deploy/install_full.py", "tools/capture_deploy/oneclick/*.py",
                   "tools/tracker_deploy/src/vt_tracker_deploy/*.py")


class BuildError(ValueError):
    pass


def digest(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def _entry(root, path):
    return {"path": path.relative_to(root).as_posix(), "bytes": path.stat().st_size,
            "sha256": digest(path)}


def source_manifest(root):
    paths = sorted({path for pattern in SOURCE_PATTERNS for path in root.glob(pattern)})
    required = ("tools/capture_deploy/install_full.py", "tools/capture_deploy/oneclick/engine.py",
                "tools/capture_deploy/oneclick/delivery.py", "tools/capture_deploy/oneclick/gui.py",
                "tools/tracker_deploy/src/vt_tracker_deploy/archive.py")
    if any(root / name not in paths for name in required):
        raise BuildError("Bootstrap source files are incomplete.")
    for path in paths:
        if path.is_symlink() or not path.is_file() or not path.resolve().is_relative_to(root):
            raise BuildError("Bootstrap sources must be regular files inside the source root.")
    return [_entry(root, path) for path in paths]


def check_platform():
    try:
        release = platform.freedesktop_os_release()
    except OSError as exc:
        raise BuildError("Build on Ubuntu 24.04 x86_64.") from exc
    if (platform.system() != "Linux" or platform.machine() != "x86_64"
            or release.get("ID") != "ubuntu" or release.get("VERSION_ID") != "24.04"):
        raise BuildError("Build on Ubuntu 24.04 x86_64; cross-compilation is not supported.")


def clean_environment():
    allowed = {"HOME", "LANG", "LC_ALL", "TMPDIR", "http_proxy", "https_proxy", "no_proxy",
               "HTTP_PROXY", "HTTPS_PROXY", "NO_PROXY"}
    env = {key: value for key, value in os.environ.items() if key in allowed}
    env.update(PATH="/usr/bin:/bin:/usr/sbin:/sbin", PYTHONNOUSERSITE="1",
               PYTHONDONTWRITEBYTECODE="1", PYTHONHASHSEED="0")
    return env


def run(argv, *, cwd=None, timeout=120, check=True):
    result = subprocess.run(list(map(str, argv)), cwd=cwd, env=clean_environment(),
                            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, errors="replace",
                            timeout=timeout, check=False, umask=0o077)
    if result.returncode and check:
        raise BuildError("Build command failed: " + result.stdout[-4000:])
    return result


PROBE = '''import importlib.metadata as md, json, pathlib, platform, sys, sysconfig, tkinter
import PyInstaller
licenses = {}
for name in ('pyinstaller', 'pyinstaller-hooks-contrib', 'altgraph', 'packaging', 'setuptools'):
    try:
        distribution = md.distribution(name)
    except md.PackageNotFoundError:
        continue
    paths = [str(distribution.locate_file(item)) for item in (distribution.files or ())
             if pathlib.PurePosixPath(str(item)).name.lower().startswith(('license', 'copying'))
             and pathlib.Path(distribution.locate_file(item)).is_file()]
    if paths:
        licenses[name] = sorted(paths)
print(json.dumps({'system': platform.system(), 'machine': platform.machine(),
 'python': f'{sys.version_info.major}.{sys.version_info.minor}',
 'isolated': sys.prefix != sys.base_prefix, 'pyinstaller': PyInstaller.__version__,
 'stdlib': sysconfig.get_path('stdlib'), 'tcl': str(tkinter.TclVersion),
 'tk': str(tkinter.TkVersion), 'licenses': licenses,
 'packages': sorted(f'{d.metadata["Name"]}=={d.version}' for d in md.distributions()),
 'python_full': platform.python_version()}))
'''


def inspect_python(python):
    if not python.is_absolute() or not python.is_file() or not os.access(python, os.X_OK):
        raise BuildError("--python must be an absolute executable path in the dedicated build venv.")
    try:
        value = json.loads(run([python, "-I", "-c", PROBE]).stdout)
    except (ValueError, OSError, subprocess.SubprocessError) as exc:
        raise BuildError("Cannot inspect the build venv; install Python 3.12, Tcl/Tk and pinned PyInstaller first.") from exc
    if (any(value.get(key) != TARGET[key] for key in ("system", "machine", "python"))
            or not value.get("isolated") or value.get("pyinstaller") != PYINSTALLER_VERSION):
        raise BuildError("A dedicated Linux x86_64 Python 3.12 venv with PyInstaller 6.22.3 is required.")
    if not value.get("licenses", {}).get("pyinstaller"):
        raise BuildError("The PyInstaller distribution is missing its license/bootloader exception notice.")
    return value


def pyinstaller_arguments(python, root, workspace):
    return [str(python), "-I", "-m", "PyInstaller", "--onefile", "--windowed",
            "--name", NAME, "--noupx", "--noconfirm", "--clean",
            "--distpath", str(workspace / "dist"), "--workpath", str(workspace / "work"),
            "--specpath", str(workspace / "spec"),
            "--paths", str(root / "tools/capture_deploy"),
            "--paths", str(root / "tools/tracker_deploy/src"),
            "--collect-submodules", "oneclick", "--collect-submodules", "vt_tracker_deploy",
            "--hidden-import", "tkinter", "--exclude-module", "PySide6",
            "--exclude-module", "PyQt5", "--exclude-module", "PyQt6",
            str(root / "tools/capture_deploy/install_full.py")]


def binary_dependencies(analysis):
    try:
        value = ast.literal_eval(analysis.read_text(encoding="utf-8"))
    except (ValueError, SyntaxError, OSError) as exc:
        raise BuildError("PyInstaller analysis is missing or invalid.") from exc
    dependencies = {}
    def visit(node):
        if not isinstance(node, (tuple, list)):
            return
        if (len(node) == 3 and all(isinstance(part, str) for part in node)
                and node[2] in {"BINARY", "EXTENSION"}):
            name, path, _ = node
            if Path(path).is_absolute():
                dependencies[name] = Path(path)
        else:
            for child in node:
                visit(child)
    visit(value)
    if not dependencies or not any("_tkinter" in name for name in dependencies):
        raise BuildError("PyInstaller analysis did not include the Tk GUI runtime.")
    return dependencies


def _package_owner(path):
    candidates = {str(path), str(path.resolve())}
    for item in tuple(candidates):
        if item.startswith("/usr/lib/"):
            candidates.add(item[4:])
        elif item.startswith("/lib/"):
            candidates.add("/usr" + item)
    for candidate in sorted(candidates):
        result = run(["/usr/bin/dpkg-query", "--search", candidate], check=False)
        if result.returncode:
            continue
        for line in result.stdout.splitlines():
            owner, separator, owned_path = line.partition(": ")
            if separator and owned_path == candidate and re.fullmatch(r"[a-z0-9][a-z0-9+.:~-]*", owner):
                return owner
    raise BuildError("Cannot establish the Debian copyright owner of a bundled native library: " + path.name)


def collect_licenses(directory, analysis, probe, root):
    directory.mkdir(mode=0o755)
    copies = {}
    def copy(path, name):
        if name in copies:
            return copies[name]
        if not path.is_file() or not 0 < path.stat().st_size <= 4 * 1024 * 1024:
            raise BuildError("Required runtime license is unavailable: " + path.name)
        destination = directory / name
        shutil.copyfile(path, destination)
        destination.chmod(0o644)
        copies[name] = "LICENSES/" + name
        return copies[name]
    copy(root / "LICENSE", "VisionTactile-Apache-2.0.txt")
    copy(Path(probe["stdlib"]) / "LICENSE.txt", "Python-LICENSE.txt")
    for distribution, paths in sorted(probe["licenses"].items()):
        for index, path in enumerate(paths):
            copy(Path(path), f"{distribution}-{index + 1}-LICENSE.txt")
    for name, version in (("tcl", probe["tcl"]), ("tk", probe["tk"])):
        copy(Path("/usr/share/doc") / (name + version) / "copyright", name + "-copyright.txt")
    # Debian copyright files refer to these paths. Bundle their actual text so
    # the notice is usable after extraction on a machine without these packages.
    for path in sorted(Path("/usr/share/common-licenses").iterdir()):
        if path.is_file():
            copy(path, "common-" + path.name + ".txt")
    libraries = []
    packages = {}
    for name, path in sorted(binary_dependencies(analysis).items()):
        package = _package_owner(path)
        if package not in packages:
            package_name = package.split(":", 1)[0]
            notice = copy(Path("/usr/share/doc") / package_name / "copyright",
                          "debian-" + package.replace(":", "_") + "-copyright.txt")
            version = run(["/usr/bin/dpkg-query", "--show", "--showformat=${Version}", package]).stdout.strip()
            packages[package] = (version, notice)
        version, notice = packages[package]
        libraries.append({"name": name, "package": package, "version": version, "license": notice})
    readme = directory / "README.txt"
    readme.write_text(
        "Vision Tactile Linux bootstrap runtime notices\n\n"
        "This executable includes Python, Tcl/Tk, PyInstaller's bootloader and the native\n"
        "libraries listed in BUILD_MANIFEST.json (BOOTSTRAP_MANIFEST.json in a combined\n"
        "delivery). Their copyright and license texts are\n"
        "provided in this directory. PyInstaller's license includes its bootloader\n"
        "exception. Debian copyright references to /usr/share/common-licenses/NAME\n"
        "correspond to common-NAME.txt here. Package versions identify the matching\n"
        "Ubuntu source packages; their sources are available from archive.ubuntu.com.\n"
        "PyInstaller sources: https://github.com/pyinstaller/pyinstaller/tree/v6.22.3\n"
        "Python sources: https://www.python.org/downloads/source/\n"
        "Tcl/Tk sources: https://www.tcl-lang.org/software/tcltk/\n"
        "Application sources and their notices accompany the combined delivery.\n",
        encoding="utf-8")
    readme.chmod(0o644)
    return libraries


def verify_elf(path):
    with path.open("rb") as stream:
        header = stream.read(20)
    if (len(header) != 20 or header[:6] != b"\x7fELF\x02\x01"
            or struct.unpack("<H", header[18:20])[0] != 62):
        raise BuildError("The generated bootstrap is not an ELF x86_64 executable.")


def build(root, python, output):
    root = Path(root).expanduser().resolve()
    python = Path(python).expanduser().absolute() if Path(python).is_absolute() else Path(python)
    output = Path(output).expanduser().absolute()
    check_platform()
    if output.exists() or output.is_symlink():
        raise BuildError("Output already exists; choose a new output directory.")
    if not output.parent.is_dir() or output.resolve().is_relative_to(root):
        raise BuildError("Output must have an existing parent outside the source repository.")
    sources = source_manifest(root)
    probe = inspect_python(python)
    staging = Path(tempfile.mkdtemp(prefix=".vt-linux-bootstrap-", dir=output.parent))
    try:
        workspace = staging / "build"
        workspace.mkdir(mode=0o700)
        export = staging / "export"
        export.mkdir(mode=0o700)
        run(pyinstaller_arguments(python, root, workspace), cwd=workspace, timeout=1800)
        binary = workspace / "dist" / NAME
        verify_elf(binary)
        destination = export / NAME
        shutil.copyfile(binary, destination)
        destination.chmod(0o755)
        libraries = collect_licenses(export / "LICENSES", workspace / "work" / NAME / "Analysis-00.toc", probe, root)
        if source_manifest(root) != sources:
            raise BuildError("Bootstrap sources changed during the build; rebuild from a stable snapshot.")
        manifest = {"schema_version": 1, "kind": "visiontactile-linux-bootstrap",
                    "target": TARGET, "pyinstaller": PYINSTALLER_VERSION, "sources": sources,
                    "build_environment": {"python": probe.get("python_full"),
                                          "packages": probe.get("packages", [])},
                    "files": [_entry(export, path) for path in sorted(export.rglob("*")) if path.is_file()],
                    "runtime_libraries": libraries}
        (export / "BUILD_MANIFEST.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        # mkdir is the exclusive publication point; never replace another build.
        output.mkdir(mode=0o700)
        identity = output.stat()
        try:
            for path in export.iterdir():
                os.rename(path, output / path.name)
        except BaseException:
            current = output.lstat()
            if (current.st_dev, current.st_ino) == (identity.st_dev, identity.st_ino):
                shutil.rmtree(output)
            raise
        return {"output": str(output), "binary": str(output / NAME),
                "sha256": digest(output / NAME), "source_files": len(sources)}
    finally:
        shutil.rmtree(staging)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--python", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        report = build(args.root, args.python, args.output)
    except (ValueError, OSError, subprocess.SubprocessError) as exc:
        parser.exit(1, str(exc) + "\n")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
