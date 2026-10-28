"""Rebuild pinned pure wheels and download the locked Windows x64 wheelhouse.

Works with Python 3.12 on Windows or Linux. No model files, hardware, compiler,
private checkout, or previously configured workstation are required.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import subprocess
import sys
import tempfile
import urllib.request
import venv
import zipfile

from wheel_utils import SOURCE_DATE_EPOCH, normalize_wheel


HERE = Path(__file__).resolve().parent
BUILD_VERSIONS = {"pip": "24.0", "setuptools": "78.1.0", "wheel": "0.48.0", "packaging": "26.3"}
SOURCES = {
    "manotorch": {
        "version": "0.0.2", "commit": "a2a70c591f91551078b7bb2af9b5d9f275b626e0",
        "url": "https://codeload.github.com/lixiny/manotorch/zip/a2a70c591f91551078b7bb2af9b5d9f275b626e0",
        "sha256": "c31ab8236eca0ff75a1814c8b0b184d57a9acbf12bdcaa5083702929f4a8dfde",
    },
    "chumpy": {
        "version": "0.71", "commit": "580566eafc9ac68b2614b64d6f7aaa84eebb70da",
        "url": "https://codeload.github.com/mattloper/chumpy/zip/580566eafc9ac68b2614b64d6f7aaa84eebb70da",
        "sha256": "b668f0bd571bc8ccc0b09ebc696b52689949b3be3e893b3fa5cd27a8956cb10e",
    },
    "qmt": {
        "version": "0.2.4+python",
        "url": "https://files.pythonhosted.org/packages/f9/e0/df9cea0bfb4e702e9072ed9033c8c00ce52bdc9815852ca12717e95016e2/qmt-0.2.4.tar.gz",
        "sha256": "27fff51c04e4cf4441764623572fa84d8f3743411e24971c44273a28db0faedc",
    },
}


def sha256(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def environment():
    result = dict(os.environ)
    for key in ("PYTHONPATH", "PYTHONHOME", "VIRTUAL_ENV"):
        result.pop(key, None)
    result.update(SOURCE_DATE_EPOCH=str(SOURCE_DATE_EPOCH), PYTHONUTF8="1", PYTHONNOUSERSITE="1")
    return result


def run(arguments):
    subprocess.run([str(item) for item in arguments], env=environment(), check=True)


def download_source(name, directory):
    record = SOURCES[name]
    filename = f"{name}-{record['commit']}.zip" if "commit" in record else "qmt-0.2.4.tar.gz"
    path = directory / filename
    if path.exists():
        if sha256(path) != record["sha256"]:
            raise ValueError(f"Cached source checksum mismatch: {path}")
        return path
    temporary = path.with_suffix(path.suffix + ".download")
    try:
        with urllib.request.urlopen(record["url"], timeout=120) as source, temporary.open("wb") as target:
            while block := source.read(1024 * 1024):
                target.write(block)
        if sha256(temporary) != record["sha256"]:
            raise ValueError(f"Downloaded source checksum mismatch: {name}")
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)
    return path


def unpack_source(archive, directory):
    with zipfile.ZipFile(archive) as source:
        roots = set()
        for member in source.infolist():
            path = PurePosixPath(member.filename)
            if (path.is_absolute() or ".." in path.parts or "\\" in member.filename
                    or ":" in member.filename or (member.external_attr >> 16) & 0o170000 == 0o120000):
                raise ValueError("Invalid source archive member")
            roots.add(path.parts[0])
        if len(roots) != 1:
            raise ValueError("Source archive must have exactly one root")
        source.extractall(directory)
        return directory / roots.pop()


def builder_python(workspace, explicit=None):
    if explicit:
        # Resolving a venv's executable symlink would silently run base Python.
        python = explicit.expanduser().absolute()
        if not python.is_file():
            raise ValueError(f"Builder Python does not exist: {python}")
    else:
        target = workspace / "builder"
        venv.EnvBuilder(with_pip=True).create(target)
        python = target / ("Scripts/python.exe" if sys.platform == "win32" else "bin/python")
        run([python, "-I", "-m", "pip", "--isolated", "install", "--only-binary=:all:",
             *(f"{name}=={version}" for name, version in BUILD_VERSIONS.items())])
    script = ("import importlib.metadata as m,json,sys; "
              f"print(json.dumps(dict(python=list(sys.version_info[:2]), versions={{n:m.version(n) for n in {list(BUILD_VERSIONS)!r}}})))")
    actual = json.loads(subprocess.check_output([str(python), "-I", "-c", script], env=environment(), text=True))
    if actual != {"python": [3, 12], "versions": BUILD_VERSIONS}:
        raise ValueError(f"Source-wheel builder must be Python 3.12 with {BUILD_VERSIONS}; got {actual}")
    return python


def build_source_wheels(python, sources, output, workspace):
    wheels = {}
    for name, record in SOURCES.items():
        archive = download_source(name, sources)
        if name == "qmt":
            run([python, HERE / "prepare_qmt_wheel.py", "--source", archive, "--output", output])
        else:
            root = unpack_source(archive, workspace / name)
            run([python, "-I", "-m", "pip", "--isolated", "wheel", "--no-deps", "--no-build-isolation",
                 "--no-index", "--wheel-dir", output, root])
        wheel = output / f"{name}-{record['version']}-py3-none-any.whl"
        normalize_wheel(wheel)
        wheels[name] = {"file": wheel.name, "sha256": sha256(wheel)}
    return wheels


def read_lock(path):
    entries = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        match = re.fullmatch(r"([A-Za-z0-9_.-]+)==([^\s]+) --hash=sha256:([a-f0-9]{64})", line.strip())
        if not match or match[1].lower().replace("_", "-") in entries:
            raise ValueError(f"Expected one exact hashed requirement per line: {line}")
        entries[match[1].lower().replace("_", "-")] = dict(version=match[2], sha256=match[3], line=line)
    if not SOURCES.keys() <= entries.keys() or "torch" not in entries:
        raise ValueError("Lock must include all numerical source wheels and torch")
    return entries


def download_locked_wheels(python, output, workspace, wheels):
    entries = read_lock(HERE / "requirements-build.lock")
    for name, wheel in wheels.items():
        if (entries[name]["sha256"] != wheel["sha256"]
                or entries[name]["version"] != SOURCES[name]["version"]):
            raise ValueError(f"Rebuilt {name} does not match requirements-build.lock; do not bypass hash validation")
    common = [python, "-I", "-m", "pip", "--isolated", "download", "--no-deps", "--require-hashes",
              "--only-binary=:all:", "--platform", "win_amd64", "--python-version", "312",
              "--implementation", "cp", "--abi", "cp312", "--dest", output]
    for label, names, index in (
        ("pypi", entries.keys() - SOURCES.keys() - {"torch"}, "https://pypi.org/simple"),
        ("torch", {"torch"}, "https://download.pytorch.org/whl/cpu"),
    ):
        requirements = workspace / f"{label}.txt"
        requirements.write_text("\n".join(entries[name]["line"] for name in sorted(names)) + "\n", encoding="utf-8")
        run([*common, "--index-url", index, "-r", requirements])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="Windows wheelhouse directory")
    parser.add_argument("--sources", type=Path, required=True, help="Public source archive cache")
    parser.add_argument("--builder-python", type=Path, help="Optional existing Python with the exact build versions")
    parser.add_argument("--source-wheels-only", action="store_true", help="Maintainers: rebuild the three wheels for lock review")
    args = parser.parse_args(argv)
    if sys.version_info[:2] != (3, 12):
        parser.error("Run with Python 3.12")
    output, sources = args.output.resolve(), args.sources.resolve()
    output.mkdir(parents=True, exist_ok=True)
    sources.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="hand-viewer-dependencies-") as temporary:
        workspace = Path(temporary)
        python = builder_python(workspace, args.builder_python)
        wheels = build_source_wheels(python, sources, output, workspace)
        record = dict(schema_version=1, source_date_epoch=SOURCE_DATE_EPOCH, builder=BUILD_VERSIONS,
                      sources=SOURCES, wheels=wheels)
        (output / "source-wheel-provenance.json").write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(wheels, indent=2), flush=True)
        if not args.source_wheels_only:
            download_locked_wheels(python, output, workspace, wheels)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
