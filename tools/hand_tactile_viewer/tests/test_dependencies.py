"""Source-wheel content, provenance and portable packaging guarantees."""
import importlib.util
import io
import json
from pathlib import Path
import sys
import tarfile
import zipfile

import pytest


WINDOWS = Path(__file__).parents[1] / "windows"


def load_module(name):
    spec = importlib.util.spec_from_file_location(name, WINDOWS / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def modules(monkeypatch):
    monkeypatch.syspath_prepend(str(WINDOWS))
    return load_module("prepare_dependencies"), load_module("prepare_qmt_wheel"), load_module("wheel_utils")


def test_wheel_normalization_is_repeatable_and_retains_every_content_byte(tmp_path, modules):
    _, _, utils = modules
    contents = {"package/data.py": b"# exact upstream bytes\r\n", "example.dist-info/RECORD": b"unchanged record\n"}
    paths = [tmp_path / "first.whl", tmp_path / "second.whl"]
    for index, path in enumerate(paths):
        with zipfile.ZipFile(path, "w") as package:
            names = list(contents) if index == 0 else list(reversed(contents))
            for name in names:
                entry = zipfile.ZipInfo(name, date_time=(2020 + index, 2, 3, 4, 5, 6))
                entry.create_system = index * 3
                entry.external_attr = (0o100755 if index else 0o100644) << 16
                package.writestr(entry, contents[name], compress_type=zipfile.ZIP_DEFLATED)
        utils.normalize_wheel(path)
    assert paths[0].read_bytes() == paths[1].read_bytes()
    first = paths[0].read_bytes()
    utils.normalize_wheel(paths[0])
    assert paths[0].read_bytes() == first
    with zipfile.ZipFile(paths[0]) as package:
        assert {name: package.read(name) for name in package.namelist()} == contents


@pytest.mark.parametrize("fault", [None, "modified-source", "missing-source", "missing-license"])
def test_qmt_wheel_must_retain_all_upstream_python_and_license_texts(tmp_path, modules, fault):
    _, qmt, _ = modules
    content = b"# SPDX-License-Identifier: MIT\ndef exact_upstream(): pass\n"
    source = tmp_path / "source.tar.gz"
    with tarfile.open(source, "w:gz") as archive:
        entry = tarfile.TarInfo("qmt-0.2.4/qmt/functions/quaternion.py")
        entry.size = len(content)
        archive.addfile(entry, io.BytesIO(content))
    wheel = tmp_path / "qmt.whl"
    with zipfile.ZipFile(wheel, "w") as package:
        if fault != "missing-source":
            package.writestr("qmt/functions/quaternion.py", b"modified" if fault == "modified-source" else content)
        for name in qmt.LICENSE_SHA256:
            if fault == "missing-license" and name == "MIT.txt":
                continue
            package.writestr("qmt.dist-info/licenses/LICENSES/" + name,
                             (WINDOWS / "vendor/qmt/LICENSES" / name).read_bytes())
    if fault is None:
        qmt.validate_wheel(source, wheel)
    else:
        with pytest.raises(ValueError):
            qmt.validate_wheel(source, wheel)


def test_cached_source_corruption_is_not_silently_replaced(tmp_path, modules, monkeypatch):
    dependencies, _, _ = modules
    archive = tmp_path / "qmt-0.2.4.tar.gz"
    archive.write_bytes(b"not the pinned source")
    monkeypatch.setattr(dependencies.urllib.request, "urlopen", lambda *args, **kwargs: pytest.fail("unexpected download"))
    with pytest.raises(ValueError, match="Cached source checksum mismatch"):
        dependencies.download_source("qmt", tmp_path)


def test_existing_venv_python_keeps_its_symlink_path(tmp_path, modules, monkeypatch):
    dependencies, _, _ = modules
    python = tmp_path / "venv/bin/python"
    python.parent.mkdir(parents=True)
    python.symlink_to(sys.executable)
    called = []

    def check_output(command, **kwargs):
        called.append(command)
        return json.dumps(dict(python=[3, 12], versions=dependencies.BUILD_VERSIONS))

    monkeypatch.setattr(dependencies.subprocess, "check_output", check_output)
    assert dependencies.builder_python(tmp_path, python) == python
    assert called[0][0] == str(python)


def test_source_wheel_hash_mismatch_stops_before_index_access(tmp_path, modules, monkeypatch):
    dependencies, _, _ = modules
    lock = tmp_path / "requirements-build.lock"
    lock.write_text("\n".join(f"{name}=={record['version']} --hash=sha256:{'0' * 64}"
                              for name, record in dependencies.SOURCES.items())
                    + f"\ntorch==2.8.0+cpu --hash=sha256:{'0' * 64}\n", encoding="utf-8")
    monkeypatch.setattr(dependencies, "HERE", tmp_path)
    monkeypatch.setattr(dependencies, "run", lambda *args: pytest.fail("unexpected download"))
    with pytest.raises(ValueError, match="does not match"):
        dependencies.download_locked_wheels(Path(sys.executable), tmp_path, tmp_path,
                                           {"qmt": {"sha256": "1" * 64}})


@pytest.mark.parametrize("filename", ["../outside.py", "C:/outside.py", "source/../../outside.py"])
def test_source_extraction_refuses_escaping_members(tmp_path, modules, filename):
    dependencies, _, _ = modules
    archive = tmp_path / "source.zip"
    with zipfile.ZipFile(archive, "w") as package:
        package.writestr(filename, "not source")
    with pytest.raises(ValueError, match="Invalid source archive member"):
        dependencies.unpack_source(archive, tmp_path / "unpacked")
