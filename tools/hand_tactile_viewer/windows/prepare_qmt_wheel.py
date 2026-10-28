"""Build the pinned QMT Python functions without unused native estimators.

All qmt/**/*.py source bytes are retained. This application uses quatProject
and estimateDelta1d; neither needs qmt.cpp. The +python version identifies the
packaging variant explicitly. Madgwick/Mahony/IMU C++ estimators and C++ gyro
integration are not provided by this wheel and are not application backends.
"""
from pathlib import Path
import argparse
import hashlib
import os
import shutil
import subprocess
import sys
import tarfile
import tempfile
import zipfile

from wheel_utils import SOURCE_DATE_EPOCH, normalize_wheel

SOURCE_SHA256 = "27fff51c04e4cf4441764623572fa84d8f3743411e24971c44273a28db0faedc"
LICENSE_SHA256 = {
    "MIT.txt": "b85dcd3e453d05982552c52b5fc9e0bdd6d23c6f8e844b984a88af32570b0cc0",
    "CC0-1.0.txt": "a2010f343487d3f7618affe54f789f5487602331c0a8d03f49e9a7c547cf0499",
    "LicenseRef-Unspecified.txt": "71dff3080b463d827070e84acd3aea5c07a944d3550b9f874e8d8e4c3effaa6c",
}
HERE = Path(__file__).resolve().parent

SETUP = '''from setuptools import setup, find_packages
setup(name="qmt", version="0.2.4+python", description="QMT original Python functions; native estimators excluded",
      license="MIT", url="https://github.com/dlaidig/qmt", packages=find_packages(include=["qmt", "qmt.*"]),
      include_package_data=True, python_requires=">=3.8", license_files=["LICENSES/*.txt"],
      install_requires=["numpy", "scipy>=1.8.0", "matplotlib", "PyYAML", "transplant>=0.8.11",
                        "aiohttp>=3.8.1", "aiofiles", "orjson", "qasync", "vqf>=2.0.0"])
'''


def build(source, output):
    source, output = Path(source).resolve(), Path(output).resolve()
    if hashlib.sha256(source.read_bytes()).hexdigest() != SOURCE_SHA256:
        raise ValueError("QMT source does not match the official 0.2.4 source distribution")
    output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="qmt-python-wheel-") as temporary:
        with tarfile.open(source) as archive:
            archive.extractall(temporary, filter="data")
        root = Path(temporary) / "qmt-0.2.4"
        # Upstream's sdist omits LICENSES/. Restore exact files from its v0.2.4
        # commit; source SPDX copyright notices remain untouched in every .py.
        (root / "LICENSES").mkdir(exist_ok=True)
        for name, digest in LICENSE_SHA256.items():
            license_path = HERE / "vendor/qmt/LICENSES" / name
            if hashlib.sha256(license_path.read_bytes()).hexdigest() != digest:
                raise ValueError(f"QMT upstream license checksum mismatch: {name}")
            shutil.copyfile(license_path, root / "LICENSES" / name)
        script = root / "build_python_functions.py"
        script.write_text(SETUP, encoding="utf-8")
        environment = dict(os.environ, SOURCE_DATE_EPOCH=str(SOURCE_DATE_EPOCH))
        subprocess.run([sys.executable, str(script), "bdist_wheel", "--dist-dir", str(output)],
                       cwd=root, env=environment, check=True)
        wheel = output / "qmt-0.2.4+python-py3-none-any.whl"
        validate_wheel(source, wheel)
        normalize_wheel(wheel)
        return wheel


def validate_wheel(source, wheel):
    """Reject missing/modified Python files or missing upstream licenses."""
    with tarfile.open(source) as archive, zipfile.ZipFile(wheel) as package:
        expected = {item.name.removeprefix("qmt-0.2.4/"): archive.extractfile(item).read()
                    for item in archive if item.isfile() and item.name.startswith("qmt-0.2.4/qmt/")
                    and item.name.endswith(".py")}
        actual = {name for name in package.namelist() if name.startswith("qmt/") and name.endswith(".py")}
        if not expected or set(expected) != actual or any(package.read(name) != data for name, data in expected.items()):
            raise ValueError("QMT wheel must preserve every original qmt Python file byte for byte")
        for name, digest in LICENSE_SHA256.items():
            matches = [path for path in package.namelist() if ".dist-info/" in path and path.endswith("/" + name)]
            if len(matches) != 1 or hashlib.sha256(package.read(matches[0])).hexdigest() != digest:
                raise ValueError(f"QMT wheel is missing the original upstream license: {name}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    build(args.source, args.output)
