"""Compare official QMT 0.2.4 Python sources with the installed reference.

Run with the numeric environment's Python, an already downloaded official
sdist and --report PATH. Nothing is downloaded or installed. Both isolated
workers forbid every qmt.cpp import; original source is extracted temporarily
and remains byte-for-byte intact. This checks the two numerical functions used
by this application, not unused QMT orientation filters or GUI/web features.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile


SDIST_SHA256 = "27fff51c04e4cf4441764623572fa84d8f3743411e24971c44273a28db0faedc"
SOURCE_URL = "https://pypi.org/project/qmt/0.2.4/#files"


def _worker(source_root=None):
    import importlib.abc
    import importlib.metadata

    import numpy as np
    from scipy.spatial.transform import Rotation

    class NativeQMTImportForbidden(BaseException):
        # Expected numerical ValueError/AssertionError cases are captured
        # below; an attempted extension import must abort the entire check.
        pass

    class NoNativeQMT(importlib.abc.MetaPathFinder):
        def find_spec(self, fullname, path, target=None):
            if fullname == "qmt.cpp" or fullname.startswith("qmt.cpp."):
                raise NativeQMTImportForbidden(f"QMT native extension requested: {fullname}")

    if source_root is not None:
        sys.path.insert(0, str(source_root))
    sys.meta_path.insert(0, NoNativeQMT())
    import qmt
    from qmt.functions.heading_correction import estimateDelta1d

    rng = np.random.default_rng(8412)
    q = Rotation.random(100, random_state=rng).as_quat()[:, [3, 0, 1, 2]]
    axes = rng.normal(size=(100, 3))
    axes /= np.linalg.norm(axes, axis=1)[:, None]
    results = []
    for quaternion, axis in (
        (q, axes), (q, [0, 0, 1]), (q.reshape(5, 20, 4), axes.reshape(5, 20, 3)),
        (q[0], axes), (q[[0]], axes), (-q, axes), ([0, 0, 1, 0], [1, 0, 0]),
        ([1, 0, 0, 0], [0, 0, 0]), ([2, 0, 0, 0], [0, 1, 0]),
    ):
        try:
            with np.errstate(all="ignore"):
                value = qmt.quatProject(quaternion, axis, debug=False, plot=False)
            results.append({key: np.asarray(item).tolist() for key, item in value.items()})
        except Exception as exc:
            results.append([type(exc).__name__, str(exc)])
    for axis in ([0, 1, 0], [1, 0, 0], [0, 0, 1]):
        for constraint in ("1d_corr", "euler_1d", "euler_2d", "proj"):
            for start in (-3., 0., 3.):
                try:
                    with np.errstate(all="ignore"):
                        values = estimateDelta1d(q, np.roll(q, 1, axis=0),
                                                 np.array(axis), start, constraint, 5)
                    results.append([np.asarray(value).tolist() for value in values])
                except Exception as exc:
                    results.append([type(exc).__name__, str(exc)])
    print(json.dumps(dict(results=results,
        installed_reference_version=importlib.metadata.version("qmt"),
        quaternion_source_sha256=hashlib.sha256(
            Path(qmt.__file__).with_name("functions").joinpath("quaternion.py").read_bytes()).hexdigest(),
        native_loaded=[name for name in sys.modules if name.startswith("qmt.cpp")]), sort_keys=True))


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--worker":
        _worker(sys.argv[2] if len(sys.argv) > 2 else None)
        return 0
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdist", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    args = parser.parse_args()
    if hashlib.sha256(args.sdist.read_bytes()).hexdigest() != SDIST_SHA256:
        parser.error("sdist SHA-256 does not match the official pinned QMT 0.2.4 archive")
    with tempfile.TemporaryDirectory(prefix="vt-qmt-parity-") as directory:
        root = Path(directory)
        with tarfile.open(args.sdist) as archive:
            for member in archive.getmembers():
                if (not member.isfile() or not member.name.startswith("qmt-0.2.4/qmt/")
                        or not member.name.endswith(".py")):
                    continue
                relative = Path(member.name).relative_to("qmt-0.2.4")
                if ".." in relative.parts:
                    parser.error("unsafe archive member")
                path = root / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(archive.extractfile(member).read())
        command = [sys.executable, "-I", str(Path(__file__).resolve()), "--worker"]
        installed = json.loads(subprocess.check_output(command, text=True))
        pure = json.loads(subprocess.check_output([*command, directory], text=True))
    # JSON equality compares NaN/error behavior as well as finite numerical
    # results and exact shapes. Approximate equality could hide a changed path.
    expected = json.dumps(installed.pop("results"), sort_keys=True)
    actual_results = pure.pop("results")
    actual = json.dumps(actual_results, sort_keys=True)
    equal = expected == actual
    report = dict(sdist_sha256=SDIST_SHA256, source=SOURCE_URL,
                  cases=len(actual_results), exact_parity=equal,
                  native_imports_forbidden=True, installed=installed, python_source=pure,
                  installed_results_sha256=hashlib.sha256(expected.encode()).hexdigest(),
                  python_source_results_sha256=hashlib.sha256(actual.encode()).hexdigest())
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"QMT: {report['cases']} cases; exact parity: {equal}; report: {args.report}")
    return 0 if equal else 1


if __name__ == "__main__":
    raise SystemExit(main())
