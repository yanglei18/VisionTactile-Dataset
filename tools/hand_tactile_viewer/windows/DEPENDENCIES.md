# Preparing the Windows build dependencies

This page is for maintainers. End users install `VisionHandMotionTouch-0.1.0-windows-x64-setup.exe`; they do not prepare Python environments, source archives, or wheel caches.

From the repository root, use Python 3.12 on Windows or Linux:

```powershell
py -3.12 tools\hand_tactile_viewer\windows\prepare_dependencies.py --output C:\build\wheels --sources C:\build\sources
```

On Linux, replace `py -3.12` with `python3.12` and use your own directory paths. This step needs internet access to the public PyPI, GitHub and PyTorch download services. It does not need a private checkout, model license login, serial device, ROS, or C++ compiler.

The script downloads and verifies three public source archives, creates a temporary isolated wheel builder, builds the three pure Python wheels, and downloads every other Windows x64 / CPython 3.12 dependency from `requirements-build.lock`. Every runtime wheel must match the lock's SHA-256. The CPU PyTorch wheel comes from its official CPU index; the other published wheels come from PyPI. The source archive cache can be retained for later builds.

Then run the actual installer build on Windows x64 with Python 3.12 and Inno Setup 6.7.3:

```powershell
tools\hand_tactile_viewer\windows\build.ps1 -Resources C:\build\hand-runtime -Wheelhouse C:\build\wheels
```

`-Resources` is the runtime bundle created by this project's `prepare` command. Model assets and the selected pinned FSGlove numerical source are separate build inputs described in the application README. The wheel preparation script does not acquire models or grant redistribution rights. With the prepared wheelhouse and resource bundle, the installer build runs without downloading Python dependencies. The finished installer runs offline.

## Fixed public sources

| Wheel | Public source | Packaging treatment |
|---|---|---|
| `manotorch 0.0.2` | [lixiny/manotorch, a2a70c591f91551078b7bb2af9b5d9f275b626e0](https://github.com/lixiny/manotorch/tree/a2a70c591f91551078b7bb2af9b5d9f275b626e0) | Original source and build script. |
| `chumpy 0.71` | [mattloper/chumpy, 580566eafc9ac68b2614b64d6f7aaa84eebb70da](https://github.com/mattloper/chumpy/tree/580566eafc9ac68b2614b64d6f7aaa84eebb70da) | Original source with upstream's Python 3.12 / NumPy fixes. PyPI's `0.70` is a different revision. |
| `qmt 0.2.4+python` | [Official QMT 0.2.4 source distribution](https://pypi.org/project/qmt/0.2.4/#files) | All 27 `qmt/**/*.py` files remain byte-identical. Only the unused native extension build is omitted. |

Archive URLs and SHA-256 values are pinned in `prepare_dependencies.py`. QMT's package dependencies and Python algorithms are retained. Its `quatProject` and `estimateDelta1d` functions are exercised by the application self-check. The native C++ estimators and C++ integration backends are absent from this packaging variant and are not used by the application.

The official QMT source distribution omits `LICENSES/`. `prepare_qmt_wheel.py` restores the exact three license texts from [QMT's v0.2.4 commit](https://github.com/dlaidig/qmt/tree/0fa8d32eb461e14d78e9ddbd569664ea59bcea19/LICENSES). Their hashes and every QMT Python source byte are verified before a wheel is accepted. Original per-file SPDX notices remain present. These notices describe upstream licensing; they do not add new permissions.

PyInstaller's [`collect_all`](https://pyinstaller.org/en/stable/hooks.html#PyInstaller.utils.hooks.collect_all) includes Python source files by default. The builder uses it for QMT so calibration provenance can read and hash the real source in a frozen installation. It additionally requires `qmt/functions/quaternion.py` and `heading_correction.py` in the frozen directory; the frozen application tests the corresponding functions.

## Rebuilding and reviewing dependency changes

Source wheels use Python 3.12, `pip 24.0`, `setuptools 78.1.0`, `wheel 0.48.0`, `packaging 26.3`, and `SOURCE_DATE_EPOCH=1700000000`. The builder environment is temporary; these backend versions are separate from the application's locked runtime dependency versions. Wheel ZIP headers, ordering, permissions and timestamps are normalized without changing any internal file or `RECORD` content. Uncompressed ZIP entries avoid cross-platform compression differences.

To regenerate only the three source wheels for maintainer review:

```powershell
py -3.12 tools\hand_tactile_viewer\windows\prepare_dependencies.py --output C:\build\review-wheels --sources C:\build\sources --source-wheels-only
```

`source-wheel-provenance.json` records source URLs, commits, archive checksums, backend versions, the fixed epoch, and resulting wheel checksums. An optional `--builder-python PATH` may reuse an existing Python 3.12 environment only when all four backend versions match exactly. The script preserves a virtual environment's executable path instead of resolving its symlink to system Python.

The normal preparation command rejects source wheel hashes that differ from `requirements-build.lock`. It never edits the lock or disables hash checks. When deliberately updating dependencies, review source provenance, update the lock explicitly, repeat source and frozen application self-checks, and perform the relevant hardware acceptance test before releasing a new installer. Checksum or source-download failures stop preparation rather than silently substituting other versions.
