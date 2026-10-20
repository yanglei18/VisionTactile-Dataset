# Tactile 3D Viewer Windows installer

[简体中文](README.md) · [New-computer deployment guide](../../docs/deployment.md) · [Bilingual user guide (source)](USER_GUIDE.txt)

**This branch publishes source code; it does not provide a publicly downloadable `.exe` installer or static hand model.** End users should obtain an installer authorized for distribution from their supplier. Maintainers building an installer need the build environment and model inputs described below. The [deployment guide](../../docs/deployment.md) provides separate paths for the complete capture system and the standalone Windows viewer.

This directory maintains the standalone tactile viewer's Windows x64 build process. The installer includes the runtime and static right-hand model. End users do not need Python, ROS, PyTorch or model-path configuration. Installation and use work offline. The application installs under the current user's directory by default and provides Simplified Chinese and English installer/application interfaces, desktop and Start menu shortcuts, upgrades and uninstallation.

The installed application provides serial-port selection, connect/disconnect controls, language selection and tactile visualization on a fixed right-hand shape. The frozen package excludes the shared preview component's main-application settings, camera preview, gesture solver and capture-core modules. Resources use an explicit collection list that excludes other gesture tools' configuration and launchers. The build inspects the module archive and resource directory and stops if unexpected application modules are included.

The target is Windows 10 version 1809 or later, or Windows 11, on x64 with working desktop OpenGL. The minimum OS version follows the [Qt 6.11 support matrix](https://doc.qt.io/qt-6.11/supported-platforms.html); this does not replace this application's acceptance testing on Windows hardware. The build scripts require Windows x64 Python. Linux maintainers may run the same Windows toolchain in an isolated Wine environment; record Wine checks separately from native Windows hardware acceptance.

## Use and language selection

The installer offers Simplified Chinese and English. Its default follows the Windows interface language; upgrades default to the previous installation language. Installer language controls installation pages, shortcuts and the uninstall entry. Changing it updates this product's known shortcuts without creating duplicate Chinese and English desktop entries.

The application's language selector changes the interface immediately, preserves the connection and data, and remembers the selection. The first launch follows the system language; later launches use the saved preference. Installation does not overwrite the application's language preference, so installer and application languages can differ. `--language zh_CN` or `--language en` overrides the language for that launch only.

Start **Tactile 3D Viewer**, select the tactile board's COM port, confirm the baud rate (115200 by default), and click **Connect**. Refreshing the port list does not connect a device. Do not open the same port in the main capture application or another tool at the same time. Drag to rotate the model and use the mouse wheel to zoom. Disconnect or close the window to release the port. Reconnect manually after unplugging the device or a read error.

Colors on the fixed right-hand shape represent raw ADC values, not calibrated pressure or estimated finger poses. The first valid data sets a color range with a minimum span of 100 ADC; the range stays fixed until reconnection clears the old frame and scale. Status-line Hz measures distinct source snapshots delivered to the display, not hardware scans or simultaneous samples of all 400 points. Each block remains valid for display for 100 ms; a frame retained after interruption is marked as historical. See the [bilingual user guide](USER_GUIDE.txt) for display semantics and troubleshooting.

Close the viewer before upgrading. Uninstall through Windows Settings; user preferences are retained. The source guide is named `USER_GUIDE.txt`; its installed filename is **`使用说明.txt`**. That file contains both languages.

## Prepare the build machine

Install the following only on a maintainer's Windows build machine:

1. CPython 3.12 x64, including the Python Launcher (`py`).
2. [Inno Setup 6.7.3](https://jrsoftware.org/isdl.php). The pinned Simplified Chinese installer text is stored in this directory's `vendor/` folder and does not depend on language files installed with the compiler. See that folder for provenance and licensing.
3. This repository's source, a static right-hand JSON model explicitly authorized for the intended build/distribution, and its license text.

`requirements-build.lock` pins every Python build dependency and its Windows wheel SHA-256. Each build creates an isolated virtual environment. It does not activate or copy a developer environment, change the system PATH or connect hardware. PyInstaller collects PySide6's dynamic libraries; the build does not statically link Qt. The installer script uses UTF-8 without a BOM, which Inno Setup has [supported since 6.3](https://jrsoftware.org/ishelp/topic_unicode.htm).

The compiler version is read using a probe script and the official `VER` preprocessor variable, avoiding reliance on a PE file version that may be `0.0.0.0`. The Chinese installer text and upstream license have fixed SHA-256 checks. `vendor/.gitattributes` preserves upstream bytes so cross-platform line-ending conversion cannot invalidate those checks.

## Model resources

The build requires explicit `--model-bundle` and `--model-license` inputs. Models are not committed with the source, collected from developer directories or downloaded automatically. The license text provides a traceable record; **it does not itself grant commercial-use or redistribution rights**. MANO-derived meshes remain subject to their applicable license. Publishers must supply resources authorized for their distribution scope.

The static model uses the shared preview's schema version 1 (`neutral_geometry` and `model_axes`), with 778 vertices and 21 joints. Other topologies are not interchangeable without extending the shared mapping and validation interfaces.

An existing, authorized MANO environment can export the static model. For example, on a Linux development machine with that environment already configured:

```bash
.venv-fsglove-120hz/bin/python -I -m vt_fsglove12.tactile_preview_export \
  --mano-assets /path/to/authorized/mano-assets --hand right \
  --output /path/to/private-build-inputs/right_hand.json
```

This is a maintainer-only export step. The installed application loads the JSON directly and requires no second Python environment, PyTorch or original `.pkl` file.

## Build with one command

Run from the repository root in Windows PowerShell:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\tactile_viewer_windows\build.ps1 `
  -ModelBundle C:\private-build-inputs\right_hand.json `
  -ModelLicense C:\private-build-inputs\MODEL_LICENSE.txt
```

`ExecutionPolicy Bypass` applies only to that PowerShell process; it does not change the system execution policy. If local scripts are already permitted, run `build.ps1` directly. The Python entry point is also available:

```powershell
py -3.12 tools\tactile_viewer_windows\build.py `
  --model-bundle C:\private-build-inputs\right_hand.json `
  --model-license C:\private-build-inputs\MODEL_LICENSE.txt
```

Use `-Iscc` or `--iscc` for a nondefault Inno Setup location. `-Output` or `--output` changes the output directory. Paths may contain spaces or Chinese characters. Arguments are passed as arrays, not assembled into shell command strings.

The build performs input validation, isolated dependency installation, installation of four local Python packages, `pip check`, the source application's self-check, PyInstaller `onedir` freezing, frozen-package audit and self-check, license collection and Inno Setup compilation. A failed check stops the build; incomplete directories are not packaged. Build logs remain in a separate `build-*` directory.

The default output directory, ignored by Git, is `dist/tactile-viewer-windows/release/`:

- `VisionTactile3D-<version>-windows-x64-setup.exe`: the offline installer for users.
- Matching `.exe.sha256`: the installer's SHA-256.
- Matching `.manifest.json`: model/license, dependency-lock and installed-file checksums, plus frozen self-check results.

Users need only the installer `.exe`. Source code, build scripts, wheel caches and build-project archives are maintenance materials, not directly installable products.

The standalone product version comes only from `version` in this directory's `product.toml`, currently `0.5.1`; it is independent of the main capture application's version. A stable AppId identifies upgrades to the same application. Preferences live in the user directory, which uninstallation does not recursively delete. Rebuilding the same version replaces that version's output files. Intermediate build directories remain for diagnosis and may be cleaned up by maintainers as needed.

Upgrades rely on Inno Setup's installed-file records. Old dependency files removed by a new version may remain until uninstallation; this version does not recursively clear the installation directory. Versioned runtime directories could provide further upgrade cleanup in a future implementation.

## Build without network access

End-user installation and use require no network connection. The build machine installs pinned dependencies from PyPI by default, or can use a prepared Windows wheel cache:

```powershell
py -3.12 -m pip download --only-binary=:all: `
  -r tools\tactile_viewer_windows\requirements-build.lock -d C:\build-wheels

py -3.12 tools\tactile_viewer_windows\build.py `
  --model-bundle C:\private-build-inputs\right_hand.json `
  --model-license C:\private-build-inputs\MODEL_LICENSE.txt `
  --wheelhouse C:\build-wheels
```

`--wheelhouse` enforces `--no-index`. Local package builds disable build isolation to avoid implicitly downloading additional build dependencies. When preparing wheels from another platform, add `--platform win_amd64 --python-version 312 --implementation cp --abi cp312`. Dependency upgrades require updating the lock file as a whole and repeating frozen-package and hardware acceptance checks.

## Verification scope and release acceptance

Run platform-independent build tests:

```bash
.venv-capture-console/bin/python -m pytest -q tools/tactile_viewer_windows/tests
```

Validate real model inputs on Linux without generating an installer:

```bash
.venv-capture-console/bin/python tools/tactile_viewer_windows/build.py \
  --model-bundle /path/to/private-build-inputs/right_hand.json \
  --model-license /path/to/private-build-inputs/MODEL_LICENSE.txt --check-inputs
```

The frozen program's `VisionTactile3D.exe --self-check C:\temp\tactile-check.json` checks the model, tactile mapping, Qt/OpenGL imports and theme/language resources. It does not open a serial port or create a graphics context, so it does not replace desktop rendering or hardware checks. Before delivery, test on a clean Windows machine with no Python or ROS installed:

- Offline installation, a Chinese installation path, ordinary-user launch and a fixed hand model displayed without model-environment configuration.
- Both installer/application languages, immediate language switching and persistence after restart. Upgrades preserve application language; changing installer language leaves no duplicate shortcuts.
- A real COM connection, changing tactile values, historical-frame notices after unplugging or disconnecting, reconnection and serial-port release on close.
- Desktop OpenGL rendering, rotation and zoom, with a compatible graphics driver.
- Same-AppId upgrade, relaunch, uninstallation and shortcut cleanup, with user settings retained.

The installer does not install USB drivers; whether a device uses a built-in Windows serial driver depends on the hardware. Builds are unsigned by default. A publisher may sign the executable and installer through its organizational signing process, then regenerate the SHA-256 for the final signed file.

Maintenance references: [PyInstaller options](https://pyinstaller.org/en/stable/usage.html), [non-administrative installation](https://jrsoftware.org/ishelp/topic_admininstallmode.htm), [Inno Setup's official Chinese translation](https://jrsoftware.org/files/istrans/).
