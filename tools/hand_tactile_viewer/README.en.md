# Hand Motion and Touch Viewer

[简体中文](README.md) · [Installed bilingual user guide](windows/USER_GUIDE.txt) · [Repository home](../../README.md)

Standalone application **0.1.0** reads a motion glove and tactile device through two serial ports and displays both on **one moving right-hand model**. It uses the main application's theme and supports English and Chinese. The original [fixed-hand tactile viewer 0.6.2](../tactile_viewer_windows/README.en.md) remains available, with its own installation identity, shortcuts and settings.

## Install and use

Run the supplied `VisionHandMotionTouch-0.1.0-windows-x64-setup.exe`, then open **Hand Motion and Touch Viewer**. A complete installer includes the runtime and matching resources; users do not install Python, ROS or the main workstation, or enter model paths. The target is Windows 10 version 1809 or later, or Windows 11 x64, with working desktop OpenGL. The public repository supplies source and build tooling; maintainers prepare the model and reference resources for the intended delivery.

1. Plug in the motion glove and tactile board. The port lists update in the background every second; **Refresh** requests an immediate scan. Updates retain selected ports and manual input and wait until an open dropdown selection is finished.
2. Click **Identify types** to label ports **Motion / Touch / Unidentified** from their data protocol. Select each matching port and click its **Connect**. Either source can be used alone. Launching and refreshing never opens a port automatically. The sources need different ports, which must not be occupied by other programs.
3. For motion, expand **Motion calibration** and complete the three captures below. Touch alone works with an explicitly labeled fixed reference hand.
4. Touch uses the bundled general reference, with priority for a reference saved locally for this device. Compatible boards using the same protocol and right-hand 20×20 address layout do not require zeroing when changing computers or boards.
5. Drag to rotate, scroll to zoom, or choose **Palm / Back / Reset view**. **Interface language** changes immediately without disconnecting and is saved for the next launch.
6. Close the window and wait for the ports and processing worker to be released. The installed `使用说明.txt` contains the complete bilingual instructions and troubleshooting.

The default touch baud rate is `115200`, adjustable under **Display settings**. Identification briefly reads idle ports only when requested and sends no firmware commands. Occupied ports are skipped. A type requires consecutive matching wire frames; connection status and isolated decoded samples do not establish a protocol. Disconnect an unidentified active port before identifying it. Insufficient data, busy ports and failures have separate labels. New connections wait for identification to finish; existing connections can still be disconnected.

Protocol evidence determines the type; no board serial numbers are hardcoded. USB metadata invalidates results when a device changes. Ports without a stable identity show **this scan** and must be identified again after changing devices; temporary results do not block manual connections. Results last only for this application run and do not certify calibration or signal quality. Windows still performs native driver initialization on opening. If identification is unavailable, unplug and reconnect devices separately to locate their ports. USB drivers are supplied by the manufacturer.

Automatic refresh and type labels describe current source updates. The previously exported `0.1.0` installer does not contain them and must be rebuilt for delivery.

**Current compatibility: tactile acquisition uses the existing 26-byte protocol; the observed 44-byte row protocol is not yet supported.** Opening a port and receiving bytes do not establish correct decoding. Scattered colors from an unidentified device do not establish valid tactile samples. Devices using 44-byte frames require protocol support and verification of the cell ordering before normal tactile use. This version is a source development preview.

## Motion calibration and display states

Put on and secure the right-hand glove. Select each pose in order, settle, and click **Capture this pose**:

| Step | Action |
| --- | --- |
| 1. Flat hand | Palm down, fingers straight, held still |
| 2. Tilt fingers up | Keep fingers straight; tilt the whole hand about 50° upward |
| 3. Tilt fingers down | Keep fingers straight; tilt about 50° downward along the same axis |

Calibration is computed after all three captures. Wait for **Motion calibration is active** before checking motion. Optional **Whole-hand flexion calibration** uses the existing functional calibration: follow the status, first holding fingers straight and then flexing them slowly. Status reports actual execution; a submitted request or a partial update does not mean every finger succeeded. **Cancel capture** ends the current capture, and **Reset calibration** clears the current motion calibration. Recalibrate after changing sensor fit.

| Display state | Meaning |
| --- | --- |
| Fixed reference hand · not measured motion | Motion is disconnected; touch can be viewed on the fixed hand |
| Motion calibration required | Motion is connected; complete the three captures |
| Live motion and touch | Current valid motion and nearby touch samples appear on one model |
| Waiting for touch / matching touch | Valid motion continues without old tactile colors |
| Motion data is stale / model unavailable | The viewer waits for valid motion instead of presenting historical geometry as current |

Colors show **relative ADC response** after reference subtraction and noise thresholds, transitioning smoothly from light green to dark red. They are not pressure units. Points without valid data or reference values are gray; unmapped surfaces keep the base color. **Display settings** offers a fixed range starting at zero and a response direction; the range does not auto-scale with applied force.

To improve unloaded offsets after changing boards or fit, click **Zero touch reference (optional)** and keep tactile areas unloaded for about three seconds. A stable USB identity allows the reference to be saved for this device; without a unique identity, a new reference lasts for the current connection. Touch zeroing and motion calibration are independent. References saved separately by the original tactile viewer are not copied into this application's settings automatically.

Streams are associated using host receive time; this is **not hardware synchronization**. The release does not provide Tracker world position or automatically constrain virtual fingertips to match real finger contact. A retained shape for an unavailable joint is not an accurate measurement. The GUI's approximately 60 Hz refresh target is separate from sensor sampling or full-palm scan frequency.

## Hardware configuration

Resource preparation explicitly uses the existing [`right_new_imu.json`](../fsglove12/src/vt_fsglove12/configs/right_new_imu.json). This application reuses the existing FSGlove solver and functional calibration rather than introducing another algorithm.

| Position | Raw IMU IDs |
| --- | --- |
| Thumb | 1, 2 |
| Index / middle / ring / little finger | 3, 4 / 5, 6 / 7, 8 / 9, 10 |
| Palm | 11 |
| Outer wrist | 12; retained as raw input, not mapped into the finger model |

The user confirmed that IDs 3, 5, 7 and 9 are attached to the respective fingers' **middle phalanges**. The current model mapping is retained. Configuration values include `104 Hz` input/output targets, a gyro scale of `0.035 °/s/LSB`, and an acceleration scale of `0.000244 g/LSB`. Verification status remains recorded in the configuration; these values are not measurements of another glove. Changes to range, layout or sensor fit require matching configuration and fresh calibration.

## Maintainers: environment and resources

End users do not perform these steps. Maintainers need Python 3.12 with Qt/OpenGL, serial support, NumPy/SciPy/VQF, CPU PyTorch, manotorch, chumpy, the FSGlove core dependencies and QMT functional dependencies **in one environment**. Installing only this package's base dependencies does not prepare a complete dynamic-model runtime.

[`windows/requirements-build.lock`](windows/requirements-build.lock) pins the complete Windows environment, including transitive dependencies and SHA-256 hashes. Locally built upstream wheels must match the lock too. The Windows builder below prepares the whole environment. For Linux source execution, add PySide6 and the shared UI packages to a complete MANO/FSGlove numerical environment. See the [FSGlove guide](../fsglove12/README.md) and its `requirements-*-tested.txt` files for that environment; those direct-dependency lists do not replace the Windows lock.

From the repository root, in the prepared numerical environment, install the five local packages:

```bash
python -m pip install -e ros2_ws/src/vt_capture_core \
  -e ros2_ws/src/vt_tactile_capture -e tools/fsglove12 \
  -e tools/capture_console -e tools/hand_tactile_viewer
python -m pip check
```

Explicit inputs are an authorized asset directory containing `models/MANO_RIGHT.pkl`, a compatible FSGlove checkout, the right-hand configuration, a general tactile reference, and a model-use notice. Keep dynamic model resources private and outside public Git. They are not gathered implicitly from developer directories. A notice records provenance; it does not itself grant redistribution rights.

```bash
python -m vt_hand_tactile_viewer.prepare \
  --output /path/to/private/hand-motion-runtime \
  --mano-assets /path/to/authorized/mano-assets \
  --fsglove-source /path/to/FSGlove \
  --config tools/fsglove12/src/vt_fsglove12/configs/right_new_imu.json \
  --response-profile /path/to/private/default_response.json \
  --license-file /path/to/private/MODEL_LICENSE.txt
```

The output directory must not already exist. Preparation exports neutral geometry, model axes and rest data from the actual model, and includes the dynamic MANO asset, required upstream files, configuration and tactile reference with a checksum manifest. Both runtime and builder validate the resource bundle without depending on the original computer's absolute paths.

```bash
python -m vt_hand_tactile_viewer --resources /path/to/private/hand-motion-runtime
python -m vt_hand_tactile_viewer --resources /path/to/private/hand-motion-runtime \
  --self-check /path/to/check-report.json
```

Use `--language zh_CN` or `--language en` to set and save the interface language. Self-check loads the real model and tactile mapping and starts and stops the compute child process. It opens no serial ports and does not verify a desktop OpenGL context or hardware performance.

## Windows build and delivery

The build computer needs Windows x64 Python 3.12 and Inno Setup 6.7.3. Building uses a complete wheelhouse and explicitly prepared resources; end users need neither tool.

From the repository root in PowerShell:

```powershell
py -3.12 tools\hand_tactile_viewer\windows\prepare_dependencies.py `
  --output C:\build\wheels --sources C:\build\sources

powershell -NoProfile -ExecutionPolicy Bypass `
  -File .\tools\hand_tactile_viewer\windows\build.ps1 `
  -Resources C:\private\hand-motion-runtime -Wheelhouse C:\build\wheels
```

Initial dependency preparation needs internet access for pinned public sources and Windows wheels and can also run on Linux Python 3.12. See the [complete dependency preparation guide](windows/DEPENDENCIES.md) for sources, checksums and rebuilding. This step does not acquire model assets or require a C++ compiler. `--wheelhouse` makes the release build install locked dependencies locally. The builder creates an isolated environment, runs `pip check`, source and frozen self-checks and distribution audits, then creates the installer. Use `-Output` for another output directory or `-Iscc` for a compiler outside its default location.

The default release directory is `dist/hand-tactile-viewer-windows/release/`:

- `VisionHandMotionTouch-0.1.0-windows-x64-setup.exe`: standalone offline installer.
- Matching `.exe.sha256` and `.manifest.json`: checksum and build record.

The executable is `VisionHandMotionTouch.exe`; AppId is `VisionTactile.HandMotionTouch.Windows.x64`, separate from `VisionTactile3D`. Installation is for the current user and does not replace the original tactile tool. Settings and references are saved independently; uninstalling retains user data. Continuous raw/pose recording is off by default; only necessary calibration evidence, results and logs are retained.

## Development and verification

`window.py` handles controls and the concise bilingual UI. `composition.py` associates public motion/touch snapshots and enforces session, freshness and time boundaries. `resources.py` validates explicit resources. `app.py` assembles the existing process controller, tactile source and renderer. Algorithms, serial acquisition and reference handling stay in shared modules.

```bash
QT_QPA_PLATFORM=offscreen python -m pytest -q tools/hand_tactile_viewer/tests
```

Tests cover port exclusion, actual command acknowledgements, interruption/reconnection boundaries, display association, language changes, resource release and build inputs. Offline checks and Wine runs do not replace native Windows acceptance. Dual-port hardware, both calibrations, touch response, graphics, reconnection and sustained operation still require testing on the target computer.
