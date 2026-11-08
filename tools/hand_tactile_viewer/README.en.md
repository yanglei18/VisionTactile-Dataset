# Single-hand 3D displays for the left and right gloves

[简体中文](README.md) · [Installed bilingual user guide](windows/USER_GUIDE.txt) · [Repository home](../../README.md)

Both **0.2.4** applications share the 3D calibration guide, pose solver, brief pose hold and tactile display. Each application reads motion and touch from one glove through two serial ports and displays both on its own dynamic MANO hand. The two applications can be installed together. Each has its own port preferences, calibration sessions and unloaded touch references.

| Glove | Windows x64 offline installer | Start menu | Python launcher |
| --- | --- | --- | --- |
| Left | `VisionLeftHandDisplay-0.2.4-windows-x64-setup.exe` | Left Hand 3D Display | `vt-left-hand-display` |
| Right | `VisionRightHandDisplay-0.2.4-windows-x64-setup.exe` | Right Hand 3D Display | `vt-right-hand-display` |

## Install and use

Select the installer for the glove. It includes the runtime, model and tactile layout for Windows 10 version 1809 or later and Windows 11 x64 with desktop OpenGL.

1. Plug in the motion glove and tactile board. The port lists update in the background every second; **Refresh** requests an immediate scan. Updates retain selected ports and manual input and wait until an open dropdown selection is finished.
2. Click **Identify types** to label ports **Motion / Touch / Unidentified** from their data protocol. Select each matching port and click its **Connect**. Either source can be used alone. Launching and refreshing never opens a port automatically. The sources need different ports, which must not be occupied by other programs.
3. For motion, expand **Motion calibration** and complete the three captures below. Touch alone works with an explicitly labeled fixed reference hand.
4. Touch uses only a device or verified general reference for the new protocol. Without one, **Details** still shows live 20×20 raw ADC. Acquire an unloaded reference to enable relative-response colors; a matching saved reference avoids repeat zeroing.
5. Drag to rotate, scroll to zoom, or choose **Palm / Back / Reset view**. **Interface language** changes immediately without disconnecting and is saved for the next launch.
6. Close the window and wait for the ports and processing worker to be released. The installed `使用说明.txt` contains the complete bilingual instructions and troubleshooting.

The default touch baud rate is `115200`, adjustable under **Display settings**. Identification briefly reads idle ports only when requested and sends no firmware commands. Occupied ports are skipped. A type requires consecutive matching wire frames; connection status and isolated decoded samples do not establish a protocol. Disconnect an unidentified active port before identifying it. Insufficient data, busy ports and failures have separate labels. New connections wait for identification to finish; existing connections can still be disconnected.

Protocol evidence determines the type; no board serial numbers are hardcoded. USB metadata invalidates results when a device changes. Ports without a stable identity show **this scan** and must be identified again after changing devices; temporary results do not block manual connections. Results last only for this application run and do not certify calibration or signal quality. Windows still performs native driver initialization on opening. If identification is unavailable, unplug and reconnect devices separately to locate their ports. USB drivers are supplied by the manufacturer.

Automatic refresh and type labels describe current source updates. The previously exported `0.1.0` installer does not contain them and must be rebuilt for delivery.

**This branch supports only the new 44-byte row protocol and rejects obsolete 26-byte data and tactile reference profiles.** Frames contain `A5 5A | uint16 big-endian row 0..19 | 20 uint16 big-endian ADC values`. Connection requires consecutive matching frame boundaries. Each row retains its host receive time. Matrix decoding does not verify the physical finger/palm mapping. The protocol has no checksum, hardware timestamp or sequence; undetectable payload corruption remains possible.

Details includes raw ADC and resistance estimates from the reference circuit settings. Resistance is not pressure. An undefined resistance estimate does not invalidate a legal ADC sample.

## Motion calibration and display states

Secure the glove and hold each pose steady. Click the corresponding pose button to start. The left pane demonstrates the pose. The right pane indicates the source of the displayed hand. The motion calibration bar below the three buttons shows preparation and capture separately. Move to the next pose after the check mark appears.

| Step | Action |
| --- | --- |
| 1. Flat hand | Palm down, fingers straight, held still |
| 2. Tilt fingers up | Keep fingers straight; tilt the whole hand about 50° upward |
| 3. Tilt fingers down | Keep fingers straight; tilt about 50° downward along the same axis |

The right pane identifies the source of the displayed hand. It starts with a fixed reference. After the flat capture it can show the angle from flat. With a valid upward-tilt reference it can also rotate the reference hand using measured palm tilt. The fingers follow the solved pose once the three-step calibration takes effect.

The three check marks confirm that the poses were captured. The viewer then fits the palm calibration. When some sensors need another capture, the viewer lists the fingers that can continue. Click **4. Start finger calibration** to calibrate the available joints. Repeat the indicated mounting poses later to include the remaining joints. Hold all fingers straight until the open-hand reference is ready. Follow the demonstration to flex and extend the four fingers together for about eight seconds, then flex and extend the thumb tip separately for about eight seconds. The viewer skips the long-finger or thumb exercise if that group has no usable joints. The right pane shows the current valid pose. The result identifies any fingers that still need work. **Cancel capture** ends the current capture. **Reset calibration** clears the current motion calibration.

When the tilt is too small, repeat the pose named on screen. If a finger IMU moved differently from the palm, check its mounting. When the viewer reports missing IMU data or a lost orientation reference, the affected joints are temporarily unavailable. Recapture when prompted. Resetting motion calibration can retain a settled gyroscope bias estimate from healthy, stationary sensors in the same connection. Three-minute stability requires testing with the worn glove.

| Display state | Meaning |
| --- | --- |
| Motion port disconnected | The viewer shows a fixed reference hand. With only the touch port connected, valid touch readings appear on that hand |
| Waiting for IMU data | The motion port is open and the reference hand stays still. If the message persists, check the selected motion port |
| Motion calibration required or in progress | The reference hand stays visible. Once IMU data arrives, follow the three capture steps. The viewer switches to live motion when calibration is active and a valid pose arrives |
| Live motion and touch | The moving hand shows the latest valid readings for each tactile row. Presses and releases follow the touch stream |
| Waiting for touch | Valid motion continues without old tactile colors |
| Holding last pose · waiting for motion | During a brief palm data delay the hand keeps its last measured shape and palm/back colors. Touch updates on its own clock. The hold ends at most 0.5 seconds after the last pose's source timestamp |
| Motion data is stale / model unavailable | The viewer returns to the fixed reference hand. Live motion resumes automatically when a valid pose arrives |

While waiting for IMU data, calibration or motion to recover, the fixed hand can show current touch readings independently. Each point loses its color when its reading expires. Losing the touch reference clears all tactile colors.

“Valid readings 400/400 · responding 5” means all 400 points have usable readings and five exceed the response threshold. If a finger loses tracking, the joint status appears separately and the finger still displays valid touch readings.

After calibration, the tracked hand uses the same colors as the demonstration: pale mint on the palm and deeper teal on the back. The fixed reference hand is gray. Pressing changes only the contact area: **relative ADC response** after reference subtraction and noise thresholds advances from the base color through pale yellow and orange to red at full scale. Releasing to zero restores the base color. Within a touch frame, areas without valid readings appear muted blue on the tracked hand and light gray on the reference hand. Unmapped surfaces retain the base color. The default display range is 0–3000 ΔADC. Adjust the upper limit and response direction under **Display settings**. Connecting a device restores the display range saved with its matching reference.

To improve unloaded offsets after changing boards or fit, click **Zero touch reference (optional)** and keep tactile areas unloaded for about three seconds. A stable USB identity allows the reference to be saved for this device; without a unique identity, a new reference lasts for the current connection. Touch zeroing and motion calibration are independent. References saved separately by the original tactile viewer are not copied into this application's settings automatically.

Motion and touch update independently. The displayed skew is the absolute difference between their host receive times. The release does not provide Tracker world position or automatically constrain virtual fingertips to match real finger contact. A retained shape for an unavailable joint is not an accurate measurement. The GUI's approximately 60 Hz refresh target is separate from sensor sampling or full-palm scan frequency.

## Left glove IMUs and tactile layout

The left display renders the native MANO_LEFT model. These raw sensor IDs and anatomical assignments come from the [left IMU configuration](../fsglove12/src/vt_fsglove12/configs/left_from_right_initial.json).

| Finger | IMU nearer the tip | IMU nearer the palm |
| --- | --- | --- |
| Thumb | 9 distal | 10 proximal |
| Index | 7 middle | 8 proximal |
| Middle | 5 middle | 6 proximal |
| Ring | 3 middle | 4 proximal |
| Little | 1 middle | 2 proximal |

Node 11 is on the palm. Node 12 on the wrist supplies auxiliary raw data. Confirm the long-finger segment assignments, nominal 104 Hz cadence and input scales on the actual left glove. Run the three mounting poses after fitting or repositioning the sensors.

The left glove uses 44-byte tactile rows and the following orientation for its 20×20 native addresses. This matches the left-glove orientation reported for the dual-hand application. [Left tactile layout](../../ros2_ws/src/vt_tactile_capture/vt_tactile_capture/layouts/left_glove_from_right_initial.json)

| Native address direction | Direction on the left hand |
| --- | --- |
| Decreasing row | Toward the fingertips |
| Increasing column | Toward the thumb |

Anatomical coordinates are `(19 - row, column)`. Press the palm and each finger pad separately to verify their displayed regions. All 400 addresses are bound to the model. Region boundaries and individual skin positions remain candidates. Confirm three-minute stability on the worn left glove.

## Right glove hardware configuration

Resource preparation explicitly uses the existing [`right_new_imu.json`](../fsglove12/src/vt_fsglove12/configs/right_new_imu.json). This application reuses the existing FSGlove solver and functional calibration rather than introducing another algorithm.

| Position | Raw IMU IDs |
| --- | --- |
| Thumb | 1, 2 |
| Index / middle / ring / little finger | 3, 4 / 5, 6 / 7, 8 / 9, 10 |
| Palm | 11 |
| Outer wrist | 12; retained as raw input, not mapped into the finger model |

The user confirmed that IDs 3, 5, 7 and 9 are attached to the respective fingers' **middle phalanges**. The current model mapping is retained. Configuration values include `104 Hz` input/output targets, a gyro scale of `0.035 °/s/LSB`, and an acceleration scale of `0.000244 g/LSB`. Verification status remains recorded in the configuration; these values are not measurements of another glove. Changes to range, layout or sensor fit require matching configuration and fresh calibration.

## Maintainers: environment and resources

For the left application, install the five local packages shown below with this checkout's `.venv-left-hand/bin/python`. Provide a model directory containing `models/MANO_LEFT.pkl` and a new output directory.

```bash
.venv-left-hand/bin/python -m vt_hand_tactile_viewer.prepare \
  --output .runtime/left-hand --mano-assets /path/to/authorized/mano-assets \
  --fsglove-source /path/to/FSGlove \
  --config tools/fsglove12/src/vt_fsglove12/configs/left_from_right_initial.json \
  --tactile-layout ros2_ws/src/vt_tactile_capture/vt_tactile_capture/layouts/left_glove_from_right_initial.json \
  --license-file /path/to/private/MODEL_LICENSE.txt
tools/hand_tactile_viewer/start_left_hand.sh --self-check /tmp/left-hand-check.json
tools/hand_tactile_viewer/start_left_hand.sh
```

The left application stores preferences in `visiontactile/left-hand-display-row44.ini` under the user configuration directory. Its calibration sessions and tactile references are in the adjacent `left-hand-display-row44/` directory. When both applications run, assign each glove its own two ports.

End users do not perform these steps. Maintainers need Python 3.12 with Qt/OpenGL, serial support, NumPy/SciPy/VQF, CPU PyTorch, manotorch, chumpy, the FSGlove core dependencies and QMT functional dependencies **in one environment**. Installing only this package's base dependencies does not prepare a complete dynamic-model runtime.

[`windows/requirements-build.lock`](windows/requirements-build.lock) pins the complete Windows environment, including transitive dependencies and SHA-256 hashes. Locally built upstream wheels must match the lock too. The Windows builder below prepares the whole environment. For Linux source execution, add PySide6 and the shared UI packages to a complete MANO/FSGlove numerical environment. See the [FSGlove guide](../fsglove12/README.md) and its `requirements-*-tested.txt` files for that environment; those direct-dependency lists do not replace the Windows lock.

Build both applications from the same checkout. Create `.venv-right-hand` here, provision its dependencies and install the five packages below. The right launcher reads `.runtime/oct29-right`, and the left launcher reads `.runtime/left-hand`. The right launcher also accepts the complete October 29 right-hand resource bundle.

```bash
tools/hand_tactile_viewer/start_right_hand.sh
tools/hand_tactile_viewer/start_right_hand.sh --self-check /tmp/right-hand-check.json
```

The launcher verifies the source path of each project package. The right-hand application stores preferences in `visiontactile/right-hand-display-row44.ini` under the user configuration directory and calibration sessions and tactile references in the adjacent `right-hand-display-row44/` directory. The dual-hand application uses `hand-motion-touch`. Both windows can run together. To switch a physical glove between them, disconnect the serial port in the current owner before connecting in the other window. After a shared change, run both launchers with `--self-check`. A hardware change belongs in the corresponding IMU configuration, MANO model or tactile layout. Port identification distinguishes motion from touch. The operator selects the ports for the physical glove.

Install the five local packages with the right-hand interpreter:

```bash
.venv-right-hand/bin/python -m pip install -e ros2_ws/src/vt_capture_core \
  -e ros2_ws/src/vt_tactile_capture -e tools/fsglove12 \
  -e tools/capture_console -e tools/hand_tactile_viewer
.venv-right-hand/bin/python -m pip check
```

Explicit inputs are an authorized asset directory containing `models/MANO_RIGHT.pkl`, a compatible FSGlove checkout, the right-hand configuration, a general tactile reference, and a model-use notice. Keep dynamic model resources private and outside public Git. They are not gathered implicitly from developer directories. A notice records provenance; it does not itself grant redistribution rights.

```bash
python -m vt_hand_tactile_viewer.prepare \
  --output /path/to/private/hand-motion-runtime \
  --mano-assets /path/to/authorized/mano-assets \
  --fsglove-source /path/to/FSGlove \
  --config tools/fsglove12/src/vt_fsglove12/configs/right_new_imu.json \
  --license-file /path/to/private/MODEL_LICENSE.txt
```

A verified new-protocol reference can optionally be supplied with `--response-profile /path/to/private/default_response.json`. Resources and raw ADC reception work without it.

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

For a left-hand build, add `-Hand left` and pass the left resource directory to `-Resources`. The installer is written under `dist/left-hand-display-windows/release/` and installs `VisionLeftHandDisplay.exe`.

The default release directory for the right-hand 0.2.4 installer is `dist/right-hand-display-windows/release/`:

- `VisionRightHandDisplay-0.2.4-windows-x64-setup.exe`: standalone offline installer.
- Matching `.exe.sha256` and `.manifest.json`: checksum and build record.

The executable is `VisionRightHandDisplay.exe`; AppId is `VisionTactile.RightHandDisplay.Windows.x64`, separate from `VisionTactile3D`. Installation is for the current user and does not replace the original tactile tool. Settings and references are saved independently; uninstalling retains user data. Continuous raw/pose recording is off by default; only necessary calibration evidence, results and logs are retained.

## Development and verification

`window.py` handles controls and the concise bilingual UI. `composition.py` associates public motion/touch snapshots and enforces session, freshness and time boundaries. `resources.py` validates explicit resources. `app.py` assembles the existing process controller, tactile source and renderer. Algorithms, serial acquisition and reference handling stay in shared modules.

```bash
QT_QPA_PLATFORM=offscreen python -m pytest -q tools/hand_tactile_viewer/tests
```

Tests cover port exclusion, actual command acknowledgements, interruption/reconnection boundaries, display association, language changes, resource release and build inputs. Offline checks and Wine runs do not replace native Windows acceptance. Dual-port hardware, both calibrations, touch response, graphics, reconnection and sustained operation still require testing on the target computer.
