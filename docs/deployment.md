# Deploying on a new computer

[简体中文](deployment.zh-CN.md) · [Documentation home](../README.md) · [Daily operation (Chinese)](../tools/capture_console/README.md)

Users with a complete delivery should start with the [user quick start](quickstart.md): download once, open the graphical installer, and let it prepare the environment. The component commands below are for maintainers deploying from source, troubleshooting or adding optional modules; they are not an end-user installation checklist.

This maintainer guide applies to the `source-preview-20261019` branch. It brings environment installation into one place; the individual module manuals cover configuration and operation after installation. Run the commands on the target computer in section order. If a command fails, stop and resolve the error before continuing.

**New right-hand IMU glove:** after installing the gesture environment, select the
LSM6DSR firmware version and proximal/distal layout in Devices → Configure devices →
Right hand. Saving generates a private configuration with a compatible 104 Hz output
limit. Existing custom files are preserved. Recalibrate before checking gesture
accuracy. Firmware v2 requires coordinated master/slave flashing; installing the
desktop software does not flash hardware. See the [glove guide (Chinese)](../tools/fsglove12/README.md#图形化配置与可靠性固件-v2)
and [firmware build and upgrade guide](../data_imu_stm32/README.md).

| Your use case | Installation path | Completion criterion |
| --- | --- | --- |
| Standalone Windows Tactile 3D Viewer only | Go directly to [section 1](#windows) | Select a COM port, connect, and see tactile values change with contact |
| Ubuntu interface demonstration only | Sections 2 and 3, then the demonstration in section 7 | Synthetic views labeled DEMO display correctly |
| Real capture, playback and quality checks on Ubuntu | Sections 2–8 | Live streams update, and a short recording passes inspection and archive playback |
| Add Tracker, gesture solving or calibration | Complete the base deployment, then [section 9](#modules) | Each module passes its input and operation checks |

The public repository provides source code. Private Tracker startup inputs, the extended PyVUT implementation, external FSGlove source and licensed models must be obtained separately. Camera capture, raw tactile data and 2D tactile display do not require Tracker or a MANO model.

For an existing Tracker device set, maintainers can supply a complete delivery containing **Install-VisionTactile**. It automatically installs the base runtime, pinned camera drivers, workstation and matching Tracker inputs. Older source archives containing only `install.sh` still require system and ROS preparation by a maintainer. See the [automatic installer maintenance guide](../tools/capture_deploy/README.md) to build a complete delivery, or the [Tracker deployment guide](tracker-deployment.md) to import a standalone `.vttracker`.

<a id="windows"></a>
## 1. Windows: install only the standalone tactile viewer

Use an x64 desktop running Windows 10 version 1809 or later, or Windows 11, with working desktop OpenGL. See the [Windows build guide](../tools/tactile_viewer_windows/README.en.md) for the supported system scope and build references. ROS, Python, PyTorch and the main capture application are not required.

1. Obtain `VisionTactile3D-<version>-windows-x64-setup.exe` and its `.sha256` file from your supplier. **The current GitHub preview branch contains source only, with no public installer download.** If you need an installer, a maintainer must follow the [build guide](../tools/tactile_viewer_windows/README.en.md) and provide a model authorized for distribution. A source ZIP is not directly installable.
2. In PowerShell, verify that the file's hash matches the supplied `.sha256`:

   ```powershell
   Get-FileHash -Algorithm SHA256 .\VisionTactile3D-0.5.1-windows-x64-setup.exe
   ```

3. Close any older instance, run the installer, choose Simplified Chinese or English, and complete installation for the current user.
4. Plug in the tactile board, start **Tactile 3D Viewer**, refresh and select its actual COM port, confirm the baud rate (`115200` by default), and click **Connect**.
5. Press different tactile points and confirm that the display follows the data. Drag to rotate and use the mouse wheel to zoom. Switching language should preserve the connection.
6. Click **Disconnect**, then exit. To upgrade, run the new installer. Uninstall through Windows **Settings → Apps**; user preferences are retained.

If no COM port appears, check the data cable and Device Manager. If necessary, install the driver supplied by **that device's manufacturer**; the installer does not include USB drivers. Check the graphics driver if the 3D display fails. Do not let two applications open the same serial port.

The installed `使用说明.txt` is the complete bilingual user guide, corresponding to [USER_GUIDE.txt](../tools/tactile_viewer_windows/USER_GUIDE.txt) in the repository. Colors represent raw ADC values, not pressure units; a historical frame is not a live measurement. Serial-port operation, graphics and sustained operation must still be tested on the target Windows computer.

<a id="ubuntu"></a>
## 2. Ubuntu: prepare the base environment

The reference environment is a native Ubuntu 24.04 LTS x86_64 desktop. WSL, virtual-machine USB forwarding and other distributions are not treated as accepted configurations. Initial installation requires network access, an ordinary user account and sudo access for system package installation. Do not start sensors yet.

In a Bash terminal, check the system, Python and available space:

```bash
cat /etc/os-release
uname -m
/usr/bin/python3 --version
df -h "$HOME"
test -z "${CONDA_PREFIX:-}"
test -z "${VIRTUAL_ENV:-}"
```

Expect Ubuntu `24.04`, `x86_64` and system Python `3.12`. Exit any active Conda environment or venv before deploying ROS. Builds and environments require additional space. The default storage-capacity threshold for a 300-second three-camera recording is approximately **129.73 GiB**, separate from software installation space. See the [hardware reference](hardware-reference.md) for USB and storage conditions.

```bash
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
sudo apt update
sudo apt install -y git curl ca-certificates locales acl usbutils pkexec \
  python3-venv python3-tk libegl1 libgl1 libxkbcommon0 libxcb-cursor0 fonts-noto-cjk
locale charmap
```

If the final output is not `UTF-8`, run `sudo locale-gen en_US.UTF-8`, then `export LANG=en_US.UTF-8` in the current terminal. An existing Chinese UTF-8 locale can be retained.

<a id="source"></a>
## 3. Obtain the correct source version

**For a combined program and Tracker delivery**, skip the Git clone commands in this section. From the extracted `VisionTactile-Delivery/Vision-Tactile-Dataset` directory, run the commands below, then continue to section 4. This source snapshot has no `.git`; the outer `DELIVERY_MANIFEST.json` and source `PUBLIC_SOURCE_MANIFEST.json` record its baseline and actual file digests.

```bash
export VT_REPO="$(pwd -P)"
export VT_WS="${VT_REPO}/ros2_ws"
export VT_DATA_ROOT="${HOME}/visiontactile-data"
export VT_DRIVER_WS="${HOME}/visiontactile-drivers_ws"
mkdir -p "${VT_DATA_ROOT}/config" "${VT_DATA_ROOT}/deployment"
cp "${VT_REPO}/../DELIVERY_MANIFEST.json" "${VT_DATA_ROOT}/deployment/source-delivery.json"
```

At section 7, use the outer `bash "${VT_REPO}/../install.sh"` instead of the ordinary workstation installation command to import the bundled Tracker too. Keep the extracted directory after installation.

**For source obtained directly from GitHub**, use the clone procedure below.

Run these commands from the parent directory where you want the source checkout. The branch is explicit so that cloning does not default to the preserved older `main`:

```bash
git clone --branch source-preview-20261019 --recurse-submodules \
  https://github.com/yanglei18/VisionTactile-Dataset.git
cd VisionTactile-Dataset
export VT_REPO="$(pwd -P)"
export VT_WS="${VT_REPO}/ros2_ws"
export VT_DATA_ROOT="${HOME}/visiontactile-data"
export VT_DRIVER_WS="${HOME}/visiontactile-drivers_ws"
mkdir -p "${VT_DATA_ROOT}/config" "${VT_DATA_ROOT}/deployment"
git branch --show-current
git rev-parse HEAD
git submodule status
git rev-parse HEAD > "${VT_DATA_ROOT}/deployment/source-commit.txt"
```

Expect branch `source-preview-20261019`. Save the actual commit ID for reproducibility. The pinned public PyVUT reference is not the extended implementation required for production Tracker operation. See [third-party notices](../THIRD_PARTY_NOTICES.md) when reviewing submodule licenses.

Keep existing checkouts that contain local changes and clone into a new directory; do not overwrite them with `reset --hard`. When opening a new terminal, set the path variables again: point `VT_REPO` to the actual source directory and reconstruct the other variables. Store source, data and private inputs separately.

Users who only want the interface demonstration can go directly to [section 7](#desktop), without installing ROS or camera drivers.

<a id="ros"></a>
## 4. Install ROS 2 Jazzy

This section follows the [official ROS Ubuntu Deb installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) and its [repository configuration instructions](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/_Apt-Repositories.rst). Section 2 enabled Universe; now use the official `ros2-apt-source` package to configure the signing key and package source. Do not use installation scripts of unknown origin.

```bash
mkdir -p "${VT_DATA_ROOT}/deployment/downloads"
curl --fail --location --show-error \
  https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  -o "${VT_DATA_ROOT}/deployment/downloads/ros-apt-source-release.json"
VT_ROS_APT_VERSION="$(/usr/bin/python3 -c \
  'import json,sys; print(json.load(open(sys.argv[1]))["tag_name"])' \
  "${VT_DATA_ROOT}/deployment/downloads/ros-apt-source-release.json")"
curl --fail --location --show-error \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${VT_ROS_APT_VERSION}/ros2-apt-source_${VT_ROS_APT_VERSION}.noble_all.deb" \
  -o "${VT_DATA_ROOT}/deployment/downloads/ros2-apt-source.deb"
sudo dpkg -i "${VT_DATA_ROOT}/deployment/downloads/ros2-apt-source.deb"
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools \
  python3-colcon-common-extensions python3-rosdep \
  ros-jazzy-ros2bag ros-jazzy-rosbag2-py ros-jazzy-rosbag2-storage-mcap \
  python3-yaml python3-numpy python3-opencv python3-serial fio
source /opt/ros/jazzy/setup.bash
test "$ROS_DISTRO" = jazzy
ros2 pkg prefix rosbag2_storage_mcap
```

The repository-configuration package comes from the latest official release, and its download record is retained. This does not pin the entire apt repository to a snapshot. If apt reports dependency conflicts, first check that `/etc/apt/sources.list.d/ubuntu.sources` includes `noble-updates` and `noble-backports`, and follow the official guide. Do not mix packages from different Ubuntu releases.

Initialize rosdep on first use:

```bash
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
```

Source ROS in each of two terminals, then run `ros2 run demo_nodes_cpp talker` in one and `ros2 run demo_nodes_py listener` in the other. After receiving demonstration messages, stop both with Ctrl+C. This checks ROS communication without connecting sensors.

<a id="realsense"></a>
## 5. Pin the RealSense driver versions

The project's reference wrapper is **4.58.1**. Its upstream [CMake configuration](https://github.com/realsenseai/realsense-ros/blob/4.58.1/realsense2_camera/CMakeLists.txt) requires SDK `2.58.0` or later; this guide pins SDK **2.58.1**. Upstream labels wrapper 4.58.1 as beta, so software checks and acceptance on the target hardware remain separate requirements.

This guide builds **both the SDK and wrapper from source**, without relying on old versions remaining available in apt. Do not separately install unpinned `ros-jazzy-realsense2-camera`, `ros-jazzy-librealsense2*` or `librealsense2*` packages as substitutes. If another SDK or driver is already installed, inspect it and choose one installation source before proceeding; do not overwrite an existing environment.

### 5.1 Build SDK 2.58.1

The commands below use the RSUSB compatibility backend, matching this project's RSUSB permission tool. This route does not require kernel-module changes in this guide. Upstream has not formally validated this [compatibility installation route](https://github.com/realsenseai/librealsense/blob/v2.58.1/doc/libuvc_installation.md); it therefore does not establish a guarantee of three-camera 30 Hz performance. Deploy and test alternative V4L2/DKMS routes separately using the [upstream installation instructions](https://github.com/realsenseai/librealsense/blob/v2.58.1/doc/installation.md). Do not mix the two backends.

```bash
sudo apt install -y build-essential cmake pkg-config \
  libusb-1.0-0-dev libudev-dev libssl-dev
mkdir -p "${HOME}/visiontactile-deps"
git clone --branch v2.58.1 --depth 1 \
  https://github.com/realsenseai/librealsense.git \
  "${HOME}/visiontactile-deps/librealsense-2.58.1"
cmake -S "${HOME}/visiontactile-deps/librealsense-2.58.1" \
  -B "${HOME}/visiontactile-deps/librealsense-2.58.1/build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DFORCE_RSUSB_BACKEND=ON -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=ON \
  -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_GLSL_EXTENSIONS=OFF \
  -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_WITH_DDS=OFF -DBUILD_ROSBAG2=OFF \
  -DBUILD_UNIT_TESTS=OFF -DCHECK_FOR_UPDATES=OFF
cmake --build "${HOME}/visiontactile-deps/librealsense-2.58.1/build" --parallel 2
sudo cmake --install "${HOME}/visiontactile-deps/librealsense-2.58.1/build"
sudo ldconfig
pkg-config --modversion realsense2
```

Expect `2.58.1`. This builds command-line tools, not RealSense Viewer. Section 8 checks permissions and connectivity using this project's tools, without adding a second upstream udev rule set. `--parallel 2` limits build concurrency; reduce it to `1` if memory is insufficient. The disabled DDS/ROS 2 recording features belong to the SDK itself; this project's ROS Recorder/MCAP functionality is unaffected. See the [SDK build options](https://github.com/realsenseai/librealsense/blob/v2.58.1/CMake/lrs_options.cmake) for parameter definitions.

### 5.2 Build wrapper 4.58.1

Keep the driver source in a separate workspace outside the project. Use the fixed tag; a detached HEAD message when cloning a tag is expected.

```bash
mkdir -p "${VT_DRIVER_WS}/src"
git clone --branch 4.58.1 --depth 1 \
  https://github.com/realsenseai/realsense-ros.git \
  "${VT_DRIVER_WS}/src/realsense-ros"
source /opt/ros/jazzy/setup.bash
cd "${VT_DRIVER_WS}"
rosdep install --from-paths src/realsense-ros/realsense2_camera \
  src/realsense-ros/realsense2_camera_msgs src/realsense-ros/realsense2_description \
  --ignore-src -r -y --rosdistro jazzy \
  --skip-keys librealsense2
colcon build --packages-select realsense2_camera_msgs realsense2_description \
  realsense2_camera --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DUSE_LIFECYCLE_NODE=OFF -DBUILD_ACCELERATE_GPU_WITH_GLSL=OFF
source install/setup.bash
ros2 pkg xml realsense2_camera | sed -n 's:.*<version>\(.*\)</version>.*:\1:p'
ros2 pkg prefix realsense2_camera
```

Expect `4.58.1`, with the package path under `visiontactile-drivers_ws/install`. If another version or a wrapper path under `/opt/ros/jazzy` appears, fix the environment-loading order before building the main system. The official wrapper source is at the [fixed tag](https://github.com/realsenseai/realsense-ros/tree/4.58.1).

### 5.3 Alternative for an existing apt installation

If a maintainer has already deployed matching versions, inspect package sources and available versions with `apt-cache policy ros-jazzy-realsense2-camera ros-jazzy-realsense2-camera-msgs librealsense2-dev`. Then use the `ros2 pkg xml`, `ros2 pkg prefix` and `pkg-config` checks above to verify **the versions actually loaded**.

Use `apt install package=full-version` only when the package source provides the exact compatible Debian versions, and record their full versions and origins. This guide does not invent Debian version strings. If `4.58.1` is unavailable, use the separate source-build route in sections 5.1–5.2; another `4.58.x` version is not equivalent to `4.58.1`. A maintainer should plan changes to existing working environments and package removal. Do not mix installations on an active capture computer.

<a id="build"></a>
## 6. Build the main system

Continue using system Python. Load ROS, then the driver you just built, and finally build this project:

```bash
test -z "${CONDA_PREFIX:-}"
test -z "${VIRTUAL_ENV:-}"
source /opt/ros/jazzy/setup.bash
source "${VT_DRIVER_WS}/install/setup.bash"
cd "${VT_WS}"
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy \
  --skip-keys "librealsense2 realsense2_camera realsense2_camera_msgs"
colcon build --event-handlers console_direct+
source install/setup.bash
PYTHONPATH="${VT_REPO}/tools/multisensor_alignment/src:${VT_REPO}/ros2_ws/src/vt_realsense_capture${PYTHONPATH:+:$PYTHONPATH}" \
  colcon test --event-handlers console_direct+
colcon test-result --all --verbose
```

The skipped rosdep keys are supplied by section 5; the dependencies are still required. With a verified apt-based driver installation, omit sourcing `VT_DRIVER_WS`, but retain the version checks. Do not alternate between a regular install and `--symlink-install` within the same build directory.

The main workspace's `install/setup.bash` records the driver underlay used at build time. Loading that main workspace subsequently gives the desktop backend access to the dependency chain; see [colcon's workspace chaining documentation](https://colcon.readthedocs.io/en/released/user/using-multiple-workspaces.html). Do not move or delete the driver directory. Open a new terminal, set the section 3 path variables again, and verify:

```bash
source /opt/ros/jazzy/setup.bash
source "${VT_WS}/install/setup.bash"
ros2 pkg prefix vt_capture_manager
ros2 pkg xml realsense2_camera | sed -n 's:.*<version>\(.*\)</version>.*:\1:p'
/usr/bin/python3 -c 'import rclpy,rosbag2_py,cv2,numpy,yaml; from vt_camera_msgs.msg import CameraFrameTiming; print("runtime imports OK")'
```

Tests should have no failures, and the wrapper should still be `4.58.1`. Tests and module imports do not establish hardware acceptance. See the [contributor guide](../CONTRIBUTING.md) for separate development test environments.

<a id="desktop"></a>
## 7. Install the workbench and run the demonstration first

```bash
cd "${VT_REPO}"
bash tools/capture_deploy/install.sh
bash tools/capture_deploy/start_demo.sh
```

The installer creates an isolated Qt environment and desktop launcher. It does not install ROS or camera drivers, download models or start hardware. DEMO views should render and allow switching between Chinese and English. Demonstration recording does not produce a real bag.

If you received a private Tracker package, use `bash tools/capture_deploy/install.sh --tracker-package "/absolute/path/device-set.vttracker"` instead of the installation command above. This also installs its isolated Tracker runtime and inputs; see the [package prerequisites and steps](tracker-deployment.md). Tracker wheels install offline; workstation dependency installation can still require network access.

The installer records the actual workbench Python environment in the desktop launcher. Even when installation uses a custom `VT_CONSOLE_PYTHON`, desktop launches do not require setting that variable again. Other environment variables are not automatically saved in the launcher.

Do not copy `.venv-*` directories from the old computer. ROS uses system Python, Qt uses `.venv-capture-console`, and calibration and gesture solving use their own environments. If the demonstration fails, fix fonts, Qt system libraries or graphics issues before connecting hardware.

<a id="permissions"></a>
## 8. Permissions, device configuration and the first real recording

### 8.1 Serial ports and RealSense USB

List serial ports without opening them:

```bash
/usr/bin/python3 -m serial.tools.list_ports -v
id -nG
```

If serial access is needed and your account is not in `dialout`, run `sudo usermod -aG dialout "$(id -un)"`, then log out of the desktop completely and log in again. Prefer a verified `/dev/serial/by-id/...` path. If there is no unique ID, identify the physical device and use `/dev/serial/by-path/...`. IMU and tactile boards may have identical names; do not infer their ports from names alone, or apply a global `chmod 777`.

After connecting cameras, check them while no camera process is running:

```bash
cd "${VT_REPO}"
lsusb -t
python3 tools/capture_deploy/configure_realsense_usb.py check
```

The project's permission tool covers only the reference cameras' USB product IDs. If it reports relevant permission problems or insufficient RSUSB buffering, follow the [USB configuration guide (Chinese)](../tools/capture_deploy/REALSENSE_USB.md):

```bash
pkexec /usr/bin/python3 "${VT_REPO}/tools/capture_deploy/configure_realsense_usb.py" apply
python3 tools/capture_deploy/configure_realsense_usb.py check
rs-enumerate-devices -S
```

Confirm the actual serial numbers and USB `5000M` links. The tool's 64 MiB usbfs adjustment lasts only until reboot; check again after restarting. To undo it, use `rollback` as described in the same guide. The tool neither stops processes nor opens cameras. Unsupported product IDs require the corresponding vendor's rules.

### 8.2 Enter this computer's device identities

Copy the reference configuration outside the source tree:

```bash
cp -n "${VT_REPO}/ros2_ws/src/vt_realsense_capture/config/cameras.yaml" \
  "${VT_DATA_ROOT}/config/cameras.yaml"
```

Edit the copy and enter each device's actual `serial`, `firmware` and `asic_serial` from enumeration, keeping models and roles correctly matched. **Serial numbers in the repository belong to reference devices and cannot be reused for another camera set.** For glove/tactile recording, choose the [appropriate recording configuration (Chinese)](../ros2_ws/src/vt_glove_bridge/README.md) and explicitly include the corresponding topics in the list.

<a id="local-paths"></a>
### 8.3 Default paths and local environment detection

Before starting each new local backend, the workbench checks the current source directory and common local installation locations, fills identifiable paths and saves them automatically. You do not need to open Configure Devices, run detection or manually save basic paths first. Fresh and migrated settings use the same preparation flow. Connecting to an existing backend leaves that backend's configuration unchanged.

The data output directory defaults to `~/visiontactile-data`; set `VT_DATA_ROOT` to an absolute path in the workbench's environment to choose another default. A missing default directory is created while preparing the new backend. An existing output path is preserved. Valid software paths are kept; missing software paths are repaired only when exactly one valid candidate is available.

Use **Devices → Configure Devices** to change modules, device identities or custom locations. **Detect Local Environment** remains available for a manual recheck. Manual detection changes only the form; it does not create directories or restart the backend. Review and click **Save and Apply** to submit those changes, or Cancel to discard them. Saving also creates a missing default directory. The status line shows the number of updated fields and required settings still missing from enabled modules. Hover over a field for missing-path or multiple-candidate details.

| Setting | Standard locations checked | Environment variable for a custom location |
| --- | --- | --- |
| Data output directory | `~/visiontactile-data` | `VT_DATA_ROOT` |
| Camera configuration | `config/cameras.yaml` in the data directory; otherwise the repository reference configuration is suggested | None; choose it in the form |
| Glove Python | `.venv-fsglove-120hz/bin/python`, `.venv-fsglove/bin/python`, `.venv-glove/bin/python`, or `tools/fsglove12/.venv/bin/python` under the repository; `~/.venvs/vt-fsglove/bin/python` | `VT_FSGLOVE_PYTHON` |
| FSGlove source | `external/FSGlove` under the repository; `~/visiontactile-deps/FSGlove`; `~/.local/share/visiontactile/fsglove` | `VT_FSGLOVE_SOURCE`; also accepts `FSGLOVE12_UPSTREAM_CHECKOUT`, `FSGLOVE_SOURCE` |
| MANO resources | `external/mano-assets` under the repository; `models/mano-assets` or `models/mano` under the data directory; `~/visiontactile-data/models/mano-assets`; `~/visiontactile-deps/mano-assets` | `VT_MANO_ASSETS`; also accepts `FSGLOVE12_MANO_ASSETS`, `MANO_ASSETS` |
| Extended PyVUT source | `~/.local/share/visiontactile/pyvut-src`; `external/pyvut` under the repository | `VT_PYVUT_ROOT` |
| PyVUT dependencies | `~/.local/share/visiontactile/pyvut-venv/lib/python*/site-packages` | `VT_PYVUT_SITE` for the directory directly, or `VT_PYVUT_VENV` for the virtual environment root |
| Tracker startup bundle | Matching installed `.vttracker` version; otherwise provide an explicit path outside the repository | `VT_BUNDLE` |
| Tracker role mapping | Matching installed `.vttracker` version; otherwise `tracker-validation/roles.yaml` under the data directory | `VT_ROLE_MAP` |

Environment variables fill empty fields or repair eligible software paths; they do not override a valid current selection. For aliases listed in one row, the first variable set in the listed order takes precedence. An invalid explicit location is reported as missing instead of falling back to another directory. Pass the variables to the workbench process, for example by running `bash tools/capture_deploy/start_workbench.sh` in the same terminal where they were set. Once saved, the paths do not require those variables on subsequent launches. Paths may use `~`; the application does not expand `$variables` or execute shell expressions.

Detection preserves already entered but missing camera configurations, Tracker bundles, role mappings and calibration configurations so that it cannot silently select identities for another device set. Reselect these files manually. Serial ports are enumerated without assigning a hand or an IMU/tactile role. MANO discovery checks for the matching `models/MANO_RIGHT.pkl` or `models/MANO_LEFT.pkl`. Missing dependencies, authorized models and physical devices still need preparation according to the module manuals. **Detection checks file locations and structure; it does not establish that dependencies are fully installed or hardware has passed validation.**

An installed Tracker package also supplies its matching PyVUT source and dependency paths. To replace an old explicit device selection, use **Trackers → Import Tracker Package…**, review the filled draft, then **Save and Apply**. Import itself does not change the running backend or start hardware.

<a id="first-run"></a>
### 8.4 Accept a short recording through the workbench

Close the demonstration, then run:

```bash
cd "${VT_REPO}"
bash tools/capture_deploy/start_workbench.sh
```

1. Wait for automatic path preparation and the backend-ready status. If device settings have already been saved and identities have not changed, select the required modules and start them directly; there is no need to detect or save paths again.
2. For first-time device assignment or changes, open **Devices → Configure Devices**, enable only the required modules, check camera identities and actual serial ports, then click **Save and Apply**. This reloads the **idle backend**, not the computer; finish recording and stop managed devices before a reload. See the [previous section](#local-paths) for default locations and manual rechecks.
3. Start the required devices and confirm that previews update continuously without a historical-frame notice. The absence of Tracker does not prevent recording the configured streams that are available.
4. Review the recording plan and verify the topics you need. Enabling a preview does not automatically add new topics to the Recorder list.
5. First record a test bag of approximately 10 seconds, then stop and wait for finalization. In **Archive**, inspect the topics, quality report and embedded playback. Available topics should play back.
6. If the camera target is 30 Hz, check the observed frequency and timing gaps for each actual image stream. `COMPLETE` means only that the recording process finalized normally. The tactile display's snapshot frequency is not the full-palm scan rate.
7. At the end, stop recording and managed devices before closing the workbench. Closing the window alone does not stop backend devices.

See the [workbench manual (Chinese)](../tools/capture_console/README.md) for operation details and the [capture guide](capture-guide.md) for the CLI workflow. After the short recording passes, validate the intended duration and number of devices.

<a id="modules"></a>
## 9. Add modules as needed

| Module | Additional preparation | Operation documentation |
| --- | --- | --- |
| Tracker | Windows pairing/mapping and matching runtime/private inputs, optionally supplied as a `.vttracker` package | [Package deployment](tracker-deployment.md), [publishing and acceptance](tracker-ros2-publisher.md) |
| Gesture solving | Correct serial ports, external FSGlove source, authorized MANO resources and a separate model environment | [Gesture manual (Chinese)](../tools/fsglove12/README.md) |
| 2D tactile capture | Serial permissions, actual hardware and hand assignment; no model required | [Tactile capture (Chinese)](../ros2_ws/src/vt_tactile_capture/README.md) |
| Linux tactile viewer with a fixed hand | Qt environment, tactile serial port, and an authorized static model or separate model environment | [Standalone viewer instructions (Chinese)](../tools/capture_console/README.md#独立触觉三维小窗口) |
| Camera–Tracker calibration | Stable image and pose streams, a stationary calibration board, a rigid camera–Tracker assembly and a separate calibration environment | [Calibration installation and operation (Chinese)](../tools/tracker_camera_calibration/README.md) |
| Offline alignment | A finalized bag, matching identities/extrinsics and a separate alignment environment | [Offline alignment (Chinese)](../tools/multisensor_alignment/README.md) |

The main workbench's post-recording quality checks use system Python to load the repository's alignment module; sections 4 and 6 provide its dependencies. The standalone calibration/alignment CLIs instead use `.venv-calibration` and `.venv-alignment`, following their respective manuals. Do not install those dependencies into the Qt environment.

When transferring extrinsics, check physical device identities, rigid mounting relationships and the map. Changing computers alone does not necessarily require recalibration, but results cannot simply be reused after changing a device, mounting relationship or map.

<a id="upgrade"></a>
## 10. Record the environment, upgrade and migrate

After initial acceptance, save records in the data directory:

```bash
if [[ -f "${VT_REPO}/../DELIVERY_MANIFEST.json" ]]; then
  cp "${VT_REPO}/../DELIVERY_MANIFEST.json" "${VT_DATA_ROOT}/deployment/source-delivery.json"
else
  git -C "${VT_REPO}" rev-parse HEAD > "${VT_DATA_ROOT}/deployment/source-commit.txt"
fi
git -C "${VT_DRIVER_WS}/src/realsense-ros" rev-parse HEAD \
  > "${VT_DATA_ROOT}/deployment/realsense-ros-commit.txt"
git -C "${HOME}/visiontactile-deps/librealsense-2.58.1" rev-parse HEAD \
  > "${VT_DATA_ROOT}/deployment/librealsense-commit.txt"
dpkg-query -W > "${VT_DATA_ROOT}/deployment/ubuntu-packages.txt"
"${VT_REPO}/.venv-capture-console/bin/python" -m pip freeze \
  > "${VT_DATA_ROOT}/deployment/console-packages.txt"
```

With apt-installed drivers, record full installed package versions instead of commits for driver source directories that do not exist. Retain short-recording quality results, device firmware, USB topology and actual configuration. These are local deployment records and should not be uploaded to the public repository.

Before upgrading, stop recording, devices and the backend normally, then back up desktop mode's `~/.config/visiontactile/workbench/`, the actual state directory used by any standalone backend, and data configurations. Back up the database after stopping the backend, or use a consistent SQLite backup procedure; do not omit an active WAL. See the [workbench manual (Chinese)](../tools/capture_console/README.md) for paths.

With a clean working tree, use `git pull --ff-only` to update this branch. Preserve and resolve local changes first; do not force an overwrite. As required by the changes, rebuild ROS, rerun the desktop installer and update tool environments, then repeat the short-recording checks. After moving directories or changing computers, recreate venvs and desktop launchers; do not copy virtual environments. Before the next new local backend starts, the application automatically fills missing fields and repairs identifiable software paths while preserving valid settings and device identities. Open Configure Devices only for unresolved settings or changes to device identities, serial ports or output locations; Detect Local Environment is available for manual rechecks. See [default paths and local environment detection](#local-paths) for the selection rules.

<a id="troubleshooting"></a>
## 11. Common first-deployment problems

| Symptom | Next step |
| --- | --- |
| New features are missing after cloning | Run `git branch --show-current`; expect this guide's branch, and verify the commit |
| `ros-jazzy-*` packages cannot be found | Check Noble/Universe and the official ROS apt source; section 4 |
| Wrapper version is not 4.58.1 | Check the driver source, `ros2 pkg prefix` and environment-loading order; sections 5–6 |
| Build cannot find librealsense2 | Run `pkg-config --modversion realsense2`; confirm SDK installation completed and `ldconfig` ran |
| `xcb`, EGL or Chinese-font errors | Install the Qt system libraries in section 2 and check the desktop graphics driver |
| Settings are still missing after automatic preparation | Open Configure Devices and inspect field tooltips; install missing dependencies or confirm device identities and ports, then manually recheck and save if needed; section 8.3 |
| Serial permission denied | Check `dialout` and the active login session; changing groups without logging out of the desktop is insufficient |
| Camera is enumerated but cannot stream | Rule out another process holding the device, then check RSUSB permissions/buffers, USB links and actual identities |
| Tactile or image preview shows a historical frame | Current data is no longer fresh; check whether the source continues producing data, and do not treat cached frames as live |
| Recording completes with missing-stream/frame-drop warnings | Inspect the frozen plan, per-topic frequencies and logs; existing data can still be played in Archive |

See [troubleshooting](troubleshooting.md) and [workbench troubleshooting (Chinese)](../tools/capture_console/README.md#故障处理) for full diagnostics.

Keep the [Chinese version](deployment.zh-CN.md) synchronized when maintaining this guide. These installation commands have been checked against the source entry points and official dependency documentation. Actual installation, graphics and hardware performance on a new computer still require the acceptance steps in this guide.
