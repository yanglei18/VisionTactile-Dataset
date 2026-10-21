# Private Tracker packages: deploying on another computer

[简体中文](tracker-deployment.zh-CN.md) · [Full deployment guide](deployment.md) · [Tracker publishing and acceptance](tracker-ros2-publisher.md)

A `.vttracker` package transfers the verified extended PyVUT implementation, offline Python dependencies, and one device set's bundle, role map, and approved capture together. After one import, the workstation can fill their paths without locating each source directory, environment, and private input separately.

The public GitHub repository contains only the packaging, installation, and discovery tools. Packages contain private device inputs: deliver them privately to the operators of that device set, not through GitHub or public attachments. Checksums establish internal consistency; obtain packages from a trusted deployment maintainer because installation runs the supplied software.

## Install on a new computer

The supported environment is **Ubuntu 24.04, Linux x86_64, and system Python 3.12**. Follow the [full deployment guide](deployment.md) for ROS 2, camera drivers, and the ROS workspace first. A Tracker package does not replace those prerequisites.

Keep the complete trusted project checkout: the workstation installs this tool
in editable mode, and validation reuses the checkout's Tracker identity and role
policy. Installing only the tool's wheel is not a complete deployment environment.
The independent Windows tactile viewer does not require this tool.

1. Install the system runtime libraries:

   ```bash
   sudo apt update
   sudo apt install python3-venv libhidapi-hidraw0 libhidapi-libusb0
   ```

2. Copy the supplied `.vttracker` to a regular local directory outside the repository. From the project root, install it together with the workstation:

   ```bash
   bash tools/capture_deploy/install.sh \
     --tracker-package "/absolute/path/device-set.vttracker"
   ```

   Tracker wheels install offline without contacting a Python package index. **Initial installation of the workstation and system dependencies can still require network access.** The installer does not open Trackers, pair devices, rebuild maps, or start capture.

3. Open the workstation from the desktop. Before starting a new backend, it discovers the installation, fills paths, and saves them. PyVUT, bundle, and role-map paths do not need individual configuration. The installer remembers the actual workstation Python environment; desktop launches do not require setting `VT_CONSOLE_PYTHON` again.
4. In **Devices → Configure Devices → Trackers**, check the device set, enable Tracker, then Save and Apply. Module enablement remains an operator choice; importing a package does not start hardware.
5. Open **Calibration & Offline Tools → Tracker Initialization Wizard**. It uses the Python, script, capture, and digest from the version associated with the workstation's selected bundle. Review the inputs, explicitly approve initialization, and power devices on as instructed. Once complete, start reading and check valid poses from all three Trackers.

The original Windows pairing, map, and physical role assignments must still be valid. This transfers an existing device set's runtime materials; it does not infer wrist or torso roles for new Trackers. Successful import establishes software and input validation, not real 6DOF quality, frequency, or sustained operation. Follow the [Tracker acceptance procedure](tracker-ros2-publisher.md).

## Defaults in the initialization wizard

Wait for backend preparation or configuration application to finish before opening the wizard. It resolves confirmed Python, initialization script, and bundle paths from the workstation's Tracker configuration or an installed deployment package. The capture and approved digest come only from saved local settings or a package manifest matching the selected bundle. One status line reports the filled settings or missing items, so paths need not be copied on each use.

After changing device configuration or importing a package, close and reopen the wizard, or click **Reload Configuration**. This reloads the current configuration sources, replaces unsaved window inputs, and clears initialization approval. It does not save settings, compute a digest, or start hardware. If the workstation's selected bundle has changed, reopening or reloading does not reuse the previous device set's capture and digest; only matching inputs are filled. Editing the bundle field directly clears approval, but you must still check the capture and digest yourself; the initialization script also checks that the input evidence matches. Automatic filling does not approve device writes: review the inputs and explicitly approve each initialization.

Without matching private inputs, the relevant fields stay empty and identify what is missing. Public code contains the resolution rules, not developer-machine absolute paths, dated capture filenames, or private input contents. Those materials remain in local private settings or deployment packages.

## Import from an existing workstation

Under **Devices → Configure Devices → Trackers**, click **Import Tracker Package…**. The module need not be enabled. Select the local `.vttracker`, click **Install Package**, and follow the single validation or installation status.

Installation runs in the background. On success, four fields in the current draft are filled: PyVUT source, Python dependency directory, bundle, and role map. Other fields and module enablement stay intact. Close the import window, review the draft, then **Save and Apply** to select it for the workstation. Import does not rewrite an active backend's configuration. Follow the existing procedure to finish recording and stop devices before applying.

Cancelling or closing requests cancellation and waits for cleanup while the interface remains responsive. Closing the whole workstation also waits for this installation's cleanup. Failed or cancelled installations do not activate incomplete versions or save device settings. A version that already installed successfully may remain on disk; subsequently cancelling Configure Devices discards only its form edits.

Existing valid paths are preserved. When migrating settings, a nonempty but missing private bundle or role map is not silently replaced with another device set. Use this import entry to choose the supplied package explicitly, then Save and Apply. **Detect Local Environment** rechecks installed resources; it does not install a package.

Maintainers can also install only the Tracker package:

```bash
python3 tools/tracker_deploy/manage.py install \
  --package "/absolute/path/device-set.vttracker"
```

This registers a local installation without saving a workstation draft or restarting an existing backend. New configurations discover it when a new backend starts; existing valid device selections remain intact. To switch an existing workstation explicitly, import through the GUI and save.

## Maintainers: build a delivery package

Use the original working computer with approved inputs. The workstation must have saved the extended PyVUT, bundle, and role map for one Tracker set. Initialization settings must use the same bundle and contain the approved capture path and SHA-256 digest. The public upstream PyVUT submodule is not a substitute for the extended implementation.

Build from the original extended PyVUT Git checkout. Installed versions contain
only the approved runtime files, so they cannot replace that checkout for building
another delivery package.

Prepare a dedicated wheel directory containing `hid==1.0.9`, `numpy==1.26.4`, `scapy==2.7.0`, and `PyYAML==6.0.2` compatible with Linux x86_64 / CPython 3.12. Do not mix in other versions or packages. The [package implementation](../tools/tracker_deploy/src/vt_tracker_deploy/archive.py) defines the supported versions and formats. Acquiring these wheels is a preparation step that can use a networked environment.

```bash
python3 tools/tracker_deploy/manage.py build \
  --from-workstation \
  --wheelhouse "/absolute/path/tracker-wheels" \
  --output "/absolute/path/outside-repository/device-set.vttracker"

python3 tools/tracker_deploy/manage.py inspect \
  --package "/absolute/path/outside-repository/device-set.vttracker"
```

`build` includes only an allowlist of tracked runtime source, license notices, specified dependencies, and the three private inputs, recording source revision and file digests. It does not copy the whole workstation, virtual environments, or hardware logs. `inspect` checks integrity and prints a digest, target environment, and file count without displaying private input contents or accessing hardware. Building successfully does not replace target installation and device acceptance.

## Local files and troubleshooting

Installations live under `~/.local/share/visiontactile/tracker-deployment/versions/<installation-id>/`, with an isolated Python environment and matching inputs. `current.json` changes only after installation and validation succeed. Workstation settings save concrete version paths, so installing another version does not silently switch initialization inputs for the original bundle. Private inputs and local indices use permissions restricted to the current user; do not edit files inside a version directory manually.

| Message | Action |
| --- | --- |
| Python unavailable, unsupported, or environment creation failed | Check Ubuntu 24.04, `/usr/bin/python3` version 3.12, and `python3-venv` |
| Runtime dependency unavailable | Check `libhidapi-hidraw0`, `libhidapi-libusb0`, and the packaged wheels |
| Device configuration validation failed | Ask the maintainer to verify device identities and digests across the bundle, role map, and approved capture, then rebuild matching materials |
| Existing installation modified or incomplete | Preserve it for maintainer inspection; the installer does not overwrite unknown modifications |
| Installation succeeds but no live poses appear | Complete the wizard's hardware sequence and check device permissions, connections, and all three valid data streams |

Installation errors display fixed categories without exposing private capture or bundle contents or subprocess output. Record the error category, package version, and environment when troubleshooting.
