# User quick start

[简体中文](quickstart.zh-CN.md) · [Workbench operation guide](../tools/capture_console/README.md)

For Ubuntu 24.04 x86_64 computers with the supplied device set. Install the application, connect the devices, and begin using it. Users do not need to configure Python, ROS, source paths or Tracker input files.

## Install

1. Open the **complete download link** supplied with the equipment, click **Download installer**, and extract the downloaded archive. No account, GitHub login, or download password is required.
2. Open **Install-VisionTactile** and click **Install**. Enter the computer's administrator password when the system asks.
3. Wait for **Installation complete**, then click **Open workbench**.

The first installation needs Internet access. It downloads and configures the runtime, camera drivers and matching Tracker dependencies automatically. The window shows progress; no individual component downloads or terminal commands are required. If it stops with an error, use **Show details** and give the message to the maintainer. Retry after the issue is resolved.

Later, open **Vision Tactile Workbench** from the application menu. The graphical installer copies the application to its managed installation directory, so the downloaded and extracted folders can be removed after successful installation.

Older source deliveries containing only `install.sh` do not provide this graphical installation workflow. Ask the maintainer for the updated delivery.

## First use

1. Connect the supplied cameras, Tracker receiver and any glove devices you need. Reconnect receivers or serial devices that were already plugged in before the first installation so that the new access rules take effect.
2. Under **Devices → Configure Devices**, confirm the enabled devices and their physical roles. Runtime paths are filled automatically; left/right hands and camera positions still need to match the actual equipment.
3. If using Trackers, open **Calibration & Offline Tools → Tracker Initialization Wizard** and follow the initialization and power-on sequence.
4. Open the required devices, check that the live previews update, and start recording.
5. After recording, use the archive to inspect and replay the data.

This delivery covers the main capture workstation, cameras and matching Trackers. Separate gesture solving, licensed hand models and specialized calibration environments are optional components prepared by the maintainer.

Completing installation means the software environment is ready. The workbench identifies devices that are disconnected or still need initialization. Camera and tactile functions can be used according to the available devices.

## Common situations

| Situation | Action |
| --- | --- |
| Installation appears slow | Initial downloads and builds take time; stay connected and check the current phase |
| An administrator password is requested | Enter the password for this computer; no GitHub token is required |
| An existing driver is incompatible | Give the message to the maintainer; avoid overwriting a working environment |
| A device is missing or assigned incorrectly | Check connections and confirm its physical role in Configure Devices |
| A Tracker has no position data | Complete the initialization wizard and use the device set supplied with this package |

The sign-in-free site is hosted at [Vision Tactile Downloads](https://yanglei18.github.io/VisionTactile-Downloads/). Keep the complete link supplied with your equipment: it includes access to the matching installer, while the public home page alone does not. The browser handles downloading and decryption automatically; no key copying or manual decryption is needed. Share the complete link only with users of the matching equipment, never in public forums or repositories.
