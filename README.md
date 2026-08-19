# VisionTactile-Dataset

![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04-E95420.svg)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E.svg)
![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)

[简体中文](README.zh-CN.md)

VisionTactile-Dataset captures two D405 cameras and one D436 through ROS 2.
The public workflow launches the cameras, publishes per-camera software timing,
and owns one uncompressed rosbag Recorder process per session.

## Status

- Software target: Ubuntu 24.04, ROS 2 Jazzy, RealSense ROS 4.58.1.
- Runtime recording uses the ROS 2 Jazzy `ros2bag` CLI.
- The reference configuration is not a claim of hardware exposure
  synchronization or recorded-data quality.
- Camera capture and a separate read-only three-Tracker ROS 2 publisher are
  available; Tracker streams are not yet part of the default camera bag.
- A separate offline Tracker-to-RealSense hand-eye calibration tool is
  available; dedicated calibration bags and device extrinsics remain outside
  the default nine-topic bag.
- The camera and Tracker source trees are reproducible from a clean public
  clone with submodules. Tracker hardware startup additionally requires the
  operator's private bootstrap capture, bundle, and role map; those private
  inputs are never stored in Git.

## Default bag contract

The default bag = **6 Image + 3 CameraFrameTiming = 9 topics**. Each of
`d405_1`, `d405_2`, and `d436` contributes color, depth, and grouped timing.
The allowlist is exact and wildcard-free.

All nine recorded topics use **keep-last depth 30, best-effort, volatile** QoS
overrides.

`COMPLETE = Recorder process lifecycle complete; it is not a data-quality claim.`

There is no real-time rosbag or MCAP compression. The Recorder writes MCAP
without either compression layer enabled.

Calibration, TF, raw RealSense metadata, and the session description are
available outside the recorded allowlist where applicable; they are not in the
bag.

## Hardware reference

The tested device identities and USB-topology guidance are in the
[hardware reference](docs/hardware-reference.md).

| Camera | Model | Serial number | Firmware | ASIC serial |
| --- | --- | --- | --- | --- |
| `d405_1` | D405 | `260322278433` | `5.15.1.55` | `255323071625` |
| `d405_2` | D405 | `260322276463` | `5.15.1.55` | `255323071742` |
| `d436` | D436 | `408322071716` | `5.17.0.213` | `343123151280` |

## Data-rate planning

`1280×720@30 raw estimate = 414.72 MB/s, about 124.4 GB per 300 s.`

This is an arithmetic planning estimate for the six image streams, not a
runtime capacity gate. Measure the target filesystem and leave suitable
operating margin for each environment.

## Quick start

Use an existing absolute data directory outside the repository. Install the
dependencies and build as described in the
[capture guide](docs/capture-guide.md), then launch:

```bash
export VT_REPO="$(git rev-parse --show-toplevel)"
export VT_DATA_ROOT="${HOME}/visiontactile-data"
mkdir -p "${VT_DATA_ROOT}"
git -C "${VT_REPO}" submodule update --init --recursive
source /opt/ros/jazzy/setup.bash
cd "${VT_REPO}/ros2_ws"
colcon build --event-handlers console_direct+
source install/setup.bash
ros2 launch vt_realsense_capture triple_realsense.launch.py \
  output_root:="${VT_DATA_ROOT}"
```

## Documentation

- [End-user operations manual (release-grade)](docs/user-manual.md)
- [Launch arguments, topics, QoS, and CLI reference](docs/interface-reference.md)
- [Architecture and the nine-topic contract](docs/architecture.md)
- [Capture guide](docs/capture-guide.md)
- [Hardware reference](docs/hardware-reference.md)
- [Troubleshooting](docs/troubleshooting.md)
- [VIVE Tracker ROS 2 publisher](docs/tracker-ros2-publisher.md)
- [Offline Tracker-to-RealSense extrinsic calibration](docs/tracker-camera-calibration.md)
- [Single-entry Tracker-to-RealSense calibration operations manual](tools/tracker_camera_calibration/README.md)
- [Maintainer release checklist](docs/release-checklist.md)

## Scope and limitations

This release does not claim cross-camera hardware exposure synchronization or
implement offline cross-camera alignment, point-cloud generation, tactile
capture, online extrinsic TF publication, or combined production
camera/Tracker recording. Offline camera/Tracker hand-eye calibration software
is included, but the three physical transforms still require per-rig hardware
calibration and acceptance. The Tracker publisher emits native `vive_map`
coordinates without TF. `CameraFrameTiming` describes software grouping within
one camera; inspect recorded data separately for fitness for a particular
experiment.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for software-test and documentation
requirements. Report sensitive defects through the private process in
[SECURITY.md](SECURITY.md).

## License

Project-authored files are Apache-2.0. The pinned PyVUT submodule has its own
Apache-2.0 license and attribution notice. See [LICENSE](LICENSE),
[third-party notices](THIRD_PARTY_NOTICES.md), and the license files inside the
submodule.
