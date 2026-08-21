# Interface reference

This page is the operator-facing reference for launch arguments, ROS 2
interfaces, command-line tools, and time semantics. Commands assume the
workspace has already been built and sourced as described in the
[end-user manual](user-manual.md).

## Common environment

Use absolute paths and keep runtime data outside the Git worktree:

```bash
export VT_REPO="$(git rev-parse --show-toplevel)"
export VT_WS="${VT_REPO}/ros2_ws"
export VT_DATA_ROOT="${HOME}/visiontactile-data"
mkdir -p "${VT_DATA_ROOT}"

source /opt/ros/jazzy/setup.bash
source "${VT_WS}/install/setup.bash"
```

Every terminal in one run must use the same `ROS_DOMAIN_ID`.

## Camera launch arguments

Launch with:

```bash
ros2 launch vt_realsense_capture triple_realsense.launch.py \
  output_root:="${VT_DATA_ROOT}"
```

| Argument | Required | Default | Meaning |
| --- | --- | --- | --- |
| `output_root` | Yes | empty | Existing, writable absolute directory that is not itself a symlink. The supported workflow additionally requires a canonical path outside the repository. |
| `config_path` | No | installed `config/cameras.yaml` | Fixed D405/D405/D436 capture configuration. A replacement file must satisfy the strict topology, stream, and recording contract. |

The supported stream contract is RGB8 color plus Z16 depth at
`1280x720x30` for each camera. The recording contract is a 300-second maximum
bag duration, 128 GiB maximum bag size, and 1 GiB rosbag cache. These values
are configuration-contract values, not runtime quality guarantees.

### Recorded unified topics

The core allowlist contains exactly these 15 best-effort, volatile topics. The
Recorder applies keep-last depth 30 QoS overrides.

| Camera | Color | Depth | Color intrinsics | Timing |
| --- | --- | --- | --- | --- |
| `d405_1` | `/d405_1/color/image_raw` | `/d405_1/depth/image_rect_raw` | `/d405_1/color/camera_info` | `/d405_1/frame_timing` |
| `d405_2` | `/d405_2/color/image_raw` | `/d405_2/depth/image_rect_raw` | `/d405_2/color/camera_info` | `/d405_2/frame_timing` |
| `d436` | `/d436/color/image_raw` | `/d436/depth/image_rect_raw` | `/d436/color/camera_info` | `/d436/frame_timing` |

| Tracker role | Recorded Topic | Type |
| --- | --- | --- |
| `left_wrist` | `/vive/left_wrist/sample` | `vt_tracker_msgs/msg/TrackerSample` |
| `right_wrist` | `/vive/right_wrist/sample` | `vt_tracker_msgs/msg/TrackerSample` |
| `torso` | `/vive/torso/sample` | `vt_tracker_msgs/msg/TrackerSample` |

Calibration results, TF, raw RealSense metadata, Tracker `/pose` and `/status`,
and all `/capture/*` topics may be visible live but are not recorded. Explicit
`recording.additional_streams` entries are sorted after the 15-topic core.

The separate calibration workflow records its own dedicated bag. See
[Tracker–camera offline calibration](tracker-camera-calibration.md).

### Capture control topics

| Topic | Type | QoS | Direction and purpose |
| --- | --- | --- | --- |
| `/capture/command` | `vt_camera_msgs/msg/CaptureCommand` | reliable, volatile, depth 10 | Operator to controller; `START=1`, `STOP=2`. |
| `/capture/status` | `vt_camera_msgs/msg/CaptureStatus` | reliable, volatile, depth 10 | Current Recorder lifecycle, published periodically. |
| `/capture/event` | `vt_camera_msgs/msg/CaptureEvent` | reliable, volatile, depth 10 | Discrete informational, warning, or fatal events. |
| `/capture/session_info` | `vt_camera_msgs/msg/SessionInfo` | reliable, transient-local, depth 10 | Latest session identity and environment description. |

The successful lifecycle is:

```text
IDLE -> RECORDING -> FINALIZING -> COMPLETE
```

`INVALID` is the terminal result when finalization cannot confirm a valid
Recorder stop. `PREFLIGHT` and `WARMING_UP` remain wire-compatibility constants
but are not entered by the current Recorder-only workflow. `COMPLETE` means
only that Recorder process termination was confirmed. It does not assert topic
completeness, timing quality, absence of loss, or experiment suitability.

## Camera time semantics

`CameraFrameTiming` groups one color metadata observation and one depth
metadata observation from the same camera. It does not hardware-synchronize
three cameras.

| Field family | Clock/source | Intended use |
| --- | --- | --- |
| `header.stamp`, `shared_ros_timestamp_ns` | Common ROS stamp used to group the color/depth pair | ROS correlation within one camera. |
| `*_device_timestamp_ns` | RealSense-reported device timestamp and domain | Device-stream diagnostics; may reset or use a different domain. |
| `*_host_monotonic_raw_ns`, `group_host_monotonic_raw_ns` | Linux host monotonic raw clock | Interval and ordering measurements within the current boot. |
| `*_host_realtime_ns`, `group_host_realtime_ns` | Linux host realtime clock | Correlation with host wall-clock logs. |

The D436 device clock must not be substituted for host time. Use the explicit
host realtime fields when wall-clock correlation is required, and inspect the
validity flags before consuming any optional timestamp field.

## Tracker launch arguments

Launch the read-only publisher with its two private input files:

```bash
ros2 launch vt_vive_tracker triple_tracker.launch.py \
  bundle_path:="${VT_BUNDLE}" \
  role_map_path:="${VT_ROLE_MAP}"
```

| Argument | Required | Default | Meaning |
| --- | --- | --- | --- |
| `bundle_path` | Yes | none | Absolute path to the private, approved bootstrap bundle. It must be a regular non-symlink file, owned by the current user, with no group/other permission bits. |
| `role_map_path` | Yes | none | Absolute path to the private physical-role map, with the same ownership and permission requirements as the bundle. |
| `frame_id` | No | `vive_map` | Native mapped coordinate-frame label; it does not imply camera or world alignment. |
| `status_rate_hz` | No | `10.0` | Per-role status publication rate; valid range 0.1–100 Hz. |
| `read_timeout_ms` | No | `100` | Backend read timeout used for bounded shutdown and recovery; valid range 1–100 ms. |
| `disconnect_timeout_ms` | No | `1000` | Time without a report before a role is considered disconnected; valid range 1–60000 ms. |
| `queue_capacity` | No | `4096` | Maximum in-process sample queue depth before old samples are counted as dropped; valid range 1–1000000. |

Changing timeout, rate, or queue values creates a non-reference operating
profile and must be recorded in the experiment manifest.
Hardware input also rejects the ROS parameter `use_sim_time=true`.

### Tracker topics

Each role is one of `left_wrist`, `right_wrist`, or `torso`.

| Topic pattern | Type | QoS | Meaning |
| --- | --- | --- | --- |
| `/vive/<role>/sample` | `vt_tracker_msgs/msg/TrackerSample` | best-effort, volatile, depth 10 | Pose, motion fields, identity, tracking status, and host timestamps. |
| `/vive/<role>/pose` | `geometry_msgs/msg/PoseStamped` | best-effort, volatile, depth 10 | Convenience pose in `vive_map`. |
| `/vive/<role>/status` | `vt_tracker_msgs/msg/TrackerStatus` | reliable, transient-local, depth 1 | Connection/tracking state and cumulative counters. |

The publisher emits no TF and does not align `vive_map` with any RealSense
frame. Only the three `/sample` topics enter the unified bag; convenience pose
and status streams remain live-only.

## Visualization launch arguments

```bash
ros2 launch vt_vive_tracker tracker_visualization.launch.py
ros2 launch vt_vive_tracker_gui tracker_gui.launch.py
```

`tracker_visualization.launch.py` accepts optional `rviz_config`, defaulting to
the installed three-Tracker RViz configuration. The standalone GUI has no
launch arguments. Both are read-only ROS consumers: neither opens the Dongle,
changes pairing or mapping, starts the publisher, nor records data.

## Command-line tools

| Command | Purpose | Important arguments |
| --- | --- | --- |
| `ros2 run vt_realsense_capture storage_bench` | Optional destructive-write throughput test in a dedicated temporary file under the selected data root. | `--output-root <absolute-directory>` |
| `ros2 run vt_vive_tracker vt-vive-write-role-map` | Create a private role map from an approved bundle. | `--bundle`, `--output`, `--host`, `--client0`, `--client1` |
| `ros2 run vt_vive_tracker vt-vive-validate-topics` | Measure the three published ROS role streams and write a private JSON report. | `--duration` (default `30`), required `--output` |
| `vt-vut-validate` | Validate direct-USB or Dongle input through the separately installed validation package. | `preflight` or `run`, `--mode`, `--duration`, `--expected-trackers`, `--output` |
| `vt-tracker-camera-calibrate configure` | Create an immutable, identity-bound calibration configuration for one reference camera and one physical Tracker role. | `--camera`, `--tracker-role`, `--square-length-mm`, `--marker-length-mm`, `--output` |
| `vt-tracker-camera-calibrate board` | Render the configured printable ChArUco board with DPI metadata. | `--config`, `--output`, optional `--dpi` |
| `vt-tracker-camera-calibrate calibrate` | Read a dedicated bag, solve Tracker-to-color-optical external calibration, validate it, and export immutable artifacts. | `--bag`, `--config`, `--output` |
| `vt-tracker-camera-calibrate compare` | Verify identity consistency and pairwise repeatability across at least three valid calibration runs, then identify the run closest to their consensus. | `--inputs`, `--output`, optional repeatability thresholds |
| `vt-multisensor-align inspect` | Check one unified bag's required topics, types, identities, frames, and valid input counts without writing output. | `--bag`, `--config` |
| `vt-multisensor-align align` | Match three cameras, interpolate three Tracker streams, apply three external calibrations, and atomically export an alignment index. | `--bag`, `--config`, `--extrinsics`, `--output` |
| `vt-multisensor-align validate` | Recompute output integrity hashes and verify row count and quality verdict. | `--output` |

Use `--help` on the installed command as the final authority for CLI syntax in
the checked-out version.

## Compatibility and change control

The reference platform is Ubuntu 24.04, ROS 2 Jazzy, RealSense ROS 4.58.1,
and the PyVUT revision pinned in
[Tracker Linux validation](tracker-linux-validation.md). Replacement camera
serials require a reviewed configuration file. Firmware, RealSense wrapper,
PyVUT revision, message definition, QoS, topic allowlist, or Tracker mapping
changes require a new acceptance record; they must not be treated as a
transparent runtime change.
