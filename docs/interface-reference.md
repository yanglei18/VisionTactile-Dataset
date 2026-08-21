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
| `vt-multisensor-view` | Play a read-only three-camera/three-Tracker aligned dashboard or export one headless PNG frame. | `--alignment`, `--bag`, optional `--start`, `--speed`, `--export-frame` |

Use `--help` on the installed command as the final authority for CLI syntax in
the checked-out version.

## Aligned-data Python SDK

`vt-multisensor-alignment 0.3.0` exports a read-only SDK from the package root:

```python
from vt_multisensor_alignment import AlignedDataset

dataset = AlignedDataset.open(
    alignment_dir,
    bag_path,
    allow_rejected=False,
    verify_integrity=True,
    cache_size=8,
)
```

`alignment_dir` is the six-file output from `vt-multisensor-align align`;
`bag_path` is the exact original rosbag2 directory recorded in its manifest.
Opening validates the output and source identity. ROS 2 and the workspace
message packages are imported only when an image, timing, CameraInfo, or
extension payload is first requested.

### Metadata and payload methods

| Interface | Bag payload I/O | Result |
| --- | --- | --- |
| `len(dataset)` | No | Number of reference-camera alignment rows |
| `dataset.record(index)` | No | Immutable `FrameRecord` containing references and aligned transforms |
| `dataset.manifest` | No | Immutable manifest mapping |
| `dataset.quality_report` | No | Immutable quality mapping |
| `dataset.reference_times_ns` | No | Strictly increasing reference timeline used by realtime playback |
| `dataset.camera_info` | Yes, first access only | Camera-name mapping of `CameraInfoData` |
| `dataset.frame(index, ...)` | Yes, selected references | Immutable `AlignedFrame` |
| `dataset.iter_frames(start=0, stop=None, step=1, ...)` | Yes, forward cursor | Iterator of `AlignedFrame` |
| `dataset.close()` | — | Idempotently closes JSONL and rosbag resources |

`frame()` and `iter_frames()` share these selection arguments:

| Argument | Default | Contract |
| --- | --- | --- |
| `cameras` | `None` | All cameras, or a unique iterable of names from `dataset.camera_names` |
| `image_kinds` | `("color", "depth")` | Any unique subset of `color` and `depth` |
| `include_timing` | `True` | Deserialize each selected `CameraFrameTiming` message |
| `additional_streams` | `None` | All configured streams, or a unique subset; `()` skips payloads |

Negative frame indices follow Python sequence semantics. Iteration step must be
a positive integer. A non-null `MessageRef` is resolved by exact Topic and bag
timestamp, then checked against its source timestamp. Its sequence field is
provenance, not a storage offset.

### Returned values

| Type | Important fields and units |
| --- | --- |
| `AlignedFrame` | `frame_index`, `reference_camera`, `reference_time_ns`, cameras, Trackers, extensions, quality flags |
| `CameraSample` | host/source nanoseconds, signed delta, optional color/depth, optional timing message, attached Tracker, `world_from_camera` |
| `ImageData` | read-only NumPy `array`, original encoding, optical frame, source time, reference |
| `TrackerPose` | role, physical-ID hash, interpolation bracket, `world_from_tracker` |
| `AdditionalSample` | stream name, timestamp, signed delta, reference, deserialized ROS message |
| `CameraInfoData` | width/height, distortion model, read-only `d`, `k`, `r`, `p`, binning and ROI |
| `Transform` | translation in metres, quaternion `x,y,z,w`, read-only `as_matrix()` |

Supported Image encodings are `rgb8`, `bgr8`, `mono8`, `mono16`, `16UC1`, and
`32FC1`. The decoder honors row stride/padding and byte order. It does not swap
RGB/BGR or convert depth to metres.

Aligned JSON `null` values remain `None`. A selected non-null reference that is
not present in the bound bag raises `MissingMessageError`.

### Lifecycle and failures

Use `AlignedDataset` as a context manager. Instances are not thread-safe; each
process or data-loader worker opens its own instance. The decoded-frame LRU is
bounded by `cache_size`; zero disables it.

All reader failures inherit `DatasetError`:

| Exception | Contract violation |
| --- | --- |
| `IntegrityError` | Alignment file inventory, size, or SHA-256 mismatch |
| `RejectedDatasetError` | `REJECTED` verdict without explicit diagnostic opt-in |
| `SourceBagMismatchError` | Bag name, metadata hash, or storage identifier mismatch |
| `DatasetFormatError` | Alignment/ROS schema, Topic type, timestamp, or CameraInfo mismatch |
| `MissingMessageError` | Non-null reference cannot be resolved exactly |
| `UnsupportedEncodingError` | Image encoding is outside the supported table |
| `DatasetClosedError` | API access after `close()` |

`allow_rejected=True` is diagnostic-only. `verify_integrity=False` skips output
file hash recomputation but never disables JSON structure or source-bag identity
checks. The complete runnable workflow is in the
[alignment and SDK manual](../tools/multisensor_alignment/README.md#10-使用-python-sdk-读取对齐数据).

## Aligned-data Viewer

Install the package with its visualization extra and ensure Ubuntu Tk support
is present:

```bash
sudo apt-get install python3-tk
python -m pip install "${VT_REPO}/tools/multisensor_alignment[viewer]"
```

Interactive mode opens a `1600×900` Pillow/Tk dashboard by default:

```bash
vt-multisensor-view --alignment "${ALIGN_OUTPUT}" --bag "${BAG}"
```

| Argument | Default | Contract |
| --- | --- | --- |
| `--start` | `0` | Initial frame index; negative values use Python sequence semantics |
| `--speed` | `1.0` | Positive playback multiplier over `reference_time_ns` |
| `--width`, `--height` | `1600`, `900` | Dashboard dimensions; minimum `800×480` |
| `--depth-min-m`, `--depth-max-m` | `0.2`, `3.0` | Fixed depth-color range in metres |
| `--tracker-range-m` | `2.0` | Fixed symmetric XY/XZ plot range around world origin |
| `--allow-rejected` | off | Diagnostic-only opt-in to a `REJECTED` alignment |
| `--skip-integrity` | off | Skip output SHA-256 recomputation, not schema or bag identity checks |
| `--export-frame` | unset | Write one new PNG and exit without Tk or a display server |

Interactive playback requests only color, depth, and aligned Tracker values.
It does not deserialize timing or extension messages. When rendering is late,
the player advances to the frame corresponding to current data time rather
than accumulating latency; all source rows remain available while paused.
Controls are Space, Left/Right, Home/End, `+`/`-`, and Q/Escape. The product
workflow is in the
[Viewer manual](../tools/multisensor_alignment/README.md#11-离线可视化).

## Compatibility and change control

The reference platform is Ubuntu 24.04, ROS 2 Jazzy, RealSense ROS 4.58.1,
and the PyVUT revision pinned in
[Tracker Linux validation](tracker-linux-validation.md). Replacement camera
serials require a reviewed configuration file. Firmware, RealSense wrapper,
PyVUT revision, message definition, QoS, topic allowlist, or Tracker mapping
changes require a new acceptance record; they must not be treated as a
transparent runtime change.
