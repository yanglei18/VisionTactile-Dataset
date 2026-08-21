# Architecture

## Scope

The public pipeline is a unified ROS 2 recording workflow for a fixed
two-D405/one-D436 and three-Tracker topology. The RealSense nodes publish live
streams, `timing_normalizer` groups each camera's color and depth metadata, the
read-only Tracker publisher emits three role streams, and `capture_controller`
owns one `ros2 bag record` child process.

Recorder process lifecycle is the only completion criterion. The controller
does not perform camera identity, stream-health, storage-throughput, free-space,
frame-drop, or recorded-data validation gates.

## Package responsibilities

| Package | Responsibility |
| --- | --- |
| `vt_camera_msgs` | Defines command, status, event, session, compatibility status fields, and `CameraFrameTiming` interfaces. |
| `vt_realsense_capture` | Loads the fixed topology, launches the official RealSense nodes, groups per-camera timing metadata, starts and stops the Recorder, and publishes lifecycle status. |
| `vt_vive_tracker` | Publishes three read-only Tracker role streams in the native `vive_map` frame. |
| `tools/tracker_camera_calibration` | Independently reads a dedicated completed bag, solves `tracker_from_camera`, validates fixed-board closure, and exports immutable external-calibration artifacts. It is not a ROS runtime package. |
| `tools/multisensor_alignment` | Reads one completed unified bag, audits clocks and identities, aligns cameras and Trackers in host realtime, applies external calibration, and exports an immutable extensible index. |

## Data and control flow

```text
official realsense2_camera rs_launch.py x3
  -> six live Image topics
  -> three live CameraInfo topics
  -> timing_normalizer -> three live CameraFrameTiming topics
read-only Tracker publisher -> three live TrackerSample topics
  -> capture_controller -> ros2 bag record -> uncompressed MCAP
```

The camera nodes also expose raw RealSense metadata, extrinsics, TF, and
`device_info` services to the live ROS graph. Those supporting interfaces are
not Recorder inputs. Color `CameraInfo` is a Recorder input because it fixes the
intrinsics used by downstream data products.

The optional calibration flow is deliberately separate:

```text
live color + CameraInfo + CameraFrameTiming + read-only TrackerSample
  -> manually recorded dedicated calibration bag
  -> offline ChArUco PnP and hand-eye solver
  -> extrinsics.yaml + JSON/CSV/SVG quality evidence
```

No calibration code executes in the production Recorder process, and a
missing or rejected external calibration cannot block raw camera recording.

The production post-processing flow is also separate from capture:

```text
one unified MCAP + three VALID tracker_from_camera files
  -> topic/type/identity/host-clock audit
  -> maximum-coverage ordered camera matching + bounded Tracker interpolation
  -> aligned_frames.jsonl + manifest/catalog/residual/quality evidence
```

Capture never waits for this tool and the tool never modifies the bag.

## Unified 15 topics

The default Recorder allowlist contains exactly 15 topics:

| Source | Topic suffix/pattern | Type | Count |
| --- | --- | --- | --- |
| Each of `d405_1`, `d405_2`, `d436` | `color/image_raw` | `sensor_msgs/msg/Image` | 3 |
| Each camera | `depth/image_rect_raw` | `sensor_msgs/msg/Image` | 3 |
| Each camera | `color/camera_info` | `sensor_msgs/msg/CameraInfo` | 3 |
| Each camera | `frame_timing` | `vt_camera_msgs/msg/CameraFrameTiming` | 3 |
| Each of `left_wrist`, `right_wrist`, `torso` | `/vive/<role>/sample` | `vt_tracker_msgs/msg/TrackerSample` | 3 |

This is **6 Image + 3 CameraInfo + 3 CameraFrameTiming + 3 TrackerSample = 15
topics**. The ordering is deterministic by camera name and Tracker role, every
name is absolute, and wildcards are rejected. Versioned additional streams are
sorted after the core and must declare their exact ROS message type.

Calibration results, `/tf`, `/tf_static`, raw RealSense metadata,
Tracker `/pose` and `/status`, and `/capture/session_info` are not in the bag.
Capture command, status, and event topics are also outside the allowlist.

All 15 recorded topics use **keep-last depth 30, best-effort, volatile** QoS
overrides. The MCAP writer configuration has `compression: None`; neither
rosbag transport compression nor MCAP chunk compression runs during capture.

## Four-state capture flow

The successful public flow is:

```text
IDLE -> RECORDING -> FINALIZING -> COMPLETE
```

- `START` creates a new session directory and QoS file, spawns the Recorder,
  then moves directly from `IDLE` to `RECORDING`.
- `STOP`, planned-duration expiry, or an early Recorder exit moves the
  controller to `FINALIZING`.
- Finalization sends the Recorder its shutdown sequence and retries while
  termination remains uncertain.
- `COMPLETE` is published only after Recorder process termination is
  confirmed.

`COMPLETE` therefore means **Recorder process lifecycle complete**. It does
not mean that every expected message arrived, that timing groups were complete,
that frames were not dropped, or that the MCAP contents passed a quality check.
Operators may inspect a completed bag independently without changing its state.

Startup failures leave the controller idle and publish a fatal event. The
wire-level message definitions retain older enum values and compatibility
fields, but the Recorder-only runtime does not add extra states to the
successful four-state flow.

## Why CameraFrameTiming is group based

`CameraFrameTiming` is keyed by one shared ROS stamp and carries the color and
depth frame numbers, timestamp domains, device/sensor/backend timestamps, host
callback clocks, callback spread, camera identity, and validity flags. It is
published when one color and one depth metadata observation complete a bounded
per-camera group. An incomplete group is logged and omitted from the timing
topic.

## Timestamp grouping is not exposure synchronization

The shared stamp and host-clock fields provide deterministic software grouping
within each camera. They do not trigger sensors, distribute a common hardware
clock, or prove simultaneous exposure across the three devices. No cross-camera
hardware exposure synchronization or offline alignment claim should be inferred
from `CameraFrameTiming`.

Offline alignment uses `group_host_realtime_ns` and
`TrackerSample.host_realtime_ns`. It checks realtime increments against each
stream's own host monotonic observations, performs no extrapolation, and
reports camera/Tracker coverage. Camera `CLOCK_MONOTONIC_RAW` and Tracker
`CLOCK_MONOTONIC` values are not compared directly across streams.
