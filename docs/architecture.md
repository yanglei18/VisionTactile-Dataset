# Architecture

## Scope

The public pipeline is a Recorder-only ROS 2 camera-capture workflow for a
fixed two-D405/one-D436 topology. The RealSense nodes publish live streams,
`timing_normalizer` groups each camera's color and depth metadata, and
`capture_controller` owns one `ros2 bag record` child process.

Recorder process lifecycle is the only completion criterion. The controller
does not perform camera identity, stream-health, storage-throughput, free-space,
frame-drop, or recorded-data validation gates.

## Package responsibilities

| Package | Responsibility |
| --- | --- |
| `vt_camera_msgs` | Defines command, status, event, session, compatibility status fields, and `CameraFrameTiming` interfaces. |
| `vt_realsense_capture` | Loads the fixed topology, launches the official RealSense nodes, groups per-camera timing metadata, starts and stops the Recorder, and publishes lifecycle status. |

## Data and control flow

```text
official realsense2_camera rs_launch.py x3
  -> six live Image topics
  -> timing_normalizer -> three live CameraFrameTiming topics
  -> capture_controller -> ros2 bag record -> uncompressed MCAP
```

The camera nodes also expose camera information, raw RealSense metadata,
extrinsics, TF, and `device_info` services to the live ROS graph. Those
supporting interfaces are not Recorder inputs.

## Exact 9 topics

The default Recorder allowlist contains exactly 9 topics:

| Topic | Type |
| --- | --- |
| `/d405_1/color/image_raw` | `sensor_msgs/msg/Image` |
| `/d405_1/depth/image_rect_raw` | `sensor_msgs/msg/Image` |
| `/d405_1/frame_timing` | `vt_camera_msgs/msg/CameraFrameTiming` |
| `/d405_2/color/image_raw` | `sensor_msgs/msg/Image` |
| `/d405_2/depth/image_rect_raw` | `sensor_msgs/msg/Image` |
| `/d405_2/frame_timing` | `vt_camera_msgs/msg/CameraFrameTiming` |
| `/d436/color/image_raw` | `sensor_msgs/msg/Image` |
| `/d436/depth/image_rect_raw` | `sensor_msgs/msg/Image` |
| `/d436/frame_timing` | `vt_camera_msgs/msg/CameraFrameTiming` |

This is **6 `sensor_msgs/msg/Image` + 3
`vt_camera_msgs/msg/CameraFrameTiming` = 9 topics**. The ordering is
deterministic by camera name, every name is absolute, and wildcards are
rejected.

Calibration topics, `/tf`, `/tf_static`, raw RealSense metadata, and
`/capture/session_info` are not in the bag. Capture command, status, and
event topics are also outside the recorded allowlist.

All nine recorded topics use **keep-last depth 30, best-effort, volatile** QoS
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
