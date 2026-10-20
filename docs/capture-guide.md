# Unified capture guide

[简体中文](capture-guide.zh-CN.md) · [New computer deployment](deployment.md)

This guide records the three cameras and three Tracker sample streams in one
explicit unified bag. Commands assume Ubuntu 24.04, ROS 2 Jazzy, RealSense ROS
4.58.1, a working read-only Tracker Publisher, and an output directory outside
the source repository. These commands describe the full reference topology.
The workbench can record available configured sources without Trackers; missing
optional sources are coverage information, not recording failures.

Define reusable paths once from the repository root:

```bash
export VT_REPO="$(git rev-parse --show-toplevel)"
export VT_WS="${VT_REPO}/ros2_ws"
export VT_DATA_ROOT="${HOME}/visiontactile-data"
```

## 1. Install and verify dependencies

On a new computer, first complete the
[deployment guide](deployment.md). It covers Ubuntu and [ROS 2 Jazzy setup](deployment.md#ros),
`rosdep` initialization, [RealSense ROS 4.58.1 installation](deployment.md#realsense) and what to do when
the APT candidate differs, USB access, and the workbench environment. Return
here for the command-line capture workflow after that setup is verified.

The deployment guide installs the Jazzy `ros2bag` command and required storage
plugins, including MCAP. No rosbag or MCAP compression plugin is needed.

The commands below use the deployment guide's source-built driver workspace.
If you use its separately verified APT route, omit the driver workspace
`source` command and keep the version checks.

```bash
test -z "${CONDA_PREFIX:-}"
source /opt/ros/jazzy/setup.bash
source "${HOME}/visiontactile-drivers_ws/install/setup.bash"
test "$ROS_DISTRO" = jazzy
ros2 pkg xml realsense2_camera |
  sed -n 's:.*<version>\(.*\)</version>.*:\1:p'
ros2 pkg prefix ros2bag
ros2 bag --help
rs-enumerate-devices -s
```

The RealSense package version command must print `4.58.1`. If it differs, follow
the version selection procedure in the [deployment guide](deployment.md#realsense);
do not silently substitute a newer wrapper. Use the system Python environment
rather than Conda for project build, test, and runtime commands.

Resolve declared ROS package dependencies on a host where `rosdep` has already
been initialized:

```bash
cd "${VT_REPO}"
rosdep update
rosdep install --from-paths ros2_ws/src --ignore-src -r -y \
  --rosdistro jazzy \
  --skip-keys "librealsense2 realsense2_camera realsense2_camera_msgs"
```

These skipped dependencies are already supplied by the verified driver
installation; the skip list does not make them optional.

## 2. Build and test

Build the workspace normally. Do not use a symlinked install:

```bash
test -z "${CONDA_PREFIX:-}"
source /opt/ros/jazzy/setup.bash
source "${HOME}/visiontactile-drivers_ws/install/setup.bash"
cd "${VT_WS}"
colcon build --event-handlers console_direct+
source install/setup.bash
PYTHONPATH="${VT_REPO}/tools/multisensor_alignment/src:${VT_REPO}/ros2_ws/src/vt_realsense_capture${PYTHONPATH:+:$PYTHONPATH}" \
  colcon test --event-handlers console_direct+
colcon test-result --all --verbose
```

Proceed only after the software tests report zero failures. Keep `build/`,
`install/`, and `log/` out of Git. The main workspace's `install/setup.bash`
retains the driver underlay selected at build time, so keep that driver
workspace in place for the later launch commands.

## 3. Prepare storage and USB topology

Create an absolute, existing, writable output root outside the repository:

```bash
mkdir -p "${VT_DATA_ROOT}"
test "$(realpath "${VT_DATA_ROOT}")" = "${VT_DATA_ROOT}"
test -w "${VT_DATA_ROOT}"
df -h "${VT_DATA_ROOT}"
lsusb -t
```

The raw image planning estimate is 414.72 MB/s, or about 124.4 GB for the
configured 300-second maximum session. Before creating a session and again
before starting the Recorder, the controller checks free capacity against the
planned duration, 10% message/container overhead, and a stop reserve that
includes the Recorder cache. The default requirements are about 15.02 GiB for
30 seconds, 27.77 GiB for 60 seconds, and 129.73 GiB for 300 seconds. No
compression or dropped-frame savings are assumed. This is a capacity check;
sustained write throughput and recorded data quality still need validation.

While recording, free space is checked every 0.5 seconds. Reaching the stop
reserve, or failing to query capacity, triggers the existing stop/finalization
workflow. A confirmed early exit retains its reason; an unconfirmed exit stays
in `FINALIZING`. The reserve cannot guarantee recovery from another process
exhausting the filesystem suddenly or a Recorder that fails to stop.

Optional settings belong under `recording.disk_guard` in the selected camera
YAML, alongside the existing recording settings:

```yaml
disk_guard:
  min_free_bytes: 2147483648
  check_interval_sec: 0.5
  additional_stream_bytes_per_second: 0
```

The effective stop reserve is at least `min_free_bytes`, and can be larger to
cover the cache and normal stop latency. The additional-stream estimate is the
combined byte rate of extra topics. Zero means no estimate is configured; when
extra topics are present the controller explicitly marks the budget as covering
known image streams only. Runtime free-space checks remain active. Settings
take effect when the controller next starts.

Move the D436 to an independent USB root bus if `lsusb -t` shows it sharing
the same constrained root path as the two D405 cameras. Different physical
sockets do not necessarily mean different host controllers.

Check USB access and the RSUSB buffer budget without opening any camera:

```bash
python3 tools/capture_deploy/configure_realsense_usb.py check
```

If correction is needed, follow the
[RealSense USB check/apply/rollback guide](../tools/capture_deploy/REALSENSE_USB.md).
Stop camera processes before applying changes. The helper's buffer setting
lasts for the current boot; run `check` again after reboot. The RSUSB buffer
requirement does not diagnose V4L2 frame loss.

Optionally measure the target filesystem before a formal run. This writes a
dedicated 16 GiB test file, removes that data file, and leaves a JSON report;
it is not a Recorder lifecycle gate:

```bash
ros2 run vt_realsense_capture storage_bench \
  --output-root "${VT_DATA_ROOT}"
```

## 4. Launch the Tracker Publisher

Complete the approved manual bootstrap, then launch the read-only Publisher in
its own terminal as described in the
[Tracker runbook](tracker-ros2-publisher.md):

```bash
source /opt/ros/jazzy/setup.bash
source "${VT_WS}/install/setup.bash"
ros2 launch vt_vive_tracker triple_tracker.launch.py \
  bundle_path:="${VT_BUNDLE}" \
  role_map_path:="${VT_ROLE_MAP}"
```

The Recorder never starts or configures the Tracker hardware. Experiments that
need all three poses should verify the three `/vive/<role>/sample` streams before
START. Workbench recording itself does not require Trackers: new plans freeze
receiving sources and keep absent candidates optional. Late sources on the
configured allowlist are still recorded.

## 5. Launch the cameras and controller

First create the machine-specific `config/cameras.yaml` described in the
[deployment guide](deployment.md#permissions), including your actual camera identities.
Keep the launch process in the foreground:

```bash
source /opt/ros/jazzy/setup.bash
source "${VT_WS}/install/setup.bash"
ros2 launch vt_realsense_capture triple_realsense.launch.py \
  config_path:="${VT_DATA_ROOT}/config/cameras.yaml" \
  output_root:="${VT_DATA_ROOT}"
```

The launch starts three serial-bound RealSense nodes, `timing_normalizer`, and
`capture_controller`. It does not begin recording until a `START` command
is accepted.

## 6. Check the live graph

In another shell with the same ROS environment and `ROS_DOMAIN_ID`, check the
live graph:

```bash
ros2 topic list
for topic in \
  /d405_1/color/image_raw /d405_1/depth/image_rect_raw \
  /d405_1/color/camera_info /d405_1/frame_timing \
  /d405_1/depth/camera_info /d405_1/extrinsics/depth_to_color \
  /d405_2/color/image_raw /d405_2/depth/image_rect_raw \
  /d405_2/color/camera_info /d405_2/frame_timing \
  /d405_2/depth/camera_info /d405_2/extrinsics/depth_to_color \
  /d436/color/image_raw /d436/depth/image_rect_raw \
  /d436/color/camera_info /d436/frame_timing \
  /d436/depth/camera_info /d436/extrinsics/depth_to_color \
  /vive/left_wrist/sample /vive/right_wrist/sample /vive/torso/sample
do
  ros2 topic info -v "$topic"
done
```

The timing topics have type `vt_camera_msgs/msg/CameraFrameTiming`; Tracker
sample topics have type `vt_tracker_msgs/msg/TrackerSample`. TF, raw RealSense
metadata, Tracker pose/status convenience topics, and control topics may also
appear live, but are not in the bag. The current allowlist contains 21 candidates,
including color/depth CameraInfo and factory `extrinsics/depth_to_color` for each
camera. Extrinsics subscriptions use reliable/transient-local/keep-last depth 1
to receive calibration retained by cameras started earlier. The remaining core
topics use keep-last depth 30, volatile: the six managed image subscriptions
request reliable delivery; CameraInfo, timing and Tracker remain best-effort.

Factory calibration is recorded without Tracker or space-view configuration. It
describes intrinsics and the depth-to-color relation inside one camera, not an
external mounting/world transform. New Recipe v2 uses `unified-dataset-v2`; saved
Recipe v1 plans retain their original four-topic camera bundle and fingerprints.
To inspect an already published factory transform, use a transient-local reader:

```bash
ros2 topic echo --once --qos-reliability reliable --qos-durability transient_local \
  /d405_1/extrinsics/depth_to_color
```

### Camera image transport

The managed RealSense publishers explicitly use the wrapper's `DEFAULT` color
and depth QoS (reliable, volatile, keep-last depth 10), rather than an unspecified
system default. These are node parameters in `realsense_options.yaml`; the
Recorder requests compatible reliable image subscriptions. External image
publishers and optional sources are not assumed to offer reliable delivery.

The workbench manager and direct `triple_realsense.launch.py` entry point use
the same packaged Fast DDS profile unless the operator supplies middleware or
transport configuration. The profile provisions a 64 MiB SHM segment per
participant and a 4096-entry port queue, retaining UDPv4 discovery and remote
communication. It keeps the existing 65500-byte transport message limit, so
large images still fragment. Each camera produces approximately 138.24 MB/s
of raw RGB8/Z16 payload; the segment provides bounded burst capacity rather
than allocating unbounded image history.

The profile is applied before ROS initialization and inherited by camera,
controller and Recorder processes. It is independent of the 1 GiB rosbag writer
cache, which cannot recover samples lost before Recorder reception. Existing
processes must be stopped and restarted to apply a changed transport profile.
Explicit DDS profiles and non-Fast-DDS middleware remain operator-controlled.
These settings are not a promise of lossless recording: inspect each recorded
image stream's timestamps, rate, gaps and Recorder transport-loss log.

### High-rate glove and tactile recording buffers

The optional `cameras.with_right_glove_tactile.yaml` profile records solved
`/gloves/right/state` and raw `/gloves/right/tactile/samples` alongside the
21 core topics. Recorder subscription history is selected by the **configured
message type**, not by guessing from the topic name:

| Recorded message type | Default keep-last depth |
| --- | ---: |
| `vt_tactile_msgs/msg/TactileSampleArray` | 512 |
| `vt_glove_msgs/msg/GloveState` | 256 |
| Managed factory extrinsics | 1 (reliable, transient-local) |
| Other camera, Tracker and other types | 30 |

Optional sources retain `best_effort` / `volatile`; their publisher settings,
sensor sampling rates and 20×20 tactile state format are unchanged. At 500 tactile batches/s,
512 entries represent approximately one second of subscriber history, **not a
promise of lossless recording**. DDS resource limits, publisher history, sustained
storage throughput, and the existing rosbag2 cache still matter. A larger history
helps with short scheduling bursts; it cannot fix a disk that is persistently slow.

For a particular deployment, add optional exact-topic overrides to the existing
`recording` section of your camera configuration (do not replace its other keys):

```yaml
recording:
  # Retain max_bag_duration_seconds, max_bag_size_bytes,
  # max_cache_size_bytes and additional_streams from your existing file.
  qos_overrides:
    /gloves/right/tactile/samples:
      depth: 1024
    /gloves/right/state:
      depth: 256
      reliability: best_effort
```

Only `depth` (integer 1–4096) and `reliability` (`best_effort` or `reliable`)
are accepted. A topic must already belong to the explicit recorded topic list;
wildcards, unknown keys and non-recorded topics are rejected at configuration
load. Omitted fields inherit the source/type defaults, and existing configurations without
`qos_overrides` continue to work. Use `reliable` only when the publisher offers
reliable QoS and the deployment has been tested: it will not connect to a
best-effort-only publisher and may introduce retransmission/backpressure costs.

The exact merged profiles used by `ros2 bag record` are persisted in each session's
`qos_overrides.yaml`; the recorder receives that file through
`--qos-profile-overrides-path`. Changes take effect on the next controller launch
and recording session, not in an already running bag. These are buffer choices,
not live quality gates: a degraded sensor does not stop other streams.

## 7. Send START

The following command requests a session with the configured 300-second planned
duration:

```bash
ros2 topic pub --once /capture/command vt_camera_msgs/msg/CaptureCommand \
  "{request_id: run-001, command: 1, session_label: trial, planned_duration_sec: 300}"
```

Use a new nonempty `request_id` for each distinct command. Repeating the same
request ID and command is idempotent; reusing it for the other command is a
conflict.

## 8. Observe status

```bash
ros2 topic echo /capture/status
```

The successful state flow is:

```text
IDLE -> RECORDING -> FINALIZING -> COMPLETE
```

The status detail `Recorder process lifecycle complete` means the Recorder
was started and its termination was confirmed. `COMPLETE` is not a
recorded-data quality claim.

Keep the three measurement stages separate when diagnosing frame loss:

- The workbench's **Receive Hz** counts messages received by the manager during
  the last second. Its subscription is independent of the Recorder; this value
  is neither a camera hardware counter nor a count of messages written to bag.
- `CaptureStatus.streams` and `camera_groups` are currently empty, and
  `observed_write_mb_s` is a reserved field that currently reports zero.
  That zero means the measurement is unavailable, not confirmed zero throughput.
  File growth alone would also not prove complete recording or durable writes.
- After recording ends, open **Archive → Trial Details → Quality Report**.
  Only a completed report provides the checks it actually performed. Missing,
  queued, interrupted or failed checks provide no quality verdict, and a report
  with no warnings does not certify properties outside its inspected scope.

Real-time per-topic Recorder loss counters and sustained full-device 30 Hz
acceptance remain separate work; preview rate and software tests do not replace
those measurements.

For managed recording, a session directory is allocated before the Recorder is
started. Its existence alone is not proof of recording. A matching controller
instance and START request, an idle state and a confirmed absence of an active
Recorder resolve the trial as `FAILED`; its diagnostic directory is retained,
and the next explicit START is allowed. The same rule applies after an earlier
completed or invalid session.

If startup raises after a child may have been created, the controller keeps
ownership and remains `FINALIZING` until termination is confirmed. Timeouts,
changed controller instances and conflicting session identities remain
unresolved; they never authorize an automatic retry. Session lifecycle journals
retain the startup error and finalization reason, return code and termination
confirmation separately from any quality verdict.

Events and the current session identity can be observed separately:

```bash
ros2 topic echo /capture/event
ros2 topic echo --once /capture/session_info \
  --qos-durability transient_local
```

`/capture/status`, `/capture/event`, and `/capture/session_info` are
control-plane topics and are not recorded.

## 9. Send STOP

A session finalizes automatically at its planned duration. To stop it earlier,
publish:

```bash
ros2 topic pub --once /capture/command vt_camera_msgs/msg/CaptureCommand \
  "{request_id: stop-001, command: 2, session_label: '', planned_duration_sec: 0}"
```

Wait for `COMPLETE` before stopping the foreground launch process. While
`FINALIZING`, the controller is confirming that the Recorder process has
terminated.

## 10. Inspect and align offline

After `COMPLETE`, inspect the session manually if desired:

```bash
ros2 bag info "${VT_DATA_ROOT}/<session-id>/bag"
```

`ros2 bag info` is optional human inspection. It never changes
`COMPLETE`, and the controller does not parse its output. The default allowlist
contains six `sensor_msgs/msg/Image`, six `sensor_msgs/msg/CameraInfo`, three
`realsense2_camera_msgs/msg/Extrinsics`, three `vt_camera_msgs/msg/CameraFrameTiming`
and three `vt_tracker_msgs/msg/TrackerSample` topics. Actual recorded coverage
depends on available sources; a missing optional Tracker does not block recording.

The bag intentionally excludes external Tracker–camera calibration files, TF, raw RealSense metadata, session
description, capture commands, status, and events. There is no real-time rosbag
or MCAP compression; copy or compress a completed session only as a separate
offline operation.

Run the separate offline audit and alignment workflow before using the data:

```bash
vt-multisensor-align inspect --bag "${VT_DATA_ROOT}/<session-id>/bag" \
  --config "${ALIGN_CONFIG}"
```

The complete install, external-calibration, alignment, output, and quality-gate
procedure is in the
[single-entry offline alignment manual](../tools/multisensor_alignment/README.md).
All launch arguments, topic QoS, and timestamp semantics are collected in the
[interface reference](interface-reference.md).
