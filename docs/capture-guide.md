# Unified capture guide

This guide records the three cameras and three Tracker sample streams in one
explicit unified bag. Commands assume Ubuntu 24.04, ROS 2 Jazzy, RealSense ROS
4.58.1, a working read-only Tracker Publisher, and an output directory outside
the source repository.

Define reusable paths once from the repository root:

```bash
export VT_REPO="$(git rev-parse --show-toplevel)"
export VT_WS="${VT_REPO}/ros2_ws"
export VT_DATA_ROOT="${HOME}/visiontactile-data"
```

## 1. Install and verify dependencies

Install the RealSense wrapper, the Jazzy `ros2bag` CLI, librealsense tools,
and the Python YAML dependency:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  ros-jazzy-realsense2-camera \
  ros-jazzy-ros2bag \
  librealsense2-utils fio python3-yaml
```

The `ros2bag` package supplies the Jazzy `ros2 bag` command and its default
storage plugins, including MCAP. No rosbag or MCAP compression plugin is needed.

```bash
test -z "${CONDA_PREFIX:-}"
source /opt/ros/jazzy/setup.bash
test "$ROS_DISTRO" = jazzy
ros2 pkg xml realsense2_camera |
  sed -n 's:.*<version>\(.*\)</version>.*:\1:p'
ros2 pkg prefix ros2bag
ros2 bag --help
rs-enumerate-devices -s
```

The RealSense package version command must print `4.58.1`. Use the system
Python environment rather than Conda for project build, test, and runtime
commands.

Resolve declared ROS package dependencies on a host where `rosdep` has already
been initialized:

```bash
cd "${VT_REPO}"
rosdep update
rosdep install --from-paths ros2_ws/src --ignore-src -r -y \
  --rosdistro jazzy
```

## 2. Build and test

Build the workspace normally. Do not use a symlinked install:

```bash
test -z "${CONDA_PREFIX:-}"
source /opt/ros/jazzy/setup.bash
cd "${VT_WS}"
colcon build --event-handlers console_direct+
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --all --verbose
```

Proceed only after the software tests report zero failures. Keep `build/`,
`install/`, and `log/` out of Git.

## 3. Prepare storage and USB topology

Create an absolute, existing, writable output root outside the repository:

```bash
mkdir -p "${VT_DATA_ROOT}"
test "$(realpath "${VT_DATA_ROOT}")" = "${VT_DATA_ROOT}"
test -w "${VT_DATA_ROOT}"
df -h "${VT_DATA_ROOT}"
lsusb -t
```

The raw planning estimate is 414.72 MB/s, or about 124.4 GB for the configured
300-second maximum session. This estimate helps the operator provision storage;
the controller does not run a throughput or capacity quality gate.

Move the D436 to an independent USB root bus if `lsusb -t` shows it sharing
the same constrained root path as the two D405 cameras. Different physical
sockets do not necessarily mean different host controllers.

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

The Recorder never starts or configures the Tracker hardware. For a formal
unified run, all three `/vive/<role>/sample` streams must be valid before START.

## 5. Launch the cameras and controller

Keep the launch process in the foreground:

```bash
source /opt/ros/jazzy/setup.bash
source "${VT_WS}/install/setup.bash"
ros2 launch vt_realsense_capture triple_realsense.launch.py \
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
  /d405_2/color/image_raw /d405_2/depth/image_rect_raw \
  /d405_2/color/camera_info /d405_2/frame_timing \
  /d436/color/image_raw /d436/depth/image_rect_raw \
  /d436/color/camera_info /d436/frame_timing \
  /vive/left_wrist/sample /vive/right_wrist/sample /vive/torso/sample
do
  ros2 topic info -v "$topic"
done
```

The timing topics have type `vt_camera_msgs/msg/CameraFrameTiming`; Tracker
sample topics have type `vt_tracker_msgs/msg/TrackerSample`. TF, raw RealSense
metadata, Tracker pose/status convenience topics, and control topics may also
appear live, but are not in the bag. All 15 core recorded topics use keep-last
depth 30, best-effort, volatile QoS overrides.

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
`COMPLETE`, and the controller does not parse its output. The default bag
contains six `sensor_msgs/msg/Image`, three `sensor_msgs/msg/CameraInfo`, three
`vt_camera_msgs/msg/CameraFrameTiming`, and three
`vt_tracker_msgs/msg/TrackerSample` topics.

The bag intentionally excludes calibration, TF, raw RealSense metadata, session
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
