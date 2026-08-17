# Troubleshooting

Run diagnostics from a non-Conda shell after sourcing ROS 2 Jazzy and the same
workspace install used to launch the stack. Diagnostics help an operator assess
the system and data; they do not add quality gates to the Recorder lifecycle.

## Symptom guide

| Symptom | Checks | Interpretation and action |
| --- | --- | --- |
| `ros2 topic list` shows no camera topics | Confirm that the three RealSense processes, `timing_normalizer`, and `capture_controller` are running. Check `ROS_DOMAIN_ID`, `ros2 node list`, USB visibility, and launch logs. | Source ROS 2 before the workspace overlay, use the same domain in every shell, restart discovery if necessary, and relaunch after all three cameras are visible. |
| RViz has no Color image | Add an RViz **Image** display for one exact `*/color/image_raw` topic. Set reliability to **Best Effort** and inspect `ros2 topic info -v`. | A best-effort publisher can be invisible to an incompatible reliable subscriber. An Image display does not require a TF fixed frame. During formal recording, do not run three high-resolution RViz Image displays; their extra subscriptions and rendering load can distort the observation. |
| `Incomplete timing group ... missing color observation` or `Incomplete timing group ... missing depth observation` | Compare color/depth metadata rates, the per-camera `/frame_timing` rate, endpoint QoS, launch logs, and `lsusb -t`. | One color/depth metadata pair did not complete within 150,000,000 ns or four newer stamps, so that timing group was logged and omitted. Isolated or recurring messages are informational Recorder diagnostics: they do not mark a recording failed or prevent `COMPLETE`. Investigate source, QoS, driver, and USB load, then judge data fitness separately. |
| D436 clock reset or timestamp discontinuity | Check the D436 driver log, reconnect history, kernel USB messages, and adjacent `CameraFrameTiming` device timestamps. | A D436 clock reset is an informational diagnostic and does not mark the recording failed. It can make timing unsuitable for a particular analysis, so preserve the log context and inspect the affected interval independently. |
| Recorder stays in `FINALIZING` | Inspect `recorder.log`, process state, filesystem errors, and available space. | The controller has not confirmed Recorder termination. Do not kill the launch shell blindly; allow its stop/kill escalation to finish, then investigate the filesystem or process problem. |
| `COMPLETE` but data looks incomplete | Run `ros2 bag info`, inspect topic counts/durations, and sample messages using normal rosbag tools. | `COMPLETE` says only that the Recorder process lifecycle completed. It is not evidence of frame coverage, timing quality, or experimental suitability. |
| Tracker cards are all red although the devices are green | Confirm the Publisher is running, all terminals use the same `ROS_DOMAIN_ID`, and `/vive/*/status` exists. Check that no bootstrap, validator, or second Publisher owns the Dongle. | Green hardware LEDs show radio/mapping readiness, not that this ROS process is receiving reports. Keep working hardware powered; correct the process/domain or private-input problem first. |
| Host publishes but one Client does not | Echo all three `/vive/<role>/status` topics, inspect tracker IDs and counters, then run `vt-vive-validate-topics`. | The Publisher degrades per role and should continue publishing healthy roles. A three-role acceptance run still fails until the missing role recovers. Do not automatically rewrite the role map. |
| Tracker becomes blue-flashing only after a program starts | Stop all Dongle owners and verify that the approved manual bootstrap has exited before starting the read-only Publisher. | Competing HID owners or an unapproved startup sequence can disturb the session. The ROS visualizers themselves are read-only consumers and do not own the Dongle. |
| GUI motion is visibly choppy while topic rate is healthy | Compare each GUI card's receive Rate with `/vive/<role>/pose` rate and distinguish it from the header render FPS. | Restart only the GUI for a display-only problem. Do not reset or power-cycle a valid three-role Publisher session. |

## Graph, process, and discovery commands

```bash
test -z "${CONDA_PREFIX:-}"
type -a ros2
printenv ROS_DISTRO ROS_DOMAIN_ID AMENT_PREFIX_PATH
pgrep -a -f 'realsense2_camera|timing_normalizer|capture_controller|ros2 bag'
ros2 daemon stop
ros2 daemon start
ros2 node list
ros2 topic list
ros2 service list -t
find -L "${ROS_LOG_DIR:-$HOME/.ros/log}/latest" \
  -maxdepth 2 -type f -print
lsusb -t
rs-enumerate-devices -s
journalctl -k -b --no-pager | tail -n 200
```

All shells participating in one run must use the same `ROS_DOMAIN_ID` and
must source `/opt/ros/jazzy/setup.bash` before the selected workspace install.

## Image and timing topic commands

Use sensor-data QoS for image and timing diagnostics:

```bash
ros2 topic info -v /d405_1/color/image_raw
timeout 10 ros2 topic hz /d405_1/color/image_raw
ros2 topic echo --once /d405_1/color/image_raw sensor_msgs/msg/Image \
  --qos-profile sensor_data --no-arr

ros2 topic info -v /d405_1/frame_timing
timeout 10 ros2 topic hz /d405_1/frame_timing
ros2 topic echo --once /d405_1/frame_timing \
  vt_camera_msgs/msg/CameraFrameTiming --qos-profile sensor_data

ros2 topic info -v /d405_1/color/metadata
timeout 10 ros2 topic hz /d405_1/color/metadata
ros2 topic info -v /d405_1/depth/metadata
timeout 10 ros2 topic hz /d405_1/depth/metadata
```

Repeat for `d405_2` and `d436`. Image and timing publishers use
best-effort, volatile sensor-data QoS. A lower timing rate indicates that some
color/depth metadata observations did not form complete groups; the Recorder
continues independently.

## USB identity and profile commands

```bash
lsusb -t
rs-enumerate-devices -s
ros2 service list -t
for camera in d405_1 d405_2 d436
do
  ros2 service call "/$camera/device_info" \
    realsense2_camera_msgs/srv/DeviceInfo '{}'
  ros2 param get "/$camera" depth_module.depth_profile
  ros2 param get "/$camera" depth_module.depth_format
done
ros2 param get /d405_1 depth_module.color_profile
ros2 param get /d405_1 depth_module.color_format
ros2 param get /d405_2 depth_module.color_profile
ros2 param get /d405_2 depth_module.color_format
ros2 param get /d436 rgb_camera.color_profile
ros2 param get /d436 rgb_camera.color_format
journalctl -k -b --no-pager | tail -n 200
```

Use `lsusb -t` to confirm `5000M` links and place the D436 on an independent
root bus. A USB path is transient diagnostic evidence; the configured serial
number binds a launch instance to a camera.

## Session inspection

```bash
export VT_DATA_ROOT="${HOME}/visiontactile-data"
read -r -p "Session ID to inspect: " SESSION_ID
sed -n '1,240p' "${VT_DATA_ROOT}/${SESSION_ID}/recorder.log"
sed -n '1,240p' "${VT_DATA_ROOT}/${SESSION_ID}/bag/metadata.yaml"
ros2 bag info "${VT_DATA_ROOT}/${SESSION_ID}/bag"
```

`ros2 bag info` is an optional operator check. Its result never changes a
`COMPLETE` state. Treat lifecycle status and data-quality assessment as
separate questions.
