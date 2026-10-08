# VIVE Ultimate Tracker Monitor

`vt_vive_tracker_gui` is a standalone, read-only ROS consumer for an
already-running `vt_vive_tracker` Publisher. It subscribes to the fixed `left_wrist`,
`right_wrist`, and `torso` pose and status topics and never owns the Tracker
hardware.

## Launch

Build the workspace first, then launch the installed GUI from a sourced
terminal:

```bash
cd /absolute/path/to/VisionTactile-Dataset
source /opt/ros/jazzy/setup.bash
export VT_REPO="$(pwd -P)"
source "${VT_REPO}/ros2_ws/install/setup.bash"
ros2 launch vt_vive_tracker_gui tracker_gui.launch.py
```

该命令只是 ROS 2 只读订阅端：不会打开 Dongle，不会调用 PyVUT，不会配对或建图，不会开始录制，也不会启动、停止或重启 Tracker Publisher。

The Publisher must already be running in the same ROS domain. Closing or
restarting this window does not affect it.

## Reading the dashboard

Role identity is fixed everywhere in the window: `left_wrist` is cyan,
`right_wrist` is magenta, and `torso` is orange. Each model has local axes that
rotate with its quaternion: X is red, Y is green, and Z is blue. Its position
history is a three-second trail in the role color.

All poses are displayed in the Publisher's `vive_map` frame. Position x/y/z is
in metres. Quaternion x/y/z/w is the unitless ROS XYZW quaternion. RPY r/p/y is
roll/pitch/yaw in degrees.

The independent health color and label mean:

- green `FRESH`: the latest tracking pose is no more than 250 ms old;
- yellow `DELAYED`: the pose is over 250 ms but no more than 1000 ms old, or
  status is connected but not currently tracking/valid;
- red `OFFLINE`: no pose has arrived, the pose is over 1000 ms old, or status
  says the role is disconnected.

The overall state is `LIVE` only when all three cards are `FRESH`, `DEGRADED`
when their health is mixed, and `DISCONNECTED` when all are `OFFLINE`. The FPS
value is the GUI render rate, while each card's Rate is its recent ROS pose
receive rate.

## Camera controls

The five buttons select 俯视 (top), 前视 (front), 侧视 (side), 适应全部
(fit all current poses), and 重置视角 (reset the default camera). Drag with the
left mouse button to orbit, use the mouse wheel to zoom, and double-click the
scene to reset the camera.

On startup the diagnostic bar waits for ROS 2 tracker data and all cards remain
red `OFFLINE` until messages arrive. If input pauses, cards age through
`DELAYED` to `OFFLINE`; the last pose may remain visible but is gray when
offline. The GUI keeps waiting and returns to current data automatically when
the existing Publisher resumes. Do not reset hardware or restart the Publisher
to correct a GUI-only display problem; close or restart only this GUI and record
the defect.
