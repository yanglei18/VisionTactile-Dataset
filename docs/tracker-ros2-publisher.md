# VIVE Ultimate Tracker ROS 2 Publisher

This workflow publishes three already mapped VIVE Ultimate Trackers on Linux.
Windows mapping is mandatory before the first Linux session. The ROS node is
read-only: it does not pair Trackers, change maps, send configuration commands,
or run the manual startup sequence.

## Prerequisites

- Complete the Windows map and verify all three Trackers there.
- Keep the approved private capture and generated bundle outside this Git
  repository.
- Initialize the repository's pinned `third_party/pyvut` submodule.
- Assign the physical dataset roles `left_wrist`, `right_wrist`, and `torso`
  before writing the private role map.

The pinned PyVUT submodule contains the extended `live_*`, bundle, HID, and
pose-decoder modules imported by this Publisher. Device-specific packet
captures, bootstrap bundles, Wi-Fi credentials, and role maps remain private
runtime inputs and must never be committed.

The protocol labels Host, Client 0, and Client 1 are session labels, not
permanent physical identities. The private role map uses stable SHA-256
identifiers so a Windows re-pair that only changes RF slot bits does not swap
ROS topics.

## Build

From the repository root:

```bash
export VT_REPO="$(git rev-parse --show-toplevel)"
export VT_WS="${VT_REPO}/ros2_ws"
source /opt/ros/jazzy/setup.bash
cd "${VT_WS}"
colcon build --event-handlers console_direct+
source "${VT_WS}/install/setup.bash"
```

Install the pinned submodule into an isolated virtual environment and make it
importable by the system Python used by ROS 2:

```bash
git -C "${VT_REPO}" submodule update --init --recursive
export VT_PYVUT_ROOT="${VT_REPO}/third_party/pyvut"
export VT_PYVUT_VENV="${HOME}/.local/share/visiontactile/pyvut-venv"
test "$(git -C "${VT_PYVUT_ROOT}" rev-parse HEAD)" = \
  "7da6b081ad4ebfa0a0f2f242cdecb9ffc47341f1"
python3 -m venv "${VT_PYVUT_VENV}"
"${VT_PYVUT_VENV}/bin/pip" install -e "${VT_PYVUT_ROOT}[pcap]"
export VT_PYVUT_SITE="$("${VT_PYVUT_VENV}/bin/python" -c \
  'import site; print(site.getsitepackages()[0])')"
export PYTHONPATH="${VT_PYVUT_ROOT}:${VT_PYVUT_SITE}:${PYTHONPATH:-}"
test -f "${VT_PYVUT_ROOT}/pyvut/live_hid.py"
test -f "${VT_PYVUT_ROOT}/pyvut/live_bootstrap_bundle.py"
test -f "${VT_PYVUT_ROOT}/pyvut/pose_decoder.py"
```

## Manual startup, role map, and ROS node

Set the approved private inputs. The capture digest must be supplied
explicitly:

```bash
export VT_DATA_ROOT="${HOME}/visiontactile-data"
export VT_CAPTURE="${VT_DATA_ROOT}/private/vut/01_cold_reconnect.pcapng"
export VT_BUNDLE="${VT_DATA_ROOT}/private/vut/live-bootstrap.json"
: "${VT_CAPTURE_SHA256:?set VT_CAPTURE_SHA256 to the approved capture digest}"
source "${VT_WS}/install/setup.bash"
```

Run the verified manual startup process first. It is the only process in this
workflow authorized to perform the capture-locked device writes:

```bash
"${VT_PYVUT_VENV}/bin/python" \
  "${VT_PYVUT_ROOT}/scripts/live_windows_bootstrap.py" \
  --capture "${VT_CAPTURE}" \
  --expected-sha256 "${VT_CAPTURE_SHA256}" \
  --bundle "${VT_BUNDLE}" \
  --execute-feature-writes
```

Wait for this exact terminal evidence before continuing:

```text
status=COMPLETE failure=none context=none last_sequence=51 feature_out=51 feature_get=0 close=1
```

Do not run that process and the ROS node at the same time.

Write the private physical-role map:

```bash
mkdir -p "${VT_DATA_ROOT}/tracker-validation"
ros2 run vt_vive_tracker vt-vive-write-role-map \
  --bundle "${VT_BUNDLE}" \
  --host torso --client0 left_wrist --client1 right_wrist \
  --output "${VT_DATA_ROOT}/tracker-validation/roles.yaml"
```

This command is normally run once. Run it again only after an intentional
physical-role change or after the node reports an identity mismatch. Windows
re-pairing does not by itself justify silently changing dataset roles.

Launch the read-only publisher:

```bash
ros2 launch vt_vive_tracker triple_tracker.launch.py \
  bundle_path:="${VT_BUNDLE}" \
  role_map_path:="${VT_DATA_ROOT}/tracker-validation/roles.yaml"
```

It publishes exactly three topics per role:

```text
/vive/left_wrist/sample
/vive/left_wrist/pose
/vive/left_wrist/status
/vive/right_wrist/sample
/vive/right_wrist/pose
/vive/right_wrist/status
/vive/torso/sample
/vive/torso/pose
/vive/torso/status
```

`sample` and `pose` use best-effort sensor QoS. `status` uses reliable,
transient-local QoS. The frame ID `vive_map` means the native coordinate frame
created by Windows mapping; it does not claim alignment with a camera, robot,
world frame, or ROS ENU. No TF is published.

## RViz visualization

Keep the publisher running and open a second sourced terminal:

```bash
source /opt/ros/jazzy/setup.bash
source "${VT_WS}/install/setup.bash"
ros2 launch vt_vive_tracker tracker_visualization.launch.py
```

This command is a ROS-only consumer. It does not start or stop the Tracker
publisher, open the Dongle, run bootstrap, pair Trackers, or change the map.
Closing RViz therefore leaves the hardware reader and its topic counters
running.

Role identity remains visible through fixed colors: `left_wrist` is cyan,
`right_wrist` is magenta, and `torso` is orange. Each role shows an orientation
arrow, a three-second position trail, recent receive Hz, and the latest valid,
invalid, and queue-drop counters.

The small health marker and label use these colors:

- green: a `TRACKING` pose arrived within 250 ms;
- yellow: input is 250--1000 ms old, or the status reports connected without
  tracking or invalid data;
- red: no pose has arrived, input is older than 1000 ms, or the role is
  disconnected.

A stale pose is dimmed and never remains green. If a role has never provided a
pose, its red `OFFLINE` sphere and label appear on a diagnostic row near the
`vive_map` origin; this is expected and makes a complete input outage visible.

## Standalone desktop monitor

The installed `vt_vive_tracker_gui` package provides a dashboard that can run
beside RViz or on its own. Keep the existing Publisher running and use a second
terminal:

```bash
source /opt/ros/jazzy/setup.bash
source "${VT_WS}/install/setup.bash"
ros2 launch vt_vive_tracker_gui tracker_gui.launch.py
```

该命令只是 ROS 2 只读订阅端：不会打开 Dongle，不会调用 PyVUT，不会配对或建图，不会开始录制，也不会启动、停止或重启 Tracker Publisher。

The GUI consumes the already-published pose and status topics in the current
ROS domain. Closing or restarting the GUI leaves the Publisher, Tracker power
states, Dongle ownership, RViz, and other processes unchanged.

Role colors are fixed: `left_wrist` is cyan, `right_wrist` is magenta, and
`torso` is orange. The model for each role shows local X/Y/Z axes in
red/green/blue that rotate with the pose, plus a three-second position trail in
the role color. All values remain in the native `vive_map` frame. Position
x/y/z is measured in metres, quaternion x/y/z/w is the unitless ROS XYZW
quaternion, and RPY r/p/y is roll/pitch/yaw in degrees.

Health colors describe each role independently:

- green `FRESH`: a tracking pose arrived no more than 250 ms ago;
- yellow `DELAYED`: pose age is over 250 ms but no more than 1000 ms, or the
  status is connected without current tracking/valid data;
- red `OFFLINE`: no pose has arrived, pose age is over 1000 ms, or the role is
  disconnected.

The header reads `LIVE` only while all three roles are `FRESH`, `DEGRADED` for
mixed health, and `DISCONNECTED` when all are `OFFLINE`. Header FPS measures
GUI redraws; each card's Rate is its one-second ROS pose receive rate.

The five camera buttons are 俯视 (top), 前视 (front), 侧视 (side), 适应全部
(fit all received poses), and 重置视角 (reset the default camera). Left-button
drag orbits, the mouse wheel zooms, and a double-click also resets the camera.

At startup the GUI reports that it is waiting for ROS 2 tracker data. Cards stay
red `OFFLINE` until messages arrive. A pause ages a role from `FRESH` through
`DELAYED` to `OFFLINE`; an offline last pose is gray rather than falsely green.
The subscriber continues waiting and recovers automatically as soon as the
existing Publisher's messages resume. For a GUI-only display problem, do not
reset a Tracker, replug the Dongle, or restart the Publisher: record the defect
and close or restart only the GUI.

## Thirty-second acceptance

In a second sourced terminal, move each physical Tracker separately during the
30-second window:

```bash
source /opt/ros/jazzy/setup.bash
source "${VT_WS}/install/setup.bash"
ros2 run vt_vive_tracker vt-vive-validate-topics --duration 30 \
  --output "${VT_DATA_ROOT}/tracker-validation/tracker-ros2-30s.json"
```

Passing output is:

```text
status=PASS roles=3 identity_swaps=0 dropped=0
```

Every role must provide at least 30 valid poses per second, at least 90%
complete tracking, no gap above 100 ms, exact host realtime/header agreement,
strictly increasing host monotonic receive time, stable unique identity, and
zero queue drops. The JSON report is private mode `0600` and must remain
outside the Git worktree.

## Failure handling

- No samples within three seconds: stop the ROS node, then run the approved
  manual startup process once.
- `DISCONNECTED`: check power, Dongle ownership, USB, and process state. The
  node only re-enumerates and reopens read-only.
- `INVALID_DATA`: inspect status counters; another role continues independently.
- Identity mismatch or collision: stop and verify the physical role assignment.
  Do not rename topics or regenerate the map automatically.
- Never clear the Windows map, clear pairing, update firmware, or send unknown
  commands as an automatic recovery action.

See the [interface reference](interface-reference.md) for all Publisher launch
arguments, topic types, QoS profiles, and coordinate-frame limits.
