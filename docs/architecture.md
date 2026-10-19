# Architecture

## Scope

The public pipeline is a unified ROS 2 recording workflow for a fixed
two-D405/one-D436 and three-Tracker topology. The RealSense nodes publish live
streams, `timing_normalizer` groups each camera's color and depth metadata, the
read-only Tracker publisher emits three role streams, and `capture_controller`
owns one `ros2 bag record` child process.

Recorder process lifecycle is the only completion criterion. The controller
checks available storage against its configured recording budget and stop
reserve. It does not certify camera identity, stream health, storage throughput,
frame drops or recorded-data quality. The manager's planned-START checks are
described separately below.

## Package responsibilities

| Package | Responsibility |
| --- | --- |
| `vt_camera_msgs` | Defines command, status, event, session, compatibility status fields, and `CameraFrameTiming` interfaces. |
| `vt_realsense_capture` | Loads the fixed topology, launches the official RealSense nodes, groups per-camera timing metadata, starts and stops the Recorder, and publishes lifecycle status. |
| `vt_vive_tracker` | Publishes three read-only Tracker role streams in the native `vive_map` frame. |
| `vt_capture_core` | Defines ROS/Qt-independent Recipes, immutable resolved capture plans, Topic fingerprints and versioned IPC contracts. |
| `vt_capture_manager` | Persists recording intents and frozen plan evidence, checks monitored inputs before planned START, and schedules read-only post-record quality checks. |
| `tools/capture_console` | Presents configured plans and shared live/archive views; submits explicit user operations through negotiated IPC. |
| `tools/tracker_camera_calibration` | A separately installed worker guides dedicated acquisition and reuses the completed-bag solver for `tracker_from_camera`; CLI/offline use remains available. It does not own active configuration or calibration publication. |
| `tools/multisensor_alignment` | Reads one completed unified bag, audits clocks and identities, aligns cameras and Trackers in host realtime, applies external calibration, and exports an immutable extensible index. |

## Shared capture and archive presentation

The Qt console imports reusable device cards from
`vt_capture_console.ui.widgets.device_card`. Shared widgets must not import
`ui.main_window`; an architecture test enforces that dependency direction.
`MainWindow.DeviceCard` remains a compatibility import for existing callers.
New integrations should use the shared module directly.

`DeviceCard.update_device()` accepts a complete presentation snapshot.
`update_metadata()` updates the same sample's labels and ages without decoding
again; a changed identity, status, context or payload must use the full update.
The `device_snapshot` property returns an isolated copy, so callers never need
to write a card or preview's private fields. An embedded archive card normalizes
its input to historical presentation, including empty samples, and emits a
selection request instead of opening another window.

`PreviewWidget` owns decoded frames, renderers and display-cache comparisons.
Consumers use its public state properties, `copy_view_state_from()` and existing
display-copy methods. `clear_frame()` discards cached measurements and updates
renderers while retaining device identity and inspection preferences. Archive
seek/load boundaries clear these caches before accepting another snapshot.
Copying presentation does not create a subscription or control a sensor.

Consumers also use `CaptureStage.refresh_layout()` after presentation changes,
`DeviceControls.control_available` for the last refreshed control capability,
and `PreviewWidget.paint_history_overlay()` for the camera canvas. Spatial
backend changes clear samples through `SpatialViewModel.clear_frame()` and
request a later scene fit through `SpatialScene.reset_view_fit()`. Clearing
samples retains view preferences and the user's frozen/unfrozen choice.

The component-boundary tests derive private member names from the shared
components and check consumers across the console, including literal
`getattr`/`setattr` access. This is a targeted source guard, not Python type
analysis: foreign access to these private names is conservatively rejected.
Component implementation classes may copy their own caches; consumers use
public snapshots and lifecycle methods. Behavioral tests cover detached
snapshots, clearing frozen samples, layout refresh and historical overlays.

The archive overview reuses unchanged samples while updating their ages. Its
sample comparison includes payload content: equal timestamps alone do not
establish that two samples are identical. Preview and space continue to consume
the same published playback snapshot. This component boundary does not change
Recorder topic selection, quality policy or the database schema.

## Desktop operation coordination

`vt_capture_console.viewmodels.operations.OperationCoordinator` owns the single
pending management operation as an immutable `PendingOperation`. `submit()`
captures the current manager/controller identity and existing batch IDs before
calling the supplied sender once. `observe()` consumes snapshots and receipts;
it retains no backend reference and cannot send or retry commands.

Matching request and trial evidence determines completion. A queued or merely
persisted START remains pending. Rejected/unknown outcomes and a replaced
backend without matching trial evidence end local waiting without inventing a
Recorder outcome. Live batch creation does not infer ownership from newly
appearing rows; automatic selection is limited to the in-memory demo's single
new batch. These rules are tested without Qt or hardware.

The coordinator returns an `OperationUpdate` containing display text, optional
batch-selection changes and a detail-refresh request. `MainWindow` applies this
result to widgets; its compatibility `pending_*` properties are read-only views
of the coordinator, not a second state store. This separates command outcome
rules from table updates, page layout and language changes.

The shared IPC backend retains up to 128 per-request receipts, protecting queued
and in-flight commands from eviction. Recording, device and Tracker consumers
query their own request through `operation_for`; a different command cannot hide
its outcome. Expired receipts explicitly become unknown, without resending a
command. `latest_operation` remains a compatibility view, not the source used to
associate concurrent callers with results.

## Playback input and decoder lifecycle

Topic and spatial playback use the same `ReplayDecoder` transport. Each owned
process is a session with increasing request IDs; responses must match the ID
and requested topic/time or spatial frame index. Late success and error replies
are discarded within the current request's deadline. Cancelling a selection also
invalidates its in-flight frame. A newly loaded source remains locally owned
until the first validated frame is published; failure, cancellation and shutdown
close it even before the backend has adopted it.

`vt_multisensor_alignment.bag_input` validates both bag paths. Metadata is limited
to 4 MiB, the storage list to 4096 entries, and files must be regular files with
no symlinks in their path components. Descriptor-relative no-follow traversal
and bounded reads prevent metadata checks from following a swapped link. Nested
regular storage files remain supported. File fingerprints include device, inode,
mode, size, mtime and ctime. Bags must remain closed and immutable while reading:
native ROS storage libraries reopen paths themselves, and the decoder process
is not an OS sandbox or a hard memory quota for hostile input.

## Durable recovery and background ownership

A new trial cannot reuse the START ID of the observed controller session.
Initial session ownership also requires a revision newer than the persisted
START guard; a request-ID conflict cannot claim an old recording. This does not
rewrite historical completed trials. Temporary SQLite failures pause quality
scheduling with an observable error and bounded backoff. Completed results are
retained for the same attempt and committed without rerunning its subprocess.
If the manager exits before durability, the existing restart recovery creates
a separate read-only attempt and preserves previous report files.

Serial profiles retain their configured aliases. Each explicit start resolves
them again and freezes the same actual resources into preflight, launch arguments
and the process journal. Active groups keep their original resource identity.
Log preparation precedes the durable possible-launch marker, which still precedes
`Popen`; a known failure before fork cannot leave an orphan ownership hint.

Desktop-managed log retention runs inside the manager, independently of the GUI.
The `--log-path` option validates a private regular file against the existing
stdout inode and requires append output. Every 250 ms it checks for more than
5 MiB and retains the last 1 MiB. This is periodic retention, not a disk quota;
writes may briefly exceed the threshold and truncation may drop concurrent log
bytes. Closing the GUI leaves both the manager and its retention thread running.
Manual CLI startup without the option keeps its existing logging behavior.

## Explicit capture plans

`vt_capture_core.capture_plan` resolves Recipe v1/v2 into an immutable candidate
allowlist of topics, declared types and device identities. Recipes can describe
any nonempty subset of the supported cameras, Trackers and hand streams; a
configured camera retains its complete six-topic bundle in Recipe v2. Recipe v1
retains the original four-topic bundle and `unified-dataset-v1` schema; new v2
recipes use `unified-dataset-v2`. Historical frozen plan versions and fingerprints
remain unchanged. Unknown extensions
are rejected instead of silently disappearing from the plan. The production
Recorder still uses its configured allowlist, including late-arriving sources.

IPC v5 requires the Recipe and an `expected_topics_fingerprint` for
`capture.start`. Instance, revision, duration and the exact configured allowlist
are checked before accepting an operation. Versions 1–4 keep their schemas;
planned operations cannot downgrade to a legacy START after a reconnect.

New frozen capture plans use version 2. The manager marks recently receiving
streams as required and absent candidates as optional. At least one monitored
source must be receiving; missing Trackers do not block a camera recording.
Arrival activity is independent of payload validity: invalid poses, malformed
messages and clock faults can still be recorded and inspected. CameraInfo, factory
extrinsics and frame timing are required when an image from that camera is receiving, although
the manager does not monitor those auxiliary topics. Tactile recording
activity observes raw samples separately from preview state heartbeats; a
disconnected status publisher does not establish sample availability. Runtime
message types are not verified; storage admission remains owned by the Recorder.

`capture_plan` and `capture_preflight` are frozen with the trial outside the
optional provenance summary's truncation. Immediately before dispatch, the
manager rechecks the accepted required streams. A source disappearing while
queued rejects START rather than silently weakening the saved expectation.
Optional sources can arrive later and are still recorded. The ROS command and
Recorder lifecycle are unchanged; unresolved outcomes remain unknown.

Quality checks use the frozen required/optional partition. Missing optional
streams are listed as coverage information, without a quality warning; any
actual messages, including late optional streams, receive the usual checks.
Required streams missing from the bag still warn. Camera rate checks remain
30 Hz for configured image streams with data. A changed finish allowlist emits
an advisory; corrupt plans fail the check. Historical version 1 plans retain
their strict all-required contract, and trials without plans retain their
original finish-context expectations. No historical metadata is rewritten.
Archive playback continues to use the data actually present in each bag.

## Data and control flow

```text
official realsense2_camera rs_launch.py x3
  -> six live Image topics
  -> six live CameraInfo topics
  -> three retained factory depth-to-color Extrinsics topics
  -> timing_normalizer -> three live CameraFrameTiming topics
read-only Tracker publisher -> three live TrackerSample topics
  -> capture_controller -> ros2 bag record -> uncompressed MCAP
```

The Recorder includes color and native-depth `CameraInfo` and each camera's
factory `extrinsics/depth_to_color`. Their acquisition does not depend on
Tracker or space-view configuration. They do not establish the external mounting
pose relative to another device or the world. The camera nodes also expose raw
RealSense metadata, TF and `device_info` services; those interfaces remain
outside the recording allowlist.

The optional calibration flow is deliberately separate:

```text
live color + CameraInfo + CameraFrameTiming + read-only TrackerSample
  -> bounded online guidance + owned dedicated calibration Recorder
  -> confirmed closed bag -> existing ChArUco PnP and hand-eye solver
  -> independent validation -> three-run repeatability
  -> immutable result -> manager active revision -> typed calibration topic
```

No calibration code executes in the production Recorder process, and a
missing or rejected external calibration cannot block raw camera recording.

The embedded Qt workspace submits version 6 commands through the existing IPC
client. It does not import ROS/OpenCV or control subprocesses. Its viewmodel maps
snapshots to one current instruction, progress and folded diagnostics. Shared
preview components are used through public APIs. Main-window navigation retains
the existing top-level page identities; calibration is a device sub-workspace
entered through **Calibration & Offline Tools**.

`CalibrationService` owns the worker, private session files and the active-result
store. Its commands and activation run under the same manager admission lock as
formal recording and device maintenance. The worker uses a frozen configuration,
bounded input buffers, explicit clock domains and the same pose selection and
motion-excitation rules as offline solving. Detection can skip preview frames;
acceptance is rechecked against the actual closed bag. Independent validation
poses do not participate in method selection or final fitting.

The active reference file is the durable commit point. Failures before commit
preserve the prior result. A post-commit load/publication failure remains
`APPLIED_PENDING_PUBLICATION` and is retried with the same revision. Restoring a
previous result creates a new audit revision after checking the expected current
revision, live identities and recording admission. Camera identity verification
and calibration publication work even when the spatial scene is disabled.
Formal trial start metadata freezes the active result documents and manifests;
later calibration changes do not rewrite that provenance.

标定页面复用主程序主题，入口保持“标定与离线工具”。界面只发送操作和展示快照；
原始数据订阅、录制、求解、验证和配置应用由独立 worker 与后台管理。三轮独立
采集均通过质量及重复性检查后自动应用。单个离线包只生成候选。失败候选不影响
已有有效标定；配置提交后发布未完成会显示待恢复状态。恢复旧结果生成新的审计
版本，正式录制时保存当时使用的外参与版本信息。

The production post-processing flow is also separate from capture:

```text
one unified MCAP + three VALID tracker_from_camera files
  -> topic/type/identity/host-clock audit
  -> maximum-coverage ordered camera matching + bounded Tracker interpolation
  -> aligned_frames.jsonl + manifest/catalog/residual/quality evidence
```

Capture never waits for this tool and the tool never modifies the bag.

## Unified 21 topics

The default Recorder allowlist contains exactly 21 candidate topics:

| Source | Topic suffix/pattern | Type | Count |
| --- | --- | --- | --- |
| Each of `d405_1`, `d405_2`, `d436` | `color/image_raw` | `sensor_msgs/msg/Image` | 3 |
| Each camera | `depth/image_rect_raw` | `sensor_msgs/msg/Image` | 3 |
| Each camera | `color/camera_info` | `sensor_msgs/msg/CameraInfo` | 3 |
| Each camera | `depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 3 |
| Each camera | `extrinsics/depth_to_color` | `realsense2_camera_msgs/msg/Extrinsics` | 3 |
| Each camera | `frame_timing` | `vt_camera_msgs/msg/CameraFrameTiming` | 3 |
| Each of `left_wrist`, `right_wrist`, `torso` | `/vive/<role>/sample` | `vt_tracker_msgs/msg/TrackerSample` | 3 |

This is **6 Image + 6 CameraInfo + 3 Extrinsics + 3 CameraFrameTiming +
3 TrackerSample = 21 topics**. The ordering is deterministic by camera name and Tracker role, every
name is absolute, and wildcards are rejected. Versioned additional streams are
sorted after the core and must declare their exact ROS message type.

External Tracker–camera calibration results, `/tf`, `/tf_static`, raw RealSense metadata,
Tracker `/pose` and `/status`, and `/capture/session_info` are not in the bag.
Capture command, status, and event topics are also outside the allowlist.

Factory extrinsics use **reliable, transient-local, keep-last depth 1** so
a late Recorder receives the driver's retained transform. Other core topics use
**keep-last depth 30, volatile** QoS overrides. The controller identifies the
six managed image sources from the camera contract and requests reliable
delivery. CameraInfo, timing, Tracker and optional sources remain best-effort by
default; operator overrides win. Reliability is not inferred from an arbitrary
topic's name or Image type. The MCAP writer configuration has `compression: None`; neither
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
