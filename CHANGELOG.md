# Changelog

All notable changes to this project are documented here.

## [Unreleased]

These changes describe the current source development preview. Component
versions such as the workbench's `0.5.0` do not establish a published project
release or a completed hardware acceptance run.

### Added

- Portable device-configuration defaults and a bilingual Detect Local Environment
  action. The workbench finds installed dependencies in documented locations,
  reports missing or ambiguous paths, and creates the default data directory
  when saving. Existing device identities and serial assignments remain explicit.
- Matching Chinese/English new-computer deployment guides with explicit preview-branch checkout, ROS setup, fixed RealSense source dependencies, USB/serial permissions, first recording checks, and migration steps. Complete Chinese capture, troubleshooting and contributor guides, and an English Windows build guide.

- Chinese/English switching in the independent tactile viewer, with separate
  saved preferences and no serial or renderer restart. The Windows installer
  and usage guide are bilingual; standalone release versions now come from
  `tools/tactile_viewer_windows/product.toml` (0.5.1), preserving the upgrade AppId.
- Windows x64 installer tooling for the independent tactile viewer: native COM
  access, bundled static-hand loading without a model Python runtime, pinned
  PyInstaller/Inno builds, installation diagnostics, and native Windows CI.
  Restricted model assets remain explicit private inputs. Windows hardware and
  installer acceptance are separate from source and Linux verification.
  Public source preparation includes the build tooling, not a model bundle or
  a redistribution-authorized Windows binary release.
- Embedded camera–Tracker calibration under Calibration & Offline Tools, with
  guided fixed-board acquisition, three independently verified rounds, automatic
  solving, immutable active versions and audited restoration. The backend owns
  the dedicated Recorder and worker; IPC v6 carries bounded progress and preview
  snapshots while retaining v1–v5 compatibility. A typed calibration Topic
  publishes version, device identity and expiring validity. Offline CLI workflows
  remain available; physical guided accuracy and endurance require acceptance.
- Explicit IPC v5 capture plans derived from the configured Recorder allowlist,
  with monitored-stream checks before acceptance and dispatch, frozen trial
  expectations for quality checks, and a bilingual recording-page summary.
  Unobserved inputs and tactile readiness proxies remain explicit; legacy
  protocols keep their schemas and planned requests never silently downgrade.
- Inline archive playback for partial bags: recorded-topic inventory, missing-stream
  notices, bag-time seek and speed controls, sensor previews and bounded message
  inspection. Missing Trackers or calibration and quality warnings do not block
  existing raw streams; playback neither publishes messages nor changes reports.
- Archive replay now shares the acquisition focus stage, device cards and spatial
  view, with one historical timeline across up to 24 visual streams. The spatial
  view shows recorded Tracker poses in a common frame; camera geometry and point
  clouds remain outside this compatibility view. Live monitoring stays independent.
- Recording capacity protection: START checks the planned image-volume budget,
  message/container allowance and stop reserve; recording polls free space and
  requests an orderly stop on low capacity or a failed query. Additional stream
  rates require an explicit estimate. Stop reasons are retained in trial records
  and available in Chinese and English.
- Hardware OpenGL camera preview with renderer diagnostics and a CPU fallback
  when a hardware context is unavailable.
- A joint spatial page and independent enlarged window for three Trackers in
  their native common world frame, calibrated camera bodies/frusta, source-time
  trails, layer controls, independent freeze, and explicit synthetic demo.
- Bounded, independently versioned read-only scene IPC; camera geometry requires
  verified device identity and matching calibration instead of guessed poses.
- A separate RGB-D point-cloud worker with bounded binary IPC, source-time pose
  interpolation, per-camera display controls, and recorded geometry sidecars.
- Offline spatial playback of closed ROS bags and saved alignment results, with
  background decoding, playback/seek controls, and explicit missing-geometry states.

- A v0.5 development-preview desktop workbench with an isolated Qt installer,
  hardware-free demo, graphical device setup, explicit device controls,
  persistent trials, and post-record quality reports.
- Camera, Tracker, glove, and tactile previews with independent enlarged
  windows; closing a preview does not change sensor or Recorder state.
- A Tracker initialization wizard that persists local input paths and the
  approved capture fingerprint, but not approval or initialized state. Reading
  starts only on a separate request after initialization completes.
- Independent left/right FSGlove pose bridges and tactile acquisition, with
  explicit recording extensions and typed offline validity/continuity checks.
- Workbench dynamic MANO hand preview with skeleton fallback and per-hand
  calibration-window access, plus 2D/matrix/static-MANO tactile views. Model
  files and original FSGlove source remain external prerequisites.
- Hardware-free workbench lifecycle, persistence, preview, and manager tests,
  and software tests for glove/tactile transport and reconstruction boundaries.
- Clean-clone desktop setup, optional-hardware guidance, and public source
  preparation instructions with private-input and Git-history boundaries.
- A standalone unified-bag offline alignment package with strict topic/type and
  identity gates, host-clock audit, ordered three-camera matching, bounded
  three-Tracker interpolation, external-calibration composition, generic future
  stream adapters, and atomic JSON/JSONL/CSV/SVG output validation.
- An independent offline Tracker-to-RealSense hand-eye calibration package
  with ChArUco generation/detection, rosbag2 time correlation, five OpenCV
  solvers, holdout validation, immutable YAML/JSON/CSV/SVG outputs, and
  synthetic known-transform tests.
- A single-entry product calibration runbook plus immutable configuration
  generation and three-run identity/repeatability comparison commands.
- ROS-compatible NumPy and OpenCV dependency bounds for the standalone
  calibration environment.
- A product-oriented end-user manual, complete launch/topic/CLI interface
  reference, and maintainer release checklist.
- Third-party dependency and license-boundary notices.
- Relative Markdown link validation in the public-tree contract.

### Fixed

- Public-source checks now reject compiled installers/libraries and exported
  static-hand geometry even when renamed. Vendored installer text keeps its
  original CRLF bytes without failing the source whitespace check.
- Qt test teardown now destroys each test's windows and processes deferred
  deletion, preventing language/style tests from accumulating hidden interfaces.
- Public documentation uses portable paths and consistent guided-calibration,
  tactile-viewer and recording-contract descriptions in both READMEs.
- Reconcile a definitively failed Recorder start as a failed trial, preserving
  its session directory without blocking the next explicit START. Keep uncertain
  process ownership and controller changes unresolved until confirmed.
- Check all Tracker poses and observation gaps inside each calibration stability
  window, and require each selected pose to differ from every previously selected
  pose. An outward-and-return motion or a repeated earlier pose can no longer
  pass through endpoint-only or any-pose comparisons.
- Audit each calibration input stream's host clock continuity before sorting;
  reject resets, reordered records and realtime/monotonic step disagreements
  above 5 ms without adding a dependency on the separate alignment tool.
- Release point-cloud objects when sources disappear or change, and withdraw
  cloud demand from minimized or frozen spatial windows while preserving demand
  from other viewers of the same backend.
- Keep the shared data refresh at 100 ms when an independent camera, Tracker,
  glove, tactile or spatial window is foreground; use the 500 ms background
  interval only when none of the data windows is active. These are UI polling
  intervals, not sensor or recorded frame-rate guarantees.
- Paint camera previews into the OpenGL framebuffer during widget snapshots
  and page transitions, avoiding image artifacts and permanent CPU fallback.
- Normalize `32FC1` depth-preview contrast independently of metre versus
  millimetre units, including narrow near-field ranges; recorded images remain
  unchanged.
- Forward termination signals received during the desktop launcher's child-process
  handoff, preventing an orphan session when a child becomes ready immediately.

### Changed

- Use optional vectorized finite-coordinate validation for point-cloud IPC,
  retaining little-endian decoding, input bounds and a dependency-free fallback.
- Clarify that workbench Receive Hz measures manager reception, while recording
  completion and post-recording quality are separate results. The reserved
  write-rate field does not yet provide a throughput measurement.
- Cache ambient illumination and update only moving decoration regions to
  reduce desktop motion rendering cost; inactive windows pause decorations.
- Public source preparation separates source-only validation from Git-history,
  third-party permission and hardware acceptance checks.
- **Breaking:** the default `unified-dataset-v2` MCAP contract is one unified
  bag with 21 candidate topics: six images, six `CameraInfo`, three factory
  depth-to-color `Extrinsics`, three `CameraFrameTiming`, and three
  `TrackerSample` streams, plus only explicitly configured extensions.
  Existing `unified-dataset-v1` plans retain their original 15-topic contract.
- **Breaking:** `COMPLETE` now reports only that the Recorder process
  lifecycle finished; it makes no recorded-data quality claim.
- Capture control now follows the direct
  `IDLE -> RECORDING -> FINALIZING -> COMPLETE` workflow.
- Runtime recording depends on the ROS 2 Jazzy `ros2bag` CLI.
- Public-tree validation now accepts the pinned `pyvut` submodule while
  rejecting packet captures, ZIP output, and local authorization screenshots.
- CI now builds and tests the standalone Tracker GUI package.
- Root documentation distinguishes the legacy standalone glove prototype from
  the ROS-connected FSGlove path and uses pytest for offline GUI/workflow tests.
- Current desktop features are documented as a development preview, not a
  completed hardware acceptance or commercial delivery. Left-hand mapping,
  pose accuracy, and full-rig endurance still require physical validation.

### Limitations

- The default recording contract contains 21 candidate topics and records
  available configured streams; enabling a device preview does not automatically
  add topics to the recording allowlist. Legacy v1 plans retain 15 candidates.
- Workbench tactile colors show raw ADC, not pressure or baseline differences.
  Static 3D tactile and dynamic glove views are separate, not synchronized
  pose–tactile overlays.
- Licensed MANO models, extended Tracker runtime inputs, captured data, and
  per-machine private configuration are not distributed with the source.
- The public PyVUT pin lacks a license file and the production Tracker path
  requires extensions absent from that pin. FSGlove numerical source and MANO
  models must be obtained separately under their own terms. A source-only
  clone cannot reproduce the full sensor rig without these prerequisites.
- Capacity protection does not measure write throughput or establish frame
  completeness. Real-time recorded-frame/drop statistics and full-rig sustained
  30 Hz recording acceptance remain incomplete.

### Removed

- Runtime camera-health, timing-quality, storage-throughput, and post-recording
  content gates from the Recorder lifecycle. Free-capacity checks remain active
  through the recording capacity protection described above.
- Recorded calibration, TF, raw RealSense metadata, session, command, status,
  and event topics.
- Real-time rosbag and MCAP compression from the capture path.

## [0.1.0] - 2026-07-19

### Added

- Triple RealSense ROS 2 Jazzy launch for two D405 cameras and one D436.
- Camera-group timing, fail-closed capture control, MCAP recording, and validation.
- Synthetic end-to-end tests and bilingual public documentation.

### Changed

- Camera identities are configurable for the fixed D405/D405/D436 topology.

### Removed

- Unimplemented alignment and legacy per-stream timing message interfaces.
