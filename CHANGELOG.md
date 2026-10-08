# Changelog

All notable changes to this project are documented here.

## [Unreleased]

### Added

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

### Changed

- **Breaking:** the default MCAP contract is now one unified bag containing six
  image, three `CameraInfo`, three `CameraFrameTiming`, and three
  `TrackerSample` topics, plus only explicitly configured extensions.
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

- The default recording contract remains the 15 core Topics; enabling a device
  preview does not automatically record its data.
- Workbench tactile colors show raw ADC, not pressure or baseline differences.
  Static 3D tactile and dynamic glove views are separate, not synchronized
  pose–tactile overlays.
- Licensed MANO models, extended Tracker runtime inputs, captured data, and
  per-machine private configuration are not distributed with the source.

### Removed

- Runtime camera-health, timing-quality, storage-throughput, free-capacity, and
  post-recording content gates.
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
