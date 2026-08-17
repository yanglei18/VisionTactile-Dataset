# Changelog

All notable changes to this project are documented here.

## [0.2.0] - 2026-08-17

### Added
- Read-only ROS 2 publication for three mapped VIVE Ultimate Trackers,
  including stable physical-role mapping and 30-second topic acceptance.
- RViz2 markers and a standalone desktop monitor for the three Tracker roles.
- Linux direct-USB and wireless-Dongle validation tooling.
- A product-oriented end-user manual, complete launch/topic/CLI interface
  reference, and maintainer release checklist.
- Third-party dependency and license-boundary notices.
- Relative Markdown link validation in the public-tree contract.
- Reachable-history validation that prevents removed private artifacts from
  remaining recoverable in a public release branch.
- Licensed PyVUT 0.2.0 submodule with capture-locked live three-Tracker
  bootstrap, fixed-role pose streaming, CI, and a sanitized one-commit history.

### Changed
- **Breaking:** the default MCAP contract is now exactly six image topics and
  three `CameraFrameTiming` topics, written without real-time compression.
- **Breaking:** `COMPLETE` now reports only that the Recorder process
  lifecycle finished; it makes no recorded-data quality claim.
- Capture control now follows the direct
  `IDLE -> RECORDING -> FINALIZING -> COMPLETE` workflow.
- Runtime recording depends on the ROS 2 Jazzy `ros2bag` CLI.
- Public-tree validation rejects packet captures, ZIP output, local
  authorization screenshots, and broken relative documentation links.
- PyVUT is pinned as a reproducible Git submodule; device-specific captures,
  bundles, credentials, and role maps remain external private inputs.
- CI now builds and tests the standalone Tracker GUI package.

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
