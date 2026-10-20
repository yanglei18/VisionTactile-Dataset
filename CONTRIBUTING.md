# Contributing

[简体中文](CONTRIBUTING.zh-CN.md) · [New computer deployment](docs/deployment.md)

Thank you for helping keep the public capture contract small, testable, and
truthfully documented.

## Development environment

- Develop and test on Ubuntu 24.04 with ROS 2 Jazzy.
- Use system Python for ROS nodes and builds. Keep the PySide6 workbench,
  offline tools, and optional PyQt5/FSGlove/MANO runtime in separate virtual
  environments; Conda must not be active in capture, build, or test shells.
- Use RealSense ROS 4.58.1 when a change depends on the production wrapper.
- Use the Jazzy `ros2bag` CLI for runtime recording.
- Work on a focused feature branch. Do not develop directly on a release
  branch.

Start from a recursive clone of `source-preview-20261019`, or run
`git submodule update --init --recursive` in an existing clone of that branch.
Preserve the pinned submodule revision. Follow the
[deployment guide](docs/deployment.md) for system packages, [ROS initialization](docs/deployment.md#ros),
[RealSense wrapper version selection](docs/deployment.md#realsense), USB access and workspace setup; it includes
the exact clone command. The [quick start](README.md#quick-start) provides the
shorter route for a prepared host.
Public software tests must not require local Tracker credentials, a physical
sensor, external FSGlove source, or licensed MANO assets. Keep optional tests
for those prerequisites explicit and report skips separately from passes.

## Change workflow

Write or update a failing test before every behavior change, confirm that it
fails for the intended reason, then implement the smallest change that passes.
Documentation-only corrections must still satisfy the public-tree contract.

Before opening a pull request:

1. Build and run the software tests:

   These commands use the deployment guide's source-built driver workspace.
   If using its separately verified APT route, omit the driver workspace
   `source` command.

   ```bash
   export VT_REPO="$(git rev-parse --show-toplevel)"
   test -z "${CONDA_PREFIX:-}"
   source /opt/ros/jazzy/setup.bash
   source "${HOME}/visiontactile-drivers_ws/install/setup.bash"
   cd "${VT_REPO}/ros2_ws"
   colcon build --event-handlers console_direct+
   source install/setup.bash
   PYTHONPATH="${VT_REPO}/tools/multisensor_alignment/src:${VT_REPO}/ros2_ws/src/vt_realsense_capture${PYTHONPATH:+:$PYTHONPATH}" \
     colcon test --event-handlers console_direct+
   colcon test-result --all --verbose
   cd "${VT_REPO}"
   ```

   Do not use a symlinked install for this workflow. Run the relevant isolated
   tool suites below as well: `unittest discover` does not collect all pytest
   GUI and workflow tests.
   The explicit test paths expose the offline replay parser and pure capture
   controller for cross-component regressions; they do not start ROS hardware.

2. Check the current release candidate, then stage only intended public files
   and run the index checks. The release check does not stage anything. The
   public-tree checker reads Git index blobs deliberately, so an unstaged edit
   cannot make an older staged contract appear current.

   ```bash
   python3 tools/release/prepare_release.py check
   git add -- path/to/changed-file
   python3 tools/test_check_public_tree.py
   python3 tools/check_public_tree.py
   git diff --cached --check
   ```

3. Update English and Chinese user documentation together whenever public
   behavior, commands, compatibility, or limits change.

4. Describe lifecycle states exactly as the software implements them. In
   particular, `COMPLETE` means Recorder process termination was confirmed;
   do not present it as proof of message coverage, timing quality, absence of
   drops, or experiment suitability.

Keep the change focused and explain its observable effect and software-test
results in the pull request. Hardware observations can be useful context, but
they are not required acceptance gates for the Recorder-only lifecycle.

## Console component boundaries

Keep command outcome rules in the Qt-independent operation coordinator and
domain rules in `vt_capture_core`; the main window applies presentation updates.
Shared widgets expose snapshots and lifecycle methods. Consumers must not read
their private caches or invoke private layout/rendering helpers. Extend the
existing public API when a new use case needs one, and add a behavioral test.
See [the architecture guide](docs/architecture.md#desktop-operation-coordination)
and `tools/capture_console/tests/test_shared_component_boundaries.py` for the
enforced dependency direction and targeted source guard.

Before recording a local development baseline, inspect tracked and untracked
files explicitly. Keep unrelated work identifiable in separate commits, and
run the public-tree check against the actual staged index. Ignored local
reference material stays outside that index; removing it from tracking must
not delete the operator's local copy. A clean current tree does not certify
older history for publication; follow the separate release preparation guide.

## Hardware-free tool tests

Run these commands from the repository root. The offline tools use Python 3.12
on Ubuntu 24.04; source the built ROS workspace when testing ROS message and
bag serialization. Tk tests need a display, so Xvfb supplies one on a headless
host:

```bash
sudo apt install -y python3-venv python3-tk xvfb xauth
python3 -m venv --system-site-packages "${HOME}/.venvs/vt-offline-tests"
"${HOME}/.venvs/vt-offline-tests/bin/python" -m pip install \
  -e 'tools/tracker_camera_calibration[test]' \
  -e 'tools/multisensor_alignment[test]' \
  -e ros2_ws/src/vt_capture_core -e ros2_ws/src/vt_capture_manager
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 xvfb-run -a \
  "${HOME}/.venvs/vt-offline-tests/bin/python" -m pytest \
  tools/tracker_camera_calibration/tests -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  "${HOME}/.venvs/vt-offline-tests/bin/python" -m pytest \
  tools/multisensor_alignment/tests -q
```

Run those suites separately: they contain test modules with the same names.
The console has its own tested Qt dependency set; do not install the offline
`[test]` extras into it because their pytest version bounds differ:

```bash
bash tools/capture_deploy/install.sh
.venv-capture-console/bin/python -m pip install \
  -e ros2_ws/src/vt_capture_manager -e tools/multisensor_alignment \
  'opencv-python-headless>=4.9,<4.12'
env PYTHONPATH=ros2_ws/src/vt_realsense_capture:tools/multisensor_alignment/src \
  QT_QPA_PLATFORM=offscreen PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  .venv-capture-console/bin/python -m pytest -q \
  ros2_ws/src/vt_capture_core/test ros2_ws/src/vt_capture_manager/test \
  tools/capture_console/tests tools/capture_deploy/tests
QT_QPA_PLATFORM=offscreen bash tools/capture_deploy/start_demo.sh \
  --smoke-seconds 2
```

These commands install a current-user desktop entry and run synthetic tests;
they do not start a sensor or record a bag. Optional ROS integration tests use
synthetic publishers and dedicated test domains; follow the
[workbench test guide](tools/capture_console/README.md#测试与截图) and keep them
separate from a live acquisition domain. Pure software tests and offscreen
rendering are not hardware, OpenGL-driver, pose-accuracy, or endurance acceptance.
The explicit source paths above expose the CMake package's pure-Python capture
controller and the offline replay parser for controller/manager fault-recovery
and replay tests; they do not start ROS nodes or a Recorder. Keep these
cross-component tests enabled when changing either side of the managed recording
contract.
For changes to a specific adapter, also run the tests in its linked
[hand](tools/fsglove12/README.md), [bridge](ros2_ws/src/vt_glove_bridge/README.md),
or [tactile](ros2_ws/src/vt_tactile_capture/README.md) guide.

## Distribution checks

When changing package dependencies, entry points, resources or release tooling,
validate installed artifacts as well as the relevant source tests. On Python
3.12, use a new output directory for each run:

```bash
python3 -m unittest discover -s tools/release/tests -v
python3 tools/release/verify_distributions.py \
  --profile all --output .runtime/release/distribution-check-1
```

The verifier builds wheels from source distributions, checks package licenses
and resources, then installs the wheels into fresh environments and runs
hardware-free smoke checks outside the checkout. It separates PySide6 and
PyQt5. The `core`, `offline` and `console` profiles allow focused validation;
`--build-only` does not establish successful installation or runtime behavior.
The full profile still does not replace ROS builds or hardware acceptance.
See the [release preparation guide](tools/release/README.md#安装包验证) for
prerequisites, dependency constraints, retained evidence and limitations.

## Data and evidence policy

Do not commit captured data or local evidence, including bags, MCAP files,
images, videos, recordings, credentials, private scene data, device dumps,
packet captures, bootstrap bundles, role maps, workstation settings, or large
generated logs. Keep those artifacts outside the repository. Do not add MANO
models, derived mesh caches, or separately obtained FSGlove/Tracker source to a
source archive; this repository's license does not grant rights to those inputs.

If a change makes a hardware-specific claim, state the tested device and
topology narrowly and separate the observation from software guarantees. Share
only sanitized textual summaries through an appropriate review channel; never
add raw evidence to Git.

Before the first public upload, follow the
[release preparation guide](tools/release/README.md). Current-tree checks and
`.gitignore` do not remove sensitive or unlicensed material from previous Git
commits. Review publication history separately or use the documented clean
source export; do not assume a passing check authorizes publishing the existing
Git history. Exporting source does not bundle the PyVUT submodule's contents.
