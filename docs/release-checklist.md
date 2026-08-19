# Release checklist

This checklist is for maintainers preparing a public source release. It is
separate from runtime acceptance: a repository can pass CI while untested on
hardware, and a hardware run can succeed while the repository still contains
private or generated artifacts.

## 1. Define the release

- Select one commit and assign a semantic version.
- Record supported Ubuntu, ROS 2, RealSense ROS, librealsense, firmware, and
  the pinned PyVUT submodule revision.
- List user-visible changes and known limitations in `CHANGELOG.md`.
- Confirm that package versions and the intended Git tag agree.
- Review `THIRD_PARTY_NOTICES.md`, `.gitmodules`, and the `LICENSE`/`NOTICE`
  files at every pinned third-party revision.
- Confirm whether the release covers camera capture, Tracker publishing, or
  both. The default bag does not include Tracker topics.

## 2. Sanitize the public tree

The public repository must contain source, configuration examples, tests, and
documentation only. It must not contain raw evidence, captures, credentials,
role maps, generated bundles, camera scenes, logs, build output, or internal
design notes.

Review tracked paths before publishing:

```bash
git status --short
git ls-files | sort
git submodule status --recursive
git ls-files | rg '\.(bag|db3|mcap|pcap|pcapng|webm|zip)$'
git ls-files | rg '(^|/)(build|install|log|artifacts|bags|\.worktrees)/'
git rev-list --objects HEAD | \
  rg 'docs/superpowers|lark-|\.(bag|db3|mcap|pcap|pcapng|webm|zip)$'
```

If sensitive material has ever been pushed, removing it in a later commit does
not remove it from Git history. Rotate exposed credentials if applicable and
use an explicitly reviewed history-rewrite or a new clean public repository.
Do not perform that operation as part of a routine release command.

## 3. Verify documentation

- The English and Chinese READMEs describe the same product scope.
- The [end-user manual](user-manual.md) can be followed from a fresh clone.
- The [single-entry calibration operations manual](../tools/tracker_camera_calibration/README.md)
  independently covers the offline-calibration boundary, installation, input
  gates, recording, solving, three-run repeatability, recovery, and acceptance.
- Every public launch argument, command, topic, QoS profile, output path, and
  lifecycle state matches the implementation and
  [interface reference](interface-reference.md).
- Commands use repository-relative paths or documented environment variables;
  no developer worktree path remains.
- Limitations are explicit: no hardware exposure synchronization, no shipped
  device-specific camera/Tracker transform, no online extrinsic TF, and no
  Tracker topics in the default camera bag.
- Private prerequisites are labeled as such and are not implied to ship in the
  public repository.
- All relative Markdown links resolve and screenshots contain no private data.

## 4. Run software verification

From a non-Conda shell:

```bash
test -z "${CONDA_PREFIX:-}"
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --event-handlers console_direct+
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --all --verbose
cd ..
PYTHONPATH=tools/tracker_camera_calibration/src \
  python3 -m unittest discover \
  -s tools/tracker_camera_calibration/tests -v
python3 tools/test_check_public_tree.py
python3 tools/check_public_tree.py
python3 third_party/pyvut/tools/check_public_tree.py
git diff --check
```

Do not waive failures in the public-tree checker. It is the release boundary,
not an informational lint.

## 5. Run reference hardware acceptance

Preserve sanitized textual results outside Git and record the exact tested
commit.

| Gate | Minimum evidence |
| --- | --- |
| Camera discovery | Three serial-bound nodes and all nine contract topics are visible. |
| Camera recording | START reaches `RECORDING`; planned or manual STOP reaches `COMPLETE`; `ros2 bag info` shows the exact topic allowlist. |
| Storage | Target filesystem has adequate measured margin for the intended session; result is recorded as environment evidence, not a Recorder gate. |
| Tracker startup | Approved manual bootstrap completes and the read-only publisher owns the Dongle alone. |
| Tracker ROS output | `vt-vive-validate-topics` reports `status=PASS roles=3 identity_swaps=0 dropped=0`. |
| Tracker-camera calibration | Each final rigid pair exports `status: VALID`, holdout closure is at most 10 mm and 1 degree, and three independent runs meet the repeatability requirement. |
| Visualization | All roles transition to fresh/green when moved; closing the GUI or RViz does not affect the publisher. |
| Long run | Project-specific 300-second camera and Tracker evidence is reviewed for drops, gaps, reconnects, and thermal/storage behavior. |

`COMPLETE` is not a camera-data quality result. Hardware acceptance therefore
includes both Recorder lifecycle evidence and an independent data review.

## 6. Publish and verify

- Push the reviewed commit and signed or annotated version tag.
- Create release notes containing compatibility, migrations, limitations, and
  links to the manual and changelog.
- Clone the public URL into a new directory with submodules and repeat the
  repository contract plus a source build.
- Verify issue and private security-reporting entry points.
- Keep the previous supported tag and its rollback instructions available.

## 7. Support handoff

A useful issue report contains the software version/commit, OS and ROS
versions, hardware/firmware inventory, USB topology, command used, relevant
timestamps, expected versus actual behavior, and sanitized logs. Never request
or attach a raw camera scene, `pcapng`, private bundle, or role map in a public
issue.
